/*********************************************************************************************************************
 * @file        vision_tag_tracker.cpp
 * @brief       TC377 单目灰度相机高对比标志视觉跟踪
 *
 * 算法：64 级 Otsu -> 黑像素积分图 -> 白静区/黑外框/固定 ID0 码字模板搜索
 *       -> 锁定后 ROI 搜索 -> alpha-beta 定点滤波 -> 短时丢失预测。
 *
 * 设计约束：
 * 1. 无 malloc/free，无递归，无可变长数组；内存和最坏循环次数可预估。
 * 2. 热路径以整数运算为主，适合 TC377；没有完整 AprilTag3 的 double/POSIX 依赖。
 * 3. 只负责视觉感知。控制端应读取 vision_tag_result_t，自行完成 PID 和安全策略。
 ********************************************************************************************************************/
#include "vision_tag_tracker.h"

#include <stddef.h>
#include <string.h>

#if defined(__TASKING__)
/* Keep legacy camera/UART drivers at -O0; optimize only this CPU1 hot path. */
#pragma optimize acefgiklmnoprsuvwy
#pragma tradeoff 0
#endif

constexpr auto VISION_TAG_INTEGRAL_PITCH = VISION_TAG_MAX_WIDTH + 1U;
constexpr auto VISION_TAG_INTEGRAL_SIZE = (VISION_TAG_MAX_WIDTH + 1U) * (VISION_TAG_MAX_HEIGHT + 1U);
constexpr auto VISION_TAG_HISTOGRAM_BINS = 64U;
constexpr auto VISION_TAG_MAX_PIXELS = VISION_TAG_MAX_WIDTH * VISION_TAG_MAX_HEIGHT;
constexpr auto VISION_TAG_VISITED_BYTES = (VISION_TAG_MAX_PIXELS + 7U) / 8U;
constexpr auto VISION_TAG_QUAD_EXTREMES = 8U;

constexpr auto VISION_TAG_MIN_RING_BLACK = 580U;
constexpr auto VISION_TAG_MIN_QUIET_WHITE = 580U;
constexpr auto VISION_TAG_MIN_INNER_BLACK = 120U;
constexpr auto VISION_TAG_MAX_INNER_BLACK = 880U;
constexpr auto VISION_TAG_MIN_CODE_MATCH = 700U;

constexpr auto VISION_TAG_Q8_ONE = 256L;
constexpr auto VISION_TAG_Q15_MAX = 32767L;

struct vision_tag_candidate_t
{
    int16_t x;
    int16_t y;
    int16_t width;
    int16_t height;
    uint16_t score;
    int16_t center_x;
    int16_t center_y;
    uint16_t size_px;
    uint16_t contrast;
    vision_tag_point_t corners[4];
    uint8_t has_corners;
    uint8_t perspective_corrected;
};

struct vision_tag_homography_t
{
    int32_t a_q16;
    int32_t b_q16;
    int32_t c_q16;
    int32_t d_q16;
    int32_t e_q16;
    int32_t f_q16;
    int32_t g_q16;
    int32_t h_q16;
};

/* tag36h11 ID 0 的 6 x 6 数据区；bit5 对应每行最左侧。支持四个 90 度旋转方向。 */
struct vision_frame_stats_t
{
    uint8_t p10;
    uint8_t p50;
    uint8_t p90;
    uint16_t dark_permille;
    uint16_t saturated_permille;
};

static const uint8_t s_tag36h11_id0_rows[6] =
{
    0x0AU, /* 001010 */
    0x22U, /* 100010 */
    0x27U, /* 100111 */
    0x17U, /* 010111 */
    0x29U, /* 101001 */
    0x3BU  /* 111011 */
};

/* 188 x 120 时约占 45.7 KiB，放在静态区，绝不能放到函数栈。 */
#if defined(__TASKING__)
#pragma section all "cpu1_dsram"
#endif
static uint16_t s_integral[VISION_TAG_INTEGRAL_SIZE];
static uint8_t s_cc_visited[VISION_TAG_VISITED_BYTES];
static uint16_t s_cc_stack[VISION_TAG_MAX_PIXELS];
static vision_tag_config_t s_config;
static vision_tag_result_t s_result;

static int32_t s_center_x_q8;
static int32_t s_center_y_q8;
static int32_t s_velocity_x_q8;
static int32_t s_velocity_y_q8;
static int32_t s_size_q8;
static uint8_t s_hit_streak;
static uint8_t s_locked;
static uint8_t s_full_search_countdown;

#if defined(__TASKING__)
#pragma section all restore
#pragma section code "cpu1_psram"
#endif

static int32_t vision_tag_clamp_i32(int32_t value, int32_t low, int32_t high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

static uint16_t vision_tag_min_u16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

static uint16_t vision_tag_max_u16(uint16_t a, uint16_t b)
{
    return (a > b) ? a : b;
}

static int32_t vision_tag_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint16_t vision_tag_isqrt_u32(uint32_t value)
{
    uint32_t result = 0U;
    uint32_t bit = 1UL << 30;

    while (bit > value)
    {
        bit >>= 2U;
    }
    while (bit != 0U)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1U) + bit;
        }
        else
        {
            result >>= 1U;
        }
        bit >>= 2U;
    }
    return static_cast<uint16_t>(result);
}

static uint32_t vision_tag_point_distance_sq(const vision_tag_point_t *a,
                                             const vision_tag_point_t *b)
{
    int32_t dx = static_cast<int32_t>(a->x) - b->x;
    int32_t dy = static_cast<int32_t>(a->y) - b->y;
    return static_cast<uint32_t>(dx * dx + dy * dy);
}

static int32_t vision_tag_cross(const vision_tag_point_t *a,
                                const vision_tag_point_t *b,
                                const vision_tag_point_t *c)
{
    return (static_cast<int32_t>(b->x) - a->x) * (static_cast<int32_t>(c->y) - b->y)
         - (static_cast<int32_t>(b->y) - a->y) * (static_cast<int32_t>(c->x) - b->x);
}

static uint8_t vision_tag_quad_is_convex(const vision_tag_point_t corners[4])
{
    int32_t first = 0;
    uint8_t index;

    for (index = 0U; index < 4U; ++index)
    {
        int32_t cross = vision_tag_cross(&corners[index],
                                         &corners[(index + 1U) & 3U],
                                         &corners[(index + 2U) & 3U]);
        if (cross == 0)
        {
            return 0U;
        }
        if (first == 0)
        {
            first = cross;
        }
        else if (((first < 0) && (cross > 0)) || ((first > 0) && (cross < 0)))
        {
            return 0U;
        }
    }
    return 1U;
}

static uint32_t vision_tag_quad_area2(const vision_tag_point_t corners[4])
{
    int32_t area = 0;
    uint8_t index;

    for (index = 0U; index < 4U; ++index)
    {
        const vision_tag_point_t *a = &corners[index];
        const vision_tag_point_t *b = &corners[(index + 1U) & 3U];
        area += static_cast<int32_t>(a->x) * b->y - static_cast<int32_t>(a->y) * b->x;
    }
    return static_cast<uint32_t>(vision_tag_abs_i32(area));
}

static uint8_t vision_tag_visited_get(uint32_t index)
{
    return static_cast<uint8_t>((s_cc_visited[index >> 3U] >> (index & 7U)) & 1U);
}

static void vision_tag_visited_set(uint32_t index)
{
    s_cc_visited[index >> 3U] |= static_cast<uint8_t>(1U << (index & 7U));
}

#if defined(__GNUC__) && !defined(__TASKING__)
/*
 * Cygwin GCC 12 miscompiles the 64-bin reduction at -O2 when its loop
 * vectorizer is enabled (the compressed-tag golden case then disappears).
 * Keep the workaround local to host GCC; the production TASKING build is
 * unaffected and the scalar calculation is bit-for-bit identical.
 */
__attribute__((optimize("no-tree-loop-vectorize")))
#endif
static uint8_t vision_tag_histogram_threshold(const uint8_t *image,
                                              uint16_t stride,
                                              uint16_t x0,
                                              uint16_t y0,
                                              uint16_t x1,
                                              uint16_t y1,
                                              vision_frame_stats_t *stats)
{
    uint32_t histogram[VISION_TAG_HISTOGRAM_BINS];
    uint32_t total;
    uint32_t total_sum;
    uint32_t background_count;
    uint32_t background_sum;
    uint64_t best_score;
    uint8_t best_bin;
    uint8_t min_value;
    uint8_t max_value;
    uint16_t x;
    uint16_t y;
    uint16_t bin;

    for(auto &value : histogram)
    {
        value = 0U;
    }
    total = static_cast<uint32_t>(x1 - x0) * static_cast<uint32_t>(y1 - y0);
    total_sum = 0U;
    min_value = 255U;
    max_value = 0U;

    for (y = y0; y < y1; ++y)
    {
        const uint8_t *row = image + static_cast<uint32_t>(y) * stride;
        for (x = x0; x < x1; ++x)
        {
            uint8_t value = row[x];
            uint8_t index = static_cast<uint8_t>(value >> 2);
            ++histogram[index];
            if (value < min_value)
            {
                min_value = value;
            }
            if (value > max_value)
            {
                max_value = value;
            }
        }
    }

    if (stats != nullptr)
    {
        uint32_t cumulative = 0U;
        uint32_t target10 = (total + 9U) / 10U;
        uint32_t target50 = (total + 1U) / 2U;
        uint32_t target90 = (total * 9U + 9U) / 10U;
        uint8_t found10 = 0U;
        uint8_t found50 = 0U;

        stats->p10 = min_value;
        stats->p50 = min_value;
        stats->p90 = max_value;
        for (bin = 0U; bin < VISION_TAG_HISTOGRAM_BINS; ++bin)
        {
            cumulative += histogram[bin];
            if ((found10 == 0U) && (cumulative >= target10))
            {
                stats->p10 = static_cast<uint8_t>((bin << 2) + 2U);
                found10 = 1U;
            }
            if ((found50 == 0U) && (cumulative >= target50))
            {
                stats->p50 = static_cast<uint8_t>((bin << 2) + 2U);
                found50 = 1U;
            }
            if (cumulative >= target90)
            {
                stats->p90 = static_cast<uint8_t>((bin << 2) + 2U);
                break;
            }
        }
        stats->dark_permille = static_cast<uint16_t>(((histogram[0] + histogram[1]
                                           + histogram[2] + histogram[3]) * 1000U) / total);
        stats->saturated_permille = static_cast<uint16_t>(((histogram[62] + histogram[63]) * 1000U) / total);
    }

    if (static_cast<uint16_t>(static_cast<uint16_t>(max_value) - static_cast<uint16_t>(min_value)) < static_cast<uint16_t>(12U))
    {
        return static_cast<uint8_t>((static_cast<uint16_t>(max_value) + static_cast<uint16_t>(min_value)) / 2U);
    }

    for (bin = 0U; bin < VISION_TAG_HISTOGRAM_BINS; ++bin)
    {
        total_sum += static_cast<uint32_t>(bin) * histogram[bin];
    }

    background_count = 0U;
    background_sum = 0U;
    best_score = 0U;
    best_bin = 31U;

    for (bin = 0U; bin < (VISION_TAG_HISTOGRAM_BINS - 1U); ++bin)
    {
        uint32_t foreground_count;
        uint32_t mean_background;
        uint32_t mean_foreground;
        uint32_t mean_difference;
        uint64_t score;

        background_count += histogram[bin];
        background_sum += static_cast<uint32_t>(bin) * histogram[bin];
        if (background_count == 0U)
        {
            continue;
        }

        foreground_count = total - background_count;
        if (foreground_count == 0U)
        {
            break;
        }

        mean_background = background_sum / background_count;
        mean_foreground = (total_sum - background_sum) / foreground_count;
        mean_difference = (mean_foreground >= mean_background)
                        ? (mean_foreground - mean_background)
                        : (mean_background - mean_foreground);

        score = static_cast<uint64_t>(background_count) * static_cast<uint64_t>(foreground_count);
        score *= static_cast<uint64_t>(mean_difference) * static_cast<uint64_t>(mean_difference);
        if (score > best_score)
        {
            best_score = score;
            best_bin = static_cast<uint8_t>(bin);
        }
    }

    {
        uint16_t threshold = static_cast<uint16_t>((static_cast<uint16_t>(best_bin) << 2) + 2U);
        threshold = static_cast<uint16_t>(vision_tag_clamp_i32(static_cast<int32_t>(threshold), 24, 232));
        return static_cast<uint8_t>(threshold);
    }
}

static uint8_t vision_tag_otsu_threshold(const uint8_t *image,
                                         uint16_t width,
                                         uint16_t height,
                                         uint16_t stride,
                                         vision_frame_stats_t *stats,
                                         uint8_t *global_threshold)
{
    uint8_t threshold = vision_tag_histogram_threshold(image, stride, 0U, 0U,
                                                       width, height, stats);

    if (global_threshold != nullptr)
    {
        *global_threshold = threshold;
    }

    if (((s_locked != 0U) || (s_hit_streak != 0U)) && (s_size_q8 > 0))
    {
        int32_t center_x = s_center_x_q8 / VISION_TAG_Q8_ONE;
        int32_t center_y = s_center_y_q8 / VISION_TAG_Q8_ONE;
        int32_t size = s_size_q8 / VISION_TAG_Q8_ONE;
        int32_t half = static_cast<int32_t>(vision_tag_max_u16(24U, static_cast<uint16_t>(size * 2)));
        uint16_t rx0 = static_cast<uint16_t>(vision_tag_clamp_i32(center_x - half, 0, width - 1));
        uint16_t ry0 = static_cast<uint16_t>(vision_tag_clamp_i32(center_y - half, 0, height - 1));
        uint16_t rx1 = static_cast<uint16_t>(vision_tag_clamp_i32(center_x + half, rx0 + 1, width));
        uint16_t ry1 = static_cast<uint16_t>(vision_tag_clamp_i32(center_y + half, ry0 + 1, height));

        threshold = vision_tag_histogram_threshold(image, stride, rx0, ry0, rx1, ry1, nullptr);
    }

    return threshold;
}

static void vision_tag_build_integral(const uint8_t *image,
                                      uint16_t width,
                                      uint16_t height,
                                      uint16_t stride,
                                      uint8_t threshold)
{
    uint16_t x;
    uint16_t y;
    uint16_t pitch = static_cast<uint16_t>(VISION_TAG_INTEGRAL_PITCH);

    for(uint16_t index = 0U; index <= width; ++index)
    {
        s_integral[index] = 0U;
    }
    for (y = 0U; y < height; ++y)
    {
        uint16_t row_black = 0U;
        uint32_t current_offset = static_cast<uint32_t>(y + 1U) * pitch;
        uint32_t previous_offset = static_cast<uint32_t>(y) * pitch;
        const uint8_t *row = image + static_cast<uint32_t>(y) * stride;

        s_integral[current_offset] = 0U;
        for (x = 0U; x < width; ++x)
        {
            if (row[x] < threshold)
            {
                ++row_black;
            }
            s_integral[current_offset + x + 1U] =
                static_cast<uint16_t>(s_integral[previous_offset + x + 1U] + row_black);
        }
    }
}

/* 半开区间 [x0, x1) x [y0, y1) 中的黑像素数。 */
static uint16_t vision_tag_rect_black(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
    uint32_t pitch = VISION_TAG_INTEGRAL_PITCH;
    uint32_t a = s_integral[static_cast<uint32_t>(y0) * pitch + static_cast<uint16_t>(x0)];
    uint32_t b = s_integral[static_cast<uint32_t>(y0) * pitch + static_cast<uint16_t>(x1)];
    uint32_t c = s_integral[static_cast<uint32_t>(y1) * pitch + static_cast<uint16_t>(x0)];
    uint32_t d = s_integral[static_cast<uint32_t>(y1) * pitch + static_cast<uint16_t>(x1)];
    return static_cast<uint16_t>(d + a - b - c);
}

static uint8_t vision_tag_black_at_least(int16_t x0,
                                         int16_t y0,
                                         int16_t x1,
                                         int16_t y1,
                                         uint16_t minimum_permille)
{
    uint32_t area = static_cast<uint32_t>(x1 - x0) * static_cast<uint32_t>(y1 - y0);
    uint32_t black = vision_tag_rect_black(x0, y0, x1, y1);
    return static_cast<uint8_t>(((black * VISION_TAG_SCORE_MAX) >= (area * minimum_permille)) ? 1U : 0U);
}

static uint16_t vision_tag_black_ratio(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
    uint32_t area = static_cast<uint32_t>(x1 - x0) * static_cast<uint32_t>(y1 - y0);
    uint32_t black;
    if (area == 0U)
    {
        return 0U;
    }
    black = vision_tag_rect_black(x0, y0, x1, y1);
    return static_cast<uint16_t>((black * VISION_TAG_SCORE_MAX) / area);
}

static uint16_t vision_tag_white_ratio(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
    return static_cast<uint16_t>(VISION_TAG_SCORE_MAX - vision_tag_black_ratio(x0, y0, x1, y1));
}

static uint8_t vision_tag_expected_bit(uint8_t row, uint8_t column, uint8_t rotation)
{
    uint8_t source_row;
    uint8_t source_column;

    switch (rotation & 3U)
    {
        case 1U:
            source_row = static_cast<uint8_t>(5U - column);
            source_column = row;
            break;
        case 2U:
            source_row = static_cast<uint8_t>(5U - row);
            source_column = static_cast<uint8_t>(5U - column);
            break;
        case 3U:
            source_row = column;
            source_column = static_cast<uint8_t>(5U - row);
            break;
        default:
            source_row = row;
            source_column = column;
            break;
    }

    return static_cast<uint8_t>((s_tag36h11_id0_rows[source_row] >> (5U - source_column)) & 1U);
}

static uint16_t vision_tag_code_match(int16_t x, int16_t y, int16_t width, int16_t height)
{
    uint16_t best_score = 0U;
    uint8_t rotation;

    for (rotation = 0U; rotation < 4U; ++rotation)
    {
        uint32_t score_sum = 0U;
        uint8_t row;

        for (row = 0U; row < 6U; ++row)
        {
            uint8_t column;
            int16_t cell_y0 = static_cast<int16_t>(y + (static_cast<int32_t>(height) * (static_cast<int32_t>(row) + 1)) / 8L);
            int16_t cell_y1 = static_cast<int16_t>(y + (static_cast<int32_t>(height) * (static_cast<int32_t>(row) + 2)) / 8L);

            for (column = 0U; column < 6U; ++column)
            {
                int16_t cell_x0 = static_cast<int16_t>(x + (static_cast<int32_t>(width) * (static_cast<int32_t>(column) + 1)) / 8L);
                int16_t cell_x1 = static_cast<int16_t>(x + (static_cast<int32_t>(width) * (static_cast<int32_t>(column) + 2)) / 8L);
                uint16_t black_ratio;

                /* 大目标只取单元格中央，减小候选框 1 px 偏差和透视边缘的影响。 */
                if ((cell_x1 - cell_x0) >= 4)
                {
                    ++cell_x0;
                    --cell_x1;
                }
                if ((cell_y1 - cell_y0) >= 4)
                {
                    ++cell_y0;
                    --cell_y1;
                }

                black_ratio = vision_tag_black_ratio(cell_x0, cell_y0, cell_x1, cell_y1);
                if (vision_tag_expected_bit(row, column, rotation) != 0U)
                {
                    score_sum += black_ratio;
                }
                else
                {
                    score_sum += static_cast<uint16_t>(VISION_TAG_SCORE_MAX - black_ratio);
                }
            }
        }

        {
            uint16_t rotation_score = static_cast<uint16_t>(score_sum / 36U);
            if (rotation_score > best_score)
            {
                best_score = rotation_score;
            }
        }
    }

    return best_score;
}

static uint8_t vision_tag_homography_init(vision_tag_homography_t *homography,
                                          const vision_tag_point_t corners[4])
{
    int32_t dx1 = static_cast<int32_t>(corners[1].x) - corners[2].x;
    int32_t dx2 = static_cast<int32_t>(corners[3].x) - corners[2].x;
    int32_t dx3 = static_cast<int32_t>(corners[0].x) - corners[1].x
                + corners[2].x - corners[3].x;
    int32_t dy1 = static_cast<int32_t>(corners[1].y) - corners[2].y;
    int32_t dy2 = static_cast<int32_t>(corners[3].y) - corners[2].y;
    int32_t dy3 = static_cast<int32_t>(corners[0].y) - corners[1].y
                + corners[2].y - corners[3].y;
    int32_t denominator = dx1 * dy2 - dx2 * dy1;
    int64_t g_q16;
    int64_t h_q16;

    if ((homography == nullptr) || (vision_tag_abs_i32(denominator) < 4))
    {
        return 0U;
    }

    g_q16 = ((static_cast<int64_t>(dx3) * dy2 - static_cast<int64_t>(dx2) * dy3) << 16) / denominator;
    h_q16 = ((static_cast<int64_t>(dx1) * dy3 - static_cast<int64_t>(dx3) * dy1) << 16) / denominator;
    if ((g_q16 < -262144L) || (g_q16 > 262144L)
        || (h_q16 < -262144L) || (h_q16 > 262144L))
    {
        return 0U;
    }

    homography->g_q16 = static_cast<int32_t>(g_q16);
    homography->h_q16 = static_cast<int32_t>(h_q16);
    homography->a_q16 = (static_cast<int32_t>(corners[1].x) - corners[0].x) * 65536L
                      + homography->g_q16 * corners[1].x;
    homography->b_q16 = (static_cast<int32_t>(corners[3].x) - corners[0].x) * 65536L
                      + homography->h_q16 * corners[3].x;
    homography->c_q16 = static_cast<int32_t>(corners[0].x) * 65536L;
    homography->d_q16 = (static_cast<int32_t>(corners[1].y) - corners[0].y) * 65536L
                      + homography->g_q16 * corners[1].y;
    homography->e_q16 = (static_cast<int32_t>(corners[3].y) - corners[0].y) * 65536L
                      + homography->h_q16 * corners[3].y;
    homography->f_q16 = static_cast<int32_t>(corners[0].y) * 65536L;
    return 1U;
}

static uint8_t vision_tag_homography_point(const vision_tag_homography_t *homography,
                                           int32_t u_q16,
                                           int32_t v_q16,
                                           int16_t *x,
                                           int16_t *y)
{
    int64_t denominator_q16;
    int64_t x_numerator_q16;
    int64_t y_numerator_q16;
    int64_t x_q16;
    int64_t y_q16;

    denominator_q16 = 65536L
                    + ((static_cast<int64_t>(homography->g_q16) * u_q16) >> 16)
                    + ((static_cast<int64_t>(homography->h_q16) * v_q16) >> 16);
    if ((denominator_q16 > -1024L) && (denominator_q16 < 1024L))
    {
        return 0U;
    }

    x_numerator_q16 = ((static_cast<int64_t>(homography->a_q16) * u_q16) >> 16)
                    + ((static_cast<int64_t>(homography->b_q16) * v_q16) >> 16)
                    + homography->c_q16;
    y_numerator_q16 = ((static_cast<int64_t>(homography->d_q16) * u_q16) >> 16)
                    + ((static_cast<int64_t>(homography->e_q16) * v_q16) >> 16)
                    + homography->f_q16;
    x_q16 = (x_numerator_q16 << 16) / denominator_q16;
    y_q16 = (y_numerator_q16 << 16) / denominator_q16;
    *x = static_cast<int16_t>((x_q16 + 32768L) >> 16);
    *y = static_cast<int16_t>((y_q16 + 32768L) >> 16);
    return 1U;
}

static uint8_t vision_tag_homography_sample(const vision_tag_homography_t *homography,
                                            const uint8_t *image,
                                            uint16_t width,
                                            uint16_t height,
                                            uint16_t stride,
                                            int32_t u_q16,
                                            int32_t v_q16,
                                            uint8_t *value)
{
    int16_t x = 0;
    int16_t y = 0;

    if ((vision_tag_homography_point(homography, u_q16, v_q16, &x, &y) == 0U)
        || (x < 0) || (y < 0) || (x >= static_cast<int16_t>(width)) || (y >= static_cast<int16_t>(height)))
    {
        return 0U;
    }
    *value = image[static_cast<uint32_t>(y) * stride + static_cast<uint16_t>(x)];
    return 1U;
}

static uint16_t vision_tag_quad_cell_black(const vision_tag_homography_t *homography,
                                           const uint8_t *image,
                                           uint16_t width,
                                           uint16_t height,
                                           uint16_t stride,
                                           uint8_t threshold,
                                           uint8_t cell_x,
                                           uint8_t cell_y)
{
    static const uint8_t offsets[2] = {3U, 5U};
    uint16_t black = 0U;
    uint16_t valid = 0U;
    uint8_t sample_y;

    for (sample_y = 0U; sample_y < 2U; ++sample_y)
    {
        uint8_t sample_x;
        for (sample_x = 0U; sample_x < 2U; ++sample_x)
        {
            uint8_t value = 0U;
            int32_t u_q16 = static_cast<int32_t>(static_cast<uint16_t>(cell_x) * 8U + offsets[sample_x]) * 1024L;
            int32_t v_q16 = static_cast<int32_t>(static_cast<uint16_t>(cell_y) * 8U + offsets[sample_y]) * 1024L;
            if (vision_tag_homography_sample(homography, image, width, height, stride,
                                             u_q16, v_q16, &value) != 0U)
            {
                ++valid;
                if (value < threshold)
                {
                    ++black;
                }
            }
        }
    }
    return (valid != 0U) ? static_cast<uint16_t>((black * VISION_TAG_SCORE_MAX) / valid) : 0U;
}

static uint16_t vision_tag_score_quad(const uint8_t *image,
                                      uint16_t width,
                                      uint16_t height,
                                      uint16_t stride,
                                      uint8_t threshold,
                                      const vision_tag_point_t corners[4],
                                      uint16_t *contrast)
{
    vision_tag_homography_t homography;
    uint16_t cell_black[8][8];
    uint32_t top = 0U;
    uint32_t bottom = 0U;
    uint32_t left = 0U;
    uint32_t right = 0U;
    uint32_t inner_sum = 0U;
    uint16_t ring_score;
    uint16_t quiet_score;
    uint16_t inner_black;
    uint16_t inner_mix;
    uint16_t best_code = 0U;
    uint32_t quiet_gray_sum = 0U;
    uint32_t ring_gray_sum = 0U;
    uint16_t quiet_count = 0U;
    uint16_t quiet_white = 0U;
    uint16_t ring_count = 0U;
    uint8_t row;

    if (vision_tag_homography_init(&homography, corners) == 0U)
    {
        return 0U;
    }

    for (row = 0U; row < 8U; ++row)
    {
        uint8_t column;
        for (column = 0U; column < 8U; ++column)
        {
            uint8_t value = 0U;
            cell_black[row][column] = vision_tag_quad_cell_black(&homography, image,
                                                                  width, height, stride,
                                                                  threshold, column, row);
            if ((row == 0U) || (row == 7U) || (column == 0U) || (column == 7U))
            {
                if (vision_tag_homography_sample(&homography, image, width, height, stride,
                                                 static_cast<int32_t>(2U * column + 1U) * 4096L,
                                                 static_cast<int32_t>(2U * row + 1U) * 4096L,
                                                 &value) != 0U)
                {
                    ring_gray_sum += value;
                    ++ring_count;
                }
            }
            else
            {
                inner_sum += cell_black[row][column];
            }
        }
        top += cell_black[0][row];
        bottom += cell_black[7][row];
        left += cell_black[row][0];
        right += cell_black[row][7];
    }

    ring_score = vision_tag_min_u16(static_cast<uint16_t>(top / 8U), static_cast<uint16_t>(bottom / 8U));
    ring_score = vision_tag_min_u16(ring_score, static_cast<uint16_t>(left / 8U));
    ring_score = vision_tag_min_u16(ring_score, static_cast<uint16_t>(right / 8U));
    if (ring_score < VISION_TAG_MIN_RING_BLACK)
    {
        return 0U;
    }

    inner_black = static_cast<uint16_t>(inner_sum / 36U);
    if ((inner_black < VISION_TAG_MIN_INNER_BLACK) || (inner_black > VISION_TAG_MAX_INNER_BLACK))
    {
        return 0U;
    }

    {
        uint8_t rotation;
        for (rotation = 0U; rotation < 4U; ++rotation)
        {
            uint32_t code_sum = 0U;
            for (row = 0U; row < 6U; ++row)
            {
                uint8_t column;
                for (column = 0U; column < 6U; ++column)
                {
                    uint16_t black = cell_black[row + 1U][column + 1U];
                    code_sum += (vision_tag_expected_bit(row, column, rotation) != 0U)
                              ? black : (VISION_TAG_SCORE_MAX - black);
                }
            }
            if (static_cast<uint16_t>(code_sum / 36U) > best_code)
            {
                best_code = static_cast<uint16_t>(code_sum / 36U);
            }
        }
    }
    if (best_code < VISION_TAG_MIN_CODE_MATCH)
    {
        return 0U;
    }

    for (row = 0U; row < 8U; ++row)
    {
        uint8_t value = 0U;
        int32_t center_q16 = static_cast<int32_t>(2U * row + 1U) * 4096L;
        const int32_t outside_low_q16 = -4096L;
        const int32_t outside_high_q16 = 69632L;
        if (vision_tag_homography_sample(&homography, image, width, height, stride,
                                         center_q16, outside_low_q16, &value) != 0U)
        {
            quiet_gray_sum += value; ++quiet_count;
            if (value >= threshold) { ++quiet_white; }
        }
        if (vision_tag_homography_sample(&homography, image, width, height, stride,
                                         center_q16, outside_high_q16, &value) != 0U)
        {
            quiet_gray_sum += value; ++quiet_count;
            if (value >= threshold) { ++quiet_white; }
        }
        if (vision_tag_homography_sample(&homography, image, width, height, stride,
                                         outside_low_q16, center_q16, &value) != 0U)
        {
            quiet_gray_sum += value; ++quiet_count;
            if (value >= threshold) { ++quiet_white; }
        }
        if (vision_tag_homography_sample(&homography, image, width, height, stride,
                                         outside_high_q16, center_q16, &value) != 0U)
        {
            quiet_gray_sum += value; ++quiet_count;
            if (value >= threshold) { ++quiet_white; }
        }
    }
    quiet_score = (quiet_count != 0U)
                ? static_cast<uint16_t>((static_cast<uint32_t>(quiet_white) * VISION_TAG_SCORE_MAX) / quiet_count)
                : VISION_TAG_SCORE_MAX;
    if (quiet_score < VISION_TAG_MIN_QUIET_WHITE)
    {
        return 0U;
    }

    if (contrast != nullptr)
    {
        uint32_t ring_mean = (ring_count != 0U) ? ring_gray_sum / ring_count : 0U;
        uint32_t quiet_mean = (quiet_count != 0U) ? quiet_gray_sum / quiet_count : 255U;
        *contrast = static_cast<uint16_t>((quiet_mean > ring_mean) ? (quiet_mean - ring_mean) : 0U);
    }

    inner_mix = (inner_black >= 500U) ? static_cast<uint16_t>(1500U - inner_black)
                                      : static_cast<uint16_t>(500U + inner_black);
    if (inner_mix > VISION_TAG_SCORE_MAX)
    {
        inner_mix = VISION_TAG_SCORE_MAX;
    }
    return static_cast<uint16_t>((static_cast<uint32_t>(ring_score) * 45U
                     + static_cast<uint32_t>(quiet_score) * 25U
                     + static_cast<uint32_t>(best_code) * 25U
                     + static_cast<uint32_t>(inner_mix) * 5U) / 100U);
}

static uint16_t vision_tag_score_candidate(int16_t x,
                                           int16_t y,
                                           int16_t width,
                                           int16_t height,
                                           uint16_t image_width,
                                           uint16_t image_height)
{
    int16_t border_x = static_cast<int16_t>(width / 8);
    int16_t border_y = static_cast<int16_t>(height / 8);
    int16_t quiet_x = border_x;
    int16_t quiet_y = border_y;
    int16_t quiet_x0;
    int16_t quiet_y0;
    int16_t quiet_x1;
    int16_t quiet_y1;
    uint16_t ring_score;
    uint16_t quiet_score;
    uint16_t inner_black;
    uint16_t inner_mix;
    uint16_t code_match;
    uint16_t value;
    uint32_t weighted;

    if (border_x < 2)
    {
        border_x = 2;
        quiet_x = 2;
    }
    if (border_y < 2)
    {
        border_y = 2;
        quiet_y = 2;
    }

    /* The full black frame must be visible, but the white quiet zone may be
     * cropped at an image edge when the tag is still usable for centering. */
    if ((x < 0) || (y < 0)
        || (static_cast<int32_t>(x) + width > image_width)
        || (static_cast<int32_t>(y) + height > image_height)
        || (width <= (border_x * 2 + 2))
        || (height <= (border_y * 2 + 2)))
    {
        return 0U;
    }

    quiet_x0 = (x >= quiet_x) ? static_cast<int16_t>(x - quiet_x) : 0;
    quiet_y0 = (y >= quiet_y) ? static_cast<int16_t>(y - quiet_y) : 0;
    quiet_x1 = ((static_cast<int32_t>(x) + width + quiet_x) <= image_width)
             ? static_cast<int16_t>(x + width + quiet_x) : static_cast<int16_t>(image_width);
    quiet_y1 = ((static_cast<int32_t>(y) + height + quiet_y) <= image_height)
             ? static_cast<int16_t>(y + height + quiet_y) : static_cast<int16_t>(image_height);

    /* Reject the overwhelming majority of windows without integer divisions. */
    if ((vision_tag_black_at_least(x, y, static_cast<int16_t>(x + width),
                                   static_cast<int16_t>(y + border_y), VISION_TAG_MIN_RING_BLACK) == 0U)
        || (vision_tag_black_at_least(x, static_cast<int16_t>(y + height - border_y),
                                      static_cast<int16_t>(x + width), static_cast<int16_t>(y + height),
                                      VISION_TAG_MIN_RING_BLACK) == 0U)
        || (vision_tag_black_at_least(x, static_cast<int16_t>(y + border_y),
                                      static_cast<int16_t>(x + border_x), static_cast<int16_t>(y + height - border_y),
                                      VISION_TAG_MIN_RING_BLACK) == 0U)
        || (vision_tag_black_at_least(static_cast<int16_t>(x + width - border_x),
                                      static_cast<int16_t>(y + border_y), static_cast<int16_t>(x + width),
                                      static_cast<int16_t>(y + height - border_y), VISION_TAG_MIN_RING_BLACK) == 0U))
    {
        return 0U;
    }

    ring_score = vision_tag_black_ratio(x, y, static_cast<int16_t>(x + width), static_cast<int16_t>(y + border_y));
    value = vision_tag_black_ratio(x, static_cast<int16_t>(y + height - border_y),
                                   static_cast<int16_t>(x + width), static_cast<int16_t>(y + height));
    ring_score = vision_tag_min_u16(ring_score, value);
    value = vision_tag_black_ratio(x, static_cast<int16_t>(y + border_y),
                                   static_cast<int16_t>(x + border_x), static_cast<int16_t>(y + height - border_y));
    ring_score = vision_tag_min_u16(ring_score, value);
    value = vision_tag_black_ratio(static_cast<int16_t>(x + width - border_x), static_cast<int16_t>(y + border_y),
                                   static_cast<int16_t>(x + width), static_cast<int16_t>(y + height - border_y));
    ring_score = vision_tag_min_u16(ring_score, value);

    if (ring_score < VISION_TAG_MIN_RING_BLACK)
    {
        return 0U;
    }

    quiet_score = VISION_TAG_SCORE_MAX;
    if (y >= quiet_y)
    {
        value = vision_tag_white_ratio(quiet_x0, quiet_y0, quiet_x1, y);
        quiet_score = vision_tag_min_u16(quiet_score, value);
    }
    if ((static_cast<int32_t>(y) + height + quiet_y) <= image_height)
    {
        value = vision_tag_white_ratio(quiet_x0, static_cast<int16_t>(y + height),
                                       quiet_x1, quiet_y1);
        quiet_score = vision_tag_min_u16(quiet_score, value);
    }
    if (x >= quiet_x)
    {
        value = vision_tag_white_ratio(quiet_x0, y,
                                       x, static_cast<int16_t>(y + height));
        quiet_score = vision_tag_min_u16(quiet_score, value);
    }
    if ((static_cast<int32_t>(x) + width + quiet_x) <= image_width)
    {
        value = vision_tag_white_ratio(static_cast<int16_t>(x + width), y,
                                       quiet_x1, static_cast<int16_t>(y + height));
        quiet_score = vision_tag_min_u16(quiet_score, value);
    }

    if (quiet_score < VISION_TAG_MIN_QUIET_WHITE)
    {
        return 0U;
    }

    inner_black = vision_tag_black_ratio(static_cast<int16_t>(x + border_x), static_cast<int16_t>(y + border_y),
                                         static_cast<int16_t>(x + width - border_x),
                                         static_cast<int16_t>(y + height - border_y));
    if ((inner_black < VISION_TAG_MIN_INNER_BLACK) || (inner_black > VISION_TAG_MAX_INNER_BLACK))
    {
        return 0U;
    }

    /* Require the exact 6x6 payload of the selected tag36h11 ID 0 marker.
     * Ring-only matching is intentionally not accepted: it creates false
     * locks on arbitrary black rectangles in the competition environment. */
    code_match = vision_tag_code_match(x, y, width, height);
    if (code_match < VISION_TAG_MIN_CODE_MATCH)
    {
        return 0U;
    }

    /* 内部接近均衡黑白时得分较高，但该项权重较低，允许不同 ID 的码字密度差异。 */
    inner_mix = (inner_black >= 500U) ? static_cast<uint16_t>(1500U - inner_black)
                                      : static_cast<uint16_t>(500U + inner_black);
    if (inner_mix > VISION_TAG_SCORE_MAX)
    {
        inner_mix = VISION_TAG_SCORE_MAX;
    }

    weighted = static_cast<uint32_t>(ring_score) * 45U
             + static_cast<uint32_t>(quiet_score) * 25U
             + static_cast<uint32_t>(code_match) * 25U
             + static_cast<uint32_t>(inner_mix) * 5U;
    return static_cast<uint16_t>(weighted / 100U);
}

static void vision_tag_try_candidate(vision_tag_candidate_t *best,
                                     int16_t x,
                                     int16_t y,
                                     int16_t width,
                                     int16_t height,
                                     uint16_t image_width,
                                     uint16_t image_height)
{
    uint16_t score = vision_tag_score_candidate(x, y, width, height, image_width, image_height);
    if (score > best->score)
    {
        best->x = x;
        best->y = y;
        best->width = width;
        best->height = height;
        best->score = score;
        best->center_x = static_cast<int16_t>(x + width / 2);
        best->center_y = static_cast<int16_t>(y + height / 2);
        best->size_px = static_cast<uint16_t>((static_cast<uint16_t>(width) + static_cast<uint16_t>(height)) / 2U);
        best->contrast = 0U;
        best->corners[0].x = x;
        best->corners[0].y = y;
        best->corners[1].x = static_cast<int16_t>(x + width - 1);
        best->corners[1].y = y;
        best->corners[2].x = static_cast<int16_t>(x + width - 1);
        best->corners[2].y = static_cast<int16_t>(y + height - 1);
        best->corners[3].x = x;
        best->corners[3].y = static_cast<int16_t>(y + height - 1);
        best->has_corners = 1U;
        best->perspective_corrected = 0U;
    }
}

static uint8_t vision_tag_quad_from_extremes(const vision_tag_point_t extremes[VISION_TAG_QUAD_EXTREMES],
                                             uint16_t min_size,
                                             vision_tag_point_t corners[4])
{
    vision_tag_point_t unique[VISION_TAG_QUAD_EXTREMES];
    uint8_t unique_count = 0U;
    uint32_t best_area = 0U;
    uint8_t index;

    for (index = 0U; index < VISION_TAG_QUAD_EXTREMES; ++index)
    {
        uint8_t duplicate = 0U;
        uint8_t previous;
        for (previous = 0U; previous < unique_count; ++previous)
        {
            if ((unique[previous].x == extremes[index].x)
                && (unique[previous].y == extremes[index].y))
            {
                duplicate = 1U;
                break;
            }
        }
        if (duplicate == 0U)
        {
            unique[unique_count++] = extremes[index];
        }
    }

    if (unique_count < 4U)
    {
        return 0U;
    }

    {
        uint8_t a;
        for (a = 0U; a + 3U < unique_count; ++a)
        {
            uint8_t b;
            for (b = static_cast<uint8_t>(a + 1U); b + 2U < unique_count; ++b)
            {
                uint8_t c;
                for (c = static_cast<uint8_t>(b + 1U); c + 1U < unique_count; ++c)
                {
                    uint8_t d;
                    for (d = static_cast<uint8_t>(c + 1U); d < unique_count; ++d)
                    {
                        vision_tag_point_t candidate[4];
                        uint32_t area;
                        uint32_t min_edge_sq;
                        uint16_t min_edge = vision_tag_max_u16(7U, static_cast<uint16_t>(min_size / 2U));

                        candidate[0] = unique[a];
                        candidate[1] = unique[b];
                        candidate[2] = unique[c];
                        candidate[3] = unique[d];
                        if (vision_tag_quad_is_convex(candidate) == 0U)
                        {
                            continue;
                        }
                        min_edge_sq = static_cast<uint32_t>(min_edge) * min_edge;
                        if ((vision_tag_point_distance_sq(&candidate[0], &candidate[1]) < min_edge_sq)
                            || (vision_tag_point_distance_sq(&candidate[1], &candidate[2]) < min_edge_sq)
                            || (vision_tag_point_distance_sq(&candidate[2], &candidate[3]) < min_edge_sq)
                            || (vision_tag_point_distance_sq(&candidate[3], &candidate[0]) < min_edge_sq))
                        {
                            continue;
                        }
                        area = vision_tag_quad_area2(candidate);
                        if ((area > best_area) && (area >= static_cast<uint32_t>(min_size) * min_size))
                        {
                            best_area = area;
                            corners[0] = candidate[0];
                            corners[1] = candidate[1];
                            corners[2] = candidate[2];
                            corners[3] = candidate[3];
                        }
                    }
                }
            }
        }
    }
    return (best_area != 0U) ? 1U : 0U;
}

static void vision_tag_try_quad_component(vision_tag_candidate_t *best,
                                          const uint8_t *image,
                                          uint16_t image_width,
                                          uint16_t image_height,
                                          uint16_t stride,
                                          uint8_t threshold,
                                          const vision_tag_point_t extremes[VISION_TAG_QUAD_EXTREMES],
                                          int16_t min_x,
                                          int16_t min_y,
                                          int16_t max_x,
                                          int16_t max_y,
                                          uint32_t component_pixels,
                                          uint16_t min_size,
                                          uint16_t max_size)
{
    vision_tag_point_t corners[4];
    uint16_t bbox_width = static_cast<uint16_t>(max_x - min_x + 1);
    uint16_t bbox_height = static_cast<uint16_t>(max_y - min_y + 1);
    uint16_t max_bbox = static_cast<uint16_t>(max_size + max_size / 2U + 4U);
    uint16_t contrast = 0U;
    uint16_t score;
    uint32_t edge_sum = 0U;
    uint8_t index;

    if ((component_pixels < static_cast<uint32_t>(min_size) * 2U)
        || (bbox_width < min_size) || (bbox_height < min_size)
        || (bbox_width > max_bbox) || (bbox_height > max_bbox)
        || (static_cast<uint32_t>(bbox_width) * 3U < bbox_height)
        || (static_cast<uint32_t>(bbox_height) * 3U < bbox_width)
        || (vision_tag_quad_from_extremes(extremes, min_size, corners) == 0U))
    {
        return;
    }

    score = vision_tag_score_quad(image, image_width, image_height, stride,
                                  threshold, corners, &contrast);
    if ((score <= best->score) || (score < s_config.min_confidence))
    {
        return;
    }

    for (index = 0U; index < 4U; ++index)
    {
        edge_sum += vision_tag_isqrt_u32(vision_tag_point_distance_sq(&corners[index],
                                                                      &corners[(index + 1U) & 3U]));
        best->corners[index] = corners[index];
    }
    best->x = min_x;
    best->y = min_y;
    best->width = static_cast<int16_t>(bbox_width);
    best->height = static_cast<int16_t>(bbox_height);
    best->center_x = static_cast<int16_t>((static_cast<int32_t>(corners[0].x) + corners[1].x
                              + corners[2].x + corners[3].x) / 4L);
    best->center_y = static_cast<int16_t>((static_cast<int32_t>(corners[0].y) + corners[1].y
                              + corners[2].y + corners[3].y) / 4L);
    best->size_px = static_cast<uint16_t>(edge_sum / 4U);
    best->contrast = contrast;
    best->score = score;
    best->has_corners = 1U;
    best->perspective_corrected = 1U;
}

static void vision_tag_search_quadrilaterals(vision_tag_candidate_t *best,
                                             const uint8_t *image,
                                             uint16_t image_width,
                                             uint16_t image_height,
                                             uint16_t stride,
                                             uint8_t threshold,
                                             int16_t region_x0,
                                             int16_t region_y0,
                                             int16_t region_x1,
                                             int16_t region_y1,
                                             uint16_t min_size,
                                             uint16_t max_size)
{
    int16_t seed_x;
    int16_t seed_y;

    region_x0 = static_cast<int16_t>(vision_tag_clamp_i32(region_x0, 0, image_width));
    region_y0 = static_cast<int16_t>(vision_tag_clamp_i32(region_y0, 0, image_height));
    region_x1 = static_cast<int16_t>(vision_tag_clamp_i32(region_x1, 0, image_width));
    region_y1 = static_cast<int16_t>(vision_tag_clamp_i32(region_y1, 0, image_height));
    for(auto &value : s_cc_visited)
    {
        value = 0U;
    }

    for (seed_y = region_y0; seed_y < region_y1; ++seed_y)
    {
        for (seed_x = region_x0; seed_x < region_x1; ++seed_x)
        {
            uint32_t seed_index = static_cast<uint32_t>(static_cast<uint16_t>(seed_y)) * image_width + static_cast<uint16_t>(seed_x);
            uint32_t stack_size;
            uint32_t component_pixels;
            int16_t min_x;
            int16_t min_y;
            int16_t max_x;
            int16_t max_y;
            vision_tag_point_t extremes[VISION_TAG_QUAD_EXTREMES];

            if ((vision_tag_visited_get(seed_index) != 0U)
                || (image[static_cast<uint32_t>(static_cast<uint16_t>(seed_y)) * stride + static_cast<uint16_t>(seed_x)] >= threshold))
            {
                continue;
            }

            vision_tag_visited_set(seed_index);
            s_cc_stack[0] = static_cast<uint16_t>(seed_index);
            stack_size = 1U;
            component_pixels = 0U;
            min_x = seed_x; max_x = seed_x;
            min_y = seed_y; max_y = seed_y;
            {
                uint8_t extreme;
                for (extreme = 0U; extreme < VISION_TAG_QUAD_EXTREMES; ++extreme)
                {
                    extremes[extreme].x = seed_x;
                    extremes[extreme].y = seed_y;
                }
            }

            while (stack_size != 0U)
            {
                uint16_t packed = s_cc_stack[--stack_size];
                int16_t x = static_cast<int16_t>(packed % image_width);
                int16_t y = static_cast<int16_t>(packed / image_width);
                int32_t difference = static_cast<int32_t>(x) - y;
                int32_t sum = static_cast<int32_t>(x) + y;
                int8_t dy;

                ++component_pixels;
                if (x < min_x) { min_x = x; }
                if (x > max_x) { max_x = x; }
                if (y < min_y) { min_y = y; }
                if (y > max_y) { max_y = y; }

                if ((y < extremes[0].y) || ((y == extremes[0].y) && (x < extremes[0].x)))
                    { extremes[0].x = x; extremes[0].y = y; }
                if ((difference > static_cast<int32_t>(extremes[1].x) - extremes[1].y)
                    || ((difference == static_cast<int32_t>(extremes[1].x) - extremes[1].y) && (x > extremes[1].x)))
                    { extremes[1].x = x; extremes[1].y = y; }
                if ((x > extremes[2].x) || ((x == extremes[2].x) && (y < extremes[2].y)))
                    { extremes[2].x = x; extremes[2].y = y; }
                if ((sum > static_cast<int32_t>(extremes[3].x) + extremes[3].y)
                    || ((sum == static_cast<int32_t>(extremes[3].x) + extremes[3].y) && (x > extremes[3].x)))
                    { extremes[3].x = x; extremes[3].y = y; }
                if ((y > extremes[4].y) || ((y == extremes[4].y) && (x > extremes[4].x)))
                    { extremes[4].x = x; extremes[4].y = y; }
                if ((difference < static_cast<int32_t>(extremes[5].x) - extremes[5].y)
                    || ((difference == static_cast<int32_t>(extremes[5].x) - extremes[5].y) && (x < extremes[5].x)))
                    { extremes[5].x = x; extremes[5].y = y; }
                if ((x < extremes[6].x) || ((x == extremes[6].x) && (y > extremes[6].y)))
                    { extremes[6].x = x; extremes[6].y = y; }
                if ((sum < static_cast<int32_t>(extremes[7].x) + extremes[7].y)
                    || ((sum == static_cast<int32_t>(extremes[7].x) + extremes[7].y) && (x < extremes[7].x)))
                    { extremes[7].x = x; extremes[7].y = y; }

                for (dy = -1; dy <= 1; ++dy)
                {
                    int8_t dx;
                    for (dx = -1; dx <= 1; ++dx)
                    {
                        int16_t nx;
                        int16_t ny;
                        uint32_t neighbor_index;
                        if ((dx == 0) && (dy == 0)) { continue; }
                        nx = static_cast<int16_t>(x + dx);
                        ny = static_cast<int16_t>(y + dy);
                        if ((nx < region_x0) || (nx >= region_x1)
                            || (ny < region_y0) || (ny >= region_y1))
                        {
                            continue;
                        }
                        neighbor_index = static_cast<uint32_t>(static_cast<uint16_t>(ny)) * image_width + static_cast<uint16_t>(nx);
                        if ((vision_tag_visited_get(neighbor_index) == 0U)
                            && (image[static_cast<uint32_t>(static_cast<uint16_t>(ny)) * stride + static_cast<uint16_t>(nx)] < threshold))
                        {
                            vision_tag_visited_set(neighbor_index);
                            s_cc_stack[stack_size++] = static_cast<uint16_t>(neighbor_index);
                        }
                    }
                }
            }

            vision_tag_try_quad_component(best, image, image_width, image_height, stride,
                                          threshold, extremes, min_x, min_y, max_x, max_y,
                                          component_pixels, min_size, max_size);
        }
    }
}

static void vision_tag_search_region(vision_tag_candidate_t *best,
                                     uint16_t image_width,
                                     uint16_t image_height,
                                     int16_t region_x0,
                                     int16_t region_y0,
                                     int16_t region_x1,
                                     int16_t region_y1,
                                     uint16_t min_size,
                                     uint16_t max_size,
                                     uint8_t include_aspect_variants)
{
    uint16_t size;

    region_x0 = static_cast<int16_t>(vision_tag_clamp_i32(region_x0, 0, image_width));
    region_y0 = static_cast<int16_t>(vision_tag_clamp_i32(region_y0, 0, image_height));
    region_x1 = static_cast<int16_t>(vision_tag_clamp_i32(region_x1, 0, image_width));
    region_y1 = static_cast<int16_t>(vision_tag_clamp_i32(region_y1, 0, image_height));

    for (size = min_size; size <= max_size; )
    {
        /*
         * 黑边只有约 1/8 边长，尺度跨得太大会直接错过边框。尺寸固定每次加 2 px；
         * 坐标至少 2 px 一步，大目标再适度加粗，兼顾可靠性与全图重捕获耗时。
         */
        uint16_t size_step = 2U;
        uint16_t position_step = vision_tag_max_u16(2U, static_cast<uint16_t>(size / 24U));
        uint8_t aspect_index;
        uint8_t aspect_count = (include_aspect_variants != 0U) ? 3U : 1U;

        for (aspect_index = 0U; aspect_index < aspect_count; ++aspect_index)
        {
            int16_t candidate_width;
            int16_t candidate_height;
            int16_t x;
            int16_t y;

            if (aspect_index == 0U)
            {
                candidate_width = static_cast<int16_t>(size);
                candidate_height = static_cast<int16_t>(size);
            }
            else if (aspect_index == 1U)
            {
                candidate_width = static_cast<int16_t>((size * 3U) / 4U);
                candidate_height = static_cast<int16_t>(size);
            }
            else
            {
                candidate_width = static_cast<int16_t>(size);
                candidate_height = static_cast<int16_t>((size * 3U) / 4U);
            }

            if ((candidate_width < 12) || (candidate_height < 12))
            {
                continue;
            }

            for (y = region_y0; static_cast<int32_t>(y) + candidate_height <= region_y1; y = static_cast<int16_t>(y + position_step))
            {
                for (x = region_x0; static_cast<int32_t>(x) + candidate_width <= region_x1; x = static_cast<int16_t>(x + position_step))
                {
                    vision_tag_try_candidate(best, x, y, candidate_width, candidate_height,
                                             image_width, image_height);
                }
            }
        }

        if (static_cast<uint32_t>(size) + size_step > max_size)
        {
            break;
        }
        size = static_cast<uint16_t>(size + size_step);
    }
}

static vision_tag_candidate_t vision_tag_find_best(const uint8_t *image,
                                                   uint16_t width,
                                                   uint16_t height,
                                                   uint16_t stride,
                                                   uint8_t threshold,
                                                   uint8_t allow_full_search)
{
    vision_tag_candidate_t best;
    uint16_t max_size = s_config.max_size_px;

    best = {};
    if ((max_size == 0U) || (max_size > vision_tag_min_u16(width, height)))
    {
        max_size = vision_tag_min_u16(width, height);
    }

    /* 锁定或刚命中时先搜预测点附近，可显著降低平均计算量。 */
    if ((s_locked != 0U) || (s_hit_streak != 0U))
    {
        int16_t center_x = static_cast<int16_t>(s_center_x_q8 / VISION_TAG_Q8_ONE);
        int16_t center_y = static_cast<int16_t>(s_center_y_q8 / VISION_TAG_Q8_ONE);
        uint16_t old_size = static_cast<uint16_t>(vision_tag_clamp_i32(s_size_q8 / VISION_TAG_Q8_ONE,
                                                           s_config.min_size_px, max_size));
        uint16_t roi_half = vision_tag_max_u16(24U, static_cast<uint16_t>(old_size * 2U));
        uint16_t roi_min_size = vision_tag_max_u16(s_config.min_size_px,
                                                   static_cast<uint16_t>((old_size * 5U) / 8U));
        uint16_t roi_max_size = vision_tag_min_u16(max_size,
                                                   static_cast<uint16_t>((old_size * 13U) / 8U));

        vision_tag_search_region(&best, width, height,
                                 static_cast<int16_t>(center_x - static_cast<int16_t>(roi_half)),
                                 static_cast<int16_t>(center_y - static_cast<int16_t>(roi_half)),
                                 static_cast<int16_t>(center_x + static_cast<int16_t>(roi_half)),
                                 static_cast<int16_t>(center_y + static_cast<int16_t>(roi_half)),
                                 roi_min_size, roi_max_size, 1U);

        if (best.score < s_config.min_confidence)
        {
            vision_tag_search_quadrilaterals(&best, image, width, height, stride, threshold,
                                             static_cast<int16_t>(center_x - static_cast<int16_t>(roi_half)),
                                             static_cast<int16_t>(center_y - static_cast<int16_t>(roi_half)),
                                             static_cast<int16_t>(center_x + static_cast<int16_t>(roi_half)),
                                             static_cast<int16_t>(center_y + static_cast<int16_t>(roi_half)),
                                             roi_min_size, roi_max_size);
        }
    }

    /* ROI 未达到门限时立即全图重捕获，避免错误锁定把真正目标排除在外。 */
    if ((best.score < s_config.min_confidence) && (allow_full_search != 0U))
    {
        best = {};
        /* Connected black-border components provide four corners for both
         * rotated and perspective-compressed tags.  Exact ID0 decoding is
         * performed after projective normalization. */
        vision_tag_search_quadrilaterals(&best, image, width, height, stride, threshold,
                                         0, 0, static_cast<int16_t>(width), static_cast<int16_t>(height),
                                         s_config.min_size_px, max_size);

        /* The printed marker is square when first acquired.  Scanning only
         * square candidates cuts the expensive full-frame search to roughly
         * one third.  After lock, the ROI path above still tests 3:4 and 4:3
         * candidates so normal perspective changes remain supported. */
        if (best.score < s_config.min_confidence)
        {
            vision_tag_search_region(&best, width, height, 0, 0,
                                     static_cast<int16_t>(width), static_cast<int16_t>(height),
                                     s_config.min_size_px, max_size, 0U);
        }
    }

    return best;
}

static uint16_t vision_tag_measure_contrast(const uint8_t *image,
                                            uint16_t stride,
                                            const vision_tag_candidate_t *candidate)
{
    int16_t border_x = static_cast<int16_t>(candidate->width / 8);
    int16_t border_y = static_cast<int16_t>(candidate->height / 8);
    int16_t x0 = candidate->x;
    int16_t y0 = candidate->y;
    int16_t x1 = static_cast<int16_t>(x0 + candidate->width);
    int16_t y1 = static_cast<int16_t>(y0 + candidate->height);
    uint32_t ring_sum = 0U;
    uint32_t quiet_sum = 0U;
    uint32_t ring_count = 0U;
    uint32_t quiet_count = 0U;
    int16_t x;
    int16_t y;

    if (border_x < 2)
    {
        border_x = 2;
    }
    if (border_y < 2)
    {
        border_y = 2;
    }

    for (y = static_cast<int16_t>(y0 - border_y); y < static_cast<int16_t>(y1 + border_y); ++y)
    {
        const uint8_t *row = image + static_cast<uint32_t>(y) * stride;
        for (x = static_cast<int16_t>(x0 - border_x); x < static_cast<int16_t>(x1 + border_x); ++x)
        {
            if ((x < x0) || (x >= x1) || (y < y0) || (y >= y1))
            {
                quiet_sum += row[x];
                ++quiet_count;
            }
            else if ((x < static_cast<int16_t>(x0 + border_x)) || (x >= static_cast<int16_t>(x1 - border_x))
                     || (y < static_cast<int16_t>(y0 + border_y)) || (y >= static_cast<int16_t>(y1 - border_y)))
            {
                ring_sum += row[x];
                ++ring_count;
            }
        }
    }

    if ((ring_count == 0U) || (quiet_count == 0U))
    {
        return 0U;
    }
    {
        uint32_t ring_mean = ring_sum / ring_count;
        uint32_t quiet_mean = quiet_sum / quiet_count;
        return static_cast<uint16_t>((quiet_mean > ring_mean) ? (quiet_mean - ring_mean) : 0U);
    }
}

static void vision_tag_update_bbox_from_filter(uint16_t width, uint16_t height)
{
    int32_t center_x = s_center_x_q8 / VISION_TAG_Q8_ONE;
    int32_t center_y = s_center_y_q8 / VISION_TAG_Q8_ONE;
    int32_t size = s_size_q8 / VISION_TAG_Q8_ONE;
    int32_t x;
    int32_t y;

    center_x = vision_tag_clamp_i32(center_x, 0, static_cast<int32_t>(width) - 1);
    center_y = vision_tag_clamp_i32(center_y, 0, static_cast<int32_t>(height) - 1);
    size = vision_tag_clamp_i32(size, 0, vision_tag_min_u16(width, height));
    x = center_x - size / 2;
    y = center_y - size / 2;
    x = vision_tag_clamp_i32(x, 0, static_cast<int32_t>(width) - 1);
    y = vision_tag_clamp_i32(y, 0, static_cast<int32_t>(height) - 1);

    s_result.center_x = static_cast<int16_t>(center_x);
    s_result.center_y = static_cast<int16_t>(center_y);
    s_result.size_px = static_cast<uint16_t>(size);
    s_result.bbox.x = static_cast<int16_t>(x);
    s_result.bbox.y = static_cast<int16_t>(y);
    s_result.bbox.width = static_cast<int16_t>(vision_tag_clamp_i32(size, 0, static_cast<int32_t>(width) - x));
    s_result.bbox.height = static_cast<int16_t>(vision_tag_clamp_i32(size, 0, static_cast<int32_t>(height) - y));
}

static void vision_tag_update_errors(uint16_t width, uint16_t height)
{
    int32_t error_x = static_cast<int32_t>(s_result.center_x) - static_cast<int32_t>(width / 2U);
    int32_t error_y = static_cast<int32_t>(s_result.center_y) - static_cast<int32_t>(height / 2U);
    int32_t half_width = static_cast<int32_t>(width / 2U);
    int32_t normalized = 0;

    if (half_width > 0)
    {
        normalized = (error_x * VISION_TAG_Q15_MAX) / half_width;
    }

    s_result.error_x_px = static_cast<int16_t>(error_x);
    s_result.error_y_px = static_cast<int16_t>(error_y);
    s_result.error_x_q15 = static_cast<int16_t>(vision_tag_clamp_i32(normalized,
                                                         -VISION_TAG_Q15_MAX,
                                                         VISION_TAG_Q15_MAX));
}

static void vision_tag_update_distance()
{
    s_result.distance_mm = 0U;
    s_result.distance_zone = static_cast<uint8_t>(VISION_TAG_DISTANCE_UNKNOWN);
    if ((s_result.valid == 0U) || (s_result.size_px == 0U)
        || (s_config.distance_scale_mm_px == 0U))
    {
        return;
    }

    {
        uint32_t distance = (s_config.distance_scale_mm_px + s_result.size_px / 2U)
                          / s_result.size_px;
        s_result.distance_mm = static_cast<uint16_t>((distance > 65535U) ? 65535U : distance);
    }

    if ((s_config.follow_near_mm == 0U) || (s_config.follow_far_mm == 0U)
        || (s_config.follow_near_mm >= s_config.follow_far_mm))
    {
        return;
    }
    if (s_result.distance_mm < s_config.follow_near_mm)
    {
        s_result.distance_zone = static_cast<uint8_t>(VISION_TAG_DISTANCE_TOO_CLOSE);
    }
    else if (s_result.distance_mm > s_config.follow_far_mm)
    {
        s_result.distance_zone = static_cast<uint8_t>(VISION_TAG_DISTANCE_TOO_FAR);
    }
    else
    {
        s_result.distance_zone = static_cast<uint8_t>(VISION_TAG_DISTANCE_IN_RANGE);
    }
}

void vision_tag_tracker_default_config(vision_tag_config_t *config)
{
    if (config == nullptr)
    {
        return;
    }

    config->min_size_px = 14U;
    config->max_size_px = 96U;
    /* 粗步进全图搜索会使最佳框与真实边缘相差 1~2 px，650 为较稳妥的首版门限。 */
    config->min_confidence = 650U;
    config->acquire_frames = 2U;
    config->coast_frames = 3U;
    config->full_search_interval = 4U;
    config->position_alpha_q8 = 176U;
    config->velocity_beta_q8 = 36U;
    config->size_alpha_q8 = 144U;
    config->min_tag_contrast = 20U;
    config->distance_scale_mm_px = 0U;
    config->follow_near_mm = 0U;
    config->follow_far_mm = 0U;
}

uint32_t vision_tag_distance_scale_from_sample(uint16_t known_distance_mm,
                                               uint16_t observed_size_px)
{
    return static_cast<uint32_t>(known_distance_mm) * static_cast<uint32_t>(observed_size_px);
}

void vision_tag_tracker_init(const vision_tag_config_t *config)
{
    vision_tag_config_t defaults;

    vision_tag_tracker_default_config(&defaults);
    s_config = (config != nullptr) ? *config : defaults;

    if (s_config.min_size_px < 12U)
    {
        s_config.min_size_px = 12U;
    }
    if ((s_config.min_confidence == 0U) || (s_config.min_confidence > VISION_TAG_SCORE_MAX))
    {
        s_config.min_confidence = defaults.min_confidence;
    }
    if (s_config.acquire_frames == 0U)
    {
        s_config.acquire_frames = 1U;
    }
    if (s_config.coast_frames == 255U)
    {
        s_config.coast_frames = 254U;
    }
    if (s_config.full_search_interval == 0U)
    {
        s_config.full_search_interval = 1U;
    }
    if ((s_config.max_size_px != 0U) && (s_config.max_size_px < s_config.min_size_px))
    {
        s_config.max_size_px = s_config.min_size_px;
    }

    s_result = {};
    s_center_x_q8 = 0;
    s_center_y_q8 = 0;
    s_velocity_x_q8 = 0;
    s_velocity_y_q8 = 0;
    s_size_q8 = 0;
    s_hit_streak = 0U;
    s_locked = 0U;
    s_full_search_countdown = 0U;
}

const vision_tag_result_t *vision_tag_tracker_process(const uint8_t *image,
                                                       uint16_t width,
                                                       uint16_t height,
                                                       uint16_t stride)
{
    vision_tag_candidate_t candidate;
    vision_frame_stats_t frame_stats;
    uint8_t threshold;
    uint8_t global_threshold;
    uint8_t allow_full_search;
    uint8_t skip_search;
    uint32_t next_frame_id = s_result.frame_id + 1U;

    if ((image == nullptr) || (width == 0U) || (height == 0U) || (stride < width)
        || (width > VISION_TAG_MAX_WIDTH) || (height > VISION_TAG_MAX_HEIGHT))
    {
        s_result = {};
        s_result.frame_id = next_frame_id;
        s_center_x_q8 = 0;
        s_center_y_q8 = 0;
        s_velocity_x_q8 = 0;
        s_velocity_y_q8 = 0;
        s_size_q8 = 0;
        s_locked = 0U;
        s_hit_streak = 0U;
        s_full_search_countdown = 0U;
        return &s_result;
    }

    s_result.frame_id = next_frame_id;
    s_result.detected = 0U;
    s_result.predicted = 0U;
    s_result.tag_contrast = 0U;
    s_result.has_corners = 0U;
    s_result.perspective_corrected = 0U;
    candidate = {};
    frame_stats = {};

    /*
     * 锁定后每帧只搜预测 ROI；ROI 第一次失败时允许全图重捕获。
     * 持续未锁定时按 full_search_interval 降低全图扫描频率，避免无目标时占满 20 ms 帧周期。
     */
    skip_search = static_cast<uint8_t>(((s_locked == 0U) && (s_hit_streak == 0U)
                          && (s_full_search_countdown != 0U)) ? 1U : 0U);
    allow_full_search = static_cast<uint8_t>((s_full_search_countdown == 0U) ? 1U : 0U);

    if (skip_search != 0U)
    {
        --s_full_search_countdown;
    }
    else
    {
        threshold = vision_tag_otsu_threshold(image, width, height, stride,
                                              &frame_stats, &global_threshold);
        s_result.threshold = threshold;
        s_result.gray_p10 = frame_stats.p10;
        s_result.gray_p50 = frame_stats.p50;
        s_result.gray_p90 = frame_stats.p90;
        s_result.dark_permille = frame_stats.dark_permille;
        s_result.saturated_permille = frame_stats.saturated_permille;
        vision_tag_build_integral(image, width, height, stride, threshold);
        candidate = vision_tag_find_best(image, width, height, stride, threshold,
                                         static_cast<uint8_t>((threshold == global_threshold)
                                                   ? allow_full_search : 0U));

        /* A locked ROI uses local Otsu. If it fails, rebuild once with global Otsu before full recovery. */
        if ((candidate.score < s_config.min_confidence)
            && (allow_full_search != 0U) && (threshold != global_threshold))
        {
            threshold = global_threshold;
            s_result.threshold = threshold;
            vision_tag_build_integral(image, width, height, stride, threshold);
            candidate = vision_tag_find_best(image, width, height, stride, threshold, 1U);
        }

        if (candidate.score >= s_config.min_confidence)
        {
            s_result.tag_contrast = (candidate.perspective_corrected != 0U)
                                  ? candidate.contrast
                                  : vision_tag_measure_contrast(image, stride, &candidate);
            if (s_result.tag_contrast < s_config.min_tag_contrast)
            {
                candidate = {};
            }
        }

        if (candidate.score >= s_config.min_confidence)
        {
            s_full_search_countdown = 0U;
        }
        else if (allow_full_search != 0U)
        {
            s_full_search_countdown = static_cast<uint8_t>(s_config.full_search_interval - 1U);
        }
        else if (s_full_search_countdown != 0U)
        {
            --s_full_search_countdown;
        }
    }

    if (candidate.score >= s_config.min_confidence)
    {
        int32_t measured_x_q8 = static_cast<int32_t>(candidate.center_x) * VISION_TAG_Q8_ONE;
        int32_t measured_y_q8 = static_cast<int32_t>(candidate.center_y) * VISION_TAG_Q8_ONE;
        int32_t measured_size_q8 = static_cast<int32_t>(candidate.size_px) * VISION_TAG_Q8_ONE;
        uint8_t corner_index;

        s_result.detected = 1U;
        s_result.has_corners = candidate.has_corners;
        s_result.perspective_corrected = candidate.perspective_corrected;
        for (corner_index = 0U; corner_index < 4U; ++corner_index)
        {
            s_result.corners[corner_index] = candidate.corners[corner_index];
        }
        s_result.lost_frames = 0U;
        if (s_hit_streak < 255U)
        {
            ++s_hit_streak;
        }

        if ((s_locked == 0U) && (s_hit_streak == 1U))
        {
            s_center_x_q8 = measured_x_q8;
            s_center_y_q8 = measured_y_q8;
            s_size_q8 = measured_size_q8;
            s_velocity_x_q8 = 0;
            s_velocity_y_q8 = 0;
            s_result.confidence = candidate.score;
        }
        else
        {
            int32_t predicted_x_q8 = s_center_x_q8 + s_velocity_x_q8;
            int32_t predicted_y_q8 = s_center_y_q8 + s_velocity_y_q8;
            int32_t residual_x_q8 = measured_x_q8 - predicted_x_q8;
            int32_t residual_y_q8 = measured_y_q8 - predicted_y_q8;

            s_center_x_q8 = predicted_x_q8
                          + (residual_x_q8 * s_config.position_alpha_q8) / 256L;
            s_center_y_q8 = predicted_y_q8
                          + (residual_y_q8 * s_config.position_alpha_q8) / 256L;
            s_velocity_x_q8 += (residual_x_q8 * s_config.velocity_beta_q8) / 256L;
            s_velocity_y_q8 += (residual_y_q8 * s_config.velocity_beta_q8) / 256L;
            s_size_q8 += ((measured_size_q8 - s_size_q8) * s_config.size_alpha_q8) / 256L;
            s_result.confidence = static_cast<uint16_t>((static_cast<uint32_t>(s_result.confidence)
                                            + static_cast<uint32_t>(candidate.score) * 3U) / 4U);
        }

        if (s_hit_streak >= s_config.acquire_frames)
        {
            s_locked = 1U;
        }
        s_result.valid = s_locked;
    }
    else
    {
        s_hit_streak = 0U;
        if (s_result.lost_frames < 255U)
        {
            ++s_result.lost_frames;
        }

        if ((s_locked != 0U) && (s_result.lost_frames <= s_config.coast_frames))
        {
            s_center_x_q8 += s_velocity_x_q8;
            s_center_y_q8 += s_velocity_y_q8;
            s_velocity_x_q8 = (s_velocity_x_q8 * 3L) / 4L;
            s_velocity_y_q8 = (s_velocity_y_q8 * 3L) / 4L;
            s_result.confidence = static_cast<uint16_t>((static_cast<uint32_t>(s_result.confidence) * 3U) / 4U);
            s_result.valid = 1U;
            s_result.predicted = 1U;
        }
        else
        {
            s_locked = 0U;
            s_result.valid = 0U;
            s_result.predicted = 0U;
            s_result.confidence = 0U;
            s_velocity_x_q8 = 0;
            s_velocity_y_q8 = 0;
            s_center_x_q8 = static_cast<int32_t>(width / 2U) * VISION_TAG_Q8_ONE;
            s_center_y_q8 = static_cast<int32_t>(height / 2U) * VISION_TAG_Q8_ONE;
            s_size_q8 = 0;
        }
    }

    vision_tag_update_bbox_from_filter(width, height);
    vision_tag_update_errors(width, height);
    vision_tag_update_distance();
    return &s_result;
}

const vision_tag_result_t *vision_tag_tracker_get_result()
{
    return &s_result;
}

#if defined(__TASKING__)
#pragma section code restore
#endif

