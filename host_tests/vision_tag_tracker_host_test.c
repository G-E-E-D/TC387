#if defined(__TASKING__)

typedef int vision_tag_tracker_host_test_is_not_target_firmware;

#else

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vision_tag_tracker.h"

#define TEST_W 188
#define TEST_H 120

static uint8_t frame[TEST_H][TEST_W];
static const uint8_t tag36h11_id0_rows[6] = {0x0AU, 0x22U, 0x27U, 0x17U, 0x29U, 0x3BU};

static void clear_frame(uint8_t value)
{
    memset(frame, value, sizeof(frame));
}

static int tag36h11_id0_bit(int row, int column, int rotation)
{
    int source_row;
    int source_column;

    switch (rotation & 3)
    {
        case 1: source_row = 5 - column; source_column = row; break;
        case 2: source_row = 5 - row; source_column = 5 - column; break;
        case 3: source_row = column; source_column = 5 - row; break;
        default: source_row = row; source_column = column; break;
    }
    return (tag36h11_id0_rows[source_row] >> (5 - source_column)) & 1;
}

static void draw_tag_rect_rotation(int center_x, int center_y, int width, int height, int rotation)
{
    int x0 = center_x - width / 2;
    int y0 = center_y - height / 2;
    int x;
    int y;

    for (y = y0; y < y0 + height; ++y)
    {
        for (x = x0; x < x0 + width; ++x)
        {
            int grid_x = ((x - x0) * 8) / width;
            int grid_y = ((y - y0) * 8) / height;
            int is_border = (grid_x == 0) || (grid_x == 7) || (grid_y == 0) || (grid_y == 7);
            if (is_border)
            {
                frame[y][x] = 18U;
            }
            else
            {
                int bit = tag36h11_id0_bit(grid_y - 1, grid_x - 1, rotation);
                frame[y][x] = (bit != 0) ? 25U : 225U;
            }
        }
    }
}

static void draw_tag_rect(int center_x, int center_y, int width, int height)
{
    draw_tag_rect_rotation(center_x, center_y, width, height, 0);
}

static void draw_tag(int center_x, int center_y, int size)
{
    draw_tag_rect(center_x, center_y, size, size);
}

static void draw_tag_quad(const vision_tag_point_t corners[4])
{
    double dx1 = (double)corners[1].x - corners[2].x;
    double dx2 = (double)corners[3].x - corners[2].x;
    double dx3 = (double)corners[0].x - corners[1].x + corners[2].x - corners[3].x;
    double dy1 = (double)corners[1].y - corners[2].y;
    double dy2 = (double)corners[3].y - corners[2].y;
    double dy3 = (double)corners[0].y - corners[1].y + corners[2].y - corners[3].y;
    double denominator = dx1 * dy2 - dx2 * dy1;
    double g = (dx3 * dy2 - dx2 * dy3) / denominator;
    double h = (dx1 * dy3 - dx3 * dy1) / denominator;
    double a = corners[1].x - corners[0].x + g * corners[1].x;
    double b = corners[3].x - corners[0].x + h * corners[3].x;
    double c = corners[0].x;
    double d = corners[1].y - corners[0].y + g * corners[1].y;
    double e = corners[3].y - corners[0].y + h * corners[3].y;
    double f = corners[0].y;
    int sample_y;

    for (sample_y = 0; sample_y < 384; ++sample_y)
    {
        int sample_x;
        double v = ((double)sample_y + 0.5) / 384.0;
        int grid_y = (int)(v * 8.0);
        if (grid_y > 7) grid_y = 7;
        for (sample_x = 0; sample_x < 384; ++sample_x)
        {
            double u = ((double)sample_x + 0.5) / 384.0;
            double weight = g * u + h * v + 1.0;
            int grid_x = (int)(u * 8.0);
            int x = (int)((a * u + b * v + c) / weight + 0.5);
            int y = (int)((d * u + e * v + f) / weight + 0.5);
            int is_border;
            int bit;
            if (grid_x > 7) grid_x = 7;
            if ((x < 0) || (x >= TEST_W) || (y < 0) || (y >= TEST_H)) continue;
            is_border = (grid_x == 0) || (grid_x == 7) || (grid_y == 0) || (grid_y == 7);
            bit = is_border ? 1 : tag36h11_id0_bit(grid_y - 1, grid_x - 1, 0);
            frame[y][x] = (bit != 0) ? 18U : 225U;
        }
    }
}

static void draw_filled_dark_rectangle(int center_x, int center_y, int width, int height)
{
    int x0 = center_x - width / 2;
    int y0 = center_y - height / 2;
    int x;
    int y;
    for (y = y0; y < y0 + height; ++y)
    {
        for (x = x0; x < x0 + width; ++x)
        {
            frame[y][x] = 18U;
        }
    }
}

static void draw_wrong_code_tag(int center_x, int center_y, int size)
{
    int x0 = center_x - size / 2;
    int y0 = center_y - size / 2;
    int x;
    int y;

    for (y = y0; y < y0 + size; ++y)
    {
        for (x = x0; x < x0 + size; ++x)
        {
            int grid_x = ((x - x0) * 8) / size;
            int grid_y = ((y - y0) * 8) / size;
            int is_border = (grid_x == 0) || (grid_x == 7) || (grid_y == 0) || (grid_y == 7);
            int bit = (grid_x + grid_y) & 1;
            frame[y][x] = (is_border || (bit != 0)) ? 18U : 225U;
        }
    }
}

static void add_noise(void)
{
    uint32_t state = 0x12345678U;
    int x;
    int y;
    for (y = 0; y < TEST_H; ++y)
    {
        for (x = 0; x < TEST_W; ++x)
        {
            int value;
            state = state * 1664525U + 1013904223U;
            value = (int)frame[y][x] + (int)((state >> 28) & 7U) - 3;
            if (value < 0) value = 0;
            if (value > 255) value = 255;
            frame[y][x] = (uint8_t)value;
        }
    }
}

static void apply_bright_gradient(void)
{
    int x;
    int y;
    for (y = 0; y < TEST_H; ++y)
    {
        for (x = 0; x < TEST_W; ++x)
        {
            int value = 96 + ((int)frame[y][x] * 159) / 255 + x / 5;
            frame[y][x] = (uint8_t)((value > 255) ? 255 : value);
        }
    }
}

static void wash_out_tag(void)
{
    int x;
    int y;
    for (y = 0; y < TEST_H; ++y)
    {
        for (x = 0; x < TEST_W; ++x)
        {
            if (frame[y][x] < 100U)
            {
                frame[y][x] = 238U;
            }
            else if (frame[y][x] < 255U)
            {
                frame[y][x] = 252U;
            }
        }
    }
}

static void require_true(int condition, const char *message)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    const vision_tag_result_t *result;
    vision_tag_config_t config;
    int i;

    vision_tag_tracker_init(NULL);

    for (i = 0; i < 2; ++i)
    {
        clear_frame(230U);
        draw_tag(94, 60, 40);
        add_noise();
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid != 0U, "center tag should acquire after two frames");
    require_true(abs(result->error_x_px) <= 3, "center error should be near zero");
    require_true(result->confidence >= 650U, "center tag confidence should pass threshold");

    for (i = 0; i < 4; ++i)
    {
        clear_frame(230U);
        draw_tag(126, 60, 40);
        add_noise();
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid != 0U, "moving tag should stay locked");
    require_true(result->error_x_px > 20, "right-side tag must produce positive error");

    clear_frame(230U);
    result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    require_true((result->valid != 0U) && (result->predicted != 0U),
                 "one missing frame should use prediction");

    for (i = 0; i < 4; ++i)
    {
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid == 0U, "long loss must invalidate output");
    require_true((result->error_x_px == 0) && (result->size_px == 0U),
                 "long loss must clear stale center error and size");
    require_true((result->bbox.width == 0) && (result->bbox.height == 0),
                 "long loss must clear stale bbox");

    for (i = 0; i < 5; ++i)
    {
        clear_frame(230U);
        draw_tag(62, 58, 36);
        add_noise();
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
        if (result->valid != 0U)
        {
            break;
        }
    }
    require_true(result->valid != 0U, "tag should reacquire");
    require_true(result->error_x_px < -20, "left-side tag must produce negative error");
    require_true(result->error_x_q15 < 0, "normalized left error must be negative");

    vision_tag_tracker_init(NULL);
    for (i = 0; i < 2; ++i)
    {
        clear_frame(230U);
        draw_filled_dark_rectangle(94, 60, 40, 40);
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid == 0U, "solid dark distractor must not be accepted as a tag");

    vision_tag_tracker_init(NULL);
    for (i = 0; i < 4; ++i)
    {
        clear_frame(230U);
        draw_wrong_code_tag(94, 60, 40);
        add_noise();
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid == 0U, "wrong 6x6 code must not be accepted as ID 0");

    vision_tag_tracker_init(NULL);
    for (i = 0; i < 2; ++i)
    {
        clear_frame(230U);
        draw_tag_rect(94, 60, 30, 40);
        add_noise();
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid != 0U, "3:4 perspective-compressed tag should acquire");
    require_true(abs(result->error_x_px) <= 3, "compressed tag center should remain accurate");

    vision_tag_tracker_init(NULL);
    for (i = 0; i < 2; ++i)
    {
        static const vision_tag_point_t tilted[4] =
        {
            {94, 24}, {139, 58}, {96, 98}, {49, 62}
        };
        clear_frame(230U);
        draw_tag_quad(tilted);
        add_noise();
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid != 0U, "projectively tilted tag should acquire");
    require_true(result->perspective_corrected != 0U,
                 "tilted tag should use projective code sampling");
    require_true(result->has_corners != 0U, "tilted tag should report four corners");
    require_true(abs(result->error_x_px) <= 5, "tilted tag center should remain accurate");

    vision_tag_tracker_init(NULL);
    for (i = 0; i < 2; ++i)
    {
        clear_frame(230U);
        draw_tag_rect_rotation(94, 60, 40, 40, 1);
        add_noise();
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid != 0U, "90-degree rotated ID 0 tag should acquire");

    vision_tag_tracker_init(NULL);
    for (i = 0; i < 2; ++i)
    {
        clear_frame(235U);
        draw_tag(104, 60, 40);
        apply_bright_gradient();
        add_noise();
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid != 0U, "high-brightness gradient tag should acquire");
    require_true(result->gray_p90 >= 240U, "bright scene percentile should be reported");
    require_true(result->saturated_permille > 50U, "bright scene saturation should be reported");
    require_true(result->tag_contrast >= 28U, "accepted bright tag must retain raw contrast");

    vision_tag_tracker_init(NULL);
    for (i = 0; i < 3; ++i)
    {
        clear_frame(255U);
        draw_tag(94, 60, 40);
        wash_out_tag();
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid == 0U, "washed-out low-contrast tag must be rejected");

    vision_tag_tracker_default_config(&config);
    config.distance_scale_mm_px = vision_tag_distance_scale_from_sample(1000U, 40U);
    config.follow_near_mm = 850U;
    config.follow_far_mm = 1150U;
    vision_tag_tracker_init(&config);
    for (i = 0; i < 2; ++i)
    {
        clear_frame(230U);
        draw_tag(94, 60, 40);
        result = vision_tag_tracker_process(&frame[0][0], TEST_W, TEST_H, TEST_W);
    }
    require_true(result->valid != 0U, "distance calibration sample should acquire");
    require_true((result->distance_mm >= 900U) && (result->distance_mm <= 1100U),
                 "one-point distance estimate should match its calibration sample");
    require_true(result->distance_zone == VISION_TAG_DISTANCE_IN_RANGE,
                 "calibrated sample should classify inside follow range");

    result = vision_tag_tracker_process(NULL, TEST_W, TEST_H, TEST_W);
    require_true(result->valid == 0U, "invalid input must invalidate output");
    require_true((result->confidence == 0U) && (result->size_px == 0U),
                 "invalid input must clear stale measurements");
    require_true((result->bbox.width == 0) && (result->bbox.height == 0),
                 "invalid input must clear stale bbox");

    printf("PASS all vision tracker tests\n");
    return 0;
}

#endif

