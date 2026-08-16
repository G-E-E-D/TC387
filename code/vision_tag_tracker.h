/*********************************************************************************************************************
 * @file        vision_tag_tracker.h
 * @brief       TC377 单目灰度相机高对比标志视觉跟踪
 *
 * 本模块只输出视觉测量，不包含 PID、舵机、电机或路径控制。
 * 利用 AprilTag 图案的“白色静区 + 黑色方形外框 + 内部黑白纹理”定位目标，
 * 并校验固定 tag36h11 ID 0 的 6 x 6 码字；不包含通用标签族枚举或三维位姿求解。
 ********************************************************************************************************************/
#ifndef VISION_TAG_TRACKER_H
#define VISION_TAG_TRACKER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 当前工程默认使用 188 x 120 总钻风。若修改摄像头分辨率，需要同步增大这里的上限。 */
#define VISION_TAG_MAX_WIDTH       (188U)
#define VISION_TAG_MAX_HEIGHT      (120U)
#define VISION_TAG_SCORE_MAX       (1000U)

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t width;
    int16_t height;
} vision_tag_bbox_t;

typedef struct
{
    int16_t x;
    int16_t y;
} vision_tag_point_t;

typedef enum
{
    VISION_TAG_DISTANCE_UNKNOWN = 0,
    VISION_TAG_DISTANCE_TOO_CLOSE,
    VISION_TAG_DISTANCE_IN_RANGE,
    VISION_TAG_DISTANCE_TOO_FAR
} vision_tag_distance_zone_t;

typedef struct
{
    uint8_t  detected;       /* 本帧是否得到了真实图像测量 */
    uint8_t  valid;          /* 本结果是否允许控制端使用 */
    uint8_t  predicted;      /* 1 表示短时丢失后的预测值，不是真实测量 */
    uint8_t  threshold;      /* 本帧 Otsu 二值化阈值，便于调试曝光 */

    int16_t  center_x;       /* 滤波后的目标中心，单位：像素 */
    int16_t  center_y;
    int16_t  error_x_px;     /* center_x - image_width / 2；右偏为正 */
    int16_t  error_y_px;
    int16_t  error_x_q15;    /* 归一化横向偏差，约为 [-32767, 32767] */

    uint16_t size_px;        /* 黑色外框平均边长，可作为距离的单调代理量 */
    uint16_t confidence;     /* 0..1000 */
    uint8_t  lost_frames;    /* 连续未检测到目标的帧数 */
    uint8_t  reserved;
    uint32_t frame_id;
    vision_tag_bbox_t bbox;     /* 滤波后的近似正方形包围框，不代表 AprilTag 四角姿态 */
    uint8_t  gray_p10;       /* Full-frame grayscale 10th percentile. */
    uint8_t  gray_p50;       /* Full-frame grayscale median. */
    uint8_t  gray_p90;       /* Full-frame grayscale 90th percentile. */
    uint8_t  reserved0;
    uint16_t dark_permille;  /* Pixels <= 15, in permille. */
    uint16_t saturated_permille; /* Pixels >= 248, in permille. */
    uint16_t tag_contrast;   /* Quiet-zone mean minus black-ring mean, 0..255. */
    uint16_t distance_mm;    /* One-point-calibrated pinhole estimate; 0 means unavailable. */
    uint8_t  distance_zone;  /* vision_tag_distance_zone_t */
    uint8_t  reserved1;
    vision_tag_point_t corners[4]; /* 本帧测得的四角，按图像中的顺时针顺序排列。 */
    uint8_t  has_corners;          /* 1 表示 corners 来自本帧真实测量。 */
    uint8_t  perspective_corrected;/* 1 表示使用了四边形透视采样。 */
    uint16_t reserved2;
} vision_tag_result_t;

/* This structure is a cross-core ABI.  A platform migration must not
 * silently change its byte layout. */
typedef char vision_tag_result_size_must_be_68[
    (sizeof(vision_tag_result_t) == 68U) ? 1 : -1];

typedef struct
{
    uint16_t min_size_px;        /* 允许的最小黑框边长 */
    uint16_t max_size_px;        /* 允许的最大黑框边长，0 表示自动 */
    uint16_t min_confidence;     /* 判为目标的最低分，0..1000 */
    uint8_t  acquire_frames;     /* 连续命中多少帧后 valid=1 */
    uint8_t  coast_frames;       /* 丢失后最多预测多少帧 */
    uint8_t  full_search_interval; /* 未锁定时每隔多少帧进行一次全图重捕获 */
    uint8_t  position_alpha_q8;  /* 位置滤波系数，0..255 */
    uint8_t  velocity_beta_q8;   /* 速度修正系数，0..255 */
    uint8_t  size_alpha_q8;      /* 尺度滤波系数，0..255 */
    uint8_t  min_tag_contrast; /* Reject washed-out candidates below this raw contrast. */
    uint32_t distance_scale_mm_px; /* known_distance_mm * observed_size_px; 0 disables distance. */
    uint16_t follow_near_mm;       /* Below this is TOO_CLOSE; 0 disables zoning. */
    uint16_t follow_far_mm;        /* Above this is TOO_FAR; 0 disables zoning. */
} vision_tag_config_t;

/** 填充推荐初值。调用者可修改后再传给 vision_tag_tracker_init。 */
void vision_tag_tracker_default_config(vision_tag_config_t *config);

/** 初始化单实例跟踪器；config 为 NULL 时使用推荐初值。 */
void vision_tag_tracker_init(const vision_tag_config_t *config);

/** Build the scale term from one measured frame at a known distance. */
uint32_t vision_tag_distance_scale_from_sample(uint16_t known_distance_mm,
                                               uint16_t observed_size_px);

/**
 * 处理一帧 8 bit 灰度图。
 *
 * @param image   首像素地址
 * @param width   有效宽度，不得超过 VISION_TAG_MAX_WIDTH
 * @param height  有效高度，不得超过 VISION_TAG_MAX_HEIGHT
 * @param stride  相邻两行首地址间隔；总钻风整帧可传 MT9V03X_W
 * @return        指向模块内部结果的只读指针
 * @note          只允许在调用 process() 的同一任务/CPU 中同步读取。跨核或 ISR 使用时需由上层复制到邮箱。
 */
const vision_tag_result_t *vision_tag_tracker_process(const uint8_t *image,
                                                       uint16_t width,
                                                       uint16_t height,
                                                       uint16_t stride);

/** 取得最近一次处理结果；同样仅适合同一任务/CPU 同步读取。 */
const vision_tag_result_t *vision_tag_tracker_get_result(void);

#ifdef __cplusplus
}
#endif

#endif /* VISION_TAG_TRACKER_H */


