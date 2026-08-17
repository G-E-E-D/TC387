#include "vehicle_guide_target.h"

#include <cmath>

#include "vehicle_config.h"
#include "vehicle_math.h"

static void clear_target(VehicleGuideTarget *target)
{
    if(target != nullptr)
    {
        *target = {};
    }
}

void vehicle_guide_target_default_config(VehicleGuideTargetConfig *config)
{
    if(config == nullptr)
    {
        return;
    }
    config->image_width = VISION_TAG_MAX_WIDTH;
    config->image_height = VISION_TAG_MAX_HEIGHT;
    config->min_confidence = VEHICLE_VISION_MIN_CONFIDENCE;
    config->min_size_px = 4U;
    config->tag_width_m = FORWARD_MIN_TAG_WIDTH_M;
    config->focal_x_px = FORWARD_DEFAULT_FX_PX;
    config->focal_y_px = FORWARD_DEFAULT_FY_PX;
    config->principal_x_px = 0.5f * static_cast<float>(VISION_TAG_MAX_WIDTH);
    config->principal_y_px = 0.5f * static_cast<float>(VISION_TAG_MAX_HEIGHT);
    config->camera_position_x_m = 0.0f;
    config->camera_position_y_m = 0.0f;
    config->camera_yaw_rad = 0.0f;
    config->minimum_safe_distance_m = GUIDE_TARGET_MIN_DISTANCE_M;
    config->target_timeout_us = GUIDE_TARGET_TIMEOUT_US;
}

bool vehicle_guide_target_config_is_valid(
    const VehicleGuideTargetConfig *config)
{
    return (config != nullptr) && (config->image_width > 0U) &&
           (config->image_height > 0U) &&
           (config->min_confidence <= VISION_TAG_SCORE_MAX) &&
           (config->min_size_px > 0U) &&
           (config->tag_width_m > 0.0f) &&
           (config->focal_x_px > 0.0f) && (config->focal_y_px > 0.0f) &&
           std::isfinite(config->principal_x_px) &&
           std::isfinite(config->principal_y_px) &&
           std::isfinite(config->camera_position_x_m) &&
           std::isfinite(config->camera_position_y_m) &&
           std::isfinite(config->camera_yaw_rad) &&
           (config->minimum_safe_distance_m > 0.0f) &&
           (config->target_timeout_us > 0U);
}

void vehicle_guide_target_init(VehicleGuideTargetTracker *tracker,
                               const VehicleGuideTargetConfig *config)
{
    VehicleGuideTargetConfig selected;
    if(tracker == nullptr)
    {
        return;
    }
    vehicle_guide_target_default_config(&selected);
    if(config != nullptr)
    {
        selected = *config;
    }
    *tracker = {};
    tracker->config = selected;
    tracker->initialized = vehicle_guide_target_config_is_valid(&selected);
    clear_target(&tracker->target);
}

static bool target_from_result(const VehicleGuideTargetConfig *config,
                               const vision_tag_result_t *result,
                               uint64_t timestamp_us,
                               uint32_t camera_sequence,
                               VehicleGuideTarget *target)
{
    float size_px;
    float distance_z_m;
    float lateral_camera_m;
    float camera_cos;
    float camera_sin;
    float x_m;
    float y_m;

    if((config == nullptr) || (result == nullptr) || (target == nullptr) ||
       (result->valid == 0U) || (result->detected == 0U) ||
       (result->predicted != 0U) ||
       (result->confidence < config->min_confidence) ||
       (result->size_px < config->min_size_px) ||
       (result->center_x < 0) ||
       (result->center_x >= static_cast<int16_t>(config->image_width)) ||
       (result->center_y < 0) ||
       (result->center_y >= static_cast<int16_t>(config->image_height)))
    {
        return false;
    }
    size_px = static_cast<float>(result->size_px);
    distance_z_m = (result->distance_mm > 0U)
        ? static_cast<float>(result->distance_mm) * 0.001f
        : (config->focal_x_px * config->tag_width_m / size_px);
    if(!(distance_z_m > 0.0f) || !std::isfinite(distance_z_m))
    {
        return false;
    }
    lateral_camera_m = -(static_cast<float>(result->center_x) -
                         config->principal_x_px) * distance_z_m /
                       config->focal_x_px;
    camera_cos = cosf(config->camera_yaw_rad);
    camera_sin = sinf(config->camera_yaw_rad);
    x_m = config->camera_position_x_m + camera_cos * distance_z_m -
          camera_sin * lateral_camera_m;
    y_m = config->camera_position_y_m + camera_sin * distance_z_m +
          camera_cos * lateral_camera_m;
    if(!std::isfinite(x_m) || !std::isfinite(y_m) || !(x_m > 0.0f))
    {
        return false;
    }
    target->timestamp_us = timestamp_us;
    target->age_us = 0U;
    target->camera_sequence = camera_sequence;
    target->center_x_px = result->center_x;
    target->center_y_px = result->center_y;
    target->size_px = result->size_px;
    target->confidence = result->confidence;
    target->distance_m = hypotf(x_m, y_m);
    target->target_x_forward_m = x_m;
    target->target_y_left_m = y_m;
    target->too_close =
        (target->distance_m < config->minimum_safe_distance_m) ||
        (result->distance_zone == VISION_TAG_DISTANCE_TOO_CLOSE);
    target->valid = !target->too_close;
    target->fresh = true;
    return true;
}

bool vehicle_guide_target_update(
    VehicleGuideTargetTracker *tracker,
    const vision_runtime_snapshot_t *snapshot,
    uint64_t now_us,
    VehicleGuideTarget *target)
{
    bool new_frame;
    VehicleGuideTarget next;

    if((tracker == nullptr) || (snapshot == nullptr) || (target == nullptr) ||
       !tracker->initialized)
    {
        clear_target(target);
        return false;
    }
    new_frame = snapshot->camera_sequence != tracker->last_camera_sequence;
    if(new_frame)
    {
        tracker->last_camera_sequence = snapshot->camera_sequence;
        next = {};
        if(!target_from_result(&tracker->config, &snapshot->result, now_us,
                               snapshot->camera_sequence, &next))
        {
            next.timestamp_us = now_us;
            next.camera_sequence = snapshot->camera_sequence;
            next.center_x_px = snapshot->result.center_x;
            next.center_y_px = snapshot->result.center_y;
            next.size_px = snapshot->result.size_px;
            next.confidence = snapshot->result.confidence;
            next.valid = false;
            next.fresh = true;
            next.too_close = (snapshot->result.distance_zone ==
                              VISION_TAG_DISTANCE_TOO_CLOSE);
        }
        next.process_us_last = snapshot->process_us_last;
        next.process_us_max = snapshot->process_us_max;
        tracker->target = next;
    }
    else if(tracker->target.timestamp_us != 0U &&
            now_us >= tracker->target.timestamp_us)
    {
        tracker->target.process_us_last = snapshot->process_us_last;
        tracker->target.process_us_max = snapshot->process_us_max;
        tracker->target.age_us = now_us - tracker->target.timestamp_us;
        tracker->target.fresh = tracker->target.age_us <=
                                tracker->config.target_timeout_us;
        if(!tracker->target.fresh)
        {
            tracker->target.valid = false;
        }
    }
    *target = tracker->target;
    return new_frame;
}
