#ifndef VEHICLE_GUIDE_TARGET_H
#define VEHICLE_GUIDE_TARGET_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_types.h"
#include "vision_shared.h"

struct VehicleGuideTargetConfig
{
    uint16_t image_width;
    uint16_t image_height;
    uint16_t min_confidence;
    uint16_t min_size_px;
    float tag_width_m;
    float focal_x_px;
    float focal_y_px;
    float principal_x_px;
    float principal_y_px;
    float camera_position_x_m;
    float camera_position_y_m;
    float camera_yaw_rad;
    float minimum_safe_distance_m;
    uint64_t target_timeout_us;
};

struct VehicleGuideTargetTracker
{
    VehicleGuideTargetConfig config;
    VehicleGuideTarget target;
    uint32_t last_camera_sequence;
    bool initialized;
};

void vehicle_guide_target_default_config(VehicleGuideTargetConfig *config);
bool vehicle_guide_target_config_is_valid(
    const VehicleGuideTargetConfig *config);
void vehicle_guide_target_init(VehicleGuideTargetTracker *tracker,
                               const VehicleGuideTargetConfig *config);
bool vehicle_guide_target_update(
    VehicleGuideTargetTracker *tracker,
    const vision_runtime_snapshot_t *snapshot,
    uint64_t now_us,
    VehicleGuideTarget *target);

#endif
