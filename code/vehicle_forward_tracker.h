#ifndef VEHICLE_FORWARD_TRACKER_H
#define VEHICLE_FORWARD_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_types.h"

enum VehicleForwardTrackerStatus
{
    VEHICLE_FORWARD_TRACKER_UNINITIALIZED = 0,
    VEHICLE_FORWARD_TRACKER_ACTIVE,
    VEHICLE_FORWARD_TRACKER_TARGET_STALE,
    VEHICLE_FORWARD_TRACKER_INVALID_ARGUMENT,
    VEHICLE_FORWARD_TRACKER_NUMERIC_ERROR
};

struct VehicleForwardTrackerConfig
{
    float wheelbase_m;
    float desired_follow_distance_m;
    float minimum_safe_distance_m;
    float lookahead_min_m;
    float lookahead_max_m;
    float maximum_speed_mps;
    float distance_kp;
    float curvature_speed_gain;
    float maximum_steering_rad;
    float acceleration_limit_mps2;
    float deceleration_limit_mps2;
    uint16_t minimum_confidence;
    uint64_t target_timeout_us;
};

struct VehicleForwardTrackerInput
{
    float guide_x_m;
    float guide_y_m;
    float measured_speed_mps;
    float dt_s;
    uint64_t target_timestamp_us;
    uint64_t target_age_us;
    bool target_valid;
    uint16_t target_confidence;
    bool target_is_new;
    bool distance_stable;
    float localization_quality;
    float steering_quality;
};

struct VehicleForwardTracker
{
    VehicleForwardTrackerConfig config;
    float previous_target_speed_mps;
    uint64_t last_target_timestamp_us;
    VehicleForwardTrackerStatus status;
    bool initialized;
};

void vehicle_forward_tracker_default_config(
    VehicleForwardTrackerConfig *config);
bool vehicle_forward_tracker_config_is_valid(
    const VehicleForwardTrackerConfig *config);
VehicleForwardTrackerStatus vehicle_forward_tracker_init(
    VehicleForwardTracker *tracker,
    const VehicleForwardTrackerConfig *config);
VehicleForwardTrackerStatus vehicle_forward_tracker_update(
    VehicleForwardTracker *tracker,
    const VehicleForwardTrackerInput *input,
    VehicleForwardTrackerOutput *output);
void vehicle_forward_tracker_reset(VehicleForwardTracker *tracker);
VehicleForwardTrackerStatus vehicle_forward_tracker_get_status(
    const VehicleForwardTracker *tracker);

#endif
