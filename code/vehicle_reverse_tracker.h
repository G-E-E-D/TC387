#ifndef VEHICLE_REVERSE_TRACKER_H
#define VEHICLE_REVERSE_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_types.h"

typedef enum
{
    VEHICLE_REVERSE_TRACKER_UNINITIALIZED = 0,
    VEHICLE_REVERSE_TRACKER_ACTIVE,
    VEHICLE_REVERSE_TRACKER_INDEX_REACQUIRING,
    VEHICLE_REVERSE_TRACKER_FINISHED,
    VEHICLE_REVERSE_TRACKER_INVALID_ARGUMENT,
    VEHICLE_REVERSE_TRACKER_PATH_INVALID,
    VEHICLE_REVERSE_TRACKER_LOCALIZATION_INVALID,
    VEHICLE_REVERSE_TRACKER_INDEX_LOST,
    VEHICLE_REVERSE_TRACKER_TRACKING_ERROR,
    VEHICLE_REVERSE_TRACKER_NUMERIC_ERROR
} VehicleReverseTrackerStatus;

typedef struct
{
    float wheelbase_m;
    float nominal_reverse_speed_mps;
    float maximum_reverse_speed_mps;
    float lookahead_min_m;
    float lookahead_max_m;
    float lookahead_time_s;
    float curvature_speed_gain;
    float cross_track_speed_gain;
    float cross_track_feedback_gain;
    float heading_feedback_gain;
    float curvature_feedforward_gain;
    float maximum_steering_rad;
    float acceleration_limit_mps2;
    float deceleration_limit_mps2;
    float terminal_deceleration_distance_m;
    float finish_distance_m;
    float finish_speed_mps;
    float maximum_cross_track_error_m;
    float maximum_heading_error_rad;
    uint32_t search_forward_points;
    uint32_t index_loss_limit;
} VehicleReverseTrackerConfig;

typedef struct
{
    float x_m;
    float y_m;
    float body_yaw_rad;
    float speed_mps;
    float dt_s;
    bool localization_valid;
} VehicleReverseTrackerInput;

typedef struct
{
    VehicleReverseTrackerConfig config;
    const PathPoint *path;
    uint32_t path_count;
    uint32_t progress_index;
    uint32_t nearest_index;
    uint32_t target_index;
    uint32_t index_loss_count;
    float previous_target_speed_mps;
    VehicleReverseTrackerStatus status;
    bool initialized;
} VehicleReverseTracker;

void vehicle_reverse_tracker_default_config(
    VehicleReverseTrackerConfig *config);
bool vehicle_reverse_tracker_config_is_valid(
    const VehicleReverseTrackerConfig *config);

VehicleReverseTrackerStatus vehicle_reverse_tracker_init(
    VehicleReverseTracker *tracker,
    const VehicleReverseTrackerConfig *config,
    const PathPoint *path,
    uint32_t path_count);

VehicleReverseTrackerStatus vehicle_reverse_tracker_update(
    VehicleReverseTracker *tracker,
    const VehicleReverseTrackerInput *input,
    ReverseTrackerOutput *output);

void vehicle_reverse_tracker_reset(VehicleReverseTracker *tracker);
VehicleReverseTrackerStatus vehicle_reverse_tracker_get_status(
    const VehicleReverseTracker *tracker);
bool vehicle_reverse_tracker_status_is_error(
    VehicleReverseTrackerStatus status);

#endif
