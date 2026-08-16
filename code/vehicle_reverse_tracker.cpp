#include "vehicle_reverse_tracker.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "vehicle_config.h"
#include "vehicle_math.h"
#include "vehicle_path.h"

static bool vehicle_reverse_tracker_input_is_finite(
    const VehicleReverseTrackerInput *input)
{
    return (input != nullptr) && vehicle_float_is_finite(input->x_m) &&
           vehicle_float_is_finite(input->y_m) &&
           vehicle_float_is_finite(input->body_yaw_rad) &&
           vehicle_float_is_finite(input->speed_mps) &&
           vehicle_float_is_finite(input->dt_s) && (input->dt_s > 0.0f);
}

static void vehicle_reverse_tracker_clear_output(
    const VehicleReverseTracker *tracker, ReverseTrackerOutput *output)
{
    output->target_speed_mps = 0.0f;
    output->target_steering_rad = 0.0f;
    output->cross_track_error_m = 0.0f;
    output->heading_error_rad = 0.0f;
    output->lookahead_m = 0.0f;
    output->nearest_index = (tracker != nullptr) ? tracker->nearest_index : 0U;
    output->target_index = (tracker != nullptr) ? tracker->target_index : 0U;
    output->finished = false;
    output->valid = false;
}

static VehicleReverseTrackerStatus vehicle_reverse_tracker_safe_output(
    VehicleReverseTracker *tracker,
    const VehicleReverseTrackerInput *input,
    ReverseTrackerOutput *output,
    VehicleReverseTrackerStatus status)
{
    vehicle_reverse_tracker_clear_output(tracker, output);
    if(tracker != nullptr && input != nullptr &&
       vehicle_float_is_finite(input->dt_s) && (input->dt_s > 0.0f))
    {
        tracker->previous_target_speed_mps = vehicle_rate_limit(
            0.0f, tracker->previous_target_speed_mps,
            tracker->config.acceleration_limit_mps2,
            tracker->config.deceleration_limit_mps2, input->dt_s);
        tracker->previous_target_speed_mps = vehicle_clampf(
            tracker->previous_target_speed_mps,
            -tracker->config.maximum_reverse_speed_mps, 0.0f);
        output->target_speed_mps = tracker->previous_target_speed_mps;
    }
    if(tracker != nullptr)
    {
        tracker->status = status;
        output->nearest_index = tracker->nearest_index;
        output->target_index = tracker->target_index;
    }
    return status;
}

void vehicle_reverse_tracker_default_config(
    VehicleReverseTrackerConfig *config)
{
    if(config == nullptr)
    {
        return;
    }
    config->wheelbase_m = VEHICLE_WHEELBASE_M;
    config->nominal_reverse_speed_mps = REVERSE_NOMINAL_SPEED_MPS;
    config->maximum_reverse_speed_mps = REPLAY_MAX_SPEED_MPS;
    config->lookahead_min_m = REVERSE_LOOKAHEAD_MIN_M;
    config->lookahead_max_m = REVERSE_LOOKAHEAD_MAX_M;
    config->lookahead_time_s = REVERSE_LOOKAHEAD_TIME_S;
    config->curvature_speed_gain = REVERSE_CURVATURE_SPEED_GAIN;
    config->cross_track_speed_gain = REVERSE_CROSSTRACK_SPEED_GAIN;
    config->cross_track_feedback_gain = REVERSE_CROSSTRACK_GAIN;
    config->heading_feedback_gain = REVERSE_HEADING_GAIN;
    config->curvature_feedforward_gain =
        REVERSE_CURVATURE_FEEDFORWARD_GAIN;
    config->maximum_steering_rad = REVERSE_MAX_STEERING_RAD;
    config->acceleration_limit_mps2 = REVERSE_ACCEL_LIMIT_MPS2;
    config->deceleration_limit_mps2 = REVERSE_DECEL_LIMIT_MPS2;
    config->terminal_deceleration_distance_m =
        REVERSE_TERMINAL_DECEL_DISTANCE_M;
    config->finish_distance_m = REVERSE_FINISH_DISTANCE_M;
    config->finish_speed_mps = REVERSE_FINISH_SPEED_MPS;
    config->maximum_cross_track_error_m =
        REVERSE_MAX_CROSSTRACK_ERROR_M;
    config->maximum_heading_error_rad =
        REVERSE_MAX_HEADING_ERROR_RAD;
    config->search_forward_points = REVERSE_SEARCH_FORWARD_POINTS;
    config->index_loss_limit = REVERSE_INDEX_LOSS_LIMIT;
}

bool vehicle_reverse_tracker_config_is_valid(
    const VehicleReverseTrackerConfig *config)
{
    if(config == nullptr)
    {
        return false;
    }
    return vehicle_float_is_finite(config->wheelbase_m) &&
           vehicle_float_is_finite(config->nominal_reverse_speed_mps) &&
           vehicle_float_is_finite(config->maximum_reverse_speed_mps) &&
           vehicle_float_is_finite(config->lookahead_min_m) &&
           vehicle_float_is_finite(config->lookahead_max_m) &&
           vehicle_float_is_finite(config->lookahead_time_s) &&
           vehicle_float_is_finite(config->curvature_speed_gain) &&
           vehicle_float_is_finite(config->cross_track_speed_gain) &&
           vehicle_float_is_finite(config->cross_track_feedback_gain) &&
           vehicle_float_is_finite(config->heading_feedback_gain) &&
           vehicle_float_is_finite(config->curvature_feedforward_gain) &&
           vehicle_float_is_finite(config->maximum_steering_rad) &&
           vehicle_float_is_finite(config->acceleration_limit_mps2) &&
           vehicle_float_is_finite(config->deceleration_limit_mps2) &&
           vehicle_float_is_finite(
               config->terminal_deceleration_distance_m) &&
           vehicle_float_is_finite(config->finish_distance_m) &&
           vehicle_float_is_finite(config->finish_speed_mps) &&
           vehicle_float_is_finite(config->maximum_cross_track_error_m) &&
           vehicle_float_is_finite(config->maximum_heading_error_rad) &&
           (config->wheelbase_m > 0.0f) &&
           (config->nominal_reverse_speed_mps < 0.0f) &&
           (config->maximum_reverse_speed_mps > 0.0f) &&
           (fabsf(config->nominal_reverse_speed_mps) <=
            config->maximum_reverse_speed_mps) &&
           (config->lookahead_min_m > 0.0f) &&
           (config->lookahead_max_m >= config->lookahead_min_m) &&
           (config->lookahead_time_s >= 0.0f) &&
           (config->curvature_speed_gain >= 0.0f) &&
           (config->cross_track_speed_gain >= 0.0f) &&
           (config->cross_track_feedback_gain >= 0.0f) &&
           (config->heading_feedback_gain >= 0.0f) &&
           (config->curvature_feedforward_gain >= 0.0f) &&
           (config->maximum_steering_rad > 0.0f) &&
           (config->acceleration_limit_mps2 > 0.0f) &&
           (config->deceleration_limit_mps2 > 0.0f) &&
           (config->terminal_deceleration_distance_m > 0.0f) &&
           (config->finish_distance_m > 0.0f) &&
           (config->finish_speed_mps >= 0.0f) &&
           (config->maximum_cross_track_error_m > 0.0f) &&
           (config->maximum_heading_error_rad > 0.0f) &&
           (config->search_forward_points > 0U) &&
           (config->index_loss_limit > 0U);
}

void vehicle_reverse_tracker_reset(VehicleReverseTracker *tracker)
{
    if(tracker == nullptr)
    {
        return;
    }
    *tracker = {};
    tracker->status = VEHICLE_REVERSE_TRACKER_UNINITIALIZED;
}

VehicleReverseTrackerStatus vehicle_reverse_tracker_init(
    VehicleReverseTracker *tracker,
    const VehicleReverseTrackerConfig *config,
    const PathPoint *path,
    uint32_t path_count)
{
    VehicleReverseTrackerConfig selected_config;
    VehiclePathResult path_result;

    if(tracker == nullptr)
    {
        return VEHICLE_REVERSE_TRACKER_INVALID_ARGUMENT;
    }
    vehicle_reverse_tracker_reset(tracker);

    if(config == nullptr)
    {
        vehicle_reverse_tracker_default_config(&selected_config);
    }
    else
    {
        selected_config = *config;
    }
    tracker->config = selected_config;

    if(!vehicle_reverse_tracker_config_is_valid(&selected_config) ||
       path == nullptr)
    {
        tracker->status = VEHICLE_REVERSE_TRACKER_INVALID_ARGUMENT;
        return tracker->status;
    }
    path_result = vehicle_path_validate_points(path, path_count, nullptr, nullptr);
    if(path_result != VEHICLE_PATH_RESULT_OK)
    {
        tracker->status = VEHICLE_REVERSE_TRACKER_PATH_INVALID;
        return tracker->status;
    }

    tracker->path = path;
    tracker->path_count = path_count;
    tracker->progress_index = path_count - 1U;
    tracker->nearest_index = tracker->progress_index;
    tracker->target_index = tracker->progress_index;
    tracker->index_loss_count = 0U;
    tracker->previous_target_speed_mps = 0.0f;
    tracker->status = VEHICLE_REVERSE_TRACKER_ACTIVE;
    tracker->initialized = true;
    return tracker->status;
}

VehicleReverseTrackerStatus vehicle_reverse_tracker_update(
    VehicleReverseTracker *tracker,
    const VehicleReverseTrackerInput *input,
    ReverseTrackerOutput *output)
{
    uint32_t search_lower;
    uint32_t search_index;
    uint32_t best_index;
    uint32_t target_index;
    float best_distance_squared;
    float best_distance_m;
    float travel_yaw_rad;
    float reference_travel_yaw_rad;
    float cross_track_error_m;
    float heading_error_rad;
    float lookahead_m;
    float target_delta_x;
    float target_delta_y;
    float target_distance_m;
    float target_bearing_rad;
    float target_alpha_rad;
    float pure_pursuit_curvature;
    float reference_delta_x;
    float reference_delta_y;
    float reference_target_distance_m;
    float reference_pure_pursuit_curvature = 0.0f;
    float forward_curvature;
    float travel_curvature_command;
    float steering_command_rad;
    float speed_magnitude;
    float curvature_for_speed;
    float remaining_distance_m;
    float distance_to_finish_m;
    float desired_speed_mps;
    bool finish_progress_reached;

    if(output == nullptr)
    {
        return VEHICLE_REVERSE_TRACKER_INVALID_ARGUMENT;
    }
    vehicle_reverse_tracker_clear_output(tracker, output);
    if(tracker == nullptr || input == nullptr)
    {
        return VEHICLE_REVERSE_TRACKER_INVALID_ARGUMENT;
    }
    if(!tracker->initialized || tracker->path == nullptr ||
       tracker->path_count == 0U)
    {
        return vehicle_reverse_tracker_safe_output(
            tracker, input, output, VEHICLE_REVERSE_TRACKER_UNINITIALIZED);
    }
    if(!vehicle_reverse_tracker_input_is_finite(input))
    {
        return vehicle_reverse_tracker_safe_output(
            tracker, input, output, VEHICLE_REVERSE_TRACKER_NUMERIC_ERROR);
    }
    if(!input->localization_valid)
    {
        return vehicle_reverse_tracker_safe_output(
            tracker, input, output,
            VEHICLE_REVERSE_TRACKER_LOCALIZATION_INVALID);
    }

    if(tracker->status == VEHICLE_REVERSE_TRACKER_FINISHED)
    {
        tracker->previous_target_speed_mps = 0.0f;
        output->finished = true;
        output->valid = true;
        output->nearest_index = tracker->nearest_index;
        output->target_index = tracker->target_index;
        return tracker->status;
    }
    if(tracker->status != VEHICLE_REVERSE_TRACKER_ACTIVE &&
       tracker->status != VEHICLE_REVERSE_TRACKER_INDEX_REACQUIRING)
    {
        return vehicle_reverse_tracker_safe_output(
            tracker, input, output, tracker->status);
    }

    search_lower =
        (tracker->progress_index > tracker->config.search_forward_points)
            ? tracker->progress_index -
                  tracker->config.search_forward_points
            : 0U;
    best_index = tracker->progress_index;
    best_distance_squared = INFINITY;
    search_index = tracker->progress_index;
    for(;;)
    {
        float delta_x = input->x_m - tracker->path[search_index].x;
        float delta_y = input->y_m - tracker->path[search_index].y;
        float distance_squared = delta_x * delta_x + delta_y * delta_y;

        if(vehicle_float_is_finite(distance_squared) &&
           distance_squared < best_distance_squared)
        {
            best_distance_squared = distance_squared;
            best_index = search_index;
        }
        if(search_index == search_lower)
        {
            break;
        }
        --search_index;
    }
    best_distance_m = sqrtf(best_distance_squared);
    if(!vehicle_float_is_finite(best_distance_m) ||
       best_distance_m > tracker->config.maximum_cross_track_error_m)
    {
        if(tracker->index_loss_count < UINT32_MAX)
        {
            ++tracker->index_loss_count;
        }
        if(tracker->index_loss_count >= tracker->config.index_loss_limit)
        {
            return vehicle_reverse_tracker_safe_output(
                tracker, input, output,
                VEHICLE_REVERSE_TRACKER_INDEX_LOST);
        }
        return vehicle_reverse_tracker_safe_output(
            tracker, input, output,
            VEHICLE_REVERSE_TRACKER_INDEX_REACQUIRING);
    }

    tracker->index_loss_count = 0U;
    tracker->status = VEHICLE_REVERSE_TRACKER_ACTIVE;
    if(best_index < tracker->progress_index)
    {
        tracker->progress_index = best_index;
    }
    tracker->nearest_index = tracker->progress_index;

    lookahead_m = tracker->config.lookahead_min_m +
                  tracker->config.lookahead_time_s * fabsf(input->speed_mps);
    lookahead_m = vehicle_clampf(lookahead_m,
                                tracker->config.lookahead_min_m,
                                tracker->config.lookahead_max_m);
    target_index = tracker->nearest_index;
    while(target_index > 0U &&
          (tracker->path[tracker->nearest_index].s -
           tracker->path[target_index].s) < lookahead_m)
    {
        --target_index;
    }
    tracker->target_index = target_index;

    distance_to_finish_m = hypotf(input->x_m - tracker->path[0].x,
                                  input->y_m - tracker->path[0].y);
    remaining_distance_m = tracker->path[tracker->nearest_index].s -
                           tracker->path[0].s;
    if(remaining_distance_m < 0.0f)
    {
        remaining_distance_m = 0.0f;
    }
    finish_progress_reached = remaining_distance_m <=
                              tracker->config.finish_distance_m;
    if(finish_progress_reached)
    {
        tracker->previous_target_speed_mps = vehicle_rate_limit(
            0.0f, tracker->previous_target_speed_mps,
            tracker->config.acceleration_limit_mps2,
            tracker->config.deceleration_limit_mps2, input->dt_s);
        tracker->previous_target_speed_mps = vehicle_clampf(
            tracker->previous_target_speed_mps,
            -tracker->config.maximum_reverse_speed_mps, 0.0f);
        output->target_speed_mps = tracker->previous_target_speed_mps;
        output->target_steering_rad = 0.0f;
        output->lookahead_m = lookahead_m;
        output->nearest_index = tracker->nearest_index;
        output->target_index = tracker->target_index;
        output->valid = true;
        if(fabsf(input->speed_mps) <= tracker->config.finish_speed_mps &&
           fabsf(tracker->previous_target_speed_mps) <=
               tracker->config.finish_speed_mps)
        {
            if(distance_to_finish_m > tracker->config.finish_distance_m)
            {
                return vehicle_reverse_tracker_safe_output(
                    tracker, input, output,
                    VEHICLE_REVERSE_TRACKER_TRACKING_ERROR);
            }
            tracker->previous_target_speed_mps = 0.0f;
            tracker->status = VEHICLE_REVERSE_TRACKER_FINISHED;
            output->target_speed_mps = 0.0f;
            output->finished = true;
        }
        return tracker->status;
    }

    travel_yaw_rad = vehicle_wrap_pi(input->body_yaw_rad + VEHICLE_PI_F);
    reference_travel_yaw_rad = vehicle_wrap_pi(
        tracker->path[tracker->nearest_index].yaw_body + VEHICLE_PI_F);
    cross_track_error_m =
        -sinf(reference_travel_yaw_rad) *
            (input->x_m - tracker->path[tracker->nearest_index].x) +
        cosf(reference_travel_yaw_rad) *
            (input->y_m - tracker->path[tracker->nearest_index].y);
    heading_error_rad = vehicle_angle_difference(
        reference_travel_yaw_rad, travel_yaw_rad);
    if(!vehicle_float_is_finite(cross_track_error_m) ||
       !vehicle_float_is_finite(heading_error_rad))
    {
        return vehicle_reverse_tracker_safe_output(
            tracker, input, output, VEHICLE_REVERSE_TRACKER_NUMERIC_ERROR);
    }
    if(fabsf(cross_track_error_m) >
           tracker->config.maximum_cross_track_error_m ||
       fabsf(heading_error_rad) >
           tracker->config.maximum_heading_error_rad)
    {
        output->cross_track_error_m = cross_track_error_m;
        output->heading_error_rad = heading_error_rad;
        return vehicle_reverse_tracker_safe_output(
            tracker, input, output,
            VEHICLE_REVERSE_TRACKER_TRACKING_ERROR);
    }

    target_delta_x = tracker->path[target_index].x - input->x_m;
    target_delta_y = tracker->path[target_index].y - input->y_m;
    target_distance_m = hypotf(target_delta_x, target_delta_y);
    if(!(target_distance_m > PATH_MIN_POINT_DISTANCE_M) ||
       !vehicle_float_is_finite(target_distance_m))
    {
        return vehicle_reverse_tracker_safe_output(
            tracker, input, output, VEHICLE_REVERSE_TRACKER_NUMERIC_ERROR);
    }
    target_bearing_rad = atan2f(target_delta_y, target_delta_x);
    target_alpha_rad = vehicle_angle_difference(target_bearing_rad,
                                                travel_yaw_rad);
    pure_pursuit_curvature =
        2.0f * sinf(target_alpha_rad) / target_distance_m;

    reference_delta_x = tracker->path[target_index].x -
                        tracker->path[tracker->nearest_index].x;
    reference_delta_y = tracker->path[target_index].y -
                        tracker->path[tracker->nearest_index].y;
    reference_target_distance_m = hypotf(reference_delta_x,
                                         reference_delta_y);
    if(reference_target_distance_m > PATH_MIN_POINT_DISTANCE_M)
    {
        float reference_bearing_rad = atan2f(reference_delta_y,
                                             reference_delta_x);
        float reference_alpha_rad = vehicle_angle_difference(
            reference_bearing_rad, reference_travel_yaw_rad);

        reference_pure_pursuit_curvature =
            2.0f * sinf(reference_alpha_rad) /
            reference_target_distance_m;
    }

    forward_curvature =
        0.5f * (tracker->path[tracker->nearest_index].curvature_forward +
                tracker->path[target_index].curvature_forward);
    travel_curvature_command =
        -tracker->config.curvature_feedforward_gain * forward_curvature +
        (pure_pursuit_curvature - reference_pure_pursuit_curvature) -
        tracker->config.cross_track_feedback_gain * cross_track_error_m /
            (lookahead_m * lookahead_m) +
        tracker->config.heading_feedback_gain * heading_error_rad /
            lookahead_m;
    if(!vehicle_float_is_finite(travel_curvature_command))
    {
        return vehicle_reverse_tracker_safe_output(
            tracker, input, output, VEHICLE_REVERSE_TRACKER_NUMERIC_ERROR);
    }

    steering_command_rad = -atanf(tracker->config.wheelbase_m *
                                  travel_curvature_command);
    steering_command_rad = vehicle_clampf(
        steering_command_rad, -tracker->config.maximum_steering_rad,
        tracker->config.maximum_steering_rad);

    speed_magnitude = fabsf(tracker->config.nominal_reverse_speed_mps);
    curvature_for_speed = fmaxf(fabsf(forward_curvature),
                                fabsf(travel_curvature_command));
    speed_magnitude /=
        1.0f + tracker->config.curvature_speed_gain * curvature_for_speed;
    speed_magnitude /=
        1.0f + tracker->config.cross_track_speed_gain *
                   fabsf(cross_track_error_m);
    speed_magnitude = fminf(speed_magnitude,
                           tracker->config.maximum_reverse_speed_mps);

    if(remaining_distance_m <
       tracker->config.terminal_deceleration_distance_m)
    {
        float terminal_ratio = vehicle_clampf(
            remaining_distance_m /
                tracker->config.terminal_deceleration_distance_m,
            0.0f, 1.0f);
        float terminal_speed_cap =
            fabsf(tracker->config.nominal_reverse_speed_mps) *
            sqrtf(terminal_ratio);

        speed_magnitude = fminf(speed_magnitude, terminal_speed_cap);
    }

    desired_speed_mps = -speed_magnitude;
    tracker->previous_target_speed_mps = vehicle_rate_limit(
        desired_speed_mps, tracker->previous_target_speed_mps,
        tracker->config.acceleration_limit_mps2,
        tracker->config.deceleration_limit_mps2, input->dt_s);
    tracker->previous_target_speed_mps = vehicle_clampf(
        tracker->previous_target_speed_mps,
        -tracker->config.maximum_reverse_speed_mps, 0.0f);

    output->target_speed_mps = tracker->previous_target_speed_mps;
    output->target_steering_rad = steering_command_rad;
    output->cross_track_error_m = cross_track_error_m;
    output->heading_error_rad = heading_error_rad;
    output->lookahead_m = lookahead_m;
    output->nearest_index = tracker->nearest_index;
    output->target_index = tracker->target_index;
    output->finished = false;
    output->valid = true;
    return tracker->status;
}

VehicleReverseTrackerStatus vehicle_reverse_tracker_get_status(
    const VehicleReverseTracker *tracker)
{
    return (tracker != nullptr) ? tracker->status
                             : VEHICLE_REVERSE_TRACKER_INVALID_ARGUMENT;
}

bool vehicle_reverse_tracker_status_is_error(
    VehicleReverseTrackerStatus status)
{
    return status != VEHICLE_REVERSE_TRACKER_ACTIVE &&
           status != VEHICLE_REVERSE_TRACKER_INDEX_REACQUIRING &&
           status != VEHICLE_REVERSE_TRACKER_FINISHED;
}
