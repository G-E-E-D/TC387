#include "vehicle_forward_tracker.h"

#include <cmath>

#include "vehicle_config.h"
#include "vehicle_math.h"
#include "vision_tag_tracker.h"

static void clear_output(VehicleForwardTrackerOutput *output)
{
    if(output != nullptr)
    {
        *output = {};
    }
}

void vehicle_forward_tracker_default_config(
    VehicleForwardTrackerConfig *config)
{
    if(config == nullptr)
    {
        return;
    }
    config->wheelbase_m = VEHICLE_WHEELBASE_M;
    config->desired_follow_distance_m = FORWARD_FOLLOW_DISTANCE_M;
    config->minimum_safe_distance_m = GUIDE_TARGET_MIN_DISTANCE_M;
    config->lookahead_min_m = FORWARD_LOOKAHEAD_MIN_M;
    config->lookahead_max_m = FORWARD_LOOKAHEAD_MAX_M;
    config->maximum_speed_mps = FORWARD_MAX_SPEED_MPS;
    config->distance_kp = FORWARD_DISTANCE_KP;
    config->curvature_speed_gain = FORWARD_CURVATURE_SPEED_GAIN;
    config->maximum_steering_rad = FORWARD_MAX_STEERING_RAD;
    config->acceleration_limit_mps2 = FORWARD_ACCEL_LIMIT_MPS2;
    config->deceleration_limit_mps2 = FORWARD_DECEL_LIMIT_MPS2;
    config->minimum_confidence = VEHICLE_VISION_MIN_CONFIDENCE;
    config->target_timeout_us = GUIDE_TARGET_TIMEOUT_US;
}

bool vehicle_forward_tracker_config_is_valid(
    const VehicleForwardTrackerConfig *config)
{
    return (config != nullptr) &&
           std::isfinite(config->wheelbase_m) &&
           std::isfinite(config->desired_follow_distance_m) &&
           std::isfinite(config->minimum_safe_distance_m) &&
           std::isfinite(config->lookahead_min_m) &&
           std::isfinite(config->lookahead_max_m) &&
           std::isfinite(config->maximum_speed_mps) &&
           std::isfinite(config->distance_kp) &&
           std::isfinite(config->curvature_speed_gain) &&
           std::isfinite(config->maximum_steering_rad) &&
           std::isfinite(config->acceleration_limit_mps2) &&
           std::isfinite(config->deceleration_limit_mps2) &&
           (config->wheelbase_m > 0.0f) &&
           (config->desired_follow_distance_m > 0.0f) &&
           (config->minimum_safe_distance_m > 0.0f) &&
           (config->lookahead_min_m > 0.0f) &&
           (config->lookahead_max_m >= config->lookahead_min_m) &&
           (config->maximum_speed_mps > 0.0f) &&
           (config->distance_kp >= 0.0f) &&
           (config->curvature_speed_gain >= 0.0f) &&
           (config->maximum_steering_rad > 0.0f) &&
           (config->acceleration_limit_mps2 > 0.0f) &&
           (config->deceleration_limit_mps2 > 0.0f) &&
           (config->minimum_confidence <= VISION_TAG_SCORE_MAX) &&
           (config->target_timeout_us > 0U);
}

VehicleForwardTrackerStatus vehicle_forward_tracker_init(
    VehicleForwardTracker *tracker,
    const VehicleForwardTrackerConfig *config)
{
    VehicleForwardTrackerConfig selected;
    if(tracker == nullptr)
    {
        return VEHICLE_FORWARD_TRACKER_INVALID_ARGUMENT;
    }
    vehicle_forward_tracker_default_config(&selected);
    if(config != nullptr)
    {
        selected = *config;
    }
    *tracker = {};
    tracker->config = selected;
    if(!vehicle_forward_tracker_config_is_valid(&selected))
    {
        tracker->status = VEHICLE_FORWARD_TRACKER_INVALID_ARGUMENT;
        return tracker->status;
    }
    tracker->status = VEHICLE_FORWARD_TRACKER_ACTIVE;
    tracker->initialized = true;
    return tracker->status;
}

void vehicle_forward_tracker_reset(VehicleForwardTracker *tracker)
{
    if(tracker == nullptr)
    {
        return;
    }
    tracker->previous_target_speed_mps = 0.0f;
    tracker->last_target_timestamp_us = 0U;
    tracker->status = tracker->initialized
        ? VEHICLE_FORWARD_TRACKER_ACTIVE
        : VEHICLE_FORWARD_TRACKER_UNINITIALIZED;
}

static VehicleForwardTrackerStatus safe_stop(
    VehicleForwardTracker *tracker, const VehicleForwardTrackerInput *input,
    VehicleForwardTrackerOutput *output, VehicleForwardTrackerStatus status)
{
    float dt_s = (input != nullptr && std::isfinite(input->dt_s) &&
                  input->dt_s > 0.0f) ? input->dt_s : 0.01f;
    if(tracker != nullptr)
    {
        tracker->previous_target_speed_mps = vehicle_rate_limit(
            0.0f, tracker->previous_target_speed_mps,
            tracker->config.acceleration_limit_mps2,
            tracker->config.deceleration_limit_mps2, dt_s);
        tracker->previous_target_speed_mps = vehicle_clampf(
            tracker->previous_target_speed_mps, 0.0f,
            tracker->config.maximum_speed_mps);
        tracker->status = status;
        if(output != nullptr)
        {
            output->target_speed_mps = tracker->previous_target_speed_mps;
            output->target_age_us = (input != nullptr) ?
                input->target_timestamp_us : 0U;
            output->target_stale = true;
            output->valid = true;
        }
    }
    return status;
}

VehicleForwardTrackerStatus vehicle_forward_tracker_update(
    VehicleForwardTracker *tracker,
    const VehicleForwardTrackerInput *input,
    VehicleForwardTrackerOutput *output)
{
    uint64_t age_us;
    float aim_x_m;
    float aim_y_m;
    float lookahead_squared;
    float curvature;
    float steering;
    float distance_error;
    float desired_speed;
    float quality;
    float dt_s;
    bool stale;

    clear_output(output);
    if((tracker == nullptr) || (input == nullptr) || (output == nullptr))
    {
        return VEHICLE_FORWARD_TRACKER_INVALID_ARGUMENT;
    }
    if(!tracker->initialized ||
       !vehicle_forward_tracker_config_is_valid(&tracker->config))
    {
        return safe_stop(tracker, input, output,
                         VEHICLE_FORWARD_TRACKER_UNINITIALIZED);
    }
    if(!std::isfinite(input->guide_x_m) || !std::isfinite(input->guide_y_m) ||
       !std::isfinite(input->measured_speed_mps) ||
       !std::isfinite(input->dt_s) || !(input->dt_s > 0.0f) ||
       !std::isfinite(input->localization_quality) ||
       !std::isfinite(input->steering_quality))
    {
        return safe_stop(tracker, input, output,
                         VEHICLE_FORWARD_TRACKER_NUMERIC_ERROR);
    }

    age_us = input->target_age_us;
    if(input->target_timestamp_us == 0U)
    {
        age_us = tracker->config.target_timeout_us + 1U;
    }
    if(input->target_timestamp_us != 0U &&
       tracker->last_target_timestamp_us != 0U &&
       input->target_timestamp_us < tracker->last_target_timestamp_us)
    {
        return safe_stop(tracker, input, output,
                         VEHICLE_FORWARD_TRACKER_NUMERIC_ERROR);
    }
    if(input->target_timestamp_us != 0U)
    {
        tracker->last_target_timestamp_us = input->target_timestamp_us;
    }
    stale = !input->target_valid ||
            (input->target_timestamp_us == 0U) ||
            (age_us > tracker->config.target_timeout_us) ||
            (input->target_confidence < tracker->config.minimum_confidence);
    if(stale)
    {
        return safe_stop(tracker, input, output,
                         VEHICLE_FORWARD_TRACKER_TARGET_STALE);
    }

    aim_x_m = fmaxf(input->guide_x_m -
                    tracker->config.desired_follow_distance_m,
                    tracker->config.lookahead_min_m);
    aim_y_m = input->guide_y_m;
    lookahead_squared = aim_x_m * aim_x_m + aim_y_m * aim_y_m;
    if(!(lookahead_squared > 1.0e-6f) || !std::isfinite(lookahead_squared))
    {
        return safe_stop(tracker, input, output,
                         VEHICLE_FORWARD_TRACKER_NUMERIC_ERROR);
    }
    curvature = 2.0f * aim_y_m / lookahead_squared;
    steering = atanf(tracker->config.wheelbase_m * curvature);
    distance_error = input->guide_x_m -
                     tracker->config.desired_follow_distance_m;
    desired_speed = tracker->config.distance_kp * fmaxf(distance_error, 0.0f);
    if(input->guide_x_m <= tracker->config.desired_follow_distance_m ||
       input->guide_x_m < tracker->config.minimum_safe_distance_m)
    {
        desired_speed = 0.0f;
    }
    quality = vehicle_clampf(input->localization_quality, 0.0f, 1.0f) *
              vehicle_clampf(input->steering_quality, 0.0f, 1.0f);
    quality *= vehicle_clampf(
        static_cast<float>(input->target_confidence) /
        static_cast<float>(VISION_TAG_SCORE_MAX), 0.0f, 1.0f);
    if(!input->distance_stable)
    {
        quality *= 0.55f;
    }
    desired_speed *= quality;
    desired_speed /= (1.0f + tracker->config.curvature_speed_gain *
                      fabsf(curvature));
    desired_speed = vehicle_clampf(desired_speed, 0.0f,
                                   tracker->config.maximum_speed_mps);
    /* A repeated camera sequence may not create a new acceleration command. */
    if(!input->target_is_new &&
       (desired_speed > tracker->previous_target_speed_mps))
    {
        desired_speed = tracker->previous_target_speed_mps;
    }
    dt_s = vehicle_clampf(input->dt_s, 0.0005f, 0.050f);
    tracker->previous_target_speed_mps = vehicle_rate_limit(
        desired_speed, tracker->previous_target_speed_mps,
        tracker->config.acceleration_limit_mps2,
        tracker->config.deceleration_limit_mps2, dt_s);
    tracker->previous_target_speed_mps = vehicle_clampf(
        tracker->previous_target_speed_mps, 0.0f,
        tracker->config.maximum_speed_mps);
    output->aim_x_m = aim_x_m;
    output->aim_y_m = aim_y_m;
    output->curvature_per_m = curvature;
    output->target_speed_mps = fmaxf(tracker->previous_target_speed_mps, 0.0f);
    output->target_steering_rad = vehicle_clampf(
        steering, -tracker->config.maximum_steering_rad,
        tracker->config.maximum_steering_rad);
    output->distance_error_m = distance_error;
    output->target_age_us = age_us;
    output->target_stale = false;
    output->valid = true;
    tracker->status = VEHICLE_FORWARD_TRACKER_ACTIVE;
    return tracker->status;
}

VehicleForwardTrackerStatus vehicle_forward_tracker_get_status(
    const VehicleForwardTracker *tracker)
{
    return (tracker != nullptr) ? tracker->status
        : VEHICLE_FORWARD_TRACKER_INVALID_ARGUMENT;
}
