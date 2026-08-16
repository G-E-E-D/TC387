#include "vehicle_control.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#include "vehicle_config.h"
#include "vehicle_math.h"

constexpr auto VEHICLE_CONTROL_OUTPUT_EPSILON = 0.000001f;

static int8_t sign_with_deadband(float value, float deadband)
{
    if(value > deadband)
    {
        return 1;
    }
    if(value < -deadband)
    {
        return -1;
    }
    return 0;
}

static bool wheel_config_is_valid(const VehicleWheelControllerConfig *config)
{
    return (config != nullptr) && isfinite(config->kp) &&
           isfinite(config->ki) && isfinite(config->feedforward) &&
           isfinite(config->integral_limit) &&
           isfinite(config->output_limit) &&
           isfinite(config->output_slew_per_s) &&
           isfinite(config->zero_band_mps) &&
           isfinite(config->direction_hold_s) &&
           (config->kp >= 0.0f) && (config->ki >= 0.0f) &&
           (config->feedforward >= 0.0f) &&
           (config->integral_limit >= 0.0f) &&
           (config->output_limit > 0.0f) &&
           (config->output_limit <= 1.0f) &&
           (config->output_slew_per_s > 0.0f) &&
           (config->zero_band_mps >= 0.0f) &&
           (config->direction_hold_s >= 0.0f);
}

static bool steering_config_is_valid(
    const VehicleSteeringControllerConfig *config)
{
    return (config != nullptr) && isfinite(config->kp) &&
           isfinite(config->ki) && isfinite(config->kd) &&
           isfinite(config->integral_limit) &&
           isfinite(config->output_limit) &&
           isfinite(config->output_slew_per_s) &&
           isfinite(config->position_deadband_rad) &&
           isfinite(config->stall_duty) &&
           isfinite(config->stall_timeout_s) &&
           (config->kp >= 0.0f) && (config->ki >= 0.0f) &&
           (config->kd >= 0.0f) &&
           (config->integral_limit >= 0.0f) &&
           (config->output_limit > 0.0f) &&
           (config->output_limit <= 1.0f) &&
           (config->output_slew_per_s > 0.0f) &&
           (config->position_deadband_rad >= 0.0f) &&
           (config->stall_duty >= 0.0f) &&
           (config->stall_duty <= config->output_limit) &&
           (config->stall_count_delta >= 0) &&
           (config->stall_timeout_s > 0.0f) &&
           (config->approach_switch_hysteresis_count >= 0);
}

static int steering_table_angle_direction(
    const SteeringCalibrationPoint *points,
    size_t count)
{
    size_t index;
    int direction;

    if((points == nullptr) || (count < 2U))
    {
        return 0;
    }
    if(!isfinite(points[0].equivalent_steering_angle_rad))
    {
        return 0;
    }
    direction = 0;
    for(index = 1U; index < count; ++index)
    {
        float delta;

        if((points[index].continuous_count <=
            points[index - 1U].continuous_count) ||
           !isfinite(points[index].equivalent_steering_angle_rad))
        {
            return 0;
        }
        delta = points[index].equivalent_steering_angle_rad -
                points[index - 1U].equivalent_steering_angle_rad;
        if(delta == 0.0f)
        {
            return 0;
        }
        if(direction == 0)
        {
            direction = (delta > 0.0f) ? 1 : -1;
        }
        else if(((delta > 0.0f) ? 1 : -1) != direction)
        {
            return 0;
        }
    }
    return direction;
}

static bool steering_tables_are_usable(const SteeringCalibrationTables *tables)
{
    int left_direction;
    int right_direction;

    if(tables == nullptr)
    {
        return false;
    }
    left_direction = steering_table_angle_direction(tables->from_left,
                                                     tables->from_left_count);
    right_direction = steering_table_angle_direction(tables->from_right,
                                                      tables->from_right_count);
    return (left_direction != 0) && (left_direction == right_direction);
}

static const SteeringCalibrationPoint *steering_points_for_approach(
    const SteeringCalibrationTables *tables,
    VehicleSteeringApproach approach,
    size_t *count)
{
    if((tables == nullptr) || (count == nullptr))
    {
        return nullptr;
    }
    if(approach == VEHICLE_STEERING_APPROACH_FROM_RIGHT)
    {
        *count = tables->from_right_count;
        return tables->from_right;
    }
    *count = tables->from_left_count;
    return tables->from_left;
}

static int64_t rounded_i64(double value)
{
    if(value >= static_cast<double>(INT64_MAX))
    {
        return INT64_MAX;
    }
    if(value <= static_cast<double>(INT64_MIN))
    {
        return INT64_MIN;
    }
    if(value >= 0.0)
    {
        return static_cast<int64_t>(value + 0.5);
    }
    return static_cast<int64_t>(value - 0.5);
}

static void drive_output_zero(VehicleDriveControlOutput *output)
{
    if(output != nullptr)
    {
        output->left_target_speed_mps = 0.0f;
        output->right_target_speed_mps = 0.0f;
        output->left_signed_duty = 0.0f;
        output->right_signed_duty = 0.0f;
        output->valid = false;
    }
}

static void steering_output_zero(VehicleSteeringControlOutput *output)
{
    if(output != nullptr)
    {
        output->measured_angle_rad = 0.0f;
        output->target_count = 0;
        output->signed_duty = 0.0f;
        output->approach = VEHICLE_STEERING_APPROACH_UNKNOWN;
        output->at_soft_limit = false;
        output->stalled = false;
        output->valid = false;
    }
}

void vehicle_wheel_controller_default_config(VehicleWheelControllerConfig *config)
{
    if(config != nullptr)
    {
        config->kp = WHEEL_SPEED_KP;
        config->ki = WHEEL_SPEED_KI;
        config->feedforward = WHEEL_SPEED_FEEDFORWARD;
        config->integral_limit = WHEEL_SPEED_INTEGRAL_LIMIT;
        config->output_limit = WHEEL_SPEED_OUTPUT_LIMIT;
        config->output_slew_per_s = WHEEL_SPEED_OUTPUT_SLEW_PER_S;
        config->zero_band_mps = WHEEL_SPEED_ZERO_BAND_MPS;
        config->direction_hold_s = WHEEL_SPEED_DIRECTION_HOLD_S;
    }
}

void vehicle_wheel_speed_controller_init(VehicleWheelSpeedController *controller,
                                         const VehicleWheelControllerConfig *config,
                                         float start_duty_forward,
                                         float start_duty_reverse,
                                         bool calibration_valid)
{
    VehicleWheelControllerConfig default_config;
    const VehicleWheelControllerConfig *selected_config;

    if(controller == nullptr)
    {
        return;
    }
    vehicle_wheel_controller_default_config(&default_config);
    selected_config = (config != nullptr) ? config : &default_config;
    controller->config = *selected_config;
    controller->start_duty_forward = start_duty_forward;
    controller->start_duty_reverse = start_duty_reverse;
    controller->calibration_valid = calibration_valid && CALIBRATION_VALID &&
        wheel_config_is_valid(selected_config) &&
        isfinite(start_duty_forward) && isfinite(start_duty_reverse) &&
        (start_duty_forward >= 0.0f) && (start_duty_reverse >= 0.0f) &&
        (start_duty_forward <= selected_config->output_limit) &&
        (start_duty_reverse <= selected_config->output_limit);
    vehicle_wheel_speed_controller_reset(controller);
}

void vehicle_wheel_speed_controller_reset(VehicleWheelSpeedController *controller)
{
    if(controller != nullptr)
    {
        controller->integral = 0.0f;
        controller->output = 0.0f;
        controller->zero_dwell_s = 0.0f;
        controller->active_direction = 0;
    }
}

float vehicle_wheel_speed_controller_update(VehicleWheelSpeedController *controller,
                                            float target_speed_mps,
                                            float measured_speed_mps,
                                            float dt_s)
{
    int8_t requested_direction;
    float error;
    float feedforward;
    float proposed_integral;
    float proposed_output;
    float desired_output;

    if(controller == nullptr)
    {
        return 0.0f;
    }
    if(!controller->calibration_valid ||
       !wheel_config_is_valid(&controller->config) ||
       !isfinite(target_speed_mps) || !isfinite(measured_speed_mps) ||
       !isfinite(dt_s) || !(dt_s > 0.0f))
    {
        vehicle_wheel_speed_controller_reset(controller);
        return 0.0f;
    }

    requested_direction = sign_with_deadband(target_speed_mps,
                                               controller->config.zero_band_mps);
    if(requested_direction == 0)
    {
        controller->integral = 0.0f;
        controller->output = vehicle_rate_limit(
            0.0f, controller->output,
            controller->config.output_slew_per_s,
            controller->config.output_slew_per_s, dt_s);
        if(fabsf(controller->output) <= VEHICLE_CONTROL_OUTPUT_EPSILON)
        {
            controller->output = 0.0f;
            if(fabsf(measured_speed_mps) <= controller->config.zero_band_mps)
            {
                controller->zero_dwell_s += dt_s;
                controller->zero_dwell_s = vehicle_clampf(
                    controller->zero_dwell_s, 0.0f,
                    controller->config.direction_hold_s);
            }
            else
            {
                controller->zero_dwell_s = 0.0f;
            }
        }
        return controller->output;
    }

    if(controller->active_direction == 0)
    {
        controller->active_direction = requested_direction;
        controller->zero_dwell_s = 0.0f;
    }
    else if(controller->active_direction != requested_direction)
    {
        controller->integral = 0.0f;
        controller->output = vehicle_rate_limit(
            0.0f, controller->output,
            controller->config.output_slew_per_s,
            controller->config.output_slew_per_s, dt_s);
        if((fabsf(controller->output) <= VEHICLE_CONTROL_OUTPUT_EPSILON) &&
           (fabsf(measured_speed_mps) <= controller->config.zero_band_mps))
        {
            controller->output = 0.0f;
            controller->zero_dwell_s += dt_s;
            if(controller->zero_dwell_s >= controller->config.direction_hold_s)
            {
                controller->active_direction = requested_direction;
                controller->zero_dwell_s = 0.0f;
            }
        }
        else
        {
            controller->zero_dwell_s = 0.0f;
        }
        if(controller->active_direction != requested_direction)
        {
            return controller->output;
        }
    }
    else
    {
        controller->zero_dwell_s = 0.0f;
    }

    error = target_speed_mps - measured_speed_mps;
    feedforward = controller->config.feedforward * target_speed_mps;
    feedforward += (requested_direction > 0)
        ? controller->start_duty_forward
        : -controller->start_duty_reverse;

    proposed_integral = vehicle_clampf(
        controller->integral + controller->config.ki * error * dt_s,
        -controller->config.integral_limit,
        controller->config.integral_limit);
    proposed_output = feedforward + controller->config.kp * error +
                      proposed_integral;
    if((fabsf(proposed_output) <= controller->config.output_limit) ||
       ((proposed_output * error) < 0.0f))
    {
        controller->integral = proposed_integral;
    }

    desired_output = feedforward + controller->config.kp * error +
                     controller->integral;
    desired_output = vehicle_clampf(desired_output,
                                    -controller->config.output_limit,
                                    controller->config.output_limit);
    controller->output = vehicle_rate_limit(
        desired_output, controller->output,
        controller->config.output_slew_per_s,
        controller->config.output_slew_per_s, dt_s);
    controller->output = vehicle_clampf(controller->output,
                                        -controller->config.output_limit,
                                        controller->config.output_limit);
    return controller->output;
}

bool vehicle_control_compute_wheel_targets_from_curvature(
    float center_speed_mps,
    float curvature_per_m,
    float *left_target_speed_mps,
    float *right_target_speed_mps)
{
    float yaw_rate_radps;
    float half_speed_difference;

    if((left_target_speed_mps == nullptr) || (right_target_speed_mps == nullptr))
    {
        return false;
    }
    *left_target_speed_mps = 0.0f;
    *right_target_speed_mps = 0.0f;
    if(!isfinite(center_speed_mps) || !isfinite(curvature_per_m) ||
       !(VEHICLE_REAR_TRACK_M > 0.0f))
    {
        return false;
    }
    yaw_rate_radps = center_speed_mps * curvature_per_m;
    half_speed_difference = yaw_rate_radps * VEHICLE_REAR_TRACK_M * 0.5f;
    *left_target_speed_mps = center_speed_mps - half_speed_difference;
    *right_target_speed_mps = center_speed_mps + half_speed_difference;
    if(!isfinite(*left_target_speed_mps) ||
       !isfinite(*right_target_speed_mps))
    {
        *left_target_speed_mps = 0.0f;
        *right_target_speed_mps = 0.0f;
        return false;
    }
    return true;
}

bool vehicle_control_compute_wheel_targets(float center_speed_mps,
                                           float steering_angle_rad,
                                           float *left_target_speed_mps,
                                           float *right_target_speed_mps)
{
    float curvature_per_m;

    if((left_target_speed_mps == nullptr) || (right_target_speed_mps == nullptr))
    {
        return false;
    }
    *left_target_speed_mps = 0.0f;
    *right_target_speed_mps = 0.0f;
    if(!isfinite(steering_angle_rad) || !(VEHICLE_WHEELBASE_M > 0.0f))
    {
        return false;
    }
    curvature_per_m = tanf(steering_angle_rad) / VEHICLE_WHEELBASE_M;
    return vehicle_control_compute_wheel_targets_from_curvature(
        center_speed_mps, curvature_per_m,
        left_target_speed_mps, right_target_speed_mps);
}

void vehicle_drive_controller_init(VehicleDriveController *controller,
                                   const VehicleCalibration *calibration,
                                   const SteeringCalibrationTables *tables,
                                   const VehicleWheelControllerConfig *left_config,
                                   const VehicleWheelControllerConfig *right_config)
{
    bool valid;
    float left_forward;
    float left_reverse;
    float right_forward;
    float right_reverse;

    if(controller == nullptr)
    {
        return;
    }
    valid = vehicle_calibration_is_valid(calibration, tables);
    left_forward = (calibration != nullptr)
        ? calibration->left_motor_start_duty_forward : 0.0f;
    left_reverse = (calibration != nullptr)
        ? calibration->left_motor_start_duty_reverse : 0.0f;
    right_forward = (calibration != nullptr)
        ? calibration->right_motor_start_duty_forward : 0.0f;
    right_reverse = (calibration != nullptr)
        ? calibration->right_motor_start_duty_reverse : 0.0f;
    vehicle_wheel_speed_controller_init(&controller->left, left_config,
                                        left_forward, left_reverse, valid);
    vehicle_wheel_speed_controller_init(&controller->right, right_config,
                                        right_forward, right_reverse, valid);
    controller->calibration_valid = controller->left.calibration_valid &&
                                    controller->right.calibration_valid;
}

void vehicle_drive_controller_reset(VehicleDriveController *controller)
{
    if(controller != nullptr)
    {
        vehicle_wheel_speed_controller_reset(&controller->left);
        vehicle_wheel_speed_controller_reset(&controller->right);
    }
}

void vehicle_drive_controller_update(VehicleDriveController *controller,
                                     float center_speed_mps,
                                     float steering_angle_rad,
                                     float left_speed_mps,
                                     float right_speed_mps,
                                     float dt_s,
                                     VehicleDriveControlOutput *output)
{
    bool targets_valid;

    drive_output_zero(output);
    if((controller == nullptr) || (output == nullptr))
    {
        return;
    }
    targets_valid = vehicle_control_compute_wheel_targets(
        center_speed_mps, steering_angle_rad,
        &output->left_target_speed_mps,
        &output->right_target_speed_mps);
    if(!targets_valid || !isfinite(left_speed_mps) ||
       !isfinite(right_speed_mps) || !isfinite(dt_s) || !(dt_s > 0.0f))
    {
        vehicle_drive_controller_reset(controller);
        return;
    }
    output->left_signed_duty = vehicle_wheel_speed_controller_update(
        &controller->left, output->left_target_speed_mps,
        left_speed_mps, dt_s);
    output->right_signed_duty = vehicle_wheel_speed_controller_update(
        &controller->right, output->right_target_speed_mps,
        right_speed_mps, dt_s);
    output->valid = controller->calibration_valid;
    if(!output->valid)
    {
        output->left_signed_duty = 0.0f;
        output->right_signed_duty = 0.0f;
    }
}

void vehicle_steering_controller_default_config(
    VehicleSteeringControllerConfig *config)
{
    if(config != nullptr)
    {
        config->kp = STEERING_KP;
        config->ki = STEERING_KI;
        config->kd = STEERING_KD;
        config->integral_limit = STEERING_INTEGRAL_LIMIT;
        config->output_limit = STEERING_OUTPUT_LIMIT;
        config->output_slew_per_s = STEERING_OUTPUT_SLEW_PER_S;
        config->position_deadband_rad = STEERING_POSITION_DEADBAND_RAD;
        config->stall_duty = STEERING_STALL_DUTY;
        config->stall_count_delta = STEERING_STALL_COUNT_DELTA;
        config->stall_timeout_s = STEERING_STALL_TIMEOUT_S;
        config->approach_switch_hysteresis_count =
            STEERING_STALL_COUNT_DELTA;
    }
}

bool vehicle_steering_table_count_to_angle(
    const SteeringCalibrationPoint *points,
    size_t count,
    int64_t continuous_count,
    float *angle_rad)
{
    size_t index;

    if(angle_rad == nullptr)
    {
        return false;
    }
    *angle_rad = 0.0f;
    if(steering_table_angle_direction(points, count) == 0)
    {
        return false;
    }
    if(continuous_count <= points[0].continuous_count)
    {
        *angle_rad = points[0].equivalent_steering_angle_rad;
        return true;
    }
    if(continuous_count >= points[count - 1U].continuous_count)
    {
        *angle_rad = points[count - 1U].equivalent_steering_angle_rad;
        return true;
    }
    for(index = 1U; index < count; ++index)
    {
        if(continuous_count <= points[index].continuous_count)
        {
            double count_span;
            double count_offset;
            double ratio;
            double angle;

            count_span = static_cast<double>(points[index].continuous_count) -
                         static_cast<double>(points[index - 1U].continuous_count);
            count_offset = static_cast<double>(continuous_count) -
                           static_cast<double>(points[index - 1U].continuous_count);
            ratio = count_offset / count_span;
            angle = static_cast<double>(points[index - 1U].equivalent_steering_angle_rad) +
                    ratio *
                    (static_cast<double>(points[index].equivalent_steering_angle_rad) -
                     static_cast<double>(points[index - 1U].equivalent_steering_angle_rad));
            *angle_rad = static_cast<float>(angle);
            return isfinite(*angle_rad) != 0;
        }
    }
    return false;
}

bool vehicle_steering_table_angle_to_count(
    const SteeringCalibrationPoint *points,
    size_t count,
    float angle_rad,
    int64_t *continuous_count)
{
    int direction;
    size_t index;

    if(continuous_count == nullptr)
    {
        return false;
    }
    *continuous_count = 0;
    direction = steering_table_angle_direction(points, count);
    if((direction == 0) || !isfinite(angle_rad))
    {
        return false;
    }
    if(((direction > 0) &&
        (angle_rad <= points[0].equivalent_steering_angle_rad)) ||
       ((direction < 0) &&
        (angle_rad >= points[0].equivalent_steering_angle_rad)))
    {
        *continuous_count = points[0].continuous_count;
        return true;
    }
    if(((direction > 0) &&
        (angle_rad >= points[count - 1U].equivalent_steering_angle_rad)) ||
       ((direction < 0) &&
        (angle_rad <= points[count - 1U].equivalent_steering_angle_rad)))
    {
        *continuous_count = points[count - 1U].continuous_count;
        return true;
    }
    for(index = 1U; index < count; ++index)
    {
        float current_angle;
        bool segment_contains_angle;

        current_angle = points[index].equivalent_steering_angle_rad;
        segment_contains_angle = (direction > 0)
            ? (angle_rad <= current_angle)
            : (angle_rad >= current_angle);
        if(segment_contains_angle)
        {
            double angle_span;
            double angle_offset;
            double ratio;
            double interpolated_count;

            angle_span =
                static_cast<double>(points[index].equivalent_steering_angle_rad) -
                static_cast<double>(points[index - 1U].equivalent_steering_angle_rad);
            angle_offset = static_cast<double>(angle_rad) -
                static_cast<double>(points[index - 1U].equivalent_steering_angle_rad);
            ratio = angle_offset / angle_span;
            interpolated_count =
                static_cast<double>(points[index - 1U].continuous_count) + ratio *
                (static_cast<double>(points[index].continuous_count) -
                 static_cast<double>(points[index - 1U].continuous_count));
            *continuous_count = rounded_i64(interpolated_count);
            return true;
        }
    }
    return false;
}

void vehicle_steering_controller_init(
    VehicleSteeringController *controller,
    const VehicleCalibration *calibration,
    const SteeringCalibrationTables *tables,
    const VehicleSteeringControllerConfig *config)
{
    VehicleSteeringControllerConfig default_config;
    const VehicleSteeringControllerConfig *selected_config;
    bool calibration_valid;

    if(controller == nullptr)
    {
        return;
    }
    vehicle_steering_controller_default_config(&default_config);
    selected_config = (config != nullptr) ? config : &default_config;
    controller->config = *selected_config;
    controller->calibration = calibration;
    if(tables != nullptr)
    {
        controller->tables = *tables;
    }
    else
    {
        controller->tables.from_left = nullptr;
        controller->tables.from_left_count = 0U;
        controller->tables.from_right = nullptr;
        controller->tables.from_right_count = 0U;
    }
    calibration_valid = vehicle_calibration_is_valid(calibration, tables);
    controller->calibration_valid = calibration_valid && CALIBRATION_VALID &&
        steering_config_is_valid(selected_config) &&
        steering_tables_are_usable(tables) &&
        (calibration != nullptr) &&
        isfinite(calibration->steering_start_duty_left) &&
        isfinite(calibration->steering_start_duty_right) &&
        (calibration->steering_start_duty_left >= 0.0f) &&
        (calibration->steering_start_duty_right >= 0.0f) &&
        (calibration->steering_start_duty_left <=
         selected_config->output_limit) &&
        (calibration->steering_start_duty_right <=
         selected_config->output_limit);
    vehicle_steering_controller_reset(controller);
}

void vehicle_steering_controller_reset(VehicleSteeringController *controller)
{
    if(controller != nullptr)
    {
        controller->integral = 0.0f;
        controller->previous_error_rad = 0.0f;
        controller->output = 0.0f;
        controller->stall_elapsed_s = 0.0f;
        controller->previous_count = 0;
        controller->target_count = 0;
        controller->approach = VEHICLE_STEERING_APPROACH_UNKNOWN;
        controller->previous_sample_valid = false;
        controller->stalled = false;
    }
}

static VehicleSteeringApproach steering_select_approach(
    const VehicleSteeringController *controller,
    float target_angle_rad,
    int64_t continuous_count)
{
    int64_t from_left_count;
    int64_t from_right_count;
    double delta;
    double hysteresis;
    int angle_direction;

    if((controller == nullptr) ||
       !vehicle_steering_table_angle_to_count(
           controller->tables.from_left,
           controller->tables.from_left_count,
           target_angle_rad, &from_left_count) ||
       !vehicle_steering_table_angle_to_count(
           controller->tables.from_right,
           controller->tables.from_right_count,
           target_angle_rad, &from_right_count))
    {
        return VEHICLE_STEERING_APPROACH_UNKNOWN;
    }
    angle_direction = steering_table_angle_direction(
        controller->tables.from_left,
        controller->tables.from_left_count);
    if(angle_direction == 0)
    {
        return VEHICLE_STEERING_APPROACH_UNKNOWN;
    }
    hysteresis = static_cast<double>(controller->config.approach_switch_hysteresis_count);
    if(controller->approach == VEHICLE_STEERING_APPROACH_FROM_LEFT)
    {
        delta = (static_cast<double>(from_left_count) - static_cast<double>(continuous_count)) *
                static_cast<double>(angle_direction);
        return (delta > hysteresis)
            ? VEHICLE_STEERING_APPROACH_FROM_RIGHT
            : VEHICLE_STEERING_APPROACH_FROM_LEFT;
    }
    if(controller->approach == VEHICLE_STEERING_APPROACH_FROM_RIGHT)
    {
        delta = (static_cast<double>(from_right_count) - static_cast<double>(continuous_count)) *
                static_cast<double>(angle_direction);
        return (delta < -hysteresis)
            ? VEHICLE_STEERING_APPROACH_FROM_LEFT
            : VEHICLE_STEERING_APPROACH_FROM_RIGHT;
    }

    delta = ((static_cast<double>(from_left_count) + static_cast<double>(from_right_count)) * 0.5) -
            static_cast<double>(continuous_count);
    delta *= static_cast<double>(angle_direction);
    if(delta > hysteresis)
    {
        return VEHICLE_STEERING_APPROACH_FROM_RIGHT;
    }
    return VEHICLE_STEERING_APPROACH_FROM_LEFT;
}

void vehicle_steering_controller_update(
    VehicleSteeringController *controller,
    float target_angle_rad,
    int64_t continuous_count,
    float dt_s,
    VehicleSteeringControlOutput *output)
{
    const SteeringCalibrationPoint *points;
    size_t point_count;
    VehicleSteeringApproach approach;
    float measured_angle_rad;
    float effective_target_angle_rad;
    float error_rad;
    float derivative_radps;
    float proposed_integral;
    float base_output;
    float signed_error;
    float desired_output;
    float start_duty;
    int angle_direction;
    int64_t lower_limit_count;
    int64_t upper_limit_count;
    bool at_left_limit;
    bool at_right_limit;
    bool at_soft_limit;
    bool small_count_change;

    steering_output_zero(output);
    if((controller == nullptr) || (output == nullptr))
    {
        return;
    }
    if(!controller->calibration_valid ||
       !steering_config_is_valid(&controller->config) ||
       (controller->calibration == nullptr) ||
       !isfinite(target_angle_rad) || !isfinite(dt_s) || !(dt_s > 0.0f))
    {
        vehicle_steering_controller_reset(controller);
        return;
    }
    if(controller->stalled)
    {
        output->stalled = true;
        output->approach = controller->approach;
        output->target_count = controller->target_count;
        return;
    }

    approach = steering_select_approach(controller, target_angle_rad,
                                        continuous_count);
    if(approach == VEHICLE_STEERING_APPROACH_UNKNOWN)
    {
        vehicle_steering_controller_reset(controller);
        return;
    }
    points = steering_points_for_approach(&controller->tables, approach,
                                          &point_count);
    if((points == nullptr) ||
       !vehicle_steering_table_angle_to_count(
           points, point_count, target_angle_rad,
           &controller->target_count))
    {
        vehicle_steering_controller_reset(controller);
        return;
    }
    lower_limit_count =
        (controller->calibration->steering_left_soft_limit_count <
         controller->calibration->steering_right_soft_limit_count)
            ? controller->calibration->steering_left_soft_limit_count
            : controller->calibration->steering_right_soft_limit_count;
    upper_limit_count =
        (controller->calibration->steering_left_soft_limit_count >
         controller->calibration->steering_right_soft_limit_count)
            ? controller->calibration->steering_left_soft_limit_count
            : controller->calibration->steering_right_soft_limit_count;
    if(controller->target_count < lower_limit_count)
    {
        controller->target_count = lower_limit_count;
    }
    if(controller->target_count > upper_limit_count)
    {
        controller->target_count = upper_limit_count;
    }
    if(!vehicle_steering_table_count_to_angle(
           points, point_count, continuous_count, &measured_angle_rad) ||
       !vehicle_steering_table_count_to_angle(
           points, point_count, controller->target_count,
           &effective_target_angle_rad))
    {
        vehicle_steering_controller_reset(controller);
        return;
    }
    angle_direction = steering_table_angle_direction(points, point_count);
    if(angle_direction == 0)
    {
        vehicle_steering_controller_reset(controller);
        return;
    }

    controller->approach = approach;
    error_rad = effective_target_angle_rad - measured_angle_rad;
    derivative_radps = controller->previous_sample_valid
        ? (error_rad - controller->previous_error_rad) / dt_s
        : 0.0f;
    if(!isfinite(error_rad) || !isfinite(derivative_radps))
    {
        vehicle_steering_controller_reset(controller);
        return;
    }

    output->measured_angle_rad = measured_angle_rad;
    output->target_count = controller->target_count;
    output->approach = approach;
    output->valid = true;

    if(fabsf(error_rad) <= controller->config.position_deadband_rad)
    {
        controller->integral = 0.0f;
        controller->output = 0.0f;
        controller->stall_elapsed_s = 0.0f;
        controller->previous_error_rad = error_rad;
        controller->previous_count = continuous_count;
        controller->previous_sample_valid = true;
        output->signed_duty = 0.0f;
        return;
    }

    signed_error = error_rad;
    proposed_integral = vehicle_clampf(
        controller->integral + controller->config.ki * error_rad * dt_s,
        -controller->config.integral_limit,
        controller->config.integral_limit);
    base_output = controller->config.kp * error_rad + proposed_integral +
                  controller->config.kd * derivative_radps;
    if((fabsf(base_output) <= controller->config.output_limit) ||
       ((base_output * signed_error) < 0.0f))
    {
        controller->integral = proposed_integral;
    }

    base_output = controller->config.kp * error_rad +
                  controller->integral +
                  controller->config.kd * derivative_radps;
    if(base_output > VEHICLE_CONTROL_OUTPUT_EPSILON)
    {
        start_duty = controller->calibration->steering_start_duty_left;
        desired_output = base_output + start_duty;
    }
    else if(base_output < -VEHICLE_CONTROL_OUTPUT_EPSILON)
    {
        start_duty = controller->calibration->steering_start_duty_right;
        desired_output = base_output - start_duty;
    }
    else
    {
        desired_output = 0.0f;
    }
    desired_output = vehicle_clampf(desired_output,
                                    -controller->config.output_limit,
                                    controller->config.output_limit);

    at_left_limit = (angle_direction > 0)
        ? (continuous_count >=
           controller->calibration->steering_left_soft_limit_count)
        : (continuous_count <=
           controller->calibration->steering_left_soft_limit_count);
    at_right_limit = (angle_direction > 0)
        ? (continuous_count <=
           controller->calibration->steering_right_soft_limit_count)
        : (continuous_count >=
           controller->calibration->steering_right_soft_limit_count);
    at_soft_limit = false;
    if((at_left_limit && (desired_output > 0.0f)) ||
       (at_right_limit && (desired_output < 0.0f)))
    {
        desired_output = 0.0f;
        controller->integral = 0.0f;
        at_soft_limit = true;
    }

    controller->output = vehicle_rate_limit(
        desired_output, controller->output,
        controller->config.output_slew_per_s,
        controller->config.output_slew_per_s, dt_s);
    controller->output = vehicle_clampf(controller->output,
                                        -controller->config.output_limit,
                                        controller->config.output_limit);
    if(at_soft_limit)
    {
        controller->output = 0.0f;
    }

    small_count_change = controller->previous_sample_valid &&
        (fabs(static_cast<double>(continuous_count) -
              static_cast<double>(controller->previous_count)) <=
         static_cast<double>(controller->config.stall_count_delta));
    if(!at_soft_limit && small_count_change &&
       (fabsf(controller->output) >= controller->config.stall_duty))
    {
        controller->stall_elapsed_s += dt_s;
        if(controller->stall_elapsed_s >= controller->config.stall_timeout_s)
        {
            controller->stalled = true;
            controller->output = 0.0f;
        }
    }
    else
    {
        controller->stall_elapsed_s = 0.0f;
    }

    controller->previous_error_rad = error_rad;
    controller->previous_count = continuous_count;
    controller->previous_sample_valid = true;
    output->signed_duty = controller->output;
    output->at_soft_limit = at_soft_limit;
    output->stalled = controller->stalled;
}

bool vehicle_steering_controller_is_stalled(
    const VehicleSteeringController *controller)
{
    return (controller != nullptr) && controller->stalled;
}

void vehicle_control_init(VehicleControl *control,
                          const VehicleCalibration *calibration,
                          const SteeringCalibrationTables *tables)
{
    if(control != nullptr)
    {
        vehicle_drive_controller_init(&control->drive, calibration, tables,
                                      nullptr, nullptr);
        vehicle_steering_controller_init(&control->steering, calibration,
                                         tables, nullptr);
    }
}

void vehicle_control_reset(VehicleControl *control)
{
    if(control != nullptr)
    {
        vehicle_drive_controller_reset(&control->drive);
        vehicle_steering_controller_reset(&control->steering);
    }
}

void vehicle_control_update(VehicleControl *control,
                            const VehicleControlInput *input,
                            VehicleControlOutput *output)
{
    if(output != nullptr)
    {
        drive_output_zero(&output->drive);
        steering_output_zero(&output->steering);
        output->actuators.left_motor_duty = 0.0f;
        output->actuators.right_motor_duty = 0.0f;
        output->actuators.steering_motor_duty = 0.0f;
        output->actuators.immediate_stop = false;
        output->valid = false;
    }
    if((control == nullptr) || (input == nullptr) || (output == nullptr))
    {
        return;
    }
    vehicle_drive_controller_update(
        &control->drive, input->target_center_speed_mps,
        input->target_steering_rad, input->left_speed_mps,
        input->right_speed_mps, input->dt_s, &output->drive);
    vehicle_steering_controller_update(
        &control->steering, input->target_steering_rad,
        input->steering_continuous_count, input->dt_s,
        &output->steering);

    output->valid = output->drive.valid && output->steering.valid &&
                    !output->steering.stalled;
    if(output->valid)
    {
        output->actuators.left_motor_duty =
            output->drive.left_signed_duty;
        output->actuators.right_motor_duty =
            output->drive.right_signed_duty;
        output->actuators.steering_motor_duty =
            output->steering.signed_duty;
    }
    else
    {
        output->drive.left_signed_duty = 0.0f;
        output->drive.right_signed_duty = 0.0f;
        output->steering.signed_duty = 0.0f;
        output->actuators.left_motor_duty = 0.0f;
        output->actuators.right_motor_duty = 0.0f;
        output->actuators.steering_motor_duty = 0.0f;
        output->actuators.immediate_stop = output->steering.stalled;
    }
}
