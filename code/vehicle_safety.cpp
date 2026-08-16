#include "vehicle_safety.h"

#include <math.h>
#include <stddef.h>

#include "vehicle_config.h"
#include "vehicle_math.h"

static bool safety_config_is_valid(const VehicleSafetyConfig *config)
{
    return (config != nullptr) &&
           isfinite(config->encoder_max_abs_speed_mps) &&
           isfinite(config->encoder_max_acceleration_mps2) &&
           isfinite(config->motor_stall_duty) &&
           isfinite(config->motor_stall_speed_mps) &&
           isfinite(config->motor_stall_timeout_s) &&
           isfinite(config->wheel_mismatch_min_speed_mps) &&
           isfinite(config->wheel_mismatch_max_mps) &&
           isfinite(config->wheel_mismatch_timeout_s) &&
           isfinite(config->steering_stall_duty) &&
           isfinite(config->steering_stall_timeout_s) &&
           isfinite(config->imu_max_acceleration_mps2) &&
           isfinite(config->imu_max_angular_rate_radps) &&
           isfinite(config->tracker_max_cross_track_error_m) &&
           isfinite(config->tracker_max_heading_error_rad) &&
           (config->encoder_max_abs_speed_mps > 0.0f) &&
           (config->encoder_max_acceleration_mps2 > 0.0f) &&
           (config->motor_stall_duty >= 0.0f) &&
           (config->motor_stall_duty <= 1.0f) &&
           (config->motor_stall_speed_mps >= 0.0f) &&
           (config->motor_stall_timeout_s > 0.0f) &&
           (config->wheel_mismatch_min_speed_mps >= 0.0f) &&
           (config->wheel_mismatch_max_mps > 0.0f) &&
           (config->wheel_mismatch_timeout_s > 0.0f) &&
           (config->mt6701_max_delta_count > 0) &&
           (config->mt6701_error_limit > 0U) &&
           (config->steering_stall_duty >= 0.0f) &&
           (config->steering_stall_duty <= 1.0f) &&
           (config->steering_stall_count_delta >= 0) &&
           (config->steering_stall_timeout_s > 0.0f) &&
           (config->imu_max_acceleration_mps2 > 0.0f) &&
           (config->imu_max_angular_rate_radps > 0.0f) &&
           (config->sensor_timeout_us > 0U) &&
           (config->control_timeout_us > 0U) &&
           (config->tracker_index_loss_limit > 0U) &&
           (config->tracker_max_cross_track_error_m > 0.0f) &&
           (config->tracker_max_heading_error_rad > 0.0f);
}

static bool timestamp_is_stale(uint64_t now_us, uint64_t sample_us,
                               uint64_t timeout_us)
{
    if((sample_us == 0U) || (now_us < sample_us))
    {
        return true;
    }
    return (now_us - sample_us) > timeout_us;
}

static float elapsed_seconds(uint64_t now_us, uint64_t previous_us)
{
    double elapsed;

    if((previous_us == 0U) || (now_us <= previous_us))
    {
        return 0.0f;
    }
    elapsed = static_cast<double>(now_us - previous_us) * 0.000001;
    if(elapsed > 1.0)
    {
        elapsed = 1.0;
    }
    return static_cast<float>(elapsed);
}

static float update_condition_timer(float elapsed_s, bool condition,
                                    float dt_s, float limit_s)
{
    if(!condition)
    {
        return 0.0f;
    }
    elapsed_s += dt_s;
    if(elapsed_s > limit_s)
    {
        elapsed_s = limit_s;
    }
    return elapsed_s;
}

static void safety_set_condition(VehicleSafetyMonitor *monitor,
                                 VehicleFaultManager *fault_manager,
                                 uint32_t flag, bool condition,
                                 bool immediate_stop, uint64_t timestamp_us)
{
    if(immediate_stop)
    {
        if(condition)
        {
            vehicle_fault_set_condition(fault_manager, flag, true,
                                        true, timestamp_us);
            monitor->monitored_active_flags |= flag;
        }
        else if((monitor->monitored_active_flags & flag) != 0U)
        {
            vehicle_fault_set_condition(fault_manager, flag, false,
                                        true, timestamp_us);
            monitor->monitored_active_flags &= ~flag;
        }
    }
    else if(condition)
    {
        vehicle_fault_latch(fault_manager, flag, timestamp_us);
        monitor->monitored_active_flags |= flag;
        monitor->controlled_stop_request_flags |= flag;
    }
    else
    {
        monitor->monitored_active_flags &= ~flag;
        monitor->controlled_stop_request_flags &= ~flag;
    }
}

static bool state_requires_sensors(VehicleState state)
{
    return (state == VEHICLE_STATE_STAGE1_RECORD) ||
           (state == VEHICLE_STATE_STAGE2_REVERSE);
}

static bool state_requires_control_deadline(VehicleState state)
{
    return (state == VEHICLE_STATE_STAGE1_RECORD) ||
           (state == VEHICLE_STATE_STAGE2_REVERSE);
}

static bool state_requires_path(VehicleState state)
{
    return (state == VEHICLE_STATE_STAGE1_FINISHED) ||
           (state == VEHICLE_STATE_STAGE2_REVERSE);
}

static bool values_are_finite(const VehicleSafetyInputs *inputs)
{
    const float actuator_values[] =
    {
        inputs->left_target_speed_mps,
        inputs->right_target_speed_mps,
        inputs->left_motor_duty,
        inputs->right_motor_duty,
        inputs->steering_motor_duty
    };

    if(!inputs->numeric_valid ||
       !vehicle_float_array_is_finite(actuator_values,
                                      static_cast<unsigned int>(sizeof(actuator_values) /
                                      sizeof(actuator_values[0]))) ||
       (fabsf(inputs->left_motor_duty) > 1.0f) ||
       (fabsf(inputs->right_motor_duty) > 1.0f) ||
       (fabsf(inputs->steering_motor_duty) > 1.0f))
    {
        return false;
    }
    if(inputs->left_wheel.valid &&
       !isfinite(inputs->left_wheel.speed_mps))
    {
        return false;
    }
    if(inputs->right_wheel.valid &&
       !isfinite(inputs->right_wheel.speed_mps))
    {
        return false;
    }
    if(inputs->steering.valid && !isfinite(inputs->steering.angle_rad))
    {
        return false;
    }
    if(inputs->imu.accel_gyro_valid)
    {
        if(!vehicle_float_array_is_finite(inputs->imu.acceleration_mps2, 3U) ||
           !vehicle_float_array_is_finite(inputs->imu.angular_rate_radps, 3U) ||
           !isfinite(inputs->imu.temperature_c))
        {
            return false;
        }
    }
    if(inputs->imu.magnetometer_valid &&
       !vehicle_float_array_is_finite(inputs->imu.magnetic_field_gauss, 3U))
    {
        return false;
    }
    if(inputs->pose.valid)
    {
        const float pose_values[] =
        {
            inputs->pose.x_m,
            inputs->pose.y_m,
            inputs->pose.yaw_rad,
            inputs->pose.vehicle_speed_mps,
            inputs->pose.gyro_z_bias_radps,
            inputs->pose.omega_imu_radps,
            inputs->pose.omega_wheel_radps,
            inputs->pose.omega_steering_radps
        };

        if(!vehicle_float_array_is_finite(
               pose_values,
               static_cast<unsigned int>(sizeof(pose_values) / sizeof(pose_values[0]))))
        {
            return false;
        }
    }
    if(inputs->tracker.valid)
    {
        const float tracker_values[] =
        {
            inputs->tracker.target_speed_mps,
            inputs->tracker.target_steering_rad,
            inputs->tracker.cross_track_error_m,
            inputs->tracker.heading_error_rad,
            inputs->tracker.lookahead_m
        };

        if(!vehicle_float_array_is_finite(
               tracker_values,
               static_cast<unsigned int>(sizeof(tracker_values) /
               sizeof(tracker_values[0]))))
        {
            return false;
        }
    }
    if(inputs->additional_numeric_value_count > 0U)
    {
        if((inputs->additional_numeric_values == nullptr) ||
           !vehicle_float_array_is_finite(
               inputs->additional_numeric_values,
               static_cast<unsigned int>(inputs->additional_numeric_value_count)))
        {
            return false;
        }
    }
    return true;
}

static bool encoder_jump_detected(
    const WheelEncoderSample *sample,
    bool previous_valid,
    uint64_t previous_timestamp_us,
    float previous_speed_mps,
    float meter_per_count,
    bool calibration_valid,
    const VehicleSafetyConfig *config)
{
    double dt_s;
    double maximum_count_delta;
    double count_delta;
    float maximum_speed_step;

    if(sample == nullptr)
    {
        return false;
    }
    if(!sample->valid)
    {
        return calibration_valid && (sample->timestamp_us != 0U);
    }
    if(!isfinite(sample->speed_mps))
    {
        return true;
    }
    if(fabsf(sample->speed_mps) > config->encoder_max_abs_speed_mps)
    {
        return true;
    }
    if(!previous_valid)
    {
        return false;
    }
    if(sample->timestamp_us < previous_timestamp_us)
    {
        return true;
    }
    if(sample->timestamp_us == previous_timestamp_us)
    {
        return false;
    }

    dt_s = static_cast<double>(sample->timestamp_us - previous_timestamp_us) * 0.000001;
    if(calibration_valid && isfinite(meter_per_count) &&
       (meter_per_count > 0.0f))
    {
        maximum_count_delta =
            (static_cast<double>(config->encoder_max_abs_speed_mps) * dt_s /
             static_cast<double>(meter_per_count)) + 2.0;
        count_delta = fabs(static_cast<double>(sample->delta_count));
        if(count_delta > maximum_count_delta)
        {
            return true;
        }
    }
    maximum_speed_step = config->encoder_max_acceleration_mps2 *
                         static_cast<float>(dt_s) + 0.05f;
    return fabsf(sample->speed_mps - previous_speed_mps) >
           maximum_speed_step;
}

static bool imu_range_invalid(const ImuSample *imu,
                              const VehicleSafetyConfig *config)
{
    unsigned int axis;

    if((imu == nullptr) || !imu->accel_gyro_valid)
    {
        return false;
    }
    for(axis = 0U; axis < 3U; ++axis)
    {
        if(!isfinite(imu->acceleration_mps2[axis]) ||
           !isfinite(imu->angular_rate_radps[axis]) ||
           (fabsf(imu->acceleration_mps2[axis]) >
            config->imu_max_acceleration_mps2) ||
           (fabsf(imu->angular_rate_radps[axis]) >
            config->imu_max_angular_rate_radps))
        {
            return true;
        }
    }
    return false;
}

void vehicle_safety_default_config(VehicleSafetyConfig *config)
{
    if(config != nullptr)
    {
        config->encoder_max_abs_speed_mps = ENCODER_MAX_ABS_SPEED_MPS;
        config->encoder_max_acceleration_mps2 = ENCODER_MAX_ACCEL_MPS2;
        config->motor_stall_duty = MOTOR_STALL_DUTY;
        config->motor_stall_speed_mps = MOTOR_STALL_SPEED_MPS;
        config->motor_stall_timeout_s = MOTOR_STALL_TIMEOUT_S;
        config->wheel_mismatch_min_speed_mps = WHEEL_MISMATCH_MIN_SPEED_MPS;
        config->wheel_mismatch_max_mps = WHEEL_MISMATCH_MAX_MPS;
        config->wheel_mismatch_timeout_s = WHEEL_MISMATCH_TIMEOUT_S;
        config->mt6701_max_delta_count = MT6701_MAX_DELTA_COUNT_PER_SAMPLE;
        config->mt6701_error_limit = MT6701_ERROR_LIMIT;
        config->steering_stall_duty = STEERING_STALL_DUTY;
        config->steering_stall_count_delta = STEERING_STALL_COUNT_DELTA;
        config->steering_stall_timeout_s = STEERING_STALL_TIMEOUT_S;
        config->imu_max_acceleration_mps2 = IMU_MAX_ACCEL_MPS2;
        config->imu_max_angular_rate_radps = IMU_MAX_GYRO_RADPS;
        config->sensor_timeout_us = VEHICLE_SENSOR_TIMEOUT_US;
        config->control_timeout_us = VEHICLE_CONTROL_TIMEOUT_US;
        config->tracker_index_loss_limit = REVERSE_INDEX_LOSS_LIMIT;
        config->tracker_max_cross_track_error_m =
            REVERSE_MAX_CROSSTRACK_ERROR_M;
        config->tracker_max_heading_error_rad =
            REVERSE_MAX_HEADING_ERROR_RAD;
    }
}

void vehicle_safety_init(VehicleSafetyMonitor *monitor,
                         const VehicleSafetyConfig *config,
                         const VehicleCalibration *calibration,
                         const SteeringCalibrationTables *tables)
{
    VehicleSafetyConfig default_config;
    const VehicleSafetyConfig *selected_config;

    if(monitor == nullptr)
    {
        return;
    }
    vehicle_safety_default_config(&default_config);
    selected_config = safety_config_is_valid(config) ? config : &default_config;
    monitor->config = *selected_config;
    monitor->calibration_valid =
        vehicle_calibration_is_valid(calibration, tables) && CALIBRATION_VALID;
    monitor->left_meter_per_count = (calibration != nullptr)
        ? calibration->left_meter_per_count : 0.0f;
    monitor->right_meter_per_count = (calibration != nullptr)
        ? calibration->right_meter_per_count : 0.0f;
    monitor->steering_left_limit_count = (calibration != nullptr)
        ? calibration->steering_left_soft_limit_count : 0;
    monitor->steering_right_limit_count = (calibration != nullptr)
        ? calibration->steering_right_soft_limit_count : 0;
    monitor->left_stall_elapsed_s = 0.0f;
    monitor->right_stall_elapsed_s = 0.0f;
    monitor->wheel_mismatch_elapsed_s = 0.0f;
    monitor->steering_stall_elapsed_s = 0.0f;
    monitor->previous_left_speed_mps = 0.0f;
    monitor->previous_right_speed_mps = 0.0f;
    monitor->previous_left_timestamp_us = 0U;
    monitor->previous_right_timestamp_us = 0U;
    monitor->previous_steering_count = 0;
    monitor->previous_steering_timestamp_us = 0U;
    monitor->previous_update_timestamp_us = 0U;
    monitor->mt6701_error_count = 0U;
    monitor->tracker_index_loss_count = 0U;
    monitor->monitored_active_flags = VEHICLE_FAULT_NONE;
    monitor->controlled_stop_request_flags = VEHICLE_FAULT_NONE;
    monitor->previous_left_valid = false;
    monitor->previous_right_valid = false;
    monitor->previous_steering_valid = false;
    monitor->automatic_output_inhibited = !monitor->calibration_valid;
}

void vehicle_safety_update(VehicleSafetyMonitor *monitor,
                           VehicleFaultManager *fault_manager,
                           const VehicleSafetyInputs *inputs)
{
    float dt_s;
    bool sensors_required;
    bool numeric_fault;
    bool left_jump;
    bool right_jump;
    bool left_stall_condition;
    bool right_stall_condition;
    bool wheel_mismatch_condition;
    bool mt_comm_fault;
    bool mt_timeout;
    bool mt_jump;
    bool steering_limit;
    bool steering_stall_condition;
    bool imu_comm_fault;
    bool imu_timeout;
    bool imu_range_fault;
    bool control_timeout;
    bool path_overflow;
    bool path_invalid;
    bool tracker_lost;
    bool tracking_error;
    uint32_t path_capacity;
    int64_t steering_lower_limit;
    int64_t steering_upper_limit;

    if((monitor == nullptr) || (fault_manager == nullptr))
    {
        return;
    }
    if(inputs == nullptr)
    {
        vehicle_fault_raise(fault_manager, VEHICLE_FAULT_NUMERIC, true, 0U);
        monitor->monitored_active_flags |= VEHICLE_FAULT_NUMERIC;
        monitor->automatic_output_inhibited = true;
        return;
    }

    dt_s = elapsed_seconds(inputs->timestamp_us,
                           monitor->previous_update_timestamp_us);
    sensors_required = state_requires_sensors(inputs->state) ||
        ((monitor->monitored_active_flags &
          (VEHICLE_FAULT_MT6701_COMM |
           VEHICLE_FAULT_IMU_COMM |
           VEHICLE_FAULT_IMU_RANGE |
           VEHICLE_FAULT_IMU_TIMEOUT)) != 0U);
    numeric_fault = !values_are_finite(inputs) ||
        ((monitor->previous_update_timestamp_us != 0U) &&
         (inputs->timestamp_us < monitor->previous_update_timestamp_us));

    left_jump = encoder_jump_detected(
        &inputs->left_wheel, monitor->previous_left_valid,
        monitor->previous_left_timestamp_us,
        monitor->previous_left_speed_mps,
        monitor->left_meter_per_count, monitor->calibration_valid,
        &monitor->config);
    right_jump = encoder_jump_detected(
        &inputs->right_wheel, monitor->previous_right_valid,
        monitor->previous_right_timestamp_us,
        monitor->previous_right_speed_mps,
        monitor->right_meter_per_count, monitor->calibration_valid,
        &monitor->config);

    left_stall_condition =
        (fabsf(inputs->left_motor_duty) >= monitor->config.motor_stall_duty) &&
        (!inputs->left_wheel.valid ||
         (fabsf(inputs->left_wheel.speed_mps) <=
          monitor->config.motor_stall_speed_mps));
    right_stall_condition =
        (fabsf(inputs->right_motor_duty) >= monitor->config.motor_stall_duty) &&
        (!inputs->right_wheel.valid ||
         (fabsf(inputs->right_wheel.speed_mps) <=
          monitor->config.motor_stall_speed_mps));
    monitor->left_stall_elapsed_s = update_condition_timer(
        monitor->left_stall_elapsed_s, left_stall_condition, dt_s,
        monitor->config.motor_stall_timeout_s);
    monitor->right_stall_elapsed_s = update_condition_timer(
        monitor->right_stall_elapsed_s, right_stall_condition, dt_s,
        monitor->config.motor_stall_timeout_s);

    wheel_mismatch_condition = false;
    if(inputs->left_wheel.valid && inputs->right_wheel.valid)
    {
        float actual_difference;
        float requested_difference;
        float activity_speed;

        actual_difference = inputs->left_wheel.speed_mps -
                            inputs->right_wheel.speed_mps;
        requested_difference = inputs->left_target_speed_mps -
                               inputs->right_target_speed_mps;
        activity_speed = fmaxf(
            fmaxf(fabsf(inputs->left_wheel.speed_mps),
                  fabsf(inputs->right_wheel.speed_mps)),
            fmaxf(fabsf(inputs->left_target_speed_mps),
                  fabsf(inputs->right_target_speed_mps)));
        wheel_mismatch_condition =
            (activity_speed >= monitor->config.wheel_mismatch_min_speed_mps) &&
            (fabsf(actual_difference - requested_difference) >
             monitor->config.wheel_mismatch_max_mps);
    }
    monitor->wheel_mismatch_elapsed_s = update_condition_timer(
        monitor->wheel_mismatch_elapsed_s, wheel_mismatch_condition, dt_s,
        monitor->config.wheel_mismatch_timeout_s);

    if(!inputs->mt6701_communication_ok || !inputs->steering.valid)
    {
        if(monitor->mt6701_error_count < UINT32_MAX)
        {
            ++monitor->mt6701_error_count;
        }
    }
    else
    {
        monitor->mt6701_error_count = 0U;
    }
    mt_timeout = sensors_required && timestamp_is_stale(
        inputs->timestamp_us, inputs->steering.timestamp_us,
        monitor->config.sensor_timeout_us);
    mt_comm_fault = sensors_required &&
        ((monitor->mt6701_error_count >=
          monitor->config.mt6701_error_limit) || mt_timeout);
    mt_jump = false;
    if(inputs->mt6701_communication_ok && inputs->steering.valid &&
       monitor->previous_steering_valid &&
       (inputs->steering.timestamp_us !=
        monitor->previous_steering_timestamp_us))
    {
        mt_jump = (inputs->steering.timestamp_us <
                   monitor->previous_steering_timestamp_us) ||
            (fabs(static_cast<double>(inputs->steering.continuous_count) -
                  static_cast<double>(monitor->previous_steering_count)) >
             static_cast<double>(monitor->config.mt6701_max_delta_count));
    }
    steering_lower_limit = (monitor->steering_left_limit_count <
                            monitor->steering_right_limit_count)
        ? monitor->steering_left_limit_count
        : monitor->steering_right_limit_count;
    steering_upper_limit = (monitor->steering_left_limit_count >
                            monitor->steering_right_limit_count)
        ? monitor->steering_left_limit_count
        : monitor->steering_right_limit_count;
    steering_limit = monitor->calibration_valid && inputs->steering.valid &&
        ((inputs->steering.relative_count < steering_lower_limit) ||
         (inputs->steering.relative_count > steering_upper_limit));
    steering_stall_condition = inputs->steering.valid &&
        (fabsf(inputs->steering_motor_duty) >=
         monitor->config.steering_stall_duty) &&
        monitor->previous_steering_valid &&
        (fabs(static_cast<double>(inputs->steering.continuous_count) -
              static_cast<double>(monitor->previous_steering_count)) <=
         static_cast<double>(monitor->config.steering_stall_count_delta));
    monitor->steering_stall_elapsed_s = update_condition_timer(
        monitor->steering_stall_elapsed_s, steering_stall_condition, dt_s,
        monitor->config.steering_stall_timeout_s);

    imu_comm_fault = sensors_required &&
        (!inputs->imu_communication_ok || !inputs->imu.accel_gyro_valid);
    imu_timeout = sensors_required && timestamp_is_stale(
        inputs->timestamp_us, inputs->imu.timestamp_us,
        monitor->config.sensor_timeout_us);
    imu_range_fault = sensors_required &&
        imu_range_invalid(&inputs->imu, &monitor->config);
    control_timeout = state_requires_control_deadline(inputs->state) &&
        timestamp_is_stale(inputs->timestamp_us,
                           inputs->last_control_timestamp_us,
                           monitor->config.control_timeout_us);

    path_capacity = (inputs->path_capacity > 0U)
        ? inputs->path_capacity : PATH_MAX_POINTS;
    path_overflow = !inputs->controlled_stop_in_progress &&
                    ((inputs->state == VEHICLE_STATE_STAGE1_RECORD) ||
                     state_requires_path(inputs->state)) &&
                    (inputs->path_overflow ||
                     (inputs->path_point_count > path_capacity));
    path_invalid = !inputs->controlled_stop_in_progress &&
        state_requires_path(inputs->state) &&
        (!inputs->path_valid ||
         (inputs->path_point_count < PATH_MIN_VALID_POINTS));

    if(inputs->state == VEHICLE_STATE_STAGE2_REVERSE)
    {
        if(!inputs->path_index_valid || !inputs->tracker.valid)
        {
            if(monitor->tracker_index_loss_count < UINT32_MAX)
            {
                ++monitor->tracker_index_loss_count;
            }
        }
        else
        {
            monitor->tracker_index_loss_count = 0U;
        }
    }
    else
    {
        monitor->tracker_index_loss_count = 0U;
    }
    tracker_lost = !inputs->controlled_stop_in_progress &&
        monitor->tracker_index_loss_count >=
                   monitor->config.tracker_index_loss_limit;
    tracking_error = !inputs->controlled_stop_in_progress &&
        (inputs->state == VEHICLE_STATE_STAGE2_REVERSE) &&
        inputs->tracker.valid &&
        ((fabsf(inputs->tracker.cross_track_error_m) >
          monitor->config.tracker_max_cross_track_error_m) ||
         (fabsf(inputs->tracker.heading_error_rad) >
          monitor->config.tracker_max_heading_error_rad));

    safety_set_condition(monitor, fault_manager,
                         VEHICLE_FAULT_LEFT_ENCODER_JUMP,
                         left_jump, true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager,
                         VEHICLE_FAULT_RIGHT_ENCODER_JUMP,
                         right_jump, true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager,
                         VEHICLE_FAULT_LEFT_ENCODER_STALL,
                         monitor->left_stall_elapsed_s >=
                         monitor->config.motor_stall_timeout_s,
                         true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager,
                         VEHICLE_FAULT_RIGHT_ENCODER_STALL,
                         monitor->right_stall_elapsed_s >=
                         monitor->config.motor_stall_timeout_s,
                         true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager,
                         VEHICLE_FAULT_WHEEL_MISMATCH,
                         monitor->wheel_mismatch_elapsed_s >=
                         monitor->config.wheel_mismatch_timeout_s,
                         false, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager, VEHICLE_FAULT_MT6701_COMM,
                         mt_comm_fault, true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager, VEHICLE_FAULT_MT6701_JUMP,
                         mt_jump, true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager,
                         VEHICLE_FAULT_STEERING_LIMIT,
                         steering_limit, true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager,
                         VEHICLE_FAULT_STEERING_STALL,
                         monitor->steering_stall_elapsed_s >=
                         monitor->config.steering_stall_timeout_s,
                         true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager, VEHICLE_FAULT_IMU_COMM,
                         imu_comm_fault, true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager, VEHICLE_FAULT_IMU_RANGE,
                         imu_range_fault, true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager, VEHICLE_FAULT_IMU_TIMEOUT,
                         imu_timeout, true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager,
                         VEHICLE_FAULT_CONTROL_TIMEOUT,
                         control_timeout, true, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager, VEHICLE_FAULT_PATH_OVERFLOW,
                         path_overflow, false, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager, VEHICLE_FAULT_PATH_INVALID,
                         path_invalid, false, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager,
                         VEHICLE_FAULT_PATH_INDEX_LOST,
                         tracker_lost, false, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager,
                         VEHICLE_FAULT_TRACKING_ERROR,
                         tracking_error, false, inputs->timestamp_us);
    safety_set_condition(monitor, fault_manager, VEHICLE_FAULT_NUMERIC,
                         numeric_fault, true, inputs->timestamp_us);

    if(inputs->left_wheel.valid)
    {
        monitor->previous_left_valid = true;
        monitor->previous_left_speed_mps = inputs->left_wheel.speed_mps;
        monitor->previous_left_timestamp_us = inputs->left_wheel.timestamp_us;
    }
    if(inputs->right_wheel.valid)
    {
        monitor->previous_right_valid = true;
        monitor->previous_right_speed_mps = inputs->right_wheel.speed_mps;
        monitor->previous_right_timestamp_us = inputs->right_wheel.timestamp_us;
    }
    if(inputs->mt6701_communication_ok && inputs->steering.valid)
    {
        monitor->previous_steering_valid = true;
        monitor->previous_steering_count =
            inputs->steering.continuous_count;
        monitor->previous_steering_timestamp_us =
            inputs->steering.timestamp_us;
    }
    monitor->previous_update_timestamp_us = inputs->timestamp_us;
    if(vehicle_fault_has_active(fault_manager))
    {
        monitor->automatic_output_inhibited = true;
    }
}

bool vehicle_safety_manual_reset(VehicleSafetyMonitor *monitor,
                                 VehicleFaultManager *fault_manager,
                                 uint64_t timestamp_us)
{
    bool reset_ok;

    if((monitor == nullptr) || (fault_manager == nullptr))
    {
        return false;
    }
    reset_ok = vehicle_fault_manual_reset(
        fault_manager, fault_manager->active_flags, timestamp_us);
    if(reset_ok)
    {
        monitor->automatic_output_inhibited = !monitor->calibration_valid;
        monitor->left_stall_elapsed_s = 0.0f;
        monitor->right_stall_elapsed_s = 0.0f;
        monitor->wheel_mismatch_elapsed_s = 0.0f;
        monitor->steering_stall_elapsed_s = 0.0f;
        monitor->mt6701_error_count = 0U;
        monitor->tracker_index_loss_count = 0U;
        monitor->monitored_active_flags = 0U;
        monitor->controlled_stop_request_flags = 0U;
    }
    return reset_ok;
}

uint32_t vehicle_safety_get_controlled_stop_request_flags(
    const VehicleSafetyMonitor *monitor)
{
    return (monitor != nullptr)
        ? monitor->controlled_stop_request_flags
        : VEHICLE_FAULT_NONE;
}

bool vehicle_safety_output_is_allowed(const VehicleSafetyMonitor *monitor,
                                      const VehicleFaultManager *fault_manager)
{
    return (monitor != nullptr) && (fault_manager != nullptr) &&
           monitor->calibration_valid &&
           !monitor->automatic_output_inhibited &&
           !vehicle_fault_has_active(fault_manager) &&
           !fault_manager->immediate_stop;
}

void vehicle_safety_gate_actuators(const VehicleSafetyMonitor *monitor,
                                   const VehicleFaultManager *fault_manager,
                                   VehicleActuatorCommand *command)
{
    if(command == nullptr)
    {
        return;
    }
    if(!vehicle_safety_output_is_allowed(monitor, fault_manager))
    {
        command->left_motor_duty = 0.0f;
        command->right_motor_duty = 0.0f;
        command->steering_motor_duty = 0.0f;
        command->immediate_stop = (fault_manager != nullptr)
            ? fault_manager->immediate_stop : true;
    }
}
