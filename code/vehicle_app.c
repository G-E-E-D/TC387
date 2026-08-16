#include "vehicle_app.h"

#include "vehicle_calibration.h"
#include "vehicle_config.h"
#include "vehicle_control.h"
#include "vehicle_diagnostics.h"
#include "vehicle_display.h"
#include "vehicle_encoder.h"
#include "vehicle_fault.h"
#include "vehicle_hal.h"
#include "vehicle_hardware_config.h"
#include "vehicle_imu.h"
#include "vehicle_localization.h"
#include "vehicle_log.h"
#include "vehicle_math.h"
#include "vehicle_mt6701.h"
#include "vehicle_path.h"
#include "vehicle_perception.h"
#include "vehicle_reverse_tracker.h"
#include "vehicle_safety.h"
#include "vehicle_state_machine.h"
#include "vehicle_vision_camera.h"
#include "vision_shared.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define VEHICLE_UART_RX_BUDGET          (32U)
#define VEHICLE_UART_TX_BUDGET          (64U)
#define VEHICLE_FAST_TICK_CATCHUP_LIMIT (8U)
#define VEHICLE_STOPPED_SPEED_MPS       (0.08f)
#define VEHICLE_STOPPED_DUTY            (0.03f)
#define VEHICLE_KEY_DEBOUNCE_US         (50000ULL)

typedef struct
{
    VehicleHalStatus hal_status;
    SteeringCalibrationTables steering_tables;
    VehicleEncoder left_encoder;
    VehicleEncoder right_encoder;
    VehicleMt6701 mt6701;
    VehicleImu imu;
    VehicleLocalization localization;
    VehicleControl control;
    VehicleReverseTracker reverse_tracker;
    VehicleSafetyMonitor safety;
    VehicleFaultManager faults;
    VehicleStateMachine state_machine;
    VehicleDiagnostics diagnostics;
    VehicleLog log;
    VehicleDisplay display;
    VehicleTelemetry telemetry;
    VehicleControlOutput control_output;
    uint16_t diagnostic_previous_left_raw;
    uint16_t diagnostic_previous_right_raw;
    int64_t diagnostic_left_count;
    int64_t diagnostic_right_count;
    uint32_t perception_generation;
    uint32_t controlled_stop_fault_flags;
    uint64_t perception_command_timestamp_us;
    uint64_t stage1_start_timestamp_us;
    uint64_t calibration_test_deadline_us;
    uint64_t controlled_stop_deadline_us;
    uint64_t last_control_timestamp_us;
    uint64_t last_tracker_timestamp_us;
    uint64_t next_estimator_timestamp_us;
    uint64_t next_control_timestamp_us;
    uint64_t next_tracker_timestamp_us;
    uint64_t next_safety_timestamp_us;
    uint64_t next_log_timestamp_us;
    uint64_t next_display_timestamp_us;
    uint64_t last_key_timestamp_us;
    VehicleHalTestMotor calibration_test_motor;
    float calibration_test_duty;
    float target_speed_mps;
    float commanded_speed_mps;
    float target_steering_rad;
    bool calibration_valid;
    bool localization_ready;
    bool mt_communication_ok;
    bool imu_communication_ok;
    bool path_processing_complete;
    bool path_valid;
    bool stage1_stop_pending;
    bool stage1_recording_ended;
    bool calibration_test_active;
    bool calibration_test_output_applied;
    bool controlled_stop_active;
    bool request_calibration_mode;
    bool request_idle;
    bool request_stage1_start;
    bool request_stage2_start;
    bool request_stop;
    bool request_fault_reset;
    bool reverse_finished;
    bool initialized;
} VehicleApp;

static VehicleApp g_app;

static void log_text(const char *text)
{
    (void)vehicle_log_enqueue_text(&g_app.log, text);
}

static void log_format(const char *format, ...)
{
    char line[VEHICLE_COMMAND_BUFFER_SIZE];
    va_list arguments;
    va_start(arguments, format);
    (void)vsnprintf(line, sizeof(line), format, arguments);
    va_end(arguments);
    (void)vehicle_log_enqueue_text(&g_app.log, line);
}

static void telemetry_zero_actuators(void)
{
    memset(&g_app.control_output, 0, sizeof(g_app.control_output));
    g_app.telemetry.actuators.left_motor_duty = 0.0f;
    g_app.telemetry.actuators.right_motor_duty = 0.0f;
    g_app.telemetry.actuators.steering_motor_duty = 0.0f;
    g_app.telemetry.actuators.immediate_stop = false;
}

static void clear_requests(void)
{
    g_app.request_calibration_mode = false;
    g_app.request_idle = false;
    g_app.request_stage1_start = false;
    g_app.request_stage2_start = false;
    g_app.request_stop = false;
    g_app.request_fault_reset = false;
}

static bool vehicle_app_is_stopped(void)
{
    return g_app.telemetry.left_wheel.valid &&
           g_app.telemetry.right_wheel.valid &&
           (fabsf(g_app.telemetry.left_wheel.speed_mps) <=
            VEHICLE_STOPPED_SPEED_MPS) &&
           (fabsf(g_app.telemetry.right_wheel.speed_mps) <=
            VEHICLE_STOPPED_SPEED_MPS) &&
           (fabsf(g_app.commanded_speed_mps) <= VEHICLE_STOPPED_SPEED_MPS) &&
           (fabsf(g_app.telemetry.actuators.left_motor_duty) <=
            VEHICLE_STOPPED_DUTY) &&
           (fabsf(g_app.telemetry.actuators.right_motor_duty) <=
            VEHICLE_STOPPED_DUTY);
}

static bool vehicle_app_commissioning_ready(void)
{
    return g_app.hal_status.motor_outputs_ready &&
           g_app.hal_status.rear_encoders_ready &&
           g_app.hal_status.imu_ready &&
           g_app.hal_status.timer_ready;
}

static void request_controlled_stop(uint32_t fault_flags, uint64_t now_us)
{
    if(fault_flags == VEHICLE_FAULT_NONE)
    {
        return;
    }
    if(!vehicle_state_is_automatic(g_app.state_machine.state))
    {
        vehicle_fault_raise(&g_app.faults, fault_flags, false, now_us);
        return;
    }
    vehicle_fault_latch(&g_app.faults, fault_flags, now_us);
    g_app.controlled_stop_fault_flags |= fault_flags;
    if(!g_app.controlled_stop_active)
    {
        g_app.controlled_stop_active = true;
        g_app.controlled_stop_deadline_us = now_us +
            VEHICLE_CONTROLLED_STOP_TIMEOUT_US;
    }
    g_app.target_speed_mps = 0.0f;
    g_app.target_steering_rad = 0.0f;
    if(g_app.state_machine.state == VEHICLE_STATE_STAGE1_RECORD)
    {
        g_app.stage1_stop_pending = true;
    }
}

static void promote_controlled_stop(bool immediate_stop, uint64_t now_us)
{
    uint32_t fault_flags = g_app.controlled_stop_fault_flags;

    if(fault_flags == VEHICLE_FAULT_NONE)
    {
        g_app.controlled_stop_active = false;
        return;
    }
    if(immediate_stop)
    {
        fault_flags |= VEHICLE_FAULT_CONTROL_TIMEOUT;
    }
    g_app.controlled_stop_fault_flags = VEHICLE_FAULT_NONE;
    g_app.controlled_stop_active = false;
    vehicle_fault_raise(&g_app.faults, fault_flags, immediate_stop, now_us);
}

static void update_controlled_stop(uint64_t now_us)
{
    if(!g_app.controlled_stop_active)
    {
        return;
    }
    g_app.target_speed_mps = 0.0f;
    g_app.target_steering_rad = 0.0f;
    if(vehicle_app_is_stopped())
    {
        promote_controlled_stop(false, now_us);
    }
    else if(now_us >= g_app.controlled_stop_deadline_us)
    {
        promote_controlled_stop(true, now_us);
    }
}

static void end_stage1_recording(void)
{
    VehiclePathResult result;
    if(g_app.stage1_recording_ended)
    {
        return;
    }
    result = vehicle_path_end_recording();
    g_app.stage1_recording_ended = true;
    if(vehicle_path_result_is_error(result))
    {
        request_controlled_stop(VEHICLE_FAULT_PATH_INVALID,
                                g_app.telemetry.timestamp_us);
    }
}

static void handle_state_entry(uint64_t now_us)
{
    VehicleState entered;
    VehiclePathSample sample;
    VehiclePathResult path_result;
    VehicleReverseTrackerStatus tracker_status;

    if(!vehicle_state_machine_take_entry(&g_app.state_machine, &entered))
    {
        return;
    }
    g_app.telemetry.state = entered;
    g_app.commanded_speed_mps = 0.0f;
    if(!vehicle_state_is_automatic(entered))
    {
        g_app.target_speed_mps = 0.0f;
        g_app.target_steering_rad = 0.0f;
    }
    log_format("# STATE,%s,%llu\r\n", vehicle_state_name(entered),
               (unsigned long long)now_us);

    switch(entered)
    {
        case VEHICLE_STATE_BOOT:
            vehicle_hal_force_safe_outputs();
            break;
        case VEHICLE_STATE_SENSOR_CALIBRATION:
            vehicle_hal_force_safe_outputs();
            vehicle_imu_restart_calibration(&g_app.imu);
            g_app.localization_ready = false;
            break;
        case VEHICLE_STATE_IDLE:
            vehicle_hal_force_safe_outputs();
            telemetry_zero_actuators();
            vehicle_control_reset(&g_app.control);
            g_app.target_speed_mps = 0.0f;
            g_app.target_steering_rad = 0.0f;
            g_app.calibration_test_active = false;
            g_app.calibration_test_output_applied = false;
            break;
        case VEHICLE_STATE_STAGE1_RECORD:
            vehicle_control_reset(&g_app.control);
            g_app.target_speed_mps = 0.0f;
            g_app.target_steering_rad = 0.0f;
            stage1_invalidate_external_command();
            vehicle_path_reset();
            sample.x_m = g_app.telemetry.pose.x_m;
            sample.y_m = g_app.telemetry.pose.y_m;
            sample.yaw_rad = g_app.telemetry.pose.yaw_rad;
            sample.speed_mps = g_app.telemetry.pose.vehicle_speed_mps;
            path_result = vehicle_path_start_recording(&sample);
            g_app.stage1_start_timestamp_us = now_us;
            g_app.stage1_stop_pending = false;
            g_app.stage1_recording_ended = false;
            g_app.path_processing_complete = false;
            g_app.path_valid = false;
            g_app.perception_generation = perception_get_command_generation();
            g_app.perception_command_timestamp_us = now_us;
            if(vehicle_path_result_is_error(path_result))
            {
                request_controlled_stop(VEHICLE_FAULT_PATH_INVALID, now_us);
            }
            break;
        case VEHICLE_STATE_STAGE1_FINISHED:
            vehicle_hal_force_safe_outputs();
            telemetry_zero_actuators();
            vehicle_control_reset(&g_app.control);
            if(g_app.state_machine.previous_state ==
               VEHICLE_STATE_STAGE1_RECORD)
            {
                end_stage1_recording();
                path_result = vehicle_path_preprocess();
                g_app.path_processing_complete = true;
                g_app.path_valid = (path_result == VEHICLE_PATH_RESULT_OK) &&
                                   vehicle_path_is_valid();
                if(!g_app.path_valid)
                {
                    vehicle_fault_raise(&g_app.faults,
                        (path_result == VEHICLE_PATH_RESULT_OVERFLOW)
                            ? VEHICLE_FAULT_PATH_OVERFLOW
                            : VEHICLE_FAULT_PATH_INVALID,
                                        false, now_us);
                }
            }
            break;
        case VEHICLE_STATE_STAGE2_REVERSE:
            vehicle_control_reset(&g_app.control);
            g_app.target_speed_mps = 0.0f;
            g_app.target_steering_rad = 0.0f;
            tracker_status = vehicle_reverse_tracker_init(
                &g_app.reverse_tracker, NULL, vehicle_path_get_points(),
                vehicle_path_get_count());
            g_app.reverse_finished = false;
            g_app.last_tracker_timestamp_us = now_us;
            if(vehicle_reverse_tracker_status_is_error(tracker_status))
            {
                request_controlled_stop(VEHICLE_FAULT_PATH_INVALID, now_us);
            }
            break;
        case VEHICLE_STATE_FINISHED:
            vehicle_hal_force_safe_outputs();
            telemetry_zero_actuators();
            vehicle_control_reset(&g_app.control);
            break;
        case VEHICLE_STATE_FAULT:
            promote_controlled_stop(false, now_us);
            vehicle_hal_force_safe_outputs();
            telemetry_zero_actuators();
            vehicle_control_reset(&g_app.control);
            g_app.calibration_test_active = false;
            g_app.calibration_test_output_applied = false;
            log_format("# FAULT,active=%08lX,latched=%08lX\r\n",
                       (unsigned long)g_app.faults.active_flags,
                       (unsigned long)g_app.faults.latched_flags);
            break;
        case VEHICLE_STATE_CALIBRATION_MODE:
            vehicle_hal_force_safe_outputs();
            telemetry_zero_actuators();
            vehicle_control_reset(&g_app.control);
            g_app.calibration_test_active = false;
            g_app.calibration_test_output_applied = false;
            break;
        default:
            vehicle_fault_raise(&g_app.faults, VEHICLE_FAULT_NUMERIC,
                                true, now_us);
            break;
    }
}

static void update_diagnostic_encoder(WheelEncoderSample *sample,
                                      uint16_t raw, uint16_t *previous_raw,
                                      int64_t *continuous_count,
                                      uint64_t now_us)
{
    int32_t delta = vehicle_encoder_delta16(raw, *previous_raw);
    *previous_raw = raw;
    *continuous_count += (int64_t)delta;
    sample->timestamp_us = now_us;
    sample->count = *continuous_count;
    sample->delta_count = delta;
    sample->speed_mps = 0.0f;
    sample->valid = false;
}

static void update_steering_angle_from_tables(void)
{
    float from_left;
    float from_right;
    bool left_ok;
    bool right_ok;
    if(!g_app.calibration_valid || !g_app.telemetry.steering.valid)
    {
        return;
    }
    left_ok = vehicle_steering_table_count_to_angle(
        g_app.steering_tables.from_left,
        g_app.steering_tables.from_left_count,
        g_app.telemetry.steering.relative_count, &from_left);
    right_ok = vehicle_steering_table_count_to_angle(
        g_app.steering_tables.from_right,
        g_app.steering_tables.from_right_count,
        g_app.telemetry.steering.relative_count, &from_right);
    if(left_ok && right_ok)
    {
        g_app.telemetry.steering.angle_rad = 0.5f * (from_left + from_right);
    }
    else
    {
        g_app.telemetry.steering.valid = false;
    }
}

static void update_stage1_recording(uint64_t now_us)
{
    VehiclePathSample sample;
    VehiclePathInfo path_info;
    VehiclePathResult result;
    float elapsed_s;
    float speed_mps;
    float deceleration_mps2;
    float braking_distance_m;
    float braking_time_s;
    if(g_app.stage1_recording_ended)
    {
        return;
    }
    if((now_us < g_app.stage1_start_timestamp_us) ||
       ((now_us - g_app.stage1_start_timestamp_us) >=
        (uint64_t)(RECORD_MAX_TIME_S * 1000000.0f)))
    {
        g_app.stage1_stop_pending = true;
        end_stage1_recording();
        return;
    }
    if(!g_app.telemetry.pose.valid)
    {
        return;
    }
    sample.x_m = g_app.telemetry.pose.x_m;
    sample.y_m = g_app.telemetry.pose.y_m;
    sample.yaw_rad = g_app.telemetry.pose.yaw_rad;
    sample.speed_mps = g_app.telemetry.pose.vehicle_speed_mps;
    result = vehicle_path_record_sample(&sample);
    if(result == VEHICLE_PATH_RESULT_DISTANCE_LIMIT)
    {
        g_app.stage1_stop_pending = true;
        g_app.stage1_recording_ended = true;
    }
    else if(vehicle_path_result_is_error(result))
    {
        g_app.stage1_recording_ended = true;
        request_controlled_stop(
            (result == VEHICLE_PATH_RESULT_OVERFLOW)
                ? VEHICLE_FAULT_PATH_OVERFLOW
                : VEHICLE_FAULT_PATH_INVALID,
            now_us);
    }
    else if(!g_app.stage1_stop_pending)
    {
        path_info = vehicle_path_get_info();
        elapsed_s = (float)(now_us - g_app.stage1_start_timestamp_us) *
                    1.0e-6f;
        speed_mps = fabsf(g_app.telemetry.pose.vehicle_speed_mps);
        deceleration_mps2 = g_vehicle_calibration.max_deceleration_mps2;
        if(g_app.calibration_valid && (deceleration_mps2 > 0.0f))
        {
            braking_distance_m =
                (speed_mps * speed_mps) / (2.0f * deceleration_mps2) +
                STAGE1_STOP_DISTANCE_MARGIN_M;
            braking_time_s = speed_mps / deceleration_mps2 +
                             STAGE1_STOP_TIME_MARGIN_S;
            if(((path_info.length_m + braking_distance_m) >=
                RECORD_MAX_DISTANCE_M) ||
               ((elapsed_s + braking_time_s) >= RECORD_MAX_TIME_S))
            {
                g_app.stage1_stop_pending = true;
            }
        }
    }
}

static void estimator_task(uint64_t now_us)
{
    uint16_t left_raw;
    uint16_t right_raw;
    uint16_t mt_ab_raw;
    uint32_t mt_frame = 0U;
    VehicleHalImuRaw imu_raw;
    bool steering_ok;

    (void)mt_ab_raw;
    (void)mt_frame;

    vehicle_hal_get_rear_encoder_raw(&left_raw, &right_raw);
    if(g_app.calibration_valid)
    {
        (void)vehicle_encoder_update(&g_app.left_encoder, left_raw, now_us,
                                     &g_app.telemetry.left_wheel);
        (void)vehicle_encoder_update(&g_app.right_encoder, right_raw, now_us,
                                     &g_app.telemetry.right_wheel);
    }
    else
    {
        update_diagnostic_encoder(&g_app.telemetry.left_wheel, left_raw,
                                  &g_app.diagnostic_previous_left_raw,
                                  &g_app.diagnostic_left_count, now_us);
        update_diagnostic_encoder(&g_app.telemetry.right_wheel, right_raw,
                                  &g_app.diagnostic_previous_right_raw,
                                  &g_app.diagnostic_right_count, now_us);
    }

#if VEHICLE_MT6701_INTERFACE == VEHICLE_MT6701_INTERFACE_SSI
    g_app.mt_communication_ok = vehicle_hal_read_mt6701_ssi(&mt_frame);
    steering_ok = vehicle_mt6701_update_ssi(
        &g_app.mt6701, mt_frame, now_us, g_app.mt_communication_ok,
        &g_app.telemetry.steering);
#elif VEHICLE_MT6701_INTERFACE == VEHICLE_MT6701_INTERFACE_AB
    g_app.mt_communication_ok = vehicle_hal_get_mt6701_ab_raw(&mt_ab_raw);
    steering_ok = vehicle_mt6701_update_angle(
        &g_app.mt6701, mt_ab_raw, 0U, now_us, g_app.mt_communication_ok,
        &g_app.telemetry.steering);
#else
    mt_ab_raw = 0U;
    g_app.mt_communication_ok = false;
    steering_ok = vehicle_mt6701_update_angle(
        &g_app.mt6701, mt_ab_raw, 0U, now_us, false,
        &g_app.telemetry.steering);
#endif
    (void)steering_ok;
    update_steering_angle_from_tables();

    memset(&imu_raw, 0, sizeof(imu_raw));
    g_app.imu_communication_ok = vehicle_hal_read_imu(&imu_raw);
    (void)vehicle_imu_update(
        &g_app.imu, now_us, imu_raw.acceleration, imu_raw.angular_rate,
        imu_raw.magnetic_field, imu_raw.temperature,
        imu_raw.accel_gyro_communication_ok,
        imu_raw.magnetometer_communication_ok, &g_app.telemetry.imu);

    if(vehicle_imu_is_calibrated(&g_app.imu) &&
       !g_app.localization_ready && g_app.calibration_valid)
    {
        g_app.localization_ready = vehicle_localization_init(
            &g_app.localization,
            g_vehicle_calibration.left_meter_per_count,
            g_vehicle_calibration.right_meter_per_count,
            g_app.imu.gyro_bias_radps[2]);
        if(g_app.localization_ready)
        {
            vehicle_localization_reset(&g_app.localization, 0.0f, 0.0f,
                                       0.0f, g_app.imu.gyro_bias_radps[2]);
        }
    }
    if(g_app.localization_ready && vehicle_imu_is_calibrated(&g_app.imu))
    {
        (void)vehicle_localization_update(
            &g_app.localization, now_us,
            &g_app.telemetry.left_wheel, &g_app.telemetry.right_wheel,
            &g_app.telemetry.imu, g_app.telemetry.steering.angle_rad,
            g_app.telemetry.steering.valid, &g_app.telemetry.pose);
    }
    else
    {
        g_app.telemetry.pose.timestamp_us = now_us;
        g_app.telemetry.pose.x_m = 0.0f;
        g_app.telemetry.pose.y_m = 0.0f;
        g_app.telemetry.pose.yaw_rad = 0.0f;
        g_app.telemetry.pose.vehicle_speed_mps = 0.0f;
        g_app.telemetry.pose.gyro_z_bias_radps = g_app.imu.gyro_bias_radps[2];
        g_app.telemetry.pose.valid = false;
    }
    if(g_app.state_machine.state == VEHICLE_STATE_STAGE1_RECORD)
    {
        update_stage1_recording(now_us);
    }
}

static void tracker_task(uint64_t now_us)
{
    uint32_t generation;
    Stage1ControlCommand stage1_command;
    VehicleReverseTrackerInput input;
    VehicleReverseTrackerStatus status;
    float dt_s;

    if(g_app.state_machine.state == VEHICLE_STATE_STAGE1_RECORD)
    {
        perception_update();
        generation = perception_get_command_generation();
        if(generation != g_app.perception_generation)
        {
            g_app.perception_generation = generation;
            g_app.perception_command_timestamp_us = now_us;
        }
        if(!g_app.stage1_stop_pending &&
           perception_get_stage1_command(&stage1_command) &&
           (now_us >= g_app.perception_command_timestamp_us) &&
           ((now_us - g_app.perception_command_timestamp_us) <=
            VEHICLE_COMMAND_TIMEOUT_US))
        {
            g_app.target_speed_mps = vehicle_clampf(
                stage1_command.target_speed_mps, 0.0f, STAGE1_MAX_SPEED_MPS);
            g_app.target_steering_rad = vehicle_clampf(
                stage1_command.target_steering_rad,
                -STAGE1_MAX_STEERING_RAD, STAGE1_MAX_STEERING_RAD);
        }
        else
        {
            g_app.target_speed_mps = 0.0f;
            g_app.target_steering_rad = 0.0f;
        }
        return;
    }
    if(g_app.state_machine.state != VEHICLE_STATE_STAGE2_REVERSE)
    {
        g_app.target_speed_mps = 0.0f;
        g_app.target_steering_rad = 0.0f;
        return;
    }
    if(g_app.controlled_stop_active)
    {
        g_app.target_speed_mps = 0.0f;
        g_app.target_steering_rad = 0.0f;
        return;
    }

    dt_s = (g_app.last_tracker_timestamp_us == 0U)
        ? ((float)VEHICLE_TRACKER_PERIOD_US * 1.0e-6f)
        : (float)(now_us - g_app.last_tracker_timestamp_us) * 1.0e-6f;
    g_app.last_tracker_timestamp_us = now_us;
    input.x_m = g_app.telemetry.pose.x_m;
    input.y_m = g_app.telemetry.pose.y_m;
    input.body_yaw_rad = g_app.telemetry.pose.yaw_rad;
    input.speed_mps = g_app.telemetry.pose.vehicle_speed_mps;
    input.dt_s = dt_s;
    input.localization_valid = g_app.telemetry.pose.valid;
    status = vehicle_reverse_tracker_update(&g_app.reverse_tracker, &input,
                                            &g_app.telemetry.tracker);
    g_app.target_speed_mps = vehicle_clampf(
        g_app.telemetry.tracker.target_speed_mps,
        -REPLAY_MAX_SPEED_MPS, 0.0f);
    g_app.target_steering_rad = vehicle_clampf(
        g_app.telemetry.tracker.target_steering_rad,
        -REVERSE_MAX_STEERING_RAD, REVERSE_MAX_STEERING_RAD);
    g_app.reverse_finished = (status == VEHICLE_REVERSE_TRACKER_FINISHED) &&
                             g_app.telemetry.tracker.finished;
    if(status == VEHICLE_REVERSE_TRACKER_PATH_INVALID)
    {
        request_controlled_stop(VEHICLE_FAULT_PATH_INVALID, now_us);
    }
    else if(status == VEHICLE_REVERSE_TRACKER_INDEX_LOST)
    {
        request_controlled_stop(VEHICLE_FAULT_PATH_INDEX_LOST, now_us);
    }
    else if(status == VEHICLE_REVERSE_TRACKER_TRACKING_ERROR)
    {
        request_controlled_stop(VEHICLE_FAULT_TRACKING_ERROR, now_us);
    }
    else if(status == VEHICLE_REVERSE_TRACKER_LOCALIZATION_INVALID)
    {
        request_controlled_stop(VEHICLE_FAULT_TRACKING_ERROR, now_us);
    }
    else if(status == VEHICLE_REVERSE_TRACKER_NUMERIC_ERROR)
    {
        vehicle_fault_raise(&g_app.faults, VEHICLE_FAULT_NUMERIC,
                            true, now_us);
    }
}

static void control_task(uint64_t now_us)
{
    VehicleControlInput input;
    float dt_s;
    bool automatic = vehicle_state_is_automatic(g_app.state_machine.state);
    dt_s = (g_app.last_control_timestamp_us == 0U)
        ? ((float)VEHICLE_CONTROL_PERIOD_US * 1.0e-6f)
        : (float)(now_us - g_app.last_control_timestamp_us) * 1.0e-6f;
    g_app.last_control_timestamp_us = now_us;
    dt_s = vehicle_clampf(dt_s, LOCALIZATION_MIN_DT_S,
                          LOCALIZATION_MAX_DT_S);

    if(automatic)
    {
        g_app.commanded_speed_mps = vehicle_rate_limit(
            g_app.target_speed_mps, g_app.commanded_speed_mps,
            g_vehicle_calibration.max_acceleration_mps2,
            g_vehicle_calibration.max_deceleration_mps2, dt_s);
        input.target_center_speed_mps = g_app.commanded_speed_mps;
        input.target_steering_rad = g_app.target_steering_rad;
        input.left_speed_mps = g_app.telemetry.left_wheel.speed_mps;
        input.right_speed_mps = g_app.telemetry.right_wheel.speed_mps;
        input.steering_continuous_count =
            g_app.telemetry.steering.relative_count;
        input.dt_s = dt_s;
        vehicle_control_update(&g_app.control, &input, &g_app.control_output);
        g_app.telemetry.actuators = g_app.control_output.actuators;
        if(g_app.control_output.steering.valid)
        {
            g_app.telemetry.steering.angle_rad =
                g_app.control_output.steering.measured_angle_rad;
        }
        if(g_app.control_output.steering.stalled)
        {
            vehicle_fault_raise(&g_app.faults,
                                VEHICLE_FAULT_STEERING_STALL,
                                true, now_us);
        }
    }
    else
    {
        g_app.commanded_speed_mps = 0.0f;
        telemetry_zero_actuators();
    }
    g_app.telemetry.target_speed_mps = g_app.commanded_speed_mps;
    g_app.telemetry.target_steering_rad = g_app.target_steering_rad;

    if(g_app.stage1_stop_pending &&
       (g_app.state_machine.state == VEHICLE_STATE_STAGE1_RECORD) &&
       vehicle_app_is_stopped())
    {
        end_stage1_recording();
    }
}

static void safety_task(uint64_t now_us)
{
    VehicleSafetyInputs inputs;
    VehiclePathInfo path_info = vehicle_path_get_info();
    uint32_t controlled_stop_flags;
    float numeric_values[9];
    bool tracker_index_valid;

    numeric_values[0] = g_app.telemetry.pose.x_m;
    numeric_values[1] = g_app.telemetry.pose.y_m;
    numeric_values[2] = g_app.telemetry.pose.yaw_rad;
    numeric_values[3] = g_app.telemetry.pose.vehicle_speed_mps;
    numeric_values[4] = g_app.target_speed_mps;
    numeric_values[5] = g_app.target_steering_rad;
    numeric_values[6] = g_app.telemetry.steering.angle_rad;
    numeric_values[7] = g_app.commanded_speed_mps;
    numeric_values[8] = path_info.length_m;
    tracker_index_valid =
        (vehicle_reverse_tracker_get_status(&g_app.reverse_tracker) ==
         VEHICLE_REVERSE_TRACKER_ACTIVE) ||
        (vehicle_reverse_tracker_get_status(&g_app.reverse_tracker) ==
         VEHICLE_REVERSE_TRACKER_FINISHED);

    memset(&inputs, 0, sizeof(inputs));
    inputs.timestamp_us = now_us;
    inputs.state = g_app.state_machine.state;
    inputs.left_wheel = g_app.telemetry.left_wheel;
    inputs.right_wheel = g_app.telemetry.right_wheel;
    inputs.left_target_speed_mps =
        g_app.control_output.drive.left_target_speed_mps;
    inputs.right_target_speed_mps =
        g_app.control_output.drive.right_target_speed_mps;
    inputs.left_motor_duty = g_app.telemetry.actuators.left_motor_duty;
    inputs.right_motor_duty = g_app.telemetry.actuators.right_motor_duty;
    inputs.mt6701_communication_ok = g_app.mt_communication_ok;
    inputs.steering = g_app.telemetry.steering;
    inputs.steering_motor_duty =
        g_app.telemetry.actuators.steering_motor_duty;
    inputs.imu_communication_ok = g_app.imu_communication_ok;
    inputs.imu = g_app.telemetry.imu;
    inputs.pose = g_app.telemetry.pose;
    inputs.last_control_timestamp_us = g_app.last_control_timestamp_us;
    inputs.path_overflow = path_info.overflowed;
    inputs.path_valid = g_app.path_valid;
    inputs.path_point_count = path_info.point_count;
    inputs.path_capacity = PATH_MAX_POINTS;
    inputs.path_index_valid = tracker_index_valid;
    inputs.tracker = g_app.telemetry.tracker;
    inputs.controlled_stop_in_progress = g_app.controlled_stop_active;
    inputs.numeric_valid = true;
    inputs.additional_numeric_values = numeric_values;
    inputs.additional_numeric_value_count =
        sizeof(numeric_values) / sizeof(numeric_values[0]);
    vehicle_safety_update(&g_app.safety, &g_app.faults, &inputs);
    controlled_stop_flags =
        vehicle_safety_get_controlled_stop_request_flags(&g_app.safety);
    if(controlled_stop_flags != VEHICLE_FAULT_NONE)
    {
        request_controlled_stop(controlled_stop_flags, now_us);
    }

    if(vehicle_hal_control_watchdog_expired())
    {
        vehicle_fault_raise(&g_app.faults, VEHICLE_FAULT_CONTROL_TIMEOUT,
                            true, now_us);
    }
}

static void print_configuration(void)
{
    size_t index;
    log_format("# CONFIG,CALIBRATION_VALID=%u,runtime_valid=%u\r\n",
               (unsigned int)CALIBRATION_VALID,
               g_app.calibration_valid ? 1U : 0U);
    log_format("# GEOMETRY,wheelbase=%.3f,front_track=%.3f,rear_track=%.3f\r\n",
               (double)VEHICLE_WHEELBASE_M, (double)VEHICLE_FRONT_TRACK_M,
               (double)VEHICLE_REAR_TRACK_M);
    log_format("# LIMITS,record_m=%.1f,record_s=%.1f,replay_mps=%.1f\r\n",
               (double)RECORD_MAX_DISTANCE_M, (double)RECORD_MAX_TIME_S,
               (double)REPLAY_MAX_SPEED_MPS);
    log_format("# ENCODER,L sign=%d,m_per_count=%.9g,cpr=%ld\r\n",
               g_vehicle_calibration.left_encoder_forward_sign,
               (double)g_vehicle_calibration.left_meter_per_count,
               (long)g_vehicle_calibration.left_counts_per_wheel_rev);
    log_format("# ENCODER,R sign=%d,m_per_count=%.9g,cpr=%ld\r\n",
               g_vehicle_calibration.right_encoder_forward_sign,
               (double)g_vehicle_calibration.right_meter_per_count,
               (long)g_vehicle_calibration.right_counts_per_wheel_rev);
    log_format("# MOTOR,L dir=%d,start_f=%.4f,start_r=%.4f\r\n",
               g_vehicle_calibration.left_motor_forward_direction,
               (double)g_vehicle_calibration.left_motor_start_duty_forward,
               (double)g_vehicle_calibration.left_motor_start_duty_reverse);
    log_format("# MOTOR,R dir=%d,start_f=%.4f,start_r=%.4f\r\n",
               g_vehicle_calibration.right_motor_forward_direction,
               (double)g_vehicle_calibration.right_motor_start_duty_forward,
               (double)g_vehicle_calibration.right_motor_start_duty_reverse);
    log_format("# STEER,left_dir=%d,start_l=%.4f,start_r=%.4f,limits=%lld:%lld\r\n",
               g_vehicle_calibration.steering_left_direction,
               (double)g_vehicle_calibration.steering_start_duty_left,
               (double)g_vehicle_calibration.steering_start_duty_right,
               (long long)g_vehicle_calibration.steering_left_soft_limit_count,
               (long long)g_vehicle_calibration.steering_right_soft_limit_count);
    log_format("# IMU,pos=%.4f:%.4f:%.4f,map=%d:%d:%d,sign=%d:%d:%d\r\n",
               (double)g_vehicle_calibration.imu_position_x_m,
               (double)g_vehicle_calibration.imu_position_y_m,
               (double)g_vehicle_calibration.imu_position_z_m,
               g_vehicle_calibration.imu_axis_map[0],
               g_vehicle_calibration.imu_axis_map[1],
               g_vehicle_calibration.imu_axis_map[2],
               g_vehicle_calibration.imu_axis_sign[0],
               g_vehicle_calibration.imu_axis_sign[1],
               g_vehicle_calibration.imu_axis_sign[2]);
    log_format("# DYNAMICS,accel=%.3f,decel=%.3f,lateral=%.3f\r\n",
               (double)g_vehicle_calibration.max_acceleration_mps2,
               (double)g_vehicle_calibration.max_deceleration_mps2,
               (double)g_vehicle_calibration.max_lateral_acceleration_mps2);
    for(index = 0U; index < g_app.steering_tables.from_left_count; ++index)
    {
        log_format("# STEER_TABLE,L,%u,%lld,%.7f\r\n", (unsigned int)index,
                   (long long)g_app.steering_tables.from_left[index].continuous_count,
                   (double)g_app.steering_tables.from_left[index].equivalent_steering_angle_rad);
    }
    for(index = 0U; index < g_app.steering_tables.from_right_count; ++index)
    {
        log_format("# STEER_TABLE,R,%u,%lld,%.7f\r\n", (unsigned int)index,
                   (long long)g_app.steering_tables.from_right[index].continuous_count,
                   (double)g_app.steering_tables.from_right[index].equivalent_steering_angle_rad);
    }
    if(!g_app.calibration_valid)
    {
        log_text("# MEASURE_REQUIRED: encoder signs/scales/CPR; motor directions/start duties; steering direction/start duties/soft limits/two approach tables; IMU pose/axis map/sign; acceleration/deceleration/lateral limits\r\n");
    }
    log_text("# MOTOR test sign is raw DIR level: positive=HIGH, negative=LOW, max |0.12|, timeout 2 s\r\n");
}

static void handle_diagnostic_request(const VehicleDiagnosticRequest *request,
                                      uint64_t now_us)
{
    if(request == NULL)
    {
        return;
    }
    switch(request->action)
    {
        case VEHICLE_DIAG_LOG_START:
            vehicle_log_set_csv_enabled(&g_app.log, true);
            break;
        case VEHICLE_DIAG_LOG_STOP:
            vehicle_log_set_csv_enabled(&g_app.log, false);
            break;
        case VEHICLE_DIAG_ENCODER_ZERO:
            if((g_app.state_machine.state == VEHICLE_STATE_CALIBRATION_MODE) &&
               !g_app.calibration_test_active &&
               (fabsf(g_app.telemetry.pose.vehicle_speed_mps) <=
                VEHICLE_STOPPED_SPEED_MPS))
            {
                vehicle_hal_zero_rear_encoder_raw();
                vehicle_encoder_reset(&g_app.left_encoder, 0);
                vehicle_encoder_reset(&g_app.right_encoder, 0);
                g_app.diagnostic_previous_left_raw = 0U;
                g_app.diagnostic_previous_right_raw = 0U;
                g_app.diagnostic_left_count = 0;
                g_app.diagnostic_right_count = 0;
            }
            else
            {
                log_text("# DENIED,ENC ZERO requires stopped CALIBRATION_MODE\r\n");
            }
            break;
        case VEHICLE_DIAG_STEERING_ZERO:
            if((g_app.state_machine.state != VEHICLE_STATE_CALIBRATION_MODE) ||
               g_app.calibration_test_active ||
               (fabsf(g_app.telemetry.pose.vehicle_speed_mps) >
                VEHICLE_STOPPED_SPEED_MPS))
            {
                log_text("# DENIED,STEER ZERO requires stopped CALIBRATION_MODE\r\n");
            }
            else if(!vehicle_mt6701_set_relative_zero(&g_app.mt6701))
            {
                log_text("# DENIED,steering sample not valid\r\n");
            }
            break;
        case VEHICLE_DIAG_STEERING_LIMIT_LEFT:
            log_format("# STEER_LIMIT_CANDIDATE,left,%lld\r\n",
                       (long long)g_app.telemetry.steering.relative_count);
            break;
        case VEHICLE_DIAG_STEERING_LIMIT_RIGHT:
            log_format("# STEER_LIMIT_CANDIDATE,right,%lld\r\n",
                       (long long)g_app.telemetry.steering.relative_count);
            break;
        case VEHICLE_DIAG_TEST_LEFT_MOTOR:
        case VEHICLE_DIAG_TEST_RIGHT_MOTOR:
        case VEHICLE_DIAG_TEST_STEERING_MOTOR:
            if((g_app.state_machine.state == VEHICLE_STATE_CALIBRATION_MODE) &&
               !vehicle_fault_has_active(&g_app.faults) &&
               g_app.hal_status.motor_outputs_ready)
            {
                g_app.calibration_test_motor =
                    (request->action == VEHICLE_DIAG_TEST_LEFT_MOTOR)
                        ? VEHICLE_HAL_TEST_LEFT_MOTOR
                        : ((request->action == VEHICLE_DIAG_TEST_RIGHT_MOTOR)
                           ? VEHICLE_HAL_TEST_RIGHT_MOTOR
                           : VEHICLE_HAL_TEST_STEERING_MOTOR);
                g_app.calibration_test_duty = request->signed_duty;
                g_app.calibration_test_deadline_us = now_us +
                    (uint64_t)(CALIBRATION_TEST_MAX_TIME_S * 1000000.0f);
                g_app.calibration_test_active = true;
                g_app.calibration_test_output_applied = false;
            }
            else
            {
                log_text("# DENIED,motor test requires CALIBRATION_MODE and no active fault\r\n");
            }
            break;
        case VEHICLE_DIAG_STAGE1_START:
            if(g_app.calibration_valid &&
               g_app.hal_status.critical_ready &&
               (g_app.state_machine.state == VEHICLE_STATE_IDLE ||
                g_app.state_machine.state == VEHICLE_STATE_CALIBRATION_MODE))
            {
                g_app.request_stage1_start = true;
            }
            else
            {
                log_text("# DENIED,stage1 requires all drivers, valid calibration and IDLE/CAL mode\r\n");
            }
            break;
        case VEHICLE_DIAG_STAGE1_DRIVE:
            if(g_app.calibration_valid && g_app.hal_status.critical_ready &&
               (g_app.state_machine.state == VEHICLE_STATE_STAGE1_RECORD) &&
               !g_app.stage1_stop_pending &&
               !g_app.controlled_stop_active &&
               vehicle_safety_output_is_allowed(&g_app.safety,
                                                &g_app.faults))
            {
                Stage1ControlCommand command;

                command.target_speed_mps = vehicle_clampf(
                    request->target_speed_mps, 0.0f,
                    STAGE1_DIAGNOSTIC_MAX_SPEED_MPS);
                command.target_steering_rad = vehicle_clampf(
                    request->target_steering_rad,
                    -STAGE1_DIAGNOSTIC_MAX_STEERING_RAD,
                    STAGE1_DIAGNOSTIC_MAX_STEERING_RAD);
                command.valid = true;
                stage1_set_external_command(&command);
            }
            else
            {
                stage1_invalidate_external_command();
                log_text("# DENIED,DRIVE requires active safe STAGE1_RECORD\r\n");
            }
            break;
        case VEHICLE_DIAG_STAGE1_STOP:
            if(g_app.state_machine.state == VEHICLE_STATE_STAGE1_RECORD)
            {
                g_app.stage1_stop_pending = true;
            }
            break;
        case VEHICLE_DIAG_STAGE2_START:
            if(g_app.calibration_valid && g_app.path_valid &&
               (g_app.state_machine.state == VEHICLE_STATE_STAGE1_FINISHED))
            {
                g_app.request_stage2_start = true;
            }
            else
            {
                log_text("# DENIED,stage2 requires valid processed path/calibration\r\n");
            }
            break;
        case VEHICLE_DIAG_STOP:
            g_app.calibration_test_active = false;
            g_app.calibration_test_output_applied = false;
            if(g_app.state_machine.state == VEHICLE_STATE_STAGE1_RECORD)
            {
                g_app.stage1_stop_pending = true;
            }
            else
            {
                g_app.request_stop = true;
                vehicle_hal_force_safe_outputs();
            }
            break;
        case VEHICLE_DIAG_PRINT_CONFIG:
            print_configuration();
            break;
        case VEHICLE_DIAG_FAULT_RESET:
            if((g_app.state_machine.state != VEHICLE_STATE_FAULT) ||
               !vehicle_app_is_stopped())
            {
                log_text("# DENIED,fault reset requires stopped FAULT state\r\n");
                break;
            }
            g_app.controlled_stop_active = false;
            g_app.controlled_stop_fault_flags = VEHICLE_FAULT_NONE;
            vehicle_fault_set_condition(
                &g_app.faults,
                VEHICLE_FAULT_PATH_OVERFLOW |
                VEHICLE_FAULT_PATH_INVALID |
                VEHICLE_FAULT_PATH_INDEX_LOST |
                VEHICLE_FAULT_TRACKING_ERROR |
                VEHICLE_FAULT_WHEEL_MISMATCH |
                VEHICLE_FAULT_STEERING_STALL |
                VEHICLE_FAULT_NUMERIC,
                false, false, now_us);
            if(vehicle_hal_clear_control_watchdog_fault())
            {
                vehicle_fault_set_condition(
                    &g_app.faults, VEHICLE_FAULT_CONTROL_TIMEOUT,
                    false, true, now_us);
            }
            if(vehicle_safety_manual_reset(&g_app.safety, &g_app.faults,
                                           now_us))
            {
                g_app.request_fault_reset = true;
            }
            else
            {
                log_format("# DENIED,fault still active=%08lX\r\n",
                           (unsigned long)g_app.faults.active_flags);
            }
            break;
        case VEHICLE_DIAG_CALIBRATION_MODE:
            g_app.request_calibration_mode = true;
            break;
        case VEHICLE_DIAG_IDLE:
            g_app.request_idle = true;
            break;
        default:
            break;
    }
}

static void poll_user_inputs(uint64_t now_us)
{
    uint8_t byte;
    uint32_t count = 0U;
    uint8_t keys;
    VehicleDiagnosticRequest request;
    char response[VEHICLE_COMMAND_BUFFER_SIZE];
    while((count < VEHICLE_UART_RX_BUDGET) &&
          vehicle_hal_uart_read_byte(&byte))
    {
        vehicle_diagnostics_feed_byte(&g_app.diagnostics, byte);
        count++;
    }
    if(vehicle_diagnostics_take_response(&g_app.diagnostics, response,
                                         sizeof(response)))
    {
        log_text(response);
    }
    if(vehicle_diagnostics_take_request(&g_app.diagnostics, &request))
    {
        handle_diagnostic_request(&request, now_us);
    }
    keys = vehicle_hal_read_key_events();
    if((keys != 0U) &&
       ((now_us - g_app.last_key_timestamp_us) >= VEHICLE_KEY_DEBOUNCE_US))
    {
        g_app.last_key_timestamp_us = now_us;
        if((keys & 1U) != 0U)
        {
            vehicle_display_next_page(&g_app.display);
        }
        if((keys & 2U) != 0U)
        {
            g_app.calibration_test_active = false;
            g_app.calibration_test_output_applied = false;
            if(g_app.state_machine.state == VEHICLE_STATE_STAGE1_RECORD)
            {
                g_app.stage1_stop_pending = true;
            }
            else
            {
                g_app.request_stop = true;
                vehicle_hal_force_safe_outputs();
            }
        }
    }
}

static void update_state_machine(uint64_t now_us)
{
    VehicleStateMachineEvents events;
    VehiclePathInfo path_info = vehicle_path_get_info();
    bool stage1_stopped = g_app.stage1_stop_pending &&
        g_app.stage1_recording_ended &&
        vehicle_app_is_stopped();
    memset(&events, 0, sizeof(events));
    events.boot_complete = true;
    events.critical_drivers_ready = vehicle_app_commissioning_ready();
    events.sensors_calibrated = vehicle_imu_is_calibrated(&g_app.imu) &&
        (g_app.telemetry.steering.valid || !g_app.calibration_valid);
    events.calibration_valid = g_app.calibration_valid;
    events.request_calibration_mode = g_app.request_calibration_mode;
    events.request_idle = g_app.request_idle;
    events.request_stage1_start = g_app.request_stage1_start;
    events.request_stage1_stop = false;
    events.request_stage2_start = g_app.request_stage2_start;
    events.request_stop = g_app.request_stop;
    events.request_fault_reset = g_app.request_fault_reset;
    events.stage1_limit_reached = stage1_stopped;
    events.path_processing_complete = g_app.path_processing_complete;
    events.path_valid = g_app.path_valid && !path_info.overflowed;
    events.reverse_finished = g_app.reverse_finished;
    events.fault_active = vehicle_fault_has_active(&g_app.faults);
    (void)vehicle_state_machine_update(&g_app.state_machine, &events, now_us);
    clear_requests();
    handle_state_entry(now_us);
}

static void update_telemetry(uint64_t now_us)
{
    VehiclePathInfo info = vehicle_path_get_info();
    g_app.telemetry.timestamp_us = now_us;
    g_app.telemetry.state = g_app.state_machine.state;
    g_app.telemetry.path_point_count = info.point_count;
    g_app.telemetry.path_length_m = info.length_m;
    g_app.telemetry.fault_flags = g_app.faults.active_flags |
                                  g_app.controlled_stop_fault_flags;
    g_app.telemetry.latched_fault_flags = g_app.faults.latched_flags;
}

bool vehicle_app_init(void)
{
    VehicleImuConfig imu_config;
    uint64_t now_us;
    memset(&g_app, 0, sizeof(g_app));
    vehicle_log_init(&g_app.log);
    vehicle_diagnostics_init(&g_app.diagnostics);
    vehicle_display_init(&g_app.display);
    vehicle_fault_init(&g_app.faults);
    vision_shared_init();
#if VEHICLE_VISION_ENABLE
    if(!vehicle_vision_camera_init())
    {
        log_text("# VISION camera init failed; visual commands remain invalid\r\n");
    }
#endif
    perception_init();
    vehicle_path_reset();
    vehicle_reverse_tracker_reset(&g_app.reverse_tracker);
    vehicle_mt6701_init(&g_app.mt6701);

    g_app.steering_tables = vehicle_calibration_get_steering_tables();
    g_app.calibration_valid = vehicle_calibration_is_valid(
        &g_vehicle_calibration, &g_app.steering_tables);
    (void)vehicle_encoder_init(&g_app.left_encoder,
        g_vehicle_calibration.left_encoder_forward_sign,
        g_vehicle_calibration.left_meter_per_count);
    (void)vehicle_encoder_init(&g_app.right_encoder,
        g_vehicle_calibration.right_encoder_forward_sign,
        g_vehicle_calibration.right_meter_per_count);

    g_app.hal_status = vehicle_hal_init();
    imu_config.axis_map[0] = g_vehicle_calibration.imu_axis_map[0];
    imu_config.axis_map[1] = g_vehicle_calibration.imu_axis_map[1];
    imu_config.axis_map[2] = g_vehicle_calibration.imu_axis_map[2];
    imu_config.axis_sign[0] = g_vehicle_calibration.imu_axis_sign[0];
    imu_config.axis_sign[1] = g_vehicle_calibration.imu_axis_sign[1];
    imu_config.axis_sign[2] = g_vehicle_calibration.imu_axis_sign[2];
    vehicle_hal_get_imu_scale(&imu_config.acceleration_mps2_per_lsb,
                              &imu_config.angular_rate_radps_per_lsb,
                              &imu_config.magnetic_field_gauss_per_lsb,
                              &imu_config.temperature_c_per_lsb,
                              &imu_config.temperature_offset_c);
    if(!vehicle_imu_init(&g_app.imu, &imu_config))
    {
        g_app.hal_status.imu_ready = false;
        g_app.hal_status.critical_ready = false;
    }
    vehicle_control_init(&g_app.control, &g_vehicle_calibration,
                         &g_app.steering_tables);
    vehicle_safety_init(&g_app.safety, NULL, &g_vehicle_calibration,
                        &g_app.steering_tables);

    now_us = vehicle_hal_now_us();
    vehicle_state_machine_init(&g_app.state_machine, now_us);
    g_app.telemetry.state = VEHICLE_STATE_BOOT;
    g_app.telemetry.timestamp_us = now_us;
    g_app.next_estimator_timestamp_us = now_us;
    g_app.next_control_timestamp_us = now_us;
    g_app.next_tracker_timestamp_us = now_us;
    g_app.next_safety_timestamp_us = now_us;
    g_app.next_log_timestamp_us = now_us;
    g_app.next_display_timestamp_us = now_us;
    g_app.last_key_timestamp_us = now_us;
    g_app.initialized = true;

    if(!vehicle_app_commissioning_ready())
    {
        vehicle_fault_raise(&g_app.faults, VEHICLE_FAULT_DRIVER_INIT,
                            true, now_us);
    }
    log_text("# TC387 vehicle controller boot; UART0 460800 8N1; send HELP\r\n");
    if(VEHICLE_MT6701_INTERFACE == VEHICLE_MT6701_INTERFACE_UNCONFIRMED)
    {
        log_text("# BLOCKED: automatic mode needs MT6701 P5 protocol/line order; select SSI or AB in vehicle_hardware_config.h\r\n");
    }
    log_text("# DRIVE speed_mps steering_rad: STAGE1 only, max 0.30 m/s and 0.35 rad, repeat within 150 ms\r\n");
    print_configuration();
    update_state_machine(now_us);
    update_telemetry(now_us);
    return vehicle_app_commissioning_ready();
}

void vehicle_app_process(void)
{
    uint64_t now_us;
    uint32_t catchup = 0U;
    bool power_allowed;
    bool control_updated = false;
    bool calibration_test_speed_exceeded = false;
    if(!g_app.initialized)
    {
        return;
    }
    now_us = vehicle_hal_now_us();
    poll_user_inputs(now_us);

    while(vehicle_hal_take_fast_tick() &&
          (catchup < VEHICLE_FAST_TICK_CATCHUP_LIMIT))
    {
        vehicle_hal_capture_encoder_counts();
        catchup++;
    }
    if(now_us >= g_app.next_estimator_timestamp_us)
    {
        estimator_task(now_us);
        g_app.next_estimator_timestamp_us = now_us +
                                            VEHICLE_ESTIMATOR_PERIOD_US;
    }
    if(now_us >= g_app.next_tracker_timestamp_us)
    {
        tracker_task(now_us);
        g_app.next_tracker_timestamp_us = now_us + VEHICLE_TRACKER_PERIOD_US;
    }
    if(now_us >= g_app.next_control_timestamp_us)
    {
        control_task(now_us);
        control_updated = true;
        g_app.next_control_timestamp_us = now_us + VEHICLE_CONTROL_PERIOD_US;
    }
    if(now_us >= g_app.next_safety_timestamp_us)
    {
        safety_task(now_us);
        g_app.next_safety_timestamp_us = now_us + VEHICLE_CONTROL_PERIOD_US;
    }

    if(g_app.calibration_test_active &&
       (now_us >= g_app.calibration_test_deadline_us))
    {
        g_app.calibration_test_active = false;
        g_app.calibration_test_output_applied = false;
        vehicle_hal_force_safe_outputs();
        log_text("# MOTOR test timeout; PWM=0\r\n");
    }
    if(g_app.calibration_test_active && g_app.calibration_valid)
    {
        if(g_app.calibration_test_motor == VEHICLE_HAL_TEST_LEFT_MOTOR)
        {
            calibration_test_speed_exceeded =
                g_app.telemetry.left_wheel.valid &&
                (fabsf(g_app.telemetry.left_wheel.speed_mps) >
                 CALIBRATION_TEST_MAX_SPEED_MPS);
        }
        else if(g_app.calibration_test_motor == VEHICLE_HAL_TEST_RIGHT_MOTOR)
        {
            calibration_test_speed_exceeded =
                g_app.telemetry.right_wheel.valid &&
                (fabsf(g_app.telemetry.right_wheel.speed_mps) >
                 CALIBRATION_TEST_MAX_SPEED_MPS);
        }
        else
        {
            calibration_test_speed_exceeded =
                fabsf(g_app.telemetry.pose.vehicle_speed_mps) >
                CALIBRATION_TEST_MAX_SPEED_MPS;
        }
    }
    if(calibration_test_speed_exceeded)
    {
        g_app.calibration_test_active = false;
        g_app.calibration_test_output_applied = false;
        vehicle_hal_force_safe_outputs();
        log_text("# MOTOR test speed limit; PWM=0\r\n");
    }
    update_controlled_stop(now_us);
    update_state_machine(now_us);
    update_telemetry(now_us);

    power_allowed = vehicle_state_is_automatic(g_app.state_machine.state) &&
        g_app.hal_status.critical_ready &&
        vehicle_safety_output_is_allowed(&g_app.safety, &g_app.faults);
    vehicle_safety_gate_actuators(&g_app.safety, &g_app.faults,
                                  &g_app.telemetry.actuators);
    if(g_app.calibration_test_active &&
       (g_app.state_machine.state == VEHICLE_STATE_CALIBRATION_MODE) &&
       !vehicle_fault_has_active(&g_app.faults))
    {
        if(!g_app.calibration_test_output_applied)
        {
            vehicle_hal_apply_calibration_test(g_app.calibration_test_motor,
                                               g_app.calibration_test_duty);
            g_app.calibration_test_output_applied = true;
        }
    }
    else
    {
        vehicle_hal_apply_actuators(&g_app.telemetry.actuators,
                                    &g_vehicle_calibration, power_allowed);
    }

    if(now_us >= g_app.next_log_timestamp_us)
    {
        if(vehicle_log_is_csv_enabled(&g_app.log))
        {
            (void)vehicle_log_enqueue_csv(&g_app.log, &g_app.telemetry);
        }
        g_app.next_log_timestamp_us = now_us + VEHICLE_LOG_PERIOD_US;
    }
    if(vehicle_log_dropped_lines(&g_app.log) > 0U)
    {
        g_app.faults.latched_flags |= VEHICLE_FAULT_LOG_OVERFLOW;
        g_app.telemetry.latched_fault_flags = g_app.faults.latched_flags;
    }
    (void)vehicle_log_flush(&g_app.log, VEHICLE_UART_TX_BUDGET,
                            vehicle_hal_uart_try_write_byte, NULL);

    if(now_us >= g_app.next_display_timestamp_us)
    {
        vehicle_display_render(&g_app.display, &g_app.telemetry,
                               g_app.calibration_valid,
                               vehicle_log_dropped_lines(&g_app.log),
                               vehicle_hal_display_line, NULL);
        g_app.next_display_timestamp_us = now_us + VEHICLE_DISPLAY_PERIOD_US;
    }
    if(control_updated ||
       !vehicle_state_is_automatic(g_app.state_machine.state))
    {
        vehicle_hal_service_control_watchdog();
    }
}
