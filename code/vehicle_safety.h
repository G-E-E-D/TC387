#ifndef VEHICLE_SAFETY_H
#define VEHICLE_SAFETY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "vehicle_calibration.h"
#include "vehicle_fault.h"
#include "vehicle_types.h"

typedef struct
{
    float encoder_max_abs_speed_mps;
    float encoder_max_acceleration_mps2;
    float motor_stall_duty;
    float motor_stall_speed_mps;
    float motor_stall_timeout_s;
    float wheel_mismatch_min_speed_mps;
    float wheel_mismatch_max_mps;
    float wheel_mismatch_timeout_s;
    int64_t mt6701_max_delta_count;
    uint32_t mt6701_error_limit;
    float steering_stall_duty;
    int64_t steering_stall_count_delta;
    float steering_stall_timeout_s;
    float imu_max_acceleration_mps2;
    float imu_max_angular_rate_radps;
    uint64_t sensor_timeout_us;
    uint64_t control_timeout_us;
    uint32_t tracker_index_loss_limit;
    float tracker_max_cross_track_error_m;
    float tracker_max_heading_error_rad;
} VehicleSafetyConfig;

typedef struct
{
    uint64_t timestamp_us;
    VehicleState state;
    WheelEncoderSample left_wheel;
    WheelEncoderSample right_wheel;
    float left_target_speed_mps;
    float right_target_speed_mps;
    float left_motor_duty;
    float right_motor_duty;
    bool mt6701_communication_ok;
    SteeringSample steering;
    float steering_motor_duty;
    bool imu_communication_ok;
    ImuSample imu;
    VehiclePose pose;
    uint64_t last_control_timestamp_us;
    bool path_overflow;
    bool path_valid;
    uint32_t path_point_count;
    uint32_t path_capacity;
    bool path_index_valid;
    ReverseTrackerOutput tracker;
    bool controlled_stop_in_progress;
    bool numeric_valid;
    const float *additional_numeric_values;
    size_t additional_numeric_value_count;
} VehicleSafetyInputs;

typedef struct
{
    VehicleSafetyConfig config;
    bool calibration_valid;
    float left_meter_per_count;
    float right_meter_per_count;
    int64_t steering_left_limit_count;
    int64_t steering_right_limit_count;
    float left_stall_elapsed_s;
    float right_stall_elapsed_s;
    float wheel_mismatch_elapsed_s;
    float steering_stall_elapsed_s;
    float previous_left_speed_mps;
    float previous_right_speed_mps;
    uint64_t previous_left_timestamp_us;
    uint64_t previous_right_timestamp_us;
    int64_t previous_steering_count;
    uint64_t previous_steering_timestamp_us;
    uint64_t previous_update_timestamp_us;
    uint32_t mt6701_error_count;
    uint32_t tracker_index_loss_count;
    uint32_t monitored_active_flags;
    uint32_t controlled_stop_request_flags;
    bool previous_left_valid;
    bool previous_right_valid;
    bool previous_steering_valid;
    bool automatic_output_inhibited;
} VehicleSafetyMonitor;

void vehicle_safety_default_config(VehicleSafetyConfig *config);
void vehicle_safety_init(VehicleSafetyMonitor *monitor,
                         const VehicleSafetyConfig *config,
                         const VehicleCalibration *calibration,
                         const SteeringCalibrationTables *tables);
void vehicle_safety_update(VehicleSafetyMonitor *monitor,
                           VehicleFaultManager *fault_manager,
                           const VehicleSafetyInputs *inputs);
bool vehicle_safety_manual_reset(VehicleSafetyMonitor *monitor,
                                 VehicleFaultManager *fault_manager,
                                 uint64_t timestamp_us);
bool vehicle_safety_output_is_allowed(const VehicleSafetyMonitor *monitor,
                                      const VehicleFaultManager *fault_manager);
uint32_t vehicle_safety_get_controlled_stop_request_flags(
    const VehicleSafetyMonitor *monitor);
void vehicle_safety_gate_actuators(const VehicleSafetyMonitor *monitor,
                                   const VehicleFaultManager *fault_manager,
                                   VehicleActuatorCommand *command);

#endif
