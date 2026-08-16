#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_calibration.h"
#include "vehicle_types.h"

struct VehicleWheelControllerConfig
{
    float kp;
    float ki;
    float feedforward;
    float integral_limit;
    float output_limit;
    float output_slew_per_s;
    float zero_band_mps;
    float direction_hold_s;
};

struct VehicleWheelSpeedController
{
    VehicleWheelControllerConfig config;
    float start_duty_forward;
    float start_duty_reverse;
    float integral;
    float output;
    float zero_dwell_s;
    int8_t active_direction;
    bool calibration_valid;
};

struct VehicleDriveController
{
    VehicleWheelSpeedController left;
    VehicleWheelSpeedController right;
    bool calibration_valid;
};

struct VehicleDriveControlOutput
{
    float left_target_speed_mps;
    float right_target_speed_mps;
    float left_signed_duty;
    float right_signed_duty;
    bool valid;
};

enum VehicleSteeringApproach
{
    VEHICLE_STEERING_APPROACH_UNKNOWN = 0,
    VEHICLE_STEERING_APPROACH_FROM_LEFT,
    VEHICLE_STEERING_APPROACH_FROM_RIGHT
};

struct VehicleSteeringControllerConfig
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float output_slew_per_s;
    float position_deadband_rad;
    float stall_duty;
    int64_t stall_count_delta;
    float stall_timeout_s;
    int64_t approach_switch_hysteresis_count;
};

struct VehicleSteeringController
{
    VehicleSteeringControllerConfig config;
    const VehicleCalibration *calibration;
    SteeringCalibrationTables tables;
    float integral;
    float previous_error_rad;
    float output;
    float stall_elapsed_s;
    int64_t previous_count;
    int64_t target_count;
    VehicleSteeringApproach approach;
    bool calibration_valid;
    bool previous_sample_valid;
    bool stalled;
};

struct VehicleSteeringControlOutput
{
    float measured_angle_rad;
    int64_t target_count;
    /* Positive duty turns the road wheels left (positive steering angle). */
    float signed_duty;
    VehicleSteeringApproach approach;
    bool at_soft_limit;
    bool stalled;
    bool valid;
};

struct VehicleControl
{
    VehicleDriveController drive;
    VehicleSteeringController steering;
};

struct VehicleControlInput
{
    float target_center_speed_mps;
    float target_steering_rad;
    float left_speed_mps;
    float right_speed_mps;
    int64_t steering_continuous_count;
    float dt_s;
};

struct VehicleControlOutput
{
    VehicleDriveControlOutput drive;
    VehicleSteeringControlOutput steering;
    VehicleActuatorCommand actuators;
    bool valid;
};

void vehicle_wheel_controller_default_config(VehicleWheelControllerConfig *config);
void vehicle_wheel_speed_controller_init(VehicleWheelSpeedController *controller,
                                         const VehicleWheelControllerConfig *config,
                                         float start_duty_forward,
                                         float start_duty_reverse,
                                         bool calibration_valid);
void vehicle_wheel_speed_controller_reset(VehicleWheelSpeedController *controller);
float vehicle_wheel_speed_controller_update(VehicleWheelSpeedController *controller,
                                            float target_speed_mps,
                                            float measured_speed_mps,
                                            float dt_s);

bool vehicle_control_compute_wheel_targets(float center_speed_mps,
                                           float steering_angle_rad,
                                           float *left_target_speed_mps,
                                           float *right_target_speed_mps);
bool vehicle_control_compute_wheel_targets_from_curvature(
    float center_speed_mps,
    float curvature_per_m,
    float *left_target_speed_mps,
    float *right_target_speed_mps);

void vehicle_drive_controller_init(VehicleDriveController *controller,
                                   const VehicleCalibration *calibration,
                                   const SteeringCalibrationTables *tables,
                                   const VehicleWheelControllerConfig *left_config,
                                   const VehicleWheelControllerConfig *right_config);
void vehicle_drive_controller_reset(VehicleDriveController *controller);
void vehicle_drive_controller_update(VehicleDriveController *controller,
                                     float center_speed_mps,
                                     float steering_angle_rad,
                                     float left_speed_mps,
                                     float right_speed_mps,
                                     float dt_s,
                                     VehicleDriveControlOutput *output);

void vehicle_steering_controller_default_config(
    VehicleSteeringControllerConfig *config);
bool vehicle_steering_table_count_to_angle(
    const SteeringCalibrationPoint *points,
    size_t count,
    int64_t continuous_count,
    float *angle_rad);
bool vehicle_steering_table_angle_to_count(
    const SteeringCalibrationPoint *points,
    size_t count,
    float angle_rad,
    int64_t *continuous_count);
void vehicle_steering_controller_init(
    VehicleSteeringController *controller,
    const VehicleCalibration *calibration,
    const SteeringCalibrationTables *tables,
    const VehicleSteeringControllerConfig *config);
void vehicle_steering_controller_reset(VehicleSteeringController *controller);
void vehicle_steering_controller_update(
    VehicleSteeringController *controller,
    float target_angle_rad,
    int64_t continuous_count,
    float dt_s,
    VehicleSteeringControlOutput *output);
bool vehicle_steering_controller_is_stalled(
    const VehicleSteeringController *controller);

void vehicle_control_init(VehicleControl *control,
                          const VehicleCalibration *calibration,
                          const SteeringCalibrationTables *tables);
void vehicle_control_reset(VehicleControl *control);
void vehicle_control_update(VehicleControl *control,
                            const VehicleControlInput *input,
                            VehicleControlOutput *output);

#endif
