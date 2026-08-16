#ifndef VEHICLE_CALIBRATION_H
#define VEHICLE_CALIBRATION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MEASURE_REQUIRED
constexpr auto CALIBRATION_VALID = false;
constexpr auto STEERING_CALIBRATION_MAX_POINTS = 24U;

struct VehicleCalibration
{
    bool valid;

    int8_t left_encoder_forward_sign;
    int8_t right_encoder_forward_sign;
    float left_meter_per_count;
    float right_meter_per_count;
    int32_t left_counts_per_wheel_rev;
    int32_t right_counts_per_wheel_rev;

    int8_t left_motor_forward_direction;
    int8_t right_motor_forward_direction;
    float left_motor_start_duty_forward;
    float left_motor_start_duty_reverse;
    float right_motor_start_duty_forward;
    float right_motor_start_duty_reverse;

    /* Raw DIR sign that physically steers left; positive steering is left. */
    int8_t steering_left_direction;
    float steering_start_duty_left;
    float steering_start_duty_right;
    /* Relative MT6701 counts measured at the physical left/right limits. */
    int64_t steering_left_soft_limit_count;
    int64_t steering_right_soft_limit_count;

    float imu_position_x_m;
    float imu_position_y_m;
    float imu_position_z_m;
    int8_t imu_axis_map[3];
    int8_t imu_axis_sign[3];

    float max_acceleration_mps2;
    float max_deceleration_mps2;
    float max_lateral_acceleration_mps2;
};

struct SteeringCalibrationPoint
{
    int64_t continuous_count;
    float equivalent_steering_angle_rad;
};

struct SteeringCalibrationTables
{
    const SteeringCalibrationPoint *from_left;
    size_t from_left_count;
    const SteeringCalibrationPoint *from_right;
    size_t from_right_count;
};

extern const VehicleCalibration g_vehicle_calibration;
extern const SteeringCalibrationPoint g_steering_calibration_from_left[];
extern const size_t g_steering_calibration_from_left_count;
extern const SteeringCalibrationPoint g_steering_calibration_from_right[];
extern const size_t g_steering_calibration_from_right_count;

bool vehicle_calibration_is_valid(const VehicleCalibration *calibration,
                                  const SteeringCalibrationTables *tables);
SteeringCalibrationTables vehicle_calibration_get_steering_tables();

#endif
