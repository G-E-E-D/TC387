#ifndef VEHICLE_TYPES_H
#define VEHICLE_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum VehicleState
{
    VEHICLE_STATE_BOOT = 0,
    VEHICLE_STATE_SENSOR_CALIBRATION,
    VEHICLE_STATE_IDLE,
    VEHICLE_STATE_STAGE1_RECORD,
    VEHICLE_STATE_STAGE1_FINISHED,
    VEHICLE_STATE_STAGE2_REVERSE,
    VEHICLE_STATE_FINISHED,
    VEHICLE_STATE_FAULT,
    VEHICLE_STATE_CALIBRATION_MODE
};

struct Stage1ControlCommand
{
    float target_speed_mps;
    float target_steering_rad;
    bool valid;
};

struct WheelEncoderSample
{
    uint64_t timestamp_us;
    int64_t count;
    int32_t delta_count;
    float speed_mps;
    bool valid;
};

struct SteeringSample
{
    uint64_t timestamp_us;
    uint16_t raw_angle;
    uint8_t magnetic_status;
    int64_t continuous_count;
    int64_t relative_count;
    float angle_rad;
    bool valid;
};

struct ImuSample
{
    uint64_t timestamp_us;
    int16_t raw_acc[3];
    int16_t raw_gyro[3];
    int16_t raw_mag[3];
    int16_t raw_temperature;
    float acceleration_mps2[3];
    float angular_rate_radps[3];
    float magnetic_field_gauss[3];
    float temperature_c;
    bool accel_gyro_valid;
    bool magnetometer_valid;
};

struct VehiclePose
{
    uint64_t timestamp_us;
    float x_m;
    float y_m;
    float yaw_rad;
    float vehicle_speed_mps;
    float gyro_z_bias_radps;
    float omega_imu_radps;
    float omega_wheel_radps;
    float omega_steering_radps;
    bool stationary;
    bool wheel_slip;
    bool valid;
};

struct PathPoint
{
    float x;
    float y;
    float yaw_body;
    float s;
    float curvature_forward;
    float recorded_speed;
};

struct ReverseTrackerOutput
{
    float target_speed_mps;
    float target_steering_rad;
    float cross_track_error_m;
    float heading_error_rad;
    float lookahead_m;
    uint32_t nearest_index;
    uint32_t target_index;
    bool finished;
    bool valid;
};

struct VehicleActuatorCommand
{
    float left_motor_duty;
    float right_motor_duty;
    float steering_motor_duty;
    bool immediate_stop;
};

struct VehicleTelemetry
{
    VehicleState state;
    uint64_t timestamp_us;
    WheelEncoderSample left_wheel;
    WheelEncoderSample right_wheel;
    SteeringSample steering;
    ImuSample imu;
    VehiclePose pose;
    ReverseTrackerOutput tracker;
    float target_speed_mps;
    float target_steering_rad;
    VehicleActuatorCommand actuators;
    uint32_t path_point_count;
    float path_length_m;
    uint32_t fault_flags;
    uint32_t latched_fault_flags;
};

#endif
