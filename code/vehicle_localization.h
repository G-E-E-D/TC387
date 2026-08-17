#ifndef VEHICLE_LOCALIZATION_H
#define VEHICLE_LOCALIZATION_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_types.h"

struct VehicleLocalizationDiagnostics
{
    float imu_weight;
    float wheel_weight;
    float steering_weight;
    float wheel_speed_residual_mps;
    float imu_wheel_residual_radps;
    float imu_steering_residual_radps;
    float wheel_steering_residual_radps;
    bool model_disagreement;
    bool imu_suspect;
};

struct VehicleLocalization
{
    VehiclePose pose;
    VehicleLocalizationDiagnostics diagnostics;
    float left_meter_per_count;
    float right_meter_per_count;
    uint64_t last_timestamp_us;
    uint32_t invalid_update_count;
    bool configured;
    bool initialized;
};

bool vehicle_localization_init(VehicleLocalization *localization,
                               float left_meter_per_count,
                               float right_meter_per_count,
                               float initial_gyro_z_bias_radps);
void vehicle_localization_reset(VehicleLocalization *localization,
                                float x_m, float y_m, float yaw_rad,
                                float gyro_z_bias_radps);
bool vehicle_localization_update(VehicleLocalization *localization,
                                 uint64_t timestamp_us,
                                 const WheelEncoderSample *left_wheel,
                                 const WheelEncoderSample *right_wheel,
                                 const ImuSample *imu,
                                 float steering_angle_rad,
                                 bool steering_valid,
                                 VehiclePose *pose);
const VehicleLocalizationDiagnostics *vehicle_localization_get_diagnostics(
    const VehicleLocalization *localization);

#endif
