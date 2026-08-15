#ifndef VEHICLE_IMU_H
#define VEHICLE_IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_types.h"

#define VEHICLE_IMU_AXIS_COUNT       (3U)
#define VEHICLE_STANDARD_GRAVITY_MPS2 (9.80665f)

typedef struct
{
    int8_t axis_map[VEHICLE_IMU_AXIS_COUNT];
    int8_t axis_sign[VEHICLE_IMU_AXIS_COUNT];
    float acceleration_mps2_per_lsb;
    float angular_rate_radps_per_lsb;
    float magnetic_field_gauss_per_lsb;
    float temperature_c_per_lsb;
    float temperature_offset_c;
} VehicleImuConfig;

typedef struct
{
    VehicleImuConfig config;
    ImuSample last_sample;
    float gyro_bias_radps[VEHICLE_IMU_AXIS_COUNT];
    float gyro_noise_stddev_radps[VEHICLE_IMU_AXIS_COUNT];
    double calibration_gyro_sum[VEHICLE_IMU_AXIS_COUNT];
    double calibration_gyro_square_sum[VEHICLE_IMU_AXIS_COUNT];
    uint64_t calibration_start_timestamp_us;
    uint64_t last_valid_timestamp_us;
    uint32_t calibration_sample_count;
    uint32_t invalid_sample_count;
    bool configured;
    bool initialized;
    bool calibrated;
    bool stationary;
} VehicleImu;

bool vehicle_imu_config_is_valid(const VehicleImuConfig *config);
bool vehicle_imu_init(VehicleImu *imu, const VehicleImuConfig *config);
void vehicle_imu_restart_calibration(VehicleImu *imu);
bool vehicle_imu_update(VehicleImu *imu, uint64_t timestamp_us,
                        const int16_t raw_acceleration[VEHICLE_IMU_AXIS_COUNT],
                        const int16_t raw_angular_rate[VEHICLE_IMU_AXIS_COUNT],
                        const int16_t raw_magnetic_field[VEHICLE_IMU_AXIS_COUNT],
                        int16_t raw_temperature, bool accel_gyro_communication_ok,
                        bool magnetometer_communication_ok, ImuSample *sample);
bool vehicle_imu_is_calibrated(const VehicleImu *imu);
bool vehicle_imu_get_corrected_angular_rate(
    const VehicleImu *imu,
    float corrected_radps[VEHICLE_IMU_AXIS_COUNT]);
bool vehicle_imu_is_fresh(const VehicleImu *imu, uint64_t now_us,
                          uint64_t timeout_us);

#endif
