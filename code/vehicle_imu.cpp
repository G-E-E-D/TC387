#include "vehicle_imu.h"

#include <math.h>
#include <stddef.h>

#include "vehicle_config.h"
#include "vehicle_math.h"

constexpr auto VEHICLE_IMU_CALIBRATION_TIME_US = 3000000ULL;

static void vehicle_imu_increment_error(VehicleImu *imu)
{
    if(imu->invalid_sample_count < UINT32_MAX)
    {
        ++imu->invalid_sample_count;
    }
}

static void vehicle_imu_clear_calibration_accumulators(VehicleImu *imu)
{
    unsigned int axis;

    imu->calibration_start_timestamp_us = 0U;
    imu->calibration_sample_count = 0U;
    for(axis = 0U; axis < VEHICLE_IMU_AXIS_COUNT; ++axis)
    {
        imu->calibration_gyro_sum[axis] = 0.0;
        imu->calibration_gyro_square_sum[axis] = 0.0;
    }
}

static bool vehicle_imu_mapping_is_valid(const VehicleImuConfig *config)
{
    bool used[VEHICLE_IMU_AXIS_COUNT] = {false, false, false};
    unsigned int axis;

    for(axis = 0U; axis < VEHICLE_IMU_AXIS_COUNT; ++axis)
    {
        int32_t source_axis;

        source_axis = static_cast<int32_t>(config->axis_map[axis]);
        if((source_axis < 0) ||
           (source_axis >= static_cast<int32_t>(VEHICLE_IMU_AXIS_COUNT)) ||
           used[static_cast<unsigned int>(source_axis)] ||
           ((config->axis_sign[axis] != INT8_C(1)) &&
            (config->axis_sign[axis] != -INT8_C(1))))
        {
            return false;
        }
        used[static_cast<unsigned int>(source_axis)] = true;
    }
    return true;
}

static bool vehicle_imu_measurement_in_range(const ImuSample *sample)
{
    unsigned int axis;

    if(!vehicle_float_array_is_finite(sample->acceleration_mps2,
                                      VEHICLE_IMU_AXIS_COUNT) ||
       !vehicle_float_array_is_finite(sample->angular_rate_radps,
                                      VEHICLE_IMU_AXIS_COUNT) ||
       !vehicle_float_is_finite(sample->temperature_c))
    {
        return false;
    }

    for(axis = 0U; axis < VEHICLE_IMU_AXIS_COUNT; ++axis)
    {
        if((fabsf(sample->acceleration_mps2[axis]) > IMU_MAX_ACCEL_MPS2) ||
           (fabsf(sample->angular_rate_radps[axis]) > IMU_MAX_GYRO_RADPS))
        {
            return false;
        }
    }
    return true;
}

static bool vehicle_imu_measurement_is_stationary(const VehicleImu *imu,
                                                  const ImuSample *sample)
{
    float acceleration_norm;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float gyro_norm;

    acceleration_norm = sqrtf(
        sample->acceleration_mps2[0] * sample->acceleration_mps2[0] +
        sample->acceleration_mps2[1] * sample->acceleration_mps2[1] +
        sample->acceleration_mps2[2] * sample->acceleration_mps2[2]);

    gyro_x = sample->angular_rate_radps[0] - imu->gyro_bias_radps[0];
    gyro_y = sample->angular_rate_radps[1] - imu->gyro_bias_radps[1];
    gyro_z = sample->angular_rate_radps[2] - imu->gyro_bias_radps[2];
    gyro_norm = sqrtf(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);

    return vehicle_float_is_finite(acceleration_norm) &&
           vehicle_float_is_finite(gyro_norm) &&
           (fabsf(acceleration_norm - VEHICLE_STANDARD_GRAVITY_MPS2) <=
            IMU_CALIBRATION_MAX_ACCEL_DELTA_MPS2) &&
           (gyro_norm <= IMU_CALIBRATION_MAX_GYRO_RADPS);
}

static void vehicle_imu_accumulate_calibration(VehicleImu *imu,
                                               const ImuSample *sample)
{
    unsigned int axis;

    if(imu->calibration_sample_count < UINT32_MAX)
    {
        ++imu->calibration_sample_count;
    }
    for(axis = 0U; axis < VEHICLE_IMU_AXIS_COUNT; ++axis)
    {
        double value;

        value = static_cast<double>(sample->angular_rate_radps[axis]);
        imu->calibration_gyro_sum[axis] += value;
        imu->calibration_gyro_square_sum[axis] += value * value;
    }
}

static bool vehicle_imu_finish_calibration(VehicleImu *imu,
                                           uint64_t timestamp_us)
{
    unsigned int axis;
    double sample_count;

    if((imu->calibration_sample_count < IMU_CALIBRATION_MIN_SAMPLES) ||
       (timestamp_us < imu->calibration_start_timestamp_us) ||
       ((timestamp_us - imu->calibration_start_timestamp_us) <
        VEHICLE_IMU_CALIBRATION_TIME_US))
    {
        return false;
    }

    sample_count = static_cast<double>(imu->calibration_sample_count);
    for(axis = 0U; axis < VEHICLE_IMU_AXIS_COUNT; ++axis)
    {
        double mean;
        double variance;

        mean = imu->calibration_gyro_sum[axis] / sample_count;
        variance = (imu->calibration_gyro_square_sum[axis] / sample_count) -
                   (mean * mean);
        if(variance < 0.0)
        {
            variance = 0.0;
        }
        imu->gyro_bias_radps[axis] = static_cast<float>(mean);
        imu->gyro_noise_stddev_radps[axis] = static_cast<float>(sqrt(variance));
        if(!vehicle_float_is_finite(imu->gyro_bias_radps[axis]) ||
           !vehicle_float_is_finite(imu->gyro_noise_stddev_radps[axis]))
        {
            return false;
        }
    }
    imu->calibrated = true;
    return true;
}

bool vehicle_imu_config_is_valid(const VehicleImuConfig *config)
{
    if((config == nullptr) || !vehicle_imu_mapping_is_valid(config))
    {
        return false;
    }
    return vehicle_float_is_finite(config->acceleration_mps2_per_lsb) &&
           (config->acceleration_mps2_per_lsb > 0.0f) &&
           vehicle_float_is_finite(config->angular_rate_radps_per_lsb) &&
           (config->angular_rate_radps_per_lsb > 0.0f) &&
           vehicle_float_is_finite(config->magnetic_field_gauss_per_lsb) &&
           (config->magnetic_field_gauss_per_lsb > 0.0f) &&
           vehicle_float_is_finite(config->temperature_c_per_lsb) &&
           (config->temperature_c_per_lsb != 0.0f) &&
           vehicle_float_is_finite(config->temperature_offset_c);
}

bool vehicle_imu_init(VehicleImu *imu, const VehicleImuConfig *config)
{
    unsigned int axis;
    bool valid;

    if(imu == nullptr)
    {
        return false;
    }

    valid = vehicle_imu_config_is_valid(config);
    for(axis = 0U; axis < VEHICLE_IMU_AXIS_COUNT; ++axis)
    {
        imu->config.axis_map[axis] = valid ? config->axis_map[axis] :
                                            static_cast<int8_t>(axis);
        imu->config.axis_sign[axis] = valid ? config->axis_sign[axis] :
                                             INT8_C(1);
        imu->gyro_bias_radps[axis] = 0.0f;
        imu->gyro_noise_stddev_radps[axis] = 0.0f;
    }
    imu->config.acceleration_mps2_per_lsb =
        valid ? config->acceleration_mps2_per_lsb : 0.0f;
    imu->config.angular_rate_radps_per_lsb =
        valid ? config->angular_rate_radps_per_lsb : 0.0f;
    imu->config.magnetic_field_gauss_per_lsb =
        valid ? config->magnetic_field_gauss_per_lsb : 0.0f;
    imu->config.temperature_c_per_lsb =
        valid ? config->temperature_c_per_lsb : 0.0f;
    imu->config.temperature_offset_c =
        valid ? config->temperature_offset_c : 0.0f;

    imu->last_valid_timestamp_us = 0U;
    imu->invalid_sample_count = 0U;
    imu->configured = valid;
    imu->initialized = false;
    imu->calibrated = false;
    imu->stationary = false;
    vehicle_imu_clear_calibration_accumulators(imu);

    imu->last_sample.timestamp_us = 0U;
    imu->last_sample.raw_temperature = 0;
    imu->last_sample.temperature_c = 0.0f;
    imu->last_sample.accel_gyro_valid = false;
    imu->last_sample.magnetometer_valid = false;
    for(axis = 0U; axis < VEHICLE_IMU_AXIS_COUNT; ++axis)
    {
        imu->last_sample.raw_acc[axis] = 0;
        imu->last_sample.raw_gyro[axis] = 0;
        imu->last_sample.raw_mag[axis] = 0;
        imu->last_sample.acceleration_mps2[axis] = 0.0f;
        imu->last_sample.angular_rate_radps[axis] = 0.0f;
        imu->last_sample.magnetic_field_gauss[axis] = 0.0f;
    }
    return valid;
}

void vehicle_imu_restart_calibration(VehicleImu *imu)
{
    unsigned int axis;

    if(imu == nullptr)
    {
        return;
    }
    for(axis = 0U; axis < VEHICLE_IMU_AXIS_COUNT; ++axis)
    {
        imu->gyro_bias_radps[axis] = 0.0f;
        imu->gyro_noise_stddev_radps[axis] = 0.0f;
    }
    imu->calibrated = false;
    imu->stationary = false;
    vehicle_imu_clear_calibration_accumulators(imu);
}

bool vehicle_imu_update(VehicleImu *imu, uint64_t timestamp_us,
                        const int16_t raw_acceleration[VEHICLE_IMU_AXIS_COUNT],
                        const int16_t raw_angular_rate[VEHICLE_IMU_AXIS_COUNT],
                        const int16_t raw_magnetic_field[VEHICLE_IMU_AXIS_COUNT],
                        int16_t raw_temperature, bool accel_gyro_communication_ok,
                        bool magnetometer_communication_ok, ImuSample *sample)
{
    unsigned int axis;
    bool time_valid;
    bool measurement_valid;

    if((imu == nullptr) || (sample == nullptr))
    {
        return false;
    }

    *sample = imu->last_sample;
    sample->timestamp_us = timestamp_us;
    sample->accel_gyro_valid = false;
    sample->magnetometer_valid = false;

    if(!imu->configured || (raw_acceleration == nullptr) ||
       (raw_angular_rate == nullptr))
    {
        vehicle_imu_increment_error(imu);
        return false;
    }

    for(axis = 0U; axis < VEHICLE_IMU_AXIS_COUNT; ++axis)
    {
        int32_t source_axis;
        float sign;

        source_axis = static_cast<int32_t>(imu->config.axis_map[axis]);
        sign = static_cast<float>(imu->config.axis_sign[axis]);
        sample->raw_acc[axis] = raw_acceleration[axis];
        sample->raw_gyro[axis] = raw_angular_rate[axis];
        sample->raw_mag[axis] = (raw_magnetic_field != nullptr) ?
                                raw_magnetic_field[axis] : 0;
        sample->acceleration_mps2[axis] =
            static_cast<float>(raw_acceleration[source_axis]) * sign *
            imu->config.acceleration_mps2_per_lsb;
        sample->angular_rate_radps[axis] =
            static_cast<float>(raw_angular_rate[source_axis]) * sign *
            imu->config.angular_rate_radps_per_lsb;
        sample->magnetic_field_gauss[axis] =
            (raw_magnetic_field != nullptr) ?
            (static_cast<float>(raw_magnetic_field[source_axis]) * sign *
             imu->config.magnetic_field_gauss_per_lsb) : 0.0f;
    }
    sample->raw_temperature = raw_temperature;
    sample->temperature_c = imu->config.temperature_offset_c +
                            static_cast<float>(raw_temperature) *
                            imu->config.temperature_c_per_lsb;

    time_valid = !imu->initialized ||
                 (timestamp_us > imu->last_valid_timestamp_us);
    measurement_valid = accel_gyro_communication_ok && time_valid &&
                        vehicle_imu_measurement_in_range(sample);
    sample->accel_gyro_valid = measurement_valid;
    sample->magnetometer_valid = measurement_valid &&
        magnetometer_communication_ok &&
        (raw_magnetic_field != nullptr) &&
        vehicle_float_array_is_finite(sample->magnetic_field_gauss,
                                      VEHICLE_IMU_AXIS_COUNT);

    if(!measurement_valid)
    {
        vehicle_imu_increment_error(imu);
        imu->stationary = false;
        return false;
    }

    imu->stationary = vehicle_imu_measurement_is_stationary(imu, sample);
    if(!imu->calibrated)
    {
        if(!imu->stationary)
        {
            vehicle_imu_clear_calibration_accumulators(imu);
        }
        else
        {
            if(imu->calibration_sample_count == 0U)
            {
                imu->calibration_start_timestamp_us = timestamp_us;
            }
            vehicle_imu_accumulate_calibration(imu, sample);
            static_cast<void>(vehicle_imu_finish_calibration(imu, timestamp_us));
        }
    }

    imu->last_sample = *sample;
    imu->last_valid_timestamp_us = timestamp_us;
    imu->initialized = true;
    return true;
}

bool vehicle_imu_is_calibrated(const VehicleImu *imu)
{
    return (imu != nullptr) && imu->configured && imu->calibrated;
}

bool vehicle_imu_get_corrected_angular_rate(
    const VehicleImu *imu,
    float corrected_radps[VEHICLE_IMU_AXIS_COUNT])
{
    unsigned int axis;

    if((imu == nullptr) || (corrected_radps == nullptr) || !imu->calibrated ||
       !imu->last_sample.accel_gyro_valid)
    {
        return false;
    }
    for(axis = 0U; axis < VEHICLE_IMU_AXIS_COUNT; ++axis)
    {
        corrected_radps[axis] = imu->last_sample.angular_rate_radps[axis] -
                                imu->gyro_bias_radps[axis];
    }
    return true;
}

bool vehicle_imu_is_fresh(const VehicleImu *imu, uint64_t now_us,
                          uint64_t timeout_us)
{
    if((imu == nullptr) || !imu->initialized ||
       (now_us < imu->last_valid_timestamp_us))
    {
        return false;
    }
    return (now_us - imu->last_valid_timestamp_us) <= timeout_us;
}
