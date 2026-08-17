#ifndef VEHICLE_MT6701_H
#define VEHICLE_MT6701_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_types.h"

constexpr auto VEHICLE_MT6701_COUNTS_PER_REVOLUTION = 16384U;
constexpr auto VEHICLE_MT6701_ANGLE_MASK = 0x3FFFU;
constexpr auto VEHICLE_MT6701_STATUS_FIELD_MASK = 0x03U;
constexpr auto VEHICLE_MT6701_STATUS_PUSH_MASK = 0x04U;
constexpr auto VEHICLE_MT6701_STATUS_TRACK_LOSS_MASK = 0x08U;

struct VehicleMt6701Frame
{
    uint16_t raw_angle;
    uint8_t magnetic_status;
    uint8_t received_crc;
    uint8_t calculated_crc;
    bool crc_valid;
};

struct VehicleMt6701
{
    uint16_t previous_raw_angle;
    uint8_t last_magnetic_status;
    int64_t continuous_count;
    int64_t relative_zero_count;
    uint64_t last_observation_timestamp_us;
    uint64_t last_valid_timestamp_us;
    uint32_t communication_error_count;
    uint32_t consecutive_communication_errors;
    uint32_t magnetic_error_count;
    uint32_t jump_error_count;
    bool initialized;
};

uint8_t vehicle_mt6701_crc6(uint32_t angle_and_status_18bits);
bool vehicle_mt6701_parse_ssi_frame(uint32_t frame_24bits,
                                    VehicleMt6701Frame *decoded);
bool vehicle_mt6701_magnetic_status_valid(uint8_t magnetic_status);
int32_t vehicle_mt6701_delta14(uint16_t current_angle,
                               uint16_t previous_angle);
void vehicle_mt6701_init(VehicleMt6701 *sensor);
bool vehicle_mt6701_set_relative_zero(VehicleMt6701 *sensor);
bool vehicle_mt6701_update_angle(VehicleMt6701 *sensor, uint16_t raw_angle,
                                 uint8_t magnetic_status,
                                 uint64_t timestamp_us,
                                 bool communication_ok,
                                 SteeringSample *sample);
bool vehicle_mt6701_update_ssi(VehicleMt6701 *sensor, uint32_t frame_24bits,
                               uint64_t timestamp_us,
                               bool communication_ok,
                               SteeringSample *sample);
bool vehicle_mt6701_is_fresh(const VehicleMt6701 *sensor, uint64_t now_us,
                             uint64_t timeout_us);

#endif
