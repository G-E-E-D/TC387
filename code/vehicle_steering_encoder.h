#ifndef VEHICLE_STEERING_ENCODER_H
#define VEHICLE_STEERING_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

struct VehicleSteeringEncoderSample
{
    uint64_t timestamp_us;
    uint16_t raw_count;       /* 0..4095 */
    int32_t relative_count;   /* delta12(raw, fixed center) */
    int64_t continuous_count; /* diagnostic only */
    uint32_t communication_error_count;
    uint32_t jump_error_count;
    bool valid;
};

struct VehicleSteeringEncoderConfig
{
    uint16_t center_raw_count;
    int32_t maximum_delta_count;
    uint64_t timeout_us;
};

typedef bool (*VehicleSteeringEncoderReadRaw)(uint16_t *raw_count,
                                               void *context);

struct VehicleSteeringEncoderIo
{
    VehicleSteeringEncoderReadRaw read_raw;
    void *context;
};

struct VehicleSteeringEncoder
{
    VehicleSteeringEncoderConfig config;
    VehicleSteeringEncoderIo io;
    uint16_t previous_raw_count;
    uint16_t last_raw_count;
    int64_t continuous_count;
    uint64_t last_observation_timestamp_us;
    uint64_t last_valid_timestamp_us;
    uint32_t communication_error_count;
    uint32_t jump_error_count;
    bool configured;
    bool initialized;
};

void vehicle_steering_encoder_default_config(
    VehicleSteeringEncoderConfig *config);
bool vehicle_steering_encoder_config_is_valid(
    const VehicleSteeringEncoderConfig *config);
bool vehicle_steering_encoder_init(VehicleSteeringEncoder *encoder,
                                   const VehicleSteeringEncoderConfig *config,
                                   const VehicleSteeringEncoderIo *io);
int32_t vehicle_steering_encoder_delta12(uint16_t now, uint16_t previous);
bool vehicle_steering_encoder_read(
    VehicleSteeringEncoder *encoder, uint64_t now_us,
    VehicleSteeringEncoderSample *sample);
bool vehicle_steering_encoder_is_fresh(
    const VehicleSteeringEncoder *encoder, uint64_t now_us,
    uint64_t timeout_us);

#endif
