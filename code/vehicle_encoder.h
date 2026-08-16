#ifndef VEHICLE_ENCODER_H
#define VEHICLE_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_types.h"

struct VehicleEncoder
{
    int8_t forward_sign;
    float meter_per_count;
    uint16_t previous_raw_count;
    uint64_t previous_timestamp_us;
    int64_t continuous_count;
    float speed_mps;
    uint32_t invalid_sample_count;
    bool configured;
    bool initialized;
};

int32_t vehicle_encoder_delta16(uint16_t current_count,
                                uint16_t previous_count);
bool vehicle_encoder_init(VehicleEncoder *encoder, int8_t forward_sign,
                          float meter_per_count);
void vehicle_encoder_reset(VehicleEncoder *encoder, int64_t count);
bool vehicle_encoder_update(VehicleEncoder *encoder, uint16_t raw_count,
                            uint64_t timestamp_us,
                            WheelEncoderSample *sample);
bool vehicle_encoder_is_fresh(const VehicleEncoder *encoder,
                              uint64_t now_us, uint64_t timeout_us);

#endif
