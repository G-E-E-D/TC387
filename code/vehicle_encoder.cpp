#include "vehicle_encoder.h"

#include <math.h>
#include <stddef.h>

#include "vehicle_config.h"
#include "vehicle_math.h"

static void vehicle_encoder_increment_error(VehicleEncoder *encoder)
{
    if(encoder->invalid_sample_count < UINT32_MAX)
    {
        ++encoder->invalid_sample_count;
    }
}

static bool vehicle_encoder_add_count(int64_t count, int32_t delta,
                                      int64_t *result)
{
    if(result == nullptr)
    {
        return false;
    }
    if((delta > 0) && (count > (INT64_MAX - static_cast<int64_t>(delta))))
    {
        *result = INT64_MAX;
        return false;
    }
    if((delta < 0) && (count < (INT64_MIN - static_cast<int64_t>(delta))))
    {
        *result = INT64_MIN;
        return false;
    }
    *result = count + static_cast<int64_t>(delta);
    return true;
}

static void vehicle_encoder_fill_sample(const VehicleEncoder *encoder,
                                        uint64_t timestamp_us,
                                        int32_t delta_count, bool valid,
                                        WheelEncoderSample *sample)
{
    if(sample != nullptr)
    {
        sample->timestamp_us = timestamp_us;
        sample->count = encoder->continuous_count;
        sample->delta_count = delta_count;
        sample->speed_mps = valid ? encoder->speed_mps : 0.0f;
        sample->valid = valid;
    }
}

int32_t vehicle_encoder_delta16(uint16_t current_count,
                                uint16_t previous_count)
{
    uint32_t modular_delta;

    modular_delta = (static_cast<uint32_t>(current_count) - static_cast<uint32_t>(previous_count)) &
                    UINT32_C(0xFFFF);
    if(modular_delta >= UINT32_C(0x8000))
    {
        return static_cast<int32_t>(modular_delta) - INT32_C(65536);
    }
    return static_cast<int32_t>(modular_delta);
}

bool vehicle_encoder_init(VehicleEncoder *encoder, int8_t forward_sign,
                          float meter_per_count)
{
    bool valid;

    if(encoder == nullptr)
    {
        return false;
    }

    valid = ((forward_sign == INT8_C(1)) ||
             (forward_sign == -INT8_C(1))) &&
            vehicle_float_is_finite(meter_per_count) &&
            (meter_per_count > 0.0f);

    encoder->forward_sign = forward_sign;
    encoder->meter_per_count = meter_per_count;
    encoder->previous_raw_count = 0U;
    encoder->previous_timestamp_us = 0U;
    encoder->continuous_count = 0;
    encoder->speed_mps = 0.0f;
    encoder->invalid_sample_count = 0U;
    encoder->configured = valid;
    encoder->initialized = false;
    return valid;
}

void vehicle_encoder_reset(VehicleEncoder *encoder, int64_t count)
{
    if(encoder != nullptr)
    {
        encoder->previous_raw_count = 0U;
        encoder->previous_timestamp_us = 0U;
        encoder->continuous_count = count;
        encoder->speed_mps = 0.0f;
        encoder->invalid_sample_count = 0U;
        encoder->initialized = false;
    }
}

bool vehicle_encoder_update(VehicleEncoder *encoder, uint16_t raw_count,
                            uint64_t timestamp_us,
                            WheelEncoderSample *sample)
{
    uint64_t delta_time_us;
    int32_t raw_delta;
    int32_t signed_delta;
    int64_t next_count;
    float delta_time_s;
    float next_speed_mps;
    bool count_valid;

    if((encoder == nullptr) || (sample == nullptr))
    {
        return false;
    }

    if(!encoder->configured)
    {
        vehicle_encoder_increment_error(encoder);
        vehicle_encoder_fill_sample(encoder, timestamp_us, 0, false, sample);
        return false;
    }

    if(!encoder->initialized)
    {
        encoder->previous_raw_count = raw_count;
        encoder->previous_timestamp_us = timestamp_us;
        encoder->speed_mps = 0.0f;
        encoder->initialized = true;
        vehicle_encoder_fill_sample(encoder, timestamp_us, 0, true, sample);
        return true;
    }

    if(timestamp_us <= encoder->previous_timestamp_us)
    {
        vehicle_encoder_increment_error(encoder);
        vehicle_encoder_fill_sample(encoder, timestamp_us, 0, false, sample);
        return false;
    }

    delta_time_us = timestamp_us - encoder->previous_timestamp_us;
    delta_time_s = static_cast<float>(delta_time_us) * 1.0e-6f;
    raw_delta = vehicle_encoder_delta16(raw_count,
                                        encoder->previous_raw_count);
    signed_delta = raw_delta * static_cast<int32_t>(encoder->forward_sign);
    next_speed_mps = (static_cast<float>(signed_delta) * encoder->meter_per_count) /
                     delta_time_s;
    count_valid = vehicle_encoder_add_count(encoder->continuous_count,
                                            signed_delta, &next_count);

    encoder->previous_raw_count = raw_count;
    encoder->previous_timestamp_us = timestamp_us;

    if(!count_valid || !vehicle_float_is_finite(next_speed_mps) ||
       (fabsf(next_speed_mps) > ENCODER_MAX_ABS_SPEED_MPS))
    {
        vehicle_encoder_increment_error(encoder);
        encoder->speed_mps = 0.0f;
        vehicle_encoder_fill_sample(encoder, timestamp_us, signed_delta,
                                    false, sample);
        return false;
    }

    encoder->continuous_count = next_count;
    encoder->speed_mps = next_speed_mps;
    vehicle_encoder_fill_sample(encoder, timestamp_us, signed_delta, true,
                                sample);
    return true;
}

bool vehicle_encoder_is_fresh(const VehicleEncoder *encoder,
                              uint64_t now_us, uint64_t timeout_us)
{
    if((encoder == nullptr) || !encoder->initialized ||
       (now_us < encoder->previous_timestamp_us))
    {
        return false;
    }
    return (now_us - encoder->previous_timestamp_us) <= timeout_us;
}
