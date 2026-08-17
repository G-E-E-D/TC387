#include "vehicle_steering_encoder.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "vehicle_config.h"

static void clear_sample(const VehicleSteeringEncoder *encoder,
                         uint64_t timestamp_us,
                         VehicleSteeringEncoderSample *sample)
{
    if((encoder == nullptr) || (sample == nullptr))
    {
        return;
    }
    sample->timestamp_us = timestamp_us;
    sample->raw_count = encoder->last_raw_count;
    sample->relative_count = vehicle_steering_encoder_delta12(
        encoder->last_raw_count, encoder->config.center_raw_count);
    sample->continuous_count = encoder->continuous_count;
    sample->communication_error_count = encoder->communication_error_count;
    sample->jump_error_count = encoder->jump_error_count;
    sample->valid = false;
}

static void increment(uint32_t *counter)
{
    if((counter != nullptr) && (*counter < UINT32_MAX))
    {
        ++(*counter);
    }
}

void vehicle_steering_encoder_default_config(
    VehicleSteeringEncoderConfig *config)
{
    if(config == nullptr)
    {
        return;
    }
    config->center_raw_count = 0U;
    config->maximum_delta_count =
        STEERING_ENCODER_MAX_DELTA_COUNT_PER_SAMPLE;
    config->timeout_us = VEHICLE_SENSOR_TIMEOUT_US;
}

bool vehicle_steering_encoder_config_is_valid(
    const VehicleSteeringEncoderConfig *config)
{
    return (config != nullptr) &&
           (config->center_raw_count <= 4095U) &&
           (config->maximum_delta_count > 0) &&
           (config->maximum_delta_count < 2048) &&
           (config->timeout_us > 0U);
}

bool vehicle_steering_encoder_init(VehicleSteeringEncoder *encoder,
                                   const VehicleSteeringEncoderConfig *config,
                                   const VehicleSteeringEncoderIo *io)
{
    VehicleSteeringEncoderConfig selected;

    if(encoder == nullptr)
    {
        return false;
    }
    vehicle_steering_encoder_default_config(&selected);
    if(config != nullptr)
    {
        selected = *config;
    }
    *encoder = {};
    encoder->config = selected;
    if(io != nullptr)
    {
        encoder->io = *io;
    }
    encoder->configured = vehicle_steering_encoder_config_is_valid(&selected) &&
                         (encoder->io.read_raw != nullptr);
    return encoder->configured;
}

int32_t vehicle_steering_encoder_delta12(uint16_t now, uint16_t previous)
{
    int32_t delta = static_cast<int32_t>(now & 0x0FFFU) -
                    static_cast<int32_t>(previous & 0x0FFFU);
    if(delta >= 2048)
    {
        delta -= 4096;
    }
    else if(delta < -2048)
    {
        delta += 4096;
    }
    return delta;
}

bool vehicle_steering_encoder_read(
    VehicleSteeringEncoder *encoder, uint64_t now_us,
    VehicleSteeringEncoderSample *sample)
{
    uint16_t raw_count = 0U;
    int32_t delta;

    if((encoder == nullptr) || (sample == nullptr) ||
       !encoder->configured)
    {
        return false;
    }
    clear_sample(encoder, now_us, sample);
    if((encoder->last_observation_timestamp_us != 0U) &&
       (now_us <= encoder->last_observation_timestamp_us))
    {
        increment(&encoder->communication_error_count);
        sample->communication_error_count = encoder->communication_error_count;
        return false;
    }
    encoder->last_observation_timestamp_us = now_us;
    if(!encoder->io.read_raw(&raw_count, encoder->io.context) ||
       (raw_count > 4095U))
    {
        increment(&encoder->communication_error_count);
        clear_sample(encoder, now_us, sample);
        return false;
    }
    raw_count &= 0x0FFFU;

    if(!encoder->initialized)
    {
        encoder->previous_raw_count = raw_count;
        encoder->last_raw_count = raw_count;
        encoder->continuous_count = static_cast<int64_t>(raw_count);
        encoder->last_valid_timestamp_us = now_us;
        encoder->initialized = true;
    }
    else
    {
        delta = vehicle_steering_encoder_delta12(
            raw_count, encoder->previous_raw_count);
        if(std::abs(delta) > encoder->config.maximum_delta_count)
        {
            increment(&encoder->jump_error_count);
            clear_sample(encoder, now_us, sample);
            return false;
        }
        if((delta > 0) &&
           (encoder->continuous_count >
            (std::numeric_limits<int64_t>::max() - delta)))
        {
            increment(&encoder->jump_error_count);
            clear_sample(encoder, now_us, sample);
            return false;
        }
        if((delta < 0) &&
           (encoder->continuous_count <
            (std::numeric_limits<int64_t>::min() - delta)))
        {
            increment(&encoder->jump_error_count);
            clear_sample(encoder, now_us, sample);
            return false;
        }
        encoder->previous_raw_count = raw_count;
        encoder->last_raw_count = raw_count;
        encoder->continuous_count += static_cast<int64_t>(delta);
        encoder->last_valid_timestamp_us = now_us;
    }

    clear_sample(encoder, now_us, sample);
    sample->valid = true;
    return true;
}

bool vehicle_steering_encoder_is_fresh(
    const VehicleSteeringEncoder *encoder, uint64_t now_us,
    uint64_t timeout_us)
{
    if((encoder == nullptr) || !encoder->initialized ||
       (encoder->last_valid_timestamp_us == 0U) ||
       (now_us < encoder->last_valid_timestamp_us))
    {
        return false;
    }
    if(timeout_us == 0U)
    {
        timeout_us = encoder->config.timeout_us;
    }
    return (now_us - encoder->last_valid_timestamp_us) <= timeout_us;
}
