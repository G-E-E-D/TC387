#include "vehicle_mt6701.h"

#include <stddef.h>

#include "vehicle_config.h"
#include "vehicle_math.h"

#define VEHICLE_MT6701_FRAME_MASK UINT32_C(0x00FFFFFF)
#define VEHICLE_MT6701_DATA_MASK  UINT32_C(0x0003FFFF)
#define VEHICLE_MT6701_CRC_MASK   UINT32_C(0x0000003F)

static void vehicle_mt6701_increment(uint32_t *counter)
{
    if((counter != NULL) && (*counter < UINT32_MAX))
    {
        ++(*counter);
    }
}

static bool vehicle_mt6701_add_count(int64_t count, int32_t delta,
                                     int64_t *result)
{
    if(result == NULL)
    {
        return false;
    }
    if((delta > 0) && (count > (INT64_MAX - (int64_t)delta)))
    {
        return false;
    }
    if((delta < 0) && (count < (INT64_MIN - (int64_t)delta)))
    {
        return false;
    }
    *result = count + (int64_t)delta;
    return true;
}

static void vehicle_mt6701_fill_sample(const VehicleMt6701 *sensor,
                                       uint64_t timestamp_us, bool valid,
                                       SteeringSample *sample)
{
    int64_t relative_count;

    if(sample == NULL)
    {
        return;
    }

    relative_count = sensor->continuous_count - sensor->relative_zero_count;
    sample->timestamp_us = timestamp_us;
    sample->raw_angle = sensor->previous_raw_angle;
    sample->magnetic_status = sensor->last_magnetic_status;
    sample->continuous_count = sensor->continuous_count;
    sample->relative_count = relative_count;
    sample->angle_rad = (float)relative_count * VEHICLE_TWO_PI_F /
                        (float)VEHICLE_MT6701_COUNTS_PER_REVOLUTION;
    sample->valid = valid;
}

uint8_t vehicle_mt6701_crc6(uint32_t angle_and_status_18bits)
{
    uint8_t crc;
    unsigned int bit_index;

    crc = 0U;
    angle_and_status_18bits &= VEHICLE_MT6701_DATA_MASK;
    for(bit_index = 18U; bit_index > 0U; --bit_index)
    {
        uint8_t input_bit;
        uint8_t feedback;

        input_bit = (uint8_t)((angle_and_status_18bits >>
                               (bit_index - 1U)) & UINT32_C(1));
        feedback = (uint8_t)(((crc >> 5U) & UINT8_C(1)) ^ input_bit);
        crc = (uint8_t)((crc << 1U) & UINT8_C(0x3F));
        if(feedback != 0U)
        {
            crc ^= UINT8_C(0x03);
        }
    }
    return crc;
}

bool vehicle_mt6701_parse_ssi_frame(uint32_t frame_24bits,
                                    VehicleMt6701Frame *decoded)
{
    uint32_t payload;

    if(decoded == NULL)
    {
        return false;
    }

    decoded->raw_angle = 0U;
    decoded->magnetic_status = 0U;
    decoded->received_crc = 0U;
    decoded->calculated_crc = 0U;
    decoded->crc_valid = false;

    if((frame_24bits & ~VEHICLE_MT6701_FRAME_MASK) != 0U)
    {
        return false;
    }

    /* The first SSI bit shifted in is represented as bit 23. */
    payload = (frame_24bits >> 6U) & VEHICLE_MT6701_DATA_MASK;
    decoded->raw_angle = (uint16_t)((payload >> 4U) &
                                    VEHICLE_MT6701_ANGLE_MASK);
    decoded->magnetic_status = (uint8_t)(payload & UINT32_C(0x0F));
    decoded->received_crc = (uint8_t)(frame_24bits &
                                      VEHICLE_MT6701_CRC_MASK);
    decoded->calculated_crc = vehicle_mt6701_crc6(payload);
    decoded->crc_valid = decoded->received_crc == decoded->calculated_crc;
    return decoded->crc_valid;
}

bool vehicle_mt6701_magnetic_status_valid(uint8_t magnetic_status)
{
    uint8_t invalid_mask;

    invalid_mask = VEHICLE_MT6701_STATUS_FIELD_MASK |
                   VEHICLE_MT6701_STATUS_TRACK_LOSS_MASK;
    return ((magnetic_status & UINT8_C(0xF0)) == 0U) &&
           ((magnetic_status & invalid_mask) == 0U);
}

int32_t vehicle_mt6701_delta14(uint16_t current_angle,
                               uint16_t previous_angle)
{
    int32_t delta;

    delta = (int32_t)(current_angle & VEHICLE_MT6701_ANGLE_MASK) -
            (int32_t)(previous_angle & VEHICLE_MT6701_ANGLE_MASK);
    if(delta >= INT32_C(8192))
    {
        delta -= INT32_C(16384);
    }
    else if(delta < -INT32_C(8192))
    {
        delta += INT32_C(16384);
    }
    return delta;
}

void vehicle_mt6701_init(VehicleMt6701 *sensor)
{
    if(sensor != NULL)
    {
        sensor->previous_raw_angle = 0U;
        sensor->last_magnetic_status = 0U;
        sensor->continuous_count = 0;
        sensor->relative_zero_count = 0;
        sensor->last_observation_timestamp_us = 0U;
        sensor->last_valid_timestamp_us = 0U;
        sensor->communication_error_count = 0U;
        sensor->consecutive_communication_errors = 0U;
        sensor->magnetic_error_count = 0U;
        sensor->jump_error_count = 0U;
        sensor->initialized = false;
    }
}

bool vehicle_mt6701_set_relative_zero(VehicleMt6701 *sensor)
{
    if((sensor == NULL) || !sensor->initialized)
    {
        return false;
    }
    sensor->relative_zero_count = sensor->continuous_count;
    return true;
}

bool vehicle_mt6701_update_angle(VehicleMt6701 *sensor, uint16_t raw_angle,
                                 uint8_t magnetic_status,
                                 uint64_t timestamp_us,
                                 bool communication_ok,
                                 SteeringSample *sample)
{
    int32_t delta;
    int64_t next_count;

    if((sensor == NULL) || (sample == NULL))
    {
        return false;
    }

    if((sensor->last_observation_timestamp_us != 0U) &&
       (timestamp_us <= sensor->last_observation_timestamp_us))
    {
        communication_ok = false;
    }

    if(!communication_ok ||
       ((raw_angle & (uint16_t)~VEHICLE_MT6701_ANGLE_MASK) != 0U) ||
       ((magnetic_status & UINT8_C(0xF0)) != 0U))
    {
        vehicle_mt6701_increment(&sensor->communication_error_count);
        vehicle_mt6701_increment(&sensor->consecutive_communication_errors);
        vehicle_mt6701_fill_sample(sensor, timestamp_us, false, sample);
        return false;
    }

    sensor->last_observation_timestamp_us = timestamp_us;
    sensor->consecutive_communication_errors = 0U;

    if(!vehicle_mt6701_magnetic_status_valid(magnetic_status))
    {
        sensor->previous_raw_angle = raw_angle;
        sensor->last_magnetic_status = magnetic_status;
        vehicle_mt6701_increment(&sensor->magnetic_error_count);
        vehicle_mt6701_fill_sample(sensor, timestamp_us, false, sample);
        return false;
    }

    if(!sensor->initialized)
    {
        sensor->previous_raw_angle = raw_angle;
        sensor->last_magnetic_status = magnetic_status;
        sensor->continuous_count = (int64_t)raw_angle;
        sensor->relative_zero_count = sensor->continuous_count;
        sensor->last_valid_timestamp_us = timestamp_us;
        sensor->initialized = true;
        vehicle_mt6701_fill_sample(sensor, timestamp_us, true, sample);
        return true;
    }

    delta = vehicle_mt6701_delta14(raw_angle, sensor->previous_raw_angle);
    sensor->previous_raw_angle = raw_angle;
    sensor->last_magnetic_status = magnetic_status;

    if((delta > MT6701_MAX_DELTA_COUNT_PER_SAMPLE) ||
       (delta < -MT6701_MAX_DELTA_COUNT_PER_SAMPLE) ||
       !vehicle_mt6701_add_count(sensor->continuous_count, delta,
                                 &next_count))
    {
        vehicle_mt6701_increment(&sensor->jump_error_count);
        vehicle_mt6701_fill_sample(sensor, timestamp_us, false, sample);
        return false;
    }

    sensor->continuous_count = next_count;
    sensor->last_valid_timestamp_us = timestamp_us;
    vehicle_mt6701_fill_sample(sensor, timestamp_us, true, sample);
    return true;
}

bool vehicle_mt6701_update_ssi(VehicleMt6701 *sensor, uint32_t frame_24bits,
                               uint64_t timestamp_us,
                               bool communication_ok,
                               SteeringSample *sample)
{
    VehicleMt6701Frame decoded;
    bool frame_valid;

    frame_valid = vehicle_mt6701_parse_ssi_frame(frame_24bits, &decoded);
    return vehicle_mt6701_update_angle(sensor, decoded.raw_angle,
                                       decoded.magnetic_status, timestamp_us,
                                       communication_ok && frame_valid,
                                       sample);
}

bool vehicle_mt6701_is_fresh(const VehicleMt6701 *sensor, uint64_t now_us,
                             uint64_t timeout_us)
{
    if((sensor == NULL) || !sensor->initialized ||
       (now_us < sensor->last_valid_timestamp_us))
    {
        return false;
    }
    return (now_us - sensor->last_valid_timestamp_us) <= timeout_us;
}
