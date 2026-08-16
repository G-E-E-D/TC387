#include "vision_shared.h"

#include "IfxCpu.h"

#include <string.h>

constexpr auto VISION_EXPOSURE_PENDING = 0x80000000U;

#if defined(__TASKING__)
#pragma section all "cpu0_dsram"
#endif
IFX_ALIGN(4) volatile vision_tag_result_t g_vision_result;
IFX_ALIGN(4) static volatile uint32_t s_result_sequence;
IFX_ALIGN(4) static volatile uint32_t s_camera_sequence;
IFX_ALIGN(4) static volatile uint32_t s_process_us_last;
IFX_ALIGN(4) static volatile uint32_t s_process_us_max;
IFX_ALIGN(4) static volatile uint32_t s_exposure_request;
IFX_ALIGN(4) static volatile uint32_t s_exposure_status;
static uint16_t s_exposure_failures;
#if defined(__TASKING__)
#pragma section all restore
#endif

void vision_shared_init()
{
    vision_tag_result_t empty{};

    s_result_sequence = 1U;
    __dsync();
    g_vision_result = empty;
    s_camera_sequence = 0U;
    s_process_us_last = 0U;
    s_process_us_max = 0U;
    s_exposure_request = 0U;
    s_exposure_status = 0U;
    s_exposure_failures = 0U;
    __dsync();
    s_result_sequence = 2U;
}

void vision_shared_publish(const vision_tag_result_t *result,
                           uint32_t camera_sequence,
                           uint32_t process_us)
{
    uint32_t sequence;

    if (result == nullptr)
    {
        return;
    }

    sequence = s_result_sequence;
    if ((sequence & 1U) != 0U)
    {
        ++sequence;
    }
    s_result_sequence = sequence + 1U;
    __dsync();
    g_vision_result = *result;
    s_camera_sequence = camera_sequence;
    s_process_us_last = process_us;
    if (process_us > s_process_us_max)
    {
        s_process_us_max = process_us;
    }
    __dsync();
    s_result_sequence = sequence + 2U;
}

uint8_t vision_shared_read(vision_runtime_snapshot_t *snapshot)
{
    uint32_t before;
    uint32_t after;
    uint32_t exposure_status;
    uint32_t attempts;

    if (snapshot == nullptr)
    {
        return 0U;
    }

    for(attempts = 0U; attempts < 64U; ++attempts)
    {
        before = s_result_sequence;
        if ((before & 1U) != 0U)
        {
            continue;
        }
        __dsync();
        snapshot->result = g_vision_result;
        snapshot->camera_sequence = s_camera_sequence;
        snapshot->process_us_last = s_process_us_last;
        snapshot->process_us_max = s_process_us_max;
        __dsync();
        after = s_result_sequence;
        if((before == after) && ((after & 1U) == 0U))
        {
            exposure_status = s_exposure_status;
            snapshot->exposure_applied = static_cast<uint16_t>(exposure_status & 0xFFFFU);
            snapshot->exposure_failures = static_cast<uint16_t>(exposure_status >> 16);
            return 1U;
        }
    }

    /* A stalled publisher must not block the vehicle control loop forever. */
    *snapshot = {};
    return 0U;
}

void vision_shared_request_exposure(uint16_t exposure)
{
    __swap((void *)&s_exposure_request, VISION_EXPOSURE_PENDING | static_cast<uint32_t>(exposure));
}

uint8_t vision_shared_take_exposure_request(uint16_t *exposure)
{
    uint32_t request = __swap((void *)&s_exposure_request, 0U);
    if ((request & VISION_EXPOSURE_PENDING) == 0U)
    {
        return 0U;
    }
    if (exposure != nullptr)
    {
        *exposure = static_cast<uint16_t>(request & 0xFFFFU);
    }
    return 1U;
}

void vision_shared_set_exposure_status(uint16_t exposure, uint8_t success)
{
    if ((success == 0U) && (s_exposure_failures < 0xFFFFU))
    {
        ++s_exposure_failures;
    }
    __swap((void *)&s_exposure_status,
           (static_cast<uint32_t>(s_exposure_failures) << 16) | static_cast<uint32_t>(exposure));
}

