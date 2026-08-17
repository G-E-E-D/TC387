#include "vehicle_vision_camera.h"

#include "zf_device_mt9v03x.h"

#include <string.h>

#if defined(__TASKING__)
#pragma section all "cpu1_dsram"
#endif
static uint8_t s_frame[MT9V03X_H][MT9V03X_W];
#if defined(__TASKING__)
#pragma section all restore
#endif

static bool s_initialized;
static bool s_frame_owned;
static uint32_t s_sequence;

bool vehicle_vision_camera_init()
{
    s_initialized = (mt9v03x_init() == 0U);
    s_frame_owned = false;
    s_sequence = 0U;
    return s_initialized;
}

const uint8_t *vehicle_vision_camera_acquire(uint32_t *sequence)
{
    if(!s_initialized || s_frame_owned || (mt9v03x_finish_flag == 0U))
    {
        return nullptr;
    }

    /*
     * The stock driver exposes one completed DMA frame. Copy it into CPU1
     * DSPR before clearing the completion flag; this avoids processing a
     * buffer that the next DMA transfer can overwrite. The camera ISR hooks
     * are installed in user/isr_vision.cpp.
     */
    s_frame_owned = true;
    memcpy(s_frame, mt9v03x_image, sizeof(s_frame));
    __dsync();
    mt9v03x_finish_flag = 0U;
    if(s_sequence < UINT32_MAX)
    {
        s_sequence++;
    }
    if(sequence != nullptr)
    {
        *sequence = s_sequence;
    }
    return &s_frame[0][0];
}

void vehicle_vision_camera_release()
{
    s_frame_owned = false;
}

