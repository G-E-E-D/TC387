#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"

#include "vehicle_vision_camera.h"
#include "vision_shared.h"
#include "vision_tag_tracker.h"

#pragma section all "cpu1_dsram"

void core1_main()
{
    vision_tag_config_t tracker_config;

    disable_Watchdog();
    interrupt_global_enable(0);
    cpu_wait_event_ready();

    vision_tag_tracker_default_config(&tracker_config);
    /* Distance calibration remains in vehicle_calibration; do not consume the
     * vision package's DFlash pages from the vehicle controller. */
    tracker_config.distance_scale_mm_px = 0U;
    tracker_config.follow_near_mm = 0U;
    tracker_config.follow_far_mm = 0U;
    vision_tag_tracker_init(&tracker_config);

    while(true)
    {
        const uint8_t *frame;
        uint32_t sequence;
        const vision_tag_result_t *result;
        uint32_t process_us;

        frame = vehicle_vision_camera_acquire(&sequence);
        if(frame == nullptr)
        {
            continue;
        }

        system_start();
        result = vision_tag_tracker_process(frame,
                                            MT9V03X_W,
                                            MT9V03X_H,
                                            MT9V03X_W);
        process_us = system_getval_us();
        vision_shared_publish(result, sequence, process_us);
        vehicle_vision_camera_release();
    }
}

#pragma section all restore
