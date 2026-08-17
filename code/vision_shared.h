#ifndef VISION_SHARED_H
#define VISION_SHARED_H

#include <stdint.h>

#include "vision_tag_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    vision_tag_result_t result;
    uint32_t camera_sequence;
    uint32_t process_us_last;
    uint32_t process_us_max;
    uint16_t exposure_applied;
    uint16_t exposure_failures;
} vision_runtime_snapshot_t;

/* Compatibility symbol. Cross-core readers must use vision_shared_read(). */
extern volatile vision_tag_result_t g_vision_result;

void vision_shared_init();
void vision_shared_publish(const vision_tag_result_t *result,
                           uint32_t camera_sequence,
                           uint32_t process_us);
uint8_t vision_shared_read(vision_runtime_snapshot_t *snapshot);

void vision_shared_request_exposure(uint16_t exposure);
uint8_t vision_shared_take_exposure_request(uint16_t *exposure);
void vision_shared_set_exposure_status(uint16_t exposure, uint8_t success);

#ifdef __cplusplus
}
#endif

#endif /* VISION_SHARED_H */


