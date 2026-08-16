#ifndef VISION_IPS_FAST_H
#define VISION_IPS_FAST_H

#include <stdint.h>

#include "vision_tag_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Optimized native-size MT9V03X grayscale transfer for the SPI IPS200. */
void vision_ips_fast_init(void);
void vision_ips_fast_show_gray(const uint8_t *image,
                               const vision_tag_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* VISION_IPS_FAST_H */


