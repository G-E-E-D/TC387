#ifndef VEHICLE_VISION_CAMERA_H
#define VEHICLE_VISION_CAMERA_H

#include <stdbool.h>
#include <stdint.h>

bool vehicle_vision_camera_init();
const uint8_t *vehicle_vision_camera_acquire(uint32_t *sequence);
void vehicle_vision_camera_release();

#endif

