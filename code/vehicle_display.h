#ifndef VEHICLE_DISPLAY_H
#define VEHICLE_DISPLAY_H

#include "vehicle_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef void (*VehicleDisplayLineWriter)(uint8_t row, const char *text,
                                         void *context);

typedef struct
{
    uint8_t page;
    uint8_t refreshes_on_page;
} VehicleDisplay;

void vehicle_display_init(VehicleDisplay *display);
void vehicle_display_next_page(VehicleDisplay *display);
void vehicle_display_render(VehicleDisplay *display,
                            const VehicleTelemetry *telemetry,
                            bool calibration_valid,
                            uint32_t log_dropped_lines,
                            VehicleDisplayLineWriter writer, void *context);

#endif
