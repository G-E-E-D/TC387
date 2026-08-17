#ifndef VEHICLE_LOG_H
#define VEHICLE_LOG_H

#include "vehicle_config.h"
#include "vehicle_types.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef bool (*VehicleLogTryWriteByte)(uint8_t byte, void *context);

struct VehicleLog
{
    uint8_t data[VEHICLE_LOG_BUFFER_SIZE];
    uint32_t read_index;
    uint32_t write_index;
    uint32_t used;
    uint32_t dropped_lines;
    bool csv_enabled;
    bool header_pending;
};

void vehicle_log_init(VehicleLog *log);
void vehicle_log_set_csv_enabled(VehicleLog *log, bool enabled);
bool vehicle_log_is_csv_enabled(const VehicleLog *log);
bool vehicle_log_enqueue_text(VehicleLog *log, const char *text);
bool vehicle_log_enqueue_csv(VehicleLog *log, const VehicleTelemetry *telemetry);
size_t vehicle_log_flush(VehicleLog *log, size_t byte_budget,
                         VehicleLogTryWriteByte writer, void *context);
uint32_t vehicle_log_dropped_lines(const VehicleLog *log);

#endif
