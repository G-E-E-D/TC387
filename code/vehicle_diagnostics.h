#ifndef VEHICLE_DIAGNOSTICS_H
#define VEHICLE_DIAGNOSTICS_H

#include "vehicle_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    VEHICLE_DIAG_NONE = 0,
    VEHICLE_DIAG_LOG_START,
    VEHICLE_DIAG_LOG_STOP,
    VEHICLE_DIAG_ENCODER_ZERO,
    VEHICLE_DIAG_STEERING_ZERO,
    VEHICLE_DIAG_STEERING_LIMIT_LEFT,
    VEHICLE_DIAG_STEERING_LIMIT_RIGHT,
    VEHICLE_DIAG_TEST_LEFT_MOTOR,
    VEHICLE_DIAG_TEST_RIGHT_MOTOR,
    VEHICLE_DIAG_TEST_STEERING_MOTOR,
    VEHICLE_DIAG_STAGE1_START,
    VEHICLE_DIAG_STAGE1_STOP,
    VEHICLE_DIAG_STAGE2_START,
    VEHICLE_DIAG_STOP,
    VEHICLE_DIAG_PRINT_CONFIG,
    VEHICLE_DIAG_FAULT_RESET,
    VEHICLE_DIAG_CALIBRATION_MODE,
    VEHICLE_DIAG_IDLE
} VehicleDiagnosticAction;

typedef struct
{
    VehicleDiagnosticAction action;
    float signed_duty;
} VehicleDiagnosticRequest;

typedef struct
{
    char command[VEHICLE_COMMAND_BUFFER_SIZE];
    uint32_t length;
    VehicleDiagnosticRequest pending_request;
    bool request_pending;
    char response[VEHICLE_COMMAND_BUFFER_SIZE];
    bool response_pending;
} VehicleDiagnostics;

void vehicle_diagnostics_init(VehicleDiagnostics *diagnostics);
void vehicle_diagnostics_feed_byte(VehicleDiagnostics *diagnostics, uint8_t byte);
bool vehicle_diagnostics_take_request(VehicleDiagnostics *diagnostics,
                                      VehicleDiagnosticRequest *request);
bool vehicle_diagnostics_take_response(VehicleDiagnostics *diagnostics,
                                       char *response, size_t response_size);
const char *vehicle_state_name(unsigned int state);

#endif
