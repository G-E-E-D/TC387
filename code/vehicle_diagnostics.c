#include "vehicle_diagnostics.h"

#include "vehicle_math.h"
#include "vehicle_types.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void set_response(VehicleDiagnostics *diagnostics, const char *text)
{
    if((diagnostics == NULL) || (text == NULL))
    {
        return;
    }
    (void)snprintf(diagnostics->response, sizeof(diagnostics->response), "%s\r\n", text);
    diagnostics->response_pending = true;
}

static bool parse_duty(const char *text, float *duty)
{
    char *end;
    float value;
    if((text == NULL) || (duty == NULL))
    {
        return false;
    }
    value = strtof(text, &end);
    while((*end == ' ') || (*end == '\t'))
    {
        ++end;
    }
    if((end == text) || (*end != '\0') || !vehicle_float_is_finite(value))
    {
        return false;
    }
    *duty = vehicle_clampf(value, -CALIBRATION_TEST_MAX_DUTY,
                          CALIBRATION_TEST_MAX_DUTY);
    return true;
}

static bool parse_stage1_drive(const char *text, float *speed_mps,
                               float *steering_rad)
{
    char *end;
    float speed;
    float steering;

    if((text == NULL) || (speed_mps == NULL) || (steering_rad == NULL))
    {
        return false;
    }
    speed = strtof(text, &end);
    if(end == text)
    {
        return false;
    }
    while((*end == ' ') || (*end == '\t'))
    {
        ++end;
    }
    if(*end == '\0')
    {
        return false;
    }
    text = end;
    steering = strtof(text, &end);
    while((*end == ' ') || (*end == '\t'))
    {
        ++end;
    }
    if((end == text) || (*end != '\0') ||
       !vehicle_float_is_finite(speed) ||
       !vehicle_float_is_finite(steering) ||
       (speed < 0.0f) ||
       (speed > STAGE1_DIAGNOSTIC_MAX_SPEED_MPS) ||
       (fabsf(steering) > STAGE1_DIAGNOSTIC_MAX_STEERING_RAD))
    {
        return false;
    }
    *speed_mps = speed;
    *steering_rad = steering;
    return true;
}

static void queue_action(VehicleDiagnostics *diagnostics,
                         VehicleDiagnosticAction action, float duty)
{
    if(diagnostics->request_pending)
    {
        const VehicleDiagnosticAction pending_action =
            diagnostics->pending_request.action;
        const bool incoming_stop = (action == VEHICLE_DIAG_STOP) ||
                                   (action == VEHICLE_DIAG_STAGE1_STOP);
        const bool pending_global_stop =
            pending_action == VEHICLE_DIAG_STOP;

        /* Stop commands must never be lost behind a motion command.  A
         * pending global STOP also cannot be downgraded by STAGE1 STOP. */
        if(!incoming_stop || pending_global_stop)
        {
            set_response(diagnostics, "ERR command queue busy");
            return;
        }
    }
    diagnostics->pending_request.action = action;
    diagnostics->pending_request.signed_duty = duty;
    diagnostics->pending_request.target_speed_mps = 0.0f;
    diagnostics->pending_request.target_steering_rad = 0.0f;
    diagnostics->request_pending = true;
    set_response(diagnostics, "OK");
}

static void queue_stage1_drive(VehicleDiagnostics *diagnostics,
                               float speed_mps, float steering_rad)
{
    if(diagnostics->request_pending)
    {
        set_response(diagnostics, "ERR command queue busy");
        return;
    }
    diagnostics->pending_request.action = VEHICLE_DIAG_STAGE1_DRIVE;
    diagnostics->pending_request.signed_duty = 0.0f;
    diagnostics->pending_request.target_speed_mps = speed_mps;
    diagnostics->pending_request.target_steering_rad = steering_rad;
    diagnostics->request_pending = true;
    set_response(diagnostics, "OK");
}

static void parse_command(VehicleDiagnostics *diagnostics)
{
    char *command = diagnostics->command;
    char *p;
    float duty;
    float speed_mps;
    float steering_rad;
    for(p = command; *p != '\0'; ++p)
    {
        *p = (char)toupper((unsigned char)*p);
    }
    while((*command == ' ') || (*command == '\t'))
    {
        ++command;
    }
    if((*command == '\0') || (strcmp(command, "HELP") == 0))
    {
        set_response(diagnostics,
            "CMDS LOG/ENC/STEER/MOTOR/STAGE1/DRIVE/STAGE2/STOP/CONFIG/FAULT/CAL/IDLE");
    }
    else if(strcmp(command, "LOG START") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_LOG_START, 0.0f);
    }
    else if(strcmp(command, "LOG STOP") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_LOG_STOP, 0.0f);
    }
    else if(strcmp(command, "ENC ZERO") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_ENCODER_ZERO, 0.0f);
    }
    else if(strcmp(command, "STEER ZERO") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_STEERING_ZERO, 0.0f);
    }
    else if(strcmp(command, "STEER LIMIT LEFT") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_STEERING_LIMIT_LEFT, 0.0f);
    }
    else if(strcmp(command, "STEER LIMIT RIGHT") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_STEERING_LIMIT_RIGHT, 0.0f);
    }
    else if(strncmp(command, "MOTOR LEFT ", 11U) == 0)
    {
        if(parse_duty(command + 11, &duty))
            queue_action(diagnostics, VEHICLE_DIAG_TEST_LEFT_MOTOR, duty);
        else set_response(diagnostics, "ERR duty [-0.12,0.12]");
    }
    else if(strncmp(command, "MOTOR RIGHT ", 12U) == 0)
    {
        if(parse_duty(command + 12, &duty))
            queue_action(diagnostics, VEHICLE_DIAG_TEST_RIGHT_MOTOR, duty);
        else set_response(diagnostics, "ERR duty [-0.12,0.12]");
    }
    else if(strncmp(command, "MOTOR STEER ", 12U) == 0)
    {
        if(parse_duty(command + 12, &duty))
            queue_action(diagnostics, VEHICLE_DIAG_TEST_STEERING_MOTOR, duty);
        else set_response(diagnostics, "ERR duty [-0.12,0.12]");
    }
    else if(strcmp(command, "STAGE1 START") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_STAGE1_START, 0.0f);
    }
    else if(strncmp(command, "DRIVE ", 6U) == 0)
    {
        if(parse_stage1_drive(command + 6, &speed_mps, &steering_rad))
        {
            queue_stage1_drive(diagnostics, speed_mps, steering_rad);
        }
        else
        {
            set_response(diagnostics,
                "ERR DRIVE speed[0,0.30] steering[-0.35,0.35]");
        }
    }
    else if(strcmp(command, "STAGE1 STOP") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_STAGE1_STOP, 0.0f);
    }
    else if(strcmp(command, "STAGE2 START") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_STAGE2_START, 0.0f);
    }
    else if(strcmp(command, "STOP") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_STOP, 0.0f);
    }
    else if(strcmp(command, "CONFIG") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_PRINT_CONFIG, 0.0f);
    }
    else if(strcmp(command, "FAULT RESET") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_FAULT_RESET, 0.0f);
    }
    else if(strcmp(command, "CAL MODE") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_CALIBRATION_MODE, 0.0f);
    }
    else if(strcmp(command, "IDLE") == 0)
    {
        queue_action(diagnostics, VEHICLE_DIAG_IDLE, 0.0f);
    }
    else
    {
        set_response(diagnostics, "ERR unknown command; send HELP");
    }
}

void vehicle_diagnostics_init(VehicleDiagnostics *diagnostics)
{
    if(diagnostics != NULL)
    {
        diagnostics->command[0] = '\0';
        diagnostics->length = 0U;
        diagnostics->pending_request.action = VEHICLE_DIAG_NONE;
        diagnostics->pending_request.signed_duty = 0.0f;
        diagnostics->pending_request.target_speed_mps = 0.0f;
        diagnostics->pending_request.target_steering_rad = 0.0f;
        diagnostics->request_pending = false;
        diagnostics->response[0] = '\0';
        diagnostics->response_pending = false;
    }
}

void vehicle_diagnostics_feed_byte(VehicleDiagnostics *diagnostics, uint8_t byte)
{
    if(diagnostics == NULL)
    {
        return;
    }
    if((byte == '\r') || (byte == '\n'))
    {
        if(diagnostics->length > 0U)
        {
            diagnostics->command[diagnostics->length] = '\0';
            parse_command(diagnostics);
            diagnostics->length = 0U;
        }
        return;
    }
    if((byte == 8U) || (byte == 127U))
    {
        if(diagnostics->length > 0U)
        {
            diagnostics->length--;
        }
        return;
    }
    if(isprint((unsigned char)byte) == 0)
    {
        return;
    }
    if(diagnostics->length + 1U >= VEHICLE_COMMAND_BUFFER_SIZE)
    {
        diagnostics->length = 0U;
        set_response(diagnostics, "ERR command too long");
        return;
    }
    diagnostics->command[diagnostics->length++] = (char)byte;
}

bool vehicle_diagnostics_take_request(VehicleDiagnostics *diagnostics,
                                      VehicleDiagnosticRequest *request)
{
    if((diagnostics == NULL) || (request == NULL) || !diagnostics->request_pending)
    {
        return false;
    }
    *request = diagnostics->pending_request;
    diagnostics->request_pending = false;
    diagnostics->pending_request.action = VEHICLE_DIAG_NONE;
    diagnostics->pending_request.signed_duty = 0.0f;
    diagnostics->pending_request.target_speed_mps = 0.0f;
    diagnostics->pending_request.target_steering_rad = 0.0f;
    return true;
}

bool vehicle_diagnostics_take_response(VehicleDiagnostics *diagnostics,
                                       char *response, size_t response_size)
{
    if((diagnostics == NULL) || (response == NULL) || (response_size == 0U) ||
       !diagnostics->response_pending)
    {
        return false;
    }
    (void)snprintf(response, response_size, "%s", diagnostics->response);
    diagnostics->response_pending = false;
    return true;
}

const char *vehicle_state_name(unsigned int state)
{
    static const char *const names[] = {
        "BOOT", "SENSOR_CAL", "IDLE", "STAGE1_RECORD", "STAGE1_DONE",
        "STAGE2_REVERSE", "FINISHED", "FAULT", "CALIBRATION"
    };
    return (state < (sizeof(names) / sizeof(names[0]))) ? names[state] : "UNKNOWN";
}
