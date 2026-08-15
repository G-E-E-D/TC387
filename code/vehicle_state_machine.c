#include "vehicle_state_machine.h"

#include <stddef.h>

static bool transition(VehicleStateMachine *machine, VehicleState next,
                       uint64_t timestamp_us)
{
    if(machine->state == next)
    {
        return false;
    }
    machine->previous_state = machine->state;
    machine->state = next;
    machine->state_entry_timestamp_us = timestamp_us;
    machine->transition_count++;
    machine->entry_pending = true;
    return true;
}

void vehicle_state_machine_init(VehicleStateMachine *machine,
                                uint64_t timestamp_us)
{
    if(machine != NULL)
    {
        machine->state = VEHICLE_STATE_BOOT;
        machine->previous_state = VEHICLE_STATE_BOOT;
        machine->state_entry_timestamp_us = timestamp_us;
        machine->transition_count = 0U;
        machine->entry_pending = true;
    }
}

bool vehicle_state_machine_update(VehicleStateMachine *machine,
                                  const VehicleStateMachineEvents *events,
                                  uint64_t timestamp_us)
{
    VehicleState next;
    if((machine == NULL) || (events == NULL))
    {
        return false;
    }
    next = machine->state;
    if(events->fault_active && (machine->state != VEHICLE_STATE_FAULT))
    {
        next = VEHICLE_STATE_FAULT;
    }
    else
    {
        switch(machine->state)
        {
            case VEHICLE_STATE_BOOT:
                if(events->boot_complete)
                {
                    next = events->critical_drivers_ready
                        ? VEHICLE_STATE_SENSOR_CALIBRATION : VEHICLE_STATE_FAULT;
                }
                break;
            case VEHICLE_STATE_SENSOR_CALIBRATION:
                if(events->request_calibration_mode)
                {
                    next = VEHICLE_STATE_CALIBRATION_MODE;
                }
                else if(events->request_idle)
                {
                    next = VEHICLE_STATE_IDLE;
                }
                else if(events->sensors_calibrated)
                {
                    next = VEHICLE_STATE_IDLE;
                }
                break;
            case VEHICLE_STATE_IDLE:
                if(events->request_calibration_mode)
                {
                    next = VEHICLE_STATE_CALIBRATION_MODE;
                }
                else if(events->request_stage1_start && events->calibration_valid)
                {
                    next = VEHICLE_STATE_STAGE1_RECORD;
                }
                break;
            case VEHICLE_STATE_STAGE1_RECORD:
                if(events->request_stage1_stop || events->request_stop ||
                   events->stage1_limit_reached)
                {
                    next = VEHICLE_STATE_STAGE1_FINISHED;
                }
                break;
            case VEHICLE_STATE_STAGE1_FINISHED:
                if(events->path_processing_complete && !events->path_valid)
                {
                    next = VEHICLE_STATE_FAULT;
                }
                else if(events->request_stage2_start && events->path_valid &&
                        events->calibration_valid)
                {
                    next = VEHICLE_STATE_STAGE2_REVERSE;
                }
                else if(events->request_calibration_mode)
                {
                    next = VEHICLE_STATE_CALIBRATION_MODE;
                }
                break;
            case VEHICLE_STATE_STAGE2_REVERSE:
                if(events->reverse_finished)
                {
                    next = VEHICLE_STATE_FINISHED;
                }
                else if(events->request_stop)
                {
                    next = VEHICLE_STATE_STAGE1_FINISHED;
                }
                break;
            case VEHICLE_STATE_FINISHED:
                if(events->request_idle)
                {
                    next = VEHICLE_STATE_IDLE;
                }
                break;
            case VEHICLE_STATE_FAULT:
                if(events->request_fault_reset && !events->fault_active)
                {
                    next = events->calibration_valid
                        ? VEHICLE_STATE_IDLE : VEHICLE_STATE_CALIBRATION_MODE;
                }
                break;
            case VEHICLE_STATE_CALIBRATION_MODE:
                if(events->request_idle || events->request_stop)
                {
                    next = VEHICLE_STATE_IDLE;
                }
                else if(events->request_stage1_start && events->calibration_valid)
                {
                    next = VEHICLE_STATE_STAGE1_RECORD;
                }
                break;
            default:
                next = VEHICLE_STATE_FAULT;
                break;
        }
    }
    return transition(machine, next, timestamp_us);
}

bool vehicle_state_machine_take_entry(VehicleStateMachine *machine,
                                      VehicleState *entered_state)
{
    if((machine == NULL) || (entered_state == NULL) || !machine->entry_pending)
    {
        return false;
    }
    *entered_state = machine->state;
    machine->entry_pending = false;
    return true;
}

bool vehicle_state_is_automatic(VehicleState state)
{
    return (state == VEHICLE_STATE_STAGE1_RECORD) ||
           (state == VEHICLE_STATE_STAGE2_REVERSE);
}
