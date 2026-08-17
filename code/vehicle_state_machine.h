#ifndef VEHICLE_STATE_MACHINE_H
#define VEHICLE_STATE_MACHINE_H

#include "vehicle_types.h"

#include <stdbool.h>
#include <stdint.h>

struct VehicleStateMachineEvents
{
    bool boot_complete;
    bool critical_drivers_ready;
    bool sensors_calibrated;
    bool calibration_valid;
    bool request_calibration_mode;
    bool request_idle;
    bool request_stage1_start;
    bool request_stage1_stop;
    bool request_stage2_start;
    bool request_stop;
    bool request_fault_reset;
    bool stage1_limit_reached;
    bool path_processing_complete;
    bool path_valid;
    bool reverse_finished;
    bool fault_active;
};

struct VehicleStateMachine
{
    VehicleState state;
    VehicleState previous_state;
    uint64_t state_entry_timestamp_us;
    uint32_t transition_count;
    bool entry_pending;
};

void vehicle_state_machine_init(VehicleStateMachine *machine,
                                uint64_t timestamp_us);
bool vehicle_state_machine_update(VehicleStateMachine *machine,
                                  const VehicleStateMachineEvents *events,
                                  uint64_t timestamp_us);
bool vehicle_state_machine_take_entry(VehicleStateMachine *machine,
                                      VehicleState *entered_state);
bool vehicle_state_is_automatic(VehicleState state);

#endif
