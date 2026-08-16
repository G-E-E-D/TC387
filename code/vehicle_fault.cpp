#include "vehicle_fault.h"

#include <stddef.h>

void vehicle_fault_init(VehicleFaultManager *manager)
{
    if(manager != nullptr)
    {
        manager->active_flags = VEHICLE_FAULT_NONE;
        manager->latched_flags = VEHICLE_FAULT_NONE;
        manager->first_fault_timestamp_us = 0U;
        manager->last_fault_timestamp_us = 0U;
        manager->immediate_stop = false;
    }
}

void vehicle_fault_latch(VehicleFaultManager *manager, uint32_t flags,
                         uint64_t timestamp_us)
{
    if((manager == nullptr) || (flags == VEHICLE_FAULT_NONE))
    {
        return;
    }
    if(manager->latched_flags == VEHICLE_FAULT_NONE)
    {
        manager->first_fault_timestamp_us = timestamp_us;
    }
    manager->latched_flags |= flags;
    manager->last_fault_timestamp_us = timestamp_us;
}

void vehicle_fault_raise(VehicleFaultManager *manager, uint32_t flags,
                         bool immediate_stop, uint64_t timestamp_us)
{
    if((manager == nullptr) || (flags == VEHICLE_FAULT_NONE))
    {
        return;
    }
    vehicle_fault_latch(manager, flags, timestamp_us);
    manager->active_flags |= flags;
    manager->immediate_stop = manager->immediate_stop || immediate_stop;
}

void vehicle_fault_set_condition(VehicleFaultManager *manager, uint32_t flags,
                                 bool condition, bool immediate_stop,
                                 uint64_t timestamp_us)
{
    if(manager == nullptr)
    {
        return;
    }
    if(condition)
    {
        vehicle_fault_raise(manager, flags, immediate_stop, timestamp_us);
    }
    else
    {
        manager->active_flags &= ~flags;
    }
}

bool vehicle_fault_has_active(const VehicleFaultManager *manager)
{
    return (manager != nullptr) && (manager->active_flags != VEHICLE_FAULT_NONE);
}

bool vehicle_fault_manual_reset(VehicleFaultManager *manager,
                                uint32_t still_present_flags,
                                uint64_t timestamp_us)
{
    if(manager == nullptr)
    {
        return false;
    }
    manager->active_flags = still_present_flags;
    manager->last_fault_timestamp_us = timestamp_us;
    if(still_present_flags != VEHICLE_FAULT_NONE)
    {
        return false;
    }
    manager->immediate_stop = false;
    return true;
}
