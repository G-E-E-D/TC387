#ifndef VEHICLE_FAULT_H
#define VEHICLE_FAULT_H

#include <stdbool.h>
#include <stdint.h>

enum VehicleFaultFlag
{
    VEHICLE_FAULT_NONE                  = 0U,
    VEHICLE_FAULT_DRIVER_INIT           = 1UL << 0,
    VEHICLE_FAULT_LEFT_ENCODER_JUMP     = 1UL << 1,
    VEHICLE_FAULT_RIGHT_ENCODER_JUMP    = 1UL << 2,
    VEHICLE_FAULT_LEFT_ENCODER_STALL    = 1UL << 3,
    VEHICLE_FAULT_RIGHT_ENCODER_STALL   = 1UL << 4,
    VEHICLE_FAULT_WHEEL_MISMATCH        = 1UL << 5,
    VEHICLE_FAULT_STEERING_ENCODER_COMM = 1UL << 6,
    VEHICLE_FAULT_STEERING_ENCODER_JUMP = 1UL << 7,
    VEHICLE_FAULT_STEERING_LIMIT        = 1UL << 8,
    VEHICLE_FAULT_STEERING_STALL        = 1UL << 9,
    VEHICLE_FAULT_IMU_COMM              = 1UL << 10,
    VEHICLE_FAULT_IMU_RANGE             = 1UL << 11,
    VEHICLE_FAULT_IMU_TIMEOUT           = 1UL << 12,
    VEHICLE_FAULT_CONTROL_TIMEOUT       = 1UL << 13,
    VEHICLE_FAULT_PATH_OVERFLOW         = 1UL << 14,
    VEHICLE_FAULT_PATH_INVALID          = 1UL << 15,
    VEHICLE_FAULT_PATH_INDEX_LOST       = 1UL << 16,
    VEHICLE_FAULT_TRACKING_ERROR        = 1UL << 17,
    VEHICLE_FAULT_NUMERIC               = 1UL << 18,
    VEHICLE_FAULT_LOG_OVERFLOW          = 1UL << 19,
    VEHICLE_FAULT_STEERING_TIMEOUT      = 1UL << 20,
    VEHICLE_FAULT_STEERING_CENTER       = 1UL << 21,
    VEHICLE_FAULT_VISION_FRAME_STALE    = 1UL << 22,
    VEHICLE_FAULT_VISION_TAG_LOST       = 1UL << 23,
    VEHICLE_FAULT_VISION_TAG_CONFIDENCE = 1UL << 24,
    VEHICLE_FAULT_VISION_POSITION       = 1UL << 25,
    VEHICLE_FAULT_VISION_PROCESS        = 1UL << 26,
    VEHICLE_FAULT_FORWARD_TRACKER       = 1UL << 27
};

/* Source-compatible aliases for historical bench logs/tests. */
constexpr auto VEHICLE_FAULT_MT6701_COMM =
    VEHICLE_FAULT_STEERING_ENCODER_COMM;
constexpr auto VEHICLE_FAULT_MT6701_JUMP =
    VEHICLE_FAULT_STEERING_ENCODER_JUMP;

struct VehicleFaultManager
{
    uint32_t active_flags;
    uint32_t latched_flags;
    uint64_t first_fault_timestamp_us;
    uint64_t last_fault_timestamp_us;
    bool immediate_stop;
};

void vehicle_fault_init(VehicleFaultManager *manager);
void vehicle_fault_latch(VehicleFaultManager *manager, uint32_t flags,
                         uint64_t timestamp_us);
void vehicle_fault_raise(VehicleFaultManager *manager, uint32_t flags,
                         bool immediate_stop, uint64_t timestamp_us);
void vehicle_fault_set_condition(VehicleFaultManager *manager, uint32_t flags,
                                 bool condition, bool immediate_stop,
                                 uint64_t timestamp_us);
bool vehicle_fault_has_active(const VehicleFaultManager *manager);
bool vehicle_fault_manual_reset(VehicleFaultManager *manager,
                                uint32_t still_present_flags,
                                uint64_t timestamp_us);

#endif
