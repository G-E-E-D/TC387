#include "vehicle_perception.h"

#include "vehicle_config.h"
#include "vehicle_math.h"

#if VEHICLE_VISION_ENABLE
#include "vision_shared.h"
#endif

#include <stddef.h>

static Stage1ControlCommand g_external_command;
static uint32_t g_command_generation;
static VehicleGuideTargetTracker g_guide_tracker;
static VehicleGuideTarget g_guide_target;

void perception_init()
{
    g_external_command.target_speed_mps = 0.0f;
    g_external_command.target_steering_rad = 0.0f;
    g_external_command.valid = false;
    g_command_generation = 0U;
    vehicle_guide_target_init(&g_guide_tracker, nullptr);
    g_guide_target = {};
}

void perception_configure(const VehicleGuideTargetConfig *guide_config)
{
    vehicle_guide_target_init(&g_guide_tracker, guide_config);
    g_guide_target = {};
}

void perception_update(uint64_t now_us)
{
#if VEHICLE_VISION_ENABLE
    vision_runtime_snapshot_t snapshot;
    if(vision_shared_read(&snapshot) != 0U)
    {
        static_cast<void>(vehicle_guide_target_update(
            &g_guide_tracker, &snapshot, now_us, &g_guide_target));
    }
    else if(g_guide_target.timestamp_us != 0U &&
            now_us >= g_guide_target.timestamp_us)
    {
        g_guide_target.age_us = now_us - g_guide_target.timestamp_us;
        g_guide_target.fresh = g_guide_target.age_us <=
                               g_guide_tracker.config.target_timeout_us;
        if(!g_guide_target.fresh)
        {
            g_guide_target.valid = false;
        }
    }
#else
    static_cast<void>(now_us);
    g_guide_target = {};
#endif
}

bool perception_get_guide_target(VehicleGuideTarget *target)
{
    if(target == nullptr)
    {
        return false;
    }
    *target = g_guide_target;
    return target->valid;
}

bool perception_get_stage1_command(Stage1ControlCommand *command)
{
    if(command == nullptr)
    {
        return false;
    }
    *command = g_external_command;
    return command->valid;
}

void stage1_set_external_command(const Stage1ControlCommand *command)
{
    if((command == nullptr) || !command->valid ||
       !vehicle_float_is_finite(command->target_speed_mps) ||
       !vehicle_float_is_finite(command->target_steering_rad))
    {
        stage1_invalidate_external_command();
        return;
    }
    g_external_command.target_speed_mps =
        vehicle_clampf(command->target_speed_mps, 0.0f, STAGE1_MAX_SPEED_MPS);
    g_external_command.target_steering_rad =
        vehicle_clampf(command->target_steering_rad,
                       -STAGE1_MAX_STEERING_RAD,
                       STAGE1_MAX_STEERING_RAD);
    g_external_command.valid = true;
    if(g_command_generation < UINT32_MAX)
    {
        g_command_generation++;
    }
}

void stage1_invalidate_external_command()
{
    g_external_command.target_speed_mps = 0.0f;
    g_external_command.target_steering_rad = 0.0f;
    g_external_command.valid = false;
    if(g_command_generation < UINT32_MAX)
    {
        g_command_generation++;
    }
}

uint32_t perception_get_command_generation()
{
    return g_command_generation;
}
