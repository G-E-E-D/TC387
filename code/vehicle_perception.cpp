#include "vehicle_perception.h"

#include "vehicle_config.h"
#include "vehicle_math.h"

#if VEHICLE_VISION_ENABLE
#include "vision_shared.h"
#endif

#include <stddef.h>

static Stage1ControlCommand g_external_command;
static uint32_t g_command_generation;

void perception_init()
{
    g_external_command.target_speed_mps = 0.0f;
    g_external_command.target_steering_rad = 0.0f;
    g_external_command.valid = false;
    g_command_generation = 0U;
}

void perception_update()
{
#if VEHICLE_VISION_ENABLE
    vision_runtime_snapshot_t snapshot;
    Stage1ControlCommand command;
    int32_t error_q15;

    /* A predicted/lost frame is deliberately not allowed to drive the car. */
    if((vision_shared_read(&snapshot) == 0U) ||
       (snapshot.result.valid == 0U) ||
       (snapshot.result.predicted > VEHICLE_VISION_MAX_LOST_FRAMES) ||
       (snapshot.result.confidence < VEHICLE_VISION_MIN_CONFIDENCE))
    {
        stage1_invalidate_external_command();
        return;
    }

    error_q15 = static_cast<int32_t>(snapshot.result.error_x_q15);
    command.target_speed_mps = VEHICLE_VISION_TARGET_SPEED_MPS;
    /* error_x_q15 is positive when the tag is right; positive vehicle
     * steering is left, so the visual correction has the opposite sign. */
    command.target_steering_rad = -(static_cast<float>(error_q15) / 32767.0f) *
                                  VEHICLE_VISION_STEERING_RAD_LIMIT;
    command.target_steering_rad = vehicle_clampf(
        command.target_steering_rad,
        -STAGE1_MAX_STEERING_RAD,
        STAGE1_MAX_STEERING_RAD);
    command.valid = true;
    stage1_set_external_command(&command);
#else
    /* Serial DRIVE remains the explicit Stage-1 command source. */
#endif
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
