#include "vehicle_perception.h"

#include "vehicle_config.h"
#include "vehicle_math.h"

#include <stddef.h>

static Stage1ControlCommand g_external_command;
static uint32_t g_command_generation;

void perception_init(void)
{
    g_external_command.target_speed_mps = 0.0f;
    g_external_command.target_steering_rad = 0.0f;
    g_external_command.valid = false;
    g_command_generation = 0U;
}

void perception_update(void)
{
    /* The integration boundary is complete; recognition is supplied later. */
}

bool perception_get_stage1_command(Stage1ControlCommand *command)
{
    if(command == NULL)
    {
        return false;
    }
    *command = g_external_command;
    return command->valid;
}

void stage1_set_external_command(const Stage1ControlCommand *command)
{
    if((command == NULL) || !command->valid ||
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

void stage1_invalidate_external_command(void)
{
    g_external_command.target_speed_mps = 0.0f;
    g_external_command.target_steering_rad = 0.0f;
    g_external_command.valid = false;
    if(g_command_generation < UINT32_MAX)
    {
        g_command_generation++;
    }
}

uint32_t perception_get_command_generation(void)
{
    return g_command_generation;
}
