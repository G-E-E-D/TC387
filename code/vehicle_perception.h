#ifndef VEHICLE_PERCEPTION_H
#define VEHICLE_PERCEPTION_H

#include "vehicle_types.h"

void perception_init();
void perception_update();
bool perception_get_stage1_command(Stage1ControlCommand *command);
void stage1_set_external_command(const Stage1ControlCommand *command);
void stage1_invalidate_external_command();
uint32_t perception_get_command_generation();

#endif
