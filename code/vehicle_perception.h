#ifndef VEHICLE_PERCEPTION_H
#define VEHICLE_PERCEPTION_H

#include "vehicle_forward_tracker.h"
#include "vehicle_guide_target.h"
#include "vehicle_types.h"

void perception_init();
void perception_configure(const VehicleGuideTargetConfig *guide_config);
void perception_update(uint64_t now_us);
bool perception_get_guide_target(VehicleGuideTarget *target);
bool perception_get_stage1_command(Stage1ControlCommand *command);
void stage1_set_external_command(const Stage1ControlCommand *command);
void stage1_invalidate_external_command();
uint32_t perception_get_command_generation();

#endif
