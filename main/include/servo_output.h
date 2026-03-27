#pragma once

#include "config_store.h"

void servo_output_init(const calibration_config_t *calibration);
void servo_output_update_calibration(const calibration_config_t *calibration);
void servo_output_apply_mid_pose(void);
void servo_output_set_servo_angle_deg(int leg, int joint, float base_angle_deg);
void servo_output_set_group_angles_deg(const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG]);
