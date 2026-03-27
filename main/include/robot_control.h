#pragma once

#include "config_store.h"
#include "imu_service.h"
#include "rc_input.h"

typedef enum {
    ROBOT_ACTION_NONE = 0,
    ROBOT_ACTION_STAND,
    ROBOT_ACTION_LIE_DOWN,
    ROBOT_ACTION_WALK,
    ROBOT_ACTION_FORWARD,
    ROBOT_ACTION_BACKWARD,
    ROBOT_ACTION_SHIFT_LEFT,
    ROBOT_ACTION_SHIFT_RIGHT,
    ROBOT_ACTION_TURN_LEFT,
    ROBOT_ACTION_TURN_RIGHT,
} robot_action_t;

typedef enum {
    ROBOT_MODE_STAND = 0,
    ROBOT_MODE_WALK = 1,
} robot_mode_t;

void robot_control_init(const system_config_t *config);
void robot_control_apply_mid_pose(void);
void robot_control_update_calibration(const calibration_config_t *calibration);
void robot_control_update_gait(const gait_config_t *gait);
void robot_control_control_tick(const rc_command_t *command, const attitude_state_t *attitude);
void robot_control_set_action(robot_action_t action);
robot_mode_t robot_control_get_mode(void);
