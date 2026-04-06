#pragma once

#include "config_store.h"
#include "imu_service.h"
#include "kinematics.h"
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

typedef enum {
    ROBOT_MOTION_SPEED_SLOW = 0,
    ROBOT_MOTION_SPEED_MEDIUM = 1,
    ROBOT_MOTION_SPEED_FAST = 2,
} robot_motion_speed_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float height_mm;
} robot_target_pose_t;

/*
 * robot_control 模块是三层结构里的“调度层”：
 *
 * - 输入：
 *   遥控、IMU、配置、动作状态机
 *
 * - 输出：
 *   1. 给执行层的目标舵机角
 *   2. 给显示层使用的当前足端/机身状态
 *
 * 它自己不应该承担：
 * - PCA9685 的真实输出细节
 * - OLED 如何画线
 *
 * 这样可以保证：
 * - 纯运动学层只做几何
 * - 执行层只做真实机械映射
 * - 显示层只做预览
 */

void robot_control_init(const system_config_t *config);
void robot_control_apply_mid_pose(void);
void robot_control_update_calibration(const calibration_config_t *calibration);
void robot_control_update_gait(const gait_config_t *gait);
void robot_control_control_tick(const rc_command_t *command, const attitude_state_t *attitude);
void robot_control_set_action(robot_action_t action);
robot_mode_t robot_control_get_mode(void);
robot_target_pose_t robot_control_get_target_pose(const rc_command_t *command);
void robot_control_set_motion_speed(robot_motion_speed_t speed);
robot_motion_speed_t robot_control_get_motion_speed(void);
void robot_control_cancel_calibration_mode(void);
bool robot_control_get_leg_target_angles(int leg, float out_angles_deg[SCARGO_JOINTS_PER_LEG]);
bool robot_control_get_current_feet_world(vec3f_t out_feet_world[SCARGO_LEG_COUNT]);
body_pose_t robot_control_get_current_body_pose(void);
float robot_control_get_current_height_mm(void);
