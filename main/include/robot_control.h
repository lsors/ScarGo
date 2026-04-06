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
 * robot_control 模块是四层结构中的“调度层”：
 *
 * 1. 纯几何层
 * 2. 安装层
 * 3. 舵机层
 * 4. 显示层
 *
 * 它的职责是：
 * - 读取遥控、IMU、配置、动作状态机
 * - 维护当前足端/机身目标状态
 * - 把目标状态送入几何层求解
 * - 把求解结果再送到舵机层
 * - 同时给显示层提供“当前足端/机身状态”
 *
 * 它不应该承担：
 * - PCA9685 的真实输出细节
 * - OLED 的投影和画线细节
 * - 预览层自己的几何公式
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
