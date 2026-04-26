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

typedef enum {
    ROBOT_WALK_STATUS_IDLE = 0,
    ROBOT_WALK_STATUS_START,
    ROBOT_WALK_STATUS_STEADY,
    ROBOT_WALK_STATUS_STOP,
} robot_walk_status_t;

typedef enum {
    ROBOT_WALK_LEG_STATE_IDLE = 0,
    ROBOT_WALK_LEG_STATE_START,
    ROBOT_WALK_LEG_STATE_SWING,
    ROBOT_WALK_LEG_STATE_SUPPORT,
    ROBOT_WALK_LEG_STATE_STOP,
} robot_walk_leg_state_t;

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
void robot_control_update_imu(const imu_config_t *imu);
void robot_control_control_tick(const rc_command_t *command, const attitude_state_t *attitude);
void robot_control_set_action(robot_action_t action);
robot_mode_t robot_control_get_mode(void);
robot_target_pose_t robot_control_get_target_pose(const rc_command_t *command);
void robot_control_set_motion_speed(robot_motion_speed_t speed);
robot_motion_speed_t robot_control_get_motion_speed(void);
void robot_control_cancel_calibration_mode(void);
// 查询当前是否正处于标定模式。
//
// Web 标定页需要用这个状态区分两种情况：
// 1. 还没进入标定模式：更新偏置后需要显式进入中位
// 2. 已经在标定模式：更新偏置后不应再次触发“进入标定”的过渡
bool robot_control_is_calibration_mode_active(void);
bool robot_control_get_leg_target_angles(int leg, float out_angles_deg[SCARGO_JOINTS_PER_LEG]);
bool robot_control_get_current_feet_world(vec3f_t out_feet_world[SCARGO_LEG_COUNT]);
body_pose_t robot_control_get_current_body_pose(void);
float robot_control_get_current_height_mm(void);
/*
 * Walk 调试接口
 * ==============
 *
 * 这组接口专门给 OLED 和调试日志使用，用来统一读取当前步态状态。
 * 它们只暴露“状态观察值”，不参与控制决策。
 *
 * - status       : start / steady / stop / idle
 * - phase        : 当前稳态周期相位，范围 [0, 1)
 * - leg states   : 每条腿当前属于 start / swing / support / stop
 *
 * 这样可以保证：
 * - OLED 下方状态显示
 * - 串口慢速调试日志
 * 使用的是同一份状态语义，不会再次出现两边各自推断导致的不一致。
 */
robot_walk_status_t robot_control_get_walk_status(void);
float robot_control_get_walk_phase_value(void);
bool robot_control_get_walk_leg_states(robot_walk_leg_state_t out_states[SCARGO_LEG_COUNT]);
