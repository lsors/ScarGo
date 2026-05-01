#pragma once

#include "config_store.h"
#include "imu_service.h"
#include "kinematics.h"
#include "rc_input.h"

// ── 一次性动作指令 ────────────────────────────────────────────────────────────
// 由外部（按键、Web UI）触发，机器人执行完后自动回到当前模式
typedef enum {
    ROBOT_ACTION_NONE         = 0,   // 无动作（默认）
    ROBOT_ACTION_STAND,              // 立刻进入站立状态
    ROBOT_ACTION_LIE_DOWN,           // 趴下/收腿
    ROBOT_ACTION_WALK,               // 切换到行走模式
    ROBOT_ACTION_FORWARD,            // 向前行进一步（调试用）
    ROBOT_ACTION_BACKWARD,           // 向后行进一步（调试用）
    ROBOT_ACTION_SHIFT_LEFT,         // 向左横移一步（调试用）
    ROBOT_ACTION_SHIFT_RIGHT,        // 向右横移一步（调试用）
    ROBOT_ACTION_TURN_LEFT,          // 原地左转一步（调试用）
    ROBOT_ACTION_TURN_RIGHT,         // 原地右转一步（调试用）
} robot_action_t;

// ── 机器人运动模式 ────────────────────────────────────────────────────────────
typedef enum {
    ROBOT_MODE_STAND = 0,   // 站立模式：保持姿态，响应机身旋转/平移指令
    ROBOT_MODE_WALK  = 1,   // 行走模式：执行步态，响应前进/转向/横移指令
} robot_mode_t;

// ── 运动速度档位 ──────────────────────────────────────────────────────────────
// 通过缩放步幅和步频实现慢 / 中 / 快三档
typedef enum {
    ROBOT_MOTION_SPEED_SLOW   = 0,
    ROBOT_MOTION_SPEED_MEDIUM = 1,
    ROBOT_MOTION_SPEED_FAST   = 2,
} robot_motion_speed_t;

// ── 行走状态机 ────────────────────────────────────────────────────────────────
// 描述整体步态的生命周期阶段
typedef enum {
    ROBOT_WALK_STATUS_IDLE   = 0,   // 静止（未行走）
    ROBOT_WALK_STATUS_START,        // 起步过渡阶段（腿从停止位逐步进入步态相位）
    ROBOT_WALK_STATUS_STEADY,       // 稳态行走（所有腿均在正常步态周期内）
    ROBOT_WALK_STATUS_STOP,         // 停步过渡阶段（腿逐步回到初始支撑位）
} robot_walk_status_t;

// ── 单腿步态状态 ──────────────────────────────────────────────────────────────
// 描述每条腿在步态中的当前阶段
typedef enum {
    ROBOT_WALK_LEG_STATE_IDLE    = 0,   // 静止（未参与步态）
    ROBOT_WALK_LEG_STATE_START,         // 起步过渡
    ROBOT_WALK_LEG_STATE_SWING,         // 摆动相（腿离地向目标点移动）
    ROBOT_WALK_LEG_STATE_SUPPORT,       // 支撑相（脚在地上，推进身体）
    ROBOT_WALK_LEG_STATE_STOP,          // 停步过渡
} robot_walk_leg_state_t;

// ── 机身目标姿态 ──────────────────────────────────────────────────────────────
// 由遥控输入映射而来，作为控制层的期望目标
typedef struct {
    float roll_deg;     // 期望横滚角（度），绕 Y 轴
    float pitch_deg;    // 期望俯仰角（度），绕 X 轴
    float yaw_deg;      // 期望偏航角（度），绕 Z 轴
    float height_mm;    // 期望站立高度（mm）
} robot_target_pose_t;

/*
 * robot_control 模块是四层结构中的"调度层"：
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
 * - 同时给显示层提供"当前足端/机身状态"
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
// 2. 已经在标定模式：更新偏置后不应再次触发"进入标定"的过渡
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
 * 它们只暴露"状态观察值"，不参与控制决策。
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
