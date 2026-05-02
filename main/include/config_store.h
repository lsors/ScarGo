#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_defaults.h"

// ── 标定配置 ──────────────────────────────────────────────────────────────────
// 每个关节的安装偏置（度），用于补偿舵机装配时的角度误差
// 正负号含义与舵机安装层一致（正值 = 让关节向"增大方向"偏移）
typedef struct {
    int16_t servo_offsets_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG];
} calibration_config_t;

// ── 步态参数 ──────────────────────────────────────────────────────────────────
typedef struct {
    // 站立高度（mm），由左摇杆上下（throttle）在 min~max 间映射
    float stand_height_default_mm;
    float stand_height_min_mm;
    float stand_height_max_mm;

    // 站立模式机身最大姿态角（度），遥控满舵时对应这个极限
    float max_body_yaw_deg;    // 偏航极限（左摇杆左右）
    float max_body_roll_deg;   // 横滚极限（右摇杆左右）
    float max_body_pitch_deg;  // 俯仰极限（右摇杆上下）

    // 摆动相抬脚高度（mm）
    float step_height_mm;
    float step_height_min_mm;
    float step_height_max_mm;

    // 单步周期（ms）：越小步频越快，受舵机响应和机械极限限制
    int step_cycle_ms;
    int step_cycle_min_ms;
    int step_cycle_max_ms;

    // 步幅缩放比（1.0 = 满幅，0.5 = 半幅）
    float step_scale;
    float step_scale_min;
    float step_scale_max;

    // 支撑相比例（0~1）：一个周期中脚在地面的时间占比，越大越稳越慢
    float stance_ratio;
    float stance_ratio_min;
    float stance_ratio_max;

    // 对角步态中后对角腿相对前对角腿的相位偏移（0 = 标准同步对角步态）
    float diagonal_phase_offset;
} gait_config_t;

// ── OLED 显示配置 ─────────────────────────────────────────────────────────────
typedef struct {
    int page;       // 当前显示页面编号（对应 display_page_t）
    int leg;        // 单腿预览页选中的腿编号（0~3）
    int leg_view;   // 单腿预览视角（对应 display_preview_view_t）
    int robot_view; // 整机预览视角（对应 display_preview_view_t）
} oled_config_t;

// ── IMU 配置 ──────────────────────────────────────────────────────────────────
typedef struct {
    int  mount_rotation_deg;    // IMU 绕 Z 轴 CCW 安装旋转角度（0 / 90 / 180 / 270）
    bool mount_flip;            // IMU 元器件面朝下时置 true（Z 轴反向）
} imu_config_t;

// ── 系统总配置 ────────────────────────────────────────────────────────────────
typedef struct {
    calibration_config_t calibration;
    gait_config_t        gait;
    oled_config_t        oled;
    imu_config_t         imu;
} system_config_t;

void config_store_set_defaults(system_config_t *config);
bool config_store_validate(system_config_t *config);
