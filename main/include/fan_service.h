#pragma once

#include <stdbool.h>
#include <stdint.h>

// ── 风扇速度档位 ──────────────────────────────────────────────────────────────
typedef enum {
    FAN_SPEED_OFF    = 0,   // 停转
    FAN_SPEED_LOW    = 1,   // 低速（待机散热）
    FAN_SPEED_MEDIUM = 2,   // 中速（行走时自动档）
    FAN_SPEED_HIGH   = 3,   // 高速（强制散热或手动最大档）
} fan_speed_level_t;

// ── 风扇运行状态（只读观察） ──────────────────────────────────────────────────
typedef struct {
    bool manual_override;          // 是否处于手动控制模式（true = 忽略自动逻辑）
    bool rc_link_up;               // 遥控链路是否正常（影响自动档策略）
    bool walk_mode;                // 是否处于行走模式（行走时自动升速）
    bool at_min_height;            // 机器人是否处于最低高度（趴下状态，降速）
    bool tach_ready;               // 转速测量是否有效（需要至少一个脉冲）
    fan_speed_level_t manual_level;     // 手动设定的目标档位
    fan_speed_level_t effective_level;  // 当前实际执行的档位（自动或手动）
    uint32_t rpm;                  // 当前测量转速（转/分钟）
} fan_status_t;

void fan_service_init(void);
void fan_service_tick(void);
void fan_service_set_rc_link(bool link_up);
void fan_service_set_motion_mode(bool walk_mode);
void fan_service_set_at_min_height(bool at_min_height);
void fan_service_set_manual_level(fan_speed_level_t level);
void fan_service_clear_manual_override(void);
// 站立模式下的高度比例（0.0 = 最低高度，1.0 = 最高高度）。
// 风扇在站立模式下按此比例在 LOW~MEDIUM 之间线性插值占空比。
void fan_service_set_stand_height_ratio(float ratio);
fan_status_t fan_service_get_status(void);
