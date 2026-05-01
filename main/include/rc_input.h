#pragma once

#include <stdbool.h>
#include <stdint.h>

// 归一化遥控指令，由原始 CRSF 通道值换算而来
// 摇杆轴：归一化范围 -1.0 ~ +1.0，经过死区处理
// 拨档轴：原始 CRSF 值（172~1811），用 SCARGO_CRSF_SWITCH_LOW/HIGH 判断档位
typedef struct {
    float    throttle;    // 左摇杆上下（ch2）：+1 = 最高，-1 = 最低；控制站立高度
    float    yaw;         // 左摇杆左右（ch3）：+1 = 左，-1 = 右；站立偏航 / 行走转向
    float    pitch;       // 右摇杆上下（ch1）：+1 = 向前，-1 = 向后；站立俯仰 / 行走前进
    float    roll;        // 右摇杆左右（ch0）：+1 = 左，-1 = 右；站立横滚 / 行走横移
    int16_t  aux_sa;      // 拨档 SA（ch4）：原始值，低/高对应两档
    int16_t  aux_sb;      // 拨档 SB（ch5）：原始值，低/高对应两档
    int16_t  aux_sc;      // 拨档 SC（ch6）：原始值
    int16_t  aux_sd;      // 拨档 SD（ch7）：原始值
    bool     walk_mode;   // 是否处于行走模式（由特定拨档通道组合触发）
    uint16_t raw_channels[12];   // 全部 12 路 CRSF 原始通道值（172~1811）
    bool     link_up;     // 遥控链路是否正常（false = 失控保护）
} rc_command_t;

void rc_input_init(void);
void rc_input_tick(void);
rc_command_t rc_input_get_latest(void);
