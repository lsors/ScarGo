#pragma once

#include "config_store.h"

// 初始化舵机输出层，并缓存当前标定值。
void servo_output_init(const calibration_config_t *calibration);

// 更新标定值；标定页保存或预览后会调用这里。
void servo_output_update_calibration(const calibration_config_t *calibration);

// 直接把全部舵机打到 90 度安装位。
// 主要用于极简测试，不参与正常足端轨迹控制。
void servo_output_apply_mid_pose(void);

// 将“控制层给出的目标角”转换为“最终下发到 PCA9685 的实际舵机角”。
// 这一步会叠加：
// 1. SERVO_MAP 的方向镜像
// 2. 标定偏置
float servo_output_get_actual_angle_deg(int leg, int joint, float base_angle_deg);

// 单路舵机输出。
void servo_output_set_servo_angle_deg(int leg, int joint, float base_angle_deg);

// 一组舵机输出。
// 机器人正常工作时，所有姿态/步态更新最终都会走这里。
void servo_output_set_group_angles_deg(const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG]);
