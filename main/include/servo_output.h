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

// 标定模式专用的一组舵机输出。
//
// 这一条链只用于“安装中位校准”：
// - 所有关节都直接按安装层给出的基础角输出
// - 不再对小腿额外叠加运行态的真实机构修正公式
//
// 这样当目标角是 90/90/90 时，所有舵机真实输出都会停在安装中位附近，
// 便于通过最后一步的 offset 去修正装配误差。
void servo_output_set_calibration_group_angles_deg(const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG]);

// 计算标定模式下的一组最终真实舵机角。
//
// 与 servo_output_set_calibration_group_angles_deg() 使用同一条“标定专用输出语义”，
// 但这里只做计算，不实际驱动舵机。
void servo_output_compute_calibration_actual_group_angles_deg(
    const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG],
    float out_actual_angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG]);

// 打印小腿在“安装层基础角 -> 舵机层修正 -> 最终实际角”这一整条链路。
//
// 这个函数只用于定位小腿方向/角度问题，不参与正式控制逻辑。
void servo_output_log_calf_pipeline(int leg, float thigh_base_angle_deg, float calf_base_angle_deg);

// 计算当前一组基础角在真实执行层映射后的最终舵机角。
//
// 这个接口不直接发给 PCA9685，只做“基础角 -> 最终真实角”的换算，
// 主要用于模式切换时把“当前真实输出姿态”作为过渡起点。
void servo_output_compute_actual_group_angles_deg(
    const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG],
    float out_actual_angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG]);

// 直接按“最终真实舵机角”输出一组角度。
//
// 这个接口不会再做：
// - 小腿运行态修正
// - servo_sign 镜像
// - 标定偏置叠加
//
// 它只负责把调用方已经算好的最终真实角，直接发给 PCA9685。
// 主要用于模式切换时在“真实舵机角空间”里做平滑过渡。
void servo_output_set_actual_group_angles_deg(
    const float actual_angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG]);
