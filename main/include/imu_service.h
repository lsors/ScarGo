#pragma once

#include <stdbool.h>

// IMU 输出的机身姿态与角速率
// 坐标系与机身保持一致：X=前后，Y=左右，Z=上下
typedef struct {
    float roll_deg;        // 横滚角（度），绕 Y 轴，左侧倾为正
    float pitch_deg;       // 俯仰角（度），绕 X 轴，前倾为正
    float yaw_deg;         // 偏航角（度），绕 Z 轴，左转为正（由陀螺仪积分，有漂移）
    float roll_rate_dps;   // 横滚角速率（度/秒）
    float pitch_rate_dps;  // 俯仰角速率（度/秒）
    float yaw_rate_dps;    // 偏航角速率（度/秒）
    bool  ready;           // 数据是否有效（IMU 初始化成功且至少完成一次滤波后为 true）
} attitude_state_t;

void imu_service_init(void);
void imu_service_tick(void);
attitude_state_t imu_service_get_attitude(void);
void imu_service_zero_yaw(void);
void imu_service_update_mount(int rotation_deg, bool flip);
