#pragma once

#include <stdbool.h>

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    bool ready;
} attitude_state_t;

void imu_service_init(void);
void imu_service_tick(void);
attitude_state_t imu_service_get_attitude(void);
void imu_service_zero_yaw(void);
