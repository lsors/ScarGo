#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_defaults.h"

typedef struct {
    int16_t servo_offsets_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG];
} calibration_config_t;

typedef struct {
    float stand_height_default_mm;
    float stand_height_min_mm;
    float stand_height_max_mm;
    float max_body_yaw_deg;
    float max_body_roll_deg;
    float max_body_pitch_deg;
    float step_height_mm;
    float step_height_min_mm;
    float step_height_max_mm;
    int step_cycle_ms;
    int step_cycle_min_ms;
    int step_cycle_max_ms;
    float step_scale;
    float step_scale_min;
    float step_scale_max;
    float stance_ratio;
    float stance_ratio_min;
    float stance_ratio_max;
    float diagonal_phase_offset;
} gait_config_t;

typedef struct {
    int page;
    int leg;
    int leg_view;
    int robot_view;
} oled_config_t;

typedef struct {
    calibration_config_t calibration;
    gait_config_t gait;
    oled_config_t oled;
} system_config_t;

void config_store_set_defaults(system_config_t *config);
bool config_store_validate(system_config_t *config);
