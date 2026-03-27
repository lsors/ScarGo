#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float throttle;
    float yaw;
    float pitch;
    float roll;
    int16_t aux_sa;
    int16_t aux_sb;
    int16_t aux_sc;
    int16_t aux_sd;
    bool walk_mode;
    uint16_t raw_channels[12];
    bool link_up;
} rc_command_t;

void rc_input_init(void);
void rc_input_tick(void);
rc_command_t rc_input_get_latest(void);
