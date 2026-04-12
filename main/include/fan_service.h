#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FAN_SPEED_OFF = 0,
    FAN_SPEED_LOW = 1,
    FAN_SPEED_MEDIUM = 2,
    FAN_SPEED_HIGH = 3,
} fan_speed_level_t;

typedef struct {
    bool manual_override;
    bool rc_link_up;
    bool walk_mode;
    bool at_min_height;
    bool tach_ready;
    fan_speed_level_t manual_level;
    fan_speed_level_t effective_level;
    uint32_t rpm;
} fan_status_t;

void fan_service_init(void);
void fan_service_tick(void);
void fan_service_set_rc_link(bool link_up);
void fan_service_set_motion_mode(bool walk_mode);
void fan_service_set_at_min_height(bool at_min_height);
void fan_service_set_manual_level(fan_speed_level_t level);
void fan_service_clear_manual_override(void);
fan_status_t fan_service_get_status(void);
