#pragma once

#include <stdbool.h>
#include <stdint.h>

bool pca9685_driver_init(void);
bool pca9685_driver_set_angle_deg(uint8_t channel, float angle_deg);
bool pca9685_driver_set_all_off(void);
