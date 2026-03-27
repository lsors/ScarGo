#pragma once

#include <stdbool.h>

#include "config_store.h"

bool storage_service_init(void);
bool storage_service_load_all(system_config_t *config);
bool storage_service_save_calibration(const calibration_config_t *config);
bool storage_service_save_gait(const gait_config_t *config);
