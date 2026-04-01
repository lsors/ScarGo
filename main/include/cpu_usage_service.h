#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

typedef struct {
    uint8_t core_count;
    bool ready;
    float usage_pct[portNUM_PROCESSORS];
} cpu_usage_snapshot_t;

void cpu_usage_service_init(void);
void cpu_usage_service_tick(void);
cpu_usage_snapshot_t cpu_usage_service_get_snapshot(void);
