#include "cpu_usage_service.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/task.h"

static cpu_usage_snapshot_t s_snapshot = {
    .core_count = portNUM_PROCESSORS,
    .ready = false,
};
static configRUN_TIME_COUNTER_TYPE s_prev_idle_runtime[portNUM_PROCESSORS];
static configRUN_TIME_COUNTER_TYPE s_prev_total_runtime;
static bool s_initialized;

static bool starts_with_idle(const char *name)
{
    return name != NULL &&
           name[0] == 'I' &&
           name[1] == 'D' &&
           name[2] == 'L' &&
           name[3] == 'E';
}

void cpu_usage_service_init(void)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.core_count = portNUM_PROCESSORS;
    memset(s_prev_idle_runtime, 0, sizeof(s_prev_idle_runtime));
    s_prev_total_runtime = 0;
    s_initialized = false;
}

void cpu_usage_service_tick(void)
{
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    if (task_count == 0) {
        return;
    }

    TaskStatus_t *task_array = calloc(task_count, sizeof(TaskStatus_t));
    if (task_array == NULL) {
        return;
    }

    configRUN_TIME_COUNTER_TYPE total_runtime = 0;
    UBaseType_t actual_count = uxTaskGetSystemState(task_array, task_count, &total_runtime);
    if (actual_count == 0 || total_runtime == 0) {
        free(task_array);
        return;
    }

    configRUN_TIME_COUNTER_TYPE idle_runtime[portNUM_PROCESSORS] = {0};
    for (UBaseType_t i = 0; i < actual_count; ++i) {
        if (!starts_with_idle(task_array[i].pcTaskName)) {
            continue;
        }

        BaseType_t core = -1;
        size_t len = strlen(task_array[i].pcTaskName);
        if (len > 0) {
            char last = task_array[i].pcTaskName[len - 1];
            if (last >= '0' && last < ('0' + portNUM_PROCESSORS)) {
                core = (BaseType_t)(last - '0');
            }
        }

        if (core >= 0 && core < portNUM_PROCESSORS) {
            idle_runtime[core] += task_array[i].ulRunTimeCounter;
        }
    }

    if (s_initialized) {
        configRUN_TIME_COUNTER_TYPE delta_total = total_runtime - s_prev_total_runtime;
        if (delta_total > 0) {
            float per_core_budget = (float)delta_total;
            for (int core = 0; core < portNUM_PROCESSORS; ++core) {
                configRUN_TIME_COUNTER_TYPE delta_idle = idle_runtime[core] - s_prev_idle_runtime[core];
                float usage = 0.0f;
                if (per_core_budget > 0.0f) {
                    usage = 100.0f * (1.0f - ((float)delta_idle / per_core_budget));
                }
                if (usage < 0.0f) {
                    usage = 0.0f;
                } else if (usage > 100.0f) {
                    usage = 100.0f;
                }
                s_snapshot.usage_pct[core] = usage;
            }
            s_snapshot.ready = true;
        }
    }

    memcpy(s_prev_idle_runtime, idle_runtime, sizeof(idle_runtime));
    s_prev_total_runtime = total_runtime;
    s_initialized = true;
    free(task_array);
}

cpu_usage_snapshot_t cpu_usage_service_get_snapshot(void)
{
    return s_snapshot;
}
