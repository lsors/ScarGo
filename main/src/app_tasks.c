#include "app_tasks.h"

#include "display_service.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_service.h"
#include "rc_input.h"
#include "robot_control.h"
#include "web_ui.h"

static const char *TAG = "scargo_tasks";

static void servo_task(void *arg)
{
    (void)arg;
    const TickType_t delay_ticks = pdMS_TO_TICKS(1000 / SCARGO_SERVO_RATE_HZ);

    while (true) {
        rc_command_t command = rc_input_get_latest();
        attitude_state_t attitude = imu_service_get_attitude();
        robot_control_control_tick(&command, &attitude);
        vTaskDelay(delay_ticks);
    }
}

static void imu_task(void *arg)
{
    (void)arg;
    const TickType_t delay_ticks = pdMS_TO_TICKS(1000 / SCARGO_IMU_RATE_HZ);

    while (true) {
        imu_service_tick();
        vTaskDelay(delay_ticks);
    }
}

static void rc_task(void *arg)
{
    (void)arg;
    const TickType_t delay_ticks = pdMS_TO_TICKS(1000 / SCARGO_CONTROL_RATE_HZ);

    while (true) {
        rc_input_tick();
        vTaskDelay(delay_ticks);
    }
}

static void ui_task(void *arg)
{
    (void)arg;
    const TickType_t delay_ticks = pdMS_TO_TICKS(50);

    while (true) {
        display_service_tick();
        vTaskDelay(delay_ticks);
    }
}

void app_tasks_start(const system_config_t *config)
{
    robot_control_init(config);
    imu_service_init();
    rc_input_init();
    display_service_init();
    web_ui_start((system_config_t *)config);
    robot_control_apply_mid_pose();

    ESP_LOGI(TAG, "Creating FreeRTOS tasks");

    xTaskCreatePinnedToCore(servo_task, "servo_task", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 9, NULL, 1);
    xTaskCreatePinnedToCore(rc_task, "rc_task", 4096, NULL, 7, NULL, 0);
    xTaskCreatePinnedToCore(ui_task, "ui_task", 4096, NULL, 3, NULL, 0);
}
