#include "app_tasks.h"

#include "buzzer_service.h"
#include "cpu_usage_service.h"
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
    bool last_link_up = false;

    while (true) {
        rc_input_tick();
        rc_command_t command = rc_input_get_latest();
        if (!last_link_up && command.link_up) {
            buzzer_service_elrs_connect_beep();
        }
        last_link_up = command.link_up;
        vTaskDelay(delay_ticks);
    }
}

static void ui_task(void *arg)
{
    (void)arg;
    const TickType_t delay_ticks = pdMS_TO_TICKS(50);

    while (true) {
        cpu_usage_service_tick();
        display_service_tick();
        vTaskDelay(delay_ticks);
    }
}

static void buzzer_task(void *arg)
{
    (void)arg;
    const TickType_t delay_ticks = pdMS_TO_TICKS(2000);

    while (true) {
        vTaskDelay(delay_ticks);
        buzzer_service_test_beep();
    }
}

void app_tasks_start(const system_config_t *config)
{
    robot_control_init(config);
    cpu_usage_service_init();
    imu_service_init();
    rc_input_init();
    display_service_init();
    display_service_set_page((display_page_t)config->oled.page);
    display_service_set_leg_preview_selection(config->oled.leg);
    display_service_set_leg_preview_view((display_preview_view_t)config->oled.leg_view);
    display_service_set_robot_preview_view((display_preview_view_t)config->oled.robot_view);
    web_ui_start((system_config_t *)config);

    ESP_LOGI(TAG, "Creating FreeRTOS tasks");

    xTaskCreatePinnedToCore(servo_task, "servo_task", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 9, NULL, 1);
    xTaskCreatePinnedToCore(rc_task, "rc_task", 4096, NULL, 7, NULL, 0);
    xTaskCreatePinnedToCore(ui_task, "ui_task", 4096, NULL, 3, NULL, 0);
    // Keep the test beep task implementation for future hardware revisions,
    // but leave it disabled on the current board.
    // xTaskCreatePinnedToCore(buzzer_task, "buzzer_task", 2048, NULL, 2, NULL, 0);
}
