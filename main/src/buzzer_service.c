#include "buzzer_service.h"

#include "board_defaults.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "buzzer";
static gpio_num_t s_buzzer_gpio = GPIO_NUM_NC;
static scargo_buzzer_mode_t s_buzzer_mode = SCARGO_BUZZER_MODE_GPIO;

static const buzzer_beep_step_t BOOT_BEEP_PATTERN[] = {
    {.on_ms = 40, .off_ms = 0},
};

static const buzzer_beep_step_t TEST_BEEP_PATTERN[] = {
    {.on_ms = 120, .off_ms = 0},
};

static const buzzer_beep_step_t PAGE_CONNECT_PATTERN[] = {
    {.on_ms = 35, .off_ms = 55},
    {.on_ms = 35, .off_ms = 0},
};

static const buzzer_beep_step_t ELRS_CONNECT_PATTERN[] = {
    {.on_ms = 45, .off_ms = 70},
    {.on_ms = 45, .off_ms = 0},
};

static const buzzer_beep_step_t SAVE_BEEP_PATTERN[] = {
    {.on_ms = 180, .off_ms = 0},
};

static void buzzer_set_on(bool enabled)
{
    if (s_buzzer_gpio == GPIO_NUM_NC) {
        return;
    }

    if (s_buzzer_mode == SCARGO_BUZZER_MODE_PWM) {
        if (enabled) {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        } else {
            ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
            ESP_ERROR_CHECK(gpio_set_level(s_buzzer_gpio, 0));
        }
        return;
    }

    ESP_ERROR_CHECK(gpio_set_level(s_buzzer_gpio, enabled ? 1 : 0));
}

void buzzer_service_init(void)
{
    const scargo_gpio_map_t *pins = board_defaults_gpio_map();
    s_buzzer_gpio = (gpio_num_t)pins->buzzer;
    s_buzzer_mode = board_defaults_buzzer_mode();

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << pins->buzzer,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
    ESP_ERROR_CHECK(gpio_set_level(s_buzzer_gpio, 0));
    if (s_buzzer_mode == SCARGO_BUZZER_MODE_PWM) {
        ledc_timer_config_t timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_10_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 2000,
            .clk_cfg = LEDC_AUTO_CLK,
        };

        ledc_channel_config_t channel = {
            .gpio_num = pins->buzzer,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };

        ESP_ERROR_CHECK(ledc_timer_config(&timer));
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }

    ESP_LOGI(TAG, "Buzzer mode: %s", s_buzzer_mode == SCARGO_BUZZER_MODE_PWM ? "PWM" : "GPIO");
    buzzer_set_on(false);
}

void buzzer_service_boot_beep(void)
{
    ESP_LOGI(TAG, "Boot beep");
    buzzer_service_play_pattern(BOOT_BEEP_PATTERN, sizeof(BOOT_BEEP_PATTERN) / sizeof(BOOT_BEEP_PATTERN[0]));
}

void buzzer_service_test_beep(void)
{
    ESP_LOGI(TAG, "Test beep");
    buzzer_service_play_pattern(TEST_BEEP_PATTERN, sizeof(TEST_BEEP_PATTERN) / sizeof(TEST_BEEP_PATTERN[0]));
}

void buzzer_service_page_connect_beep(void)
{
    ESP_LOGI(TAG, "Page connect beep");
    buzzer_service_play_pattern(PAGE_CONNECT_PATTERN, sizeof(PAGE_CONNECT_PATTERN) / sizeof(PAGE_CONNECT_PATTERN[0]));
}

void buzzer_service_elrs_connect_beep(void)
{
    ESP_LOGI(TAG, "ELRS connect beep");
    buzzer_service_play_pattern(ELRS_CONNECT_PATTERN, sizeof(ELRS_CONNECT_PATTERN) / sizeof(ELRS_CONNECT_PATTERN[0]));
}

void buzzer_service_save_beep(void)
{
    ESP_LOGI(TAG, "Save beep");
    buzzer_service_play_pattern(SAVE_BEEP_PATTERN, sizeof(SAVE_BEEP_PATTERN) / sizeof(SAVE_BEEP_PATTERN[0]));
}

void buzzer_service_play_pattern(const buzzer_beep_step_t *steps, size_t count)
{
    if (steps == NULL || count == 0) {
        return;
    }

    for (size_t i = 0; i < count; ++i) {
        if (steps[i].on_ms > 0) {
            buzzer_set_on(true);
            vTaskDelay(pdMS_TO_TICKS(steps[i].on_ms));
        }

        buzzer_set_on(false);

        if (steps[i].off_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(steps[i].off_ms));
        }
    }
}
