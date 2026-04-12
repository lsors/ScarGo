#include "fan_service.h"

#include <inttypes.h>
#include <stddef.h>

#include "board_defaults.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "fan";

#define SCARGO_FAN_PWM_FREQ_HZ 25000
#define SCARGO_FAN_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define SCARGO_FAN_PWM_MAX_DUTY ((1 << 10) - 1)
#define SCARGO_FAN_TACH_PULSES_PER_REV 2U
#define SCARGO_FAN_TACH_SAMPLE_MS 500U
#define SCARGO_FAN_LOG_INTERVAL_MS 2000U

static fan_status_t s_status;
static bool s_ready;
static volatile uint32_t s_tach_edge_count;
static uint32_t s_last_tach_edge_count;
static TickType_t s_last_sample_tick;
static TickType_t s_last_log_tick;
static int s_tach_gpio = -1;
static fan_speed_level_t s_last_logged_level;
static bool s_last_logged_manual_override;
static bool s_last_logged_rc_link_up;
static bool s_last_logged_walk_mode;
static bool s_last_logged_at_min_height;

static uint32_t fan_level_to_duty(fan_speed_level_t level)
{
    switch (level) {
    case FAN_SPEED_OFF:
        return 0;
    case FAN_SPEED_LOW:
        return (SCARGO_FAN_PWM_MAX_DUTY * 35U) / 100U;
    case FAN_SPEED_HIGH:
        return SCARGO_FAN_PWM_MAX_DUTY;
    case FAN_SPEED_MEDIUM:
    default:
        return (SCARGO_FAN_PWM_MAX_DUTY * 60U) / 100U;
    }
}

static fan_speed_level_t auto_level_from_context(void)
{
    if (!s_status.rc_link_up) {
        return FAN_SPEED_OFF;
    }
    if (s_status.at_min_height) {
        return FAN_SPEED_OFF;
    }
    return s_status.walk_mode ? FAN_SPEED_MEDIUM : FAN_SPEED_LOW;
}

static void fan_tach_isr_handler(void *arg)
{
    (void)arg;
    s_tach_edge_count++;
}

static void apply_effective_level(void)
{
    fan_speed_level_t level = s_status.manual_override ? s_status.manual_level : auto_level_from_context();
    s_status.effective_level = level;

    if (!s_ready) {
        return;
    }

    uint32_t duty = fan_level_to_duty(level);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));

    bool changed = (level != s_last_logged_level) ||
                   (s_status.manual_override != s_last_logged_manual_override) ||
                   (s_status.rc_link_up != s_last_logged_rc_link_up) ||
                   (s_status.walk_mode != s_last_logged_walk_mode) ||
                   (s_status.at_min_height != s_last_logged_at_min_height);

    if (changed) {
        ESP_LOGI(TAG,
                 "fan level=%d duty=%" PRIu32 " manual=%d rc=%d walk=%d min_height=%d",
                 (int)level,
                 duty,
                 s_status.manual_override,
                 s_status.rc_link_up,
                 s_status.walk_mode,
                 s_status.at_min_height);
        s_last_logged_level = level;
        s_last_logged_manual_override = s_status.manual_override;
        s_last_logged_rc_link_up = s_status.rc_link_up;
        s_last_logged_walk_mode = s_status.walk_mode;
        s_last_logged_at_min_height = s_status.at_min_height;
    }
}

void fan_service_init(void)
{
    const scargo_gpio_map_t *pins = board_defaults_gpio_map();

    s_status = (fan_status_t){
        .manual_override = false,
        .rc_link_up = false,
        .walk_mode = false,
        .at_min_height = true,
        .tach_ready = false,
        .manual_level = FAN_SPEED_OFF,
        .effective_level = FAN_SPEED_OFF,
        .rpm = 0,
    };
    s_tach_edge_count = 0;
    s_last_tach_edge_count = 0;
    s_last_sample_tick = xTaskGetTickCount();
    s_last_log_tick = s_last_sample_tick;
    s_tach_gpio = pins->fan_tach;
    s_last_logged_level = (fan_speed_level_t)-1;
    s_last_logged_manual_override = false;
    s_last_logged_rc_link_up = false;
    s_last_logged_walk_mode = false;
    s_last_logged_at_min_height = false;

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SCARGO_FAN_PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = SCARGO_FAN_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t channel = {
        .gpio_num = pins->fan_pwm,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ESP_ERROR_CHECK(ledc_channel_config(&channel));

    gpio_config_t tach_config = {
        .pin_bit_mask = (1ULL << pins->fan_tach),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&tach_config));

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
    } else {
        err = gpio_isr_handler_add(pins->fan_tach, fan_tach_isr_handler, NULL);
        if (err == ESP_OK) {
            s_status.tach_ready = true;
        } else {
            ESP_LOGW(TAG, "Fan tach ISR add failed: %s", esp_err_to_name(err));
        }
    }

    s_ready = true;
    apply_effective_level();
    ESP_LOGI(TAG,
             "Fan service initialized pwm=%d tach=%d pullup=1 intr=negedge pulses_per_rev=%u sample_ms=%u",
             pins->fan_pwm,
             pins->fan_tach,
             SCARGO_FAN_TACH_PULSES_PER_REV,
             SCARGO_FAN_TACH_SAMPLE_MS);
}

void fan_service_tick(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ticks = now - s_last_sample_tick;
    uint32_t elapsed_ms = (uint32_t)(elapsed_ticks * portTICK_PERIOD_MS);

    if (elapsed_ms < SCARGO_FAN_TACH_SAMPLE_MS) {
        return;
    }

    uint32_t edge_count = s_tach_edge_count;
    uint32_t delta_edges = edge_count - s_last_tach_edge_count;

    if (s_status.tach_ready && elapsed_ms > 0U) {
        s_status.rpm = (delta_edges * 60000U) / (SCARGO_FAN_TACH_PULSES_PER_REV * elapsed_ms);
    } else {
        s_status.rpm = 0;
    }

    uint32_t log_elapsed_ms = (uint32_t)((now - s_last_log_tick) * portTICK_PERIOD_MS);
    if (log_elapsed_ms >= SCARGO_FAN_LOG_INTERVAL_MS) {
        int tach_level = (s_tach_gpio >= 0) ? gpio_get_level((gpio_num_t)s_tach_gpio) : -1;
        ESP_LOGI(TAG,
                 "tach sample edges=%" PRIu32 " delta=%" PRIu32 " rpm=%" PRIu32 " gpio=%d tach_level=%d ready=%d effective=%d",
                 edge_count,
                 delta_edges,
                 s_status.rpm,
                 s_tach_gpio,
                 tach_level,
                 s_status.tach_ready,
                 (int)s_status.effective_level);
        s_last_log_tick = now;
    }

    s_last_tach_edge_count = edge_count;
    s_last_sample_tick = now;
}

void fan_service_set_rc_link(bool link_up)
{
    s_status.rc_link_up = link_up;
    apply_effective_level();
}

void fan_service_set_motion_mode(bool walk_mode)
{
    s_status.walk_mode = walk_mode;
    apply_effective_level();
}

void fan_service_set_at_min_height(bool at_min_height)
{
    s_status.at_min_height = at_min_height;
    apply_effective_level();
}

void fan_service_set_manual_level(fan_speed_level_t level)
{
    if (level < FAN_SPEED_OFF || level > FAN_SPEED_HIGH) {
        return;
    }
    s_status.manual_override = true;
    s_status.manual_level = level;
    apply_effective_level();
}

void fan_service_clear_manual_override(void)
{
    s_status.manual_override = false;
    apply_effective_level();
}

fan_status_t fan_service_get_status(void)
{
    return s_status;
}
