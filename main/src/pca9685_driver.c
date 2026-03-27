#include "pca9685_driver.h"

#include <math.h>

#include "board_defaults.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "shared_i2c.h"

#define PCA9685_MODE1_REG 0x00
#define PCA9685_PRESCALE_REG 0xFE
#define PCA9685_LED0_ON_L_REG 0x06

#define PCA9685_MODE1_SLEEP 0x10
#define PCA9685_MODE1_AI 0x20
#define PCA9685_MODE1_RESTART 0x80

static const char *TAG = "pca9685";
static i2c_master_dev_handle_t s_device_handle;
static bool s_ready;

static bool write_register(uint8_t reg, uint8_t value)
{
    uint8_t payload[2] = {reg, value};
    esp_err_t err = i2c_master_transmit(s_device_handle, payload, sizeof(payload), -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write register 0x%02x failed: %s", reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool write_pwm(uint8_t channel, uint16_t on_count, uint16_t off_count)
{
    uint8_t reg = PCA9685_LED0_ON_L_REG + (uint8_t)(4 * channel);
    uint8_t payload[5] = {
        reg,
        (uint8_t)(on_count & 0xFF),
        (uint8_t)((on_count >> 8) & 0x0F),
        (uint8_t)(off_count & 0xFF),
        (uint8_t)((off_count >> 8) & 0x0F),
    };
    esp_err_t err = i2c_master_transmit(s_device_handle, payload, sizeof(payload), -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write PWM channel %u failed: %s", channel, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool pca9685_driver_init(void)
{
    if (s_ready) {
        return true;
    }

    const scargo_gpio_map_t *pins = board_defaults_gpio_map();

    if (!shared_i2c_add_device(0, pins->pca9685_i2c_address, 400000, &s_device_handle)) {
        ESP_LOGE(TAG, "Failed to add PCA9685 device");
        return false;
    }

    if (!write_register(PCA9685_MODE1_REG, PCA9685_MODE1_SLEEP)) {
        return false;
    }
    if (!write_register(PCA9685_PRESCALE_REG, 121)) {
        return false;
    }
    if (!write_register(PCA9685_MODE1_REG, PCA9685_MODE1_AI)) {
        return false;
    }
    if (!write_register(PCA9685_MODE1_REG, PCA9685_MODE1_AI | PCA9685_MODE1_RESTART)) {
        return false;
    }

    s_ready = true;
    ESP_LOGI(TAG, "PCA9685 initialized at I2C address 0x%02x", pins->pca9685_i2c_address);
    return true;
}

bool pca9685_driver_set_angle_deg(uint8_t channel, float angle_deg)
{
    if (!s_ready || channel >= SCARGO_SERVO_COUNT) {
        return false;
    }

    float clamped = angle_deg;
    if (clamped < 0.0f) {
        clamped = 0.0f;
    }
    if (clamped > 180.0f) {
        clamped = 180.0f;
    }

    float pulse_us = 500.0f + (clamped / 180.0f) * 2000.0f;
    uint16_t off_count = (uint16_t)lroundf((pulse_us / 20000.0f) * 4096.0f);
    return write_pwm(channel, 0, off_count);
}

bool pca9685_driver_set_all_off(void)
{
    if (!s_ready) {
        return false;
    }

    for (uint8_t channel = 0; channel < SCARGO_SERVO_COUNT; ++channel) {
        if (!write_pwm(channel, 0, 0)) {
            return false;
        }
    }

    return true;
}
