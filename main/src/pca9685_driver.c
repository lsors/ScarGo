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
#define PCA9685_CHANNEL_COUNT 16

/*
 * 预分频计算公式：prescale = round(f_osc / (4096 × f_pwm)) - 1
 * 内部振荡器标称 25 MHz，目标 PWM 频率 50 Hz（20 ms 周期）：
 *   round(25 000 000 / (4096 × 50)) - 1 = round(122.07) - 1 = 121
 *
 * 选择 50 Hz 的原因：
 *   模拟 RC 舵机（MG996R 等）要求 40~60 Hz 更新率；
 *   数字舵机可接受更高频率，但 50 Hz 对两者均安全。
 *
 * 注：内部振荡器存在 ±12% 误差，实际频率约在 44~56 Hz，
 *     标准舵机对此容差范围均能正常响应。
 */
#define PCA9685_PRESCALE_50HZ 121U

/*
 * 脉宽映射（0° → 500 µs，180° → 2500 µs）：
 * 覆盖大多数标准 RC 舵机的全行程范围。
 * 若使用窄行程舵机（1000~2000 µs），需修改下方两个常量。
 */
#define PCA9685_PULSE_MIN_US   500.0f
#define PCA9685_PULSE_RANGE_US 2000.0f

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
    if (!write_register(PCA9685_PRESCALE_REG, PCA9685_PRESCALE_50HZ)) {
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
    if (!s_ready || channel >= PCA9685_CHANNEL_COUNT) {
        return false;
    }

    float clamped = angle_deg;
    if (clamped < 0.0f) {
        clamped = 0.0f;
    }
    if (clamped > 180.0f) {
        clamped = 180.0f;
    }

    float pulse_us = PCA9685_PULSE_MIN_US + (clamped / 180.0f) * PCA9685_PULSE_RANGE_US;
    uint16_t off_count = (uint16_t)lroundf((pulse_us / 20000.0f) * 4096.0f);
    return write_pwm(channel, 0, off_count);
}

bool pca9685_driver_set_all_off(void)
{
    if (!s_ready) {
        return false;
    }

    for (uint8_t channel = 0; channel < PCA9685_CHANNEL_COUNT; ++channel) {
        if (!write_pwm(channel, 0, 0)) {
            return false;
        }
    }

    return true;
}
