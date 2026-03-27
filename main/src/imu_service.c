#include "imu_service.h"

#include <math.h>

#include "board_defaults.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "shared_i2c.h"

#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

static const char *TAG = "imu";
static attitude_state_t s_attitude;
static i2c_master_dev_handle_t s_device;
static bool s_ready;
static uint16_t s_device_addr;

static bool write_register(uint8_t reg, uint8_t value)
{
    uint8_t payload[2] = {reg, value};
    return i2c_master_transmit(s_device, payload, sizeof(payload), -1) == ESP_OK;
}

static bool read_registers(uint8_t reg, uint8_t *buffer, size_t size)
{
    return i2c_master_transmit_receive(s_device, &reg, 1, buffer, size, -1) == ESP_OK;
}

static float gyro_to_dps(int16_t raw)
{
    return (float)raw / 65.5f;
}

static float accel_to_g(int16_t raw)
{
    return (float)raw / 16384.0f;
}

void imu_service_init(void)
{
    s_attitude = (attitude_state_t){0};

    if (shared_i2c_probe(1, SCARGO_MPU6050_ADDR)) {
        s_device_addr = SCARGO_MPU6050_ADDR;
    } else if (shared_i2c_probe(1, SCARGO_MPU6050_ADDR_ALT)) {
        s_device_addr = SCARGO_MPU6050_ADDR_ALT;
    } else {
        ESP_LOGW(TAG, "MPU6050 not detected on I2C_1 (tried 0x%02x and 0x%02x)", SCARGO_MPU6050_ADDR, SCARGO_MPU6050_ADDR_ALT);
        return;
    }

    if (!shared_i2c_add_device(1, s_device_addr, 400000, &s_device)) {
        ESP_LOGW(TAG, "MPU6050 device add failed at 0x%02x", s_device_addr);
        return;
    }

    if (!write_register(MPU6050_REG_PWR_MGMT_1, 0x01) ||
        !write_register(MPU6050_REG_ACCEL_CONFIG, 0x00) ||
        !write_register(MPU6050_REG_GYRO_CONFIG, 0x08)) {
        ESP_LOGW(TAG, "MPU6050 initialization failed");
        return;
    }

    s_ready = true;
    ESP_LOGI(TAG, "MPU6050 service initialized at 100 Hz addr=0x%02x", s_device_addr);
}

void imu_service_tick(void)
{
    if (!s_ready) {
        return;
    }

    uint8_t raw[14];
    if (!read_registers(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        return;
    }

    int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

    float ax = accel_to_g(ax_raw);
    float ay = accel_to_g(ay_raw);
    float az = accel_to_g(az_raw);
    float gx = gyro_to_dps(gx_raw);
    float gy = gyro_to_dps(gy_raw);
    float gz = gyro_to_dps(gz_raw);

    const float dt = 1.0f / (float)SCARGO_IMU_RATE_HZ;
    float accel_roll = atan2f(ay, az) * 180.0f / (float)M_PI;
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;
    const float alpha = 0.96f;

    s_attitude.roll_deg = alpha * (s_attitude.roll_deg + gx * dt) + (1.0f - alpha) * accel_roll;
    s_attitude.pitch_deg = alpha * (s_attitude.pitch_deg + gy * dt) + (1.0f - alpha) * accel_pitch;
    s_attitude.yaw_deg += gz * dt;
    s_attitude.roll_rate_dps = gx;
    s_attitude.pitch_rate_dps = gy;
    s_attitude.yaw_rate_dps = gz;
    s_attitude.ready = true;
}

attitude_state_t imu_service_get_attitude(void)
{
    return s_attitude;
}
