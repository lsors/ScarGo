#include "shared_i2c.h"

#include "board_defaults.h"
#include "esp_log.h"

static const char *TAG = "shared_i2c";
static i2c_master_bus_handle_t s_bus_handles[2];
static bool s_bus_ready[2];

bool shared_i2c_init_bus(uint8_t bus_index)
{
    if (bus_index >= 2) {
        return false;
    }
    if (s_bus_ready[bus_index]) {
        return true;
    }

    const scargo_gpio_map_t *pins = board_defaults_gpio_map();
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = bus_index == 0 ? pins->pca9685_i2c_port : pins->mpu_oled_i2c_port,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (bus_index == 0) {
        bus_config.sda_io_num = pins->i2c0_sda;
        bus_config.scl_io_num = pins->i2c0_scl;
    } else {
        bus_config.sda_io_num = pins->i2c1_sda;
        bus_config.scl_io_num = pins->i2c1_scl;
    }

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus_handles[bus_index]);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Create bus %u failed: %s", bus_index, esp_err_to_name(err));
        return false;
    }

    s_bus_ready[bus_index] = true;
    return true;
}

bool shared_i2c_add_device(uint8_t bus_index, uint16_t address, uint32_t speed_hz, i2c_master_dev_handle_t *out_handle)
{
    if (!shared_i2c_init_bus(bus_index)) {
        return false;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = speed_hz,
    };

    esp_err_t err = i2c_master_bus_add_device(s_bus_handles[bus_index], &device_config, out_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Add device 0x%02x on bus %u failed: %s", address, bus_index, esp_err_to_name(err));
        return false;
    }

    return true;
}

bool shared_i2c_probe(uint8_t bus_index, uint16_t address)
{
    if (!shared_i2c_init_bus(bus_index)) {
        return false;
    }

    esp_err_t err = i2c_master_probe(s_bus_handles[bus_index], address, 20);
    if (err != ESP_OK) {
        return false;
    }
    return true;
}
