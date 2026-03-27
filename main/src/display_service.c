#include "display_service.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "board_defaults.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "imu_service.h"
#include "rc_input.h"
#include "robot_control.h"
#include "shared_i2c.h"

static const char *TAG = "display";
static display_page_t s_page = DISPLAY_PAGE_STATUS;
static i2c_master_dev_handle_t s_oled;
static bool s_ready;
static uint8_t s_buffer[128 * 8];
static uint16_t s_oled_addr;

static const uint8_t FONT_5X7[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5f,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7f,0x14,0x7f,0x14},
    {0x24,0x2a,0x7f,0x2a,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1c,0x22,0x41,0x00},{0x00,0x41,0x22,0x1c,0x00},{0x14,0x08,0x3e,0x08,0x14},{0x08,0x08,0x3e,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3e,0x51,0x49,0x45,0x3e},{0x00,0x42,0x7f,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},
    {0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3c,0x4a,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1e},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3e},{0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},{0x3e,0x41,0x41,0x41,0x22},
    {0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},{0x7f,0x09,0x09,0x09,0x01},{0x3e,0x41,0x49,0x49,0x7a},
    {0x7f,0x08,0x08,0x08,0x7f},{0x00,0x41,0x7f,0x41,0x00},{0x20,0x40,0x41,0x3f,0x01},{0x7f,0x08,0x14,0x22,0x41},
    {0x7f,0x40,0x40,0x40,0x40},{0x7f,0x02,0x0c,0x02,0x7f},{0x7f,0x04,0x08,0x10,0x7f},{0x3e,0x41,0x41,0x41,0x3e},
    {0x7f,0x09,0x09,0x09,0x06},{0x3e,0x41,0x51,0x21,0x5e},{0x7f,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7f,0x01,0x01},{0x3f,0x40,0x40,0x40,0x3f},{0x1f,0x20,0x40,0x20,0x1f},{0x3f,0x40,0x38,0x40,0x3f},
    {0x63,0x14,0x08,0x14,0x63},{0x03,0x04,0x78,0x04,0x03},{0x61,0x51,0x49,0x45,0x43}
};

static bool oled_write_cmd(uint8_t cmd)
{
    uint8_t payload[2] = {0x00, cmd};
    return i2c_master_transmit(s_oled, payload, sizeof(payload), -1) == ESP_OK;
}

static bool oled_write_data(const uint8_t *data, size_t size)
{
    uint8_t payload[17];
    payload[0] = 0x40;

    while (size > 0) {
        size_t chunk = size > 16 ? 16 : size;
        memcpy(&payload[1], data, chunk);
        if (i2c_master_transmit(s_oled, payload, chunk + 1U, -1) != ESP_OK) {
            return false;
        }
        data += chunk;
        size -= chunk;
    }
    return true;
}

static void oled_clear(void)
{
    memset(s_buffer, 0, sizeof(s_buffer));
}

static void oled_draw_char(int x, int page, char ch)
{
    if (page < 0 || page >= 8 || x < 0 || x > 122) {
        return;
    }

    int index = ch - 32;
    if (index < 0 || index >= (int)(sizeof(FONT_5X7) / sizeof(FONT_5X7[0]))) {
        index = 0;
    }
    memcpy(&s_buffer[page * 128 + x], FONT_5X7[index], 5);
}

static void oled_draw_text(int x, int page, const char *text)
{
    while (*text != '\0' && x <= 122) {
        oled_draw_char(x, page, *text++);
        x += 6;
    }
}

static void oled_flush(void)
{
    for (int page = 0; page < 8; ++page) {
        if (!oled_write_cmd((uint8_t)(0xB0 | page)) ||
            !oled_write_cmd(0x00) ||
            !oled_write_cmd(0x10) ||
            !oled_write_data(&s_buffer[page * 128], 128)) {
            ESP_LOGW(TAG, "OLED communication lost, disabling display updates");
            s_ready = false;
            return;
        }
    }
}

static void render_status_page(void)
{
    char line[24];
    attitude_state_t attitude = imu_service_get_attitude();
    rc_command_t command = rc_input_get_latest();

    oled_clear();
    oled_draw_text(0, 0, "SCARGO");
    oled_draw_text(0, 1, robot_control_get_mode() == ROBOT_MODE_WALK ? "MODE WALK" : "MODE STAND");
    snprintf(line, sizeof(line), "THR %+.2f", command.throttle);
    oled_draw_text(0, 2, line);
    snprintf(line, sizeof(line), "R %+.1f", attitude.roll_deg);
    oled_draw_text(0, 3, line);
    snprintf(line, sizeof(line), "P %+.1f", attitude.pitch_deg);
    oled_draw_text(0, 4, line);

    if (s_page == DISPLAY_PAGE_CALIBRATION) {
        oled_draw_text(0, 6, "CALIB READY");
    } else if (s_page == DISPLAY_PAGE_TEST) {
        oled_draw_text(0, 6, "TEST READY");
    } else {
        oled_draw_text(0, 6, "K1 PAGE K2 ACT");
    }
}

static void handle_buttons(void)
{
    static int last_k1 = 1;
    static int last_k2 = 1;
    const scargo_gpio_map_t *pins = board_defaults_gpio_map();
    int k1 = gpio_get_level(pins->button_k1);
    int k2 = gpio_get_level(pins->button_k2);

    if (last_k1 == 1 && k1 == 0) {
        s_page = (display_page_t)(((int)s_page + 1) % 3);
    }
    if (last_k2 == 1 && k2 == 0) {
        if (s_page == DISPLAY_PAGE_CALIBRATION) {
            robot_control_apply_mid_pose();
        } else if (s_page == DISPLAY_PAGE_TEST) {
            robot_control_set_action(robot_control_get_mode() == ROBOT_MODE_WALK ? ROBOT_ACTION_STAND : ROBOT_ACTION_WALK);
        }
    }

    last_k1 = k1;
    last_k2 = k2;
}

void display_service_init(void)
{
    const scargo_gpio_map_t *pins = board_defaults_gpio_map();
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << pins->button_k1) | (1ULL << pins->button_k2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&button_config);

    if (shared_i2c_probe(1, SCARGO_SSD1306_ADDR)) {
        s_oled_addr = SCARGO_SSD1306_ADDR;
    } else if (shared_i2c_probe(1, SCARGO_SSD1306_ADDR_ALT)) {
        s_oled_addr = SCARGO_SSD1306_ADDR_ALT;
    } else {
        ESP_LOGW(TAG, "SSD1306 not detected on I2C_1 (tried 0x%02x and 0x%02x)", SCARGO_SSD1306_ADDR, SCARGO_SSD1306_ADDR_ALT);
        return;
    }

    if (!shared_i2c_add_device(1, s_oled_addr, 400000, &s_oled)) {
        ESP_LOGW(TAG, "OLED add device failed at 0x%02x", s_oled_addr);
        return;
    }

    const uint8_t init_seq[] = {
        0xAE,0x20,0x00,0x40,0xA1,0xC8,0x81,0x7F,0xA6,0xA8,0x3F,
        0xD3,0x00,0xD5,0x80,0xD9,0xF1,0xDA,0x12,0xDB,0x40,0x8D,
        0x14,0xAF
    };
    for (size_t i = 0; i < sizeof(init_seq); ++i) {
        if (!oled_write_cmd(init_seq[i])) {
            ESP_LOGW(TAG, "SSD1306 init failed at addr=0x%02x", s_oled_addr);
            s_ready = false;
            return;
        }
    }
    s_ready = true;
    ESP_LOGI(TAG, "OLED display service initialized addr=0x%02x", s_oled_addr);
}

void display_service_tick(void)
{
    if (!s_ready) {
        return;
    }

    handle_buttons();
    render_status_page();
    oled_flush();
}

display_page_t display_service_get_page(void)
{
    return s_page;
}
