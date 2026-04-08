#include "rc_input.h"

#include <string.h>

#include "board_defaults.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "rc_input";
static rc_command_t s_command;
static bool s_uart_ready;
static int64_t s_last_frame_us;

enum {
    SCARGO_RC_LINK_TIMEOUT_US = 500000,
};

enum {
    CRSF_FRAME_MAX = 64,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RECEIVER = 0xEC,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
};

static uint8_t crc8_d5(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0xD5U) : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static float apply_deadzone(float value)
{
    if (value > -SCARGO_RC_DEADZONE && value < SCARGO_RC_DEADZONE) {
        return 0.0f;
    }
    return value;
}

static float normalize_centered_channel(uint16_t raw)
{
    float normalized = ((float)raw - (float)SCARGO_CRSF_MID_VALUE) /
                       ((float)(SCARGO_CRSF_MAX_VALUE - SCARGO_CRSF_MIN_VALUE) * 0.5f);
    if (normalized < -1.0f) {
        normalized = -1.0f;
    }
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }
    return apply_deadzone(normalized);
}

static float normalize_throttle_channel(uint16_t raw)
{
    float normalized = ((float)raw - (float)SCARGO_CRSF_MIN_VALUE) /
                       (float)(SCARGO_CRSF_MAX_VALUE - SCARGO_CRSF_MIN_VALUE);
    if (normalized < 0.0f) {
        normalized = 0.0f;
    }
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }
    return normalized * 2.0f - 1.0f;
}

static int16_t decode_switch(uint16_t raw)
{
    if (raw <= SCARGO_CRSF_SWITCH_LOW) {
        return -1;
    }
    if (raw >= SCARGO_CRSF_SWITCH_HIGH) {
        return 1;
    }
    return 0;
}

static void update_command_from_channels(const uint16_t *channels, size_t count)
{
    size_t copy_count = count < 12U ? count : 12U;
    memcpy(s_command.raw_channels, channels, sizeof(uint16_t) * copy_count);

    if (count < 8U) {
        return;
    }

    /*
     * 当前遥控器四个主通道顺序已经通过实测确认：
     * - ch0 : roll
     * - ch1 : pitch
     * - ch2 : throttle
     * - ch3 : yaw
     */
    s_command.roll = normalize_centered_channel(channels[0]);
    s_command.pitch = normalize_centered_channel(channels[1]);
    s_command.throttle = normalize_throttle_channel(channels[2]);
    s_command.yaw = normalize_centered_channel(channels[3]);
    s_command.aux_sa = decode_switch(channels[4]);
    s_command.aux_sb = decode_switch(channels[5]);
    s_command.aux_sc = decode_switch(channels[6]);
    s_command.aux_sd = decode_switch(channels[7]);
    s_command.walk_mode = s_command.aux_sa > 0;
    s_command.link_up = true;
    s_last_frame_us = esp_timer_get_time();
}

static void decode_channels_payload(const uint8_t *payload, size_t payload_len)
{
    if (payload_len < 22U) {
        return;
    }

    uint16_t channels[16] = {0};
    uint32_t bit_buffer = 0;
    int bit_count = 0;
    int channel_index = 0;

    for (size_t i = 0; i < payload_len && channel_index < 16; ++i) {
        bit_buffer |= ((uint32_t)payload[i]) << bit_count;
        bit_count += 8;

        while (bit_count >= 11 && channel_index < 16) {
            channels[channel_index++] = (uint16_t)(bit_buffer & 0x07FFU);
            bit_buffer >>= 11;
            bit_count -= 11;
        }
    }

    update_command_from_channels(channels, 12);
}

static void consume_uart_bytes(const uint8_t *buffer, int length)
{
    int index = 0;
    while (index + 2 < length) {
        uint8_t address = buffer[index];
        if (address != CRSF_ADDRESS_FLIGHT_CONTROLLER && address != CRSF_ADDRESS_RECEIVER) {
            ++index;
            continue;
        }

        uint8_t frame_size = buffer[index + 1];
        if (frame_size < 2U || frame_size > (CRSF_FRAME_MAX - 2U)) {
            ++index;
            continue;
        }

        int frame_end = index + 2 + frame_size;
        if (frame_end > length) {
            break;
        }

        const uint8_t *frame = &buffer[index + 2];
        uint8_t type = frame[0];
        uint8_t crc = frame[frame_size - 1];
        if (crc8_d5(frame, (size_t)frame_size - 1U) == crc && type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            decode_channels_payload(&frame[1], (size_t)frame_size - 2U);
        }

        index = frame_end;
    }
}

void rc_input_init(void)
{
    memset(&s_command, 0, sizeof(s_command));
    s_last_frame_us = 0;

    const scargo_gpio_map_t *pins = board_defaults_gpio_map();
    uart_config_t uart_config = {
        .baud_rate = SCARGO_CRSF_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 512, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, pins->uart_com1_tx, pins->uart_com1_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_uart_ready = true;
    ESP_LOGI(TAG, "CRSF input service initialized on UART1");
}

void rc_input_tick(void)
{
    if (!s_uart_ready) {
        return;
    }

    uint8_t buffer[128];
    int length = uart_read_bytes(UART_NUM_1, buffer, sizeof(buffer), 0);
    if (length > 0) {
        consume_uart_bytes(buffer, length);
    }

    // CRSF 是持续刷新的协议。如果在一段时间内收不到新帧，就认为遥控已经失联。
    // 这样当遥控关闭或断开后，运行时档位覆盖会自动失效，Web/默认配置值才能重新直接生效。
    if (s_command.link_up && s_last_frame_us > 0) {
        int64_t now_us = esp_timer_get_time();
        if ((now_us - s_last_frame_us) > SCARGO_RC_LINK_TIMEOUT_US) {
            s_command.link_up = false;
        }
    }
}

rc_command_t rc_input_get_latest(void)
{
    return s_command;
}
