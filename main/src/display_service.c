#include "display_service.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "board_defaults.h"
#include "cpu_usage_service.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_service.h"
#include "kinematics.h"
#include "rc_input.h"
#include "robot_control.h"
#include "servo_output.h"
#include "shared_i2c.h"

static const char *TAG = "display";
static display_page_t s_page = DISPLAY_PAGE_ROBOT_PREVIEW;
static int s_leg_preview_selection = SCARGO_LEG_FRONT_RIGHT;
static display_preview_view_t s_leg_preview_view = DISPLAY_PREVIEW_VIEW_ISO;
static display_preview_view_t s_robot_preview_view = DISPLAY_PREVIEW_VIEW_ISO;
static i2c_master_dev_handle_t s_oled;
static bool s_ready;
static uint8_t s_buffer[128 * 8];
static uint16_t s_oled_addr;
static TickType_t s_boot_splash_deadline;

typedef enum {
    DISPLAY_LAYOUT_LANDSCAPE = 0,
    DISPLAY_LAYOUT_PORTRAIT_CCW = 1,
} display_layout_t;

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

static void oled_set_pixel(int x, int y);

static void oled_draw_char_rotated_cw(int x, int y, char ch)
{
    int index = ch - 32;
    if (index < 0 || index >= (int)(sizeof(FONT_5X7) / sizeof(FONT_5X7[0]))) {
        index = 0;
    }

    for (int col = 0; col < 5; ++col) {
        uint8_t bits = FONT_5X7[index][col];
        for (int row = 0; row < 7; ++row) {
            if ((bits >> row) & 0x01U) {
                oled_set_pixel(x + (6 - row), y + col);
            }
        }
    }
}

static void oled_draw_text_rotated_cw(int x, int y, const char *text)
{
    while (*text != '\0') {
        oled_draw_char_rotated_cw(x, y, *text++);
        y += 6;
    }
}

static void oled_draw_text_rotated_cw_compact(int x, int y, const char *text)
{
    while (*text != '\0') {
        oled_draw_char_rotated_cw(x, y, *text++);
        y += 5;
    }
}

static void oled_draw_char_rotated_ccw(int x, int y, char ch)
{
    int index = ch - 32;
    if (index < 0 || index >= (int)(sizeof(FONT_5X7) / sizeof(FONT_5X7[0]))) {
        index = 0;
    }

    for (int col = 0; col < 5; ++col) {
        uint8_t bits = FONT_5X7[index][col];
        for (int row = 0; row < 7; ++row) {
            if ((bits >> row) & 0x01U) {
                oled_set_pixel(x + row, y + (4 - col));
            }
        }
    }
}

static void oled_draw_text_rotated_ccw(int x, int y, const char *text)
{
    size_t len = strlen(text);
    for (size_t i = 0; i < len; ++i) {
        oled_draw_char_rotated_ccw(x, y, text[len - 1 - i]);
        y += 6;
    }
}

static void oled_draw_text_rotated_ccw_compact(int x, int y, const char *text)
{
    size_t len = strlen(text);
    for (size_t i = 0; i < len; ++i) {
        oled_draw_char_rotated_ccw(x, y, text[len - 1 - i]);
        y += 5;
    }
}

/*
 * 显示布局辅助层
 * ==============
 *
 * 这里把“横屏/竖屏”的常用变换抽成统一接口，避免页面代码里散落着：
 * - 点位旋转
 * - 文本方向切换
 * - 方向常量判断
 *
 * 目前先支持两种：
 * - DISPLAY_LAYOUT_LANDSCAPE   : 原始横屏
 * - DISPLAY_LAYOUT_PORTRAIT_CCW: 基于横屏内容逆时针旋转 90 度
 *
 * 后面如果别的页面也要旋转显示，只需要复用这里的接口。
 */
static void display_layout_project_point(display_layout_t layout, float src_x, float src_y,
                                         float *dst_x, float *dst_y)
{
    switch (layout) {
    case DISPLAY_LAYOUT_PORTRAIT_CCW:
        /*
         * 按“原横屏内容逆时针 90 度后”的观察语义来放置几何点。
         * 如果腿本体方向与用户感知相反，只需要在这里翻转，不要去改页面里的局部公式。
         */
        *dst_x = src_y;
        *dst_y = -src_x;
        break;
    case DISPLAY_LAYOUT_LANDSCAPE:
    default:
        *dst_x = src_x;
        *dst_y = src_y;
        break;
    }
}

static void display_layout_draw_title(display_layout_t layout, int x, int y, const char *text)
{
    switch (layout) {
    case DISPLAY_LAYOUT_PORTRAIT_CCW:
        /*
         * 文本锚点统一按“左对齐”理解：
         * - x/y 永远表示旋转后视角下的左上起点
         * - 文字阅读顺序始终保持正常，不允许反着写
         */
        oled_draw_text_rotated_ccw(x, y, text);
        break;
    case DISPLAY_LAYOUT_LANDSCAPE:
    default:
        oled_draw_text(x, y / 8, text);
        break;
    }
}

static void display_layout_draw_footer(display_layout_t layout, int x, int y, const char *text)
{
    switch (layout) {
    case DISPLAY_LAYOUT_PORTRAIT_CCW:
        oled_draw_text_rotated_ccw_compact(x, y, text);
        break;
    case DISPLAY_LAYOUT_LANDSCAPE:
    default:
        oled_draw_text(x, y / 8, text);
        break;
    }
}

static void oled_set_pixel(int x, int y)
{
    if (x < 0 || x >= 128 || y < 0 || y >= 64) {
        return;
    }

    s_buffer[(y / 8) * 128 + x] |= (uint8_t)(1U << (y % 8));
}

static void oled_draw_line(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        oled_set_pixel(x0, y0);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void oled_draw_marker(int x, int y)
{
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            oled_set_pixel(x + dx, y + dy);
        }
    }
}

static void oled_draw_rect(int x, int y, int w, int h)
{
    if (w <= 0 || h <= 0) {
        return;
    }
    oled_draw_line(x, y, x + w, y);
    oled_draw_line(x, y + h, x + w, y + h);
    oled_draw_line(x, y, x, y + h);
    oled_draw_line(x + w, y, x + w, y + h);
}

static void oled_fill_rect(int x, int y, int w, int h)
{
    if (w <= 0 || h <= 0) {
        return;
    }
    for (int iy = y; iy < y + h; ++iy) {
        for (int ix = x; ix < x + w; ++ix) {
            oled_set_pixel(ix, iy);
        }
    }
}

static void oled_fill_triangle(int x0, int y0, int x1, int y1, int x2, int y2)
{
    int min_x = x0;
    int max_x = x0;
    int min_y = y0;
    int max_y = y0;

    if (x1 < min_x) min_x = x1;
    if (x2 < min_x) min_x = x2;
    if (x1 > max_x) max_x = x1;
    if (x2 > max_x) max_x = x2;
    if (y1 < min_y) min_y = y1;
    if (y2 < min_y) min_y = y2;
    if (y1 > max_y) max_y = y1;
    if (y2 > max_y) max_y = y2;

    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            int w0 = (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0);
            int w1 = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1);
            int w2 = (x0 - x2) * (y - y2) - (y0 - y2) * (x - x2);
            if ((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0)) {
                oled_set_pixel(x, y);
            }
        }
    }
}

static void oled_fill_quad(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3)
{
    oled_fill_triangle(x0, y0, x1, y1, x2, y2);
    oled_fill_triangle(x0, y0, x2, y2, x3, y3);
}

static void rotate_point(float x, float y, float z,
                         float roll_deg, float pitch_deg, float yaw_deg,
                         float *out_x, float *out_y, float *out_z)
{
    const float roll = roll_deg * (float)M_PI / 180.0f;
    const float pitch = pitch_deg * (float)M_PI / 180.0f;
    const float yaw = yaw_deg * (float)M_PI / 180.0f;
    const float cr = cosf(roll);
    const float sr = sinf(roll);
    const float cp = cosf(pitch);
    const float sp = sinf(pitch);
    const float cy = cosf(yaw);
    const float sy = sinf(yaw);

    // x+: right, y+: forward, z+: up
    // pitch around +x, roll around +y, yaw around +z
    float x1 = x;
    float y1 = y * cp - z * sp;
    float z1 = y * sp + z * cp;

    float x2 = x1 * cr + z1 * sr;
    float y2 = y1;
    float z2 = -x1 * sr + z1 * cr;

    *out_x = x2 * cy - y2 * sy;
    *out_y = x2 * sy + y2 * cy;
    *out_z = z2;
}

static void project_point(float x, float y, float z, float *out_x, float *out_y)
{
    // Rear 5-degree top-down projection with a 15-degree leftward azimuth bias.
    // This keeps the robot attitude axes unchanged and only adjusts the viewer angle.
    const float azimuth_sin = 0.25881905f;
    const float azimuth_cos = 0.96592583f;
    const float tilt_sin = 0.08715574f;
    const float tilt_cos = 0.99619470f;
    const float view_x = x * azimuth_cos - y * azimuth_sin;
    const float view_y = x * azimuth_sin + y * azimuth_cos;
    *out_x = view_x;
    *out_y = -(view_y * tilt_sin + z * tilt_cos);
}

static bool preview_view_valid(display_preview_view_t view)
{
    return view >= DISPLAY_PREVIEW_VIEW_FRONT && view <= DISPLAY_PREVIEW_VIEW_ISO;
}

static void project_preview_point(float x, float y, float z, display_preview_view_t view, float *out_x, float *out_y)
{
    switch (view) {
    case DISPLAY_PREVIEW_VIEW_FRONT:
        *out_x = x;
        *out_y = -z;
        break;
    case DISPLAY_PREVIEW_VIEW_BACK:
        *out_x = -x;
        *out_y = -z;
        break;
    case DISPLAY_PREVIEW_VIEW_LEFT:
        *out_x = y;
        *out_y = -z;
        break;
    case DISPLAY_PREVIEW_VIEW_RIGHT:
        *out_x = -y;
        *out_y = -z;
        break;
    case DISPLAY_PREVIEW_VIEW_TOP:
        *out_x = x;
        *out_y = -y;
        break;
    case DISPLAY_PREVIEW_VIEW_ISO:
    default:
        project_point(x, y, z, out_x, out_y);
        break;
    }
}

static void render_imu_box_at(float roll_deg, float pitch_deg, float yaw_deg,
                              int region_x, int region_y, int region_w, int region_h)
{
    static const float vertices[8][3] = {
        {-0.67f, -1.0f, -0.10f}, {0.67f, -1.0f, -0.10f}, {0.67f, 1.0f, -0.10f}, {-0.67f, 1.0f, -0.10f},
        {-0.67f, -1.0f, 0.10f},  {0.67f, -1.0f, 0.10f},  {0.67f, 1.0f, 0.10f},  {-0.67f, 1.0f, 0.10f},
    };
    static const int edges[12][2] = {
        {0,1}, {1,2}, {2,3}, {3,0},
        {4,5}, {5,6}, {6,7}, {7,4},
        {0,4}, {1,5}, {2,6}, {3,7},
    };
    float projected_float[8][2];
    int projected[8][2];
    float fx, fy, fz;
    float ux, uy, uz;
    float front_proj[2];
    float up_proj[2];
    float min_x = 9999.0f;
    float max_x = -9999.0f;
    float min_y = 9999.0f;
    float max_y = -9999.0f;
    const float margin = 4.0f;
    const int cx = region_x + region_w / 2;
    const int cy = region_y + region_h / 2;

    for (int i = 0; i < 8; ++i) {
        float rx, ry, rz;
        rotate_point(vertices[i][0], vertices[i][1], vertices[i][2], roll_deg, pitch_deg, yaw_deg, &rx, &ry, &rz);
        project_point(rx, ry, rz, &projected_float[i][0], &projected_float[i][1]);
        if (projected_float[i][0] < min_x) min_x = projected_float[i][0];
        if (projected_float[i][0] > max_x) max_x = projected_float[i][0];
        if (projected_float[i][1] < min_y) min_y = projected_float[i][1];
        if (projected_float[i][1] > max_y) max_y = projected_float[i][1];
    }

    rotate_point(0.f, 1.0f, 0.f, roll_deg, pitch_deg, yaw_deg, &fx, &fy, &fz);
    rotate_point(0.f, 0.f, 0.75f, roll_deg, pitch_deg, yaw_deg, &ux, &uy, &uz);
    project_point(fx, fy, fz, &front_proj[0], &front_proj[1]);
    project_point(ux, uy, uz, &up_proj[0], &up_proj[1]);

    if (front_proj[0] < min_x) min_x = front_proj[0];
    if (front_proj[0] > max_x) max_x = front_proj[0];
    if (front_proj[1] < min_y) min_y = front_proj[1];
    if (front_proj[1] > max_y) max_y = front_proj[1];
    if (up_proj[0] < min_x) min_x = up_proj[0];
    if (up_proj[0] > max_x) max_x = up_proj[0];
    if (up_proj[1] < min_y) min_y = up_proj[1];
    if (up_proj[1] > max_y) max_y = up_proj[1];

    float span_x = fmaxf(max_x - min_x, 0.2f);
    float span_y = fmaxf(max_y - min_y, 0.2f);
    float scale = fminf(((float)region_w - margin * 2.0f) / span_x,
                        ((float)region_h - margin * 2.0f) / span_y);
    float center_x = (min_x + max_x) * 0.5f;
    float center_y = (min_y + max_y) * 0.5f;

    for (int i = 0; i < 8; ++i) {
        projected[i][0] = cx + (int)lroundf((projected_float[i][0] - center_x) * scale);
        projected[i][1] = cy + (int)lroundf((projected_float[i][1] - center_y) * scale);
    }

    // Fill the rear face so front/back is easier to distinguish.
    oled_fill_quad(projected[0][0], projected[0][1],
                   projected[1][0], projected[1][1],
                   projected[5][0], projected[5][1],
                   projected[4][0], projected[4][1]);

    for (int i = 0; i < 12; ++i) {
        oled_draw_line(projected[edges[i][0]][0],
                       projected[edges[i][0]][1],
                       projected[edges[i][1]][0],
                       projected[edges[i][1]][1]);
    }
    int front_x, front_y, up_x, up_y;
    front_x = cx + (int)lroundf((front_proj[0] - center_x) * scale);
    front_y = cy + (int)lroundf((front_proj[1] - center_y) * scale);
    up_x = cx + (int)lroundf((up_proj[0] - center_x) * scale);
    up_y = cy + (int)lroundf((up_proj[1] - center_y) * scale);

    oled_draw_marker(front_x, front_y);
    oled_draw_marker(up_x, up_y);
    oled_draw_text(front_x > 116 ? 110 : front_x + 3, front_y / 8, "F");
    oled_draw_text(up_x > 116 ? 110 : up_x + 3, up_y / 8, "U");
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
    attitude_state_t attitude = imu_service_get_attitude();

    oled_clear();
    render_imu_box_at(attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg, 1, 1, 126, 62);
}

static void render_rc_target_page(void)
{
    rc_command_t command = rc_input_get_latest();
    robot_target_pose_t target = robot_control_get_target_pose(&command);
    int throttle_pct = (int)lroundf(((command.throttle + 1.0f) * 0.5f) * 100.0f);
    if (throttle_pct < 0) {
        throttle_pct = 0;
    } else if (throttle_pct > 100) {
        throttle_pct = 100;
    }
    const int bar_x = 4;
    const int bar_y = 8;
    const int bar_w = 12;
    const int bar_h = 46;
    const int fill_h = (bar_h * throttle_pct) / 100;
    char throttle_text[8];
    snprintf(throttle_text, sizeof(throttle_text), "%3d", throttle_pct);

    oled_clear();
    render_imu_box_at(target.roll_deg, target.pitch_deg, target.yaw_deg, 18, 1, 108, 62);
    oled_draw_rect(bar_x, bar_y, bar_w, bar_h);
    if (fill_h > 0) {
        oled_fill_rect(bar_x + 2, bar_y + bar_h - fill_h, bar_w - 3, fill_h - (fill_h > 1 ? 1 : 0));
    }
    oled_draw_text(0, 0, "THR");
    oled_draw_text(0, 7, throttle_text);
}

static void render_cpu_page(void)
{
    cpu_usage_snapshot_t cpu = cpu_usage_service_get_snapshot();
    char line0[20];
    char line1[20];
    char line2[20];

    oled_clear();
    oled_draw_text(26, 1, "CPU LOAD");
    if (cpu.ready && cpu.core_count >= 2) {
        snprintf(line0, sizeof(line0), "CORE0 %5.1f%%", (double)cpu.usage_pct[0]);
        snprintf(line1, sizeof(line1), "CORE1 %5.1f%%", (double)cpu.usage_pct[1]);
        oled_draw_text(12, 3, line0);
        oled_draw_text(12, 5, line1);
    } else if (cpu.ready && cpu.core_count == 1) {
        snprintf(line0, sizeof(line0), "CORE0 %5.1f%%", (double)cpu.usage_pct[0]);
        oled_draw_text(12, 4, line0);
    } else {
        oled_draw_text(28, 4, "CPU --");
    }
    snprintf(line2, sizeof(line2), "CORES %u", cpu.core_count);
    oled_draw_text(28, 7, line2);
}

static void render_leg_preview_page(void)
{
    const display_layout_t layout = DISPLAY_LAYOUT_PORTRAIT_CCW;
    char label[16];
    vec3f_t chain_points[4];
    vec3f_t feet_world[SCARGO_LEG_COUNT];
    vec3f_t feet_body[SCARGO_LEG_COUNT];
    leg_joint_pose_t joint_pose;
    float projected_float[4][2];
    int projected[4][2];
    /*
     * 单腿页采用“逆时针 90 度后的竖屏视角”：
     * - 左上角保留给 LEGx
     * - 腿本体在剩余区域做水平/垂直居中
     * - 当前不显示底部角度文字，所以把更多空间留给腿本体
     */
    const int title_left = 1;
    const int title_top = 1;
    const int region_x = 12;
    const int region_y = 2;
    const int region_w = 114;
    const int region_h = 61;
    const float margin = 2.0f;

    oled_clear();
    if (!robot_control_get_current_feet_world(feet_world)) {
        oled_draw_text(18, 3, "LEG PREVIEW");
        oled_draw_text(26, 5, "NO DATA");
        return;
    }
    /*
     * ====================
     * 第三层：预览显示层
     * ====================
     *
     * 单腿实时页只做“安装层预览”：
     * 1. 先从控制层读取当前足端世界坐标与机身姿态
     * 2. 转回机身局部 feet_body
     * 3. 用 installation_pose 解出 shoulder / alpha / beta
     * 4. 再把安装层链点画出来
     *
     * 这里刻意不走：
     * - SERVO_MAP
     * - 标定偏置
     * - 小腿真实执行修正
     *
     * 因为这个页面要表达的是“当前控制目标在安装语义下长什么样”，
     * 而不是“PCA9685 最终打给舵机的电角度”。
     */
    body_pose_t pose = robot_control_get_current_body_pose();
    kinematics_apply_body_pose(feet_body, feet_world, robot_control_get_current_height_mm(), &pose);
    if (!kinematics_solve_leg_installation_pose((scargo_leg_id_t)s_leg_preview_selection,
                                                &feet_body[s_leg_preview_selection],
                                                &joint_pose) ||
        !kinematics_compute_leg_chain_from_installation_pose((scargo_leg_id_t)s_leg_preview_selection, &joint_pose,
                                                             chain_points)) {
        oled_draw_text(18, 3, "LEG PREVIEW");
        oled_draw_text(26, 5, "NO DATA");
        return;
    }

    for (int i = 0; i < 4; ++i) {
        float x = chain_points[i].x_mm;
        float y = chain_points[i].y_mm;
        float z = chain_points[i].z_mm;
        project_preview_point(x, y, z, s_leg_preview_view, &projected_float[i][0], &projected_float[i][1]);
    }

    {
        const scargo_mechanics_t *mech = board_defaults_mechanics();
        const float leg_max_y = mech->thigh_length_mm + mech->calf_length_mm;
        const float leg_max_z = mech->shoulder_length_mm + mech->thigh_length_mm + mech->calf_length_mm;
        /*
         * 单腿页采用更自然的竖向观感：
         * - 基于原横屏内容整体旋转 90 度后的视角
         * - 先旋转投影点
         * - 再根据旋转后的真实边界做自适应缩放
         * - 优先保证腿本体不碰 OLED 边缘
         */
        float min_rx = 9999.0f;
        float max_rx = -9999.0f;
        float min_ry = 9999.0f;
        float max_ry = -9999.0f;
        for (int i = 0; i < 4; ++i) {
            float rx;
            float ry;
            display_layout_project_point(layout, projected_float[i][0], projected_float[i][1], &rx, &ry);
            if (rx < min_rx) min_rx = rx;
            if (rx > max_rx) max_rx = rx;
            if (ry < min_ry) min_ry = ry;
            if (ry > max_ry) max_ry = ry;
        }

        /*
         * 给一点额外的几何安全边距，避免腿在极限姿态下压到 OLED 边缘。
         * 文字允许更靠边，但腿本体要完整显示。
         */
        min_rx -= leg_max_y * 0.05f;
        max_rx += leg_max_y * 0.05f;
        min_ry -= leg_max_z * 0.05f;
        max_ry += leg_max_z * 0.05f;

        float span_x = fmaxf(max_rx - min_rx, 1.0f);
        float span_y = fmaxf(max_ry - min_ry, 1.0f);
        float scale = fminf(((float)region_w - margin * 2.0f) / span_x,
                            ((float)region_h - margin * 2.0f) / span_y);
        float center_x = (min_rx + max_rx) * 0.5f;
        float center_y = (min_ry + max_ry) * 0.5f;
        int cx = region_x + region_w / 2;
        int cy = region_y + region_h / 2;
        for (int i = 0; i < 4; ++i) {
            float rotated_x;
            float rotated_y;
            display_layout_project_point(layout,
                                         projected_float[i][0],
                                         projected_float[i][1],
                                         &rotated_x,
                                         &rotated_y);
            projected[i][0] = cx + (int)lroundf((rotated_x - center_x) * scale);
            projected[i][1] = cy + (int)lroundf((rotated_y - center_y) * scale);
        }
    }

    oled_draw_line(projected[0][0], projected[0][1], projected[1][0], projected[1][1]);
    oled_draw_line(projected[1][0], projected[1][1], projected[2][0], projected[2][1]);
    oled_draw_line(projected[2][0], projected[2][1], projected[3][0], projected[3][1]);
    oled_draw_marker(projected[0][0], projected[0][1]);
    oled_draw_marker(projected[1][0], projected[1][1]);
    oled_draw_marker(projected[2][0], projected[2][1]);
    oled_draw_marker(projected[3][0], projected[3][1]);

    snprintf(label, sizeof(label), "LEG%d", s_leg_preview_selection);
    display_layout_draw_title(layout, title_left, title_top, label);
}

static bool compute_leg_model_points(int leg, float points[4][3])
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    vec3f_t chain_points[4];
    vec3f_t feet_world[SCARGO_LEG_COUNT];
    vec3f_t feet_body[SCARGO_LEG_COUNT];
    leg_joint_pose_t joint_pose;
    if (!robot_control_get_current_feet_world(feet_world)) {
        return false;
    }
    /*
     * 整机实时页与单腿实时页必须使用同一条“安装层预览链”。
     * 这里不允许额外混入任何舵机层补偿，否则单腿页和整机页就会分叉。
     */
    body_pose_t pose = robot_control_get_current_body_pose();
    kinematics_apply_body_pose(feet_body, feet_world, robot_control_get_current_height_mm(), &pose);
    if (!kinematics_solve_leg_installation_pose((scargo_leg_id_t)leg, &feet_body[leg], &joint_pose) ||
        !kinematics_compute_leg_chain_from_installation_pose((scargo_leg_id_t)leg, &joint_pose, chain_points)) {
        return false;
    }
    const float half_width = mech->body_width_mm * 0.5f;
    const float half_length = mech->body_length_mm * 0.5f;
    const float hip_x = (leg == SCARGO_LEG_FRONT_LEFT || leg == SCARGO_LEG_REAR_LEFT) ? half_width : -half_width;
    const float hip_y = (leg == SCARGO_LEG_FRONT_LEFT || leg == SCARGO_LEG_FRONT_RIGHT) ? half_length : -half_length;
    for (int i = 0; i < 4; ++i) {
        points[i][0] = hip_x + chain_points[i].x_mm;
        points[i][1] = hip_y + chain_points[i].y_mm;
        points[i][2] = chain_points[i].z_mm;
    }
    return true;
}

static void render_robot_preview_page(void)
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    const float half_width = mech->body_width_mm * 0.5f;
    const float half_length = mech->body_length_mm * 0.5f;
    const float body[4][3] = {
        { half_width,  half_length, 0.0f},
        {-half_width,  half_length, 0.0f},
        {-half_width, -half_length, 0.0f},
        { half_width, -half_length, 0.0f},
    };
    const int body_edges[4][2] = {{0,1},{1,2},{2,3},{3,0}};
    float body_projected[4][2];
    int body_points[4][2];
    float leg_points_3d[SCARGO_LEG_COUNT][4][3];
    float leg_projected[SCARGO_LEG_COUNT][4][2];
    int leg_points[SCARGO_LEG_COUNT][4][2];
    float min_x = 9999.0f;
    float max_x = -9999.0f;
    float min_y = 9999.0f;
    float max_y = -9999.0f;
    float front_arrow[4][2];
    int front_arrow_points[4][2];
    const int region_x = 0;
    const int region_y = 0;
    const int region_w = 128;
    const int region_h = 64;
    const float margin = 2.0f;

    oled_clear();

    for (int i = 0; i < 4; ++i) {
        project_preview_point(body[i][0], body[i][1], body[i][2], s_robot_preview_view,
                              &body_projected[i][0], &body_projected[i][1]);
        if (body_projected[i][0] < min_x) min_x = body_projected[i][0];
        if (body_projected[i][0] > max_x) max_x = body_projected[i][0];
        if (body_projected[i][1] < min_y) min_y = body_projected[i][1];
        if (body_projected[i][1] > max_y) max_y = body_projected[i][1];
    }

    {
        const float front_center[3] = {0.0f, half_length, 0.0f};
        const float front_tip[3] = {0.0f, half_length + 22.0f, 0.0f};
        const float front_left[3] = {8.0f, half_length + 12.0f, 0.0f};
        const float front_right[3] = {-8.0f, half_length + 12.0f, 0.0f};
        const float arrow_vertices[4][3] = {
            {front_center[0], front_center[1], front_center[2]},
            {front_tip[0], front_tip[1], front_tip[2]},
            {front_left[0], front_left[1], front_left[2]},
            {front_right[0], front_right[1], front_right[2]},
        };
        for (int i = 0; i < 4; ++i) {
            project_preview_point(arrow_vertices[i][0], arrow_vertices[i][1], arrow_vertices[i][2],
                                  s_robot_preview_view, &front_arrow[i][0], &front_arrow[i][1]);
            if (front_arrow[i][0] < min_x) min_x = front_arrow[i][0];
            if (front_arrow[i][0] > max_x) max_x = front_arrow[i][0];
            if (front_arrow[i][1] < min_y) min_y = front_arrow[i][1];
            if (front_arrow[i][1] > max_y) max_y = front_arrow[i][1];
        }
    }

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        if (!compute_leg_model_points(leg, leg_points_3d[leg])) {
            return;
        }
        for (int joint = 0; joint < 4; ++joint) {
            project_preview_point(leg_points_3d[leg][joint][0],
                                  leg_points_3d[leg][joint][1],
                                  leg_points_3d[leg][joint][2],
                                  s_robot_preview_view,
                                  &leg_projected[leg][joint][0], &leg_projected[leg][joint][1]);
            if (leg_projected[leg][joint][0] < min_x) min_x = leg_projected[leg][joint][0];
            if (leg_projected[leg][joint][0] > max_x) max_x = leg_projected[leg][joint][0];
            if (leg_projected[leg][joint][1] < min_y) min_y = leg_projected[leg][joint][1];
            if (leg_projected[leg][joint][1] > max_y) max_y = leg_projected[leg][joint][1];
        }
    }

    {
        float span_x = fmaxf(max_x - min_x, 1.0f);
        float span_y = fmaxf(max_y - min_y, 1.0f);
        float scale = fminf(((float)region_w - margin * 2.0f) / span_x,
                            ((float)region_h - margin * 2.0f) / span_y);
        float center_x = (min_x + max_x) * 0.5f;
        float center_y = (min_y + max_y) * 0.5f;
        int cx = region_x + region_w / 2;
        int cy = region_y + region_h / 2;

        for (int i = 0; i < 4; ++i) {
            body_points[i][0] = cx + (int)lroundf((body_projected[i][0] - center_x) * scale);
            body_points[i][1] = cy + (int)lroundf((body_projected[i][1] - center_y) * scale);
            front_arrow_points[i][0] = cx + (int)lroundf((front_arrow[i][0] - center_x) * scale);
            front_arrow_points[i][1] = cy + (int)lroundf((front_arrow[i][1] - center_y) * scale);
        }
        for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
            for (int joint = 0; joint < 4; ++joint) {
                leg_points[leg][joint][0] = cx + (int)lroundf((leg_projected[leg][joint][0] - center_x) * scale);
                leg_points[leg][joint][1] = cy + (int)lroundf((leg_projected[leg][joint][1] - center_y) * scale);
            }
        }
    }

    for (int i = 0; i < 4; ++i) {
        oled_draw_line(body_points[body_edges[i][0]][0],
                       body_points[body_edges[i][0]][1],
                       body_points[body_edges[i][1]][0],
                       body_points[body_edges[i][1]][1]);
    }
    oled_draw_line(front_arrow_points[0][0], front_arrow_points[0][1], front_arrow_points[1][0], front_arrow_points[1][1]);
    oled_draw_line(front_arrow_points[1][0], front_arrow_points[1][1], front_arrow_points[2][0], front_arrow_points[2][1]);
    oled_draw_line(front_arrow_points[1][0], front_arrow_points[1][1], front_arrow_points[3][0], front_arrow_points[3][1]);
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        oled_draw_line(leg_points[leg][0][0], leg_points[leg][0][1], leg_points[leg][1][0], leg_points[leg][1][1]);
        oled_draw_line(leg_points[leg][1][0], leg_points[leg][1][1], leg_points[leg][2][0], leg_points[leg][2][1]);
        oled_draw_line(leg_points[leg][2][0], leg_points[leg][2][1], leg_points[leg][3][0], leg_points[leg][3][1]);
    }
}

static void render_boot_splash(void)
{
    oled_clear();
    oled_draw_text(28, 1, "SCARGO");
    oled_draw_text(12, 3, "QUADRUPED CTRL");
    oled_draw_text(18, 5, "ESP32-S3 ONLINE");
    oled_draw_text(24, 7, "BOOTING...");
}

static void handle_buttons(void)
{
    static int last_k1 = 1;
    static int last_k2 = 1;
    const scargo_gpio_map_t *pins = board_defaults_gpio_map();
    int k1 = gpio_get_level(pins->button_k1);
    int k2 = gpio_get_level(pins->button_k2);

    if (last_k1 == 1 && k1 == 0) {
        s_page = (display_page_t)(((int)s_page + 1) % 7);
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
    s_boot_splash_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1800);
    render_boot_splash();
    oled_flush();
    ESP_LOGI(TAG, "OLED display service initialized addr=0x%02x", s_oled_addr);
}

void display_service_tick(void)
{
    if (!s_ready) {
        return;
    }

    handle_buttons();
    if (xTaskGetTickCount() < s_boot_splash_deadline) {
        render_boot_splash();
    } else {
        switch (s_page) {
        case DISPLAY_PAGE_RC_TARGET:
            render_rc_target_page();
            break;
        case DISPLAY_PAGE_CPU:
            render_cpu_page();
            break;
        case DISPLAY_PAGE_STATUS:
            render_status_page();
            break;
        case DISPLAY_PAGE_LEG_PREVIEW:
            render_leg_preview_page();
            break;
        case DISPLAY_PAGE_ROBOT_PREVIEW:
            render_robot_preview_page();
            break;
        case DISPLAY_PAGE_CALIBRATION:
        case DISPLAY_PAGE_TEST:
        default:
            render_rc_target_page();
            break;
        }
    }
    oled_flush();
}

display_page_t display_service_get_page(void)
{
    return s_page;
}

void display_service_set_page(display_page_t page)
{
    if (page < DISPLAY_PAGE_STATUS || page > DISPLAY_PAGE_ROBOT_PREVIEW) {
        return;
    }
    s_page = page;
}

int display_service_get_leg_preview_selection(void)
{
    return s_leg_preview_selection;
}

void display_service_set_leg_preview_selection(int leg)
{
    if (leg < 0 || leg >= SCARGO_LEG_COUNT) {
        return;
    }
    s_leg_preview_selection = leg;
}

display_preview_view_t display_service_get_leg_preview_view(void)
{
    return s_leg_preview_view;
}

void display_service_set_leg_preview_view(display_preview_view_t view)
{
    if (!preview_view_valid(view)) {
        return;
    }
    s_leg_preview_view = view;
}

display_preview_view_t display_service_get_robot_preview_view(void)
{
    return s_robot_preview_view;
}

void display_service_set_robot_preview_view(display_preview_view_t view)
{
    if (!preview_view_valid(view)) {
        return;
    }
    s_robot_preview_view = view;
}
