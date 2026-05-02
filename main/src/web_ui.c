#include "web_ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <sys/stat.h>

#include "cJSON.h"
#include "board_defaults.h"
#include "buzzer_service.h"
#include "config_store.h"
#include "display_service.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "fan_service.h"
#include "imu_service.h"
#include "log_service.h"
#include "imu_service.h"
#include "ota_service.h"
#include "rc_input.h"
#include "wifi_config.h"
#include "robot_control.h"
#include "storage_service.h"
#include "version.h"

static const char *TAG = "web_ui";
static httpd_handle_t s_server;
static system_config_t *s_config;
static const char *HTML_PATH = "/spiffs/index.html";

static const char FALLBACK_HTML[] =
    "<!doctype html><html><head><meta charset='utf-8'><title>ScarGo</title></head>"
    "<body><h1>ScarGo Control Surface</h1><p>Missing /spiffs/index.html asset.</p></body></html>";

static char *load_html_asset(size_t *out_size)
{
    FILE *file = fopen(HTML_PATH, "rb");
    if (file == NULL) {
        return NULL;
    }

    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return NULL;
    }

    long size = ftell(file);
    if (size <= 0) {
        fclose(file);
        return NULL;
    }
    rewind(file);

    char *buffer = calloc(1, (size_t)size + 1U);
    if (buffer == NULL) {
        fclose(file);
        return NULL;
    }

    size_t bytes_read = fread(buffer, 1U, (size_t)size, file);
    fclose(file);
    if (bytes_read != (size_t)size) {
        free(buffer);
        return NULL;
    }

    *out_size = (size_t)size;
    return buffer;
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

static const char *display_page_name(display_page_t page)
{
    switch (page) {
    case DISPLAY_PAGE_STATUS:
        return "imu";
    case DISPLAY_PAGE_CALIBRATION:
        return "calibration";
    case DISPLAY_PAGE_TEST:
        return "test";
    case DISPLAY_PAGE_RC_TARGET:
        return "rc";
    case DISPLAY_PAGE_CPU:
        return "cpu";
    case DISPLAY_PAGE_LEG_PREVIEW:
        return "leg";
    case DISPLAY_PAGE_ROBOT_PREVIEW:
        return "robot";
    default:
        return "rc";
    }
}

static bool display_page_from_name(const char *name, display_page_t *out_page)
{
    if (strcmp(name, "imu") == 0) {
        *out_page = DISPLAY_PAGE_STATUS;
        return true;
    }
    if (strcmp(name, "calibration") == 0) {
        *out_page = DISPLAY_PAGE_CALIBRATION;
        return true;
    }
    if (strcmp(name, "test") == 0) {
        *out_page = DISPLAY_PAGE_TEST;
        return true;
    }
    if (strcmp(name, "rc") == 0) {
        *out_page = DISPLAY_PAGE_RC_TARGET;
        return true;
    }
    if (strcmp(name, "cpu") == 0) {
        *out_page = DISPLAY_PAGE_CPU;
        return true;
    }
    if (strcmp(name, "leg") == 0) {
        *out_page = DISPLAY_PAGE_LEG_PREVIEW;
        return true;
    }
    if (strcmp(name, "robot") == 0) {
        *out_page = DISPLAY_PAGE_ROBOT_PREVIEW;
        return true;
    }
    return false;
}

static const char *preview_view_name(display_preview_view_t view)
{
    switch (view) {
    case DISPLAY_PREVIEW_VIEW_FRONT:
        return "front";
    case DISPLAY_PREVIEW_VIEW_BACK:
        return "back";
    case DISPLAY_PREVIEW_VIEW_LEFT:
        return "left";
    case DISPLAY_PREVIEW_VIEW_RIGHT:
        return "right";
    case DISPLAY_PREVIEW_VIEW_TOP:
        return "top";
    case DISPLAY_PREVIEW_VIEW_ISO:
    default:
        return "iso";
    }
}

static bool preview_view_from_name(const char *name, display_preview_view_t *out_view)
{
    if (strcmp(name, "front") == 0) {
        *out_view = DISPLAY_PREVIEW_VIEW_FRONT;
        return true;
    }
    if (strcmp(name, "back") == 0) {
        *out_view = DISPLAY_PREVIEW_VIEW_BACK;
        return true;
    }
    if (strcmp(name, "left") == 0) {
        *out_view = DISPLAY_PREVIEW_VIEW_LEFT;
        return true;
    }
    if (strcmp(name, "right") == 0) {
        *out_view = DISPLAY_PREVIEW_VIEW_RIGHT;
        return true;
    }
    if (strcmp(name, "top") == 0) {
        *out_view = DISPLAY_PREVIEW_VIEW_TOP;
        return true;
    }
    if (strcmp(name, "iso") == 0 || strcmp(name, "stereo") == 0) {
        *out_view = DISPLAY_PREVIEW_VIEW_ISO;
        return true;
    }
    return false;
}

static const char *fan_level_name(fan_speed_level_t level)
{
    switch (level) {
    case FAN_SPEED_OFF:
        return "off";
    case FAN_SPEED_LOW:
        return "low";
    case FAN_SPEED_HIGH:
        return "high";
    case FAN_SPEED_MEDIUM:
    default:
        return "medium";
    }
}

static bool fan_level_from_name(const char *name, fan_speed_level_t *out_level)
{
    if (strcmp(name, "off") == 0) {
        *out_level = FAN_SPEED_OFF;
        return true;
    }
    if (strcmp(name, "low") == 0) {
        *out_level = FAN_SPEED_LOW;
        return true;
    }
    if (strcmp(name, "medium") == 0) {
        *out_level = FAN_SPEED_MEDIUM;
        return true;
    }
    if (strcmp(name, "high") == 0) {
        *out_level = FAN_SPEED_HIGH;
        return true;
    }
    return false;
}

static char *read_request_body(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        return NULL;
    }

    char *buffer = calloc(1, (size_t)req->content_len + 1U);
    if (buffer == NULL) {
        return NULL;
    }

    int total = 0;
    while (total < req->content_len) {
        int received = httpd_req_recv(req, buffer + total, (size_t)(req->content_len - total));
        if (received <= 0) {
            free(buffer);
            return NULL;
        }
        total += received;
    }

    return buffer;
}

static esp_err_t root_handler(httpd_req_t *req)
{
    buzzer_service_page_connect_beep();

    /* 用 stat() 获取文件大小作为 ETag，无需读取整个文件即可完成 304 协商，
     * 避免每次缓存命中都从 SPIFFS 读 98KB。 */
    struct stat st;
    if (stat(HTML_PATH, &st) == 0 && st.st_size > 0) {
        char etag[24];
        snprintf(etag, sizeof(etag), "\"%ld\"", (long)st.st_size);
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        httpd_resp_set_hdr(req, "ETag", etag);

        char if_none_match[32] = {0};
        if (httpd_req_get_hdr_value_str(req, "If-None-Match", if_none_match, sizeof(if_none_match)) == ESP_OK
            && strcmp(if_none_match, etag) == 0) {
            httpd_resp_set_status(req, "304 Not Modified");
            return httpd_resp_send(req, NULL, 0);
        }
    }

    httpd_resp_set_type(req, "text/html");
    size_t html_size = 0;
    char *html = load_html_asset(&html_size);
    if (html == NULL) {
        ESP_LOGW(TAG, "Falling back to embedded HTML because %s could not be loaded", HTML_PATH);
        return httpd_resp_send(req, FALLBACK_HTML, HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t err = httpd_resp_send(req, html, (ssize_t)html_size);
    free(html);
    return err;
}

static esp_err_t calibration_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *legs = cJSON_AddArrayToObject(root, "legs");

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        cJSON *leg_item = cJSON_CreateObject();
        cJSON *offsets = cJSON_AddArrayToObject(leg_item, "offsets_deg");
        cJSON_AddNumberToObject(leg_item, "leg", leg);
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            cJSON_AddItemToArray(offsets, cJSON_CreateNumber(s_config->calibration.servo_offsets_deg[leg][joint]));
        }
        cJSON_AddItemToArray(legs, leg_item);
    }

    char *json = cJSON_PrintUnformatted(root);
    esp_err_t err = send_json(req, json);
    cJSON_free(json);
    cJSON_Delete(root);
    return err;
}

static esp_err_t gait_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "step_height_mm", s_config->gait.step_height_mm);
    cJSON_AddNumberToObject(root, "step_cycle_ms", s_config->gait.step_cycle_ms);
    cJSON_AddNumberToObject(root, "step_scale", s_config->gait.step_scale);
    cJSON_AddNumberToObject(root, "stance_ratio", s_config->gait.stance_ratio);
    cJSON_AddNumberToObject(root, "diagonal_phase_offset", s_config->gait.diagonal_phase_offset);
    cJSON_AddNumberToObject(root, "stand_height_default_mm", s_config->gait.stand_height_default_mm);

    char *json = cJSON_PrintUnformatted(root);
    esp_err_t err = send_json(req, json);
    cJSON_free(json);
    cJSON_Delete(root);
    return err;
}

static esp_err_t calibration_post_handler(httpd_req_t *req)
{
    char *content = read_request_body(req);
    if (content == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No payload");
    }

    cJSON *root = cJSON_Parse(content);
    free(content);
    if (root == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }

    cJSON *legs = cJSON_GetObjectItem(root, "legs");
    if (cJSON_IsArray(legs)) {
        cJSON *leg_item = NULL;
        cJSON_ArrayForEach(leg_item, legs) {
            cJSON *leg_index = cJSON_GetObjectItem(leg_item, "leg");
            cJSON *offsets = cJSON_GetObjectItem(leg_item, "offsets_deg");
            if (!cJSON_IsNumber(leg_index) || !cJSON_IsArray(offsets)) {
                continue;
            }
            int leg = leg_index->valueint;
            if (leg < 0 || leg >= SCARGO_LEG_COUNT) {
                continue;
            }
            for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
                cJSON *value = cJSON_GetArrayItem(offsets, joint);
                if (cJSON_IsNumber(value)) {
                    s_config->calibration.servo_offsets_deg[leg][joint] = (int16_t)value->valuedouble;
                }
            }
        }
    }

    cJSON_Delete(root);
    config_store_validate(s_config);
    robot_control_update_calibration(&s_config->calibration);
    /*
     * 保存标定值时，如果当前已经在标定模式，就不要再次触发“进入标定”的过渡，
     * 否则每次 +/- 都会重新走一遍中位插值，表现成先跳一下再落到正确位置。
     */
    if (!robot_control_is_calibration_mode_active()) {
        robot_control_apply_mid_pose();
    }
    if (!storage_service_save_calibration(&s_config->calibration)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save calibration");
    }
    buzzer_service_save_beep();
    return send_json(req, "{\"status\":\"saved\"}");
}

static esp_err_t calibration_preview_post_handler(httpd_req_t *req)
{
    char *content = read_request_body(req);
    if (content == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No payload");
    }

    calibration_config_t preview = s_config->calibration;
    cJSON *root = cJSON_Parse(content);
    free(content);
    if (root == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }

    cJSON *legs = cJSON_GetObjectItem(root, "legs");
    if (cJSON_IsArray(legs)) {
        cJSON *leg_item = NULL;
        cJSON_ArrayForEach(leg_item, legs) {
            cJSON *leg_index = cJSON_GetObjectItem(leg_item, "leg");
            cJSON *offsets = cJSON_GetObjectItem(leg_item, "offsets_deg");
            if (!cJSON_IsNumber(leg_index) || !cJSON_IsArray(offsets)) {
                continue;
            }
            int leg = leg_index->valueint;
            if (leg < 0 || leg >= SCARGO_LEG_COUNT) {
                continue;
            }
            for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
                cJSON *value = cJSON_GetArrayItem(offsets, joint);
                if (cJSON_IsNumber(value)) {
                    preview.servo_offsets_deg[leg][joint] = (int16_t)value->valuedouble;
                }
            }
        }
    }

    cJSON_Delete(root);
    robot_control_update_calibration(&preview);
    /*
     * 预览偏置调整时同样遵循：
     * - 已在标定模式：只刷新偏置，不重复进入中位
     * - 未在标定模式：首次进入中位
     */
    if (!robot_control_is_calibration_mode_active()) {
        robot_control_apply_mid_pose();
    }
    return send_json(req, "{\"status\":\"preview\"}");
}

static esp_err_t calibration_start_post_handler(httpd_req_t *req)
{
    char *content = read_request_body(req);
    if (content == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No payload");
    }

    calibration_config_t preview = s_config->calibration;
    cJSON *root = cJSON_Parse(content);
    free(content);
    if (root == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }

    cJSON *legs = cJSON_GetObjectItem(root, "legs");
    if (cJSON_IsArray(legs)) {
        cJSON *leg_item = NULL;
        cJSON_ArrayForEach(leg_item, legs) {
            cJSON *leg_index = cJSON_GetObjectItem(leg_item, "leg");
            cJSON *offsets = cJSON_GetObjectItem(leg_item, "offsets_deg");
            if (!cJSON_IsNumber(leg_index) || !cJSON_IsArray(offsets)) {
                continue;
            }
            int leg = leg_index->valueint;
            if (leg < 0 || leg >= SCARGO_LEG_COUNT) {
                continue;
            }
            for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
                cJSON *value = cJSON_GetArrayItem(offsets, joint);
                if (cJSON_IsNumber(value)) {
                    preview.servo_offsets_deg[leg][joint] = (int16_t)value->valuedouble;
                }
            }
        }
    }

    cJSON_Delete(root);
    robot_control_update_calibration(&preview);
    robot_control_apply_mid_pose();
    return send_json(req, "{\"status\":\"calibration_started\"}");
}

static esp_err_t gait_post_handler(httpd_req_t *req)
{
    char *content = read_request_body(req);
    if (content == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No payload");
    }

    cJSON *root = cJSON_Parse(content);
    free(content);
    if (root == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }

    cJSON *item = cJSON_GetObjectItem(root, "step_height_mm");
    if (cJSON_IsNumber(item)) {
        s_config->gait.step_height_mm = (float)item->valuedouble;
    }
    item = cJSON_GetObjectItem(root, "step_cycle_ms");
    if (cJSON_IsNumber(item)) {
        s_config->gait.step_cycle_ms = item->valueint;
    }
    item = cJSON_GetObjectItem(root, "step_scale");
    if (cJSON_IsNumber(item)) {
        s_config->gait.step_scale = (float)item->valuedouble;
    }
    item = cJSON_GetObjectItem(root, "stance_ratio");
    if (cJSON_IsNumber(item)) {
        s_config->gait.stance_ratio = (float)item->valuedouble;
    }
    item = cJSON_GetObjectItem(root, "diagonal_phase_offset");
    if (cJSON_IsNumber(item)) {
        s_config->gait.diagonal_phase_offset = (float)item->valuedouble;
    }
    item = cJSON_GetObjectItem(root, "stand_height_default_mm");
    if (cJSON_IsNumber(item)) {
        s_config->gait.stand_height_default_mm = (float)item->valuedouble;
    }

    cJSON_Delete(root);
    config_store_validate(s_config);
    robot_control_update_gait(&s_config->gait);
    if (!storage_service_save_gait(&s_config->gait)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save gait");
    }
    buzzer_service_save_beep();
    return send_json(req, "{\"status\":\"saved\"}");
}

static robot_action_t action_from_name(const char *name)
{
    if (strcmp(name, "stand") == 0) {
        return ROBOT_ACTION_STAND;
    }
    if (strcmp(name, "lie_down") == 0) {
        return ROBOT_ACTION_LIE_DOWN;
    }
    if (strcmp(name, "walk") == 0) {
        return ROBOT_ACTION_WALK;
    }
    if (strcmp(name, "turn_left") == 0) {
        return ROBOT_ACTION_TURN_LEFT;
    }
    if (strcmp(name, "turn_right") == 0) {
        return ROBOT_ACTION_TURN_RIGHT;
    }
    if (strcmp(name, "forward") == 0) {
        return ROBOT_ACTION_FORWARD;
    }
    if (strcmp(name, "backward") == 0) {
        return ROBOT_ACTION_BACKWARD;
    }
    return ROBOT_ACTION_NONE;
}

static esp_err_t action_post_handler(httpd_req_t *req)
{
    char *content = read_request_body(req);
    if (content == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No payload");
    }

    cJSON *root = cJSON_Parse(content);
    free(content);
    if (root == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }

    cJSON *action_item = cJSON_GetObjectItem(root, "action");
    if (!cJSON_IsString(action_item)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing action");
    }

    robot_action_t action = action_from_name(action_item->valuestring);
    robot_control_set_action(action);
    cJSON_Delete(root);
    return send_json(req, "{\"status\":\"action_queued\"}");
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    attitude_state_t attitude = imu_service_get_attitude();
    rc_command_t command = rc_input_get_latest();
    fan_status_t fan = fan_service_get_status();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "mode", robot_control_get_mode() == ROBOT_MODE_WALK ? "walk" : "stand");
    cJSON_AddBoolToObject(root, "rc_link", command.link_up);
    cJSON_AddBoolToObject(root, "imu_ready", attitude.ready);
    cJSON_AddNumberToObject(root, "roll_deg", attitude.roll_deg);
    cJSON_AddNumberToObject(root, "pitch_deg", attitude.pitch_deg);
    cJSON_AddNumberToObject(root, "yaw_deg", attitude.yaw_deg);
    cJSON_AddNumberToObject(root, "roll_rate_dps", attitude.roll_rate_dps);
    cJSON_AddNumberToObject(root, "pitch_rate_dps", attitude.pitch_rate_dps);
    cJSON_AddNumberToObject(root, "throttle", command.throttle);
    robot_target_pose_t target = robot_control_get_target_pose(&command);
    cJSON_AddNumberToObject(root, "target_roll_deg", target.roll_deg);
    cJSON_AddNumberToObject(root, "target_pitch_deg", target.pitch_deg);
    cJSON_AddNumberToObject(root, "target_yaw_deg", target.yaw_deg);
    cJSON_AddNumberToObject(root, "target_height_mm", target.height_mm);
    cJSON_AddStringToObject(root, "oled_page", display_page_name(display_service_get_page()));
    cJSON_AddNumberToObject(root, "oled_leg", display_service_get_leg_preview_selection());
    cJSON_AddStringToObject(root, "oled_leg_view", preview_view_name(display_service_get_leg_preview_view()));
    cJSON_AddStringToObject(root, "oled_robot_view", preview_view_name(display_service_get_robot_preview_view()));
    cJSON_AddStringToObject(root, "fan_effective_level", fan_level_name(fan.effective_level));
    cJSON_AddStringToObject(root, "fan_manual_level", fan_level_name(fan.manual_level));
    cJSON_AddBoolToObject(root, "fan_manual_override", fan.manual_override);
    cJSON_AddBoolToObject(root, "fan_rc_link_up", fan.rc_link_up);
    cJSON_AddBoolToObject(root, "fan_at_min_height", fan.at_min_height);
    cJSON_AddBoolToObject(root, "fan_tach_ready", fan.tach_ready);
    cJSON_AddNumberToObject(root, "fan_rpm", (double)fan.rpm);

    cJSON_AddNumberToObject(root, "rc_roll", command.roll);
    cJSON_AddNumberToObject(root, "rc_pitch", command.pitch);
    cJSON_AddNumberToObject(root, "rc_yaw", command.yaw);
    cJSON_AddNumberToObject(root, "rc_throttle", command.throttle);
    cJSON *rc_aux = cJSON_AddArrayToObject(root, "rc_aux");
    cJSON_AddItemToArray(rc_aux, cJSON_CreateNumber(command.aux_sa));
    cJSON_AddItemToArray(rc_aux, cJSON_CreateNumber(command.aux_sb));
    cJSON_AddItemToArray(rc_aux, cJSON_CreateNumber(command.aux_sc));
    cJSON_AddItemToArray(rc_aux, cJSON_CreateNumber(command.aux_sd));
    for (int i = 8; i < 12; i++) {
        int16_t sw = command.raw_channels[i] <= SCARGO_CRSF_SWITCH_LOW ? -1 :
                     command.raw_channels[i] >= SCARGO_CRSF_SWITCH_HIGH ? 1 : 0;
        cJSON_AddItemToArray(rc_aux, cJSON_CreateNumber(sw));
    }

    char *json = cJSON_PrintUnformatted(root);
    esp_err_t err = send_json(req, json);
    cJSON_free(json);
    cJSON_Delete(root);
    return err;
}

static esp_err_t oled_page_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "page", display_page_name(display_service_get_page()));
    cJSON_AddNumberToObject(root, "leg", display_service_get_leg_preview_selection());
    cJSON_AddStringToObject(root, "leg_view", preview_view_name(display_service_get_leg_preview_view()));
    cJSON_AddStringToObject(root, "robot_view", preview_view_name(display_service_get_robot_preview_view()));
    char *json = cJSON_PrintUnformatted(root);
    esp_err_t err = send_json(req, json);
    cJSON_free(json);
    cJSON_Delete(root);
    return err;
}

static esp_err_t oled_page_post_handler(httpd_req_t *req)
{
    char *content = read_request_body(req);
    if (content == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No payload");
    }

    cJSON *root = cJSON_Parse(content);
    free(content);
    if (root == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }

    cJSON *page_item = cJSON_GetObjectItem(root, "page");
    if (!cJSON_IsString(page_item)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing page");
    }

    display_page_t page;
    if (!display_page_from_name(page_item->valuestring, &page)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid page");
    }

    display_service_set_page(page);
    s_config->oled.page = page;
    cJSON *leg_item = cJSON_GetObjectItem(root, "leg");
    if (cJSON_IsNumber(leg_item)) {
        display_service_set_leg_preview_selection(leg_item->valueint);
        s_config->oled.leg = leg_item->valueint;
    }

    cJSON *leg_view_item = cJSON_GetObjectItem(root, "leg_view");
    if (cJSON_IsString(leg_view_item)) {
        display_preview_view_t leg_view;
        if (!preview_view_from_name(leg_view_item->valuestring, &leg_view)) {
            cJSON_Delete(root);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid leg view");
        }
        display_service_set_leg_preview_view(leg_view);
        s_config->oled.leg_view = (int)leg_view;
    }

    cJSON *robot_view_item = cJSON_GetObjectItem(root, "robot_view");
    if (cJSON_IsString(robot_view_item)) {
        display_preview_view_t robot_view;
        if (!preview_view_from_name(robot_view_item->valuestring, &robot_view)) {
            cJSON_Delete(root);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid robot view");
        }
        display_service_set_robot_preview_view(robot_view);
        s_config->oled.robot_view = (int)robot_view;
    }

    config_store_validate(s_config);
    cJSON_Delete(root);
    if (!storage_service_save_oled(&s_config->oled)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save oled");
    }
    return send_json(req, "{\"status\":\"oled_page_set\"}");
}

static esp_err_t imu_yaw_zero_post_handler(httpd_req_t *req)
{
    imu_service_zero_yaw();
    return send_json(req, "{\"status\":\"imu_yaw_zeroed\"}");
}

static esp_err_t fan_get_handler(httpd_req_t *req)
{
    fan_status_t fan = fan_service_get_status();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "manual_override", fan.manual_override);
    cJSON_AddBoolToObject(root, "rc_link_up", fan.rc_link_up);
    cJSON_AddStringToObject(root, "manual_level", fan_level_name(fan.manual_level));
    cJSON_AddStringToObject(root, "effective_level", fan_level_name(fan.effective_level));
    cJSON_AddStringToObject(root, "mode", fan.walk_mode ? "walk" : "stand");
    cJSON_AddBoolToObject(root, "at_min_height", fan.at_min_height);
    cJSON_AddBoolToObject(root, "tach_ready", fan.tach_ready);
    cJSON_AddNumberToObject(root, "rpm", (double)fan.rpm);
    cJSON_AddStringToObject(root, "source", fan.manual_override ? "manual" : "mode");

    char *json = cJSON_PrintUnformatted(root);
    esp_err_t err = send_json(req, json);
    cJSON_free(json);
    cJSON_Delete(root);
    return err;
}

static esp_err_t fan_post_handler(httpd_req_t *req)
{
    char *content = read_request_body(req);
    if (content == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No payload");
    }

    cJSON *root = cJSON_Parse(content);
    free(content);
    if (root == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }

    cJSON *mode_item = cJSON_GetObjectItem(root, "mode");
    if (cJSON_IsString(mode_item) && strcmp(mode_item->valuestring, "auto") == 0) {
        fan_service_clear_manual_override();
        cJSON_Delete(root);
        return send_json(req, "{\"status\":\"fan_saved\"}");
    }

    cJSON *level_item = cJSON_GetObjectItem(root, "level");
    if (!cJSON_IsString(level_item)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing level");
    }

    fan_speed_level_t level;
    if (!fan_level_from_name(level_item->valuestring, &level)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid level");
    }

    fan_service_set_manual_level(level);
    cJSON_Delete(root);
    return send_json(req, "{\"status\":\"fan_saved\"}");
}

static esp_err_t calibration_file_get_handler(httpd_req_t *req)
{
    char *json = storage_service_read_calibration_json();
    if (json == NULL) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Calibration file not found");
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, json);
    free(json);
    return err;
}

static esp_err_t gait_file_get_handler(httpd_req_t *req)
{
    char *json = storage_service_read_gait_json();
    if (json == NULL) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Gait file not found");
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, json);
    free(json);
    return err;
}

static esp_err_t imu_config_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "balance_enabled", s_config->imu.balance_enabled);
    cJSON_AddNumberToObject(root, "mount_rotation_deg", s_config->imu.mount_rotation_deg);
    cJSON_AddBoolToObject(root, "mount_flip", s_config->imu.mount_flip);
    char *json = cJSON_PrintUnformatted(root);
    esp_err_t err = send_json(req, json);
    cJSON_free(json);
    cJSON_Delete(root);
    return err;
}

static esp_err_t imu_config_post_handler(httpd_req_t *req)
{
    char *content = read_request_body(req);
    if (content == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No payload");
    }
    cJSON *root = cJSON_Parse(content);
    free(content);
    if (root == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }
    cJSON *item = cJSON_GetObjectItem(root, "balance_enabled");
    if (cJSON_IsBool(item)) {
        s_config->imu.balance_enabled = cJSON_IsTrue(item);
    }
    item = cJSON_GetObjectItem(root, "mount_rotation_deg");
    if (cJSON_IsNumber(item)) {
        const int r = item->valueint;
        s_config->imu.mount_rotation_deg = (r == 90 || r == 180 || r == 270) ? r : 0;
    }
    item = cJSON_GetObjectItem(root, "mount_flip");
    if (cJSON_IsBool(item)) {
        s_config->imu.mount_flip = cJSON_IsTrue(item);
    }
    cJSON_Delete(root);
    robot_control_update_imu(&s_config->imu);
    imu_service_update_mount(s_config->imu.mount_rotation_deg, s_config->imu.mount_flip);
    if (!storage_service_save_imu(&s_config->imu)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save imu config");
    }
    return send_json(req, "{\"status\":\"saved\"}");
}

static esp_err_t version_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "fw", SCARGO_FW_VERSION_STR);
    char *json = cJSON_PrintUnformatted(root);
    esp_err_t err = send_json(req, json);
    cJSON_free(json);
    cJSON_Delete(root);
    return err;
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    return ota_service_handle_upload(req);
}

static esp_err_t ota_spiffs_post_handler(httpd_req_t *req)
{
    return ota_service_handle_spiffs_upload(req);
}

static esp_err_t log_get_handler(httpd_req_t *req)
{
    char query_buf[32] = "";
    char cursor_str[16] = "0";
    if (httpd_req_get_url_query_str(req, query_buf, sizeof(query_buf)) == ESP_OK) {
        httpd_query_key_value(query_buf, "cursor", cursor_str, sizeof(cursor_str));
    }
    uint32_t cursor = (uint32_t)strtoul(cursor_str, NULL, 10);
    char *json = log_service_snapshot_json(cursor);
    if (json == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    }
    esp_err_t err = send_json(req, json);
    free(json);
    return err;
}

static esp_err_t wifi_get_handler(httpd_req_t *req)
{
    wifi_info_t info = wifi_config_get_info();
    const char *status_str;
    switch (info.sta_status) {
    case WIFI_STA_CONNECTED:  status_str = "connected";  break;
    case WIFI_STA_CONNECTING: status_str = "connecting"; break;
    case WIFI_STA_AUTH_FAIL:  status_str = "auth_fail";  break;
    case WIFI_STA_FAILED:     status_str = "failed";     break;
    default:                  status_str = "idle";       break;
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "sta_configured", info.sta_configured);
    cJSON_AddStringToObject(root, "sta_status", status_str);
    cJSON_AddStringToObject(root, "sta_ssid", info.sta_ssid);
    cJSON_AddStringToObject(root, "sta_ip", info.sta_ip);
    cJSON_AddStringToObject(root, "ap_ssid", info.ap_ssid);
    char *json = cJSON_PrintUnformatted(root);
    esp_err_t err = send_json(req, json);
    cJSON_free(json);
    cJSON_Delete(root);
    return err;
}

static esp_err_t wifi_post_handler(httpd_req_t *req)
{
    char *content = read_request_body(req);
    if (content == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No payload");
    }
    cJSON *root = cJSON_Parse(content);
    free(content);
    if (root == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }
    cJSON *ssid_item = cJSON_GetObjectItem(root, "ssid");
    if (!cJSON_IsString(ssid_item) || ssid_item->valuestring[0] == '\0') {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing ssid");
    }
    char ssid[33] = {0};
    char pass[65] = {0};
    strlcpy(ssid, ssid_item->valuestring, sizeof(ssid));
    cJSON *pass_item = cJSON_GetObjectItem(root, "password");
    if (cJSON_IsString(pass_item)) {
        strlcpy(pass, pass_item->valuestring, sizeof(pass));
    }
    cJSON_Delete(root);
    send_json(req, "{\"status\":\"wifi_saving\"}");
    wifi_config_save_and_restart(ssid, pass);
    return ESP_OK;
}

static esp_err_t wifi_forget_post_handler(httpd_req_t *req)
{
    send_json(req, "{\"status\":\"wifi_forgetting\"}");
    wifi_config_forget_and_restart();
    return ESP_OK;
}

void web_ui_start(system_config_t *config)
{
    s_config = config;
    wifi_config_start();

    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.max_uri_handlers = 28;
    server_config.recv_wait_timeout = 30;

    ESP_ERROR_CHECK(httpd_start(&s_server, &server_config));

    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t calibration_get = {
        .uri = "/api/calibration",
        .method = HTTP_GET,
        .handler = calibration_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t calibration_post = {
        .uri = "/api/calibration",
        .method = HTTP_POST,
        .handler = calibration_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t calibration_preview_post = {
        .uri = "/api/calibration/preview",
        .method = HTTP_POST,
        .handler = calibration_preview_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t calibration_start_post = {
        .uri = "/api/calibration/start",
        .method = HTTP_POST,
        .handler = calibration_start_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t gait_get = {
        .uri = "/api/gait",
        .method = HTTP_GET,
        .handler = gait_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t gait_post = {
        .uri = "/api/gait",
        .method = HTTP_POST,
        .handler = gait_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t action_post = {
        .uri = "/api/action",
        .method = HTTP_POST,
        .handler = action_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t status_get = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = status_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t oled_page_get = {
        .uri = "/api/oled/page",
        .method = HTTP_GET,
        .handler = oled_page_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t oled_page_post = {
        .uri = "/api/oled/page",
        .method = HTTP_POST,
        .handler = oled_page_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t imu_yaw_zero_post = {
        .uri = "/api/imu/yaw-zero",
        .method = HTTP_POST,
        .handler = imu_yaw_zero_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t fan_get = {
        .uri = "/api/fan",
        .method = HTTP_GET,
        .handler = fan_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t fan_post = {
        .uri = "/api/fan",
        .method = HTTP_POST,
        .handler = fan_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t calibration_file_get = {
        .uri = "/api/debug/calibration-file",
        .method = HTTP_GET,
        .handler = calibration_file_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t gait_file_get = {
        .uri = "/api/debug/gait-file",
        .method = HTTP_GET,
        .handler = gait_file_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t imu_config_get = {
        .uri = "/api/imu",
        .method = HTTP_GET,
        .handler = imu_config_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t imu_config_post = {
        .uri = "/api/imu",
        .method = HTTP_POST,
        .handler = imu_config_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t version_get = {
        .uri = "/api/version",
        .method = HTTP_GET,
        .handler = version_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t ota_post = {
        .uri = "/api/ota",
        .method = HTTP_POST,
        .handler = ota_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t log_get = {
        .uri = "/api/log",
        .method = HTTP_GET,
        .handler = log_get_handler,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &calibration_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &calibration_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &calibration_preview_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &calibration_start_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &gait_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &gait_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &action_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &status_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &oled_page_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &oled_page_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &imu_yaw_zero_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &fan_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &fan_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &calibration_file_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &gait_file_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &imu_config_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &imu_config_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &version_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &ota_post));
    httpd_uri_t ota_spiffs_post = {
        .uri = "/api/ota/spiffs",
        .method = HTTP_POST,
        .handler = ota_spiffs_post_handler,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &ota_spiffs_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &log_get));

    httpd_uri_t wifi_get = {
        .uri = "/api/wifi",
        .method = HTTP_GET,
        .handler = wifi_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t wifi_post = {
        .uri = "/api/wifi",
        .method = HTTP_POST,
        .handler = wifi_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t wifi_forget_post = {
        .uri = "/api/wifi/forget",
        .method = HTTP_POST,
        .handler = wifi_forget_post_handler,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &wifi_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &wifi_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &wifi_forget_post));

    ESP_LOGI(TAG, "Calibration/test web UI started");
}
