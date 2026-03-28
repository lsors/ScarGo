#include "web_ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "config_store.h"
#include "board_defaults.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "imu_service.h"
#include "rc_input.h"
#include "robot_control.h"
#include "storage_service.h"

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
    robot_control_apply_mid_pose();
    if (!storage_service_save_calibration(&s_config->calibration)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save calibration");
    }
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
    robot_control_apply_mid_pose();
    return send_json(req, "{\"status\":\"preview\"}");
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
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "mode", robot_control_get_mode() == ROBOT_MODE_WALK ? "walk" : "stand");
    cJSON_AddBoolToObject(root, "rc_link", command.link_up);
    cJSON_AddNumberToObject(root, "roll_deg", attitude.roll_deg);
    cJSON_AddNumberToObject(root, "pitch_deg", attitude.pitch_deg);
    cJSON_AddNumberToObject(root, "yaw_deg", attitude.yaw_deg);
    cJSON_AddNumberToObject(root, "throttle", command.throttle);

    char *json = cJSON_PrintUnformatted(root);
    esp_err_t err = send_json(req, json);
    cJSON_free(json);
    cJSON_Delete(root);
    return err;
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

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SCARGO_WIFI_AP_SSID,
            .ssid_len = 0,
            .password = SCARGO_WIFI_AP_PASSWORD,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    if (strlen((char *)wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void web_ui_start(system_config_t *config)
{
    s_config = config;
    wifi_init_softap();

    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.max_uri_handlers = 10;

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

    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &calibration_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &calibration_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &calibration_preview_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &gait_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &gait_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &action_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &status_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &calibration_file_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &gait_file_get));

    ESP_LOGI(TAG, "Calibration/test web UI started");
}
