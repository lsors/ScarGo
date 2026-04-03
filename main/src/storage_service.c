#include "storage_service.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "cJSON.h"
#include "config_store.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_spiffs.h"

static const char *TAG = "storage";
static const char *CALIBRATION_PATH = "/spiffs/calibration.json";
static const char *GAIT_PATH = "/spiffs/gait.json";
static const char *OLED_PATH = "/spiffs/oled.json";

static void log_json_blob(const char *label, const char *json_text)
{
    if (json_text == NULL) {
        ESP_LOGW(TAG, "%s JSON is null", label);
        return;
    }
    ESP_LOGI(TAG, "%s JSON:\n%s", label, json_text);
}

static char *read_text_file(const char *path)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        return NULL;
    }

    fseek(file, 0, SEEK_END);
    long size = ftell(file);
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
    return buffer;
}

static bool write_text_file(const char *path, const char *content)
{
    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        return false;
    }
    size_t written = fwrite(content, 1U, strlen(content), file);
    fclose(file);
    return written == strlen(content);
}

static cJSON *create_calibration_json(const calibration_config_t *config)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *legs = cJSON_AddArrayToObject(root, "legs");

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        cJSON *leg_item = cJSON_CreateObject();
        cJSON *offsets = cJSON_AddArrayToObject(leg_item, "offsets_deg");

        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            cJSON_AddItemToArray(offsets, cJSON_CreateNumber(config->servo_offsets_deg[leg][joint]));
        }

        cJSON_AddNumberToObject(leg_item, "leg", leg);
        cJSON_AddItemToArray(legs, leg_item);
    }

    return root;
}

static cJSON *create_gait_json(const gait_config_t *config)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "stand_height_default_mm", config->stand_height_default_mm);
    cJSON_AddNumberToObject(root, "stand_height_min_mm", config->stand_height_min_mm);
    cJSON_AddNumberToObject(root, "stand_height_max_mm", config->stand_height_max_mm);
    cJSON_AddNumberToObject(root, "max_body_yaw_deg", config->max_body_yaw_deg);
    cJSON_AddNumberToObject(root, "max_body_roll_deg", config->max_body_roll_deg);
    cJSON_AddNumberToObject(root, "max_body_pitch_deg", config->max_body_pitch_deg);
    cJSON_AddNumberToObject(root, "step_height_mm", config->step_height_mm);
    cJSON_AddNumberToObject(root, "step_height_min_mm", config->step_height_min_mm);
    cJSON_AddNumberToObject(root, "step_height_max_mm", config->step_height_max_mm);
    cJSON_AddNumberToObject(root, "step_cycle_ms", config->step_cycle_ms);
    cJSON_AddNumberToObject(root, "step_cycle_min_ms", config->step_cycle_min_ms);
    cJSON_AddNumberToObject(root, "step_cycle_max_ms", config->step_cycle_max_ms);
    cJSON_AddNumberToObject(root, "step_scale", config->step_scale);
    cJSON_AddNumberToObject(root, "step_scale_min", config->step_scale_min);
    cJSON_AddNumberToObject(root, "step_scale_max", config->step_scale_max);
    cJSON_AddNumberToObject(root, "stance_ratio", config->stance_ratio);
    cJSON_AddNumberToObject(root, "stance_ratio_min", config->stance_ratio_min);
    cJSON_AddNumberToObject(root, "stance_ratio_max", config->stance_ratio_max);
    cJSON_AddNumberToObject(root, "diagonal_phase_offset", config->diagonal_phase_offset);
    return root;
}

static cJSON *create_oled_json(const oled_config_t *config)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "page", config->page);
    cJSON_AddNumberToObject(root, "leg", config->leg);
    cJSON_AddNumberToObject(root, "leg_view", config->leg_view);
    cJSON_AddNumberToObject(root, "robot_view", config->robot_view);
    return root;
}

static bool save_json_file(const char *path, cJSON *root)
{
    char *serialized = cJSON_Print(root);
    bool ok = false;
    if (serialized != NULL) {
        ok = write_text_file(path, serialized);
        if (ok) {
            log_json_blob(path, serialized);
        } else {
            ESP_LOGW(TAG, "Failed to write JSON file: %s", path);
        }
        cJSON_free(serialized);
    }
    cJSON_Delete(root);
    return ok;
}

static void load_calibration_from_json(calibration_config_t *config, cJSON *root)
{
    cJSON *legs = cJSON_GetObjectItem(root, "legs");
    if (!cJSON_IsArray(legs)) {
        return;
    }

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
                config->servo_offsets_deg[leg][joint] = (int16_t)value->valuedouble;
            }
        }
    }
}

static void load_gait_from_json(gait_config_t *config, cJSON *root)
{
    const struct {
        const char *key;
        float *target;
    } float_fields[] = {
        {"stand_height_default_mm", &config->stand_height_default_mm},
        {"stand_height_min_mm", &config->stand_height_min_mm},
        {"stand_height_max_mm", &config->stand_height_max_mm},
        {"max_body_yaw_deg", &config->max_body_yaw_deg},
        {"max_body_roll_deg", &config->max_body_roll_deg},
        {"max_body_pitch_deg", &config->max_body_pitch_deg},
        {"step_height_mm", &config->step_height_mm},
        {"step_height_min_mm", &config->step_height_min_mm},
        {"step_height_max_mm", &config->step_height_max_mm},
        {"step_scale", &config->step_scale},
        {"step_scale_min", &config->step_scale_min},
        {"step_scale_max", &config->step_scale_max},
        {"stance_ratio", &config->stance_ratio},
        {"stance_ratio_min", &config->stance_ratio_min},
        {"stance_ratio_max", &config->stance_ratio_max},
        {"diagonal_phase_offset", &config->diagonal_phase_offset},
    };

    for (size_t i = 0; i < sizeof(float_fields) / sizeof(float_fields[0]); ++i) {
        cJSON *item = cJSON_GetObjectItem(root, float_fields[i].key);
        if (cJSON_IsNumber(item)) {
            *float_fields[i].target = (float)item->valuedouble;
        }
    }

    const struct {
        const char *key;
        int *target;
    } int_fields[] = {
        {"step_cycle_ms", &config->step_cycle_ms},
        {"step_cycle_min_ms", &config->step_cycle_min_ms},
        {"step_cycle_max_ms", &config->step_cycle_max_ms},
    };

    for (size_t i = 0; i < sizeof(int_fields) / sizeof(int_fields[0]); ++i) {
        cJSON *item = cJSON_GetObjectItem(root, int_fields[i].key);
        if (cJSON_IsNumber(item)) {
            *int_fields[i].target = item->valueint;
        }
    }
}

static void load_oled_from_json(oled_config_t *config, cJSON *root)
{
    const struct {
        const char *key;
        int *target;
    } int_fields[] = {
        {"page", &config->page},
        {"leg", &config->leg},
        {"leg_view", &config->leg_view},
        {"robot_view", &config->robot_view},
    };

    for (size_t i = 0; i < sizeof(int_fields) / sizeof(int_fields[0]); ++i) {
        cJSON *item = cJSON_GetObjectItem(root, int_fields[i].key);
        if (cJSON_IsNumber(item)) {
            *int_fields[i].target = item->valueint;
        }
    }
}

bool storage_service_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 6,
        .format_if_mount_failed = true,
    };

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS: %s", esp_err_to_name(err));
        return false;
    }

    size_t total = 0;
    size_t used = 0;
    if (esp_spiffs_info(NULL, &total, &used) == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted total=%u used=%u", (unsigned)total, (unsigned)used);
    }

    return true;
}

bool storage_service_load_all(system_config_t *config)
{
    bool ok = true;

    char *calibration_text = read_text_file(CALIBRATION_PATH);
    if (calibration_text == NULL) {
        ok &= storage_service_save_calibration(&config->calibration);
    } else {
        log_json_blob("loaded calibration", calibration_text);
        cJSON *root = cJSON_Parse(calibration_text);
        if (root != NULL) {
            load_calibration_from_json(&config->calibration, root);
            cJSON_Delete(root);
        }
        free(calibration_text);
    }

    char *gait_text = read_text_file(GAIT_PATH);
    if (gait_text == NULL) {
        ok &= storage_service_save_gait(&config->gait);
    } else {
        log_json_blob("loaded gait", gait_text);
        cJSON *root = cJSON_Parse(gait_text);
        if (root != NULL) {
            load_gait_from_json(&config->gait, root);
            cJSON_Delete(root);
        }
        free(gait_text);
    }

    char *oled_text = read_text_file(OLED_PATH);
    if (oled_text == NULL) {
        ok &= storage_service_save_oled(&config->oled);
    } else {
        log_json_blob("loaded oled", oled_text);
        cJSON *root = cJSON_Parse(oled_text);
        if (root != NULL) {
            load_oled_from_json(&config->oled, root);
            cJSON_Delete(root);
        }
        free(oled_text);
    }

    return ok;
}

bool storage_service_save_calibration(const calibration_config_t *config)
{
    return save_json_file(CALIBRATION_PATH, create_calibration_json(config));
}

bool storage_service_save_gait(const gait_config_t *config)
{
    return save_json_file(GAIT_PATH, create_gait_json(config));
}

bool storage_service_save_oled(const oled_config_t *config)
{
    return save_json_file(OLED_PATH, create_oled_json(config));
}

char *storage_service_read_calibration_json(void)
{
    return read_text_file(CALIBRATION_PATH);
}

char *storage_service_read_gait_json(void)
{
    return read_text_file(GAIT_PATH);
}

char *storage_service_read_oled_json(void)
{
    return read_text_file(OLED_PATH);
}
