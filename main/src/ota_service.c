#include "ota_service.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota_service";

esp_err_t ota_service_handle_upload(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Length required");
    }

    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (part == NULL) {
        ESP_LOGE(TAG, "No OTA update partition found");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
    }

    ESP_LOGI(TAG, "OTA begin: writing %d bytes to partition '%s'", req->content_len, part->label);

    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
    }

    char buf[1024];
    int remaining = req->content_len;
    bool failed = false;

    while (remaining > 0) {
        int chunk = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
        int received = httpd_req_recv(req, buf, (size_t)chunk);
        if (received <= 0) {
            ESP_LOGE(TAG, "OTA receive error at %d bytes remaining", remaining);
            failed = true;
            break;
        }
        err = esp_ota_write(handle, buf, (size_t)received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            failed = true;
            break;
        }
        remaining -= received;
    }

    if (failed) {
        esp_ota_abort(handle);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA receive failed");
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA validation failed");
    }

    err = esp_ota_set_boot_partition(part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA set boot failed");
    }

    ESP_LOGI(TAG, "OTA complete, rebooting into '%s'", part->label);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ota_ok\"}");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

esp_err_t ota_service_handle_spiffs_upload(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Length required");
    }

    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "storage");
    if (part == NULL) {
        ESP_LOGE(TAG, "storage partition not found");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No storage partition");
    }

    int total = req->content_len;
    if (total > (int)part->size) {
        ESP_LOGE(TAG, "SPIFFS image too large: %d > %u", total, (unsigned)part->size);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image too large");
    }

    ESP_LOGI(TAG, "SPIFFS OTA begin: %d bytes -> partition '%s'", total, part->label);

    /* 先卸载 SPIFFS，再擦写，避免文件系统读到残留数据 */
    esp_vfs_spiffs_unregister("storage");

    size_t erase_size = ((size_t)total + 0xFFFu) & ~0xFFFu;
    esp_err_t err = esp_partition_erase_range(part, 0, erase_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "erase failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Erase failed");
    }

    static char buf[1024];
    int remaining = total;
    size_t offset = 0;
    bool failed = false;

    while (remaining > 0) {
        int chunk = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
        int received = httpd_req_recv(req, buf, (size_t)chunk);
        if (received <= 0) {
            ESP_LOGE(TAG, "receive error at offset %u", (unsigned)offset);
            failed = true;
            break;
        }
        err = esp_partition_write(part, offset, buf, (size_t)received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "write failed at offset %u: %s", (unsigned)offset, esp_err_to_name(err));
            failed = true;
            break;
        }
        offset += (size_t)received;
        remaining -= received;
    }

    if (failed) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
    }

    ESP_LOGI(TAG, "SPIFFS OTA complete (%u bytes), rebooting", (unsigned)offset);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"spiffs_ok\"}");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}
