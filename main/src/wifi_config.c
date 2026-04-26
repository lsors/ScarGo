#include "wifi_config.h"

#include <string.h>

#include "board_defaults.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mdns.h"
#include "nvs.h"

static const char *TAG = "wifi_config";

#define NVS_NAMESPACE "wifi_cfg"
#define NVS_KEY_SSID  "ssid"
#define NVS_KEY_PASS  "pass"

static wifi_info_t s_info;
static SemaphoreHandle_t s_mutex;

static bool load_credentials(char *ssid, size_t ssid_cap, char *pass, size_t pass_cap)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }
    bool ok = (nvs_get_str(nvs, NVS_KEY_SSID, ssid, &ssid_cap) == ESP_OK &&
               nvs_get_str(nvs, NVS_KEY_PASS, pass, &pass_cap) == ESP_OK &&
               ssid[0] != '\0');
    nvs_close(nvs);
    return ok;
}

static void save_credentials(const char *ssid, const char *pass)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed");
        return;
    }
    nvs_set_str(nvs, NVS_KEY_SSID, ssid);
    nvs_set_str(nvs, NVS_KEY_PASS, pass);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Credentials saved for SSID: %s", ssid);
}

static void clear_credentials(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }
    nvs_erase_key(nvs, NVS_KEY_SSID);
    nvs_erase_key(nvs, NVS_KEY_PASS);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Credentials cleared");
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "STA start, connecting...");
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGW(TAG, "STA disconnected, reason=%d", d->reason);

            xSemaphoreTake(s_mutex, portMAX_DELAY);
            memset(s_info.sta_ip, 0, sizeof(s_info.sta_ip));
            if (d->reason == WIFI_REASON_AUTH_FAIL ||
                d->reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT) {
                s_info.sta_status = WIFI_STA_AUTH_FAIL;
                xSemaphoreGive(s_mutex);
                ESP_LOGW(TAG, "Auth failed — check password");
            } else {
                s_info.sta_status = WIFI_STA_CONNECTING;
                xSemaphoreGive(s_mutex);
                esp_wifi_connect();
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_info.sta_status = WIFI_STA_CONNECTED;
        snprintf(s_info.sta_ip, sizeof(s_info.sta_ip), IPSTR, IP2STR(&event->ip_info.ip));
        xSemaphoreGive(s_mutex);
        ESP_LOGI(TAG, "STA connected — IP: %s", s_info.sta_ip);
    }
}

void wifi_config_start(void)
{
    s_mutex = xSemaphoreCreateMutex();
    memset(&s_info, 0, sizeof(s_info));

    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_ap();

    char ssid[33] = {0};
    char pass[65] = {0};
    bool has_creds = load_credentials(ssid, sizeof(ssid), pass, sizeof(pass));

    if (has_creds) {
        esp_netif_create_default_wifi_sta();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    wifi_config_t ap_cfg = {
        .ap = {
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    strlcpy((char *)ap_cfg.ap.ssid, SCARGO_WIFI_AP_SSID, sizeof(ap_cfg.ap.ssid));
    strlcpy((char *)ap_cfg.ap.password, SCARGO_WIFI_AP_PASSWORD, sizeof(ap_cfg.ap.password));
    if (strlen(SCARGO_WIFI_AP_PASSWORD) == 0) {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    strlcpy(s_info.ap_ssid, SCARGO_WIFI_AP_SSID, sizeof(s_info.ap_ssid));

    if (has_creds) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));

        wifi_config_t sta_cfg = {0};
        strlcpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid));
        strlcpy((char *)sta_cfg.sta.password, pass, sizeof(sta_cfg.sta.password));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));

        s_info.sta_configured = true;
        s_info.sta_status = WIFI_STA_CONNECTING;
        strlcpy(s_info.sta_ssid, ssid, sizeof(s_info.sta_ssid));
        ESP_LOGI(TAG, "AP+STA mode, connecting to '%s'", ssid);
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
        ESP_LOGI(TAG, "AP-only mode");
    }

    ESP_ERROR_CHECK(esp_wifi_start());

    if (mdns_init() == ESP_OK) {
        mdns_hostname_set("scargo");
        mdns_instance_name_set("ScarGo Robot");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        ESP_LOGI(TAG, "mDNS ready: http://scargo.local");
    }
}

wifi_info_t wifi_config_get_info(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    wifi_info_t info = s_info;
    xSemaphoreGive(s_mutex);
    return info;
}

void wifi_config_save_and_restart(const char *ssid, const char *password)
{
    save_credentials(ssid, password);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

void wifi_config_forget_and_restart(void)
{
    clear_credentials();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}
