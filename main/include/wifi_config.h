#pragma once

#include <stdbool.h>

typedef enum {
    WIFI_STA_IDLE = 0,
    WIFI_STA_CONNECTING,
    WIFI_STA_CONNECTED,
    WIFI_STA_AUTH_FAIL,
    WIFI_STA_FAILED,
} wifi_sta_status_t;

typedef struct {
    bool sta_configured;
    wifi_sta_status_t sta_status;
    char sta_ssid[33];
    char sta_ip[16];
    char ap_ssid[33];
} wifi_info_t;

// Initializes WiFi (AP always on; STA if credentials are stored in NVS) + mDNS.
// Must be called after esp_event_loop_create_default() and nvs_flash_init().
void wifi_config_start(void);

wifi_info_t wifi_config_get_info(void);

// Save new home-network credentials to NVS, then restart.
void wifi_config_save_and_restart(const char *ssid, const char *password);

// Erase stored credentials from NVS, then restart (back to AP-only mode).
void wifi_config_forget_and_restart(void);
