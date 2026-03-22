#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

static const char *TAG = "ScarGo";

void app_main(void)
{
    esp_chip_info_t chip_info;
    uint32_t flash_size = 0;

    esp_chip_info(&chip_info);
    esp_flash_get_size(NULL, &flash_size);

    ESP_LOGI(TAG, "Hello world from ScarGo on ESP32-S3");
    ESP_LOGI(TAG, "This chip has %d CPU core(s), WiFi%s%s, silicon revision v%d.%d, %lu MB %s flash",
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             chip_info.revision / 100,
             chip_info.revision % 100,
             flash_size / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    while (1) {
        ESP_LOGI(TAG, "ScarGo heartbeat");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
