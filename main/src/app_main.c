#include "app_tasks.h"
#include "board_defaults.h"
#include "buzzer_service.h"
#include "config_store.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "storage_service.h"

static const char *TAG = "scargo_app";
static system_config_t s_config;

static void configure_cpu_frequency(void)
{
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = SCARGO_CPU_FREQ_MHZ,
        .min_freq_mhz = SCARGO_CPU_FREQ_MHZ,
        .light_sleep_enable = false,
    };
    esp_err_t err = esp_pm_configure(&pm_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_pm_configure failed: %s", esp_err_to_name(err));
    }
#else
    ESP_LOGI(TAG, "Power management is disabled in sdkconfig; keeping default CPU frequency");
#endif
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    configure_cpu_frequency();

    buzzer_service_init();
    buzzer_service_boot_beep();

    config_store_set_defaults(&s_config);

    if (!storage_service_init()) {
        ESP_LOGW(TAG, "Storage init failed, continuing with default config");
    } else if (!storage_service_load_all(&s_config)) {
        ESP_LOGW(TAG, "Using default runtime config");
    }

    if (!config_store_validate(&s_config)) {
        ESP_LOGW(TAG, "Loaded config was out of range, clamped to safe defaults");
    }

    app_tasks_start(&s_config);
}
