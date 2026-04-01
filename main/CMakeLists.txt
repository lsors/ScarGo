set(SOURCES
    src/app_main.c
    src/app_tasks.c
    src/board_defaults.c
    src/buzzer_service.c
    src/config_store.c
    src/cpu_usage_service.c
    src/display_service.c
    src/imu_service.c
    src/kinematics.c
    src/pca9685_driver.c
    src/rc_input.c
    src/robot_control.c
    src/servo_output.c
    src/shared_i2c.c
    src/storage_service.c
    src/web_ui.c
    )

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_uart esp_event esp_http_server esp_netif esp_pm esp_wifi json nvs_flash spiffs
    )
