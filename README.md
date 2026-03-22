# ScarGo

ScarGo is an ESP-IDF firmware project for an ESP32-S3 based quadruped robot dog.

## Current Status

The repository currently contains:

- a minimal ESP-IDF application that builds for ESP32-S3
- an initial hardware specification document
- early architecture notes for the firmware stack

## Project Baseline

- Main MCU: ESP32-S3
- Framework: ESP-IDF
- Degrees of freedom: 12
- Servo controller: PCA9685
- IMU: MPU6050
- Display: SSD1306 OLED
- RC input: ELRS receiver over UART

## Important Documents

- `docs/index.md`: documentation index
- `docs/index.zh-CN.md`: Chinese documentation index
- `README.zh-CN.md`: Chinese project overview
- `docs/hardware-spec.md`: current mechanical and electrical specification
- `docs/architecture.md`: current firmware architecture direction

## Build

Example local build flow:

```sh
export IDF_PATH=/Users/mac/esp/v5.4/esp-idf
. "$IDF_PATH/export.sh"
idf.py set-target esp32s3 build
```

## Notes

This project is expected to evolve quickly. Hardware interpretation, calibration values, and joint formulas should be kept in documentation before they are locked into code.
