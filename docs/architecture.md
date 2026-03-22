# ScarGo Architecture Notes

## Current Project Direction

ScarGo is an ESP-IDF firmware project for an ESP32-S3 based 12-DOF quadruped robot.

The current hardware direction is:

- ESP32-S3 as the only main controller
- PCA9685 for 12 servo outputs
- MPU6050 for IMU sensing
- SSD1306 for local UI display
- ELRS receiver input over UART
- tandem thigh/calf transmission per leg

For the current mechanical and electrical definition, see `docs/hardware-spec.md`.

## Planned Firmware Layers

- board layer: pin map, clocks, startup, board services
- driver layer: I2C, UART, PWM, PCA9685, MPU6050, SSD1306, buttons, buzzer, fan
- device layer: servo abstraction, RC input abstraction, IMU abstraction
- model layer: body geometry, leg geometry, calibration tables, joint mapping
- control layer: kinematics, posture control, gait generation, balance, PID
- app layer: modes, UI, telemetry, debug, fault handling

## Immediate Engineering Priorities

- lock the hardware definition into documentation
- implement a clean board pin map for ESP32-S3
- bring up PCA9685, OLED, and MPU6050 on the assigned buses
- define servo zero offsets and motion limits
- formalize the tandem calf joint mapping
- build a minimal standing and calibration workflow

## Known Uncertainties

- final sign convention for all mirrored joints
- exact joint-angle formulas for the tandem calf mechanism
- final calibration values after assembly and test
