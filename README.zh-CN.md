# ScarGo

ScarGo 是一个基于 ESP32-S3 的四足机器狗 `ESP-IDF` 固件项目。

## 当前状态

当前仓库包含以下内容：

- 一个可为 ESP32-S3 编译通过的最小 `ESP-IDF` 应用
- 一份初始硬件规格文档
- 一份早期固件架构说明

## 项目基线

- 主控 MCU：ESP32-S3
- 开发框架：ESP-IDF
- 自由度：12
- 舵机控制器：PCA9685
- IMU：MPU6050
- 显示器：SSD1306 OLED
- 遥控输入：通过 UART 接入 ELRS 接收机

## 重要文档

- `docs/index.md`：英文版文档索引
- `docs/index.zh-CN.md`：中文版文档索引
- `docs/hardware-spec.md`：英文版机械与电气规格
- `docs/hardware-spec.zh-CN.md`：中文版机械与电气规格
- `docs/architecture.md`：英文版固件架构方向
- `docs/architecture.zh-CN.md`：中文版固件架构方向

## 编译

本地示例编译流程：

```sh
export IDF_PATH=/Users/mac/esp/v5.4/esp-idf
. "$IDF_PATH/export.sh"
idf.py set-target esp32s3 build
```

## 说明

本项目预计会快速迭代。在把关节公式、校准值和控制逻辑固化进代码之前，应优先把最新理解同步到文档中。
