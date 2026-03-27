#pragma once

#include <stdint.h>

#define SCARGO_SERVO_COUNT 12
#define SCARGO_LEG_COUNT 4
#define SCARGO_JOINTS_PER_LEG 3

#define SCARGO_CONTROL_RATE_HZ 100
#define SCARGO_IMU_RATE_HZ 100
#define SCARGO_SERVO_RATE_HZ 100

#define SCARGO_DEFAULT_STAND_HEIGHT_MM 140.0f
#define SCARGO_MIN_HEIGHT_MM 80.0f
#define SCARGO_MAX_HEIGHT_MM 170.0f

#define SCARGO_DEFAULT_STEP_HEIGHT_MM 40.0f
#define SCARGO_MIN_STEP_HEIGHT_MM 20.0f
#define SCARGO_MAX_STEP_HEIGHT_MM 80.0f

#define SCARGO_DEFAULT_STEP_CYCLE_MS 350
#define SCARGO_MIN_STEP_CYCLE_MS 150
#define SCARGO_MAX_STEP_CYCLE_MS 3000

#define SCARGO_DEFAULT_STEP_SCALE 1.0f
#define SCARGO_MIN_STEP_SCALE 0.5f
#define SCARGO_MAX_STEP_SCALE 1.0f

#define SCARGO_DEFAULT_STANCE_RATIO 0.5f
#define SCARGO_MIN_STANCE_RATIO 0.5f
#define SCARGO_MAX_STANCE_RATIO 0.8f

#define SCARGO_DEFAULT_PHASE_OFFSET 0.0f
#define SCARGO_MAX_BODY_YAW_DEG 45.0f
#define SCARGO_MAX_BODY_ROLL_DEG 20.0f
#define SCARGO_MAX_BODY_PITCH_DEG 20.0f

#define SCARGO_CPU_FREQ_MHZ 240
#define SCARGO_WIFI_AP_SSID "ScarGo-Calib"
#define SCARGO_WIFI_AP_PASSWORD "scargo123"

#define SCARGO_CRSF_UART_BAUD 420000
#define SCARGO_CRSF_MIN_VALUE 172
#define SCARGO_CRSF_MAX_VALUE 1811
#define SCARGO_CRSF_MID_VALUE 992
#define SCARGO_CRSF_SWITCH_LOW 400
#define SCARGO_CRSF_SWITCH_HIGH 1600

#define SCARGO_MPU6050_ADDR 0x68
#define SCARGO_MPU6050_ADDR_ALT 0x69
#define SCARGO_SSD1306_ADDR 0x3C
#define SCARGO_SSD1306_ADDR_ALT 0x3D

#define SCARGO_RC_DEADZONE 0.04f
#define SCARGO_BALANCE_KP_ROLL 0.12f
#define SCARGO_BALANCE_KI_ROLL 0.00f
#define SCARGO_BALANCE_KD_ROLL 0.02f
#define SCARGO_BALANCE_KP_PITCH 0.12f
#define SCARGO_BALANCE_KI_PITCH 0.00f
#define SCARGO_BALANCE_KD_PITCH 0.02f

#define SCARGO_BODY_SIDE_SIGN(leg) (((leg) == SCARGO_LEG_FRONT_LEFT || (leg) == SCARGO_LEG_REAR_LEFT) ? 1.0f : -1.0f)
#define SCARGO_BODY_FRONT_SIGN(leg) (((leg) == SCARGO_LEG_FRONT_LEFT || (leg) == SCARGO_LEG_FRONT_RIGHT) ? 1.0f : -1.0f)

typedef enum {
    SCARGO_LEG_FRONT_RIGHT = 0,
    SCARGO_LEG_FRONT_LEFT = 1,
    SCARGO_LEG_REAR_RIGHT = 2,
    SCARGO_LEG_REAR_LEFT = 3,
} scargo_leg_id_t;

typedef enum {
    SCARGO_JOINT_SHOULDER = 0,
    SCARGO_JOINT_THIGH = 1,
    SCARGO_JOINT_CALF = 2,
} scargo_joint_id_t;

typedef struct {
    uint8_t pca9685_i2c_port;
    uint8_t mpu_oled_i2c_port;
    uint8_t pca9685_i2c_address;
    int i2c0_sda;
    int i2c0_scl;
    int i2c1_sda;
    int i2c1_scl;
    int uart_com1_rx;
    int uart_com1_tx;
    int uart_com2_rx;
    int uart_com2_tx;
    int fan_pwm;
    int fan_tach;
    int button_k1;
    int button_k2;
    int buzzer;
} scargo_gpio_map_t;

typedef struct {
    float body_width_mm;
    float body_length_mm;
    float shoulder_length_mm;
    float thigh_length_mm;
    float calf_length_mm;
} scargo_mechanics_t;

typedef struct {
    uint8_t pca9685_channel;
    int8_t servo_sign;
} scargo_servo_binding_t;

typedef struct {
    int8_t knee_coupling_sign;
    float knee_coupling_offset_deg;
} scargo_leg_kinematics_binding_t;

const scargo_gpio_map_t *board_defaults_gpio_map(void);
const scargo_mechanics_t *board_defaults_mechanics(void);
const scargo_servo_binding_t (*board_defaults_servo_map(void))[SCARGO_JOINTS_PER_LEG];
const scargo_leg_kinematics_binding_t *board_defaults_leg_kinematics(void);
