#include "board_defaults.h"

static const scargo_gpio_map_t GPIO_MAP = {
    .pca9685_i2c_port = 0,
    .mpu_oled_i2c_port = 1,
    .pca9685_i2c_address = 0x40,
    .i2c0_sda = 13,
    .i2c0_scl = 12,
    .i2c1_sda = 11,
    .i2c1_scl = 10,
    .uart_com1_rx = 9,
    .uart_com1_tx = 8,
    .uart_com2_rx = 1,
    .uart_com2_tx = 2,
    .fan_pwm = 4,
    .fan_tach = 5,
    .button_k1 = 6,
    .button_k2 = 7,
    .buzzer = 3,
};

static const scargo_mechanics_t MECHANICS = {
    .body_width_mm = 100.0f,
    .body_length_mm = 150.0f,
    .shoulder_length_mm = 40.0f,
    .thigh_length_mm = 100.0f,
    .calf_length_mm = 100.0f,
};

static const scargo_servo_binding_t SERVO_MAP[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG] = {
    {{2, 1}, {1, 1}, {0, -1}},
    {{13, -1}, {14, 1}, {15, 1}},
    {{5, -1}, {6, -1}, {7, 1}},
    {{10, 1}, {9, 1}, {8, 1}},
};

static const scargo_leg_kinematics_binding_t LEG_KINEMATICS[SCARGO_LEG_COUNT] = {
    [SCARGO_LEG_FRONT_RIGHT] = {.shoulder_sign = -1, .knee_coupling_sign = -1, .knee_coupling_offset_deg = 90.0f},
    [SCARGO_LEG_FRONT_LEFT] = {.shoulder_sign = 1, .knee_coupling_sign = 1, .knee_coupling_offset_deg = -90.0f},
    [SCARGO_LEG_REAR_RIGHT] = {.shoulder_sign = -1, .knee_coupling_sign = 1, .knee_coupling_offset_deg = -90.0f},
    [SCARGO_LEG_REAR_LEFT] = {.shoulder_sign = 1, .knee_coupling_sign = -1, .knee_coupling_offset_deg = 90.0f},
};

static const scargo_buzzer_mode_t BUZZER_MODE = SCARGO_BUZZER_MODE_GPIO;

const scargo_gpio_map_t *board_defaults_gpio_map(void)
{
    return &GPIO_MAP;
}

const scargo_mechanics_t *board_defaults_mechanics(void)
{
    return &MECHANICS;
}

const scargo_servo_binding_t (*board_defaults_servo_map(void))[SCARGO_JOINTS_PER_LEG]
{
    return SERVO_MAP;
}

const scargo_leg_kinematics_binding_t *board_defaults_leg_kinematics(void)
{
    return LEG_KINEMATICS;
}

scargo_buzzer_mode_t board_defaults_buzzer_mode(void)
{
    return BUZZER_MODE;
}
