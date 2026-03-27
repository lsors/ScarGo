#include "servo_output.h"

#include "board_defaults.h"
#include "esp_log.h"
#include "pca9685_driver.h"

static const char *TAG = "servo_output";
static calibration_config_t s_calibration;

static float apply_calibration_and_sign(int leg, int joint, float base_angle_deg)
{
    const scargo_servo_binding_t (*servo_map)[SCARGO_JOINTS_PER_LEG] = board_defaults_servo_map();
    const scargo_servo_binding_t *binding = &servo_map[leg][joint];
    float centered_angle = base_angle_deg - 90.0f;
    float signed_angle = centered_angle * (float)binding->servo_sign;
    return 90.0f + signed_angle + (float)s_calibration.servo_offsets_deg[leg][joint];
}

void servo_output_init(const calibration_config_t *calibration)
{
    s_calibration = *calibration;
    if (!pca9685_driver_init()) {
        ESP_LOGE(TAG, "PCA9685 initialization failed");
    }
}

void servo_output_update_calibration(const calibration_config_t *calibration)
{
    s_calibration = *calibration;
}

void servo_output_apply_mid_pose(void)
{
    float mid_pose[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG];
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            mid_pose[leg][joint] = 90.0f;
        }
    }
    servo_output_set_group_angles_deg(mid_pose);
}

void servo_output_set_servo_angle_deg(int leg, int joint, float base_angle_deg)
{
    const scargo_servo_binding_t (*servo_map)[SCARGO_JOINTS_PER_LEG] = board_defaults_servo_map();
    const scargo_servo_binding_t *binding = &servo_map[leg][joint];
    float actual_angle = apply_calibration_and_sign(leg, joint, base_angle_deg);
    if (!pca9685_driver_set_angle_deg(binding->pca9685_channel, actual_angle)) {
        ESP_LOGW(TAG, "Failed to update leg=%d joint=%d channel=%u", leg, joint, binding->pca9685_channel);
    }
}

void servo_output_set_group_angles_deg(const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG])
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            servo_output_set_servo_angle_deg(leg, joint, angles_deg[leg][joint]);
        }
    }
}
