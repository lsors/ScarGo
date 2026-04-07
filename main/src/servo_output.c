#include "servo_output.h"

#include "board_defaults.h"
#include "esp_log.h"
#include "pca9685_driver.h"

static const char *TAG = "servo_output";
static calibration_config_t s_calibration;

/*
 * ====================
 * 第二层：真实执行映射层
 * ====================
 *
 * 这一层专门负责把“控制层给出的基础舵机角”翻译成“最终真正打给舵机的角”。
 *
 * 这里允许出现的东西：
 * - 安装中位 90 度
 * - SERVO_MAP 中的 servo_sign
 * - 标定偏置
 * - 当前真实小腿机构需要的实验性补偿
 *
 * 这里不应该反过来影响：
 * - 纯几何逆解
 * - OLED / Web 预览的几何语义
 *
 * 当前小腿执行修正采用统一的 alpha / beta 语言：
 * - alpha = 大腿舵机相对 90 度中位的偏移
 * - beta  = 通过当前腿的 beta_sign / beta_offset 还原出来的大夹角
 *
 * 然后四条腿统一使用同一条小腿基础公式：
 *   calf = alpha - beta + 90
 *
 * 左右镜像、安装方向差异继续交给：
 * - board_defaults 的 beta_sign / beta_offset
 * - SERVO_MAP 的 servo_sign
 */
static float derive_beta_deg(int leg, float thigh_base_angle_deg, float calf_base_angle_deg)
{
    const scargo_leg_installation_binding_t *leg_bindings = board_defaults_leg_installation();
    return (float)leg_bindings[leg].beta_sign *
           (calf_base_angle_deg - thigh_base_angle_deg - leg_bindings[leg].beta_offset_deg);
}

static float apply_calf_experimental_formula(int leg, float thigh_base_angle_deg, float calf_base_angle_deg)
{
    const float alpha_deg = thigh_base_angle_deg - 90.0f;
    const float beta_deg = derive_beta_deg(leg, thigh_base_angle_deg, calf_base_angle_deg);
    /*
     * 运行态小腿真实执行修正。
     *
     * 当前这版先满足一个已经被日志证明的硬约束：
     * - 安装中位时，输入 thigh=90 / calf=90
     * - 还原后 alpha=0 / beta=90
     * - 最终小腿真实输出必须仍然落在 90 附近，不能被拉到 0/180
     *
     * 因此在原先 alpha-beta+90 的基础上整体平移 +90 度：
     *   calf = alpha - beta + 180
     *
     * 这样中位姿态会得到：
     *   0 - 90 + 180 = 90
     *
     * 这是一条舵机层修正公式，不影响纯几何层和安装层定义。
     */
    return alpha_deg - beta_deg + 180.0f;
}

/*
 * 仅用于调试小腿执行链。
 *
 * 这里把“安装层已经算好的大腿/小腿基础角”，以及经过舵机层小腿修正后的结果一起打印出来，
 * 方便定位问题到底发生在：
 * - 安装层 joint_to_servo()
 * - 还是舵机层 apply_calf_experimental_formula()
 */
void servo_output_log_calf_pipeline(int leg, float thigh_base_angle_deg, float calf_base_angle_deg)
{
    float beta_deg = derive_beta_deg(leg, thigh_base_angle_deg, calf_base_angle_deg);
    float corrected_calf_deg = apply_calf_experimental_formula(leg, thigh_base_angle_deg, calf_base_angle_deg);
    float actual_calf_deg = servo_output_get_actual_angle_deg(leg, SCARGO_JOINT_CALF, corrected_calf_deg);
    ESP_LOGI(TAG,
             "calf pipe leg=%d thigh_base=%.1f calf_input=%.1f beta=%.1f calf_after_formula=%.1f actual=%.1f",
             leg,
             thigh_base_angle_deg,
             calf_base_angle_deg,
             beta_deg,
             corrected_calf_deg,
             actual_calf_deg);
}

/*
 * 把“控制层目标角”转换为“最终舵机实际角”。
 *
 * 这是执行映射层的最后一步：
 * 1. 先把角度换成相对 90 度安装中位的偏移
 * 2. 再乘以 servo_sign 处理舵机安装镜像
 * 3. 最后叠加标定偏置
 *
 * 注意：
 * - 这里的结果只用于真实舵机输出
 * - OLED / Web 预览不应该直接使用这里的值来画腿形
 */
static float apply_calibration_and_sign(int leg, int joint, float base_angle_deg)
{
    const scargo_servo_binding_t (*servo_map)[SCARGO_JOINTS_PER_LEG] = board_defaults_servo_map();
    const scargo_servo_binding_t *binding = &servo_map[leg][joint];
    float centered_angle = base_angle_deg - 90.0f;
    float signed_angle = centered_angle * (float)binding->servo_sign;
    return 90.0f + signed_angle + (float)s_calibration.servo_offsets_deg[leg][joint];
}

float servo_output_get_actual_angle_deg(int leg, int joint, float base_angle_deg)
{
    return apply_calibration_and_sign(leg, joint, base_angle_deg);
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
    float actual_angle = servo_output_get_actual_angle_deg(leg, joint, base_angle_deg);
    if (!pca9685_driver_set_angle_deg(binding->pca9685_channel, actual_angle)) {
        ESP_LOGW(TAG, "Failed to update leg=%d joint=%d channel=%u", leg, joint, binding->pca9685_channel);
    }
}

void servo_output_set_group_angles_deg(const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG])
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            float base_angle_deg = angles_deg[leg][joint];

            if (joint == 2) {
                base_angle_deg = apply_calf_experimental_formula(leg, angles_deg[leg][1], angles_deg[leg][2]);
            }

            servo_output_set_servo_angle_deg(leg, joint, base_angle_deg);
        }
    }
}

void servo_output_compute_actual_group_angles_deg(
    const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG],
    float out_actual_angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG])
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            float base_angle_deg = angles_deg[leg][joint];

            if (joint == 2) {
                base_angle_deg = apply_calf_experimental_formula(leg, angles_deg[leg][1], angles_deg[leg][2]);
            }

            out_actual_angles_deg[leg][joint] = servo_output_get_actual_angle_deg(leg, joint, base_angle_deg);
        }
    }
}

void servo_output_set_actual_group_angles_deg(
    const float actual_angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG])
{
    const scargo_servo_binding_t (*servo_map)[SCARGO_JOINTS_PER_LEG] = board_defaults_servo_map();

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            const scargo_servo_binding_t *binding = &servo_map[leg][joint];
            if (!pca9685_driver_set_angle_deg(binding->pca9685_channel, actual_angles_deg[leg][joint])) {
                ESP_LOGW(TAG, "Failed to update leg=%d joint=%d channel=%u", leg, joint, binding->pca9685_channel);
            }
        }
    }
}

/*
 * 标定模式专用输出。
 *
 * 这里和正常输出链唯一的区别是：
 * - 小腿不再经过 apply_calf_experimental_formula()
 *
 * 原因是标定模式的目标是校准“安装中位”，
 * 此时 90/90/90 就应该对应机械安装位本身，不能再套用运行态的小腿联动修正。
 */
void servo_output_set_calibration_group_angles_deg(const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG])
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            servo_output_set_servo_angle_deg(leg, joint, angles_deg[leg][joint]);
        }
    }
}

void servo_output_compute_calibration_actual_group_angles_deg(
    const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG],
    float out_actual_angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG])
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            out_actual_angles_deg[leg][joint] =
                servo_output_get_actual_angle_deg(leg, joint, angles_deg[leg][joint]);
        }
    }
}
