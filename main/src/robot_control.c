#include "robot_control.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "board_defaults.h"
#include "buzzer_service.h"
#include "esp_log.h"
#include "fan_service.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "kinematics.h"
#include "servo_output.h"

static const char *TAG = "robot";
static const float SCARGO_LIE_HEIGHT_MM = 92.0f;
static const float SCARGO_POSTURE_LIFT_MM = 18.0f;
static const float SCARGO_POSTURE_LIFT_LIE_MM = 10.0f;
/*
 * walk 起步/收步参考相位。
 *
 * 以当前步态分组定义来看：
 * - phase = 0.75 时，0/3 腿位于摆动相顶点
 * - phase = 0.75 时，1/2 腿位于支撑相中点
 *
 * 这正好对应我们希望的“独立起步/收步”目标姿态。
 */
static const float SCARGO_WALK_REFERENCE_PHASE = 0.75f;

typedef enum {
    ROBOT_POSTURE_STAND = 0,
    ROBOT_POSTURE_LIE = 1,
    ROBOT_POSTURE_DIAGONAL_A = 2,  // FR(0)+RL(3) 着地，FL(1)+RR(2) 抬起
    ROBOT_POSTURE_DIAGONAL_B = 3,  // FL(1)+RR(2) 着地，FR(0)+RL(3) 抬起
} robot_posture_t;

typedef enum {
    WALK_PHASE_IDLE = 0,
    WALK_PHASE_START,
    WALK_PHASE_STEADY,
    WALK_PHASE_STOP,
} walk_phase_t;

static system_config_t s_config;
static robot_mode_t s_mode = ROBOT_MODE_STAND;
static robot_action_t s_pending_action = ROBOT_ACTION_NONE;
static float s_target_angles[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG];
static vec3f_t s_default_feet_world[SCARGO_LEG_COUNT];
static vec3f_t s_rest_feet_world[SCARGO_LEG_COUNT];
static vec3f_t s_current_feet_world[SCARGO_LEG_COUNT];
static vec3f_t s_transition_start_feet_world[SCARGO_LEG_COUNT];
static vec3f_t s_transition_target_feet_world[SCARGO_LEG_COUNT];
static body_pose_t s_current_body_pose;
static body_pose_t s_transition_start_pose;
static body_pose_t s_transition_target_pose;
static float s_current_height_mm = SCARGO_DEFAULT_STAND_HEIGHT_MM;
static float s_transition_start_height_mm = SCARGO_DEFAULT_STAND_HEIGHT_MM;
static float s_transition_target_height_mm = SCARGO_DEFAULT_STAND_HEIGHT_MM;
static float s_transition_duration_s;
static float s_transition_progress_s;
static float s_transition_lift_mm;
static bool s_transition_active;
static robot_posture_t s_posture = ROBOT_POSTURE_STAND;
static robot_motion_speed_t s_motion_speed = ROBOT_MOTION_SPEED_MEDIUM;
static bool s_calibration_mode_active;
static bool s_log_calibration_exit_once;
static bool s_log_stand_takeover_once;
static bool s_rc_control_unlocked;

/*
 * 打印四条腿小腿关节在当前输出链下的基角/实际角，专门用于定位标定与运行态切换问题。
 *
 * 这里只看 joint=2（小腿），因为当前最关键的问题就集中在：
 * - 标定稳定态时是否真正停在安装中位附近
 * - 退出标定第一帧时，小腿是否被某条运行态公式重新翻到另一侧
 */
static void log_all_calf_outputs(const char *label,
                                 const float angles_deg[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG],
                                 const calibration_config_t *calibration)
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        float base = angles_deg[leg][SCARGO_JOINT_CALF];
        float actual = servo_output_get_actual_angle_deg(leg, SCARGO_JOINT_CALF, base);
        ESP_LOGI(TAG, "%s leg=%d joint=2 base=%.1f actual=%.1f offset=%d",
                 label,
                 leg,
                 base,
                 actual,
                 calibration->servo_offsets_deg[leg][SCARGO_JOINT_CALF]);
    }
}
static bool s_servo_enabled;
static int16_t s_last_aux_sa;
static int16_t s_last_aux_sc;
static int16_t s_last_aux_sd;
static float s_calibration_start_angles[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG];
static float s_calibration_target_angles[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG];
static float s_calibration_start_actual_angles[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG];
static float s_calibration_target_actual_angles[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG];
static float s_calibration_progress_s;
static float s_calibration_duration_s;
static vec3f_t s_calibration_preview_start_feet_world[SCARGO_LEG_COUNT];
static vec3f_t s_calibration_preview_target_feet_world[SCARGO_LEG_COUNT];
static body_pose_t s_calibration_preview_pose;
static float s_calibration_preview_height_mm;
static walk_phase_t s_walk_phase = WALK_PHASE_IDLE;
static float s_walk_phase_progress;
static float s_walk_steady_phase_origin;
static float s_walk_display_phase;
static bool s_walk_stop_pending;
static vec3f_t s_walk_transition_start_feet_world[SCARGO_LEG_COUNT];
static vec3f_t s_walk_transition_target_feet_world[SCARGO_LEG_COUNT];
static body_pose_t s_walk_transition_start_pose;
static body_pose_t s_walk_transition_target_pose;
static float s_walk_transition_start_height_mm;
static float s_walk_transition_target_height_mm;
static float s_walk_transition_duration_s;
static float s_roll_integral;
static float s_pitch_integral;
static float s_prev_roll_error;
static float s_prev_pitch_error;
static SemaphoreHandle_t s_state_mutex;

static void copy_feet(vec3f_t dst[SCARGO_LEG_COUNT], const vec3f_t src[SCARGO_LEG_COUNT]);
/* 根据当前稳态周期相位，判断某条腿处于摆动相还是支撑相。 */
static robot_walk_leg_state_t walk_leg_state_for_phase(int leg, float phase);
/* 把 walk 全局状态转换成日志/OLED 统一使用的短字符串。 */
static const char *walk_status_name(robot_walk_status_t status);
/* 把单腿状态转换成日志/OLED 统一使用的短字符串。 */
static const char *walk_leg_state_name(robot_walk_leg_state_t state);

static void update_fan_auto_context(bool rc_link_up)
{
    bool walk_mode = s_mode == ROBOT_MODE_WALK;
    float stand_height_range_mm = s_config.gait.stand_height_max_mm - s_config.gait.stand_height_min_mm;
    float fan_off_band_mm = fmaxf(1.0f, stand_height_range_mm * 0.05f);
    bool at_min_height = !walk_mode && s_current_height_mm <= (s_config.gait.stand_height_min_mm + fan_off_band_mm);
    fan_service_set_rc_link(rc_link_up);
    fan_service_set_motion_mode(walk_mode);
    fan_service_set_at_min_height(at_min_height);
}

static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float deg2rad_local(float deg)
{
    return deg * ((float)M_PI / 180.0f);
}

static vec3f_t rotate_x_local(vec3f_t input, float radians)
{
    const float c = cosf(radians);
    const float s = sinf(radians);
    return (vec3f_t){
        .x_mm = input.x_mm,
        .y_mm = input.y_mm * c - input.z_mm * s,
        .z_mm = input.y_mm * s + input.z_mm * c,
    };
}

static vec3f_t rotate_y_local(vec3f_t input, float radians)
{
    const float c = cosf(radians);
    const float s = sinf(radians);
    return (vec3f_t){
        .x_mm = input.x_mm * c + input.z_mm * s,
        .y_mm = input.y_mm,
        .z_mm = -input.x_mm * s + input.z_mm * c,
    };
}

static vec3f_t rotate_z_local(vec3f_t input, float radians)
{
    const float c = cosf(radians);
    const float s = sinf(radians);
    return (vec3f_t){
        .x_mm = input.x_mm * c - input.y_mm * s,
        .y_mm = input.x_mm * s + input.y_mm * c,
        .z_mm = input.z_mm,
    };
}

static float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

static float smoothstepf(float t)
{
    t = clampf_local(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static vec3f_t lerp_vec3(vec3f_t a, vec3f_t b, float t)
{
    return (vec3f_t){
        .x_mm = lerpf(a.x_mm, b.x_mm, t),
        .y_mm = lerpf(a.y_mm, b.y_mm, t),
        .z_mm = lerpf(a.z_mm, b.z_mm, t),
    };
}

// 腿坐标 → 世界坐标（kinematics_apply_body_pose 的完整逆变换）。
//
// 步骤：
//   1. 加髋关节偏移 → 身体中心坐标（机身轴对齐）
//   2. 正向旋转（roll(Y)→pitch(X)→yaw(Z)）→ 身体中心坐标（世界轴对齐）
//   3. 加身体中心在世界中的位置（offset_x/y + stand_height）→ 世界坐标
static vec3f_t body_foot_to_world(scargo_leg_id_t leg, vec3f_t foot_leg, float stand_height_mm, const body_pose_t *pose)
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    const float half_width = mech->body_width_mm * 0.5f;
    const float half_length = mech->body_length_mm * 0.5f;
    const vec3f_t hip_offsets[SCARGO_LEG_COUNT] = {
        [SCARGO_LEG_FRONT_RIGHT] = {.x_mm = -half_width, .y_mm =  half_length, .z_mm = 0.0f},
        [SCARGO_LEG_FRONT_LEFT]  = {.x_mm =  half_width, .y_mm =  half_length, .z_mm = 0.0f},
        [SCARGO_LEG_REAR_RIGHT]  = {.x_mm = -half_width, .y_mm = -half_length, .z_mm = 0.0f},
        [SCARGO_LEG_REAR_LEFT]   = {.x_mm =  half_width, .y_mm = -half_length, .z_mm = 0.0f},
    };

    vec3f_t from_center = {
        .x_mm = foot_leg.x_mm + hip_offsets[leg].x_mm,
        .y_mm = foot_leg.y_mm + hip_offsets[leg].y_mm,
        .z_mm = foot_leg.z_mm + hip_offsets[leg].z_mm,
    };
    vec3f_t p = rotate_y_local(from_center, deg2rad_local(pose->roll_deg));
    p = rotate_x_local(p, deg2rad_local(pose->pitch_deg));
    p = rotate_z_local(p, deg2rad_local(pose->yaw_deg));

    return (vec3f_t){
        .x_mm = p.x_mm + pose->offset_x_mm,
        .y_mm = p.y_mm + pose->offset_y_mm,
        .z_mm = p.z_mm + stand_height_mm,
    };
}

static float feet_distance_mm(vec3f_t a, vec3f_t b)
{
    float dx = b.x_mm - a.x_mm;
    float dy = b.y_mm - a.y_mm;
    float dz = b.z_mm - a.z_mm;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static float motion_speed_mm_s(robot_motion_speed_t speed)
{
    switch (speed) {
    case ROBOT_MOTION_SPEED_SLOW:
        return 80.0f;
    case ROBOT_MOTION_SPEED_FAST:
        return 220.0f;
    case ROBOT_MOTION_SPEED_MEDIUM:
    default:
        return 140.0f;
    }
}

/*
 * walk 起步/收步都定义成“独立于稳态周期”的过渡过程。
 *
 * 用户当前约定：
 * - 起步：从站立脚点平滑走到稳态周期 phase=0.5 时的脚点
 * - 收步：从最近一次 phase=0.5 时的脚点平滑收回站立脚点
 * - 两者时长都取当前周期的四分之一
 */
static float walk_transition_duration_s(float cycle_s)
{
    return clampf_local(cycle_s * 0.25f, 0.10f, 1.20f);
}

static float body_angle_speed_deg_s(robot_motion_speed_t speed)
{
    switch (speed) {
    case ROBOT_MOTION_SPEED_SLOW:
        return 45.0f;
    case ROBOT_MOTION_SPEED_FAST:
        return 120.0f;
    case ROBOT_MOTION_SPEED_MEDIUM:
    default:
        return 75.0f;
    }
}

static float move_towards(float current, float target, float max_delta)
{
    float delta = target - current;
    if (delta > max_delta) {
        return current + max_delta;
    }
    if (delta < -max_delta) {
        return current - max_delta;
    }
    return target;
}

static void begin_calibration_mid_pose_transition(void)
{
    const float speed_deg_s = body_angle_speed_deg_s(s_motion_speed);
    float max_delta_deg = 0.0f;

    s_calibration_mode_active = true;
    s_calibration_progress_s = 0.0f;
    copy_feet(s_calibration_preview_start_feet_world, s_current_feet_world);
    s_calibration_preview_pose = (body_pose_t){0};
    s_calibration_preview_height_mm = s_current_height_mm;

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            s_calibration_start_angles[leg][joint] = s_target_angles[leg][joint];
            s_calibration_target_angles[leg][joint] = 90.0f;
            float delta_deg = fabsf(s_calibration_target_angles[leg][joint] - s_calibration_start_angles[leg][joint]);
            if (delta_deg > max_delta_deg) {
                max_delta_deg = delta_deg;
            }
        }
    }

    /*
     * 标定进入过渡的起点必须使用“当前真实舵机角”，而不是安装层基础角。
     *
     * 否则如果运行态小腿已经经过了执行层修正，直接拿基础角做起点，
     * 真机会先从当前实际角瞬间跳到基础角，再开始往 90 度过渡。
     */
    servo_output_compute_actual_group_angles_deg(s_calibration_start_angles, s_calibration_start_actual_angles);
    servo_output_compute_calibration_actual_group_angles_deg(
        s_calibration_target_angles, s_calibration_target_actual_angles);

    /*
     * 标定预览目标不从“90/90/90 舵机角”反推，
     * 而是直接使用我们定义的中位几何姿态：
     * - shoulder = 0
     * - alpha    = 0   (大腿垂直向下)
     * - beta     = 90  (小腿相对大腿垂直向前)
     *
     * 这样 OLED 看到的是用户定义的安装中位语义，而不是执行层安装约束后的反推结果。
     */
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        leg_joint_pose_t mid_joint_pose = {
            .shoulder_deg = 0.0f,
            .thigh_deg = 0.0f,
            .beta_deg = 90.0f,
        };
        vec3f_t chain_points[4];
        if (!kinematics_compute_leg_chain_from_installation_pose((scargo_leg_id_t)leg, &mid_joint_pose, chain_points)) {
            s_calibration_preview_target_feet_world[leg] = s_current_feet_world[leg];
            continue;
        }
        s_calibration_preview_target_feet_world[leg] =
            body_foot_to_world((scargo_leg_id_t)leg, chain_points[3], s_calibration_preview_height_mm, &s_calibration_preview_pose);
    }

    s_calibration_duration_s = clampf_local(max_delta_deg / fmaxf(speed_deg_s, 1.0f), 0.20f, 1.20f);
}

static bool update_calibration_transition(float dt_s)
{
    if (!s_calibration_mode_active) {
        return false;
    }

    s_calibration_progress_s += dt_s;
    float progress = s_calibration_progress_s / fmaxf(s_calibration_duration_s, 0.01f);
    if (progress >= 1.0f) {
        /*
         * 标定进入过渡完成后，必须把控制权交给下面的“稳定标定态”分支。
         *
         * 如果这里继续返回 true，control_tick() 会永远停在过渡链里，
         * Web 每次修改偏置时虽然数据已经更新，但稳定标定输出链永远不会被执行，
         * 于是看起来就像“改偏置没有任何反应”。
         */
        progress = 1.0f;
        for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
            s_current_feet_world[leg] = s_calibration_preview_target_feet_world[leg];
        }
        s_current_body_pose = s_calibration_preview_pose;
        s_current_height_mm = s_calibration_preview_height_mm;
        return false;
    }
    float eased = smoothstepf(progress);

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            s_target_angles[leg][joint] = lerpf(
                s_calibration_start_actual_angles[leg][joint],
                s_calibration_target_actual_angles[leg][joint],
                eased);
        }
        s_current_feet_world[leg] = lerp_vec3(
            s_calibration_preview_start_feet_world[leg],
            s_calibration_preview_target_feet_world[leg],
            eased);
    }

    s_current_body_pose = s_calibration_preview_pose;
    s_current_height_mm = s_calibration_preview_height_mm;
    /*
     * 标定进入过程虽然仍然使用角度插值，但真实输出必须切到“标定专用输出链”。
     * 这样小腿在逼近 90/90/90 时，不会被运行态的小腿修正公式提前拉到 0/180 一侧。
     */
    servo_output_set_actual_group_angles_deg(s_target_angles);
    return true;
}

static float gait_cycle_effective_ms(const rc_command_t *command)
{
    // 启动时 aux_sb 默认是 0，此时自然落到 Web 配置的中档值。
    // 遥控连上后，SB 负责在 min/mid/max 三档中切换。
    // 遥控临时断开时，不立即回退到配置，而是保持最后一次收到的档位。
    if (command == NULL) {
        return s_config.gait.step_cycle_ms;
    }
    if (command->aux_sb < 0) {
        return s_config.gait.step_cycle_min_ms;
    }
    if (command->aux_sb > 0) {
        return s_config.gait.step_cycle_max_ms;
    }
    return s_config.gait.step_cycle_ms;
}

/*
 * 站立/行走两种模式下，油门统一表示“身体高度”。
 *
 * 归一化规则：
 * - 遥控器低位(-1) -> 最低站立高度
 * - 遥控器高位(+1) -> 最高站立高度
 *
 * 这样现在：
 * - stand 模式：油门控制站高
 * - walk  模式：油门也控制行进高度
 */
static float body_height_ratio_from_throttle(const rc_command_t *command)
{
    if (command == NULL) {
        return 0.5f;
    }
    return clampf_local((command->throttle + 1.0f) * 0.5f, 0.0f, 1.0f);
}

/*
 * 身体高度由油门统一控制：
 * - 低油门 -> 低站高
 * - 高油门 -> 高站高
 */
static float body_height_from_throttle(const rc_command_t *command)
{
    float ratio = body_height_ratio_from_throttle(command);
    return lerpf(s_config.gait.stand_height_min_mm, s_config.gait.stand_height_max_mm, ratio);
}

/*
 * 抬腿高度与行进站高同步变化。
 *
 * 这样做的目的：
 * - 站得低时，抬腿也低，避免在低姿态下腿部折叠过大造成机械干涉
 * - 站得高时，抬腿同步增大，保证步态仍有足够离地间隙
 *
 * 这里直接复用 gait 配置里的最小/最大抬腿高度，让联动关系简单、连续、可控。
 */
static float walk_step_height_from_throttle(const rc_command_t *command)
{
    float ratio = body_height_ratio_from_throttle(command);
    return lerpf(s_config.gait.step_height_min_mm, s_config.gait.step_height_max_mm, ratio);
}

/* 站立模式最大 yaw 角随高度线性缩放：
 * 低姿态时腿的展开角小，yaw 过大会让腿互相干涉导致堵转；
 * 高姿态时腿有足够展开空间，允许更大偏航。 */
static float dynamic_max_yaw_deg(const rc_command_t *command)
{
    float ratio = body_height_ratio_from_throttle(command);
    return lerpf(SCARGO_MAX_BODY_YAW_DEG_LOW, s_config.gait.max_body_yaw_deg, ratio);
}

static body_pose_t move_pose_towards(body_pose_t current, body_pose_t target, float max_delta_deg)
{
    current.roll_deg = move_towards(current.roll_deg, target.roll_deg, max_delta_deg);
    current.pitch_deg = move_towards(current.pitch_deg, target.pitch_deg, max_delta_deg);
    current.yaw_deg = move_towards(current.yaw_deg, target.yaw_deg, max_delta_deg);
    return current;
}

static void copy_feet(vec3f_t dst[SCARGO_LEG_COUNT], const vec3f_t src[SCARGO_LEG_COUNT])
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        dst[leg] = src[leg];
    }
}

// 开始一次“姿态/高度/脚点”的统一平滑过渡。
//
// 这是当前项目里所有模式切换的总入口：
// - stand -> lie
// - lie -> stand
// - walk -> stand
// - 进入标定安装位
//
// 这样做的原因是：任何模式切换都不允许瞬时跳变，而是必须在每个控制周期里
// 重新计算一个中间时刻的目标足端和机身姿态，保证机械动作连续。
static void begin_posture_transition(robot_posture_t posture, float target_height_mm,
                                     const vec3f_t target_feet_world[SCARGO_LEG_COUNT],
                                     const body_pose_t *target_pose, float lift_mm)
{
    float max_distance_mm = fabsf(target_height_mm - s_current_height_mm);
    float speed_mm_s = motion_speed_mm_s(s_motion_speed);

    s_posture = posture;
    s_transition_active = true;
    s_transition_progress_s = 0.0f;
    s_transition_start_height_mm = s_current_height_mm;
    s_transition_target_height_mm = target_height_mm;
    s_transition_start_pose = s_current_body_pose;
    s_transition_target_pose = *target_pose;
    s_transition_lift_mm = lift_mm;
    copy_feet(s_transition_start_feet_world, s_current_feet_world);
    copy_feet(s_transition_target_feet_world, target_feet_world);

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        float distance_mm = feet_distance_mm(s_transition_start_feet_world[leg], s_transition_target_feet_world[leg]);
        if (distance_mm > max_distance_mm) {
            max_distance_mm = distance_mm;
        }
    }

    s_transition_duration_s = clampf_local(max_distance_mm / fmaxf(speed_mm_s, 1.0f), 0.35f, 1.80f);
}

static void update_transition(float dt_s)
{
    if (!s_transition_active) {
        return;
    }

    s_transition_progress_s += dt_s;
    float progress = s_transition_progress_s / fmaxf(s_transition_duration_s, 0.01f);
    float eased = smoothstepf(progress);

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        vec3f_t foot = lerp_vec3(s_transition_start_feet_world[leg], s_transition_target_feet_world[leg], eased);
        foot.z_mm += s_transition_lift_mm * sinf((float)M_PI * eased);
        s_current_feet_world[leg] = foot;
    }

    s_current_height_mm = lerpf(s_transition_start_height_mm, s_transition_target_height_mm, eased);
    s_current_body_pose.roll_deg = lerpf(s_transition_start_pose.roll_deg, s_transition_target_pose.roll_deg, eased);
    s_current_body_pose.pitch_deg = lerpf(s_transition_start_pose.pitch_deg, s_transition_target_pose.pitch_deg, eased);
    s_current_body_pose.yaw_deg = lerpf(s_transition_start_pose.yaw_deg, s_transition_target_pose.yaw_deg, eased);

    if (progress >= 1.0f) {
        s_transition_active = false;
        s_current_height_mm = s_transition_target_height_mm;
        s_current_body_pose = s_transition_target_pose;
        copy_feet(s_current_feet_world, s_transition_target_feet_world);
    }
}

// 遥控输入 -> 机身目标姿态。
//
// 注意这里分了 stand / walk 两种语义：
// - stand : 摇杆主要控制机身姿态和站高
// - walk  : 摇杆主要控制步态方向和步长，不直接在这里给姿态角
static robot_target_pose_t make_target_pose(const rc_command_t *command, robot_mode_t mode)
{
    robot_target_pose_t target = {
        .roll_deg = 0.0f,
        .pitch_deg = 0.0f,
        .yaw_deg = 0.0f,
        .height_mm = s_config.gait.stand_height_default_mm,
    };

    if (mode == ROBOT_MODE_STAND) {
        target.height_mm = body_height_from_throttle(command);
        target.yaw_deg = command->yaw * dynamic_max_yaw_deg(command);
        // 站立模式下取消 SB 对姿态控制的分支影响。
        // 右摇杆映射：
        // - yaw   -> 左摇杆左右：机身绕 z 轴旋转（±45°）
        // - roll  -> 右摇杆左右：机身横滚倾斜
        // - pitch -> 右摇杆上下：机身俯仰倾斜
        target.roll_deg = -command->roll * s_config.gait.max_body_roll_deg;
        target.pitch_deg = -command->pitch * s_config.gait.max_body_pitch_deg;
        return target;
    }

    target.height_mm = body_height_from_throttle(command);
    target.yaw_deg = 0.0f;
    target.pitch_deg = 0.0f;
    target.roll_deg = 0.0f;
    return target;
}

static void fill_mid_pose(float target[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG])
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        target[leg][SCARGO_JOINT_SHOULDER] = 90.0f;
        target[leg][SCARGO_JOINT_THIGH] = 90.0f;
        target[leg][SCARGO_JOINT_CALF] = 90.0f;
    }
}

static void log_mid_pose_forward_kinematics(void)
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        leg_joint_pose_t mid_joint_pose = {
            .shoulder_deg = 0.0f,
            .thigh_deg = 0.0f,
            .beta_deg = 90.0f,
        };
        vec3f_t chain_points[4];
        if (kinematics_compute_leg_chain_from_installation_pose((scargo_leg_id_t)leg, &mid_joint_pose, chain_points)) {
            vec3f_t foot_body = chain_points[3];
            ESP_LOGI(TAG,
                     "mid FK leg=%d foot_body=(x=%.1f,y=%.1f,z=%.1f)",
                     leg,
                     foot_body.x_mm,
                     foot_body.y_mm,
                     foot_body.z_mm);
        }
    }
}

/*
 * 把“世界坐标中的目标脚点 + 机身高度/姿态”真正解成 12 路舵机角。
 *
 * 这是控制层进入三层结构的总入口：
 *
 *   足端目标（世界坐标）
 *   -> feet_body（机身局部）
 *   -> 纯几何逆解（shoulder / alpha / beta）
 *   -> 执行映射（joint_to_servo）
 *   -> s_target_angles
 *   -> servo_output
 *
 * 因此：
 * - 这里是“几何层 -> 执行层”的边界
 * - 显示层不应该倒着从这里反推几何语义
 */
static void solve_and_apply_feet(const vec3f_t feet_world[SCARGO_LEG_COUNT], float height_mm, const body_pose_t *pose)
{
    vec3f_t feet_body[SCARGO_LEG_COUNT];
    kinematics_apply_body_pose(feet_body, feet_world, height_mm, pose);

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        leg_joint_pose_t joint_pose;
        leg_servo_pose_t pose_solution;
        if (!kinematics_solve_leg_geometry((scargo_leg_id_t)leg, &feet_body[leg], &joint_pose) ||
            !kinematics_joint_to_servo((scargo_leg_id_t)leg, &joint_pose, &pose_solution)) {
            continue;
        }
        s_target_angles[leg][SCARGO_JOINT_SHOULDER] = pose_solution.shoulder_deg;
        s_target_angles[leg][SCARGO_JOINT_THIGH] = pose_solution.thigh_servo_deg;
        s_target_angles[leg][SCARGO_JOINT_CALF] = pose_solution.calf_servo_deg;

        if (s_log_stand_takeover_once) {
            servo_output_log_calf_pipeline(leg, pose_solution.thigh_servo_deg, pose_solution.calf_servo_deg);
        }
    }

    if (s_log_stand_takeover_once) {
        s_log_stand_takeover_once = false;
    }
}

static void apply_balance_impl(body_pose_t *pose, const attitude_state_t *attitude,
                               float kp_roll, float ki_roll, float kd_roll,
                               float kp_pitch, float ki_pitch, float kd_pitch)
{
    if (!s_config.imu.balance_enabled || !attitude->ready) {
        return;
    }

    const float dt = 1.0f / (float)SCARGO_CONTROL_RATE_HZ;
    float roll_error = pose->roll_deg - attitude->roll_deg;
    float pitch_error = pose->pitch_deg - attitude->pitch_deg;

    s_roll_integral += roll_error * dt;
    s_pitch_integral += pitch_error * dt;

    float roll_derivative = (roll_error - s_prev_roll_error) / dt;
    float pitch_derivative = (pitch_error - s_prev_pitch_error) / dt;

    pose->roll_deg += roll_error * kp_roll + s_roll_integral * ki_roll + roll_derivative * kd_roll;
    pose->pitch_deg += pitch_error * kp_pitch + s_pitch_integral * ki_pitch + pitch_derivative * kd_pitch;

    s_prev_roll_error = roll_error;
    s_prev_pitch_error = pitch_error;
}

static void apply_balance(body_pose_t *pose, const attitude_state_t *attitude)
{
    apply_balance_impl(pose, attitude,
                       SCARGO_BALANCE_KP_ROLL, SCARGO_BALANCE_KI_ROLL, SCARGO_BALANCE_KD_ROLL,
                       SCARGO_BALANCE_KP_PITCH, SCARGO_BALANCE_KI_PITCH, SCARGO_BALANCE_KD_PITCH);
}

static void apply_balance_diagonal(body_pose_t *pose, const attitude_state_t *attitude)
{
    apply_balance_impl(pose, attitude,
                       SCARGO_DIAG_BALANCE_KP_ROLL, SCARGO_DIAG_BALANCE_KI_ROLL, SCARGO_DIAG_BALANCE_KD_ROLL,
                       SCARGO_DIAG_BALANCE_KP_PITCH, SCARGO_DIAG_BALANCE_KI_PITCH, SCARGO_DIAG_BALANCE_KD_PITCH);
}

static const bool s_diagonal_pair_a_support[SCARGO_LEG_COUNT] = {
    [SCARGO_LEG_FRONT_RIGHT] = true,
    [SCARGO_LEG_FRONT_LEFT]  = false,
    [SCARGO_LEG_REAR_RIGHT]  = false,
    [SCARGO_LEG_REAR_LEFT]   = true,
};
static const bool s_diagonal_pair_b_support[SCARGO_LEG_COUNT] = {
    [SCARGO_LEG_FRONT_RIGHT] = false,
    [SCARGO_LEG_FRONT_LEFT]  = true,
    [SCARGO_LEG_REAR_RIGHT]  = true,
    [SCARGO_LEG_REAR_LEFT]   = false,
};

static void begin_diagonal_stand_transition(robot_posture_t posture)
{
    const bool *support = (posture == ROBOT_POSTURE_DIAGONAL_A) ? s_diagonal_pair_a_support : s_diagonal_pair_b_support;
    float step_height = s_config.gait.step_height_mm;
    vec3f_t target_feet[SCARGO_LEG_COUNT];

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        target_feet[leg] = s_default_feet_world[leg];
        if (!support[leg]) {
            target_feet[leg].z_mm = step_height;
        }
    }
    s_roll_integral = 0.0f;
    s_pitch_integral = 0.0f;
    s_prev_roll_error = 0.0f;
    s_prev_pitch_error = 0.0f;
    begin_posture_transition(posture, s_current_height_mm, target_feet, &(body_pose_t){0}, 0.0f);
}

// 对角腿站立模式控制。
//
// 着地腿：固定在默认世界坐标脚点（z=0）
// 抬起腿：保持在步态抬腿高度（z=step_height_mm）
// 姿态：不跟随遥控摇杆，完全由 IMU PID 管控
// 高度：由油门控制
static void apply_diagonal_stand_pose(const rc_command_t *command, const attitude_state_t *attitude)
{
    const float dt_s = 1.0f / (float)SCARGO_CONTROL_RATE_HZ;
    const float height_speed_mm_s = motion_speed_mm_s(s_motion_speed);
    const bool *support = (s_posture == ROBOT_POSTURE_DIAGONAL_A) ? s_diagonal_pair_a_support : s_diagonal_pair_b_support;
    float step_height = s_config.gait.step_height_mm;
    float target_height = body_height_from_throttle(command);

    fill_mid_pose(s_target_angles);
    s_current_height_mm = move_towards(s_current_height_mm, target_height, height_speed_mm_s * dt_s);

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        if (support[leg]) {
            s_current_feet_world[leg].x_mm = s_default_feet_world[leg].x_mm;
            s_current_feet_world[leg].y_mm = s_default_feet_world[leg].y_mm;
            s_current_feet_world[leg].z_mm = 0.0f;
        } else {
            s_current_feet_world[leg].x_mm = s_default_feet_world[leg].x_mm;
            s_current_feet_world[leg].y_mm = s_default_feet_world[leg].y_mm;
            s_current_feet_world[leg].z_mm = step_height;
        }
    }

    body_pose_t solved_pose = s_current_body_pose;
    apply_balance_diagonal(&solved_pose, attitude);
    solve_and_apply_feet(s_current_feet_world, s_current_height_mm, &solved_pose);
}

// 站立模式控制。
//
// 站立模式的核心约束是：
// - 四个足端相对地面固定
// - 机身高度、姿态、平移由遥控输入缓慢逼近目标
//
// 当前实现里，站立模式完全不再使用 SB。
// - 油门      ：站高
// - yaw      ：机身绕 z 轴
// - pitch    ：机身绕 x 轴
// - roll     ：机身绕 y 轴
static void apply_stand_pose(const rc_command_t *command, const attitude_state_t *attitude)
{
    const float dt_s = 1.0f / (float)SCARGO_CONTROL_RATE_HZ;
    const float height_speed_mm_s = motion_speed_mm_s(s_motion_speed);
    const float body_speed_deg_s = body_angle_speed_deg_s(s_motion_speed);
    robot_target_pose_t target = make_target_pose(command, ROBOT_MODE_STAND);
    body_pose_t pose_target = {
        .yaw_deg = target.yaw_deg,
        .pitch_deg = target.pitch_deg,
        .roll_deg = target.roll_deg,
    };
    body_pose_t solved_pose;
    vec3f_t feet_target[SCARGO_LEG_COUNT];

    fill_mid_pose(s_target_angles);
    copy_feet(feet_target, s_default_feet_world);

    if (s_posture == ROBOT_POSTURE_LIE && !s_transition_active) {
        pose_target = (body_pose_t){0};
        target.height_mm = SCARGO_LIE_HEIGHT_MM;
        copy_feet(s_current_feet_world, s_rest_feet_world);
    } else if (!s_transition_active) {
        s_current_height_mm = move_towards(s_current_height_mm, target.height_mm, height_speed_mm_s * dt_s);
        s_current_body_pose = move_pose_towards(s_current_body_pose, pose_target, body_speed_deg_s * dt_s);
        for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
            s_current_feet_world[leg].x_mm = move_towards(s_current_feet_world[leg].x_mm, feet_target[leg].x_mm, height_speed_mm_s * dt_s);
            s_current_feet_world[leg].y_mm = move_towards(s_current_feet_world[leg].y_mm, feet_target[leg].y_mm, height_speed_mm_s * dt_s);
            s_current_feet_world[leg].z_mm = feet_target[leg].z_mm;
        }
    }

    solved_pose = s_current_body_pose;
    apply_balance(&solved_pose, attitude);
    solve_and_apply_feet(s_current_feet_world, s_current_height_mm, &solved_pose);
}

static void apply_lie_hold_pose(void)
{
    fill_mid_pose(s_target_angles);
    solve_and_apply_feet(s_current_feet_world, s_current_height_mm, &s_current_body_pose);
}

// 摆线方程的相位函数。
// 用来生成摆动相时较平滑的足端轨迹。
static float cycloid_phase(float phase)
{
    return phase - sinf(2.0f * (float)M_PI * phase) / (2.0f * (float)M_PI);
}

// 计算某条腿在主相位 phase 下的组相位（归一化到 [0, 1)）。
//
// 对角腿分组：
//   - FRONT_RIGHT / REAR_LEFT 为主相位组（偏移量 0）
//   - FRONT_LEFT  / REAR_RIGHT 滞后半个周期（偏移量 0.5）
// diagonal_phase_offset 仅作用于右侧腿，用于微调左右组的相对相位差。
static float leg_group_phase(int leg, float phase)
{
    float group_phase = phase + ((leg == SCARGO_LEG_FRONT_RIGHT || leg == SCARGO_LEG_REAR_LEFT) ? 0.0f : 0.5f);
    if (leg == SCARGO_LEG_REAR_RIGHT || leg == SCARGO_LEG_FRONT_RIGHT) {
        group_phase += s_config.gait.diagonal_phase_offset;
    }
    group_phase = fmodf(group_phase, 1.0f);
    if (group_phase < 0.0f) {
        group_phase += 1.0f;
    }
    return group_phase;
}

// 按某一时刻的步态相位，生成四条腿的目标足端。
//
// 当前 walk 模式的语义：
// - 支撑相：足端在 x/y 平面内做直线移动
// - 摆动相：足端在 x/y 方向随相位推进，同时 z 方向按摆线抬脚
//
// 遥控映射：
// - pitch : y 方向步长
// - roll  : x 方向步长
// - yaw   : 左右两侧在 y 上做差分，用于转向
static void apply_walk_feet_for_phase(const rc_command_t *command, const attitude_state_t *attitude,
                                      float phase, float stride, float stand_height, float step_height, body_pose_t *pose,
                                      vec3f_t feet_world[SCARGO_LEG_COUNT])
{
    kinematics_default_feet(feet_world, stand_height);
    const float step_vec_y = clampf_local(command->pitch, -1.0f, 1.0f) * stride;
    const float step_vec_x = -clampf_local(command->roll, -1.0f, 1.0f) * stride;
    const float yaw_diff_y = clampf_local(command->yaw, -1.0f, 1.0f) * stride * 0.5f;

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        float group_phase = leg_group_phase(leg, phase);
        bool in_swing = group_phase >= s_config.gait.stance_ratio;
        float phase_local;
        float side_sign = SCARGO_BODY_SIDE_SIGN(leg);
        float leg_step_y = step_vec_y + side_sign * yaw_diff_y;
        float leg_step_x = step_vec_x;

        if (in_swing) {
            phase_local = (group_phase - s_config.gait.stance_ratio) / fmaxf(1.0f - s_config.gait.stance_ratio, 0.01f);
            float cycloid = cycloid_phase(phase_local);
            feet_world[leg].y_mm += (-0.5f + cycloid) * leg_step_y;
            feet_world[leg].x_mm += (-0.5f + cycloid) * leg_step_x;
            feet_world[leg].z_mm += step_height * sinf((float)M_PI * phase_local);
        } else {
            phase_local = group_phase / fmaxf(s_config.gait.stance_ratio, 0.01f);
            feet_world[leg].y_mm += (0.5f - phase_local) * leg_step_y;
            feet_world[leg].x_mm += (0.5f - phase_local) * leg_step_x;
        }
    }
    apply_balance(pose, attitude);
}

static void begin_walk_start_transition(const rc_command_t *command, const attitude_state_t *attitude, float cycle_s)
{
    robot_target_pose_t target = make_target_pose(command, ROBOT_MODE_WALK);
    body_pose_t target_pose = {
        .yaw_deg = target.yaw_deg,
        .pitch_deg = target.pitch_deg,
        .roll_deg = target.roll_deg,
    };
    vec3f_t target_feet_world[SCARGO_LEG_COUNT];
    float step_height = walk_step_height_from_throttle(command);
    float stride = step_height * s_config.gait.step_scale;

    copy_feet(s_walk_transition_start_feet_world, s_current_feet_world);
    s_walk_transition_start_pose = s_current_body_pose;
    s_walk_transition_start_height_mm = s_current_height_mm;

    /*
     * 起步目标固定在参考相位：
     * - 0/3 腿处于摆动顶点
     * - 1/2 腿处于支撑中点
     *
     * 这样起步才会是一个真正可见的“抬脚进入周期”的独立过程。
     */
    apply_walk_feet_for_phase(command, attitude, SCARGO_WALK_REFERENCE_PHASE, stride,
                              target.height_mm, step_height, &target_pose, target_feet_world);

    copy_feet(s_walk_transition_target_feet_world, target_feet_world);
    s_walk_transition_target_pose = target_pose;
    s_walk_transition_target_height_mm = target.height_mm;
    s_walk_transition_duration_s = walk_transition_duration_s(cycle_s);
    s_walk_phase_progress = 0.0f;
    s_walk_phase = WALK_PHASE_START;
    s_walk_steady_phase_origin = SCARGO_WALK_REFERENCE_PHASE;
    s_walk_stop_pending = false;
}

static void begin_walk_stop_transition(const rc_command_t *command, const vec3f_t current_feet_world[SCARGO_LEG_COUNT],
                                       float current_height_mm, const body_pose_t *current_pose, float cycle_s)
{
    robot_target_pose_t target = make_target_pose(command, ROBOT_MODE_STAND);

    copy_feet(s_walk_transition_start_feet_world, current_feet_world);
    s_walk_transition_start_pose = *current_pose;
    s_walk_transition_start_height_mm = current_height_mm;
    copy_feet(s_walk_transition_target_feet_world, s_default_feet_world);
    s_walk_transition_target_pose = (body_pose_t){
        .yaw_deg = target.yaw_deg,
        .pitch_deg = target.pitch_deg,
        .roll_deg = target.roll_deg,
    };
    s_walk_transition_target_height_mm = target.height_mm;
    s_walk_transition_duration_s = walk_transition_duration_s(cycle_s);
    s_walk_phase_progress = 0.0f;
    s_walk_phase = WALK_PHASE_STOP;
    s_walk_stop_pending = false;
}

static bool phase_crossed_target(float previous_phase, float current_phase, float target_phase)
{
    if (current_phase >= previous_phase) {
        return previous_phase < target_phase && current_phase >= target_phase;
    }
    return previous_phase < target_phase || current_phase >= target_phase;
}

// 行走模式控制。
//
// 目前分成 3 段：
// 1. WALK_START  : 起步过渡
// 2. WALK_STEADY : 稳态周期行走
// 3. WALK_STOP   : 预留给优雅收步，当前主要通过切回统一姿态过渡实现
static void apply_walk_pose(const rc_command_t *command, const attitude_state_t *attitude)
{
    static uint32_t steady_tick;
    static float previous_steady_phase = SCARGO_WALK_REFERENCE_PHASE;
    const float dt_s = 1.0f / (float)SCARGO_CONTROL_RATE_HZ;
    float step_height = walk_step_height_from_throttle(command);
    float stride = step_height * s_config.gait.step_scale;
    float cycle = fmaxf(gait_cycle_effective_ms(command) / 1000.0f, 0.15f);
    robot_target_pose_t target = make_target_pose(command, ROBOT_MODE_WALK);
    float stand_height = s_transition_active ? s_current_height_mm : target.height_mm;
    body_pose_t pose = {
        .yaw_deg = target.yaw_deg,
        .pitch_deg = target.pitch_deg,
        .roll_deg = target.roll_deg,
    };
    vec3f_t feet_world[SCARGO_LEG_COUNT];
    float base_phase = 0.0f;
    float gait_step_height = step_height;
    bool feet_already_resolved = false;

    fill_mid_pose(s_target_angles);

    switch (s_walk_phase) {
    case WALK_PHASE_START:
    {
        s_walk_phase_progress += dt_s / fmaxf(s_walk_transition_duration_s, 0.01f);
        float eased = smoothstepf(s_walk_phase_progress);
        copy_feet(feet_world, s_walk_transition_start_feet_world);
        for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
            feet_world[leg] = lerp_vec3(s_walk_transition_start_feet_world[leg],
                                        s_walk_transition_target_feet_world[leg],
                                        eased);
        }
        pose.roll_deg = lerpf(s_walk_transition_start_pose.roll_deg, s_walk_transition_target_pose.roll_deg, eased);
        pose.pitch_deg = lerpf(s_walk_transition_start_pose.pitch_deg, s_walk_transition_target_pose.pitch_deg, eased);
        pose.yaw_deg = lerpf(s_walk_transition_start_pose.yaw_deg, s_walk_transition_target_pose.yaw_deg, eased);
        stand_height = lerpf(s_walk_transition_start_height_mm, s_walk_transition_target_height_mm, eased);
        s_walk_display_phase = SCARGO_WALK_REFERENCE_PHASE * eased;
        feet_already_resolved = true;
        if (s_walk_phase_progress >= 1.0f) {
            if (s_walk_stop_pending) {
                begin_walk_stop_transition(command,
                                           s_walk_transition_target_feet_world,
                                           s_walk_transition_target_height_mm,
                                           &s_walk_transition_target_pose,
                                           cycle);
                copy_feet(feet_world, s_walk_transition_start_feet_world);
                pose = s_walk_transition_start_pose;
                stand_height = s_walk_transition_start_height_mm;
            } else {
                s_walk_phase = WALK_PHASE_STEADY;
                s_walk_phase_progress = 0.0f;
                steady_tick = 0;
                s_walk_steady_phase_origin = SCARGO_WALK_REFERENCE_PHASE;
                s_walk_display_phase = s_walk_steady_phase_origin;
                previous_steady_phase = s_walk_steady_phase_origin;
            }
        }
        break;
    }
    case WALK_PHASE_STOP:
    {
        s_walk_phase_progress += dt_s / fmaxf(s_walk_transition_duration_s, 0.01f);
        float eased = smoothstepf(s_walk_phase_progress);
        copy_feet(feet_world, s_walk_transition_start_feet_world);
        for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
            feet_world[leg] = lerp_vec3(s_walk_transition_start_feet_world[leg],
                                        s_walk_transition_target_feet_world[leg],
                                        eased);
        }
        pose.roll_deg = lerpf(s_walk_transition_start_pose.roll_deg, s_walk_transition_target_pose.roll_deg, eased);
        pose.pitch_deg = lerpf(s_walk_transition_start_pose.pitch_deg, s_walk_transition_target_pose.pitch_deg, eased);
        pose.yaw_deg = lerpf(s_walk_transition_start_pose.yaw_deg, s_walk_transition_target_pose.yaw_deg, eased);
        stand_height = lerpf(s_walk_transition_start_height_mm, s_walk_transition_target_height_mm, eased);
        s_walk_display_phase = SCARGO_WALK_REFERENCE_PHASE;
        feet_already_resolved = true;
        if (s_walk_phase_progress >= 1.0f) {
            s_walk_phase = WALK_PHASE_IDLE;
            s_walk_phase_progress = 0.0f;
            s_mode = ROBOT_MODE_STAND;
            copy_feet(s_current_feet_world, s_walk_transition_target_feet_world);
            s_current_body_pose = s_walk_transition_target_pose;
            s_current_height_mm = s_walk_transition_target_height_mm;
        }
        break;
    }
    case WALK_PHASE_STEADY:
        steady_tick++;
        base_phase = fmodf(s_walk_steady_phase_origin + (((float)(steady_tick) * dt_s) / cycle), 1.0f);
        s_walk_display_phase = base_phase;
        if (s_walk_stop_pending && phase_crossed_target(previous_steady_phase, base_phase, SCARGO_WALK_REFERENCE_PHASE)) {
            apply_walk_feet_for_phase(command, attitude, base_phase, stride, stand_height, gait_step_height, &pose, feet_world);
            begin_walk_stop_transition(command, feet_world, stand_height, &pose, cycle);
            previous_steady_phase = base_phase;
            feet_already_resolved = true;
            break;
        }
        previous_steady_phase = base_phase;
        break;
    case WALK_PHASE_IDLE:
    default:
        base_phase = 0.0f;
        s_walk_display_phase = 0.0f;
        break;
    }

    if (!feet_already_resolved) {
        apply_walk_feet_for_phase(command, attitude, base_phase, stride, stand_height, gait_step_height, &pose, feet_world);
    }
    copy_feet(s_current_feet_world, feet_world);
    s_current_height_mm = stand_height;
    s_current_body_pose = pose;
    solve_and_apply_feet(feet_world, stand_height, &pose);
}

void robot_control_init(const system_config_t *config)
{
    s_state_mutex = xSemaphoreCreateRecursiveMutex();
    vec3f_t startup_feet_world[SCARGO_LEG_COUNT];

    s_config = *config;
    kinematics_default_feet(s_default_feet_world, config->gait.stand_height_default_mm);
    kinematics_rest_feet(s_rest_feet_world, config->gait.stand_height_default_mm);
    copy_feet(s_current_feet_world, s_rest_feet_world);
    s_current_height_mm = SCARGO_LIE_HEIGHT_MM;
    s_current_body_pose = (body_pose_t){0};
    s_posture = ROBOT_POSTURE_LIE;
    /*
     * 上电后先自动进入“最低站立安全位”。
     *
     * 此时舵机允许工作，但遥控仍处于锁定状态：
     * - 只有当用户把遥控切到约定的最低站立模式
     * - 才会发出滴滴提示音，并开始接受遥控操作
     */
    s_servo_enabled = true;
    s_rc_control_unlocked = false;
    s_log_calibration_exit_once = false;
    s_log_stand_takeover_once = false;
    s_last_aux_sa = 0;
    s_last_aux_sc = 0;
    s_last_aux_sd = 0;
    fill_mid_pose(s_target_angles);
    servo_output_init(&config->calibration);
    kinematics_default_feet(startup_feet_world, config->gait.stand_height_min_mm);
    begin_posture_transition(ROBOT_POSTURE_STAND,
                             config->gait.stand_height_min_mm,
                             startup_feet_world,
                             &(body_pose_t){0},
                             SCARGO_POSTURE_LIFT_MM * 0.5f);
    s_mode = ROBOT_MODE_STAND;
    s_walk_display_phase = 0.0f;
    solve_and_apply_feet(s_current_feet_world, s_current_height_mm, &s_current_body_pose);
    log_mid_pose_forward_kinematics();
    ESP_LOGI(TAG, "Robot control initialized");
}

void robot_control_apply_mid_pose(void)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    s_servo_enabled = true;
    begin_calibration_mid_pose_transition();
    ESP_LOGI(TAG, "Started smooth transition to calibrated mid-pose");
    xSemaphoreGiveRecursive(s_state_mutex);
}

void robot_control_update_calibration(const calibration_config_t *calibration)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    s_config.calibration = *calibration;
    servo_output_update_calibration(calibration);
    xSemaphoreGiveRecursive(s_state_mutex);
}

void robot_control_update_gait(const gait_config_t *gait)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    s_config.gait = *gait;
    kinematics_default_feet(s_default_feet_world, gait->stand_height_default_mm);
    kinematics_rest_feet(s_rest_feet_world, gait->stand_height_default_mm);
    xSemaphoreGiveRecursive(s_state_mutex);
}

void robot_control_update_imu(const imu_config_t *imu)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    s_config.imu = *imu;
    if (!imu->balance_enabled) {
        s_roll_integral = 0.0f;
        s_pitch_integral = 0.0f;
        s_prev_roll_error = 0.0f;
        s_prev_pitch_error = 0.0f;
    }
    xSemaphoreGiveRecursive(s_state_mutex);
}

void robot_control_set_action(robot_action_t action)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    s_calibration_mode_active = false;
    s_pending_action = action;
    xSemaphoreGiveRecursive(s_state_mutex);
}

robot_mode_t robot_control_get_mode(void)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    robot_mode_t mode = s_mode;
    xSemaphoreGiveRecursive(s_state_mutex);
    return mode;
}

void robot_control_set_motion_speed(robot_motion_speed_t speed)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    s_motion_speed = speed;
    xSemaphoreGiveRecursive(s_state_mutex);
}

robot_motion_speed_t robot_control_get_motion_speed(void)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    robot_motion_speed_t speed = s_motion_speed;
    xSemaphoreGiveRecursive(s_state_mutex);
    return speed;
}

void robot_control_cancel_calibration_mode(void)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    s_calibration_mode_active = false;
    s_log_calibration_exit_once = true;
    s_log_stand_takeover_once = true;
    xSemaphoreGiveRecursive(s_state_mutex);
}

bool robot_control_is_calibration_mode_active(void)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    bool active = s_calibration_mode_active;
    xSemaphoreGiveRecursive(s_state_mutex);
    return active;
}

static robot_walk_leg_state_t walk_leg_state_for_phase(int leg, float phase)
{
    float group_phase = leg_group_phase(leg, phase);
    return (group_phase >= s_config.gait.stance_ratio) ? ROBOT_WALK_LEG_STATE_SWING
                                                       : ROBOT_WALK_LEG_STATE_SUPPORT;
}

static const char *walk_status_name(robot_walk_status_t status)
{
    switch (status) {
    case ROBOT_WALK_STATUS_START:
        return "start";
    case ROBOT_WALK_STATUS_STEADY:
        return "steady";
    case ROBOT_WALK_STATUS_STOP:
        return "stop";
    case ROBOT_WALK_STATUS_IDLE:
    default:
        return "idle";
    }
}

static const char *walk_leg_state_name(robot_walk_leg_state_t state)
{
    switch (state) {
    case ROBOT_WALK_LEG_STATE_START:
        return "start";
    case ROBOT_WALK_LEG_STATE_SWING:
        return "swing";
    case ROBOT_WALK_LEG_STATE_SUPPORT:
        return "support";
    case ROBOT_WALK_LEG_STATE_STOP:
        return "stop";
    case ROBOT_WALK_LEG_STATE_IDLE:
    default:
        return "idle";
    }
}

bool robot_control_get_leg_target_angles(int leg, float out_angles_deg[SCARGO_JOINTS_PER_LEG])
{
    if (out_angles_deg == NULL || leg < 0 || leg >= SCARGO_LEG_COUNT) {
        return false;
    }

    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
        out_angles_deg[joint] = s_target_angles[leg][joint];
    }
    xSemaphoreGiveRecursive(s_state_mutex);
    return true;
}

bool robot_control_get_current_feet_world(vec3f_t out_feet_world[SCARGO_LEG_COUNT])
{
    if (out_feet_world == NULL) {
        return false;
    }
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    copy_feet(out_feet_world, s_current_feet_world);
    xSemaphoreGiveRecursive(s_state_mutex);
    return true;
}

body_pose_t robot_control_get_current_body_pose(void)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    body_pose_t pose = s_current_body_pose;
    xSemaphoreGiveRecursive(s_state_mutex);
    return pose;
}

float robot_control_get_current_height_mm(void)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    float h = s_current_height_mm;
    xSemaphoreGiveRecursive(s_state_mutex);
    return h;
}

robot_walk_status_t robot_control_get_walk_status(void)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    robot_walk_status_t status;
    switch (s_walk_phase) {
    case WALK_PHASE_START:
        status = ROBOT_WALK_STATUS_START;
        break;
    case WALK_PHASE_STEADY:
        status = ROBOT_WALK_STATUS_STEADY;
        break;
    case WALK_PHASE_STOP:
        status = ROBOT_WALK_STATUS_STOP;
        break;
    case WALK_PHASE_IDLE:
    default:
        status = ROBOT_WALK_STATUS_IDLE;
        break;
    }
    xSemaphoreGiveRecursive(s_state_mutex);
    return status;
}

float robot_control_get_walk_phase_value(void)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    float phase = s_walk_display_phase;
    xSemaphoreGiveRecursive(s_state_mutex);
    return phase;
}

bool robot_control_get_walk_leg_states(robot_walk_leg_state_t out_states[SCARGO_LEG_COUNT])
{
    if (out_states == NULL) {
        return false;
    }

    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    robot_walk_status_t status = robot_control_get_walk_status();
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        switch (status) {
        case ROBOT_WALK_STATUS_START:
            out_states[leg] = ROBOT_WALK_LEG_STATE_START;
            break;
        case ROBOT_WALK_STATUS_STOP:
            out_states[leg] = ROBOT_WALK_LEG_STATE_STOP;
            break;
        case ROBOT_WALK_STATUS_STEADY:
            out_states[leg] = walk_leg_state_for_phase(leg, s_walk_display_phase);
            break;
        case ROBOT_WALK_STATUS_IDLE:
        default:
            out_states[leg] = ROBOT_WALK_LEG_STATE_IDLE;
            break;
        }
    }
    xSemaphoreGiveRecursive(s_state_mutex);
    return true;
}

robot_target_pose_t robot_control_get_target_pose(const rc_command_t *command)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    bool walk_mode = command->walk_mode || s_mode == ROBOT_MODE_WALK;
    robot_target_pose_t target = {
        .yaw_deg = walk_mode ? 0.0f : command->yaw * dynamic_max_yaw_deg(command),
        .pitch_deg = 0.0f,
        .roll_deg = 0.0f,
        .height_mm = body_height_from_throttle(command),
    };
    if (!walk_mode) {
        target.roll_deg = -command->roll * s_config.gait.max_body_roll_deg;
        target.pitch_deg = -command->pitch * s_config.gait.max_body_pitch_deg;
    }
    xSemaphoreGiveRecursive(s_state_mutex);
    return target;
}

void robot_control_control_tick(const rc_command_t *command, const attitude_state_t *attitude)
{
    xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    static uint32_t tick_counter = 0;
    ++tick_counter;
    const float dt_s = 1.0f / (float)SCARGO_CONTROL_RATE_HZ;
    const bool rc_ready_mode = command->link_up && !command->walk_mode &&
                               command->throttle <= -0.95f && command->aux_sd <= 0;

    if (!s_servo_enabled) {
        s_last_aux_sa = command->aux_sa;
        s_last_aux_sd = command->aux_sd;
        xSemaphoreGiveRecursive(s_state_mutex);
        return;
    }

    update_transition(dt_s);

    if (!s_rc_control_unlocked) {
        if (rc_ready_mode) {
            s_rc_control_unlocked = true;
            buzzer_service_rc_unlock_beep();
        } else {
            /*
             * 遥控尚未解锁前，始终保持在最低站立安全位。
             * 这时忽略所有实时遥控输入，只维持一个稳定、可预期的起始姿态。
             */
            rc_command_t hold_command = *command;
            hold_command.walk_mode = false;
            hold_command.aux_sb = 0;
            hold_command.aux_sd = 0;
            hold_command.yaw = 0.0f;
            hold_command.pitch = 0.0f;
            hold_command.roll = 0.0f;
            hold_command.throttle = -1.0f;
            apply_stand_pose(&hold_command, attitude);
            update_fan_auto_context(command->link_up);
            s_last_aux_sa = command->aux_sa;
            s_last_aux_sc = command->aux_sc;
            s_last_aux_sd = command->aux_sd;
            xSemaphoreGiveRecursive(s_state_mutex);
            return;
        }
    }

    if (s_calibration_mode_active && command->aux_sd <= 0 && s_last_aux_sd > 0) {
        robot_control_cancel_calibration_mode();
        begin_posture_transition(ROBOT_POSTURE_STAND,
                                 body_height_from_throttle(command),
                                 s_default_feet_world,
                                 &(body_pose_t){0},
                                 SCARGO_POSTURE_LIFT_MM * 0.5f);
        s_mode = ROBOT_MODE_STAND;
        s_walk_phase = WALK_PHASE_IDLE;
        s_walk_phase_progress = 0.0f;
    }

    if (update_calibration_transition(dt_s)) {
        if ((tick_counter % 100U) == 0U) {
            ESP_LOGI(TAG, "calibration mode active");
        }
        update_fan_auto_context(command->link_up);
        s_last_aux_sa = command->aux_sa;
        s_last_aux_sd = command->aux_sd;
        xSemaphoreGiveRecursive(s_state_mutex);
        return;
    }

    if (s_calibration_mode_active) {
        /*
         * 标定稳定态：
         * - OLED 继续显示安装层定义的中位预览
         * - 真实舵机输出固定停在 90/90/90 + offset 对应的安装位
         * - 不再复用正常运行态的小腿修正链
         */
        fill_mid_pose(s_target_angles);
        copy_feet(s_current_feet_world, s_calibration_preview_target_feet_world);
        s_current_body_pose = s_calibration_preview_pose;
        s_current_height_mm = s_calibration_preview_height_mm;
        servo_output_set_calibration_group_angles_deg(s_target_angles);

        if ((tick_counter % 100U) == 0U) {
            log_all_calf_outputs("calib hold", s_target_angles, &s_config.calibration);
            ESP_LOGI(TAG, "calibration mode active");
        }
        update_fan_auto_context(command->link_up);
        s_last_aux_sa = command->aux_sa;
        s_last_aux_sd = command->aux_sd;
        xSemaphoreGiveRecursive(s_state_mutex);
        return;
    }

    if (s_log_calibration_exit_once) {
        log_all_calf_outputs("calib exit first tick", s_target_angles, &s_config.calibration);
        s_log_calibration_exit_once = false;
    }

    if (!command->walk_mode && command->aux_sd > 0 && s_last_aux_sd <= 0) {
        robot_control_apply_mid_pose();
        s_pending_action = ROBOT_ACTION_NONE;
        s_walk_phase = WALK_PHASE_IDLE;
        s_walk_phase_progress = 0.0f;
        update_fan_auto_context(command->link_up);
        s_last_aux_sc = command->aux_sc;
        s_last_aux_sd = command->aux_sd;
        xSemaphoreGiveRecursive(s_state_mutex);
        return;
    }

    /*
     * SC 三档控制对角腿站立模式：
     * - sc = -1 : Pair A（FR+RL 着地，FL+RR 抬起）
     * - sc =  0 : 普通站立（退出对角腿）
     * - sc = +1 : Pair B（FL+RR 着地，FR+RL 抬起）
     *
     * 只在站立模式、非行走、非标定时生效。
     * 切换时只响应边沿，避免持续触发。
     */
    if (!command->walk_mode && s_mode != ROBOT_MODE_WALK && command->aux_sc != s_last_aux_sc) {
        if (command->aux_sc == -1) {
            if (s_posture != ROBOT_POSTURE_DIAGONAL_A) {
                begin_diagonal_stand_transition(ROBOT_POSTURE_DIAGONAL_A);
                ESP_LOGI(TAG, "SC: enter diagonal A (FR+RL)");
            }
        } else if (command->aux_sc == 1) {
            if (s_posture != ROBOT_POSTURE_DIAGONAL_B) {
                begin_diagonal_stand_transition(ROBOT_POSTURE_DIAGONAL_B);
                ESP_LOGI(TAG, "SC: enter diagonal B (FL+RR)");
            }
        } else {
            if (s_posture == ROBOT_POSTURE_DIAGONAL_A || s_posture == ROBOT_POSTURE_DIAGONAL_B) {
                begin_posture_transition(ROBOT_POSTURE_STAND,
                                         body_height_from_throttle(command),
                                         s_default_feet_world,
                                         &(body_pose_t){0},
                                         0.0f);
                ESP_LOGI(TAG, "SC: exit diagonal, return to stand");
            }
        }
    }

    if (!command->walk_mode) {
        if (s_posture == ROBOT_POSTURE_LIE && !s_transition_active) {
            begin_posture_transition(ROBOT_POSTURE_STAND,
                                     body_height_from_throttle(command),
                                     s_default_feet_world,
                                     &(body_pose_t){0},
                                     SCARGO_POSTURE_LIFT_MM);
            s_mode = ROBOT_MODE_STAND;
        }
    }

    if (s_pending_action != ROBOT_ACTION_NONE) {
        switch (s_pending_action) {
        case ROBOT_ACTION_STAND:
        if (s_mode == ROBOT_MODE_WALK && (s_walk_phase == WALK_PHASE_STEADY || s_walk_phase == WALK_PHASE_START)) {
            s_walk_phase = WALK_PHASE_IDLE;
            s_walk_phase_progress = 0.0f;
            begin_posture_transition(ROBOT_POSTURE_STAND,
                                     body_height_from_throttle(command),
                                     s_default_feet_world,
                                     &(body_pose_t){0},
                                     SCARGO_POSTURE_LIFT_MM * 0.5f);
            s_mode = ROBOT_MODE_STAND;
        } else {
            begin_posture_transition(ROBOT_POSTURE_STAND,
                                     body_height_from_throttle(command),
                                     s_default_feet_world,
                                     &(body_pose_t){0},
                                     SCARGO_POSTURE_LIFT_MM);
                s_mode = ROBOT_MODE_STAND;
            }
            break;
        case ROBOT_ACTION_WALK:
        case ROBOT_ACTION_FORWARD:
        case ROBOT_ACTION_BACKWARD:
        case ROBOT_ACTION_SHIFT_LEFT:
        case ROBOT_ACTION_SHIFT_RIGHT:
        case ROBOT_ACTION_TURN_LEFT:
        case ROBOT_ACTION_TURN_RIGHT:
            if (s_posture == ROBOT_POSTURE_LIE) {
                begin_posture_transition(ROBOT_POSTURE_STAND,
                                         s_config.gait.stand_height_default_mm,
                                         s_default_feet_world,
                                         &(body_pose_t){0},
                                         SCARGO_POSTURE_LIFT_MM);
            }
            s_mode = ROBOT_MODE_WALK;
            if (s_walk_phase == WALK_PHASE_IDLE || s_walk_phase == WALK_PHASE_STOP) {
                s_walk_phase = WALK_PHASE_START;
                begin_walk_start_transition(command, attitude, fmaxf(gait_cycle_effective_ms(command) / 1000.0f, 0.15f));
            }
            break;
        case ROBOT_ACTION_LIE_DOWN:
            s_walk_phase = WALK_PHASE_IDLE;
            s_walk_phase_progress = 0.0f;
            begin_posture_transition(ROBOT_POSTURE_LIE,
                                     SCARGO_LIE_HEIGHT_MM,
                                     s_rest_feet_world,
                                     &(body_pose_t){0},
                                     SCARGO_POSTURE_LIFT_LIE_MM);
            s_mode = ROBOT_MODE_STAND;
            break;
        case ROBOT_ACTION_NONE:
        default:
            break;
        }
        s_pending_action = ROBOT_ACTION_NONE;
    }

    if (command->walk_mode) {
        if (s_posture == ROBOT_POSTURE_LIE ||
            s_posture == ROBOT_POSTURE_DIAGONAL_A || s_posture == ROBOT_POSTURE_DIAGONAL_B) {
            begin_posture_transition(ROBOT_POSTURE_STAND,
                                     body_height_from_throttle(command),
                                     s_default_feet_world,
                                     &(body_pose_t){0},
                                     SCARGO_POSTURE_LIFT_MM);
        }
        s_mode = ROBOT_MODE_WALK;
        if (s_walk_phase == WALK_PHASE_IDLE) {
            begin_walk_start_transition(command, attitude, fmaxf(gait_cycle_effective_ms(command) / 1000.0f, 0.15f));
        }
    }
    if (!command->walk_mode && s_mode == ROBOT_MODE_WALK &&
        (s_walk_phase == WALK_PHASE_STEADY || s_walk_phase == WALK_PHASE_START)) {
        /*
         * 如果在起步阶段就要求停止，由于起步本身就是朝 phase=0.5 收敛，
         * 允许它先完成这一小段，再在下一个 tick 进入独立收步。
         */
        s_walk_stop_pending = true;
    }

    if (s_posture == ROBOT_POSTURE_LIE && !s_transition_active && s_mode == ROBOT_MODE_STAND) {
        apply_lie_hold_pose();
    } else if ((s_posture == ROBOT_POSTURE_DIAGONAL_A || s_posture == ROBOT_POSTURE_DIAGONAL_B) && !s_transition_active) {
        apply_diagonal_stand_pose(command, attitude);
    } else if (s_mode == ROBOT_MODE_STAND || s_walk_phase == WALK_PHASE_IDLE) {
        apply_stand_pose(command, attitude);
    } else {
        apply_walk_pose(command, attitude);
    }

    update_fan_auto_context(command->link_up);
    servo_output_set_group_angles_deg(s_target_angles);

    if ((tick_counter % 100U) == 0U) {
        ESP_LOGI(TAG,
                 "mode=%s throttle=%.2f yaw=%.2f pitch=%.2f roll=%.2f attitude(rpy)=%.1f/%.1f/%.1f",
                 s_mode == ROBOT_MODE_WALK ? "walk" : "stand",
                 command->throttle,
                 command->yaw,
                 command->pitch,
                 command->roll,
                 attitude->roll_deg,
                 attitude->pitch_deg,
                 attitude->yaw_deg);
        if (s_mode == ROBOT_MODE_WALK || s_walk_phase != WALK_PHASE_IDLE) {
            robot_walk_leg_state_t leg_states[SCARGO_LEG_COUNT];
            if (robot_control_get_walk_leg_states(leg_states)) {
                ESP_LOGI(TAG,
                         "walk dbg status=%s phase=%.2f leg0=%s leg1=%s leg2=%s leg3=%s",
                         walk_status_name(robot_control_get_walk_status()),
                         robot_control_get_walk_phase_value(),
                         walk_leg_state_name(leg_states[0]),
                         walk_leg_state_name(leg_states[1]),
                         walk_leg_state_name(leg_states[2]),
                         walk_leg_state_name(leg_states[3]));
                ESP_LOGI(TAG,
                         "walk dbg dz leg0=%.1f leg1=%.1f leg2=%.1f leg3=%.1f",
                         (double)(s_current_feet_world[0].z_mm - s_default_feet_world[0].z_mm),
                         (double)(s_current_feet_world[1].z_mm - s_default_feet_world[1].z_mm),
                         (double)(s_current_feet_world[2].z_mm - s_default_feet_world[2].z_mm),
                         (double)(s_current_feet_world[3].z_mm - s_default_feet_world[3].z_mm));
            }
        }
    }

    s_last_aux_sa = command->aux_sa;
    s_last_aux_sc = command->aux_sc;
    s_last_aux_sd = command->aux_sd;
    xSemaphoreGiveRecursive(s_state_mutex);
}
