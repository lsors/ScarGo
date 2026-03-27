#include "robot_control.h"

#include <math.h>
#include <stdint.h>

#include "board_defaults.h"
#include "esp_log.h"
#include "kinematics.h"
#include "servo_output.h"

static const char *TAG = "robot";

static system_config_t s_config;
static robot_mode_t s_mode = ROBOT_MODE_STAND;
static robot_action_t s_pending_action = ROBOT_ACTION_NONE;
static float s_target_angles[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG];
static vec3f_t s_default_feet_world[SCARGO_LEG_COUNT];
static float s_roll_integral;
static float s_pitch_integral;
static float s_prev_roll_error;
static float s_prev_pitch_error;

static void fill_mid_pose(float target[SCARGO_LEG_COUNT][SCARGO_JOINTS_PER_LEG])
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        target[leg][SCARGO_JOINT_SHOULDER] = 90.0f;
        target[leg][SCARGO_JOINT_THIGH] = 90.0f;
        target[leg][SCARGO_JOINT_CALF] = 90.0f;
    }
}

static void solve_and_apply_feet(const vec3f_t feet_world[SCARGO_LEG_COUNT], float height_mm, const body_pose_t *pose)
{
    vec3f_t feet_body[SCARGO_LEG_COUNT];
    kinematics_apply_body_pose(feet_body, feet_world, height_mm, pose);

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        leg_servo_pose_t pose_solution;
        if (!kinematics_solve_leg((scargo_leg_id_t)leg, &feet_body[leg], &pose_solution)) {
            continue;
        }
        s_target_angles[leg][SCARGO_JOINT_SHOULDER] = pose_solution.shoulder_deg;
        s_target_angles[leg][SCARGO_JOINT_THIGH] = pose_solution.thigh_servo_deg;
        s_target_angles[leg][SCARGO_JOINT_CALF] = pose_solution.calf_servo_deg;
    }
}

static void apply_balance(body_pose_t *pose, const attitude_state_t *attitude)
{
    if (!attitude->ready) {
        return;
    }

    const float dt = 1.0f / (float)SCARGO_CONTROL_RATE_HZ;
    float roll_error = pose->roll_deg - attitude->roll_deg;
    float pitch_error = pose->pitch_deg - attitude->pitch_deg;

    s_roll_integral += roll_error * dt;
    s_pitch_integral += pitch_error * dt;

    float roll_derivative = (roll_error - s_prev_roll_error) / dt;
    float pitch_derivative = (pitch_error - s_prev_pitch_error) / dt;

    pose->roll_deg += roll_error * SCARGO_BALANCE_KP_ROLL +
                      s_roll_integral * SCARGO_BALANCE_KI_ROLL +
                      roll_derivative * SCARGO_BALANCE_KD_ROLL;
    pose->pitch_deg += pitch_error * SCARGO_BALANCE_KP_PITCH +
                       s_pitch_integral * SCARGO_BALANCE_KI_PITCH +
                       pitch_derivative * SCARGO_BALANCE_KD_PITCH;

    s_prev_roll_error = roll_error;
    s_prev_pitch_error = pitch_error;
}

static void apply_stand_pose(const rc_command_t *command, const attitude_state_t *attitude)
{
    float height_ratio = (command->throttle + 1.0f) * 0.5f;
    float target_height = s_config.gait.stand_height_min_mm +
                          height_ratio * (s_config.gait.stand_height_max_mm - s_config.gait.stand_height_min_mm);

    body_pose_t pose = {
        .yaw_deg = command->yaw * s_config.gait.max_body_yaw_deg,
        .pitch_deg = command->pitch * s_config.gait.max_body_pitch_deg,
        .roll_deg = command->roll * s_config.gait.max_body_roll_deg,
    };

    fill_mid_pose(s_target_angles);
    apply_balance(&pose, attitude);
    solve_and_apply_feet(s_default_feet_world, target_height, &pose);
}

static float cycloid_phase(float phase)
{
    return phase - sinf(2.0f * (float)M_PI * phase) / (2.0f * (float)M_PI);
}

static void apply_walk_pose(const rc_command_t *command, const attitude_state_t *attitude)
{
    static uint32_t phase_tick;
    phase_tick++;

    float throttle_ratio = (command->throttle + 1.0f) * 0.5f;
    float stride = throttle_ratio * s_config.gait.step_height_mm * s_config.gait.step_scale;
    float cycle = (float)s_config.gait.step_cycle_ms;
    float base_phase = fmodf(((float)(phase_tick * 10U)) / cycle, 1.0f);
    float stand_height = s_config.gait.stand_height_min_mm +
                         ((command->pitch + 1.0f) * 0.5f) *
                             (s_config.gait.stand_height_max_mm - s_config.gait.stand_height_min_mm);
    body_pose_t pose = {
        .yaw_deg = 0.0f,
        .pitch_deg = 0.0f,
        .roll_deg = 0.0f,
    };
    vec3f_t feet_world[SCARGO_LEG_COUNT];

    kinematics_default_feet(feet_world, stand_height);
    fill_mid_pose(s_target_angles);

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        float group_phase = base_phase + ((leg == SCARGO_LEG_FRONT_LEFT || leg == SCARGO_LEG_REAR_RIGHT) ? 0.0f : 0.5f);
        if (leg == SCARGO_LEG_REAR_RIGHT || leg == SCARGO_LEG_FRONT_RIGHT) {
            group_phase += s_config.gait.diagonal_phase_offset;
        }
        group_phase = fmodf(group_phase, 1.0f);

        bool in_swing = group_phase >= s_config.gait.stance_ratio;
        float phase_local;
        float side_sign = SCARGO_BODY_SIDE_SIGN(leg);

        if (in_swing) {
            phase_local = (group_phase - s_config.gait.stance_ratio) / fmaxf(1.0f - s_config.gait.stance_ratio, 0.01f);
            float cycloid = cycloid_phase(phase_local);
            feet_world[leg].y_mm += (-0.5f + cycloid) * stride + command->yaw * side_sign * stride * 0.35f;
            feet_world[leg].z_mm += s_config.gait.step_height_mm * sinf((float)M_PI * phase_local);
        } else {
            phase_local = group_phase / fmaxf(s_config.gait.stance_ratio, 0.01f);
            feet_world[leg].y_mm += (0.5f - phase_local) * stride + command->yaw * side_sign * stride * 0.35f;
        }

        feet_world[leg].x_mm += command->roll * 18.0f;
    }

    apply_balance(&pose, attitude);
    solve_and_apply_feet(feet_world, stand_height, &pose);
}

void robot_control_init(const system_config_t *config)
{
    s_config = *config;
    kinematics_default_feet(s_default_feet_world, config->gait.stand_height_default_mm);
    fill_mid_pose(s_target_angles);
    servo_output_init(&config->calibration);
    ESP_LOGI(TAG, "Robot control initialized");
}

void robot_control_apply_mid_pose(void)
{
    servo_output_apply_mid_pose();
    ESP_LOGI(TAG, "Applied calibrated mid-pose to all servos");
}

void robot_control_update_calibration(const calibration_config_t *calibration)
{
    s_config.calibration = *calibration;
    servo_output_update_calibration(calibration);
}

void robot_control_update_gait(const gait_config_t *gait)
{
    s_config.gait = *gait;
    kinematics_default_feet(s_default_feet_world, gait->stand_height_default_mm);
}

void robot_control_set_action(robot_action_t action)
{
    s_pending_action = action;
}

robot_mode_t robot_control_get_mode(void)
{
    return s_mode;
}

void robot_control_control_tick(const rc_command_t *command, const attitude_state_t *attitude)
{
    static uint32_t tick_counter = 0;
    ++tick_counter;

    if (s_pending_action != ROBOT_ACTION_NONE) {
        switch (s_pending_action) {
        case ROBOT_ACTION_STAND:
            s_mode = ROBOT_MODE_STAND;
            break;
        case ROBOT_ACTION_WALK:
        case ROBOT_ACTION_FORWARD:
        case ROBOT_ACTION_BACKWARD:
        case ROBOT_ACTION_SHIFT_LEFT:
        case ROBOT_ACTION_SHIFT_RIGHT:
        case ROBOT_ACTION_TURN_LEFT:
        case ROBOT_ACTION_TURN_RIGHT:
            s_mode = ROBOT_MODE_WALK;
            break;
        case ROBOT_ACTION_LIE_DOWN:
            fill_mid_pose(s_target_angles);
            for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
                s_target_angles[leg][SCARGO_JOINT_THIGH] = 60.0f;
                s_target_angles[leg][SCARGO_JOINT_CALF] = 50.0f;
            }
            servo_output_set_group_angles_deg(s_target_angles);
            s_pending_action = ROBOT_ACTION_NONE;
            return;
        case ROBOT_ACTION_NONE:
        default:
            break;
        }
        s_pending_action = ROBOT_ACTION_NONE;
    }

    s_mode = command->walk_mode ? ROBOT_MODE_WALK : s_mode;
    if (!command->walk_mode && s_mode != ROBOT_MODE_STAND) {
        s_mode = ROBOT_MODE_STAND;
    }

    if (s_mode == ROBOT_MODE_STAND) {
        apply_stand_pose(command, attitude);
    } else {
        apply_walk_pose(command, attitude);
    }

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
    }
}
