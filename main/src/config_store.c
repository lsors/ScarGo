#include "config_store.h"

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int clampi(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void config_store_set_defaults(system_config_t *config)
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            config->calibration.servo_offsets_deg[leg][joint] = 0;
        }
    }

    config->gait.stand_height_default_mm = SCARGO_DEFAULT_STAND_HEIGHT_MM;
    config->gait.stand_height_min_mm = SCARGO_MIN_HEIGHT_MM;
    config->gait.stand_height_max_mm = SCARGO_MAX_HEIGHT_MM;
    config->gait.max_body_yaw_deg = SCARGO_MAX_BODY_YAW_DEG;
    config->gait.max_body_roll_deg = SCARGO_MAX_BODY_ROLL_DEG;
    config->gait.max_body_pitch_deg = SCARGO_MAX_BODY_PITCH_DEG;
    config->gait.step_height_mm = SCARGO_DEFAULT_STEP_HEIGHT_MM;
    config->gait.step_height_min_mm = SCARGO_MIN_STEP_HEIGHT_MM;
    config->gait.step_height_max_mm = SCARGO_MAX_STEP_HEIGHT_MM;
    config->gait.step_cycle_ms = SCARGO_DEFAULT_STEP_CYCLE_MS;
    config->gait.step_cycle_min_ms = SCARGO_MIN_STEP_CYCLE_MS;
    config->gait.step_cycle_max_ms = SCARGO_MAX_STEP_CYCLE_MS;
    config->gait.step_scale = SCARGO_DEFAULT_STEP_SCALE;
    config->gait.step_scale_min = SCARGO_MIN_STEP_SCALE;
    config->gait.step_scale_max = SCARGO_MAX_STEP_SCALE;
    config->gait.stance_ratio = SCARGO_DEFAULT_STANCE_RATIO;
    config->gait.stance_ratio_min = SCARGO_MIN_STANCE_RATIO;
    config->gait.stance_ratio_max = SCARGO_MAX_STANCE_RATIO;
    config->gait.diagonal_phase_offset = SCARGO_DEFAULT_PHASE_OFFSET;
    config->oled.page = 6;
    config->oled.leg = 0;
    config->oled.leg_view = 5;
    config->oled.robot_view = 5;
}

bool config_store_validate(system_config_t *config)
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        for (int joint = 0; joint < SCARGO_JOINTS_PER_LEG; ++joint) {
            config->calibration.servo_offsets_deg[leg][joint] =
                clampi(config->calibration.servo_offsets_deg[leg][joint], -90, 90);
        }
    }

    config->gait.stand_height_min_mm = clampf(config->gait.stand_height_min_mm, SCARGO_MIN_HEIGHT_MM, SCARGO_MAX_HEIGHT_MM);
    config->gait.stand_height_max_mm = clampf(config->gait.stand_height_max_mm, SCARGO_MIN_HEIGHT_MM, SCARGO_MAX_HEIGHT_MM);
    if (config->gait.stand_height_min_mm > config->gait.stand_height_max_mm) {
        config->gait.stand_height_min_mm = SCARGO_MIN_HEIGHT_MM;
        config->gait.stand_height_max_mm = SCARGO_MAX_HEIGHT_MM;
    }
    config->gait.stand_height_default_mm =
        clampf(config->gait.stand_height_default_mm, config->gait.stand_height_min_mm, config->gait.stand_height_max_mm);

    config->gait.step_height_min_mm = clampf(config->gait.step_height_min_mm, SCARGO_MIN_STEP_HEIGHT_MM, SCARGO_MAX_STEP_HEIGHT_MM);
    config->gait.step_height_max_mm = clampf(config->gait.step_height_max_mm, SCARGO_MIN_STEP_HEIGHT_MM, SCARGO_MAX_STEP_HEIGHT_MM);
    if (config->gait.step_height_min_mm > config->gait.step_height_max_mm) {
        config->gait.step_height_min_mm = SCARGO_MIN_STEP_HEIGHT_MM;
        config->gait.step_height_max_mm = SCARGO_MAX_STEP_HEIGHT_MM;
    }
    config->gait.step_height_mm =
        clampf(config->gait.step_height_mm, config->gait.step_height_min_mm, config->gait.step_height_max_mm);
    config->gait.step_cycle_min_ms = clampi(config->gait.step_cycle_min_ms, SCARGO_MIN_STEP_CYCLE_MS, SCARGO_MAX_STEP_CYCLE_MS);
    config->gait.step_cycle_max_ms = clampi(config->gait.step_cycle_max_ms, SCARGO_MIN_STEP_CYCLE_MS, SCARGO_MAX_STEP_CYCLE_MS);
    if (config->gait.step_cycle_min_ms > config->gait.step_cycle_max_ms) {
        config->gait.step_cycle_min_ms = SCARGO_MIN_STEP_CYCLE_MS;
        config->gait.step_cycle_max_ms = SCARGO_MAX_STEP_CYCLE_MS;
    }
    config->gait.step_cycle_ms =
        clampi(config->gait.step_cycle_ms, config->gait.step_cycle_min_ms, config->gait.step_cycle_max_ms);
    config->gait.step_scale = clampf(config->gait.step_scale, config->gait.step_scale_min, config->gait.step_scale_max);
    config->gait.step_scale_min = clampf(config->gait.step_scale_min, SCARGO_MIN_STEP_SCALE, SCARGO_MAX_STEP_SCALE);
    config->gait.step_scale_max = clampf(config->gait.step_scale_max, SCARGO_MIN_STEP_SCALE, SCARGO_MAX_STEP_SCALE);
    if (config->gait.step_scale_min > config->gait.step_scale_max) {
        config->gait.step_scale_min = SCARGO_MIN_STEP_SCALE;
        config->gait.step_scale_max = SCARGO_MAX_STEP_SCALE;
    }
    config->gait.stance_ratio_min = clampf(config->gait.stance_ratio_min, SCARGO_MIN_STANCE_RATIO, SCARGO_MAX_STANCE_RATIO);
    config->gait.stance_ratio_max = clampf(config->gait.stance_ratio_max, SCARGO_MIN_STANCE_RATIO, SCARGO_MAX_STANCE_RATIO);
    if (config->gait.stance_ratio_min > config->gait.stance_ratio_max) {
        config->gait.stance_ratio_min = SCARGO_MIN_STANCE_RATIO;
        config->gait.stance_ratio_max = SCARGO_MAX_STANCE_RATIO;
    }
    config->gait.stance_ratio = clampf(config->gait.stance_ratio, config->gait.stance_ratio_min, config->gait.stance_ratio_max);
    config->gait.max_body_yaw_deg = clampf(config->gait.max_body_yaw_deg, 0.0f, SCARGO_MAX_BODY_YAW_DEG);
    config->gait.max_body_roll_deg = clampf(config->gait.max_body_roll_deg, 0.0f, SCARGO_MAX_BODY_ROLL_DEG);
    config->gait.max_body_pitch_deg = clampf(config->gait.max_body_pitch_deg, 0.0f, SCARGO_MAX_BODY_PITCH_DEG);
    config->gait.diagonal_phase_offset = clampf(config->gait.diagonal_phase_offset, 0.0f, 0.5f);

    config->oled.page = clampi(config->oled.page, 0, 6);
    config->oled.leg = clampi(config->oled.leg, 0, SCARGO_LEG_COUNT - 1);
    config->oled.leg_view = clampi(config->oled.leg_view, 0, 5);
    config->oled.robot_view = clampi(config->oled.robot_view, 0, 5);

    return true;
}
