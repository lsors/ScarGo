#include "kinematics.h"

#include <math.h>

static float deg2rad(float deg)
{
    return deg * ((float)M_PI / 180.0f);
}

static float rad2deg(float rad)
{
    return rad * (180.0f / (float)M_PI);
}

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

static vec3f_t rotate_x(vec3f_t input, float radians)
{
    const float c = cosf(radians);
    const float s = sinf(radians);
    return (vec3f_t){
        .x_mm = input.x_mm,
        .y_mm = input.y_mm * c - input.z_mm * s,
        .z_mm = input.y_mm * s + input.z_mm * c,
    };
}

static vec3f_t rotate_y(vec3f_t input, float radians)
{
    const float c = cosf(radians);
    const float s = sinf(radians);
    return (vec3f_t){
        .x_mm = input.x_mm * c + input.z_mm * s,
        .y_mm = input.y_mm,
        .z_mm = -input.x_mm * s + input.z_mm * c,
    };
}

static vec3f_t rotate_z(vec3f_t input, float radians)
{
    const float c = cosf(radians);
    const float s = sinf(radians);
    return (vec3f_t){
        .x_mm = input.x_mm * c - input.y_mm * s,
        .y_mm = input.x_mm * s + input.y_mm * c,
        .z_mm = input.z_mm,
    };
}

void kinematics_default_feet(vec3f_t feet_body[SCARGO_LEG_COUNT], float stand_height_mm)
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    const float half_width = mech->body_width_mm * 0.5f;
    const float half_length = mech->body_length_mm * 0.5f;
    const float outward = mech->shoulder_length_mm;
    const float knee_forward = mech->calf_length_mm;

    feet_body[SCARGO_LEG_FRONT_RIGHT] = (vec3f_t){.x_mm = -half_width - outward, .y_mm = half_length + knee_forward, .z_mm = -stand_height_mm};
    feet_body[SCARGO_LEG_FRONT_LEFT] = (vec3f_t){.x_mm = half_width + outward, .y_mm = half_length + knee_forward, .z_mm = -stand_height_mm};
    feet_body[SCARGO_LEG_REAR_RIGHT] = (vec3f_t){.x_mm = -half_width - outward, .y_mm = -half_length + knee_forward, .z_mm = -stand_height_mm};
    feet_body[SCARGO_LEG_REAR_LEFT] = (vec3f_t){.x_mm = half_width + outward, .y_mm = -half_length + knee_forward, .z_mm = -stand_height_mm};
}

void kinematics_apply_body_pose(vec3f_t feet_body[SCARGO_LEG_COUNT], const vec3f_t feet_world[SCARGO_LEG_COUNT],
                                float stand_height_mm, const body_pose_t *pose)
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    const float half_width = mech->body_width_mm * 0.5f;
    const float half_length = mech->body_length_mm * 0.5f;
    const vec3f_t hip_offsets[SCARGO_LEG_COUNT] = {
        [SCARGO_LEG_FRONT_RIGHT] = {.x_mm = -half_width, .y_mm = half_length, .z_mm = 0.0f},
        [SCARGO_LEG_FRONT_LEFT] = {.x_mm = half_width, .y_mm = half_length, .z_mm = 0.0f},
        [SCARGO_LEG_REAR_RIGHT] = {.x_mm = -half_width, .y_mm = -half_length, .z_mm = 0.0f},
        [SCARGO_LEG_REAR_LEFT] = {.x_mm = half_width, .y_mm = -half_length, .z_mm = 0.0f},
    };

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        vec3f_t relative = {
            .x_mm = feet_world[leg].x_mm - hip_offsets[leg].x_mm,
            .y_mm = feet_world[leg].y_mm - hip_offsets[leg].y_mm,
            .z_mm = feet_world[leg].z_mm + stand_height_mm - hip_offsets[leg].z_mm,
        };

        relative = rotate_z(relative, -deg2rad(pose->yaw_deg));
        relative = rotate_y(relative, -deg2rad(pose->pitch_deg));
        relative = rotate_x(relative, -deg2rad(pose->roll_deg));
        feet_body[leg] = relative;
    }
}

bool kinematics_solve_leg(scargo_leg_id_t leg, const vec3f_t *foot_body, leg_servo_pose_t *out_pose)
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    const scargo_leg_kinematics_binding_t *leg_bindings = board_defaults_leg_kinematics();

    const float side_sign = SCARGO_BODY_SIDE_SIGN(leg);
    const float lateral = side_sign * foot_body->x_mm;
    const float forward = foot_body->y_mm;
    const float vertical = foot_body->z_mm;
    const float radial = sqrtf(lateral * lateral + vertical * vertical);
    if (radial <= mech->shoulder_length_mm + 1.0f) {
        return false;
    }

    float shoulder_aux = clampf(mech->shoulder_length_mm / radial, -1.0f, 1.0f);
    float shoulder_deg = rad2deg(atan2f(-vertical, lateral) - acosf(shoulder_aux));
    float sagittal = sqrtf(fmaxf(radial * radial - mech->shoulder_length_mm * mech->shoulder_length_mm, 0.0f));
    float distance = sqrtf(forward * forward + sagittal * sagittal);
    if (distance < 1.0f || distance > (mech->thigh_length_mm + mech->calf_length_mm)) {
        return false;
    }

    float knee_cos = clampf((mech->thigh_length_mm * mech->thigh_length_mm +
                             mech->calf_length_mm * mech->calf_length_mm - distance * distance) /
                                (2.0f * mech->thigh_length_mm * mech->calf_length_mm),
                            -1.0f, 1.0f);
    float knee_deg = rad2deg((float)M_PI - acosf(knee_cos));

    float hip_target = rad2deg(atan2f(forward, -sagittal));
    float hip_cos = clampf((mech->thigh_length_mm * mech->thigh_length_mm + distance * distance -
                            mech->calf_length_mm * mech->calf_length_mm) /
                               (2.0f * mech->thigh_length_mm * distance),
                           -1.0f, 1.0f);
    float thigh_math_deg = hip_target - rad2deg(acosf(hip_cos));
    float thigh_servo_deg = 90.0f - thigh_math_deg;
    float calf_servo_deg = thigh_servo_deg +
                           (float)leg_bindings[leg].knee_coupling_sign * knee_deg +
                           leg_bindings[leg].knee_coupling_offset_deg;

    out_pose->shoulder_deg = 90.0f + shoulder_deg;
    out_pose->thigh_servo_deg = thigh_servo_deg;
    out_pose->calf_servo_deg = calf_servo_deg;
    out_pose->knee_deg = knee_deg;
    return true;
}
