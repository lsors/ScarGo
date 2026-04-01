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
    (void)stand_height_mm;

    feet_body[SCARGO_LEG_FRONT_RIGHT] = (vec3f_t){.x_mm = -half_width, .y_mm = half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_FRONT_LEFT] = (vec3f_t){.x_mm = half_width, .y_mm = half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_REAR_RIGHT] = (vec3f_t){.x_mm = -half_width, .y_mm = -half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_REAR_LEFT] = (vec3f_t){.x_mm = half_width, .y_mm = -half_length, .z_mm = 0.0f};
}

void kinematics_rest_feet(vec3f_t feet_body[SCARGO_LEG_COUNT], float reference_height_mm)
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    const float half_width = mech->body_width_mm * 0.5f;
    const float half_length = mech->body_length_mm * 0.5f;
    (void)reference_height_mm;

    // During stand/lie transitions keep each foot directly under its hip in body X/Y.
    // That makes the foot motion primarily vertical in the leg frame instead of drifting forward.
    feet_body[SCARGO_LEG_FRONT_RIGHT] = (vec3f_t){.x_mm = -half_width, .y_mm = half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_FRONT_LEFT] = (vec3f_t){.x_mm = half_width, .y_mm = half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_REAR_RIGHT] = (vec3f_t){.x_mm = -half_width, .y_mm = -half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_REAR_LEFT] = (vec3f_t){.x_mm = half_width, .y_mm = -half_length, .z_mm = 0.0f};
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
            .z_mm = feet_world[leg].z_mm - stand_height_mm - hip_offsets[leg].z_mm,
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

    float planar_vertical = -sqrtf(fmaxf(radial * radial - mech->shoulder_length_mm * mech->shoulder_length_mm, 0.0f));
    float shoulder_deg = rad2deg(atan2f(vertical, lateral) - atan2f(planar_vertical, mech->shoulder_length_mm));
    float distance = sqrtf(forward * forward + planar_vertical * planar_vertical);
    if (distance < 1.0f || distance > (mech->thigh_length_mm + mech->calf_length_mm)) {
        return false;
    }

    float knee_cos = clampf((distance * distance - mech->thigh_length_mm * mech->thigh_length_mm -
                             mech->calf_length_mm * mech->calf_length_mm) /
                                (2.0f * mech->thigh_length_mm * mech->calf_length_mm),
                            -1.0f, 1.0f);
    float knee_deg = rad2deg(acosf(knee_cos));

    float thigh_math_deg = rad2deg(atan2f(forward, -planar_vertical) -
                                   atan2f(mech->calf_length_mm * sinf(deg2rad(knee_deg)),
                                          mech->thigh_length_mm + mech->calf_length_mm * cosf(deg2rad(knee_deg))));
    float thigh_servo_deg = 90.0f - thigh_math_deg;
    float calf_servo_deg = thigh_servo_deg +
                           (float)leg_bindings[leg].knee_coupling_sign * knee_deg +
                           leg_bindings[leg].knee_coupling_offset_deg;

    out_pose->shoulder_deg = 90.0f - (float)leg_bindings[leg].shoulder_sign * shoulder_deg;
    out_pose->thigh_servo_deg = thigh_servo_deg;
    out_pose->calf_servo_deg = calf_servo_deg;
    out_pose->knee_deg = knee_deg;
    return true;
}

bool kinematics_forward_leg(scargo_leg_id_t leg, const leg_servo_pose_t *pose, vec3f_t *out_foot_body)
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    const scargo_leg_kinematics_binding_t *leg_bindings = board_defaults_leg_kinematics();
    if (pose == NULL || out_foot_body == NULL) {
        return false;
    }

    const float shoulder_deg = -(float)leg_bindings[leg].shoulder_sign * (pose->shoulder_deg - 90.0f);
    const float thigh_math_deg = 90.0f - pose->thigh_servo_deg;
    const float knee_deg =
        (float)leg_bindings[leg].knee_coupling_sign *
        (pose->calf_servo_deg - pose->thigh_servo_deg - leg_bindings[leg].knee_coupling_offset_deg);

    const float shoulder = deg2rad(shoulder_deg);
    const float thigh = deg2rad(thigh_math_deg);
    const float knee = deg2rad(knee_deg);

    const float plane_y = mech->thigh_length_mm * sinf(thigh) +
                          mech->calf_length_mm * sinf(thigh + knee);
    const float plane_z = -(mech->thigh_length_mm * cosf(thigh) +
                            mech->calf_length_mm * cosf(thigh + knee));

    const float lateral = mech->shoulder_length_mm * cosf(shoulder) + plane_z * sinf(shoulder);
    const float vertical = -mech->shoulder_length_mm * sinf(shoulder) + plane_z * cosf(shoulder);
    const float side_sign = SCARGO_BODY_SIDE_SIGN(leg);

    out_foot_body->x_mm = side_sign * lateral;
    out_foot_body->y_mm = plane_y;
    out_foot_body->z_mm = vertical;
    return true;
}
