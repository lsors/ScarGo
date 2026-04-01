#pragma once

#include <stdbool.h>

#include "board_defaults.h"

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
} vec3f_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} body_pose_t;

typedef struct {
    float shoulder_deg;
    float thigh_servo_deg;
    float calf_servo_deg;
    float knee_deg;
} leg_servo_pose_t;

void kinematics_default_feet(vec3f_t feet_body[SCARGO_LEG_COUNT], float stand_height_mm);
void kinematics_rest_feet(vec3f_t feet_body[SCARGO_LEG_COUNT], float reference_height_mm);
void kinematics_apply_body_pose(vec3f_t feet_body[SCARGO_LEG_COUNT], const vec3f_t feet_world[SCARGO_LEG_COUNT],
                                float stand_height_mm, const body_pose_t *pose);
bool kinematics_solve_leg(scargo_leg_id_t leg, const vec3f_t *foot_body, leg_servo_pose_t *out_pose);
bool kinematics_forward_leg(scargo_leg_id_t leg, const leg_servo_pose_t *pose, vec3f_t *out_foot_body);
