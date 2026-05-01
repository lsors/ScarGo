#include "kinematics.h"

#include <math.h>

/*
 * 本文件当前承载四层中的前三层：
 *
 * 1. 纯几何层
 *    - kinematics_solve_leg_geometry()
 *    - kinematics_compute_leg_chain_from_joint()
 *
 * 2. 安装层
 *    - kinematics_solve_leg_installation_pose()
 *    - kinematics_compute_leg_chain_from_installation_pose()
 *
 * 3. 舵机层前的映射辅助
 *    - kinematics_servo_to_joint()
 *    - kinematics_joint_to_servo()
 *    - kinematics_solve_leg()
 *    - kinematics_forward_leg()
 *    - kinematics_compute_leg_chain()
 *
 * display_service 不应该自己再维护一套几何公式；
 * 它只应通过安装层接口来拿腿形。
 */

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

/*
 * 按当前新的层次定义：
 * - 纯几何层里的 beta：保持标准二连杆数学语义
 * - 安装层里的 beta：只额外体现“小腿零位是与大腿垂直并向前伸”
 *
 * 因此安装层并不重定义大腿 alpha，只对小腿 beta 的零位语义做一次转换。
 * 两者互为补角：
 *   beta_install = 180 - beta_geometry
 */
static float geometry_beta_to_installation_beta(float beta_geometry_deg)
{
    return 180.0f - beta_geometry_deg;
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

// 生成标准站立脚点。
// 足端在髋关节正下方基础上向外展开半个肩膀长度，增大支撑多边形面积。
void kinematics_default_feet(vec3f_t feet_body[SCARGO_LEG_COUNT], float stand_height_mm)
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    const float half_width = mech->body_width_mm * 0.5f;
    const float half_length = mech->body_length_mm * 0.5f;
    const float spread = mech->shoulder_length_mm * SCARGO_FOOT_LATERAL_SPREAD_RATIO;
    (void)stand_height_mm;

    feet_body[SCARGO_LEG_FRONT_RIGHT] = (vec3f_t){.x_mm = -(half_width + spread), .y_mm = half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_FRONT_LEFT]  = (vec3f_t){.x_mm =  (half_width + spread), .y_mm = half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_REAR_RIGHT]  = (vec3f_t){.x_mm = -(half_width + spread), .y_mm = -half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_REAR_LEFT]   = (vec3f_t){.x_mm =  (half_width + spread), .y_mm = -half_length, .z_mm = 0.0f};
}

// 生成趴下/收腿时使用的默认脚点。
// 当前实现仍沿用“足端在髋关节正下方”的思路，避免在 stand/lie 切换时出现额外的 y 漂移。
void kinematics_rest_feet(vec3f_t feet_body[SCARGO_LEG_COUNT], float reference_height_mm)
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    const float half_width = mech->body_width_mm * 0.5f;
    const float half_length = mech->body_length_mm * 0.5f;
    const float spread = mech->shoulder_length_mm * SCARGO_FOOT_LATERAL_SPREAD_RATIO;
    (void)reference_height_mm;

    feet_body[SCARGO_LEG_FRONT_RIGHT] = (vec3f_t){.x_mm = -(half_width + spread), .y_mm = half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_FRONT_LEFT]  = (vec3f_t){.x_mm =  (half_width + spread), .y_mm = half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_REAR_RIGHT]  = (vec3f_t){.x_mm = -(half_width + spread), .y_mm = -half_length, .z_mm = 0.0f};
    feet_body[SCARGO_LEG_REAR_LEFT]   = (vec3f_t){.x_mm =  (half_width + spread), .y_mm = -half_length, .z_mm = 0.0f};
}

// 世界坐标 → 身体中心坐标（第一步：纯平移）。
//
// 身体中心在世界坐标中的位置：
//   (pose->offset_x_mm, pose->offset_y_mm, stand_height_mm)
//
// 输出：足端相对身体中心的向量，轴方向仍与世界对齐（未施加任何旋转）。
void kinematics_world_to_body(vec3f_t feet_body_center[SCARGO_LEG_COUNT],
                               const vec3f_t feet_world[SCARGO_LEG_COUNT],
                               float stand_height_mm, const body_pose_t *pose)
{
    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        feet_body_center[leg] = (vec3f_t){
            .x_mm = feet_world[leg].x_mm - pose->offset_x_mm,
            .y_mm = feet_world[leg].y_mm - pose->offset_y_mm,
            .z_mm = feet_world[leg].z_mm - stand_height_mm,
        };
    }
}

// 身体中心坐标 → 腿坐标（第二步：逆旋转 + 减髋关节偏移）。
//
// 旋转顺序（逆向）：先逆 yaw，再逆 pitch（X轴），再逆 roll（Y轴）。
// 这与 body_foot_to_world() 的正向顺序（roll(Y)→pitch(X)→yaw(Z)）互为逆变换。
void kinematics_body_to_leg(vec3f_t feet_leg[SCARGO_LEG_COUNT],
                             const vec3f_t feet_body_center[SCARGO_LEG_COUNT],
                             const body_pose_t *pose)
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

    for (int leg = 0; leg < SCARGO_LEG_COUNT; ++leg) {
        vec3f_t p = rotate_z(feet_body_center[leg], -deg2rad(pose->yaw_deg));
        p = rotate_x(p, -deg2rad(pose->pitch_deg));
        p = rotate_y(p, -deg2rad(pose->roll_deg));
        feet_leg[leg] = (vec3f_t){
            .x_mm = p.x_mm - hip_offsets[leg].x_mm,
            .y_mm = p.y_mm - hip_offsets[leg].y_mm,
            .z_mm = p.z_mm - hip_offsets[leg].z_mm,
        };
    }
}

// 便捷接口：世界坐标一步到腿坐标。
void kinematics_apply_body_pose(vec3f_t feet_leg[SCARGO_LEG_COUNT], const vec3f_t feet_world[SCARGO_LEG_COUNT],
                                float stand_height_mm, const body_pose_t *pose)
{
    vec3f_t feet_body_center[SCARGO_LEG_COUNT];
    kinematics_world_to_body(feet_body_center, feet_world, stand_height_mm, pose);
    kinematics_body_to_leg(feet_leg, feet_body_center, pose);
}

// 舵机角 -> 安装层关节角。
//
// 这里是“舵机层”回到“安装层”的语义翻译：
// - shoulder_deg : 当前安装方式下的肩膀关节角
// - thigh_deg    : 当前安装方式下的大腿关节角
// - beta_deg     : 当前安装方式下的大夹角 beta
//
// 统一后的换算关系是：
//   beta = beta_sign * (calf_servo - thigh_servo - beta_offset)
//
// 四条腿共用同一套 alpha/beta 语言体系，
// 具体差异只通过安装层参数表来区分。
bool kinematics_servo_to_joint(scargo_leg_id_t leg, const leg_servo_pose_t *servo_pose, leg_joint_pose_t *joint_pose)
{
    const scargo_leg_installation_binding_t *leg_bindings = board_defaults_leg_installation();
    if (servo_pose == NULL || joint_pose == NULL) {
        return false;
    }

    joint_pose->shoulder_deg = -(float)leg_bindings[leg].shoulder_sign * (servo_pose->shoulder_deg - 90.0f);
    joint_pose->thigh_deg = 90.0f - servo_pose->thigh_servo_deg;
    joint_pose->beta_deg =
        (float)leg_bindings[leg].beta_sign *
        (servo_pose->calf_servo_deg - servo_pose->thigh_servo_deg - leg_bindings[leg].beta_offset_deg);
    return true;
}

// 安装层关节角 -> 舵机角。
// 和 kinematics_servo_to_joint() 正好相反，逆解完成后最终会走这里。
bool kinematics_joint_to_servo(scargo_leg_id_t leg, const leg_joint_pose_t *joint_pose, leg_servo_pose_t *servo_pose)
{
    const scargo_leg_installation_binding_t *leg_bindings = board_defaults_leg_installation();
    if (joint_pose == NULL || servo_pose == NULL) {
        return false;
    }

    servo_pose->shoulder_deg = 90.0f - (float)leg_bindings[leg].shoulder_sign * joint_pose->shoulder_deg;
    servo_pose->thigh_servo_deg = 90.0f - joint_pose->thigh_deg;
    servo_pose->calf_servo_deg = servo_pose->thigh_servo_deg +
                                 (float)leg_bindings[leg].beta_sign * joint_pose->beta_deg +
                                 leg_bindings[leg].beta_offset_deg;
    servo_pose->beta_deg = joint_pose->beta_deg;
    return true;
}

// 单腿逆解。
//
// 输入：
// - foot_body : 足端在机身局部坐标中的位置
//
// 输出：
// - 三个舵机目标角
//
// 算法步骤：
// 1. 先由 x/z 求肩膀角，让腿进入对应的 y-z 腿平面
// 2. 再把问题退化成 y-z 平面中的标准二连杆逆解
// 3. 求出大腿角和夹角量，再翻译回舵机角
bool kinematics_solve_leg_geometry(scargo_leg_id_t leg, const vec3f_t *foot_body, leg_joint_pose_t *out_pose)
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    if (out_pose == NULL) {
        return false;
    }

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

    /*
     * 这里直接求几何上的 beta 大夹角。
     *
     * 注意：
     * - 这里的 beta 属于“纯几何本质层”
     * - 它不考虑安装中位、不考虑镜像、不考虑小腿真实执行修正
     * - 后面如果要真正驱动舵机，必须再经过 joint_to_servo()
     */
    float beta_cos = clampf((distance * distance - mech->thigh_length_mm * mech->thigh_length_mm -
                             mech->calf_length_mm * mech->calf_length_mm) /
                                (2.0f * mech->thigh_length_mm * mech->calf_length_mm),
                            -1.0f, 1.0f);
    float beta_deg = rad2deg(acosf(beta_cos));

    out_pose->shoulder_deg = shoulder_deg;
    out_pose->thigh_deg = rad2deg(atan2f(forward, -planar_vertical) -
                                  atan2f(mech->calf_length_mm * sinf(deg2rad(beta_deg)),
                                         mech->thigh_length_mm + mech->calf_length_mm * cosf(deg2rad(beta_deg))));
    out_pose->beta_deg = beta_deg;
    return true;
}

bool kinematics_solve_leg(scargo_leg_id_t leg, const vec3f_t *foot_body, leg_servo_pose_t *out_pose)
{
    leg_joint_pose_t joint_pose;
    if (out_pose == NULL) {
        return false;
    }
    if (!kinematics_solve_leg_geometry(leg, foot_body, &joint_pose)) {
        return false;
    }
    return kinematics_joint_to_servo(leg, &joint_pose, out_pose);
}

bool kinematics_forward_leg(scargo_leg_id_t leg, const leg_servo_pose_t *pose, vec3f_t *out_foot_body)
{
    vec3f_t points[4];
    if (out_foot_body == NULL) {
        return false;
    }

    if (!kinematics_compute_leg_chain(leg, pose, points)) {
        return false;
    }

    *out_foot_body = points[3];
    return true;
}

// 统一腿链正解。
//
// 这条函数是“显示层”和“调试层”最应该依赖的接口。
// 任何需要画出一条腿形状的地方，都应该直接用这里给出的四个链点：
//
//   点0  肩膀根部
//   点1  肩膀末端
//   点2  膝部
//   点3  足端
//
// 这样可以避免 OLED 单腿页、整机页、串口调试各写一套几何公式。
bool kinematics_compute_leg_chain_from_joint(scargo_leg_id_t leg, const leg_joint_pose_t *joint_pose, vec3f_t out_points[4])
{
    const scargo_mechanics_t *mech = board_defaults_mechanics();
    if (joint_pose == NULL || out_points == NULL) {
        return false;
    }

    /*
     * 这条函数是“关节层 -> 3D 链点”的统一实现。
     *
     * 它既可以被真实执行调试使用，也可以被预览层复用；
     * 但前提是调用者必须清楚传进来的 joint_pose 属于哪一层语义。
     *
     * 这里保留的定义只有：
     * - shoulder : 肩膀几何角
     * - alpha    : 大腿几何角
     * - beta     : 大腿延长线和小腿的夹角
     *
     * 不再引入 calf/shank/knee 之类额外语义。
     */
    const float shoulder = deg2rad(-joint_pose->shoulder_deg);
    const float alpha = deg2rad(joint_pose->thigh_deg);
    const float beta = deg2rad(joint_pose->beta_deg);

    // 在 y-z 腿平面中先求出膝部和足端位置，再由肩膀角把这条平面腿旋转到真实 3D 空间。
    const float plane_y = mech->thigh_length_mm * sinf(alpha) +
                          mech->calf_length_mm * sinf(alpha - beta);
    const float plane_z = -(mech->thigh_length_mm * cosf(alpha) +
                            mech->calf_length_mm * cosf(alpha - beta));

    const float side_sign = SCARGO_BODY_SIDE_SIGN(leg);
    const float shoulder_x = side_sign * mech->shoulder_length_mm * cosf(shoulder);
    const float shoulder_z = -mech->shoulder_length_mm * sinf(shoulder);
    const float thigh_end_x = side_sign * (mech->shoulder_length_mm * cosf(shoulder) + plane_z * sinf(shoulder));
    const float thigh_end_z = -mech->shoulder_length_mm * sinf(shoulder) + plane_z * cosf(shoulder);

    out_points[0] = (vec3f_t){.x_mm = 0.0f, .y_mm = 0.0f, .z_mm = 0.0f};
    out_points[1] = (vec3f_t){.x_mm = shoulder_x, .y_mm = 0.0f, .z_mm = shoulder_z};
    out_points[2] = (vec3f_t){.x_mm = thigh_end_x, .y_mm = mech->thigh_length_mm * sinf(alpha), .z_mm = thigh_end_z};
    out_points[3] = (vec3f_t){
        .x_mm = side_sign * (mech->shoulder_length_mm * cosf(shoulder) + plane_z * sinf(shoulder)),
        .y_mm = plane_y,
        .z_mm = -mech->shoulder_length_mm * sinf(shoulder) + plane_z * cosf(shoulder),
    };
    return true;
}

bool kinematics_compute_leg_chain(scargo_leg_id_t leg, const leg_servo_pose_t *pose, vec3f_t out_points[4])
{
    leg_joint_pose_t joint_pose;
    if (pose == NULL || out_points == NULL) {
        return false;
    }
    if (!kinematics_servo_to_joint(leg, pose, &joint_pose)) {
        return false;
    }
    return kinematics_compute_leg_chain_from_joint(leg, &joint_pose, out_points);
}

bool kinematics_solve_leg_installation_pose(scargo_leg_id_t leg, const vec3f_t *foot_body, leg_joint_pose_t *out_pose)
{
    leg_joint_pose_t geometry_pose;

    if (out_pose == NULL || foot_body == NULL) {
        return false;
    }

    /*
     * 安装层不再自己维护第二套逆解。
     *
     * 先统一走纯几何层求出：
     * - shoulder
     * - alpha
     * - beta_geometry
     *
     * 然后只对小腿 beta 做一次安装零位语义转换：
     * - 纯几何层：标准二连杆 beta
     * - 安装层  ：小腿与大腿垂直前伸为中位参考
     *
     * 这样“安装层只额外处理小腿零位”的边界就明确了，
     * 同时也避免几何层/安装层各写一套近似逆解，后面再慢慢漂移。
     */
    if (!kinematics_solve_leg_geometry(leg, foot_body, &geometry_pose)) {
        return false;
    }

    *out_pose = geometry_pose;
    out_pose->beta_deg = geometry_beta_to_installation_beta(geometry_pose.beta_deg);
    return true;
}

bool kinematics_compute_leg_chain_from_installation_pose(scargo_leg_id_t leg, const leg_joint_pose_t *joint_pose,
                                                         vec3f_t out_points[4])
{
    if (joint_pose == NULL || out_points == NULL) {
        return false;
    }

    const scargo_mechanics_t *mech = board_defaults_mechanics();
    const float side_sign = SCARGO_BODY_SIDE_SIGN(leg);
    const float shoulder = deg2rad(-joint_pose->shoulder_deg);
    const float alpha = deg2rad(joint_pose->thigh_deg);
    const float beta_install = deg2rad(joint_pose->beta_deg);
    const float knee_y = mech->thigh_length_mm * sinf(alpha);
    const float knee_z = -mech->thigh_length_mm * cosf(alpha);
    float calf_dir;
    float foot_y;
    float foot_z;
    float shoulder_x;
    float shoulder_z;
    float knee_x;
    float knee_world_z;
    float foot_x;
    float foot_world_z;

    /*
     * 这里不能简单把安装层 beta 再转换回几何层 beta，
     * 然后直接复用 kinematics_compute_leg_chain_from_joint()。
     *
     * 原因是安装层 beta 的语义是：
     * - 小腿零位与大腿垂直并向前伸
     *
     * 在这个语义下，小腿绝对方向应当按：
     *   calf_dir = alpha + PI - beta_install
     *
     * 这正是之前预览链里已经被验证过的正确构造。
     * 因此安装层链点仍然需要保留这一条专属的小腿方向公式；
     * 它只是在“链点构造”这一步专属，并不意味着安装层重新维护一套逆解。
     */
    calf_dir = alpha + (float)M_PI - beta_install;
    foot_y = knee_y + mech->calf_length_mm * sinf(calf_dir);
    foot_z = knee_z - mech->calf_length_mm * cosf(calf_dir);

    shoulder_x = side_sign * mech->shoulder_length_mm * cosf(shoulder);
    shoulder_z = -mech->shoulder_length_mm * sinf(shoulder);
    knee_x = side_sign * (mech->shoulder_length_mm * cosf(shoulder) + knee_z * sinf(shoulder));
    knee_world_z = -mech->shoulder_length_mm * sinf(shoulder) + knee_z * cosf(shoulder);
    foot_x = side_sign * (mech->shoulder_length_mm * cosf(shoulder) + foot_z * sinf(shoulder));
    foot_world_z = -mech->shoulder_length_mm * sinf(shoulder) + foot_z * cosf(shoulder);

    out_points[0] = (vec3f_t){.x_mm = 0.0f, .y_mm = 0.0f, .z_mm = 0.0f};
    out_points[1] = (vec3f_t){.x_mm = shoulder_x, .y_mm = 0.0f, .z_mm = shoulder_z};
    out_points[2] = (vec3f_t){.x_mm = knee_x, .y_mm = knee_y, .z_mm = knee_world_z};
    out_points[3] = (vec3f_t){.x_mm = foot_x, .y_mm = foot_y, .z_mm = foot_world_z};
    return true;
}
