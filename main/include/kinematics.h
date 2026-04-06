#pragma once

#include <stdbool.h>

#include "board_defaults.h"

/*
 * ScarGo 运动学三层约定
 * ====================
 *
 * 1. 纯几何层（Geometry Layer）
 *    - 只讨论单腿几何关系
 *    - 输入：足端坐标、机身姿态、连杆长度
 *    - 输出：shoulder / alpha / beta
 *    - 不包含：SERVO_MAP、标定偏置、舵机镜像、PCA9685 通道
 *
 * 2. 真实执行映射层（Mechanical / Servo Layer）
 *    - 把几何关节角翻译成真实舵机角
 *    - 这里才允许出现：
 *      - 安装中位 90 度
 *      - 左右/前后镜像
 *      - beta_sign / beta_offset
 *      - 标定偏置
 *      - 真实小腿执行修正
 *
 * 3. 预览显示层（Preview / Display Layer）
 *    - OLED / Web 预览只应使用“纯几何层”的结果来画图
 *    - 不能把执行层的 sign / offset / calibration 带进显示语义
 *
 * 当前头文件里的类型也按这个分层来理解：
 * - leg_joint_pose_t : 纯几何层关节量
 * - leg_servo_pose_t : 执行层舵机量
 */

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
} vec3f_t;

// 机身姿态定义：
// roll  : 绕机身 Y 轴转动
// pitch : 绕机身 X 轴转动
// yaw   : 绕机身 Z 轴转动
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} body_pose_t;

// 舵机层姿态。
// 这里保存的是“要发给舵机的角度”，也就是机械安装中位 90 度制下的角度。
typedef struct {
    float shoulder_deg;
    float thigh_servo_deg;
    float calf_servo_deg;
    /*
     * beta_deg 仅用于调试观察。
     *
     * 真实执行层最终只需要 3 个舵机角，但调试时经常需要同时对照：
     * - 大腿当前语义角 alpha
     * - 小腿当前语义角 beta
     *
     * 因此这里把 beta 作为辅助字段挂在舵机姿态结构上。
     * 这不会改变执行逻辑，只是为了让串口和调试页面更容易对照。
     */
    float beta_deg;
} leg_servo_pose_t;

// 关节层姿态。
// 这里保存的是“机械关节角”，已经脱离了舵机安装中位、方向镜像和差动偏置。
typedef struct {
    float shoulder_deg;
    float thigh_deg;
    // beta_deg 表示“大腿与小腿之间的大夹角 beta”。
    // 统一采用 alpha/beta 语义后，这里不再使用旧的“膝角”叫法。
    float beta_deg;
} leg_joint_pose_t;

// 生成标准站立脚点。
// feet_body 其实是机身坐标系下的四个默认足端点，供 stand / walk 的目标初始化使用。
void kinematics_default_feet(vec3f_t feet_body[SCARGO_LEG_COUNT], float stand_height_mm);

// 生成趴下/收腿状态使用的脚点。
// 当前实现中，x/y 与标准站立共用，只通过高度变化产生主要的 z 方向动作。
void kinematics_rest_feet(vec3f_t feet_body[SCARGO_LEG_COUNT], float reference_height_mm);

// 将“世界坐标系中的足端点”转换为“单腿坐标系/机身局部坐标中的足端点”。
// 后续所有单腿逆解都基于 feet_body 来完成。
void kinematics_apply_body_pose(vec3f_t feet_body[SCARGO_LEG_COUNT], const vec3f_t feet_world[SCARGO_LEG_COUNT],
                                float stand_height_mm, const body_pose_t *pose);

// 舵机角 -> 机械关节角。
// 这一步负责处理：
// 1. 肩膀左右镜像方向
// 2. 大腿舵机中位 90 度到机械角的换算
// 3. 小腿差动机构中，舵机角到 beta 大夹角的转换
bool kinematics_servo_to_joint(scargo_leg_id_t leg, const leg_servo_pose_t *servo_pose, leg_joint_pose_t *joint_pose);

// 机械关节角 -> 舵机角。
// 与 kinematics_servo_to_joint() 成对，供逆解完成后把 joint pose 再翻译回舵机层。
bool kinematics_joint_to_servo(scargo_leg_id_t leg, const leg_joint_pose_t *joint_pose, leg_servo_pose_t *servo_pose);

// 仅做几何层求解：从足端点直接求出 shoulder / thigh / beta，不翻译回舵机角。
bool kinematics_solve_leg_geometry(scargo_leg_id_t leg, const vec3f_t *foot_body, leg_joint_pose_t *out_joint_pose);

// 单腿逆解：输入一条腿在机身局部坐标中的足端点，输出三个舵机应达到的目标角。
// 这里的大腿/小腿关系统一使用 alpha/beta 语义。
bool kinematics_solve_leg(scargo_leg_id_t leg, const vec3f_t *foot_body, leg_servo_pose_t *out_pose);

// 单腿正解：输入三个舵机角，输出足端在机身局部坐标中的位置。
bool kinematics_forward_leg(scargo_leg_id_t leg, const leg_servo_pose_t *pose, vec3f_t *out_foot_body);

// 统一腿链正解接口。
// out_points:
//   0 = 肩膀根部
//   1 = 肩膀末端
//   2 = 膝部
//   3 = 足端
// OLED 单腿页、整机预览页都应复用这套链点，避免出现多套显示算法。
bool kinematics_compute_leg_chain(scargo_leg_id_t leg, const leg_servo_pose_t *pose, vec3f_t out_points[4]);

// 直接根据几何关节角生成链点，不经过舵机层符号/偏置。
bool kinematics_compute_leg_chain_from_joint(scargo_leg_id_t leg, const leg_joint_pose_t *joint_pose, vec3f_t out_points[4]);

/*
 * 纯几何预览接口
 * ---------------
 *
 * 这组接口只给 OLED / Web 预览使用。
 *
 * 设计原则：
 * - 预览必须表达“几何本质层”
 * - 预览不能混入执行层的 sign / offset / calibration
 * - 预览看到的腿形，应该回答：
 *     “从当前足端目标反推，几何上这条腿应该是什么姿态”
 *
 * 因此这里：
 * - 不考虑舵机安装镜像
 * - 不考虑执行层偏置
 * - 只按单腿几何本质求 shoulder / alpha / beta
 */
bool kinematics_solve_leg_preview_geometry(scargo_leg_id_t leg, const vec3f_t *foot_body, leg_joint_pose_t *out_joint_pose);

// 纯几何预览链点。
// 输入 shoulder / alpha / beta，直接生成肩膀末端、膝点、足端几何链。
bool kinematics_compute_leg_preview_chain_from_joint(scargo_leg_id_t leg, const leg_joint_pose_t *joint_pose, vec3f_t out_points[4]);
