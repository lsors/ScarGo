#pragma once

#include <stdint.h>

// ── 硬件拓扑 ──────────────────────────────────────────────────────────────────
#define SCARGO_SERVO_COUNT      12   // 全身舵机总数（4腿 × 3关节）
#define SCARGO_LEG_COUNT         4   // 腿数
#define SCARGO_JOINTS_PER_LEG    3   // 每腿关节数：肩膀 / 大腿 / 小腿

// ── 任务循环频率 ──────────────────────────────────────────────────────────────
#define SCARGO_CONTROL_RATE_HZ 100   // 主控制循环频率（Hz）
#define SCARGO_IMU_RATE_HZ     100   // IMU 读取与姿态滤波频率（Hz）
#define SCARGO_SERVO_RATE_HZ   100   // 舵机输出刷新频率（Hz），与 PCA9685 帧率对应

// ── 站立高度 ──────────────────────────────────────────────────────────────────
// 高度定义为足端到身体中心的垂直距离（mm）
#define SCARGO_DEFAULT_STAND_HEIGHT_MM 140.0f   // 默认站立高度
#define SCARGO_MIN_HEIGHT_MM            80.0f   // 允许的最低站立高度（过低会碰地）
#define SCARGO_MAX_HEIGHT_MM           170.0f   // 允许的最高站立高度（过高腿伸直极限）

// ── 足端横向展开 ──────────────────────────────────────────────────────────────
// 1.0 = 标准宽度（由机身宽度 + 肩长决定），调大会让腿更外八
#define SCARGO_FOOT_LATERAL_SPREAD_RATIO 1.0f

// ── 抬脚高度 ──────────────────────────────────────────────────────────────────
#define SCARGO_DEFAULT_STEP_HEIGHT_MM 40.0f   // 默认摆动相抬脚高度
#define SCARGO_MIN_STEP_HEIGHT_MM     20.0f   // 最小抬脚高度（地形平坦时可降低）
#define SCARGO_MAX_STEP_HEIGHT_MM     80.0f   // 最大抬脚高度（越野/越障时调高）

// ── 步态周期 ──────────────────────────────────────────────────────────────────
// 单步周期越短越快，但舵机响应和机械惯性有极限
#define SCARGO_DEFAULT_STEP_CYCLE_MS  600   // 默认单步周期（ms）
#define SCARGO_MIN_STEP_CYCLE_MS      350   // 最快步频对应周期（约 2.8 步/秒）
#define SCARGO_MAX_STEP_CYCLE_MS     3000   // 最慢步频对应周期（约 0.33 步/秒）

// ── 步幅缩放 ──────────────────────────────────────────────────────────────────
// 对步长整体缩放，1.0 = 满幅，0.5 = 半幅
#define SCARGO_DEFAULT_STEP_SCALE 1.0f
#define SCARGO_MIN_STEP_SCALE     0.5f
#define SCARGO_MAX_STEP_SCALE     1.0f

// ── 支撑相比例 ────────────────────────────────────────────────────────────────
// stance_ratio = 一个步态周期中"脚在地上"的时间占比
// 对角步态下最小为 0.5（两腿同时支撑），越大越稳但步速越慢
// 波形步态下最高可达 0.75（摆动相 0.25，4腿各占 0.25，任意时刻最多1腿摆动）
#define SCARGO_DEFAULT_STANCE_RATIO 0.5f
#define SCARGO_MIN_STANCE_RATIO     0.5f
#define SCARGO_MAX_STANCE_RATIO     0.75f
#define SCARGO_WAVE_GAIT_STANCE_RATIO 0.75f
// 波形步态抬腿时的机身横向平移量（mm）。
// 某条腿摆动时，机身向对侧平移，使重心回到三角支撑面内。
// 值由足端横向半宽 W = body_width/2 + shoulder = 85mm 推导：
//   三角形重心偏移 = W/3 ≈ 28mm
#define SCARGO_WAVE_SHIFT_X_MM 0.0f//不要平移了，感觉不太对劲，先关掉

// ── 步态模式 ──────────────────────────────────────────────────────────────────
// 行走模式支持两种步态
typedef enum {
    SCARGO_GAIT_MODE_DIAGONAL = 0,  // 对角步态：两腿同时摆动
    SCARGO_GAIT_MODE_WAVE = 1,      // 波形步态：4腿连续摆动，交叉对顺序（3腿支撑+1腿摆动）
} scargo_gait_mode_t;

// ── 步态相位 / 机身姿态极限 ───────────────────────────────────────────────────
// diagonal_phase_offset: 对角步态中后对角腿相对前对角腿的相位偏移（0 = 标准同步）
#define SCARGO_DEFAULT_PHASE_OFFSET  0.0f
// 站立模式遥控可调范围（度）
#define SCARGO_MAX_BODY_YAW_DEG          45.0f   // 机身绕 Z 轴最大偏航（最高高度时，左摇杆左右）
#define SCARGO_MAX_BODY_YAW_DEG_LOW      15.0f   // 机身最低高度时的最大偏航（防止低姿态下舵机干涉堵转）
#define SCARGO_MAX_BODY_ROLL_DEG         20.0f   // 机身绕 Y 轴最大横滚（右摇杆左右）
#define SCARGO_MAX_BODY_PITCH_DEG        20.0f   // 机身绕 X 轴最大俯仰（右摇杆上下）

// ── 系统配置 ──────────────────────────────────────────────────────────────────
#define SCARGO_CPU_FREQ_MHZ        240              // ESP32 CPU 主频（MHz）
#define SCARGO_WIFI_AP_SSID        "ScarGo-Calib"  // 标定模式 WiFi AP 名称
#define SCARGO_WIFI_AP_PASSWORD    "scargo123"      // 标定模式 WiFi AP 密码

// ── CRSF 遥控协议 ─────────────────────────────────────────────────────────────
#define SCARGO_CRSF_UART_BAUD    420000   // CRSF 串口波特率（ELRS 协议固定值）
#define SCARGO_CRSF_MIN_VALUE       172   // 通道原始最小值（对应摇杆/拨档最低位）
#define SCARGO_CRSF_MAX_VALUE      1811   // 通道原始最大值（对应摇杆/拨档最高位）
#define SCARGO_CRSF_MID_VALUE       992   // 通道原始中位值（对应摇杆中立）
#define SCARGO_CRSF_SWITCH_LOW      400   // 拨档通道判定为"低档"的上限阈值
#define SCARGO_CRSF_SWITCH_HIGH    1600   // 拨档通道判定为"高档"的下限阈值

// ── I2C 设备地址 ──────────────────────────────────────────────────────────────
#define SCARGO_MPU6050_ADDR     0x68   // MPU6050 默认地址（AD0 引脚接低）
#define SCARGO_MPU6050_ADDR_ALT 0x69   // MPU6050 备用地址（AD0 引脚接高）
#define SCARGO_SSD1306_ADDR     0x3C   // SSD1306 OLED 默认地址
#define SCARGO_SSD1306_ADDR_ALT 0x3D   // SSD1306 OLED 备用地址

// ── 解锁通道 ──────────────────────────────────────────────────────────────────
// 同时满足这 4 个通道的特定组合才触发机器人解锁（CRSF 通道从 0 起编号）
#define SCARGO_ARM_CHANNEL_0  3   // 解锁判断通道 0（对应遥控器 CH4）
#define SCARGO_ARM_CHANNEL_1  4   // 解锁判断通道 1（对应遥控器 CH5）
#define SCARGO_ARM_CHANNEL_2 11   // 解锁判断通道 2（对应遥控器 CH12）
#define SCARGO_ARM_CHANNEL_3 12   // 解锁判断通道 3（对应遥控器 CH13）

// ── 摇杆死区 ──────────────────────────────────────────────────────────────────
// 归一化值（-1~1），绝对值小于该值时视为零输入，防止摇杆漂移
#define SCARGO_RC_DEADZONE 0.04f

// ── 跳跃控制参数 ──────────────────────────────────────────────────────────────
#define SCARGO_JUMP_CROUCH_SPEED_MM_S       200.0f  // 蹲下速度 (mm/s)
#define SCARGO_JUMP_CROUCH_HOLD_TICKS        20     // 蹲到位后保持帧数（100Hz → 200ms）
#define SCARGO_JUMP_LAUNCH_SPEED_MM_S       600.0f  // 起跳伸腿速度 (mm/s)，全速冲
#define SCARGO_JUMP_LAUNCH_TICKS             12     // 起跳阶段帧数（100Hz → 120ms）
#define SCARGO_JUMP_AIRBORNE_TIMEOUT_TICKS  100     // 腾空等待超时保护（100Hz → 1s）
#define SCARGO_JUMP_LAND_ACCEL_G            1.8f    // 落地检测阈值：accel_z_g 超过此值视为着陆
#define SCARGO_JUMP_BUFFER_SPEED_MM_PER_G   100.0f  // 缓冲系数：每额外 1g 冲击对应的下压速度 (mm/s)
#define SCARGO_JUMP_BUFFER_SETTLE_G          0.2f   // 缓冲稳定阈值：excess_g 低于此值视为冲击消散
#define SCARGO_JUMP_BUFFER_SETTLE_TICKS       8     // 缓冲稳定所需连续帧数
#define SCARGO_JUMP_RECOVER_SPEED_MM_S       60.0f  // 恢复站立速度 (mm/s)
#define SCARGO_JUMP_BODY_LEAN_MM             15.0f  // 方向跳起跳阶段机身平移量（mm），产生朝向感
#define SCARGO_JUMP_STICK_THRESHOLD          0.95f  // 右摇杆单轴满杆阈值：任意轴超过此值触发方向跳

// ── 调试开关 ──────────────────────────────────────────────────────────────────
// 取消注释以开启站立模式姿态调试日志（每 200ms 一条，用于验证 RC/IMU/body_pose 符号一致性）
// #define SCARGO_POSE_DEBUG_LOG

// ── 平衡控制激活条件 ──────────────────────────────────────────────────────────
#define SCARGO_BALANCE_IDLE_S              2.0f   // 所有摇杆静止多少秒后才激活平衡控制
#define SCARGO_BALANCE_THROTTLE_THRESHOLD  0.02f  // 油门单帧变化量超过此值视为在打杆
#define SCARGO_BALANCE_DEADZONE_DEG        2.0f   // 姿态偏差小于此角度不做补偿（过滤微振）
// 姿态滑动平均窗口（ms）：用该时间内的均值作为修正目标，滤除高频抖动
// 越大越平滑但响应越慢；200ms = 20 个采样点（100Hz）
#define SCARGO_BALANCE_FILTER_MS  500
// 平衡修正量每帧最大变化量（度/帧，100Hz = 该值 × 100 °/s）
// 倾斜多少度目标就是多少度（1:1 映射），此速率控制逼近快慢
#define SCARGO_BALANCE_CORRECTION_RATE_DEG_PER_TICK  0.3f

// ── 平衡 PID 增益（行走模式）─────────────────────────────────────────────────
// 普通四脚站立模式使用 body_pose 姿态反解；这组 PID 当前保留给行走/过渡姿态补偿。
#define SCARGO_BALANCE_KP_ROLL   0.60f   // roll 轴比例增益（1.0=完全补偿，0.8=稳态残差44%）
#define SCARGO_BALANCE_KI_ROLL   0.00f   // roll 轴积分增益（消除静差，当前关闭）
#define SCARGO_BALANCE_KD_ROLL   0.00f   // roll 轴微分增益（D项放大IMU噪声，先关闭）
#define SCARGO_BALANCE_KP_PITCH  0.60f   // pitch 轴比例增益
#define SCARGO_BALANCE_KI_PITCH  0.00f   // pitch 轴积分增益（当前关闭）
#define SCARGO_BALANCE_KD_PITCH  0.00f   // pitch 轴微分增益（D项放大IMU噪声，先关闭）

// ── 对角腿站立平衡（平移补偿）────────────────────────────────────────────────
// 通过平移机身（offset_x/y）把 CoM 推回支撑对角线。
// 与旋转补偿不同，平移直接改变 CoM 水平位置，是两腿站立唯一有效的平衡手段。
#define SCARGO_DIAG_BALANCE_KP_MM_PER_DEG     2.0f   // P：每度倾斜对应平移量(mm)
#define SCARGO_DIAG_BALANCE_KD_MM_PER_DPS    0.20f  // D：每(度/秒)倾斜速率对应平移量(mm)，抑制倒扑
#define SCARGO_DIAG_BALANCE_MAX_OFFSET_MM    25.0f   // 最大平移量限制(mm)

// ── 腿侧 / 前后符号辅助宏 ────────────────────────────────────────────────────
// SIDE_SIGN : 左腿 = +1，右腿 = -1（用于步态中左右差动，如转弯步幅偏差）
// FRONT_SIGN: 前腿 = +1，后腿 = -1（用于前后差动场景）
#define SCARGO_BODY_SIDE_SIGN(leg)  (((leg) == SCARGO_LEG_FRONT_LEFT || (leg) == SCARGO_LEG_REAR_LEFT)  ? 1.0f : -1.0f)
#define SCARGO_BODY_FRONT_SIGN(leg) (((leg) == SCARGO_LEG_FRONT_LEFT || (leg) == SCARGO_LEG_FRONT_RIGHT) ? 1.0f : -1.0f)

// ── 腿编号 ────────────────────────────────────────────────────────────────────
typedef enum {
    SCARGO_LEG_FRONT_RIGHT = 0,   // 右前腿
    SCARGO_LEG_FRONT_LEFT  = 1,   // 左前腿
    SCARGO_LEG_REAR_RIGHT  = 2,   // 右后腿
    SCARGO_LEG_REAR_LEFT   = 3,   // 左后腿
} scargo_leg_id_t;

// ── 关节编号 ──────────────────────────────────────────────────────────────────
typedef enum {
    SCARGO_JOINT_SHOULDER = 0,   // 肩关节（横向摆动，控制外展/内收）
    SCARGO_JOINT_THIGH    = 1,   // 髋关节（前后摆动，控制大腿俯仰）
    SCARGO_JOINT_CALF     = 2,   // 膝关节（小腿伸屈，通过差动机构驱动）
} scargo_joint_id_t;

// ── GPIO 引脚映射 ─────────────────────────────────────────────────────────────
typedef struct {
    uint8_t pca9685_i2c_port;      // PCA9685 所在 I2C 总线编号（0 或 1）
    uint8_t mpu_oled_i2c_port;     // MPU6050 / SSD1306 所在 I2C 总线编号
    uint8_t pca9685_i2c_address;   // PCA9685 I2C 地址
    int i2c0_sda;    // I2C 总线 0 SDA 引脚
    int i2c0_scl;    // I2C 总线 0 SCL 引脚
    int i2c1_sda;    // I2C 总线 1 SDA 引脚
    int i2c1_scl;    // I2C 总线 1 SCL 引脚
    int uart_com1_rx;   // UART1 RX（通常接遥控接收机 CRSF TX）
    int uart_com1_tx;   // UART1 TX
    int uart_com2_rx;   // UART2 RX（备用串口）
    int uart_com2_tx;   // UART2 TX
    int fan_pwm;    // 散热风扇 PWM 输出引脚
    int fan_tach;   // 散热风扇转速反馈引脚（测速脉冲）
    int button_k1;  // 按键 K1 引脚
    int button_k2;  // 按键 K2 引脚
    int buzzer;     // 蜂鸣器输出引脚
} scargo_gpio_map_t;

// ── 蜂鸣器驱动模式 ────────────────────────────────────────────────────────────
typedef enum {
    SCARGO_BUZZER_MODE_GPIO = 0,   // 直接 GPIO 高低电平驱动（无源蜂鸣器）
    SCARGO_BUZZER_MODE_PWM  = 1,   // PWM 信号驱动（有源蜂鸣器或需要音调控制）
} scargo_buzzer_mode_t;

// ── 机身机械尺寸 ──────────────────────────────────────────────────────────────
// 所有尺寸单位：mm，用于逆运动学几何计算
typedef struct {
    float body_width_mm;      // 机身左右髋关节间距
    float body_length_mm;     // 机身前后髋关节间距
    float shoulder_length_mm; // 肩关节到大腿根部的连杆长度（横向外展臂）
    float thigh_length_mm;    // 大腿连杆长度（髋到膝）
    float calf_length_mm;     // 小腿连杆长度（膝到足端）
} scargo_mechanics_t;

// ── 舵机通道绑定 ──────────────────────────────────────────────────────────────
typedef struct {
    uint8_t pca9685_channel;   // 该关节对应的 PCA9685 输出通道号（0~15）
    int8_t  servo_sign;        // 舵机安装方向符号（+1 或 -1），修正镜像安装
} scargo_servo_binding_t;

// ── 腿安装层参数 ──────────────────────────────────────────────────────────────
// 描述当前机械安装方式对几何解的影响，处于纯几何层和舵机层之间
typedef struct {
    int8_t shoulder_sign;   // 肩膀在当前安装方式下的转动方向（+1 或 -1）
    /*
     * 安装层参数：
     * - shoulder_sign   : 肩膀在当前安装方式下的方向解释
     * - beta_sign/offset: beta 大夹角在当前安装方式下的方向和零位偏置
     *
     * 这组数据属于"安装层"，不是最终舵机层。
     * 舵机层还会在此基础上继续叠加 servo_sign、标定偏置和真实执行修正。
     */
    int8_t beta_sign;          // 小腿 beta 大夹角的方向（+1 或 -1）
    float  beta_offset_deg;    // 小腿 beta 安装零位偏置（度），"垂直向前伸"对应的 beta 基准值
} scargo_leg_installation_binding_t;

const scargo_gpio_map_t                *board_defaults_gpio_map(void);
const scargo_mechanics_t               *board_defaults_mechanics(void);
const scargo_servo_binding_t          (*board_defaults_servo_map(void))[SCARGO_JOINTS_PER_LEG];
const scargo_leg_installation_binding_t *board_defaults_leg_installation(void);
scargo_buzzer_mode_t                    board_defaults_buzzer_mode(void);
