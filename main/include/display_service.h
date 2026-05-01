#pragma once

// ── OLED 显示页面 ─────────────────────────────────────────────────────────────
typedef enum {
    DISPLAY_PAGE_STATUS       = 0,   // 主状态页：模式、链路、IMU 姿态、步态状态
    DISPLAY_PAGE_CALIBRATION,        // 标定页：当前舵机偏置预览
    DISPLAY_PAGE_TEST,               // 测试页：舵机手动测试
    DISPLAY_PAGE_RC_TARGET,          // 遥控目标页：实时显示遥控输入映射后的目标姿态
    DISPLAY_PAGE_CPU,                // CPU 使用率页：各任务负载
    DISPLAY_PAGE_LEG_PREVIEW,        // 单腿预览页：选定腿的关节链 3D 投影
    DISPLAY_PAGE_ROBOT_PREVIEW,      // 整机预览页：四腿整体 3D 投影
} display_page_t;

// ── 3D 预览视角 ───────────────────────────────────────────────────────────────
typedef enum {
    DISPLAY_PREVIEW_VIEW_FRONT = 0,  // 正面视角（从前方看）
    DISPLAY_PREVIEW_VIEW_BACK,       // 背面视角（从后方看）
    DISPLAY_PREVIEW_VIEW_LEFT,       // 左侧视角
    DISPLAY_PREVIEW_VIEW_RIGHT,      // 右侧视角
    DISPLAY_PREVIEW_VIEW_TOP,        // 俯视视角
    DISPLAY_PREVIEW_VIEW_ISO,        // 等轴测视角（斜 45° 俯视）
} display_preview_view_t;

void display_service_init(void);
void display_service_tick(void);
display_page_t display_service_get_page(void);
void display_service_set_page(display_page_t page);
int display_service_get_leg_preview_selection(void);
void display_service_set_leg_preview_selection(int leg);
display_preview_view_t display_service_get_leg_preview_view(void);
void display_service_set_leg_preview_view(display_preview_view_t view);
display_preview_view_t display_service_get_robot_preview_view(void);
void display_service_set_robot_preview_view(display_preview_view_t view);
