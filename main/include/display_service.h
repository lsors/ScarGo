#pragma once

typedef enum {
    DISPLAY_PAGE_STATUS = 0,
    DISPLAY_PAGE_CALIBRATION,
    DISPLAY_PAGE_TEST,
    DISPLAY_PAGE_RC_TARGET,
    DISPLAY_PAGE_CPU,
    DISPLAY_PAGE_LEG_PREVIEW,
    DISPLAY_PAGE_ROBOT_PREVIEW,
} display_page_t;

typedef enum {
    DISPLAY_PREVIEW_VIEW_FRONT = 0,
    DISPLAY_PREVIEW_VIEW_BACK,
    DISPLAY_PREVIEW_VIEW_LEFT,
    DISPLAY_PREVIEW_VIEW_RIGHT,
    DISPLAY_PREVIEW_VIEW_TOP,
    DISPLAY_PREVIEW_VIEW_ISO,
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
