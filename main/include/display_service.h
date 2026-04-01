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

void display_service_init(void);
void display_service_tick(void);
display_page_t display_service_get_page(void);
void display_service_set_page(display_page_t page);
int display_service_get_leg_preview_selection(void);
void display_service_set_leg_preview_selection(int leg);
