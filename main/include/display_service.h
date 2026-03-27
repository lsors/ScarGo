#pragma once

typedef enum {
    DISPLAY_PAGE_STATUS = 0,
    DISPLAY_PAGE_CALIBRATION,
    DISPLAY_PAGE_TEST,
} display_page_t;

void display_service_init(void);
void display_service_tick(void);
display_page_t display_service_get_page(void);
