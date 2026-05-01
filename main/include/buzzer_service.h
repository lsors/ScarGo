#pragma once

#include <stddef.h>
#include <stdint.h>

// 蜂鸣器单步时序：on_ms 毫秒鸣叫，off_ms 毫秒静音，循环拼接成完整音效
typedef struct {
    uint16_t on_ms;    // 鸣叫持续时间（ms）
    uint16_t off_ms;   // 静音持续时间（ms）
} buzzer_beep_step_t;

void buzzer_service_init(void);
void buzzer_service_boot_beep(void);
void buzzer_service_test_beep(void);
void buzzer_service_page_connect_beep(void);
// 遥控链路刚建立时的单声提示。
void buzzer_service_elrs_connect_beep(void);
// 遥控真正解锁、可以开始操控机器人时的双声提示。
void buzzer_service_rc_unlock_beep(void);
void buzzer_service_save_beep(void);
void buzzer_service_play_pattern(const buzzer_beep_step_t *steps, size_t count);
