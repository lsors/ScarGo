#include "log_service.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define LOG_RING_SIZE 8192U

static char s_ring[LOG_RING_SIZE];
static volatile uint32_t s_write_seq = 0;
static SemaphoreHandle_t s_mutex;

static size_t strip_ansi(const char *src, size_t src_len, char *dst, size_t dst_cap)
{
    size_t out = 0;
    for (size_t i = 0; i < src_len && out < dst_cap - 1U; i++) {
        if (src[i] == '\033' && i + 1U < src_len && src[i + 1U] == '[') {
            i += 2U;
            while (i < src_len && src[i] != 'm') {
                i++;
            }
            continue;
        }
        dst[out++] = src[i];
    }
    dst[out] = '\0';
    return out;
}

static int log_vprintf_hook(const char *fmt, va_list args)
{
    va_list copy;
    va_copy(copy, args);
    int ret = vprintf(fmt, args);

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        static char tmp[512];
        static char stripped[512];
        int len = vsnprintf(tmp, sizeof(tmp), fmt, copy);
        if (len > 0) {
            size_t raw_len = (size_t)len < sizeof(tmp) ? (size_t)len : sizeof(tmp) - 1U;
            size_t stripped_len = strip_ansi(tmp, raw_len, stripped, sizeof(stripped));
            for (size_t i = 0; i < stripped_len; i++) {
                s_ring[s_write_seq % LOG_RING_SIZE] = stripped[i];
                s_write_seq++;
            }
        }
        xSemaphoreGive(s_mutex);
    }
    va_end(copy);

    return ret;
}

void log_service_init(void)
{
    memset(s_ring, 0, sizeof(s_ring));
    s_write_seq = 0;
    s_mutex = xSemaphoreCreateMutex();
    esp_log_set_vprintf(log_vprintf_hook);
}

char *log_service_snapshot_json(uint32_t cursor_in)
{
    char *text = malloc(LOG_RING_SIZE + 1U);
    if (text == NULL) {
        return NULL;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        free(text);
        return NULL;
    }

    uint32_t write_seq = s_write_seq;
    uint32_t avail = write_seq - cursor_in;
    if (avail > LOG_RING_SIZE) {
        cursor_in = write_seq - (uint32_t)LOG_RING_SIZE;
        avail = (uint32_t)LOG_RING_SIZE;
    }

    if (avail > 0U) {
        uint32_t start = cursor_in % (uint32_t)LOG_RING_SIZE;
        for (uint32_t i = 0U; i < avail; i++) {
            text[i] = s_ring[(start + i) % (uint32_t)LOG_RING_SIZE];
        }
    }
    text[avail] = '\0';
    uint32_t new_cursor = cursor_in + avail;

    xSemaphoreGive(s_mutex);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "cursor", (double)new_cursor);
    cJSON_AddStringToObject(root, "text", text);
    free(text);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}
