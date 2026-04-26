#pragma once

#include <stdint.h>

void log_service_init(void);

// Returns a malloc'd JSON string: {"cursor":N,"text":"..."}, caller must free.
// Pass cursor_in=0 to get the most recent buffer contents.
char *log_service_snapshot_json(uint32_t cursor_in);
