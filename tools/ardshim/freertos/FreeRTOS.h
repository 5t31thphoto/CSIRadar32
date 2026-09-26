#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int BaseType_t;
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY 0xFFFFFFFF
static inline void vTaskDelay(int){}
