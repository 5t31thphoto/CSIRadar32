#pragma once
#include <freertos/FreeRTOS.h>
static inline BaseType_t xTaskCreatePinnedToCore(void(*)(void*),const char*,int,void*,int,void*,int){return pdPASS;}
