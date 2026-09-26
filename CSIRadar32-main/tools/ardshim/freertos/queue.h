#pragma once
#include <freertos/FreeRTOS.h>
typedef void* QueueHandle_t;
static inline QueueHandle_t xQueueCreate(int,size_t){return nullptr;}
static inline BaseType_t xQueueReceive(QueueHandle_t,void*,uint32_t){return 0;}
static inline BaseType_t xQueueSend(QueueHandle_t,const void*,int){return pdTRUE;}
