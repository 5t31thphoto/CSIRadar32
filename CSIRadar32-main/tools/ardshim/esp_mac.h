#pragma once
#include <stdint.h>
#define ESP_MAC_WIFI_STA 0
static inline void esp_read_mac(uint8_t*m,int){for(int i=0;i<6;i++)m[i]=(uint8_t)(0x24+i);}
