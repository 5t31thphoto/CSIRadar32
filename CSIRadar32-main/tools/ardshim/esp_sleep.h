#pragma once
#include <stdint.h>
static inline void esp_sleep_enable_timer_wakeup(uint64_t){}
static inline void esp_light_sleep_start(){}
#define ESP_SLEEP_WAKEUP_ALL 0
#define ESP_EXT1_WAKEUP_ANY_LOW 0
static inline void esp_sleep_disable_wakeup_source(int){}
static inline void esp_deep_sleep_start(){}
static inline void esp_sleep_enable_ext0_wakeup(int,int){}
static inline void esp_sleep_enable_ext1_wakeup(uint64_t,int){}
#define ESP_PD_DOMAIN_RTC_PERIPH 0
#define ESP_PD_OPTION_ON 1
#define ESP_SLEEP_WAKEUP_EXT0 2
#define ESP_SLEEP_WAKEUP_EXT1 3
#define ESP_SLEEP_WAKEUP_TIMER 4
typedef int esp_sleep_wakeup_cause_t;
static inline void esp_sleep_pd_config(int,int){}
static inline esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause(){return 0;}
static inline uint64_t esp_sleep_get_ext1_wakeup_status(){return 0;}
#define ESP_SLEEP_WAKEUP_GPIO 5
