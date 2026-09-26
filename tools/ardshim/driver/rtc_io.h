#pragma once
#include <stdint.h>
typedef int gpio_num_t;
static inline void rtc_gpio_hold_en(gpio_num_t){}
static inline void rtc_gpio_hold_dis(gpio_num_t){}
static inline void rtc_gpio_isolate(gpio_num_t){}
static inline void gpio_hold_en(gpio_num_t){}
static inline void gpio_hold_dis(gpio_num_t){}
static inline void gpio_deep_sleep_hold_en(){}
static inline void gpio_deep_sleep_hold_dis(){}
static inline void rtc_gpio_pullup_en(gpio_num_t){}
static inline void rtc_gpio_pulldown_dis(gpio_num_t){}
static inline void rtc_gpio_pullup_dis(gpio_num_t){}
static inline void rtc_gpio_pulldown_en(gpio_num_t){}
static inline void rtc_gpio_init(gpio_num_t){}
static inline void rtc_gpio_set_direction(gpio_num_t,int){}
static inline void rtc_gpio_deinit(gpio_num_t){}
