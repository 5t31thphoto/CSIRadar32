#pragma once
#include <stdint.h>
#define WIFI_IF_STA 0
#define WIFI_SECOND_CHAN_NONE 0
#define WIFI_PHY_RATE_MCS0_LGI 0
struct RxC { int rssi, noise_floor, sig_mode, rate, mcs, cwb; };
typedef struct { uint8_t mac[6]; int8_t *buf; int len; RxC rx_ctrl; } wifi_csi_info_t;
typedef struct { bool lltf_en,htltf_en,stbc_htltf2_en,ltf_merge_en,channel_filter_en,manu_scale; } wifi_csi_config_t;
static inline void esp_wifi_set_mac(int,uint8_t*){}
static inline void esp_wifi_set_promiscuous(bool){}
static inline void esp_wifi_set_channel(int,int){}
static inline void esp_wifi_set_max_tx_power(int){}
static inline void esp_wifi_config_espnow_rate(int,int){}
static inline void esp_wifi_set_csi_config(wifi_csi_config_t*){}
static inline void esp_wifi_set_csi_rx_cb(void(*)(void*,wifi_csi_info_t*),void*){}
static inline void esp_wifi_set_csi(bool){}
static inline int esp_wifi_stop(){return 0;}
static inline int esp_wifi_deinit(){return 0;}
static inline int esp_wifi_start(){return 0;}
static inline int esp_wifi_restore(){return 0;}
