#pragma once
#include <stdint.h>
#include <stddef.h>
#define ESP_OK 0
typedef struct { uint8_t src_addr[6]; } esp_now_recv_info_t;
typedef struct { uint8_t peer_addr[6]; int ifidx; bool encrypt; int channel; } esp_now_peer_info_t;
static inline int esp_now_init(){return 0;}
static inline int esp_now_send(const uint8_t*,const uint8_t*,size_t){return 0;}
static inline int esp_now_add_peer(const esp_now_peer_info_t*){return 0;}
static inline void esp_now_register_recv_cb(void(*)(const esp_now_recv_info_t*,const uint8_t*,int)){}
