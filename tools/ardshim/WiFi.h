#pragma once
#include <stdint.h>
#define WIFI_OFF 0
#define WIFI_STA 1
#define WIFI_AP 2
struct WiFiT { void mode(int){} void disconnect(bool=false,bool=false){}
  void macAddress(uint8_t*m){for(int i=0;i<6;i++)m[i]=(uint8_t)i;} };
static WiFiT WiFi;
