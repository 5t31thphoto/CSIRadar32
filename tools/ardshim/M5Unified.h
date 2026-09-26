#pragma once
#include <Arduino.h>   // the real M5Unified pulls this in too
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
namespace fonts { struct F{int _;}; extern F Font0,Font2,Font4; }
struct M5Canvas { M5Canvas(void*){}
  bool createSprite(int,int){return true;} void pushSprite(int,int){}
  void fillSprite(uint16_t){} void fillRect(int,int,int,int,uint16_t){}
  void drawRect(int,int,int,int,uint16_t){} void drawCircle(int,int,int,uint16_t){}
  void fillCircle(int,int,int,uint16_t){} void drawLine(int,int,int,int,uint16_t){}
  void setFont(const fonts::F*){} void setColorDepth(int){} void setTextColor(uint16_t,uint16_t){}
  void setCursor(int,int){} void print(const char*){} void printf(const char*,...){} };
struct Disp { void setRotation(int){} void fillScreen(uint16_t){} void setTextColor(uint16_t,uint16_t){} void setCursor(int,int){} void print(const char*){} int width(){return 320;} int height(){return 240;} };
struct BtnT { bool isPressed(){return false;} bool wasPressed(){return false;} };
struct TouchDetail { int x=0,y=0; bool isPressed(){return false;} };
struct TouchT { TouchDetail getDetail(){return TouchDetail();} };
struct PwrT { int getBatteryLevel(){return 80;} bool isCharging(){return false;}
              void setVibration(int){} };
namespace m5 { struct vec3{float x,y,z;}; struct imu_data_t{vec3 accel,gyro;}; }
struct ImuT { bool update(){return false;} m5::imu_data_t getImuData(){return {};} };
struct SpkT { void setVolume(int){} void tone(int,int,int=-1){}
              bool playWav(const uint8_t*,size_t){return true;}
              bool playRaw(const int16_t*,size_t,int,bool,int,int){return true;}
              void stop(int){} };
struct CfgT { bool internal_imu=false, internal_spk=false; };
struct M5T { Disp Display; BtnT BtnA,BtnB,BtnC; TouchT Touch; PwrT Power;
             ImuT Imu; SpkT Speaker;
             CfgT config(){return CfgT();} void begin(CfgT&){} void update(){} };
static M5T M5;
namespace m5 { typedef ::SpkT Speaker_Class; }
