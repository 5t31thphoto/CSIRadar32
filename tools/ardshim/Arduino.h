#pragma once
// Minimal Arduino.h so tools/opcode_check.cpp can include config.h on a
// plain runner with no ESP32 toolchain.
//
// The check compares opcode VALUES and struct OFFSETS.  It does not run
// anything, so it needs only enough of Arduino to let config.h parse.
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static inline uint32_t millis(){return 0;}
static inline uint32_t micros(){return 0;}
static inline void delay(uint32_t){}
static inline void pinMode(int,int){}
static inline void digitalWrite(int,int){}
static inline int  digitalRead(int){return 1;}
static inline float sq(float x){return x*x;}
static inline float constrain(float v,float a,float b){return v<a?a:(v>b?b:v);}
static inline long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
#define PROGMEM
#define F(x) x
struct SerialT { void begin(unsigned long){} void flush(){} int available(){return 0;} int read(){return -1;}
                 size_t write(const uint8_t*,size_t n){return n;} void println(const char*){} void print(const char*){}
                 void printf(const char*,...){} operator bool(){return true;} };
static SerialT Serial;
struct EspT { void restart(){} uint32_t getFreeHeap(){return 0;} uint32_t getMinFreeHeap(){return 0;}
  uint32_t getPsramSize(){return 0;} uint32_t getFreePsram(){return 0;} uint32_t getHeapSize(){return 0;} };
static EspT ESP;
// ── receiver-wide additions (pre-flight covers every .cpp now) ──
#include <stdarg.h>
#define IRAM_ATTR
#define DRAM_ATTR
#define RTC_DATA_ATTR
#define RTC_NOINIT_ATTR
#define SERIAL_8N1 0x800001c
static inline long random(long){return 0;}
static inline long random(long,long){return 0;}
static inline void randomSeed(unsigned long){}
static inline void yield(){}
static inline int analogRead(int){return 0;}
static inline uint32_t analogReadMilliVolts(int){return 0;}
struct EspT2 {};
struct HardwareSerial {
  explicit HardwareSerial(int){}
  void begin(unsigned long,uint32_t=SERIAL_8N1,int8_t=-1,int8_t=-1){}
  void end(){} void setTimeout(unsigned long){}
  int available(){return 0;} int read(){return -1;}
  size_t write(const uint8_t*,size_t n){return n;} size_t write(uint8_t){return 1;}
  size_t setRxBufferSize(size_t n){return n;} size_t setTxBufferSize(size_t n){return n;}
  void flush(){}
};
