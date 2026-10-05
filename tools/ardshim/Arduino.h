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

// ── The real Arduino.h's MACROS ───────────────────────────────
// arduino-esp32 defines these as preprocessor macros, which silently
// rewrite ANY identifier of the same name -- a local called `sq`, a
// parameter called `DISPLAY`, an enum member called `DEFAULT`.  A shim
// without them compiles code the real core rejects; with them, pre-flight
// fails the same way the real build would.
#ifndef PI
#define PI          3.1415926535897932384626433832795
#define HALF_PI     1.5707963267948966192313216916398
#define TWO_PI      6.283185307179586476925286766559
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG  57.295779513082320876798154814105
#define EULER       2.718281828459045235360287471352
#endif
#define SERIAL      0x0
#define DISPLAY     0x1
#define LSBFIRST    0
#define MSBFIRST    1
#ifndef HIGH
#define HIGH        0x1
#define LOW         0x0
#endif
#define INPUT_PULLDOWN 0x09
#define OPEN_DRAIN  0x10
#define RISING      0x01
#define FALLING     0x02
#define CHANGE      0x03
#define ONLOW       0x04
#define ONHIGH      0x05
#define DEFAULT     1
#define EXTERNAL    0
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x)        ((x)*(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define lowByte(w)   ((uint8_t) ((w) & 0xff))
#define highByte(w)  ((uint8_t) ((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bit(b)       (1UL << (b))
#define _BV(b)       (1UL << (b))
