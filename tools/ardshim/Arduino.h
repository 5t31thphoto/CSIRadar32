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
struct SerialT { void begin(int){} void println(const char*){} void print(const char*){}
                 void printf(const char*,...){} operator bool(){return true;} };
static SerialT Serial;
struct EspT { void restart(){} };
static EspT ESP;
