#pragma once
#include <M5Unified.h>
#include <vector>
#include <string>
typedef std::string String;
struct KeysStateT { std::vector<char> word; bool del=false, enter=false, fn=false; };
struct Keyboard_Class { typedef KeysStateT KeysState;
  bool isChange(){return false;} bool isPressed(){return false;}
  KeysState keysState(){return KeysState();} };
struct CardputerT { Disp Display; Keyboard_Class Keyboard; PwrT Power; BtnT BtnA; SpkT Speaker;
  void begin(CfgT&,bool=false){} void update(){} };
static CardputerT M5Cardputer;
static inline void* heap_caps_malloc(size_t,int){return nullptr;}
#define MALLOC_CAP_SPIRAM 1
