#pragma once
#include <stdint.h>
#include <stddef.h>
class AudioOutput { public:
  int hertz = 44100;
  virtual ~AudioOutput(){}
  virtual bool begin(){return true;}
  virtual bool ConsumeSample(int16_t[2]){return true;}
  virtual void flush(){}
  virtual bool stop(){return true;} };
