#pragma once
#include <AudioFileSourceID3.h>
#include <AudioOutput.h>
class AudioGeneratorMP3 { public:
  bool begin(AudioFileSource*,AudioOutput*){return false;}
  bool isRunning(){return false;} bool loop(){return false;} bool stop(){return true;} };
