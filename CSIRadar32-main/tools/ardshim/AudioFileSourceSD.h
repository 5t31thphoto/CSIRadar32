#pragma once
class AudioFileSource { public: virtual ~AudioFileSource(){} };
class AudioFileSourceSD : public AudioFileSource { public:
  AudioFileSourceSD(const char*){} bool isOpen(){return false;} };
