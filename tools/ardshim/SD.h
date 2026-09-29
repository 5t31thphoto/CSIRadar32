#pragma once
#include <SPI.h>
#include <stddef.h>
// Shape of the arduino-esp32 FS/SD API (FS.h), as used by the probes.
#define FILE_READ   "r"
#define FILE_WRITE  "w"
#define FILE_APPEND "a"
struct FileT { operator bool(){return false;} size_t size(){return 0;}
               size_t read(uint8_t*,size_t){return 0;} void close(){}
               size_t write(const uint8_t*,size_t n){return n;} size_t write(uint8_t){return 1;}
               bool seek(uint32_t){return true;} size_t position(){return 0;} void flush(){} };
typedef FileT File;
struct SDC { bool begin(int){return false;}
             bool begin(int,SPIC&,int){return false;}
             bool mkdir(const char*){return true;}
             bool exists(const char*){return false;}
             File open(const char*, const char* = FILE_READ){return File();}
             uint64_t totalBytes(){return 0;} uint64_t usedBytes(){return 0;} };
static SDC SD;
