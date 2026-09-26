#pragma once
#include <SPI.h>
struct FileT { operator bool(){return false;} size_t size(){return 0;}
               size_t read(uint8_t*,size_t){return 0;} void close(){} };
typedef FileT File;
struct SDC { bool begin(int){return false;}
             bool begin(int,SPIC&,int){return false;}
             bool mkdir(const char*){return true;}
             bool exists(const char*){return false;} File open(const char*){return File();} };
static SDC SD;
