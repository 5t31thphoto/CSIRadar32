#pragma once
// Host shim for the slice of LovyanGFX the receiver uses.  Shapes only:
// the pre-flight checks that every call site names a real method with a
// plausible signature, not that anything is drawn.
#include <Arduino.h>
#include <stdarg.h>
namespace lgfx {
  struct IFont {};
  namespace fonts { extern const IFont Font0, Font2, Font4, Font6, Font7, Font8,
                    FreeMono9pt7b, FreeMonoBold9pt7b, FreeSans9pt7b,
                    FreeSansBold9pt7b, FreeSansBold12pt7b, TomThumb; }
  struct Bus_Parallel8 { struct config_t { uint32_t freq_write; int pin_wr,pin_rd,pin_rs,
      pin_d0,pin_d1,pin_d2,pin_d3,pin_d4,pin_d5,pin_d6,pin_d7; };
      config_t config() const { return config_t(); } void config(const config_t&) {} };
  struct Light_PWM { struct config_t { int pin_bl; bool invert; uint32_t freq; int pwm_channel; };
      config_t config() const { return config_t(); } void config(const config_t&) {} };
  struct Panel_ST7789 { struct config_t { int pin_cs,pin_rst,pin_busy; int memory_width,memory_height,
      panel_width,panel_height,offset_x,offset_y,offset_rotation,dummy_read_pixel,dummy_read_bits;
      bool readable,invert,rgb_order,dlen_16bit,bus_shared; };
      config_t config() const { return config_t(); } void config(const config_t&) {}
      void setBus(Bus_Parallel8*) {} void setLight(Light_PWM*) {} };
  struct LGFXBase {
      virtual ~LGFXBase() {}
      void fillScreen(uint32_t) {}  void fillRect(int32_t,int32_t,int32_t,int32_t,uint32_t) {}
      void drawRect(int32_t,int32_t,int32_t,int32_t,uint32_t) {}
      void fillRoundRect(int32_t,int32_t,int32_t,int32_t,int32_t,uint32_t) {}
      void drawRoundRect(int32_t,int32_t,int32_t,int32_t,int32_t,uint32_t) {}
      void drawLine(int32_t,int32_t,int32_t,int32_t,uint32_t) {}
      void drawFastHLine(int32_t,int32_t,int32_t,uint32_t) {}
      void drawFastVLine(int32_t,int32_t,int32_t,uint32_t) {}
      void drawPixel(int32_t,int32_t,uint32_t) {}
      void drawCircle(int32_t,int32_t,int32_t,uint32_t) {}
      void fillCircle(int32_t,int32_t,int32_t,uint32_t) {}
      void drawArc(int32_t,int32_t,int32_t,int32_t,float,float,uint32_t) {}
      void fillArc(int32_t,int32_t,int32_t,int32_t,float,float,uint32_t) {}
      void fillTriangle(int32_t,int32_t,int32_t,int32_t,int32_t,int32_t,uint32_t) {}
      void drawTriangle(int32_t,int32_t,int32_t,int32_t,int32_t,int32_t,uint32_t) {}
      void setTextColor(uint32_t) {} void setTextColor(uint32_t,uint32_t) {}
      void setTextSize(float) {} void setTextDatum(uint8_t) {}
      void setCursor(int32_t,int32_t) {}
      void setFont(const IFont*) {}
      int32_t textWidth(const char*) { return 0; }
      int32_t fontHeight() { return 8; }
      size_t print(const char*) { return 0; } size_t print(char) { return 0; }
      size_t print(int) { return 0; } size_t print(unsigned) { return 0; }
      size_t print(long) { return 0; } size_t print(unsigned long) { return 0; }
      size_t print(double, int=2) { return 0; }
      size_t println(const char* = "") { return 0; }
      size_t printf(const char*, ...) __attribute__((format(printf,2,3))) { return 0; }
      size_t drawString(const char*,int32_t,int32_t) { return 0; }
      int32_t width() const { return 170; } int32_t height() const { return 320; }
      void startWrite() {} void endWrite() {}
      static uint16_t color565(uint8_t r,uint8_t g,uint8_t b){return (uint16_t)(((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3));}
  };
  struct LGFX_Device : LGFXBase {
      bool init() { return true; } bool begin() { return true; }
      void setRotation(uint8_t) {} void setBrightness(uint8_t) {}
      void sleep() {} void wakeup() {} void setPanel(Panel_ST7789*) {}
  };
}
typedef lgfx::LGFXBase LovyanGFX;
struct LGFX_Sprite : lgfx::LGFXBase {
    explicit LGFX_Sprite(lgfx::LGFXBase* = nullptr) {}
    void* createSprite(int32_t,int32_t) { return this; }
    void deleteSprite() {} void setColorDepth(int) {} void setPsram(bool) {}
    void fillSprite(uint32_t) {} void pushSprite(int32_t,int32_t) {}
    void pushSprite(lgfx::LGFXBase*,int32_t,int32_t) {}
};
using namespace lgfx;
#define TFT_BLACK 0x0000
#define TFT_WHITE 0xFFFF
#define TL_DATUM 0
#define TC_DATUM 1
#define MC_DATUM 4
