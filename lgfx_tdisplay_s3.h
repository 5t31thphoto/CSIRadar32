// ═══════════════════════════════════════════════════════════════
//  CSI-Radar-S3 — lgfx_tdisplay_s3.h
//  LovyanGFX board class for LilyGo T-Display-S3 (ST7789, 8-bit i80).
//  Portrait orientation: 170 x 320.
// ═══════════════════════════════════════════════════════════════
#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX_TDisplayS3 : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789   _panel;
    lgfx::Bus_Parallel8  _bus;
    lgfx::Light_PWM      _light;

public:
    LGFX_TDisplayS3() {
        // Bus (8-bit i80 parallel)
        {
            auto cfg = _bus.config();
            cfg.freq_write = 20000000;
            cfg.pin_wr   = 8;
            cfg.pin_rd   = 9;
            cfg.pin_rs   = 7;   // D/C
            cfg.pin_d0   = 39;
            cfg.pin_d1   = 40;
            cfg.pin_d2   = 41;
            cfg.pin_d3   = 42;
            cfg.pin_d4   = 45;
            cfg.pin_d5   = 46;
            cfg.pin_d6   = 47;
            cfg.pin_d7   = 48;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }

        // Panel (ST7789, 170x320)
        {
            auto cfg = _panel.config();
            cfg.pin_cs        = 6;
            cfg.pin_rst       = 5;
            cfg.pin_busy      = -1;
            cfg.memory_width  = 240;
            cfg.memory_height = 320;
            cfg.panel_width   = 170;
            cfg.panel_height  = 320;
            cfg.offset_x      = 35;   // ST7789 memory offset for 170-wide window
            cfg.offset_y      = 0;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits  = 1;
            cfg.readable       = true;
            cfg.invert         = true;
            cfg.rgb_order      = false;
            cfg.dlen_16bit     = false;
            cfg.bus_shared     = true;
            _panel.config(cfg);
        }

        // Backlight
        {
            auto cfg = _light.config();
            cfg.pin_bl      = 38;
            cfg.invert      = false;
            cfg.freq        = 44100;
            cfg.pwm_channel = 7;
            _light.config(cfg);
            _panel.setLight(&_light);
        }

        setPanel(&_panel);
    }
};
