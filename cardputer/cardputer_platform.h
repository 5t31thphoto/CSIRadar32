#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CARDPUTER-ADV PLATFORM
//  The probe that stays in the operator's hand
// ═══════════════════════════════════════════════════════════════
//
//  ── WHAT THIS DEVICE IS, AND IS NOT ──────────────────────────
//
//  The Cardputer-Adv is a FULL RECEIVER.  It runs the same inference as
//  a T-Display: CSI capture, the scene solver, the mesh layers, the
//  tomography, all of it.
//
//  What it never is, is an ANCHOR.  It stays in the operator's hand, so
//  it is a moving observer by definition and a moving observer cannot be
//  the fixed reference the stereo pair provides.  That is enforced here
//  in one place rather than being a convention people have to remember:
//  MANTIS_ROLE_LOCKED_PROBE.  Every role-assignment path consults it.
//
//  Three deployment shapes it has to work in:
//
//    with a docked stereo pair   the pair anchors; the Cardputer is the
//                                mobile probe and the remote control
//    with one T-Display anchor   anchor + probe, the classic pair
//    alone with beacons          nobody anchors.  The Cardputer runs its
//                                OWN inference mode, which is the mesh
//                                tomography plus its own CSI -- and that
//                                works precisely because the beacon mesh
//                                is autonomous and needs no receiver.
//
//  ── HARDWARE (from M5Stack documentation, not assumed) ───────
//
//    module     Stamp-S3A, ESP32-S3FN8
//    display    1.14" 240x135 LANDSCAPE   (T-Display is 170x320 portrait)
//    keyboard   56 keys, 4x14 matrix, plus dedicated arrows on the ADV
//    button     G0 only, exposed as M5Cardputer.BtnA
//    IMU        BMI270 6-axis, reached via M5.Imu (NOT M5Cardputer.Imu)
//    audio      ES8311 codec + NS4150B amp + 1 W speaker + 3.5 mm jack
//    storage    microSD
//    power      1750 mAh, M5Cardputer.Power
//
//  Toolchain requirements, from the vendor docs:
//    board manager >= 3.2.3, board = M5Cardputer
//    M5Cardputer >= 1.1.1, M5Unified >= 0.2.10, M5GFX >= 0.2.10
//
//  NO PIN NUMBERS APPEAR IN THIS FILE.  The M5Cardputer library owns the
//  pin map, and hardcoding it here would silently break on any board
//  revision.  The one exception documented by M5 -- GPIO38 gating the
//  RGB LED rail on Stamp-S3A -- is handled by the library's begin().
//
//  ── DISPLAY GEOMETRY IS THE REAL PORTING WORK ────────────────
//
//    T-Display-S3   170 x 320   portrait, tall
//    Cardputer-Adv  240 x 135   landscape, wide
//
//  That is not a scale factor, it is a rotation of the whole layout.  A
//  screen designed as a tall column of rows has to become a wide row of
//  columns.  Sizes live here so no screen hardcodes them.
// ═══════════════════════════════════════════════════════════════

#include <stdint.h>

#define MANTIS_PLATFORM_CARDPUTER 1

// The probe role is a property of the HARDWARE, not a setting.
#define MANTIS_ROLE_LOCKED_PROBE  1

// ── Screen ────────────────────────────────────────────────────
#define CP_SCREEN_W        240
#define CP_SCREEN_H        135
#define CP_HEADER_H         16
#define CP_FOOTER_H         14
#define CP_CONTENT_Y       (CP_HEADER_H)
#define CP_CONTENT_H       (CP_SCREEN_H - CP_HEADER_H - CP_FOOTER_H)

// Landscape affords a side-by-side split the T-Display never could: a
// map on the left and a status column on the right, both visible at
// once.  On a 170-wide portrait panel that would leave neither usable.
#define CP_SPLIT_X         (CP_SCREEN_W * 58 / 100)   // map / status divider
#define CP_MAP_CX          (CP_SPLIT_X / 2)
#define CP_MAP_CY          (CP_CONTENT_Y + CP_CONTENT_H / 2)
#define CP_MAP_R           (CP_CONTENT_H / 2 - 4)

// ── Input ─────────────────────────────────────────────────────
// The shared UI speaks in LEFT/RIGHT because the T-Display has two
// buttons.  The Cardputer has a keyboard, so it can express far more --
// but the mapping must keep LEFT/RIGHT working identically or every
// shared screen has to be rewritten.
//
// So: arrows and ENTER/BACKSPACE map onto the existing two-button
// vocabulary, and the EXTRA keys add capability without changing it.
typedef enum : uint8_t {
    CPK_NONE = 0,
    CPK_LEFT,      // arrow left   -> BTN_LEFT  (back)
    CPK_RIGHT,     // arrow right  -> BTN_RIGHT (advance)
    CPK_UP,        // new: previous item / scroll up
    CPK_DOWN,      // new: next item / scroll down
    CPK_ENTER,     // -> BTN_RIGHT long press (confirm)
    CPK_BACK,      // backspace -> BTN_LEFT (back out)
    CPK_ESC,       // new: jump to dashboard from anywhere
    CPK_TAB,       // new: cycle dashboard view
    CPK_CHAR,      // a printable character, in cp_last_char()
} CardputerKey;

// NOTE ON ARROW KEY CODES
//
// The ADV has dedicated arrow keys where the original Cardputer used Fn
// combinations.  The M5Cardputer KeysState exposes .word, .del, .enter,
// .fn, .ctrl, .shift, .opt, .alt -- arrows arrive as characters in
// .word, and the exact code points are a board-revision detail the
// public docs do not pin down.
//
// So the mapping below accepts BOTH the dedicated-arrow characters and
// the Fn-combination fallback, and is deliberately a single table that
// can be corrected in one place after a five-minute check on hardware.
// Guessing silently would produce a device whose arrow keys do nothing
// and no obvious reason why.
#define CP_ARROW_LEFT_CH   ','
#define CP_ARROW_RIGHT_CH  '/'
#define CP_ARROW_UP_CH     ';'
#define CP_ARROW_DOWN_CH   '.'

// ── Audio ─────────────────────────────────────────────────────
// Alarms are files on the SD card, not tones compiled into firmware, so
// an operator can change them without a rebuild.  Each has a fallback
// tone for the case where the card is missing or the file is not there
// -- an alarm that silently does not sound is worse than a beep.
// MP3, decoded in its own task.
//
// ESP8266Audio 1.9.7 (that exact version) bridged onto
// m5::Speaker_Class, which is the chain the working Cardputer-Adv MP3
// players use.  See mantis_mp3.h.
//
// Decoding runs on its own FreeRTOS task, not in loop().  Sounding an
// alarm must never stall the sensing loop -- a device that briefly stops
// watching the room in order to announce something is exactly backwards.
// The alarm enum lives in mantis_alarms.h beside the table it indexes.
// Splitting an enum from the array it indexes across two headers is how
// the Core2 build failed on CPA_COUNT.


// ── SD card ───────────────────────────────────────────────────
// THESE PINS ARE REQUIRED AND THEY ARE THE ONE EXCEPTION TO THIS FILE'S
// no-pin-numbers rule.
//
// M5Cardputer.begin() does NOT mount the card, and SD.begin() with no
// arguments uses the default VSPI pins, which are not these -- so the
// obvious call silently returns false and every SD feature is dead with
// no error to explain it.  That is a known stumbling block on this board
// and it is what the vendor's own sdcard example exists to demonstrate.
//
// From the M5Stack Cardputer microSD documentation (applies to Cardputer
// and Cardputer-Adv alike):
#define CP_SD_SCK_PIN   40
#define CP_SD_MISO_PIN  39
#define CP_SD_MOSI_PIN  14
#define CP_SD_CS_PIN    12
#define CP_SD_FREQ      25000000

// ── SD layout ─────────────────────────────────────────────────
// Having storage changes what the device can be: not just a display,
// but a recorder.  A saved session can be replayed against a changed
// inference, which is the only way to compare two algorithms on the same
// RF -- something no amount of live testing can do, because the room is
// never twice the same.
#define CP_DIR_ROOT     "/mantis"
#define CP_DIR_ALARMS   "/mantis/alarms"
#define CP_DIR_SESSIONS "/mantis/sessions"
#define CP_DIR_LOGS     "/mantis/logs"
#define CP_DIR_CONFIG   "/mantis/config"

// ── IMU ───────────────────────────────────────────────────────
// The IMU is not a nicety.  It gives the probe a second, INDEPENDENT
// measurement of its own motion, and independence is what makes a
// cowitness worth having:
//
//   rotation   the cal walk asks the operator to turn in place and the
//              RF estimates the turn.  The IMU MEASURES it.  The red
//              needle can finally show truth instead of an estimate
//              checking itself.
//   steps      a step count with an assumed stride is a crude distance,
//              but it is crude in a completely different way from RF --
//              so when the two agree the walk is sound, and when they
//              disagree one of them is wrong and the operator can be
//              told before the model is built on it.
//   aspect     turning the device while watching per-beacon RSSI sweeps
//              its antenna pattern across the room, which constrains
//              beacon BEARING from a single standing position.
#define CP_STRIDE_M_DEFAULT   0.72f   // adult average; user-adjustable
#define CP_STEP_MIN_INTERVAL_MS 250   // faster than this is not walking
#define CP_STEP_ACCEL_THRESH  1.25f   // g, peak over the 1 g rest bias
