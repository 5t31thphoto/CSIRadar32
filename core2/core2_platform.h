#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CORE2 PLATFORM
//  The same probe role as the Cardputer, on a bigger screen
// ═══════════════════════════════════════════════════════════════
//
//  ── WHAT IS THE SAME ─────────────────────────────────────────
//
//  Everything that matters: full receiver, PROBE role only, never an
//  anchor.  It runs the identical mantis inference stack and the
//  identical shared capability gating, so a deployment does not care
//  which hand-held device is present.
//
//  The IMU work is shared outright -- cardputer_imu.h is pure
//  arithmetic over accel and gyro samples and has no Cardputer in it.
//  Duplicating it for a second board would give two step counters that
//  drift apart.
//
//  ── WHAT IS DIFFERENT ────────────────────────────────────────
//
//    display    320x240 against the Cardputer's 240x135.  Nearly 2.4x
//               the pixels, which buys a genuinely larger map rather
//               than just bigger text.
//    input      three CAPACITIVE ZONES (BtnA/B/C) plus a touchscreen,
//               against 56 mechanical keys.  Fewer discrete controls,
//               but direct pointing -- a menu row can simply be touched.
//    IMU        MPU6886 on v1.0/v1.1, BMI270 on v1.3.
//    PMIC       AXP192 on v1.0/v1.3, AXP2101 on v1.1.
//
//  THE LAST TWO ARE WHY THIS FILE NAMES NO CHIPS.
//
//  M5Unified's M5.Imu and M5.Power abstract both, and reading MPU6886
//  registers directly -- which plenty of Core2 code does -- produces
//  firmware that works on one revision of the same product and silently
//  fails on another.  The board revision is not something a user should
//  have to know before flashing.
//
//  Toolchain, from the M5Stack documentation:
//    board = m5stack-core2, partitions default_16MB
//    M5Unified + M5GFX, -DBOARD_HAS_PSRAM
// ═══════════════════════════════════════════════════════════════
#include <stdint.h>

#define MANTIS_PLATFORM_CORE2   1
#define MANTIS_ROLE_LOCKED_PROBE 1

// ── Screen ────────────────────────────────────────────────────
#define C2_SCREEN_W        320
#define C2_SCREEN_H        240
#define C2_HEADER_H         22
#define C2_FOOTER_H         26      // taller: it labels three touch zones
#define C2_CONTENT_Y       (C2_HEADER_H)
#define C2_CONTENT_H       (C2_SCREEN_H - C2_HEADER_H - C2_FOOTER_H)

// The extra width pays for a wider status column than the Cardputer can
// afford, so target details are readable without cycling views.
#define C2_SPLIT_X         (C2_SCREEN_W * 62 / 100)
#define C2_MAP_CX          (C2_SPLIT_X / 2)
#define C2_MAP_CY          (C2_CONTENT_Y + C2_CONTENT_H / 2)
#define C2_MAP_R           (C2_CONTENT_H / 2 - 6)

// Menu row height.  Named here because both the renderer and the touch
// hit-test derive from it -- two places computing it separately is how
// a tap lands on the row above the one it appears to hit.
#define C2_ROW_H           18
#define C2_ROWS            (C2_CONTENT_H / C2_ROW_H)

// ── The three capacitive zones ────────────────────────────────
// They sit BELOW the glass, in the footer strip, so the footer has to
// be tall enough to label all three and the labels must line up with
// the physical dots.  Getting this wrong is the classic Core2 UI bug:
// labels that do not sit over the zone they describe.
#define C2_BTN_W           (C2_SCREEN_W / 3)
#define C2_BTN_A_CX        (C2_BTN_W / 2)
#define C2_BTN_B_CX        (C2_BTN_W + C2_BTN_W / 2)
#define C2_BTN_C_CX        (2 * C2_BTN_W + C2_BTN_W / 2)

// ── Touch ─────────────────────────────────────────────────────
// A touch inside the content area selects; a touch in the footer strip
// is a button and is handled as one.
//
// Minimum drag before a touch counts as a swipe rather than a tap.
// Below this, finger wobble on a capacitive panel turns every tap into
// a tiny swipe and list selection becomes unusable.
#define C2_SWIPE_MIN_PX    28
#define C2_TAP_MAX_MS      400

typedef enum : uint8_t {
    C2T_NONE = 0,
    C2T_TAP,
    C2T_SWIPE_UP,
    C2T_SWIPE_DOWN,
    C2T_SWIPE_LEFT,
    C2T_SWIPE_RIGHT,
} Core2Gesture;

// ── Audio ─────────────────────────────────────────────────────
// NS4168 I2S amplifier and a speaker, reached through M5.Speaker.  The
// alarm definitions and the MP3 decode path are shared with the
// Cardputer: same files on the SD card, same fallback tones, same
// guarantee that an alarm always makes a noise.
#define C2_ALARM_DIR "/mantis/alarms"

// ── SD ────────────────────────────────────────────────────────
// Unlike the Cardputer, the Core2's SD is on the standard VSPI pins and
// M5Unified brings the bus up, so plain SD.begin(GPIO_NUM_4) works.
// Recorded here explicitly because the Cardputer needing explicit pins
// and this one not is exactly the kind of difference that gets
// copy-pasted wrong.
#define C2_SD_CS_PIN       4

// IMU orientation: on the Core2 the screen-normal axis is Z, same as the
// Cardputer, so the shared turn logic applies unchanged.
#define C2_STRIDE_M_DEFAULT  0.72f
