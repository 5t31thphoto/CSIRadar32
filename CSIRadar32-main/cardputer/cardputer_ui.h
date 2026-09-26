#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CARDPUTER UI — 240x135 landscape
// ═══════════════════════════════════════════════════════════════
//
//  ── THIS IS A ROTATION, NOT A RESCALE ────────────────────────
//
//      T-Display-S3   170 x 320   portrait, tall and narrow
//      Cardputer-Adv  240 x 135   landscape, wide and short
//
//  Shrinking the T-Display layout to fit would give 135 pixels of
//  vertical space to a design built around a tall column of stacked
//  rows -- roughly four lines of text where there were twelve.  Every
//  screen has to become a wide row of columns instead.
//
//  What the landscape shape affords that portrait never could is a
//  PERMANENT SPLIT: the map on the left and live status on the right,
//  both visible at once, always.  On a 170-wide panel that split leaves
//  neither half usable; on a 240-wide one both are comfortable.  So the
//  Cardputer does not merely display the same information in a different
//  shape -- it displays MORE of it simultaneously, which is the right
//  trade for the device that stays in the operator's hand.
//
//  ── ONE CANVAS, DRAWN ONCE ───────────────────────────────────
//
//  M5GFX gives us a sprite; everything renders into it and is pushed in
//  one operation.  Drawing directly to the panel on a device this small
//  produces visible tearing on every frame, and an operator watching a
//  moving track reads tearing as the track jittering.
// ═══════════════════════════════════════════════════════════════
#include "cardputer_platform.h"
#include "cardputer_input.h"
#include "mantis_imu.h"
#include "mantis_alarms.h"
#include "mantis_caps.h"

// Palette, kept deliberately identical in MEANING to the T-Display so an
// operator moving between the two devices does not have to relearn it.
#define CPC_BG        0x0000
#define CPC_INK       0xFFFF
#define CPC_MID       0x8410
#define CPC_DIM       0x4208
#define CPC_LIME      0x07E0
#define CPC_TEAL      0x0679
#define CPC_VIOLET    0xB01F
#define CPC_ALERT     0xF800
#define CPC_WARN      0xFD20
#define CPC_MESH      0xFD60   // amber: the mesh layer, as on the T-Display

// Which screen is showing.  Mirrors the T-Display's states so the shared
// state machine drives both, with the Cardputer-only screens appended.
typedef enum : uint8_t {
    CPS_SPLASH = 0,
    CPS_DISCOVERY,
    CPS_LINK,          // what the anchor / stereo pair is doing
    CPS_DASHBOARD,     // map + status split
    CPS_CAL_WALK,      // with the IMU compass
    CPS_TACTICAL,
    CPS_ALARMS,        // Cardputer-only: configurable audio
    CPS_SESSIONS,      // Cardputer-only: SD record / replay
    CPS_MESH,          // beacon health and geometry
    CPS_SETTINGS,
    CPS_COUNT
} CardputerScreen;

static inline const char *cp_screen_name(CardputerScreen s) {
    switch (s) {
        case CPS_SPLASH:    return "MANTIS";
        case CPS_DISCOVERY: return "DISCOVERY";
        case CPS_LINK:      return "LINK";
        case CPS_DASHBOARD: return "RADAR";
        case CPS_CAL_WALK:  return "CAL WALK";
        case CPS_TACTICAL:  return "TACTICAL";
        case CPS_ALARMS:    return "ALARMS";
        case CPS_SESSIONS:  return "SESSIONS";
        case CPS_MESH:      return "MESH";
        case CPS_SETTINGS:  return "SETTINGS";
        default:            return "?";
    }
}

// ── What the device is, right now ─────────────────────────────
//
// The Cardputer supports three deployment shapes and behaves differently
// in each.  Making that an explicit enum rather than a set of booleans
// means the UI can always name the mode, and an operator is never left
// guessing why a screen looks different from yesterday.
typedef enum : uint8_t {
    CPM_SEARCHING = 0,  // looking for anything at all
    CPM_STANDALONE,     // beacons only: our own inference, no anchor
    CPM_PROBE_SOLO,     // one T-Display anchor + us
    CPM_PROBE_STEREO,   // docked stereo pair + us
} CardputerMode;

static inline const char *cp_mode_name(CardputerMode m) {
    switch (m) {
        case CPM_STANDALONE:   return "STANDALONE";
        case CPM_PROBE_SOLO:   return "PROBE+ANCHOR";
        case CPM_PROBE_STEREO: return "PROBE+STEREO";
        default:               return "SEARCHING";
    }
}

// Capability now comes from mantis_caps.h, which both firmwares share.
//
// This file previously derived its own, and two implementations of the
// same question is how a menu ends up offering something the solver
// cannot do -- the Cardputer's version keyed only on mode and beacon
// count, and knew nothing about chord counts, geometry confidence or
// whether a probe existed.
//
// The mode enum survives because it describes the DEPLOYMENT SHAPE,
// which is what the header line shows and what an operator recognises.
// What that shape can DO is a separate question with one answer.
static inline MantisDeployment cp_deployment(CardputerMode m, uint8_t n_beacons,
                                             bool geometry_known) {
    MantisDeployment d{};
    d.beacons        = n_beacons;
    d.cardputers     = 1;                 // we are one, by definition
    d.geometry_known = geometry_known;
    switch (m) {
        case CPM_PROBE_STEREO: d.tdisplays = 2; d.docked = true;  break;
        case CPM_PROBE_SOLO:   d.tdisplays = 1; d.docked = false; break;
        default:               d.tdisplays = 0; d.docked = false; break;
    }
    return d;
}

// ── Layout helpers ────────────────────────────────────────────
// Every screen asks these rather than hardcoding, so the split can move
// in one place if the panel ever changes.
static inline int cp_status_x()  { return CP_SPLIT_X + 4; }
static inline int cp_status_w()  { return CP_SCREEN_W - CP_SPLIT_X - 6; }
static inline int cp_row_y(int i){ return CP_CONTENT_Y + 2 + i * 11; }
static inline int cp_rows()      { return (CP_CONTENT_H - 4) / 11; }

// The IMU compass needle, as an angle to draw.
//
// Two needles, exactly as on the T-Display: green for the pace the
// script asks for, red for what is MEASURED.  The difference is that
// here the red one comes from the IMU rather than from RF, so it is a
// genuine second opinion instead of the estimate grading itself.
typedef struct {
    float pace_rad;      // where the script says you should be facing
    float imu_rad;       // where the IMU says you ARE facing
    float rf_rad;        // where RF thinks you are facing
    bool  have_imu, have_rf;
    float disagreement;  // 0..1, imu vs rf
} CardputerCompass;
