#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CARDPUTER AUDIO
//  Configurable alarms, and one rule about them
// ═══════════════════════════════════════════════════════════════
//
//  ── AN ALARM THAT DOES NOT SOUND IS THE WORST FAILURE HERE ───
//
//  Everything else in this system degrades usefully: a lost report is
//  healed next frame, a blind spot is reported as a blind spot, a
//  disagreement is shown rather than averaged away.  An alarm is
//  different.  If it does not sound, the operator learns nothing and has
//  no way to know they learned nothing.
//
//  So every alarm has a compiled-in TONE fallback, and the SD file is an
//  upgrade rather than a dependency.  No card, wrong filename, corrupt
//  MP3, card pulled mid-session -- in every case the device still makes
//  the noise.  It just makes a plainer one, and says so on screen.
//
//  ── WHY FILES AT ALL ─────────────────────────────────────────
//
//  Different alarms need to be distinguishable without looking at the
//  screen, and what is distinguishable depends on the environment and
//  the operator.  A tone that reads as urgent in a quiet room vanishes
//  in a loud one.  Letting the operator drop their own files in
//  /mantis/alarms means that is tunable in the field with no rebuild.
//
//  ── DEBOUNCE IS PART OF THE ALARM, NOT A DETAIL ──────────────
//
//  A target that flickers across a detection threshold would otherwise
//  produce a stutter of alarms, which is both useless and actively
//  harmful: an operator learns within a minute to ignore an alarm that
//  cries constantly, and then misses the real one.  Each alarm carries
//  its own minimum re-trigger interval, chosen from what the event
//  means rather than from a single global number.
// ═══════════════════════════════════════════════════════════════
#include "cardputer_platform.h"

typedef struct {
    const char *file;        // SD path, relative to CP_ALARM_DIR
    uint16_t    tone_hz;     // fallback tone
    uint16_t    tone_ms;
    uint8_t     repeats;
    uint32_t    min_gap_ms;  // debounce, per alarm
    const char *label;
} CardputerAlarmDef;

// Frequencies chosen to be distinguishable from each other by ear, not
// merely different numbers: roughly a musical fourth apart, and the two
// most urgent are the two highest.
static const CardputerAlarmDef CP_ALARMS[CPA_COUNT] = {
    /* CPA_NONE         */ { nullptr,          0,   0, 0,     0, "" },
    /* CPA_PERIMETER    */ { "perimeter.mp3", 1760, 180, 3,  4000, "PERIMETER" },
    /* CPA_NEW_PRESENCE */ { "presence.mp3",  1320, 220, 2,  6000, "NEW CONTACT" },
    /* CPA_MOTION       */ { "motion.mp3",     880, 120, 1, 10000, "MOTION" },
    /* CPA_TRIPWIRE     */ { "tripwire.mp3",  2093, 150, 4,  3000, "TRIPWIRE" },
    /* CPA_MESH_FAULT   */ { "meshfault.mp3",  440, 400, 2, 30000, "MESH FAULT" },
};

typedef struct {
    bool     enabled[CPA_COUNT];
    uint8_t  volume;                   // 0..255
    uint32_t last_fired_ms[CPA_COUNT];
    bool     sd_present;
    bool     file_ok[CPA_COUNT];       // did the file actually load?
    uint32_t fired_count[CPA_COUNT];
    bool     muted;                    // operator override, whole device
} CardputerAudio;

static inline void cp_audio_begin(CardputerAudio *a) {
    *a = CardputerAudio{};
    a->volume = 160;
    // Everything on by default EXCEPT general motion, which in a busy
    // room fires constantly and trains the operator to ignore the
    // speaker entirely -- taking the alarms that matter down with it.
    for (int i = 1; i < CPA_COUNT; i++) a->enabled[i] = true;
    a->enabled[CPA_MOTION] = false;
}

// Should this alarm fire right now?
//
// Separated from the sounding so the decision is testable without
// hardware, and so the UI can show WHY an alarm did not fire -- muted,
// disabled, or still inside its debounce window are three different
// answers and an operator wondering about a silent device needs the
// right one.
typedef enum : uint8_t {
    CPF_FIRE = 0,
    CPF_MUTED,
    CPF_DISABLED,
    CPF_DEBOUNCED,
    CPF_INVALID,
} CardputerFireResult;

static inline CardputerFireResult cp_audio_should_fire(const CardputerAudio *a,
                                                       CardputerAlarm k,
                                                       uint32_t now_ms) {
    if (k <= CPA_NONE || k >= CPA_COUNT) return CPF_INVALID;
    if (a->muted)                        return CPF_MUTED;
    if (!a->enabled[k])                  return CPF_DISABLED;
    const uint32_t last = a->last_fired_ms[k];
    // A zero last-fired means never fired, which must not be read as
    // "fired at time zero" -- at boot that would debounce the first
    // alarm of the session, exactly when it matters most.
    if (a->fired_count[k] > 0 && (now_ms - last) < CP_ALARMS[k].min_gap_ms)
        return CPF_DEBOUNCED;
    return CPF_FIRE;
}

static inline void cp_audio_mark_fired(CardputerAudio *a, CardputerAlarm k,
                                       uint32_t now_ms) {
    if (k <= CPA_NONE || k >= CPA_COUNT) return;
    a->last_fired_ms[k] = now_ms;
    a->fired_count[k]++;
}

// What the operator is told about an alarm's readiness.
//
// "SD" and "tone" are both working states and the screen says which, so
// a missing file is visible before the event rather than discovered
// during it.
static inline const char *cp_audio_source(const CardputerAudio *a, CardputerAlarm k) {
    if (k <= CPA_NONE || k >= CPA_COUNT) return "-";
    if (!a->enabled[k]) return "off";
    if (a->sd_present && a->file_ok[k]) return "SD";
    return "tone";
}

static inline const char *cp_fire_reason(CardputerFireResult r) {
    switch (r) {
        case CPF_FIRE:      return "fired";
        case CPF_MUTED:     return "device muted";
        case CPF_DISABLED:  return "alarm disabled";
        case CPF_DEBOUNCED: return "too soon";
        default:            return "invalid";
    }
}
