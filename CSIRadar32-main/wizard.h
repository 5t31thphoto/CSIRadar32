// ═══════════════════════════════════════════════════════════════
//  wizard.h — cal ceremony state machine (v0.4)
//
//  A calibration walk is a script of WizardStep entries.  Each step
//  has a kind (STAND / WALK / ROTATE / INTRO / END), landmark refs,
//  timing bounds, and text shown on the PROBE screen.
//
//  v0.4 change from v0.3:
//    Each step has a sub-phase (WP_ARM → WP_CAPTURE → WP_READY).
//    WALK  : WP_ARM (waiting for BEGIN press)
//          → WP_CAPTURE (transit capture running, waiting for ARRIVED)
//          → advances immediately (no WP_READY).
//    STAND : WP_CAPTURE (landmark capture running with countdown)
//          → WP_READY (countdown done, waiting for NEXT press).
//    ROTATE: same as STAND.
//    INTRO : WP_READY only (waiting for NEXT press).
//
//    No auto-advance from max_duration_ms anymore — every advance is
//    a button press.  Countdown timers only gate WHEN the button is
//    accepted (during STAND/ROTATE hold-still window).
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "config.h"

// Sub-phase inside the current step (see file header for the state
// machine per step kind).
enum WizardPhase : uint8_t {
    WP_ARM,        // waiting for user to press BEGIN (WALK only)
    WP_CAPTURE,    // capture window open, waiting for the "ok" press
    WP_READY,      // capture closed / not needed; waiting to advance
};

struct WizardStep {
    StepKind    kind;
    LandmarkId  landmark_a;
    LandmarkId  landmark_b;       // WALK only, else = landmark_a
    uint16_t    hold_ms;          // STAND/ROTATE: countdown length before
                                  //   NEXT is accepted (was min_duration).
                                  //   Ignored for WALK/INTRO.
    const char *title;
    const char *instruction;      // shown during WP_CAPTURE / WP_ARM
    const char *ready_prompt;     // shown during WP_READY (STAND/ROTATE
                                  //   after countdown; INTRO always)
};

// Lifecycle
void wizard_begin(CalMode mode);
void wizard_abort();
bool wizard_active();
bool wizard_finished();

// Called from main loop while ST_CAL_LANDMARK_WALK is active.
// Reads button edges via input.h and drives the sub-phase / step
// progression, calling scene_begin/end hooks as it goes.
void wizard_tick();

// Programmatic advance (used by peer step-sync).  Advances one step;
// returns true on success.  Respects WP_CAPTURE end for the current
// step so we don't drop a half-open capture window.
bool wizard_try_advance();

// Accessors for the UI.
const WizardStep *wizard_current_step();
int  wizard_current_index();
int  wizard_total_steps();
uint32_t wizard_step_elapsed_ms();
WizardPhase wizard_current_phase();

// For countdown display: how much of hold_ms remains during WP_CAPTURE
// on STAND/ROTATE.  Returns 0 during WP_ARM/WP_READY.
uint32_t wizard_hold_remaining_ms();

// Progress fraction 0..1 across the whole script.
float wizard_overall_progress();

// Force-jump to a specific step index (used by ANCHOR to follow PROBE).
// Skips button/timing checks; performs entry/exit housekeeping.  Always
// lands the ANCHOR in the equivalent phase of the target step.
void wizard_jump_to_step(int idx);
