// ═══════════════════════════════════════════════════════════════
//  wizard.h — data-driven cal ceremony state machine.
//
//  A calibration walk is a script of WizardStep entries the user
//  walks through.  Each step has a kind (STAND / WALK / ROTATE /
//  intro / empty-room), a landmark reference or pair, timing bounds,
//  and text shown on the PROBE screen.  Wizard advances through the
//  script based on time + button presses; on entering / leaving each
//  step it calls the appropriate scene_begin_*_capture / end_capture
//  hooks so the scene module builds its kernel.
//
//  Kept as pure data + a small step-index integer so behavior is
//  easy to reason about, easy to test, easy to modify by editing
//  the script table.
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "config.h"

struct WizardStep {
    StepKind    kind;
    LandmarkId  landmark_a;
    LandmarkId  landmark_b;       // WALK only, else = landmark_a
    uint16_t    min_duration_ms;  // 0 = press-to-advance immediately allowed
    uint16_t    max_duration_ms;  // 0 = no auto-advance
    const char *title;
    const char *instruction;
};

// Lifecycle
void wizard_begin(CalMode mode);           // start at step 0 for the given mode
void wizard_abort();
bool wizard_active();
bool wizard_finished();

// Called from the main loop while ST_CAL_LANDMARK_WALK is active.
// Consumes button state (via input.h globals) and time to drive the
// step-by-step progression.  Invokes scene::begin/end hooks as needed.
void wizard_tick();

// Try to advance to the next step (called on button press or timer).
// Returns true if advance succeeded, false if step blocks advance
// (still waiting for min_duration).
bool wizard_try_advance();

// Accessors for the UI to render the current step.
const WizardStep *wizard_current_step();
int  wizard_current_index();
int  wizard_total_steps();
uint32_t wizard_step_elapsed_ms();
uint32_t wizard_step_min_remaining_ms();

// Progress fraction 0..1 across the whole script (for a top progress bar).
float wizard_overall_progress();

// Force-jump to a specific step index (used by ANCHOR to follow PROBE's
// step hints via peer commands).  Skips button/timing checks; performs
// the same exit_step/enter_step housekeeping wizard_try_advance would.
void wizard_jump_to_step(int idx);
