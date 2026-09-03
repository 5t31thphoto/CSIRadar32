// ═══════════════════════════════════════════════════════════════
//  wizard.cpp — cal ceremony state machine
// ═══════════════════════════════════════════════════════════════
#include "wizard.h"
#include "scene.h"
#include "input.h"
#include "peer.h"
#include <Arduino.h>

// ── Script table (STEREO) ─────────────────────────────────────
// PROBE runs this; ANCHOR observes and captures kernel samples for
// each of PROBE's declared positions.  The user carries PROBE.
// Timing is generous — we prefer stable data over speed.
static const WizardStep SCRIPT_STEREO[] = {
    { STEP_INTRO, LM_RX, LM_RX, 0, 0,
      "CAL WALK",
      "You'll visit\n9 landmarks.\n~100 seconds.\nPress RIGHT." },
    { STEP_STAND, LM_RX, LM_RX, 4000, 6000,
      "AT RX",
      "Stand next to\nANCHOR.\nHold still." },
    { STEP_WALK, LM_RX, LM_BEACON_1, 2000, 15000,
      "WALK -> B1",
      "Walk to\nBEACON 1.\nPress RIGHT\non arrival." },
    { STEP_STAND, LM_BEACON_1, LM_BEACON_1, 4000, 6000,
      "AT B1",
      "Stand next to\nBEACON 1.\nHold still." },
    { STEP_WALK, LM_BEACON_1, LM_BEACON_2, 2000, 15000,
      "WALK -> B2",
      "Walk along\nedge to\nBEACON 2.\nRIGHT to advance." },
    { STEP_STAND, LM_BEACON_2, LM_BEACON_2, 4000, 6000,
      "AT B2",
      "Stand next to\nBEACON 2.\nHold still." },
    { STEP_WALK, LM_BEACON_2, LM_BEACON_3, 2000, 15000,
      "WALK -> B3",
      "Walk along\nedge to\nBEACON 3.\nRIGHT to advance." },
    { STEP_STAND, LM_BEACON_3, LM_BEACON_3, 4000, 6000,
      "AT B3",
      "Stand next to\nBEACON 3.\nHold still." },
    { STEP_WALK, LM_BEACON_3, LM_CENTROID, 2000, 15000,
      "WALK -> CENTER",
      "Walk to\nthe centroid\n(middle of\nthe triangle)." },
    { STEP_STAND, LM_CENTROID, LM_CENTROID, 4000, 6000,
      "AT CENTER",
      "Hold still\nat centroid." },
    { STEP_ROTATE, LM_CENTROID, LM_CENTROID, 10000, 12000,
      "ROTATE 360",
      "Rotate slowly\nin place\nover 10 sec.\nStart facing B1." },
    { STEP_WALK, LM_CENTROID, LM_MID_12, 2000, 12000,
      "WALK -> MID12",
      "Walk to midpoint\nof edge\nB1 - B2." },
    { STEP_STAND, LM_MID_12, LM_MID_12, 3000, 5000,
      "AT MID12",
      "Hold still\nmidway between\nB1 and B2." },
    { STEP_WALK, LM_MID_12, LM_MID_23, 2000, 12000,
      "WALK -> MID23",
      "Walk to midpoint\nof edge\nB2 - B3." },
    { STEP_STAND, LM_MID_23, LM_MID_23, 3000, 5000,
      "AT MID23",
      "Hold still\nmidway between\nB2 and B3." },
    { STEP_WALK, LM_MID_23, LM_MID_13, 2000, 12000,
      "WALK -> MID13",
      "Walk to midpoint\nof edge\nB1 - B3." },
    { STEP_STAND, LM_MID_13, LM_MID_13, 3000, 5000,
      "AT MID13",
      "Hold still\nmidway between\nB1 and B3." },
    { STEP_WALK, LM_MID_13, LM_OPPOSITE_RX, 2000, 15000,
      "WALK -> OPP RX",
      "Walk PAST the RX\nto opposite side\nof the triangle." },
    { STEP_STAND, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 4000, 6000,
      "AT OPP",
      "Hold still\nopposite RX.\n(behind sensor)" },
    { STEP_WALK, LM_OPPOSITE_RX, LM_RX, 2000, 15000,
      "WALK -> RX",
      "Return to RX.\nLoop closes here." },
    { STEP_STAND, LM_RX, LM_RX, 3000, 5000,
      "AT RX",
      "Loop closure.\nHold still 3 sec.\nDone!" },
    { STEP_END, LM_RX, LM_RX, 0, 0, "DONE", "" },
};

// ── Script table (SOLO) ───────────────────────────────────────
// User holds the single T-Display against their chest.  The
// receiver moves with them, which is fundamentally a degraded
// cal — but we do our best.  Same landmarks; empty-room baseline
// pre-cal has the user leave the T-Display at the centroid.
static const WizardStep SCRIPT_SOLO[] = {
    { STEP_INTRO, LM_RX, LM_RX, 0, 0,
      "SOLO CAL",
      "Hold the\nT-Display near\nyour chest during\nthe whole walk." },
    { STEP_STAND, LM_BEACON_1, LM_BEACON_1, 4000, 6000,
      "AT B1", "Stand next to\nBEACON 1." },
    { STEP_WALK, LM_BEACON_1, LM_BEACON_2, 2000, 15000,
      "-> B2", "Walk to B2." },
    { STEP_STAND, LM_BEACON_2, LM_BEACON_2, 4000, 6000,
      "AT B2", "Stand next to B2." },
    { STEP_WALK, LM_BEACON_2, LM_BEACON_3, 2000, 15000,
      "-> B3", "Walk to B3." },
    { STEP_STAND, LM_BEACON_3, LM_BEACON_3, 4000, 6000,
      "AT B3", "Stand next to B3." },
    { STEP_WALK, LM_BEACON_3, LM_CENTROID, 2000, 15000,
      "-> CENTER", "Walk to centroid." },
    { STEP_STAND, LM_CENTROID, LM_CENTROID, 4000, 6000,
      "AT CENTER", "Hold still." },
    { STEP_ROTATE, LM_CENTROID, LM_CENTROID, 10000, 12000,
      "ROTATE 360", "Rotate in place\nover 10 sec." },
    { STEP_WALK, LM_CENTROID, LM_MID_12, 2000, 12000,
      "-> MID12", "Walk to midpoint\nB1-B2." },
    { STEP_STAND, LM_MID_12, LM_MID_12, 3000, 5000, "AT MID12", "Hold still." },
    { STEP_WALK, LM_MID_12, LM_MID_23, 2000, 12000, "-> MID23", "Walk to mid B2-B3." },
    { STEP_STAND, LM_MID_23, LM_MID_23, 3000, 5000, "AT MID23", "Hold still." },
    { STEP_WALK, LM_MID_23, LM_MID_13, 2000, 12000, "-> MID13", "Walk to mid B1-B3." },
    { STEP_STAND, LM_MID_13, LM_MID_13, 3000, 5000, "AT MID13", "Hold still." },
    { STEP_END, LM_RX, LM_RX, 0, 0, "DONE", "" },
};

// ── Module state ──────────────────────────────────────────────
static const WizardStep *s_script = nullptr;
static int      s_script_len = 0;
static int      s_cur_idx = 0;
static bool     s_active = false;
static bool     s_finished = false;
static uint32_t s_step_enter_ms = 0;
static bool     s_capture_started = false;

// Enter a step: dispatch to scene begin/end hooks based on kind.
static void enter_step(int idx) {
    s_cur_idx = idx;
    s_step_enter_ms = millis();
    s_capture_started = false;
    const WizardStep &s = s_script[idx];
    Serial.printf("[wizard] step %d/%d kind=%d lm_a=%u lm_b=%u title=%s\n",
                  idx, s_script_len, (int)s.kind,
                  (unsigned)s.landmark_a, (unsigned)s.landmark_b,
                  s.title ? s.title : "-");
    // Tag scene so PROBE peer packets carry this step's provenance.
    // For WALK, landmark_id is 0xFF (transit); for STAND/ROTATE it's
    // the landmark we're at.
    uint8_t lm_tag = (s.kind == STEP_WALK) ? 0xFF : (uint8_t)s.landmark_a;
    scene_cal_note_step((uint8_t)idx, lm_tag);
    switch (s.kind) {
        case STEP_STAND:  scene_begin_landmark_capture(s.landmark_a); s_capture_started = true; break;
        case STEP_WALK:   scene_begin_transit_capture(s.landmark_a, s.landmark_b); s_capture_started = true; break;
        case STEP_ROTATE: scene_begin_rotate_capture(s.landmark_a); s_capture_started = true; break;
        case STEP_INTRO:
        case STEP_PLACE_ANCHOR:
        case STEP_EMPTY_ROOM:
        case STEP_END:
            break;
    }
}

// Exit a step: close any open capture window.
static void exit_step(int idx) {
    const WizardStep &s = s_script[idx];
    switch (s.kind) {
        case STEP_STAND:  if (s_capture_started) scene_end_landmark_capture(); break;
        case STEP_WALK:   if (s_capture_started) scene_end_transit_capture(); break;
        case STEP_ROTATE: if (s_capture_started) scene_end_rotate_capture(); break;
        default: break;
    }
    s_capture_started = false;
}

// ── Public API ────────────────────────────────────────────────
void wizard_begin(CalMode mode) {
    if (mode == CAL_MODE_STEREO) {
        s_script = SCRIPT_STEREO;
        s_script_len = sizeof(SCRIPT_STEREO) / sizeof(SCRIPT_STEREO[0]);
    } else {
        s_script = SCRIPT_SOLO;
        s_script_len = sizeof(SCRIPT_SOLO) / sizeof(SCRIPT_SOLO[0]);
    }
    s_active = true;
    s_finished = false;
    s_cur_idx = 0;
    enter_step(0);
    Serial.printf("[wizard] begin mode=%d steps=%d\n", (int)mode, s_script_len);
}

void wizard_abort() {
    if (s_active) exit_step(s_cur_idx);
    s_active = false;
    s_finished = false;
    Serial.println("[wizard] abort");
}

bool wizard_active()   { return s_active; }
bool wizard_finished() { return s_finished; }

const WizardStep *wizard_current_step() {
    if (!s_active || s_cur_idx >= s_script_len) return nullptr;
    return &s_script[s_cur_idx];
}
int  wizard_current_index()  { return s_cur_idx; }
int  wizard_total_steps()    { return s_script_len; }

uint32_t wizard_step_elapsed_ms() {
    return millis() - s_step_enter_ms;
}

uint32_t wizard_step_min_remaining_ms() {
    if (!s_active) return 0;
    const WizardStep &s = s_script[s_cur_idx];
    uint32_t el = millis() - s_step_enter_ms;
    if (el >= s.min_duration_ms) return 0;
    return s.min_duration_ms - el;
}

float wizard_overall_progress() {
    if (!s_active || s_script_len == 0) return 0;
    return (float)s_cur_idx / (float)(s_script_len - 1);
}

bool wizard_try_advance() {
    if (!s_active) return false;
    if (s_cur_idx >= s_script_len - 1) return false;
    const WizardStep &s = s_script[s_cur_idx];
    uint32_t el = millis() - s_step_enter_ms;
    if (el < s.min_duration_ms) return false;
    exit_step(s_cur_idx);
    int next = s_cur_idx + 1;
    if (s_script[next].kind == STEP_END) {
        s_active = false;
        s_finished = true;
        s_cur_idx = next;
        Serial.println("[wizard] finished");
        return true;
    }
    enter_step(next);
    return true;
}

void wizard_tick() {
    if (!s_active) return;
    const WizardStep &s = s_script[s_cur_idx];
    uint32_t el = millis() - s_step_enter_ms;

    // Auto-advance on max_duration
    if (s.max_duration_ms > 0 && el >= s.max_duration_ms) {
        wizard_try_advance();
        return;
    }

    // Manual advance on RIGHT button press (allowed after min_duration).
    // We check via input.h edges — see input.h wasShortPressed/wasLongPressed.
    if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT)) {
        wizard_try_advance();
        return;
    }

    // LEFT press = redo current step (re-enter without advancing).
    // Useful if the user realized they weren't at the right place.
    if (wasShortPressed(BTN_LEFT)) {
        exit_step(s_cur_idx);
        enter_step(s_cur_idx);
    }
}

void wizard_jump_to_step(int idx) {
    if (!s_script) return;
    if (idx < 0 || idx >= s_script_len) return;
    if (s_active) exit_step(s_cur_idx);
    if (s_script[idx].kind == STEP_END) {
        s_active = false;
        s_finished = true;
        s_cur_idx = idx;
        Serial.printf("[wizard] jump to END (from step hint)\n");
        return;
    }
    if (!s_active) { s_active = true; s_finished = false; }
    enter_step(idx);
}
