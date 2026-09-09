// ═══════════════════════════════════════════════════════════════
//  wizard.cpp — cal ceremony state machine (v0.4)
//
//  Three-phase per step (WP_ARM, WP_CAPTURE, WP_READY) so the walk-
//  to-a-beacon workflow is:
//    press BEGIN → walk → press ARRIVED → countdown → press NEXT.
//  No auto-advance from a timer — every step advance is a button
//  press.  Countdown timers gate WHEN a NEXT press is accepted, not
//  whether it's required.
// ═══════════════════════════════════════════════════════════════
#include "wizard.h"
#include "scene.h"
#include "input.h"
#include "peer.h"
#include <Arduino.h>

// ── Script tables, one per beacon count (v0.8) ────────────────
// Every script follows the same shape:
//   INTRO → stand at RX → (walk to Bi, stand at Bi) for i = 1..N
//   → walk to centroid, stand, rotate 360
//   → (walk to perimeter midpoint, stand) for each of the N edges
//   → walk past RX to the far side, stand → return to RX, stand → END
// which is 4N + 10 entries.  N=3 → 22, N=4 → 26, N=5 → 30, N=6 → 34.
//
// SCRIPT_STEREO_3 / SCRIPT_SOLO_3 are the v0.7 scripts VERBATIM —
// text, ordering and hold_ms all unchanged, because the docs and the
// user's muscle memory both assume them.  Do not "tidy" them to match
// the generated ones.

// ── Script table (STEREO) ─────────────────────────────────────
// PROBE runs this; ANCHOR observes and captures kernel samples for
// each of PROBE's declared positions.  hold_ms is the "hold still"
// countdown for STAND/ROTATE (was min_duration in v0.3).
static const WizardStep SCRIPT_STEREO_3[] = {
    { STEP_INTRO,  LM_RX, LM_RX, 0,
      "CAL WALK",
      "You'll visit\n9 landmarks and\nturn in place 3x.\nGates: BEGIN,\nARRIVED, NEXT.",
      "Press RIGHT\nto start." },

    { STEP_STAND,  LM_RX, LM_RX, 4000,
      "AT RX",
      "Stand next to\nANCHOR.\nHold still.",
      "OK - press\nRIGHT for next." },
    { STEP_WALK,   LM_RX, LM_BEACON_1, 0,
      "-> B1", "Walk to\nBEACON 1.", nullptr },

    { STEP_STAND,  LM_BEACON_1, LM_BEACON_1, 4000,
      "AT B1", "Hold still\nat BEACON 1.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_1, LM_BEACON_2, 0,
      "-> B2", "Walk to\nBEACON 2.", nullptr },

    { STEP_STAND,  LM_BEACON_2, LM_BEACON_2, 4000,
      "AT B2", "Hold still\nat BEACON 2.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_2, LM_BEACON_3, 0,
      "-> B3", "Walk to\nBEACON 3.", nullptr },

    { STEP_STAND,  LM_BEACON_3, LM_BEACON_3, 4000,
      "AT B3", "Hold still\nat BEACON 3.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_3, LM_CENTROID, 0,
      "-> CENTER", "Walk to the\ncentroid.", nullptr },

    { STEP_STAND,  LM_CENTROID, LM_CENTROID, 4000,
      "AT CENTER", "Hold still\nat centroid.", "OK - RIGHT." },

    { STEP_ROTATE, LM_CENTROID, LM_CENTROID, 10000,
      "ROTATE 360",
      "Rotate slowly\nin place over\n10 seconds.\nStart facing B1.",
      "Done rotating.\nPress RIGHT." },
    { STEP_WALK,   LM_CENTROID, LM_MID_12, 0,
      "-> MID12", "Walk to midpoint\nof edge B1-B2.", nullptr },

    { STEP_STAND,  LM_MID_12, LM_MID_12, 3000,
      "AT MID12", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_MID_12, LM_MID_12, 10000,
      "ROTATE MID12",
      "Turn slowly all\nthe way around,\n10 seconds.",
      "Done - RIGHT." },
    { STEP_WALK,   LM_MID_12, LM_MID_23, 0,
      "-> MID23", "Walk to midpoint\nof edge B2-B3.", nullptr },

    { STEP_STAND,  LM_MID_23, LM_MID_23, 3000,
      "AT MID23", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_23, LM_MID_13, 0,
      "-> MID13", "Walk to midpoint\nof edge B1-B3.", nullptr },

    { STEP_STAND,  LM_MID_13, LM_MID_13, 3000,
      "AT MID13", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_13, LM_OPPOSITE_RX, 0,
      "-> OUTSIDE",
      "Walk OUT past the\nbeacon ring to the\nfar side. Through a\ndoorway is fine.",
      nullptr },

    { STEP_STAND,  LM_OPPOSITE_RX, LM_OPPOSITE_RX, 4000,
      "AT OUTSIDE", "Hold still\noutside the ring.", "OK - RIGHT." },

    { STEP_ROTATE, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 10000,
      "ROTATE OUT",
      "Turn slowly all\nthe way around,\n10 seconds.\nThis teaches the\nperimeter alert.",
      "Done - RIGHT." },

    { STEP_WALK,   LM_OPPOSITE_RX, LM_RX, 0,
      "-> RX", "Return to RX.\nLoop closes.", nullptr },

    { STEP_STAND,  LM_RX, LM_RX, 3000,
      "AT RX", "Hold still\n(loop close).", "Done!\nPress RIGHT." },

    { STEP_END, LM_RX, LM_RX, 0, "DONE", "", "" },

};

// ── Script table (SOLO) ───────────────────────────────────────
// User holds the single T-Display against their chest.
static const WizardStep SCRIPT_SOLO_3[] = {
    { STEP_INTRO, LM_RX, LM_RX, 0,
      "SOLO CAL",
      "Hold T-Display\nnear your chest\nthroughout the walk.",
      "Press RIGHT." },

    { STEP_STAND, LM_BEACON_1, LM_BEACON_1, 4000,
      "AT B1", "Hold still\nat BEACON 1.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_1, LM_BEACON_2, 0,
      "-> B2", "Walk to B2.", nullptr },
    { STEP_STAND, LM_BEACON_2, LM_BEACON_2, 4000,
      "AT B2", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_2, LM_BEACON_3, 0,
      "-> B3", "Walk to B3.", nullptr },
    { STEP_STAND, LM_BEACON_3, LM_BEACON_3, 4000,
      "AT B3", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_3, LM_CENTROID, 0,
      "-> CENTER", "Walk to centroid.", nullptr },
    { STEP_STAND, LM_CENTROID, LM_CENTROID, 4000,
      "AT CENTER", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_CENTROID, LM_CENTROID, 10000,
      "ROTATE 360",
      "Rotate slowly\nin place over\n10 seconds.",
      "Done - RIGHT." },
    { STEP_WALK,  LM_CENTROID, LM_MID_12, 0,
      "-> MID12", "Walk to mid B1-B2.", nullptr },
    { STEP_STAND, LM_MID_12, LM_MID_12, 3000,
      "AT MID12", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_MID_12, LM_MID_12, 10000,
      "ROTATE MID12", "Turn slowly all\nthe way around.", "Done - RIGHT." },
    { STEP_WALK,  LM_MID_12, LM_MID_23, 0,
      "-> MID23", "Walk to mid B2-B3.", nullptr },
    { STEP_STAND, LM_MID_23, LM_MID_23, 3000,
      "AT MID23", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_23, LM_MID_13, 0,
      "-> MID13", "Walk to mid B1-B3.", nullptr },
    { STEP_STAND, LM_MID_13, LM_MID_13, 3000,
      "AT MID13", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_13, LM_OPPOSITE_RX, 0,
      "-> OUTSIDE", "Walk OUT past the\nbeacon ring.", nullptr },
    { STEP_STAND, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 4000,
      "AT OUTSIDE", "Hold still\noutside the ring.", "OK - RIGHT." },
    { STEP_ROTATE, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 10000,
      "ROTATE OUT", "Turn slowly all\nthe way around.", "Done - RIGHT." },
    { STEP_END, LM_RX, LM_RX, 0, "DONE", "", "" },

};

static const WizardStep SCRIPT_STEREO_4[] = {
    { STEP_INTRO,  LM_RX, LM_RX, 0,
      "CAL WALK",
      "You'll visit\n11 landmarks and\nturn in place 3x.\nGates: BEGIN,\nARRIVED, NEXT.",
      "Press RIGHT\nto start." },

    { STEP_STAND,  LM_RX, LM_RX, 4000,
      "AT RX",
      "Stand next to\nANCHOR.\nHold still.",
      "OK - press\nRIGHT for next." },
    { STEP_WALK,   LM_RX, LM_BEACON_1, 0,
      "-> B1", "Walk to\nBEACON 1.", nullptr },

    { STEP_STAND,  LM_BEACON_1, LM_BEACON_1, 4000,
      "AT B1", "Hold still\nat BEACON 1.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_1, LM_BEACON_2, 0,
      "-> B2", "Walk to\nBEACON 2.", nullptr },

    { STEP_STAND,  LM_BEACON_2, LM_BEACON_2, 4000,
      "AT B2", "Hold still\nat BEACON 2.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_2, LM_BEACON_3, 0,
      "-> B3", "Walk to\nBEACON 3.", nullptr },

    { STEP_STAND,  LM_BEACON_3, LM_BEACON_3, 4000,
      "AT B3", "Hold still\nat BEACON 3.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_3, LM_BEACON_4, 0,
      "-> B4", "Walk to\nBEACON 4.", nullptr },

    { STEP_STAND,  LM_BEACON_4, LM_BEACON_4, 4000,
      "AT B4", "Hold still\nat BEACON 4.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_4, LM_CENTROID, 0,
      "-> CENTER", "Walk to the\ncentroid.", nullptr },

    { STEP_STAND,  LM_CENTROID, LM_CENTROID, 4000,
      "AT CENTER", "Hold still\nat centroid.", "OK - RIGHT." },

    { STEP_ROTATE, LM_CENTROID, LM_CENTROID, 10000,
      "ROTATE 360",
      "Rotate slowly\nin place over\n10 seconds.\nStart facing B1.",
      "Done rotating.\nPress RIGHT." },
    { STEP_WALK,   LM_CENTROID, LM_MID_12, 0,
      "-> MID12", "Walk to midpoint\nof edge B1-B2.", nullptr },

    { STEP_STAND,  LM_MID_12, LM_MID_12, 3000,
      "AT MID12", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_MID_12, LM_MID_12, 10000,
      "ROTATE MID12",
      "Turn slowly all\nthe way around,\n10 seconds.",
      "Done - RIGHT." },
    { STEP_WALK,   LM_MID_12, LM_MID_23, 0,
      "-> MID23", "Walk to midpoint\nof edge B2-B3.", nullptr },

    { STEP_STAND,  LM_MID_23, LM_MID_23, 3000,
      "AT MID23", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_23, LM_MID_34, 0,
      "-> MID34", "Walk to midpoint\nof edge B3-B4.", nullptr },

    { STEP_STAND,  LM_MID_34, LM_MID_34, 3000,
      "AT MID34", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_34, LM_MID_41, 0,
      "-> MID41", "Walk to midpoint\nof edge B4-B1.", nullptr },

    { STEP_STAND,  LM_MID_41, LM_MID_41, 3000,
      "AT MID41", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_41, LM_OPPOSITE_RX, 0,
      "-> OUTSIDE",
      "Walk OUT past the\nbeacon ring to the\nfar side. Through a\ndoorway is fine.",
      nullptr },

    { STEP_STAND,  LM_OPPOSITE_RX, LM_OPPOSITE_RX, 4000,
      "AT OUTSIDE", "Hold still\noutside the ring.", "OK - RIGHT." },

    { STEP_ROTATE, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 10000,
      "ROTATE OUT",
      "Turn slowly all\nthe way around,\n10 seconds.\nThis teaches the\nperimeter alert.",
      "Done - RIGHT." },

    { STEP_WALK,   LM_OPPOSITE_RX, LM_RX, 0,
      "-> RX", "Return to RX.\nLoop closes.", nullptr },

    { STEP_STAND,  LM_RX, LM_RX, 3000,
      "AT RX", "Hold still\n(loop close).", "Done!\nPress RIGHT." },

    { STEP_END, LM_RX, LM_RX, 0, "DONE", "", "" },

};

static const WizardStep SCRIPT_STEREO_5[] = {
    { STEP_INTRO,  LM_RX, LM_RX, 0,
      "CAL WALK",
      "You'll visit\n13 landmarks and\nturn in place 3x.\nGates: BEGIN,\nARRIVED, NEXT.",
      "Press RIGHT\nto start." },

    { STEP_STAND,  LM_RX, LM_RX, 4000,
      "AT RX",
      "Stand next to\nANCHOR.\nHold still.",
      "OK - press\nRIGHT for next." },
    { STEP_WALK,   LM_RX, LM_BEACON_1, 0,
      "-> B1", "Walk to\nBEACON 1.", nullptr },

    { STEP_STAND,  LM_BEACON_1, LM_BEACON_1, 4000,
      "AT B1", "Hold still\nat BEACON 1.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_1, LM_BEACON_2, 0,
      "-> B2", "Walk to\nBEACON 2.", nullptr },

    { STEP_STAND,  LM_BEACON_2, LM_BEACON_2, 4000,
      "AT B2", "Hold still\nat BEACON 2.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_2, LM_BEACON_3, 0,
      "-> B3", "Walk to\nBEACON 3.", nullptr },

    { STEP_STAND,  LM_BEACON_3, LM_BEACON_3, 4000,
      "AT B3", "Hold still\nat BEACON 3.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_3, LM_BEACON_4, 0,
      "-> B4", "Walk to\nBEACON 4.", nullptr },

    { STEP_STAND,  LM_BEACON_4, LM_BEACON_4, 4000,
      "AT B4", "Hold still\nat BEACON 4.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_4, LM_BEACON_5, 0,
      "-> B5", "Walk to\nBEACON 5.", nullptr },

    { STEP_STAND,  LM_BEACON_5, LM_BEACON_5, 4000,
      "AT B5", "Hold still\nat BEACON 5.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_5, LM_CENTROID, 0,
      "-> CENTER", "Walk to the\ncentroid.", nullptr },

    { STEP_STAND,  LM_CENTROID, LM_CENTROID, 4000,
      "AT CENTER", "Hold still\nat centroid.", "OK - RIGHT." },

    { STEP_ROTATE, LM_CENTROID, LM_CENTROID, 10000,
      "ROTATE 360",
      "Rotate slowly\nin place over\n10 seconds.\nStart facing B1.",
      "Done rotating.\nPress RIGHT." },
    { STEP_WALK,   LM_CENTROID, LM_MID_12, 0,
      "-> MID12", "Walk to midpoint\nof edge B1-B2.", nullptr },

    { STEP_STAND,  LM_MID_12, LM_MID_12, 3000,
      "AT MID12", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_MID_12, LM_MID_12, 10000,
      "ROTATE MID12",
      "Turn slowly all\nthe way around,\n10 seconds.",
      "Done - RIGHT." },
    { STEP_WALK,   LM_MID_12, LM_MID_23, 0,
      "-> MID23", "Walk to midpoint\nof edge B2-B3.", nullptr },

    { STEP_STAND,  LM_MID_23, LM_MID_23, 3000,
      "AT MID23", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_23, LM_MID_34, 0,
      "-> MID34", "Walk to midpoint\nof edge B3-B4.", nullptr },

    { STEP_STAND,  LM_MID_34, LM_MID_34, 3000,
      "AT MID34", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_34, LM_MID_45, 0,
      "-> MID45", "Walk to midpoint\nof edge B4-B5.", nullptr },

    { STEP_STAND,  LM_MID_45, LM_MID_45, 3000,
      "AT MID45", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_45, LM_MID_51, 0,
      "-> MID51", "Walk to midpoint\nof edge B5-B1.", nullptr },

    { STEP_STAND,  LM_MID_51, LM_MID_51, 3000,
      "AT MID51", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_51, LM_OPPOSITE_RX, 0,
      "-> OUTSIDE",
      "Walk OUT past the\nbeacon ring to the\nfar side. Through a\ndoorway is fine.",
      nullptr },

    { STEP_STAND,  LM_OPPOSITE_RX, LM_OPPOSITE_RX, 4000,
      "AT OUTSIDE", "Hold still\noutside the ring.", "OK - RIGHT." },

    { STEP_ROTATE, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 10000,
      "ROTATE OUT",
      "Turn slowly all\nthe way around,\n10 seconds.\nThis teaches the\nperimeter alert.",
      "Done - RIGHT." },

    { STEP_WALK,   LM_OPPOSITE_RX, LM_RX, 0,
      "-> RX", "Return to RX.\nLoop closes.", nullptr },

    { STEP_STAND,  LM_RX, LM_RX, 3000,
      "AT RX", "Hold still\n(loop close).", "Done!\nPress RIGHT." },

    { STEP_END, LM_RX, LM_RX, 0, "DONE", "", "" },

};

static const WizardStep SCRIPT_STEREO_6[] = {
    { STEP_INTRO,  LM_RX, LM_RX, 0,
      "CAL WALK",
      "You'll visit\n15 landmarks and\nturn in place 3x.\nGates: BEGIN,\nARRIVED, NEXT.",
      "Press RIGHT\nto start." },

    { STEP_STAND,  LM_RX, LM_RX, 4000,
      "AT RX",
      "Stand next to\nANCHOR.\nHold still.",
      "OK - press\nRIGHT for next." },
    { STEP_WALK,   LM_RX, LM_BEACON_1, 0,
      "-> B1", "Walk to\nBEACON 1.", nullptr },

    { STEP_STAND,  LM_BEACON_1, LM_BEACON_1, 4000,
      "AT B1", "Hold still\nat BEACON 1.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_1, LM_BEACON_2, 0,
      "-> B2", "Walk to\nBEACON 2.", nullptr },

    { STEP_STAND,  LM_BEACON_2, LM_BEACON_2, 4000,
      "AT B2", "Hold still\nat BEACON 2.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_2, LM_BEACON_3, 0,
      "-> B3", "Walk to\nBEACON 3.", nullptr },

    { STEP_STAND,  LM_BEACON_3, LM_BEACON_3, 4000,
      "AT B3", "Hold still\nat BEACON 3.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_3, LM_BEACON_4, 0,
      "-> B4", "Walk to\nBEACON 4.", nullptr },

    { STEP_STAND,  LM_BEACON_4, LM_BEACON_4, 4000,
      "AT B4", "Hold still\nat BEACON 4.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_4, LM_BEACON_5, 0,
      "-> B5", "Walk to\nBEACON 5.", nullptr },

    { STEP_STAND,  LM_BEACON_5, LM_BEACON_5, 4000,
      "AT B5", "Hold still\nat BEACON 5.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_5, LM_BEACON_6, 0,
      "-> B6", "Walk to\nBEACON 6.", nullptr },

    { STEP_STAND,  LM_BEACON_6, LM_BEACON_6, 4000,
      "AT B6", "Hold still\nat BEACON 6.", "OK - RIGHT." },
    { STEP_WALK,   LM_BEACON_6, LM_CENTROID, 0,
      "-> CENTER", "Walk to the\ncentroid.", nullptr },

    { STEP_STAND,  LM_CENTROID, LM_CENTROID, 4000,
      "AT CENTER", "Hold still\nat centroid.", "OK - RIGHT." },

    { STEP_ROTATE, LM_CENTROID, LM_CENTROID, 10000,
      "ROTATE 360",
      "Rotate slowly\nin place over\n10 seconds.\nStart facing B1.",
      "Done rotating.\nPress RIGHT." },
    { STEP_WALK,   LM_CENTROID, LM_MID_12, 0,
      "-> MID12", "Walk to midpoint\nof edge B1-B2.", nullptr },

    { STEP_STAND,  LM_MID_12, LM_MID_12, 3000,
      "AT MID12", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_MID_12, LM_MID_12, 10000,
      "ROTATE MID12",
      "Turn slowly all\nthe way around,\n10 seconds.",
      "Done - RIGHT." },
    { STEP_WALK,   LM_MID_12, LM_MID_23, 0,
      "-> MID23", "Walk to midpoint\nof edge B2-B3.", nullptr },

    { STEP_STAND,  LM_MID_23, LM_MID_23, 3000,
      "AT MID23", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_23, LM_MID_34, 0,
      "-> MID34", "Walk to midpoint\nof edge B3-B4.", nullptr },

    { STEP_STAND,  LM_MID_34, LM_MID_34, 3000,
      "AT MID34", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_34, LM_MID_45, 0,
      "-> MID45", "Walk to midpoint\nof edge B4-B5.", nullptr },

    { STEP_STAND,  LM_MID_45, LM_MID_45, 3000,
      "AT MID45", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_45, LM_MID_56, 0,
      "-> MID56", "Walk to midpoint\nof edge B5-B6.", nullptr },

    { STEP_STAND,  LM_MID_56, LM_MID_56, 3000,
      "AT MID56", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_56, LM_MID_61, 0,
      "-> MID61", "Walk to midpoint\nof edge B6-B1.", nullptr },

    { STEP_STAND,  LM_MID_61, LM_MID_61, 3000,
      "AT MID61", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,   LM_MID_61, LM_OPPOSITE_RX, 0,
      "-> OUTSIDE",
      "Walk OUT past the\nbeacon ring to the\nfar side. Through a\ndoorway is fine.",
      nullptr },

    { STEP_STAND,  LM_OPPOSITE_RX, LM_OPPOSITE_RX, 4000,
      "AT OUTSIDE", "Hold still\noutside the ring.", "OK - RIGHT." },

    { STEP_ROTATE, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 10000,
      "ROTATE OUT",
      "Turn slowly all\nthe way around,\n10 seconds.\nThis teaches the\nperimeter alert.",
      "Done - RIGHT." },

    { STEP_WALK,   LM_OPPOSITE_RX, LM_RX, 0,
      "-> RX", "Return to RX.\nLoop closes.", nullptr },

    { STEP_STAND,  LM_RX, LM_RX, 3000,
      "AT RX", "Hold still\n(loop close).", "Done!\nPress RIGHT." },

    { STEP_END, LM_RX, LM_RX, 0, "DONE", "", "" },

};

static const WizardStep SCRIPT_SOLO_4[] = {
    { STEP_INTRO, LM_RX, LM_RX, 0,
      "SOLO CAL",
      "Hold T-Display\nnear your chest\nthroughout the walk.",
      "Press RIGHT." },

    { STEP_STAND, LM_BEACON_1, LM_BEACON_1, 4000,
      "AT B1", "Hold still\nat BEACON 1.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_1, LM_BEACON_2, 0,
      "-> B2", "Walk to B2.", nullptr },
    { STEP_STAND, LM_BEACON_2, LM_BEACON_2, 4000,
      "AT B2", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_2, LM_BEACON_3, 0,
      "-> B3", "Walk to B3.", nullptr },
    { STEP_STAND, LM_BEACON_3, LM_BEACON_3, 4000,
      "AT B3", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_3, LM_BEACON_4, 0,
      "-> B4", "Walk to B4.", nullptr },
    { STEP_STAND, LM_BEACON_4, LM_BEACON_4, 4000,
      "AT B4", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_4, LM_CENTROID, 0,
      "-> CENTER", "Walk to centroid.", nullptr },
    { STEP_STAND, LM_CENTROID, LM_CENTROID, 4000,
      "AT CENTER", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_CENTROID, LM_CENTROID, 10000,
      "ROTATE 360",
      "Rotate slowly\nin place over\n10 seconds.",
      "Done - RIGHT." },
    { STEP_WALK,  LM_CENTROID, LM_MID_12, 0,
      "-> MID12", "Walk to mid B1-B2.", nullptr },
    { STEP_STAND, LM_MID_12, LM_MID_12, 3000,
      "AT MID12", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_MID_12, LM_MID_12, 10000,
      "ROTATE MID12", "Turn slowly all\nthe way around.", "Done - RIGHT." },
    { STEP_WALK,  LM_MID_12, LM_MID_23, 0,
      "-> MID23", "Walk to mid B2-B3.", nullptr },
    { STEP_STAND, LM_MID_23, LM_MID_23, 3000,
      "AT MID23", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_23, LM_MID_34, 0,
      "-> MID34", "Walk to mid B3-B4.", nullptr },
    { STEP_STAND, LM_MID_34, LM_MID_34, 3000,
      "AT MID34", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_34, LM_MID_41, 0,
      "-> MID41", "Walk to mid B4-B1.", nullptr },
    { STEP_STAND, LM_MID_41, LM_MID_41, 3000,
      "AT MID41", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_41, LM_OPPOSITE_RX, 0,
      "-> OUTSIDE", "Walk OUT past the\nbeacon ring.", nullptr },
    { STEP_STAND, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 4000,
      "AT OUTSIDE", "Hold still\noutside the ring.", "OK - RIGHT." },
    { STEP_ROTATE, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 10000,
      "ROTATE OUT", "Turn slowly all\nthe way around.", "Done - RIGHT." },
    { STEP_END, LM_RX, LM_RX, 0, "DONE", "", "" },

};

static const WizardStep SCRIPT_SOLO_5[] = {
    { STEP_INTRO, LM_RX, LM_RX, 0,
      "SOLO CAL",
      "Hold T-Display\nnear your chest\nthroughout the walk.",
      "Press RIGHT." },

    { STEP_STAND, LM_BEACON_1, LM_BEACON_1, 4000,
      "AT B1", "Hold still\nat BEACON 1.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_1, LM_BEACON_2, 0,
      "-> B2", "Walk to B2.", nullptr },
    { STEP_STAND, LM_BEACON_2, LM_BEACON_2, 4000,
      "AT B2", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_2, LM_BEACON_3, 0,
      "-> B3", "Walk to B3.", nullptr },
    { STEP_STAND, LM_BEACON_3, LM_BEACON_3, 4000,
      "AT B3", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_3, LM_BEACON_4, 0,
      "-> B4", "Walk to B4.", nullptr },
    { STEP_STAND, LM_BEACON_4, LM_BEACON_4, 4000,
      "AT B4", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_4, LM_BEACON_5, 0,
      "-> B5", "Walk to B5.", nullptr },
    { STEP_STAND, LM_BEACON_5, LM_BEACON_5, 4000,
      "AT B5", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_5, LM_CENTROID, 0,
      "-> CENTER", "Walk to centroid.", nullptr },
    { STEP_STAND, LM_CENTROID, LM_CENTROID, 4000,
      "AT CENTER", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_CENTROID, LM_CENTROID, 10000,
      "ROTATE 360",
      "Rotate slowly\nin place over\n10 seconds.",
      "Done - RIGHT." },
    { STEP_WALK,  LM_CENTROID, LM_MID_12, 0,
      "-> MID12", "Walk to mid B1-B2.", nullptr },
    { STEP_STAND, LM_MID_12, LM_MID_12, 3000,
      "AT MID12", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_MID_12, LM_MID_12, 10000,
      "ROTATE MID12", "Turn slowly all\nthe way around.", "Done - RIGHT." },
    { STEP_WALK,  LM_MID_12, LM_MID_23, 0,
      "-> MID23", "Walk to mid B2-B3.", nullptr },
    { STEP_STAND, LM_MID_23, LM_MID_23, 3000,
      "AT MID23", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_23, LM_MID_34, 0,
      "-> MID34", "Walk to mid B3-B4.", nullptr },
    { STEP_STAND, LM_MID_34, LM_MID_34, 3000,
      "AT MID34", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_34, LM_MID_45, 0,
      "-> MID45", "Walk to mid B4-B5.", nullptr },
    { STEP_STAND, LM_MID_45, LM_MID_45, 3000,
      "AT MID45", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_45, LM_MID_51, 0,
      "-> MID51", "Walk to mid B5-B1.", nullptr },
    { STEP_STAND, LM_MID_51, LM_MID_51, 3000,
      "AT MID51", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_51, LM_OPPOSITE_RX, 0,
      "-> OUTSIDE", "Walk OUT past the\nbeacon ring.", nullptr },
    { STEP_STAND, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 4000,
      "AT OUTSIDE", "Hold still\noutside the ring.", "OK - RIGHT." },
    { STEP_ROTATE, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 10000,
      "ROTATE OUT", "Turn slowly all\nthe way around.", "Done - RIGHT." },
    { STEP_END, LM_RX, LM_RX, 0, "DONE", "", "" },

};

static const WizardStep SCRIPT_SOLO_6[] = {
    { STEP_INTRO, LM_RX, LM_RX, 0,
      "SOLO CAL",
      "Hold T-Display\nnear your chest\nthroughout the walk.",
      "Press RIGHT." },

    { STEP_STAND, LM_BEACON_1, LM_BEACON_1, 4000,
      "AT B1", "Hold still\nat BEACON 1.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_1, LM_BEACON_2, 0,
      "-> B2", "Walk to B2.", nullptr },
    { STEP_STAND, LM_BEACON_2, LM_BEACON_2, 4000,
      "AT B2", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_2, LM_BEACON_3, 0,
      "-> B3", "Walk to B3.", nullptr },
    { STEP_STAND, LM_BEACON_3, LM_BEACON_3, 4000,
      "AT B3", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_3, LM_BEACON_4, 0,
      "-> B4", "Walk to B4.", nullptr },
    { STEP_STAND, LM_BEACON_4, LM_BEACON_4, 4000,
      "AT B4", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_4, LM_BEACON_5, 0,
      "-> B5", "Walk to B5.", nullptr },
    { STEP_STAND, LM_BEACON_5, LM_BEACON_5, 4000,
      "AT B5", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_5, LM_BEACON_6, 0,
      "-> B6", "Walk to B6.", nullptr },
    { STEP_STAND, LM_BEACON_6, LM_BEACON_6, 4000,
      "AT B6", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_BEACON_6, LM_CENTROID, 0,
      "-> CENTER", "Walk to centroid.", nullptr },
    { STEP_STAND, LM_CENTROID, LM_CENTROID, 4000,
      "AT CENTER", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_CENTROID, LM_CENTROID, 10000,
      "ROTATE 360",
      "Rotate slowly\nin place over\n10 seconds.",
      "Done - RIGHT." },
    { STEP_WALK,  LM_CENTROID, LM_MID_12, 0,
      "-> MID12", "Walk to mid B1-B2.", nullptr },
    { STEP_STAND, LM_MID_12, LM_MID_12, 3000,
      "AT MID12", "Hold still.", "OK - RIGHT." },
    { STEP_ROTATE, LM_MID_12, LM_MID_12, 10000,
      "ROTATE MID12", "Turn slowly all\nthe way around.", "Done - RIGHT." },
    { STEP_WALK,  LM_MID_12, LM_MID_23, 0,
      "-> MID23", "Walk to mid B2-B3.", nullptr },
    { STEP_STAND, LM_MID_23, LM_MID_23, 3000,
      "AT MID23", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_23, LM_MID_34, 0,
      "-> MID34", "Walk to mid B3-B4.", nullptr },
    { STEP_STAND, LM_MID_34, LM_MID_34, 3000,
      "AT MID34", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_34, LM_MID_45, 0,
      "-> MID45", "Walk to mid B4-B5.", nullptr },
    { STEP_STAND, LM_MID_45, LM_MID_45, 3000,
      "AT MID45", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_45, LM_MID_56, 0,
      "-> MID56", "Walk to mid B5-B6.", nullptr },
    { STEP_STAND, LM_MID_56, LM_MID_56, 3000,
      "AT MID56", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_56, LM_MID_61, 0,
      "-> MID61", "Walk to mid B6-B1.", nullptr },
    { STEP_STAND, LM_MID_61, LM_MID_61, 3000,
      "AT MID61", "Hold still.", "OK - RIGHT." },
    { STEP_WALK,  LM_MID_61, LM_OPPOSITE_RX, 0,
      "-> OUTSIDE", "Walk OUT past the\nbeacon ring.", nullptr },
    { STEP_STAND, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 4000,
      "AT OUTSIDE", "Hold still\noutside the ring.", "OK - RIGHT." },
    { STEP_ROTATE, LM_OPPOSITE_RX, LM_OPPOSITE_RX, 10000,
      "ROTATE OUT", "Turn slowly all\nthe way around.", "Done - RIGHT." },
    { STEP_END, LM_RX, LM_RX, 0, "DONE", "", "" },

};

// Dispatch by beacon count.  Index 0..2 are null: 1 beacon uses the
// tripwire shortcut (no walk cal at all) and 2 beacons have no walk
// script yet, so both fall through to the guard in wizard_begin().
struct ScriptEntry { const WizardStep *steps; int len; };
#define SCRIPT_ENTRY(a) { a, (int)(sizeof(a) / sizeof(WizardStep)) }

static const ScriptEntry SCRIPTS_STEREO[MAX_BEACONS + 1] = {
    { nullptr, 0 },                 // 0 beacons
    { nullptr, 0 },                 // 1 — tripwire, no walk
    { nullptr, 0 },                 // 2 — line mode, no walk script yet
    SCRIPT_ENTRY(SCRIPT_STEREO_3),
    SCRIPT_ENTRY(SCRIPT_STEREO_4),
    SCRIPT_ENTRY(SCRIPT_STEREO_5),
    SCRIPT_ENTRY(SCRIPT_STEREO_6),
};

static const ScriptEntry SCRIPTS_SOLO[MAX_BEACONS + 1] = {
    { nullptr, 0 },
    { nullptr, 0 },
    { nullptr, 0 },
    SCRIPT_ENTRY(SCRIPT_SOLO_3),
    SCRIPT_ENTRY(SCRIPT_SOLO_4),
    SCRIPT_ENTRY(SCRIPT_SOLO_5),
    SCRIPT_ENTRY(SCRIPT_SOLO_6),
};

// ── Module state ──────────────────────────────────────────────
static const WizardStep *s_script = nullptr;
static int          s_script_len = 0;
static int          s_cur_idx = 0;
static bool         s_active = false;
static bool         s_finished = false;
static uint32_t     s_phase_enter_ms = 0;
static WizardPhase  s_phase = WP_READY;
static bool         s_capture_open = false;

// ── Capture-window helpers ────────────────────────────────────
static void open_capture(const WizardStep &s) {
    if (s_capture_open) return;
    switch (s.kind) {
        case STEP_STAND:  scene_begin_landmark_capture(s.landmark_a); break;
        case STEP_WALK:   scene_begin_transit_capture(s.landmark_a, s.landmark_b); break;
        case STEP_ROTATE: scene_begin_rotate_capture(s.landmark_a); break;
        default: return;   // no capture for INTRO/END
    }
    s_capture_open = true;
}
static void close_capture(const WizardStep &s) {
    if (!s_capture_open) return;
    switch (s.kind) {
        case STEP_STAND:  scene_end_landmark_capture(); break;
        case STEP_WALK:   scene_end_transit_capture(); break;
        case STEP_ROTATE: scene_end_rotate_capture(); break;
        default: break;
    }
    s_capture_open = false;
}

// Change phase — also handles capture open/close as appropriate.
static void set_phase(WizardPhase p) {
    s_phase = p;
    s_phase_enter_ms = millis();
}

// Enter a new step, choose its initial phase based on kind.
static void enter_step(int idx) {
    s_cur_idx = idx;
    const WizardStep &s = s_script[idx];
    MSLOG("[wizard] step %d/%d kind=%d lm_a=%u lm_b=%u title=%s\n",
                  idx, s_script_len, (int)s.kind,
                  (unsigned)s.landmark_a, (unsigned)s.landmark_b,
                  s.title ? s.title : "-");

    // Tag scene so PROBE peer packets carry provenance.  0xFF = transit.
    uint8_t lm_tag = (s.kind == STEP_WALK) ? 0xFF : (uint8_t)s.landmark_a;
    scene_cal_note_step((uint8_t)idx, lm_tag);

    switch (s.kind) {
        case STEP_WALK:
            // Wait for user to press BEGIN before opening the transit
            // capture window.
            set_phase(WP_ARM);
            break;
        case STEP_STAND:
        case STEP_ROTATE:
            // Open capture immediately; user hits NEXT after the hold
            // countdown expires.
            open_capture(s);
            set_phase(WP_CAPTURE);
            break;
        case STEP_INTRO:
        case STEP_END:
        default:
            // No capture; just show text and wait for NEXT.
            set_phase(WP_READY);
            break;
    }
}

// Exit a step (called before moving on).  Ensures capture is closed.
static void exit_step(int idx) {
    const WizardStep &s = s_script[idx];
    close_capture(s);
}

// ── Public API ────────────────────────────────────────────────
void wizard_begin(CalMode mode) {
    // v0.8: the script is chosen by DISCOVERED BEACON COUNT, not just
    // by cal mode.  Running the 3-beacon script against a 5-beacon
    // install would silently never visit B4 or B5, leaving two beacons
    // with no kernel coverage at all — so an out-of-range count falls
    // back to the 3-beacon script AND says so loudly on the serial log.
    const ScriptEntry *table =
        (mode == CAL_MODE_SOLO) ? SCRIPTS_SOLO : SCRIPTS_STEREO;
    const int n = g_app.beacon_count;

    if (n >= 3 && n <= MAX_BEACONS && table[n].steps != nullptr) {
        s_script     = table[n].steps;
        s_script_len = table[n].len;
    } else {
        s_script     = SCRIPT_STEREO_3;
        s_script_len = (int)(sizeof(SCRIPT_STEREO_3) / sizeof(WizardStep));
        MSLOG("[wizard] WARNING no script for %d beacons — "
                      "falling back to the 3-beacon walk\n", n);
    }
    s_active   = true;
    s_finished = false;
    s_capture_open = false;
    enter_step(0);
    MSLOG("[wizard] begin mode=%d beacons=%d steps=%d\n",
                  (int)mode, g_app.beacon_count, s_script_len);
}

void wizard_abort() {
    if (s_active) exit_step(s_cur_idx);
    s_active = false;
    s_finished = false;
    MSLOGLN("[wizard] abort");
}

bool wizard_active()   { return s_active; }
bool wizard_finished() { return s_finished; }

const WizardStep *wizard_current_step() {
    if (!s_active || s_cur_idx >= s_script_len) return nullptr;
    return &s_script[s_cur_idx];
}
int  wizard_current_index() { return s_cur_idx; }
int  wizard_total_steps()   { return s_script_len; }
WizardPhase wizard_current_phase() { return s_phase; }

uint32_t wizard_step_elapsed_ms() {
    return millis() - s_phase_enter_ms;
}

uint32_t wizard_hold_remaining_ms() {
    if (!s_active || s_phase != WP_CAPTURE) return 0;
    const WizardStep &s = s_script[s_cur_idx];
    if (s.kind != STEP_STAND && s.kind != STEP_ROTATE) return 0;
    uint32_t el = millis() - s_phase_enter_ms;
    if (el >= s.hold_ms) return 0;
    return s.hold_ms - el;
}

float wizard_overall_progress() {
    if (!s_active || s_script_len == 0) return 0;
    return (float)s_cur_idx / (float)(s_script_len - 1);
}

// Try to advance the step (called on button press or by peer sync).
// Handles the phase machine end-of-step.
bool wizard_try_advance() {
    if (!s_active) return false;
    if (s_cur_idx >= s_script_len - 1) return false;

    exit_step(s_cur_idx);
    int next = s_cur_idx + 1;
    if (s_script[next].kind == STEP_END) {
        s_active   = false;
        s_finished = true;
        s_cur_idx  = next;
        MSLOGLN("[wizard] finished");
        return true;
    }
    enter_step(next);
    return true;
}

// Handle a NEXT press within the current step, respecting the phase.
// Returns true if state changed (either phase advance or step advance).
static bool press_next() {
    if (!s_active) return false;
    const WizardStep &s = s_script[s_cur_idx];
    switch (s.kind) {
        case STEP_WALK:
            if (s_phase == WP_ARM) {
                // BEGIN pressed — open transit capture, enter capture phase.
                open_capture(s);
                set_phase(WP_CAPTURE);
                return true;
            } else if (s_phase == WP_CAPTURE) {
                // ARRIVED pressed — close transit, advance to next step.
                return wizard_try_advance();
            }
            return false;

        case STEP_STAND:
        case STEP_ROTATE:
            if (s_phase == WP_CAPTURE) {
                // Only accept the NEXT press once the hold countdown finishes.
                if (wizard_hold_remaining_ms() > 0) return false;
                // Countdown done — close capture and move to READY.  We
                // could advance directly, but READY gives the user a
                // moment to see "OK" before the next screen changes.
                close_capture(s);
                set_phase(WP_READY);
                return true;
            } else if (s_phase == WP_READY) {
                return wizard_try_advance();
            }
            return false;

        case STEP_INTRO:
        default:
            return wizard_try_advance();
    }
}

void wizard_tick() {
    if (!s_active) return;
    const WizardStep &s = s_script[s_cur_idx];

    // STAND/ROTATE: once countdown expires, auto-flip phase to READY.
    // (We do NOT auto-advance the step — that still needs a button.)
    if ((s.kind == STEP_STAND || s.kind == STEP_ROTATE)
        && s_phase == WP_CAPTURE
        && wizard_hold_remaining_ms() == 0) {
        close_capture(s);
        set_phase(WP_READY);
    }

    // RIGHT press = advance (BEGIN / ARRIVED / NEXT depending on phase).
    if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT)) {
        press_next();
        return;
    }

    // LEFT press = redo current step from the top.
    if (wasShortPressed(BTN_LEFT)) {
        exit_step(s_cur_idx);
        enter_step(s_cur_idx);
    }
}

// Force-jump — used by ANCHOR to sync to PROBE's step index.  Skips
// button checks; performs entry/exit housekeeping.
void wizard_jump_to_step(int idx) {
    if (!s_script) return;
    if (idx < 0 || idx >= s_script_len) return;
    if (s_active) exit_step(s_cur_idx);
    if (s_script[idx].kind == STEP_END) {
        s_active   = false;
        s_finished = true;
        s_cur_idx  = idx;
        MSLOGLN("[wizard] jump to END (from step hint)");
        return;
    }
    if (!s_active) { s_active = true; s_finished = false; }
    enter_step(idx);
}
