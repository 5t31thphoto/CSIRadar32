#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CORE2 INPUT
//  Three capacitive zones and a touchscreen, speaking two buttons
// ═══════════════════════════════════════════════════════════════
//
//  Every shared screen speaks wasShortPressed(BTN_LEFT / BTN_RIGHT),
//  because a T-Display has two buttons.  The Cardputer maps 56 keys
//  onto that vocabulary; the Core2 maps three zones and a touchscreen
//  onto the same one, for the same reason: two divergent UIs drift
//  apart, and a bug fixed in one is not fixed in the other.
//
//  ── THE MAPPING ──────────────────────────────────────────────
//
//    BtnA (left)    -> BTN_LEFT    back, the universal escape
//    BtnC (right)   -> BTN_RIGHT   advance, the universal act
//    BtnB (centre)  -> SELECT      the third control the T-Display
//                                  never had; used for list selection
//                                  and for mute
//    BtnC held      -> BTN_RIGHT long press
//
//  ── WHY B IS "SELECT" AND NOT SOMETHING CLEVERER ─────────────
//
//  Because left/right/centre is what the physical layout already
//  suggests, and an operator holding the device should not have to
//  learn an arbitrary assignment.  A control that is where you expect it
//  is worth more than one that does something marginally better.
//
//  ── TOUCH IS ADDITIVE, NEVER REQUIRED ────────────────────────
//
//  Every screen remains fully operable with the three zones alone.
//  Touch makes menus faster -- tap the row you want instead of stepping
//  to it -- but nothing is reachable ONLY by touch.  A capacitive panel
//  with wet or gloved hands is exactly the condition where the device
//  still has to work.
// ═══════════════════════════════════════════════════════════════
#include "core2_platform.h"

#ifndef BTN_LEFT
  #define BTN_LEFT  0
  #define BTN_RIGHT 1
#endif

typedef struct {
    // The two-button illusion every shared screen consumes.
    bool short_press[2];
    bool long_press[2];
    bool held[2];

    // The extras this hardware affords.
    bool select;           // BtnB
    bool select_long;

    // Touch.
    Core2Gesture gesture;
    int16_t  touch_x, touch_y;
    bool     touched;          // a tap landed this frame
    int8_t   touched_row;      // content row under the tap, -1 if none

    // Edge bookkeeping for the long press.
    uint32_t c_down_ms;
    bool     c_is_down, c_long_fired;
    uint32_t b_down_ms;
    bool     b_is_down, b_long_fired;
    // Gesture tracking.
    int16_t  t_start_x, t_start_y;
    uint32_t t_start_ms;
    bool     t_active;
} Core2Input;

#define C2_LONG_PRESS_MS 700

static inline void c2_input_begin(Core2Input *in) { *in = Core2Input{}; }

// Clear per-frame edges at the TOP of the poll, so a screen reading them
// late sees the same values as one reading early.
static inline void c2_input_new_frame(Core2Input *in) {
    in->short_press[0] = in->short_press[1] = false;
    in->long_press[0]  = in->long_press[1]  = false;
    in->select = in->select_long = false;
    in->gesture = C2T_NONE;
    in->touched = false;
    in->touched_row = -1;
}

// Feed the three zones.  `a`, `b`, `c` are current pressed states.
static inline void c2_input_buttons(Core2Input *in, bool a, bool b, bool c,
                                    uint32_t now_ms) {
    // A: plain edge, no long press -- "back" has no second meaning and
    // giving it one would make a held finger do something surprising.
    if (a && !in->held[BTN_LEFT]) in->short_press[BTN_LEFT] = true;
    in->held[BTN_LEFT] = a;

    // C: press/hold/release, because the shared screens use a long
    // RIGHT for "confirm strongly" (redo cal, force redeploy).
    if (c && !in->c_is_down) {
        in->c_is_down = true; in->c_down_ms = now_ms; in->c_long_fired = false;
    } else if (c && in->c_is_down && !in->c_long_fired) {
        if (now_ms - in->c_down_ms >= C2_LONG_PRESS_MS) {
            in->long_press[BTN_RIGHT] = true; in->c_long_fired = true;
        }
    } else if (!c && in->c_is_down) {
        in->c_is_down = false;
        // Short only if the long did NOT fire, or a deliberate hold also
        // triggers the short action on release and the screen advances
        // twice.
        if (!in->c_long_fired) in->short_press[BTN_RIGHT] = true;
    }
    in->held[BTN_RIGHT] = c;

    // B: same discipline, so a long centre press can mean mute.
    if (b && !in->b_is_down) {
        in->b_is_down = true; in->b_down_ms = now_ms; in->b_long_fired = false;
    } else if (b && in->b_is_down && !in->b_long_fired) {
        if (now_ms - in->b_down_ms >= C2_LONG_PRESS_MS) {
            in->select_long = true; in->b_long_fired = true;
        }
    } else if (!b && in->b_is_down) {
        in->b_is_down = false;
        if (!in->b_long_fired) in->select = true;
    }
}

// Feed the touchscreen.  `down` is whether a finger is on the glass.
//
// Gesture classification happens on RELEASE, not during the drag: a
// swipe that fires mid-drag cannot be cancelled, and on a small panel
// the finger frequently crosses the threshold and comes back.
static inline void c2_input_touch(Core2Input *in, bool down,
                                  int16_t x, int16_t y, uint32_t now_ms) {
    if (down && !in->t_active) {
        in->t_active = true;
        in->t_start_x = x; in->t_start_y = y; in->t_start_ms = now_ms;
        in->touch_x = x; in->touch_y = y;
        return;
    }
    if (down) { in->touch_x = x; in->touch_y = y; return; }
    if (!in->t_active) return;

    in->t_active = false;
    const int16_t dx = (int16_t)(x - in->t_start_x);
    const int16_t dy = (int16_t)(y - in->t_start_y);
    const int16_t adx = (int16_t)(dx < 0 ? -dx : dx);
    const int16_t ady = (int16_t)(dy < 0 ? -dy : dy);
    const uint32_t dur = now_ms - in->t_start_ms;

    if (adx < C2_SWIPE_MIN_PX && ady < C2_SWIPE_MIN_PX && dur <= C2_TAP_MAX_MS) {
        in->gesture = C2T_TAP;
        in->touched = true;
        // Which content row was tapped.  Rows are the unit menus are
        // built from, so returning a row rather than a pixel means a
        // screen never has to know the layout maths.
        if (y >= C2_CONTENT_Y && y < C2_CONTENT_Y + C2_CONTENT_H)
            in->touched_row = (int8_t)((y - C2_CONTENT_Y) / C2_ROW_H);
        return;
    }
    // A movement must actually CLEAR the swipe threshold to be a swipe.
    //
    // Falling through to swipe classification whenever it was not a tap
    // was wrong: a slow small wobble -- 8 px over 900 ms, which is a
    // finger resting on the glass -- is neither, and it was being
    // classified as a swipe and navigating the UI on its own.
    //
    // Neither tap nor swipe is a legitimate outcome and it must be
    // reported as nothing.
    if (adx < C2_SWIPE_MIN_PX && ady < C2_SWIPE_MIN_PX) {
        in->gesture = C2T_NONE;
        return;
    }
    if (adx >= ady) in->gesture = (dx > 0) ? C2T_SWIPE_RIGHT : C2T_SWIPE_LEFT;
    else            in->gesture = (dy > 0) ? C2T_SWIPE_DOWN  : C2T_SWIPE_UP;

    // Swipes fold into the SAME vocabulary rather than adding a parallel
    // one: left/right are back/forward, which is what they mean
    // everywhere else on a touch device.
    if (in->gesture == C2T_SWIPE_RIGHT) in->short_press[BTN_LEFT]  = true;
    if (in->gesture == C2T_SWIPE_LEFT)  in->short_press[BTN_RIGHT] = true;
}

// Shared-screen accessors, named to match the other platforms so screen
// code is identical across all three.
static inline bool c2_was_short(const Core2Input *in, int b) {
    return (b >= 0 && b < 2) && in->short_press[b];
}
static inline bool c2_was_long(const Core2Input *in, int b) {
    return (b >= 0 && b < 2) && in->long_press[b];
}
static inline bool c2_is_held(const Core2Input *in, int b) {
    return (b >= 0 && b < 2) && in->held[b];
}
