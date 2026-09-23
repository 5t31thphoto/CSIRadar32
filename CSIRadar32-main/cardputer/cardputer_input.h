#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CARDPUTER INPUT
//  A keyboard pretending to be two buttons, plus everything extra
// ═══════════════════════════════════════════════════════════════
//
//  ── THE CONSTRAINT ───────────────────────────────────────────
//
//  Every shared screen in this project speaks in wasShortPressed
//  (BTN_LEFT) and wasShortPressed(BTN_RIGHT), because a T-Display has
//  exactly two buttons.  That vocabulary is embedded in every state
//  handler and every footer label.
//
//  The Cardputer has 56 keys.  The temptation is to rewrite the screens
//  for a keyboard; the cost of that is two divergent UIs that drift
//  apart with every change, and bugs fixed in one and not the other.
//
//  So the keyboard is mapped ONTO the two-button vocabulary, exactly.
//  LEFT and RIGHT behave identically to a T-Display, every shared screen
//  works unmodified, and the extra keys ADD capability rather than
//  replacing it.  A screen that knows nothing about keyboards still
//  works; a screen that wants more can ask.
//
//  ── THE MAPPING ──────────────────────────────────────────────
//
//    arrow LEFT  / backspace  -> BTN_LEFT   (back, the universal escape)
//    arrow RIGHT / enter      -> BTN_RIGHT  (advance, the universal act)
//    enter held               -> BTN_RIGHT long press
//    arrow UP / DOWN          -> list navigation, new
//    ESC                      -> straight to the dashboard, new
//    TAB                      -> cycle dashboard view, new
//    M                        -> mute, because an alarm you cannot
//                                silence instantly is a liability
//    0-9                      -> direct view select, new
//
//  ── WHY BACKSPACE AND LEFT DO THE SAME THING ─────────────────
//
//  Because an operator under pressure should not have to remember which
//  of two plausible "get me out of here" keys this screen wanted.  Both
//  work everywhere.  The same reasoning makes ENTER and RIGHT
//  interchangeable.
// ═══════════════════════════════════════════════════════════════
#include "cardputer_platform.h"

// Mirrors the shared input layer's button ids so the shared screens need
// no conditional compilation at all.
#ifndef BTN_LEFT
  #define BTN_LEFT  0
  #define BTN_RIGHT 1
#endif

typedef struct {
    // The two-button illusion, consumed by every shared screen.
    bool short_press[2];
    bool long_press[2];
    bool held[2];

    // The extra vocabulary, available to screens that want it.
    bool up, down, esc, tab, mute_key;
    char last_char;          // 0 when none
    int8_t digit;            // -1 when none

    // Edge bookkeeping.
    uint32_t right_down_ms;
    bool     right_is_down;
    bool     long_fired;
} CardputerInput;

#define CP_LONG_PRESS_MS 700

static inline void cp_input_begin(CardputerInput *in) { *in = CardputerInput{}; }

// Clear the per-frame edges.  Called at the TOP of each poll so that a
// screen reading edges late in the frame sees the same values as one
// reading early -- an edge that evaporates mid-frame is the kind of bug
// that only shows up under load.
static inline void cp_input_new_frame(CardputerInput *in) {
    in->short_press[0] = in->short_press[1] = false;
    in->long_press[0]  = in->long_press[1]  = false;
    in->up = in->down = in->esc = in->tab = in->mute_key = false;
    in->last_char = 0;
    in->digit = -1;
}

// Feed one decoded key.  `pressed` is the key's current state.
//
// The M5Cardputer keyboard reports a SET of keys held, not events, so
// this is called for each key in that set and the edge logic lives here
// rather than being spread across the caller.
static inline void cp_input_key(CardputerInput *in, CardputerKey k,
                                char ch, bool pressed, uint32_t now_ms) {
    switch (k) {
        case CPK_LEFT:
        case CPK_BACK:
            if (pressed) in->short_press[BTN_LEFT] = true;
            in->held[BTN_LEFT] = pressed;
            break;

        case CPK_RIGHT:
        case CPK_ENTER:
            // RIGHT carries the long-press meaning the shared screens
            // use for "confirm strongly" (redo cal, force deploy), so it
            // needs real press/hold/release tracking rather than an edge.
            if (pressed && !in->right_is_down) {
                in->right_is_down = true;
                in->right_down_ms = now_ms;
                in->long_fired = false;
            } else if (pressed && in->right_is_down && !in->long_fired) {
                if (now_ms - in->right_down_ms >= CP_LONG_PRESS_MS) {
                    in->long_press[BTN_RIGHT] = true;
                    in->long_fired = true;
                }
            } else if (!pressed && in->right_is_down) {
                in->right_is_down = false;
                // A short press only when the long one did NOT fire --
                // otherwise a deliberate hold also triggers the short
                // action on release and the screen advances twice.
                if (!in->long_fired) in->short_press[BTN_RIGHT] = true;
            }
            in->held[BTN_RIGHT] = pressed;
            break;

        case CPK_UP:    if (pressed) in->up   = true; break;
        case CPK_DOWN:  if (pressed) in->down = true; break;
        case CPK_ESC:   if (pressed) in->esc  = true; break;
        case CPK_TAB:   if (pressed) in->tab  = true; break;
        case CPK_CHAR:
            if (!pressed) break;
            in->last_char = ch;
            if (ch >= '0' && ch <= '9') in->digit = (int8_t)(ch - '0');
            if (ch == 'm' || ch == 'M') in->mute_key = true;
            break;
        default: break;
    }
}

// Translate a raw character from M5Cardputer's KeysState into our key.
//
// The ADV's dedicated arrows and the original Cardputer's Fn-combination
// arrows BOTH land here as characters, and the exact code points are a
// board-revision detail the public docs do not pin down.  Accepting both
// costs four comparisons and removes an entire class of "the arrow keys
// do nothing and there is no error" failure.
//
// If a revision uses different codes, this is the one function to fix.
static inline CardputerKey cp_decode_char(char c) {
    switch (c) {
        case CP_ARROW_LEFT_CH:  return CPK_LEFT;
        case CP_ARROW_RIGHT_CH: return CPK_RIGHT;
        case CP_ARROW_UP_CH:    return CPK_UP;
        case CP_ARROW_DOWN_CH:  return CPK_DOWN;
        case 0x1B:              return CPK_ESC;
        case '\t':              return CPK_TAB;
        case '\r':
        case '\n':              return CPK_ENTER;
        case 0x08:
        case 0x7F:              return CPK_BACK;
        default:                return CPK_CHAR;
    }
}

// Shared-screen accessors.  Deliberately named to match the T-Display
// input layer so screen code is identical on both platforms.
static inline bool cp_was_short(const CardputerInput *in, int b) {
    return (b >= 0 && b < 2) && in->short_press[b];
}
static inline bool cp_was_long(const CardputerInput *in, int b) {
    return (b >= 0 && b < 2) && in->long_press[b];
}
static inline bool cp_is_held(const CardputerInput *in, int b) {
    return (b >= 0 && b < 2) && in->held[b];
}
