// ═══════════════════════════════════════════════════════════════
//  CSI-Radar-S3 — input.cpp
// ═══════════════════════════════════════════════════════════════
#include "input.h"
#include "config.h"

struct Btn {
    uint8_t  pin;
    bool     stable;         // debounced level (true = pressed)
    bool     raw;            // last raw sample
    uint32_t last_change_ms;
    uint32_t press_start_ms;
    bool     long_fired;     // suppress the release-short after a long press
    bool     ev_short;
    bool     ev_long;
};

static Btn s_btn[BTN_COUNT];

void input_begin() {
    s_btn[BTN_LEFT]  = {PIN_BTN_LEFT,  false, false, 0, 0, false, false, false};
    s_btn[BTN_RIGHT] = {PIN_BTN_RIGHT, false, false, 0, 0, false, false, false};

    pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
    pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
}

void input_poll() {
    uint32_t now = millis();

    for (int i = 0; i < BTN_COUNT; i++) {
        // Both buttons are active-LOW with pull-up
        bool raw = (digitalRead(s_btn[i].pin) == LOW);
        if (raw != s_btn[i].raw) {
            s_btn[i].raw = raw;
            s_btn[i].last_change_ms = now;
        }

        // Debounce: only accept change after stable window
        if ((now - s_btn[i].last_change_ms) >= BTN_DEBOUNCE_MS
            && s_btn[i].stable != s_btn[i].raw) {
            bool was = s_btn[i].stable;
            s_btn[i].stable = s_btn[i].raw;

            if (!was && s_btn[i].stable) {
                // Press edge
                s_btn[i].press_start_ms = now;
                s_btn[i].long_fired = false;
            } else if (was && !s_btn[i].stable) {
                // Release edge
                uint32_t held = now - s_btn[i].press_start_ms;
                if (!s_btn[i].long_fired && held < BTN_LONG_PRESS_MS) {
                    s_btn[i].ev_short = true;
                }
            }
        }

        // Long-press fires while still held, once
        if (s_btn[i].stable && !s_btn[i].long_fired
            && (now - s_btn[i].press_start_ms) >= BTN_LONG_PRESS_MS) {
            s_btn[i].long_fired = true;
            s_btn[i].ev_long = true;
        }
    }
}

bool wasShortPressed(ButtonId b) {
    if (b >= BTN_COUNT) return false;
    bool v = s_btn[b].ev_short;
    s_btn[b].ev_short = false;
    return v;
}

bool wasLongPressed(ButtonId b) {
    if (b >= BTN_COUNT) return false;
    bool v = s_btn[b].ev_long;
    s_btn[b].ev_long = false;
    return v;
}

bool isHeld(ButtonId b) {
    if (b >= BTN_COUNT) return false;
    return s_btn[b].stable;
}

uint32_t heldForMs(ButtonId b) {
    if (b >= BTN_COUNT || !s_btn[b].stable) return 0;
    return millis() - s_btn[b].press_start_ms;
}

uint32_t comboHeldMs() {
    // Both physically held?
    if (!s_btn[BTN_LEFT].stable || !s_btn[BTN_RIGHT].stable) return 0;
    // Combo start = the later of the two press edges (so brief overlap
    // during sequential releases doesn't count as a fresh combo).
    uint32_t start = s_btn[BTN_LEFT].press_start_ms;
    if (s_btn[BTN_RIGHT].press_start_ms > start) start = s_btn[BTN_RIGHT].press_start_ms;
    uint32_t now = millis();
    if (now < start) return 0;
    uint32_t held = now - start;
    if (held < BTN_COMBO_MIN_MS) return 0;
    return held;
}

void inputClearEdges() {
    for (int i = 0; i < BTN_COUNT; i++) {
        s_btn[i].ev_short  = false;
        s_btn[i].ev_long   = false;
        // Also mark long_fired so the pending release doesn't retroactively
        // fire a short-press event once the user lets go of the combo.
        s_btn[i].long_fired = true;
    }
}
