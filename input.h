// ═══════════════════════════════════════════════════════════════
//  CSI-Radar-S3 — input.h
//  Two buttons, debounced, with edge events + long-press.
//  Consume events with wasShortPressed() / wasLongPressed() which
//  are cleared on read.
// ═══════════════════════════════════════════════════════════════
#pragma once

#include <Arduino.h>

enum ButtonId : uint8_t { BTN_LEFT = 0, BTN_RIGHT = 1, BTN_COUNT = 2 };

void input_begin();
void input_poll();     // call every loop() iteration

bool wasShortPressed(ButtonId b);
bool wasLongPressed(ButtonId b);
bool isHeld(ButtonId b);            // current physical state (debounced)
uint32_t heldForMs(ButtonId b);     // 0 if not held

// Combo (both buttons held together) — returns milliseconds that both have
// been simultaneously held (0 if not both currently held).  Handy for
// power-off gestures without a dedicated switch.
uint32_t comboHeldMs();

// Consume any pending short-press events so that releasing the combo
// doesn't immediately fire a short-press on either button.
void inputClearEdges();
