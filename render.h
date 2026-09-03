// ═══════════════════════════════════════════════════════════════
//  render.h — visualization layer for the scene module.
//
//  Draws the primary radar view — top-down map of the normalized
//  geometry frame with the occupancy field as a subtle heatmap,
//  beacon icons at their calibrated positions, RX icon(s), and
//  target tracks as dots + confidence ellipses + fading trails.
//
//  All coordinates flow: scene output (normalized [-1..+1]) →
//  render (pixel coords on the T-Display).
//
//  The render module knows about pixels and LGFX; the scene module
//  doesn't.  Separation is deliberate — we can change one without
//  touching the other.
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "config.h"

// Init.  Called once from ui_begin().
void render_begin();

// Full-screen radar view.  Called from ui_dashboard when DV_RADAR
// is selected.  Draws:
//   - normalized geometry frame with range rings + axes
//   - occupancy field heatmap (subtle, alpha-blended)
//   - beacon icons at calibrated positions
//   - RX icon(s) at origin (single square solo, paired bar stereo)
//   - AoA rays (stereo mode) faintly from RX
//   - up to TRACK_MAX target dots with confidence ellipses + trails
//   - alert border when tripwire latched
//   - status band overlay (mode / target count / confidence)
//   - mini oscilloscope strip at the bottom (link disturbance per beacon)
void render_radar_view();

// Debug view: raw occupancy field heatmap + kernel sample overlay.
// Called when DV_FIELD is selected.
void render_field_view();
