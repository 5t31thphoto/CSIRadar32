// ═══════════════════════════════════════════════════════════════
//  CSI-Radar-S3 — ui.h
//  Every visual screen the app can show. All double-buffered
//  through a full-screen sprite for flicker-free updates.
// ═══════════════════════════════════════════════════════════════
#pragma once

#include "config.h"
#include "scene.h"    // CalReport (ui_cal_stash_report / ui_cal_results)
#include "csi.h"      // csi_baseline_progress() used in ui_cal_empty_room

void ui_begin();

// One-shot screens (called once when the state entered, use ui_should_redraw()
// inside to avoid redrawing constantly).
void ui_splash();
void ui_discovery(int found, uint32_t elapsed_ms, uint32_t deadline_ms);
void ui_peer_discovery(uint32_t elapsed_ms, uint32_t deadline_ms);
void ui_role_confirm();
void ui_geometry_guide();
void ui_rx_assembly();
void ui_center_tdisplay();
void ui_baseline_countdown(int seconds_remaining);
void ui_baseline_capture(float progress);
void ui_walk_guide();
void ui_walk_capture(float progress);

// v0.3 cal ceremony screens
void ui_cal_intro();               // explain the walk
void ui_cal_anchor_place();        // stereo: place ANCHOR at origin
void ui_cal_empty_room(uint32_t elapsed_ms);
void ui_cal_landmark_walk();       // reads wizard state; drives from that
void ui_cal_finalize(float progress);
void ui_cal_results();             // shows CalReport
void ui_cal_stash_report(const CalReport &r);  // cache report for ui_cal_results

// Secondary-mode active screen (streaming to primary)
void ui_secondary_active();

// Dashboard: sub-view dispatched by current DashView.
void ui_dashboard();

// Advance the beacon selection used by the CSI sub-view.
void ui_csi_next_beacon();

// Settings menu.
void ui_settings(int selected_row);

// Utility (used by state machine for a full-screen message)
void ui_message(const char *title, const char *line1, const char *line2 = nullptr,
                uint16_t title_color = 0xFFFF);

// Sleep-arm progress bar (both buttons held).  progress is 0..1.
// Rendered as an overlay-style full screen with a bottom bar.
void ui_sleep_arm(float progress);

// Final "going to sleep" screen shown for a moment before deep sleep.
void ui_going_to_sleep();
