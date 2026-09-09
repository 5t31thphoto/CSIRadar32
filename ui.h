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
void ui_cal_intro();
// v0.9: "press the unit you will carry" — shown on BOTH units.
void ui_pick_carry();
// Anchor-side view of the same step: informational, no input.
void ui_pick_carry_anchor();               // explain the walk
void ui_cal_anchor_place();        // stereo: place ANCHOR at origin
void ui_cal_empty_room(uint32_t elapsed_ms);
// Countdown shown before empty-room sampling begins.
void ui_cal_leave_countdown(uint32_t seconds_left);
// v0.9: shown on the PROBE before sampling — walk out, then press START.
void ui_cal_empty_prompt();
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
// v0.8: the row count is dynamic — the probe gains an undock row once
// it has a usable kernel.  The state machine must use these rather than
// a hardcoded 6 when wrapping the selection.
#define UI_SETTINGS_MAX_ROWS      9
#define UI_SETTINGS_ROW_TRIPWIRE  6   // stereo tripwire wiring
#define UI_SETTINGS_ROW_DEBUG     7   // always present
#define UI_SETTINGS_ROW_UNDOCK    8   // probe only, once a kernel exists
bool ui_settings_undock_row_visible();
int  ui_settings_row_count();
void ui_settings(int selected_row);

// v0.9: in-RAM log viewer, reachable from Settings -> Debug log.
// scroll = index of the first line shown (0 = oldest).
void ui_debug_log(int scroll);

// v0.8: full-screen view for ST_MOBILE_PROBE.  Renders the anchor's
// track list (streamed over the peer link) plus a "you are here" marker
// with a confidence ellipse at the probe's own estimated position.
// track_state may be nullptr before the first packet arrives.
void ui_mobile_probe_view(const PeerTrackStatePacket *track_state,
                          uint32_t track_state_age_ms,
                          const float probe_pos[2],
                          const float probe_cov[3],
                          float probe_conf,
                          bool acquiring);

// Utility (used by state machine for a full-screen message)
void ui_message(const char *title, const char *line1, const char *line2 = nullptr,
                uint16_t title_color = 0xFFFF);

// Sleep-arm progress bar (both buttons held).  progress is 0..1.
// Rendered as an overlay-style full screen with a bottom bar.
void ui_sleep_arm(float progress);

// Final "going to sleep" screen shown for a moment before deep sleep.
void ui_going_to_sleep();
