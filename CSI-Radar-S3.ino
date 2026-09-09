// ═══════════════════════════════════════════════════════════════
//  MantisSec CSI-Radar-S3.ino  (v0.3 — scene reconstruction)
//
//  Same firmware runs on both T-Displays.  On boot both units
//  broadcast HELLO on channel 11; the lower MAC becomes PRIMARY,
//  the other SECONDARY.  If nobody answers, we fall through to
//  SOLO and behave as a single-RX system with reduced accuracy.
//
//  v0.3 setup flow (STEREO):
//    SPLASH → PEER_DISCOVERY → ROLE_CONFIRM → DISCOVERY →
//    GEOMETRY_GUIDE → CAL_INTRO → CAL_ANCHOR_PLACE →
//    CAL_EMPTY_ROOM → CAL_LANDMARK_WALK (wizard-driven) →
//    CAL_FINALIZE → CAL_RESULTS → RX_ASSEMBLY → DASHBOARD
//
//  v0.3 setup flow (SOLO):
//    same, minus CAL_ANCHOR_PLACE and RX_ASSEMBLY.
//
//  Cal ceremony topology:
//    - PRIMARY unit = ANCHOR (stationary at beacon-triangle center).
//    - SECONDARY unit = PROBE (carried by user; shows walk UI +
//      takes button presses).
//    - PROBE runs wizard_tick() locally.  On each successful step
//      advance it sends CAL_STEP_HINT to ANCHOR so ANCHOR follows.
//    - Both units call scene_observe() every frame; only ANCHOR
//      runs scene_finalize_cal() at the end.
//
//  After cal, both units go through RX_ASSEMBLY (put them on the
//  6cm bar) and enter DASHBOARD where they behave as a normal
//  stereo pair.  The ANCHOR/PROBE distinction was cal-only.
// ═══════════════════════════════════════════════════════════════
#include "config.h"
#include "input.h"
#include "csi.h"
#include "ui.h"
#include "peer.h"
#include "stereo.h"
#include "scene.h"
#include "wizard.h"

#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <esp_wifi.h>

AppContext g_app = {};

static void enter_state(AppState s) {
    g_app.state = s;
    g_app.state_enter_ms = millis();
}
static uint32_t state_age_ms() { return millis() - g_app.state_enter_ms; }

static int s_settings_row = 0;

// After PROBE advances its wizard, note the index so we only send a
// hint once per advance.  Also tracks whether ANCHOR has fetched the
// most recent step.
static int  s_probe_last_hinted_idx = -1;
static bool s_finalize_ran = false;

// v0.4: baseline-only redo — set by settings row 2.  When true, the
// next ST_CAL_EMPTY_ROOM completion skips the walk-cal wizard and
// goes straight to finalize/results (kernel data preserved).
static bool s_baseline_only_redo = false;

// Cache for the finalize report so ui_cal_results can render it.
static CalReport s_report = {};

// ── Bidirectional state broadcast (v0.4) ──────────────────────
// Either unit may broadcast its state.  During cal, PROBE is the
// user's hand so PROBE-initiated transitions (accept, redo, next
// view, etc.) need to reach ANCHOR.  Outside cal, PRIMARY drives
// (it owns the beacon-side rate switching etc.).  Both units always
// FOLLOW peer state hints — the sender decides authority.
static AppState s_last_broadcast_state = ST_SPLASH;
static bool is_cal_state(AppState s) {
    return s == ST_CAL_INTRO || s == ST_CAL_ANCHOR_PLACE
        || s == ST_CAL_EMPTY_ROOM || s == ST_CAL_LANDMARK_WALK
        || s == ST_CAL_FINALIZE  || s == ST_CAL_RESULTS
        || s == ST_RX_ASSEMBLY;
}
// v0.8: ST_MOBILE_PROBE is a LOCAL state of the probe unit only.  It
// must never travel over the state-hint channel: if a forced role
// override ever makes the probe the PRIMARY, it would become the
// broadcast authority and drag the anchor into mobile-probe mode too,
// leaving nobody running the target inference.  Guarded on both the
// send side (maybe_broadcast_state) and the receive side
// (follow_peer_state).
static bool this_unit_is_broadcast_authority() {
    // During cal, the PROBE drives (whichever role it is).
    if (is_cal_state(g_app.state)) {
        return g_app.peer.cal_role == CAL_ROLE_PROBE
            || g_app.cal_mode == CAL_MODE_SOLO;
    }
    // Outside cal, PRIMARY drives.
    return g_app.peer.role == ROLE_PRIMARY;
}
static void maybe_broadcast_state() {
    if (!this_unit_is_broadcast_authority()) return;
    if (g_app.state == ST_MOBILE_PROBE) return;   // v0.8: local-only state
    if (g_app.state == ST_DEBUG_LOG)    return;   // v0.9: local-only state
    if (g_app.state == s_last_broadcast_state) return;
    peer_send_command(PEER_OP_STATE_HINT, (uint8_t)g_app.state);
    s_last_broadcast_state = g_app.state;
}

// ── Follow peer's state hints ─────────────────────────────────
// Both units listen; only the non-authority one actually transitions
// (the authority ignores echoes of its own state).
static void follow_peer_state() {
    uint8_t hint = g_app.peer.primary_state_hint;
    if (hint == 0) return;
    g_app.peer.primary_state_hint = 0;
    if (this_unit_is_broadcast_authority()) return;   // I'm the authority; ignore
    // v0.85: undocking is a LOCAL decision and only the user ends it.
    // Outside cal the anchor is the broadcast authority, so any time it
    // re-entered ST_DASHBOARD (e.g. backing out of its own settings) it
    // would broadcast that state and drag the undocked probe back to the
    // docked dashboard mid-session.  Ignore the routine run-state hints
    // while mobile; still honour re-cal and sleep, which must win.
    if (g_app.state == ST_MOBILE_PROBE
        && (hint == ST_DASHBOARD || hint == ST_RX_ASSEMBLY)) return;
    switch (hint) {
        case ST_CAL_INTRO:
        case ST_CAL_ANCHOR_PLACE:
        case ST_CAL_EMPTY_ROOM:
        case ST_CAL_LANDMARK_WALK:
        case ST_CAL_FINALIZE:
        case ST_CAL_RESULTS:
        case ST_RX_ASSEMBLY:
        case ST_DASHBOARD:
            if (g_app.state != hint) enter_state((AppState)hint);
            break;
        case ST_SLEEP_ARM:
            enter_state(ST_SLEEP_ARM);
            break;
        case ST_MOBILE_PROBE:
        case ST_DEBUG_LOG:
            // Never follow a peer into either of these.  Both are local
            // decisions made from this unit's own settings menu.
            break;
        default:
            break;
    }
}

// ── Cal role assignment (called on entering ST_CAL_INTRO) ─────
// v0.4: honors g_app.peer.role_override so the user can force this
// unit to be PROBE or ANCHOR from settings if the auto-pick chose
// wrong (e.g. both units trying to be ANCHOR).
static void assign_cal_roles() {
    // Solo fallback: no peer, no choice.
    if (!g_app.peer.peer_present) {
        g_app.peer.cal_role = CAL_ROLE_PROBE;
        g_app.cal_mode = CAL_MODE_SOLO;
        return;
    }
    // User override wins.
    if (g_app.peer.role_override == RO_FORCE_PROBE) {
        g_app.peer.cal_role = CAL_ROLE_PROBE;
        g_app.cal_mode = CAL_MODE_STEREO;
        return;
    }
    if (g_app.peer.role_override == RO_FORCE_ANCHOR) {
        g_app.peer.cal_role = CAL_ROLE_ANCHOR;
        g_app.cal_mode = CAL_MODE_STEREO;
        return;
    }
    // AUTO: lower MAC = ANCHOR (PRIMARY).
    if (g_app.peer.role == ROLE_PRIMARY) {
        g_app.peer.cal_role = CAL_ROLE_ANCHOR;
    } else {
        g_app.peer.cal_role = CAL_ROLE_PROBE;
    }
    g_app.cal_mode = CAL_MODE_STEREO;
}

// Are we the unit driving the wizard UI (has user's buttons)?  YES for
// PROBE in stereo, YES for the sole unit in solo.
static bool this_unit_drives_wizard() {
    return g_app.peer.cal_role == CAL_ROLE_PROBE;
}

// ── Deep sleep ────────────────────────────────────────────────
static void enter_deep_sleep() {
    MSLOGLN("[sleep] entering deep sleep");
    if (g_app.peer.role == ROLE_PRIMARY && g_app.peer.peer_present) {
        peer_send_command(PEER_OP_SLEEP);
        delay(50);
    }
    ui_going_to_sleep();
    delay(500);
    digitalWrite(PIN_LCD_POWER_ON, LOW);
    esp_wifi_stop();
    esp_wifi_deinit();
    rtc_gpio_pullup_en((gpio_num_t)PIN_BTN_LEFT);
    rtc_gpio_pullup_en((gpio_num_t)PIN_BTN_RIGHT);
    rtc_gpio_pulldown_dis((gpio_num_t)PIN_BTN_LEFT);
    rtc_gpio_pulldown_dis((gpio_num_t)PIN_BTN_RIGHT);
    const uint64_t wake_mask = (1ULL << PIN_BTN_LEFT) | (1ULL << PIN_BTN_RIGHT);
#if defined(ESP_EXT1_WAKEUP_ANY_LOW)
    esp_sleep_enable_ext1_wakeup(wake_mask, ESP_EXT1_WAKEUP_ANY_LOW);
#else
    esp_sleep_enable_ext1_wakeup(wake_mask, ESP_EXT1_WAKEUP_ALL_LOW);
#endif
    esp_deep_sleep_start();
}

// ═══════════════════════════════════════════════════════════════
//  STATE HANDLERS
// ═══════════════════════════════════════════════════════════════
static void state_splash() {
    ui_splash();
    uint32_t splash_timeout = g_app.woke_from_deep_sleep ? 900 : 4000;
    if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT)
        || (state_age_ms() > splash_timeout)) {
        g_app.woke_from_deep_sleep = false;
        csi_engine_begin();
        peer_begin();
        // v0.9: bring up the wired peer link.  Harmless with no cable
        // attached — peer_active_transport() only prefers the wire once
        // a valid frame has actually arrived on it.
        peer_wire_begin();
        scene_begin();
        csi_reset_discovery();
        peer_start_discovery();
        enter_state(ST_PEER_DISCOVERY);
    }
}

static void state_peer_discovery() {
    ui_peer_discovery(state_age_ms(), PEER_DISCOVERY_MS);
    if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT)
        || peer_discovery_done()) {
        peer_resolve_role();
        enter_state(ST_ROLE_CONFIRM);
    }
    if (wasShortPressed(BTN_LEFT)) {
        // Force SOLO fallback
        g_app.peer.peer_present = false;
        peer_resolve_role();
        enter_state(ST_ROLE_CONFIRM);
    }
}

static void state_role_confirm() {
    ui_role_confirm();
    if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT)
        || state_age_ms() > 3500) {
        enter_state(ST_DISCOVERY);
    }
    if (wasShortPressed(BTN_LEFT)) {
        g_app.peer.role = ROLE_SOLO;
        g_app.peer.peer_present = false;
        enter_state(ST_DISCOVERY);
    }
}

static void state_discovery() {
    const uint32_t deadline = 12000;
    ui_discovery(g_app.beacon_count, state_age_ms(), deadline);

    if (wasShortPressed(BTN_LEFT)) {
        csi_reset_discovery();
        g_app.state_enter_ms = millis();
    }

    if ((wasShortPressed(BTN_RIGHT) && g_app.beacon_count > 0)
        || (state_age_ms() >= deadline && g_app.beacon_count > 0)) {
        int n = g_app.beacon_count;
        if (n >= 3)      g_app.mode = RM_TRIANGLE_3;
        else if (n == 2) g_app.mode = RM_LINE_2;
        else if (n == 1) g_app.mode = RM_TRIPWIRE_1;
        else             g_app.mode = RM_NONE;
        csi_choose_mode(g_app.mode);
        csi_assign_default_geometry(300.0f);
        // Derive normalized landmark positions now that we know the geometry.
        scene_derive_landmarks_from_geometry();
        // v0.9: now that we know which beacons are out there, actually
        // TELL them how to run.  This is the step that was missing: the
        // beacon firmware has always accepted these commands and the
        // receiver never sent any, so every beacon stayed at its
        // compiled 100 Hz default while the capture path used 20.
        csi_beacon_apply_run_config();
        enter_state(ST_GEOMETRY_GUIDE);
    }
}

static void state_geometry_guide() {
    ui_geometry_guide();
    if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT)) {
        assign_cal_roles();
        enter_state(ST_CAL_INTRO);
    }
    if (wasShortPressed(BTN_LEFT) || wasLongPressed(BTN_LEFT))
        enter_state(ST_DISCOVERY);
}

static void state_cal_intro() {
    ui_cal_intro();
    if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT)) {
        if (g_app.cal_mode == CAL_MODE_STEREO) enter_state(ST_CAL_ANCHOR_PLACE);
        else                                    enter_state(ST_CAL_EMPTY_ROOM);
    }
    if (wasShortPressed(BTN_LEFT))
        enter_state(ST_GEOMETRY_GUIDE);
}

static void state_cal_anchor_place() {
    ui_cal_anchor_place();
    // Only PROBE (in the user's hand) can advance this step.
    if (this_unit_drives_wizard()) {
        if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT)) {
            csi_reset_filters(true);      // clear old baseline before recap
            enter_state(ST_CAL_EMPTY_ROOM);
        }
        if (wasShortPressed(BTN_LEFT))
            enter_state(ST_CAL_INTRO);
    }
}

static void state_cal_empty_room() {
    // Reuse the v0.2 empty-room baseline mechanism verbatim: csi.cpp
    // accumulates samples into per-beacon baseline; when done, we
    // signal scene that background is captured.
    csi_baseline_accumulate();
    float p = csi_baseline_progress();
    ui_cal_empty_room(state_age_ms());

    if (this_unit_drives_wizard()) {
        if (wasShortPressed(BTN_LEFT))
            enter_state(g_app.cal_mode == CAL_MODE_STEREO
                        ? ST_CAL_ANCHOR_PLACE : ST_CAL_INTRO);
    }
    if (p >= 1.0f) {
        csi_baseline_finalize();
        scene_cal_ack_empty_room();

        // v0.4 branch A: baseline-only redo (from settings).
        // Keep existing kernel; just refresh the empty-room reference
        // and jump to finalize/results.
        if (s_baseline_only_redo) {
            s_baseline_only_redo = false;
            s_finalize_ran = true;   // no re-finalize needed
            enter_state(ST_CAL_RESULTS);
            return;
        }

        // v0.4 branch B: tripwire (1 beacon) — no walk cal is possible
        // or needed.  Beacon 1 pose is fixed at (0, +D) from the RX.
        // Fabricate a minimal report and go straight to results.
        if (g_app.mode == RM_TRIPWIRE_1) {
            scene_cal_begin(g_app.cal_mode);   // opens the cal scaffolding
            scene_cal_tripwire_finalize();     // one-shot fill-in for 1-beacon
            s_finalize_ran = true;
            s_report.valid = true;
            s_report.mode = g_app.cal_mode;
            s_report.total_kernel_samples = 0;
            ui_cal_stash_report(s_report);
            enter_state(ST_CAL_RESULTS);
            return;
        }

        // Normal path: run the full walk-cal wizard.
        scene_cal_begin(g_app.cal_mode);
        wizard_begin(g_app.cal_mode);
        s_probe_last_hinted_idx = -1;
        s_finalize_ran = false;
        enter_state(ST_CAL_LANDMARK_WALK);
    }
}

static void state_cal_landmark_walk() {
    ui_cal_landmark_walk();

    if (this_unit_drives_wizard()) {
        // PROBE drives the wizard from its buttons.  wizard_tick reads
        // wasShortPressed(BTN_RIGHT/LEFT) internally.
        wizard_tick();

        // If wizard advanced this tick, send the new step index to ANCHOR.
        int cur = wizard_current_index();
        if (cur != s_probe_last_hinted_idx) {
            peer_send_command(PEER_OP_CAL_STEP_HINT, 0, (uint16_t)cur);
            s_probe_last_hinted_idx = cur;
        }
    }
    // ANCHOR gets step advances via peer_handle_command (see below),
    // which calls wizard_jump_to_step.  Nothing to do here for ANCHOR.

    if (wizard_finished()) {
        enter_state(ST_CAL_FINALIZE);
    }
}

static void state_cal_finalize() {
    // Progress is faked from state_age_ms.  Actual finalize runs once.
    uint32_t age = state_age_ms();
    float progress = (float)age / 1500.0f;
    if (progress > 1.0f) progress = 1.0f;
    ui_cal_finalize(progress);

    // Run the real computation once at ~500ms in (gives the user
    // a moment to see the "training your model" screen).
    if (!s_finalize_ran && age > 400) {
        if (g_app.peer.cal_role == CAL_ROLE_ANCHOR
            || g_app.cal_mode == CAL_MODE_SOLO) {
            scene_finalize_cal(s_report);
        } else {
            // PROBE: we didn't do the finalize compute (ANCHOR did).
            // Fill in a minimal report so ui_cal_results shows something.
            s_report.valid = true;
            s_report.mode = g_app.cal_mode;
            s_report.total_kernel_samples = scene_kernel_sample_count();
            // v0.8: the probe DOES have a kernel of its own — every unit
            // folds its own frames into its own s_kernel[] during the
            // walk.  It just never had the per-beacon weights computed,
            // because that happens inside scene_finalize_cal() which
            // only the anchor runs.  Doing it here is what makes the
            // probe able to localize itself after undocking.
            scene_finalize_probe_kernel();
        }
        ui_cal_stash_report(s_report);
        s_finalize_ran = true;
    }

    if (age > 2000) {
        enter_state(ST_CAL_RESULTS);
    }
}

static void state_cal_results() {
    ui_cal_results();
    if (this_unit_drives_wizard()) {
        if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT)) {
            // Accept — move to RX_ASSEMBLY (stereo) or dashboard (solo)
            if (g_app.cal_mode == CAL_MODE_STEREO)
                enter_state(ST_RX_ASSEMBLY);
            else
                enter_state(ST_DASHBOARD);
        }
        if (wasShortPressed(BTN_LEFT)) {
            // Redo cal from the empty-room step
            scene_reset();
            scene_begin();
            scene_derive_landmarks_from_geometry();
            csi_reset_filters(true);
            enter_state(ST_CAL_EMPTY_ROOM);
        }
    }
}

static void state_rx_assembly() {
    ui_rx_assembly();
    if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT))
        enter_state(ST_DASHBOARD);
    if (wasShortPressed(BTN_LEFT) || wasLongPressed(BTN_LEFT))
        enter_state(ST_CAL_RESULTS);
}

static void state_dashboard() {
    if (wasShortPressed(BTN_LEFT)) {
        g_app.dash_view = (DashView)((g_app.dash_view + 1) % DV_COUNT);
    }
    if (wasShortPressed(BTN_RIGHT)) {
        switch (g_app.dash_view) {
            case DV_TRIPWIRE:
                g_app.alert_latched = false;
                break;
            case DV_CSI:
                ui_csi_next_beacon();
                break;
            default: break;
        }
    }
    if (wasLongPressed(BTN_RIGHT)) {
        s_settings_row = 0;
        enter_state(ST_SETTINGS);
    }
    if (wasLongPressed(BTN_LEFT)) {
        csi_reset_discovery();
        enter_state(ST_DISCOVERY);
    }
    ui_dashboard();
}

// ═══════════════════════════════════════════════════════════════
//  v0.8 — ST_MOBILE_PROBE (probe unit only)
// ═══════════════════════════════════════════════════════════════
static uint32_t s_probe_undock_ts_ms   = 0;
static uint32_t s_probe_pos_last_tx_ms = 0;
static float    s_probe_pos[2]  = {0, 0};
static float    s_probe_cov[3]  = {0.1f, 0.1f, 0};
static float    s_probe_conf    = 0;

// Spec 2.9: the first fixes after undocking are noisy, because the
// kernel only ever saw the probe at landmarks and it is now somewhere
// between them.  For the first 3 s we show "acquiring", keep the fix to
// ourselves, and let the anchor carry on with no self-suppression at
// all.  After that we additionally require the confidence gate.
#define PROBE_ACQUIRE_MS        3000
#define PROBE_CONF_GATE         0.30f

static bool probe_fix_is_publishable() {
    if (millis() - s_probe_undock_ts_ms < PROBE_ACQUIRE_MS) return false;
    return s_probe_conf > PROBE_CONF_GATE;
}

static void enter_mobile_probe() {
    g_app.probe_undocked   = true;
    s_probe_undock_ts_ms   = millis();
    s_probe_pos_last_tx_ms = 0;
    // Spec 2.9: seed at the RX origin — where the probe demonstrably was
    // a moment ago, while docked on the bar.
    s_probe_pos[0] = 0; s_probe_pos[1] = 0;
    s_probe_cov[0] = 0.1f; s_probe_cov[1] = 0.1f; s_probe_cov[2] = 0;
    s_probe_conf   = 0;
    peer_send_command(PEER_OP_UNDOCK_PROBE);
    enter_state(ST_MOBILE_PROBE);
}

static void exit_mobile_probe() {
    g_app.probe_undocked = false;
    peer_send_command(PEER_OP_REDOCK_PROBE);
    peer_reset_probe_undock();
    enter_state(ST_DASHBOARD);
}

static void state_mobile_probe() {
    // 1) Self-locate from this unit's own observations against its own
    //    (probe-perspective) kernel.  scene_observe has already stashed
    //    the latest frame; g_last_obs is what it holds.
    extern FrameObservation g_last_obs;
    float pos[2], cov[3], conf;
    if (scene_estimate_probe_position(g_last_obs, pos, cov, &conf)) {
        s_probe_pos[0] = pos[0]; s_probe_pos[1] = pos[1];
        s_probe_cov[0] = cov[0]; s_probe_cov[1] = cov[1]; s_probe_cov[2] = cov[2];
        s_probe_conf   = conf;
    }

    // 2) Stream the fix to the anchor at ~10 Hz, but only once it is
    //    trustworthy — publishing a bad fix would make the anchor tag
    //    the wrong track as "YOU", and mislabelling a NEW presence is
    //    the one failure mode that is explicitly not acceptable.
    uint32_t now = millis();
    if (probe_fix_is_publishable() && (now - s_probe_pos_last_tx_ms) >= 100) {
        PeerProbePositionPacket pkt = {};
        pkt.magic       = PEER_PROBE_POS_MAGIC;
        pkt.rx_stamp_ms = now;
        pkt.pos_x       = s_probe_pos[0];
        pkt.pos_y       = s_probe_pos[1];
        pkt.cov_xx      = s_probe_cov[0];
        pkt.cov_yy      = s_probe_cov[1];
        pkt.cov_xy      = s_probe_cov[2];
        pkt.confidence  = s_probe_conf;
        peer_send_probe_position(pkt);
        s_probe_pos_last_tx_ms = now;
    }

    // 3) Render the anchor's tracks plus our own "you are here" marker.
    ui_mobile_probe_view(peer_last_track_state(),
                         peer_track_state_age_ms(),
                         s_probe_pos, s_probe_cov, s_probe_conf,
                         !probe_fix_is_publishable());

    // 4) LEFT returns to settings, where "Return to stereo" lives.
    if (wasShortPressed(BTN_LEFT) || wasLongPressed(BTN_LEFT)) {
        s_settings_row = 0;
        enter_state(ST_SETTINGS);
    }
}

// ── ANCHOR side: stream track state down to the undocked probe ──
static uint32_t s_track_state_last_tx_ms = 0;

static void anchor_service_undocked_probe() {
    if (!peer_probe_is_undocked()) return;

    // Self-suppression: tag any track sitting on top of the probe's
    // reported position.  A stale fix (probe out of range, or gone quiet
    // because its own confidence dropped) clears every tag rather than
    // freezing the last one — a "YOU" label stuck on a track after the
    // probe has stopped reporting would be worse than no label.
    float ppos[2], pcov[3];
    if (peer_get_probe_position(ppos, pcov, nullptr))
        scene_apply_self_suppression(ppos, pcov);
    else
        scene_clear_self_suppression();

    uint32_t now = millis();
    if ((now - s_track_state_last_tx_ms) < 200) return;   // ~5 Hz
    s_track_state_last_tx_ms = now;

    PeerTrackStatePacket pkt = {};
    pkt.magic       = PEER_TRACK_STATE_MAGIC;
    pkt.rx_stamp_ms = now;
    pkt.n_tracks    = TRACK_MAX;
    for (int t = 0; t < TRACK_MAX; t++) {
        const TargetTrack *tr = scene_get_track(t);
        if (!tr) continue;
        pkt.tracks[t].active     = tr->active ? 1 : 0;
        pkt.tracks[t].is_self    = tr->is_self;
        pkt.tracks[t].pos_x      = tr->pos[0];
        pkt.tracks[t].pos_y      = tr->pos[1];
        pkt.tracks[t].cov_xx     = tr->cov_xx;
        pkt.tracks[t].cov_yy     = tr->cov_yy;
        pkt.tracks[t].cov_xy     = tr->cov_xy;
        pkt.tracks[t].confidence = tr->confidence;
    }
    peer_send_track_state(pkt);
}

// v0.9 — in-RAM log viewer.  LEFT scrolls a page, long LEFT exits,
// RIGHT clears.  Local-only, like ST_MOBILE_PROBE: never broadcast.
static int s_debug_scroll = 0;
static void state_debug_log() {
    ui_debug_log(s_debug_scroll);
    if (wasLongPressed(BTN_LEFT)) {
        s_settings_row = UI_SETTINGS_ROW_DEBUG;
        enter_state(ST_SETTINGS);
        return;
    }
    if (wasShortPressed(BTN_LEFT)) {
        s_debug_scroll += 10;
        if (s_debug_scroll >= ms_log_count()) s_debug_scroll = 0;   // wrap
    }
    if (wasShortPressed(BTN_RIGHT) || wasLongPressed(BTN_RIGHT)) {
        ms_log_clear();
        s_debug_scroll = 0;
    }
}

static void state_settings() {
    // The undock row can disappear underneath the cursor (e.g. the peer
    // drops), so clamp before drawing rather than indexing off the end.
    if (s_settings_row >= ui_settings_row_count()) s_settings_row = 0;
    ui_settings(s_settings_row);
    if (wasShortPressed(BTN_LEFT)) {
        // v0.8: the row count is dynamic — the probe gains an "Undock
        // probe" row once it has a kernel to localize against.
        s_settings_row = (s_settings_row + 1) % ui_settings_row_count();
    }
    if (wasShortPressed(BTN_RIGHT)) {
        switch (s_settings_row) {
            case 0:
                g_app.sensitivity += 0.2f;
                if (g_app.sensitivity > 5.0f) g_app.sensitivity = 0.2f;
                break;
            case 1:
                // Full re-cal: wipe kernel + baseline, walk cal again.
                csi_reset_filters(true);
                scene_reset();
                scene_begin();
                scene_derive_landmarks_from_geometry();
                s_baseline_only_redo = false;
                if (this_unit_is_broadcast_authority())
                    peer_send_command(PEER_OP_RECALIBRATE);
                assign_cal_roles();
                enter_state(ST_CAL_INTRO);
                return;
            case 2:
                // Baseline-only re-cal (v0.4): keep kernel, only redo
                // the empty-room reference.  Sets a flag consumed by
                // state_cal_empty_room's finish handler which then jumps
                // straight to CAL_RESULTS instead of the walk cal.
                csi_reset_filters(true);   // clears baseline (not kernel)
                s_baseline_only_redo = true;
                if (this_unit_is_broadcast_authority())
                    peer_send_command(PEER_OP_STATE_HINT, (uint8_t)ST_CAL_EMPTY_ROOM);
                enter_state(ST_CAL_EMPTY_ROOM);
                return;
            case 3:
                g_app.mode = (RadarMode)((g_app.mode % 3) + 1);
                csi_choose_mode(g_app.mode);
                csi_assign_default_geometry(300.0f);
                scene_derive_landmarks_from_geometry();
                scene_reset();   // beacon positions changed — model is stale
                break;
            case 4:
                // v0.4: cycle cal-role override (AUTO / force PROBE / force ANCHOR)
                g_app.peer.role_override =
                    (RoleOverride)((g_app.peer.role_override + 1) % 3);
                // Re-run role assignment so change takes effect for future cal.
                assign_cal_roles();
                break;
            case 5:
                enter_state(ST_DASHBOARD);
                return;
            case UI_SETTINGS_ROW_DEBUG:
                s_debug_scroll = 0;
                enter_state(ST_DEBUG_LOG);
                return;
            case UI_SETTINGS_ROW_UNDOCK:
                // v0.8.  Only reachable when the row is visible, which
                // already requires PROBE role + a ready kernel.
                if (!ui_settings_undock_row_visible()) break;
                if (g_app.probe_undocked) exit_mobile_probe();
                else                      enter_mobile_probe();
                return;
        }
    }
    if (wasLongPressed(BTN_LEFT)) enter_state(ST_DASHBOARD);
}

static void state_sleep_arm() {
    uint32_t held = comboHeldMs();
    if (held == 0) {
        inputClearEdges();
        enter_state(g_app.pre_sleep_state);
        return;
    }
    float p = (float)held / (float)SLEEP_HOLD_MS;
    if (p >= 1.0f) {
        inputClearEdges();
        enter_deep_sleep();
        return;
    }
    ui_sleep_arm(p);
}

// ═══════════════════════════════════════════════════════════════
//  Peer command hook — handle CAL_STEP_HINT from PROBE.
//  peer.cpp calls peer_handle_command which we've extended below.
// ═══════════════════════════════════════════════════════════════
// Weak override — peer.cpp's stock peer_handle_command sets
// primary_state_hint from PEER_OP_STATE_HINT.  We add step-hint
// handling here without touching peer.cpp.  Simplest way is: peer.cpp
// exposes a hook variable, or we just intercept CAL_STEP_HINT by
// adding it to the switch in peer.cpp.  For v0.3 we take the latter —
// see peer.cpp CAL_STEP_HINT handler (added below via str_replace).

// ═══════════════════════════════════════════════════════════════
//  setup()
// ═══════════════════════════════════════════════════════════════
void setup() {
    MSLOG_BEGIN();
    delay(100);

    esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
    bool woke = (wake_cause == ESP_SLEEP_WAKEUP_EXT1)
             || (wake_cause == ESP_SLEEP_WAKEUP_GPIO)
             || (wake_cause == ESP_SLEEP_WAKEUP_EXT0);

    for (int i = 0; i < MAX_BEACONS; i++) g_app.beacon[i].active = false;
    g_app.beacon_count     = 0;
    g_app.mode             = RM_NONE;
    g_app.dash_view        = DV_RADAR;
    g_app.sensitivity      = 1.0f;
    g_app.est_x = g_app.est_y = 0;
    g_app.est_confidence   = 0;
    g_app.alert_latched    = false;
    g_app.last_alert_ms    = 0;
    g_app.boot_ms          = millis();
    g_app.pre_sleep_state  = ST_DASHBOARD;
    g_app.woke_from_deep_sleep = woke;
    g_app.peer.role                = ROLE_UNKNOWN;
    g_app.peer.peer_present        = false;
    g_app.peer.primary_state_hint  = 0;
    g_app.peer.cal_role            = CAL_ROLE_NONE;
    g_app.peer.role_override       = RO_AUTO;
    g_app.cal_mode                 = CAL_MODE_UNKNOWN;
    g_app.probe_undocked           = false;   // v0.8

    rtc_gpio_deinit((gpio_num_t)PIN_BTN_LEFT);
    rtc_gpio_deinit((gpio_num_t)PIN_BTN_RIGHT);

    input_begin();
    ui_begin();

    MSLOG("\n%s v%s booting (wake=%d, woke=%d)\n",
                  FW_NAME, FW_VERSION, (int)wake_cause, (int)woke);

    enter_state(ST_SPLASH);
}

// ═══════════════════════════════════════════════════════════════
//  loop()
// ═══════════════════════════════════════════════════════════════
void loop() {
    input_poll();

    // Combo-hold gesture (works from any state except splash/sleep_arm)
    if (g_app.state != ST_SLEEP_ARM
        && g_app.state != ST_SPLASH
        && comboHeldMs() >= BTN_COMBO_MIN_MS) {
        g_app.pre_sleep_state = g_app.state;
        enter_state(ST_SLEEP_ARM);
    }

    // Drain CSI frames + do per-beacon signal processing
    csi_process_frames();

    // Peer tick — heartbeats, timeouts
    if (g_app.state != ST_SPLASH) {
        peer_tick();
        peer_wire_poll();   // v0.9: drain the wired link, if present
        // v0.9: keep beacons on the commanded rate. A beacon that reboots
        // or powers up late would otherwise stay at the 100 Hz
        // compatibility default indefinitely.
        //
        // NOT during calibration: re-commanding changes a beacon's TX
        // rate mid-capture, so a single kernel sample would span two
        // rates, and the command itself is extra ESP-NOW airtime during
        // the measurement we are trying to make.  Anything off-rate gets
        // corrected as soon as cal finishes.
        if (g_app.beacon_count > 0 && !is_cal_state(g_app.state))
            csi_beacon_enforce_rate();
    }

    // Both units follow peer state hints; authority check is inside.
    follow_peer_state();

    // Broadcast our current state so peer can follow
    maybe_broadcast_state();

    // Stereo AoA runs at dashboard AND during cal (so kernel gets AoA)
    if (g_app.peer.role == ROLE_PRIMARY
        && (g_app.state == ST_DASHBOARD
            || g_app.state == ST_CAL_LANDMARK_WALK)) {
        stereo_update_aoa();
    }

    // Feed frame observations to scene — always during cal windows,
    // AND at runtime so scene_observe stashes the latest frame for
    // scene_update to consume.
    // v0.9: rate-gate this.  loop() yields only every 15 ms (see bottom
    // of this function), so it free-runs at hundreds of Hz, and this was
    // rebuilding a FrameObservation and calling scene_observe() on every
    // single iteration.  scene_update() already caps itself at
    // SCENE_UPDATE_MS, and the cal capture is decimated to CAP_HZ, so
    // everything downstream is throwing most of that work away.  Sample
    // slightly faster than the fastest consumer and no faster.
    {
        static uint32_t s_spatial_last_ms = 0;
        const uint32_t SPATIAL_MIN_MS = 10;   // 100 Hz ceiling
        uint32_t now_sp = millis();
        if ((g_app.state == ST_CAL_LANDMARK_WALK
             || g_app.state == ST_DASHBOARD
             || g_app.state == ST_MOBILE_PROBE)
            && (now_sp - s_spatial_last_ms) >= SPATIAL_MIN_MS) {
            s_spatial_last_ms = now_sp;
            csi_update_spatial();   // internally calls scene_observe()
        }
    }

    // Runtime reconstruction — only in dashboard, only after cal
    if (g_app.state == ST_DASHBOARD && scene_cal_complete()) {
        scene_update();
        // v0.8: on the anchor, fold in the undocked probe's reported
        // position and stream the resulting track list back to it.
        // No-ops entirely when no probe has undocked.
        anchor_service_undocked_probe();
    }

    switch (g_app.state) {
        case ST_SPLASH:             state_splash();             break;
        case ST_PEER_DISCOVERY:     state_peer_discovery();     break;
        case ST_ROLE_CONFIRM:       state_role_confirm();       break;
        case ST_DISCOVERY:          state_discovery();          break;
        case ST_GEOMETRY_GUIDE:     state_geometry_guide();     break;
        case ST_CAL_INTRO:          state_cal_intro();          break;
        case ST_CAL_ANCHOR_PLACE:   state_cal_anchor_place();   break;
        case ST_CAL_EMPTY_ROOM:     state_cal_empty_room();     break;
        case ST_CAL_LANDMARK_WALK:  state_cal_landmark_walk();  break;
        case ST_CAL_FINALIZE:       state_cal_finalize();       break;
        case ST_CAL_RESULTS:        state_cal_results();        break;
        case ST_RX_ASSEMBLY:        state_rx_assembly();        break;
        case ST_DASHBOARD:          state_dashboard();          break;
        case ST_SETTINGS:           state_settings();           break;
        case ST_MOBILE_PROBE:       state_mobile_probe();       break;
        case ST_DEBUG_LOG:          state_debug_log();          break;
        case ST_SLEEP_ARM:          state_sleep_arm();          break;
        default:                    break;
    }

    static uint32_t last_slow = 0;
    if (millis() - last_slow > 15) { last_slow = millis(); delay(1); }
}
