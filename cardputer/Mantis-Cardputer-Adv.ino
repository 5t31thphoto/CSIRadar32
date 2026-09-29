// ═══════════════════════════════════════════════════════════════
//  MANTIS — CARDPUTER-ADV PROBE
//  A full receiver that stays in the operator's hand
// ═══════════════════════════════════════════════════════════════
//
//  Same inference as a T-Display: CSI capture, the scene solver, the
//  mesh layers, the tomography, the Doppler.  What differs:
//
//    role      PROBE, always.  Never an anchor.  A device carried by a
//              moving operator cannot be the fixed reference a stereo
//              pair provides, so the lock is structural rather than a
//              setting anyone could get wrong.
//
//    display   240x135 landscape against 170x320 portrait.  A permanent
//              map/status split, which portrait cannot afford.
//
//    input     56 keys mapped onto the two-button vocabulary every
//              shared screen already speaks, plus extras.
//
//    IMU       a second, independent witness to the operator's own
//              rotation and stride.
//
//    audio     configurable alarms, with tone fallbacks so a missing SD
//              card can never produce a silent alarm.
//
//    SD        session recording, so RF can be replayed against changed
//              inference instead of re-tested in a room that has moved on.
//
//  ── STANDALONE IS A FIRST-CLASS MODE ─────────────────────────
//
//  With no T-Display present the Cardputer runs its own inference from
//  the beacon mesh alone.  That works because the mesh was built to be
//  autonomous: the beacons keep their own time, assign their own slots,
//  survey their own geometry and publish their own perspectives whether
//  or not any receiver is listening.  The Cardputer simply becomes the
//  listener.
//
//  Build: board = M5Cardputer, board manager >= 3.2.3,
//         M5Cardputer >= 1.1.1, M5Unified >= 0.2.10, M5GFX >= 0.2.10
// ═══════════════════════════════════════════════════════════════
#include <M5Cardputer.h>
#include <SPI.h>
#include <SD.h>

#include "cardputer_platform.h"
#include "cardputer_input.h"
#include "mantis_imu.h"
#include "mantis_alarms.h"
#include "mantis_mp3.h"
#include "cardputer_ui.h"
#include "cardputer_session.h"

#include "mantis_air.h"
#include "mantis_sched.h"
#include "mantis_report.h"
#include "mantis_mesh.h"
#include "mantis_tomo.h"
#include "mantis_targets.h"
#include "mantis_doppler.h"
#include "mantis_probe.h"
#include "mantis_geometry.h"
#include "mantis_static.h"
#include "mantis_receiver.h"
#include "mantis_caps.h"
#include "mantis_fuse.h"
#include "mantis_control.h"
#include "mantis_probe_radio.h"
#include "mantis_probe_app.h"  // shared: alarm policy, map, anchor remote

// ── State ─────────────────────────────────────────────────────
static M5Canvas          g_cv(&M5Cardputer.Display);
static CardputerInput    g_in;
static CardputerImu      g_imu;
static CardputerTurn     g_turn;
static CardputerAudio    g_audio;
static CardputerSession  g_sess;
static CardputerScreen   g_screen = CPS_SPLASH;
static CardputerMode     g_mode   = CPM_SEARCHING;

// ONE receiver object, exactly as the Core2 uses.
//
// This file previously drove MantisMesh, MantisTargetSet and friends
// directly, which meant it silently missed everything mantis_receiver.h
// added later: surveyed-geometry adoption, cross-channel fusion, the
// phase-rate Doppler input, and capability gating.  Two devices with the
// same role were running different inference.
//
// Anything a device needs from the mesh now goes through here.
static MantisReceiver g_rx;
#define g_mesh    (g_rx.mesh)
#define g_targets (g_rx.targets)
#define g_integ   (g_rx.integrity)
#define g_probe   (g_rx.probe)

static uint32_t g_screen_entered_ms = 0;
static uint8_t  g_dash_view = 0;
static bool     g_canvas_ok = false;
static MantisProbeLink g_link;

static uint32_t screen_age() { return millis() - g_screen_entered_ms; }
static void go(CardputerScreen s) {
    if (s == g_screen) return;
    g_screen = s;
    g_screen_entered_ms = millis();
}

// ── Input ─────────────────────────────────────────────────────
// M5Cardputer reports the SET of keys currently held, not events, so the
// edge logic lives in cardputer_input.h and this only translates.
static void poll_keys() {
    cp_input_new_frame(&g_in);
    if (!M5Cardputer.Keyboard.isChange()) return;

    const bool any = M5Cardputer.Keyboard.isPressed();
    Keyboard_Class::KeysState st = M5Cardputer.Keyboard.keysState();
    const uint32_t now = millis();

    if (st.enter) cp_input_key(&g_in, CPK_ENTER, 0, true,  now);
    else          cp_input_key(&g_in, CPK_ENTER, 0, false, now);
    if (st.del)   cp_input_key(&g_in, CPK_BACK,  0, true,  now);

    for (auto c : st.word) {
        const CardputerKey k = cp_decode_char(c);
        cp_input_key(&g_in, k, c, true, now);
    }
    if (!any) {
        // Release: the two-button illusion needs a falling edge or a
        // long press never completes.
        cp_input_key(&g_in, CPK_ENTER, 0, false, now);
    }
}

// ── Audio ─────────────────────────────────────────────────────
// Play an alarm: the WAV from SD when it is there, a tone when it is
// not.  The caller never has to know which happened, because the ONE
// guarantee this function makes is that it makes a noise.
static uint16_t g_tone_hz = 0, g_tone_ms = 0;
static uint8_t  g_tone_left = 0;
static uint32_t g_tone_next = 0;
static uint32_t g_alarm_flash_until = 0;
static CardputerAlarm g_last_alarm = CPA_NONE;

// Tone repeats run from loop(), never delay(): a blocking beep froze the
// radio pump for up to a second, exactly when the reports mattered.
static void tone_tick() {
    if (!g_tone_left || (int32_t)(millis() - g_tone_next) < 0) return;
    M5Cardputer.Speaker.tone(g_tone_hz, g_tone_ms, CP_MP3_CHANNEL + 1);
    g_tone_left--;
    g_tone_next = millis() + g_tone_ms + 60;
}

static void play_alarm(CardputerAlarm k) {
    if (k <= CPA_NONE || k >= CPA_COUNT) return;
    const uint32_t now = millis();
    g_last_alarm = k;
    g_alarm_flash_until = now + 1500;      // visual alarm fires even muted
    if (cp_audio_should_fire(&g_audio, k, now) != CPF_FIRE) return;
    cp_audio_mark_fired(&g_audio, k, now);
    const CardputerAlarmDef &d = CP_ALARMS[k];
    if (g_audio.sd_present && g_audio.file_ok[k] && !cp_mp3_busy()) {
        char path[96];
        snprintf(path, sizeof(path), "%s/%s", CP_ALARM_DIR, d.file);
        if (cp_mp3_play(path, g_audio.volume)) return;
    }
    M5Cardputer.Speaker.setVolume(g_audio.volume);
    g_tone_hz = d.tone_hz; g_tone_ms = d.tone_ms;
    g_tone_left = d.repeats; g_tone_next = now;
}

// ── IMU ───────────────────────────────────────────────────────
static void poll_imu() {
    // Per M5 docs the IMU is reached through M5, not M5Cardputer.
    if (!M5.Imu.update()) return;
    auto d = M5.Imu.getImuData();
    cp_imu_update(&g_imu, d.accel.x, d.accel.y, d.accel.z,
                  d.gyro.z, millis());
    cp_turn_update(&g_turn, &g_imu);
}

// ── Mode detection ────────────────────────────────────────────
// Decided from what is actually heard, never from a setting.  A mode
// that claims a stereo pair that is not there would silently produce
// AoA from one radio.
static void update_mode() {
    const uint32_t now = millis();
    const bool anchor_live = mantis_probe_anchor_live(&g_link, now);
    CardputerMode m;
    if (anchor_live && (g_link.st.flags & MAS_STEREO)) m = CPM_PROBE_STEREO;
    else if (anchor_live)                              m = CPM_PROBE_SOLO;
    else if (g_rx.deploy.beacons >= 1)                 m = CPM_STANDALONE;
    else                                               m = CPM_SEARCHING;
    g_mode = m;
    // What the probe can do follows from what it actually hears.
    g_rx.deploy.tdisplays = anchor_live ? ((g_link.st.flags & MAS_STEREO) ? 2 : 1) : 0;
    g_rx.deploy.docked    = anchor_live && (g_link.st.flags & MAS_STEREO);
    g_rx.caps = mantis_caps(&g_rx.deploy);
}

// ── Session recording ─────────────────────────────────────────
// The recorder the Sessions screen always promised.  One header, then
// one fixed-size frame per 100 ms: for each reporting beacon, the MEAN
// of its reported links (amp, phase, quality, RSSI) and its noise floor,
// plus where the probe thinks it is and the IMU heading and steps.  The
// geometry at capture time is in the header, so a replay knows which
// room it was.
static File     g_sess_file;
static uint32_t g_sess_seq = 0, g_sess_flush_ms = 0, g_sess_last_ms = 0;

static void session_start() {
    if (!g_sess.card_present || g_sess.recording) return;
    for (int n = 1; n < 10000; n++) {
        snprintf(g_sess.path, sizeof(g_sess.path), "%s/s%04d.bin", CP_DIR_SESSIONS, n);
        if (!SD.exists(g_sess.path)) break;
    }
    g_sess_file = SD.open(g_sess.path, FILE_WRITE);
    if (!g_sess_file) { g_sess.card_full = true; return; }
    CardputerSessionHeader h = {};
    h.magic = CP_SESSION_MAGIC; h.version = CP_SESSION_VERSION;
    h.n_beacons = g_rx.deploy.beacons; h.start_ms = millis();
    h.frame_bytes = sizeof(CardputerSessionFrame);
    h.mode = (uint8_t)g_mode; h.flags = CP_SF_HAS_IMU;
    if (mantis_mesh_geometry_confidence(&g_mesh) > 0.99f) h.flags |= CP_SF_GEOMETRY_MEASURED;
    for (int i = 0; i < 8 && i + 1 < MANTIS_SLOTS; i++) { h.beacon_x[i] = g_mesh.bx[i + 1]; h.beacon_y[i] = g_mesh.by[i + 1]; }
    snprintf(h.note, sizeof(h.note), "%s", cp_mode_name(g_mode));
    g_sess_file.write((const uint8_t *)&h, sizeof(h));
    g_sess.recording = true; g_sess.frames_written = 0; g_sess.bytes_written = sizeof(h);
    g_sess.dropped = 0; g_sess_seq = 0;
}

static void session_stop(bool truncated) {
    if (!g_sess.recording) return;
    g_sess.recording = false;
    if (!g_sess_file) return;
    // Patch the frame count in place: 0 in a file means "never closed".
    g_sess_file.seek(offsetof(CardputerSessionHeader, frame_count));
    const uint32_t n = g_sess.frames_written;
    g_sess_file.write((const uint8_t *)&n, sizeof(n));
    if (truncated) {
        g_sess_file.seek(offsetof(CardputerSessionHeader, flags));
        uint8_t f = CP_SF_HAS_IMU | CP_SF_TRUNCATED;
        g_sess_file.write(&f, 1);
    }
    g_sess_file.close();
}

static void session_tick(uint32_t now) {
    if (!g_sess.recording || now - g_sess_last_ms < 100) return;
    g_sess_last_ms = now;
    CardputerSessionFrame f = {};
    f.seq = g_sess_seq++; f.t_ms = now;
    for (uint8_t id = 1; id <= 8 && id < MANTIS_SLOTS; id++) {
        if (!mantis_store_fresh(&g_rx.store, id, now)) continue;
        const MantisPerspective &v = g_rx.store.by_id[id].view;
        if (!v.n_links) continue;
        int32_t a = 0, ph = 0, q = 0, r = 0, rn = 0;
        for (uint8_t k = 0; k < v.n_links; k++) {
            a += v.links[k].amp_q8; ph += v.links[k].phase_q12; q += v.links[k].quality;
            if (v.link_rssi[k]) { r += v.link_rssi[k]; rn++; }
        }
        const int i = id - 1;
        f.amp_q8[i] = (int16_t)(a / v.n_links); f.phase_q12[i] = (int16_t)(ph / v.n_links);
        f.quality[i] = (uint8_t)(q / v.n_links); f.rssi[i] = rn ? (int8_t)(r / rn) : 0;
        f.noise[i] = v.noise_floor; f.fresh_mask |= (uint8_t)(1u << i);
    }
    if (g_probe.known) { f.probe_x_q10 = (int16_t)(g_probe.x * 1024); f.probe_y_q10 = (int16_t)(g_probe.y * 1024); }
    f.imu_heading_q12 = (int16_t)(g_imu.heading_rad * 4096.0f / 8.0f);
    f.steps = (uint16_t)g_imu.steps;
    const size_t w = g_sess_file.write((const uint8_t *)&f, sizeof(f));
    if (w != sizeof(f)) { g_sess.card_full = true; session_stop(true); return; }
    g_sess.frames_written++; g_sess.bytes_written += w;
    if (now - g_sess_flush_ms > 2000) { g_sess_flush_ms = now; g_sess_file.flush(); }
}

// ── Drawing ───────────────────────────────────────────────────
static void draw_chrome(const char *title) {
    g_cv.fillSprite(CPC_BG);
    g_cv.fillRect(0, 0, CP_SCREEN_W, CP_HEADER_H, CPC_DIM);
    g_cv.setFont(&fonts::Font0);
    g_cv.setTextColor(CPC_INK, CPC_DIM);
    g_cv.setCursor(3, 4);
    g_cv.print(title);

    // Mode is ALWAYS on screen.  What this device can do depends on it
    // entirely, and a hidden mode is a question the operator cannot
    // answer while holding it.
    g_cv.setTextColor(CPC_MESH, CPC_DIM);
    g_cv.setCursor(CP_SCREEN_W - 84, 4);
    g_cv.print(cp_mode_name(g_mode));

    const int b = M5Cardputer.Power.getBatteryLevel();
    g_cv.setTextColor(b < 20 ? CPC_ALERT : CPC_MID, CPC_DIM);
    g_cv.setCursor(CP_SCREEN_W - 24, 4);
    g_cv.printf("%3d", b);

    if (g_audio.muted) {
        g_cv.setTextColor(CPC_WARN, CPC_BG);
        g_cv.setCursor(CP_SCREEN_W - 118, 4);
        g_cv.print("MUTE");
    }
}

static void draw_footer(const char *l, const char *r) {
    const int y = CP_SCREEN_H - CP_FOOTER_H;
    g_cv.fillRect(0, y, CP_SCREEN_W, CP_FOOTER_H, CPC_DIM);
    g_cv.setFont(&fonts::Font0);
    g_cv.setTextColor(CPC_INK, CPC_DIM);
    if (l && *l) { g_cv.setCursor(3, y + 3); g_cv.printf("<%s", l); }
    if (r && *r) { g_cv.setCursor(CP_SCREEN_W - 6 * (int)strlen(r) - 10, y + 3);
                   g_cv.printf("%s>", r); }
}

static void flush() {
    if (g_canvas_ok) g_cv.pushSprite(0, 0);
}

static const MantisProbePalette PAL = {
    CPC_BG, CPC_INK, CPC_MID, CPC_DIM, CPC_LIME, CPC_TEAL, CPC_VIOLET, CPC_ALERT, CPC_WARN, CPC_MESH };

// The map half: the SAME renderer the Core2 uses.
static void draw_map() {
    g_cv.setFont(&fonts::Font0);
    mantis_probe_draw_map(g_cv, g_rx, g_link, millis(), CP_MAP_CX, CP_MAP_CY, CP_MAP_R, PAL);
}

static void draw_status_col() {
    const int x = cp_status_x();
    g_cv.setFont(&fonts::Font0);
    int row = 0;

    g_cv.setTextColor(CPC_MID, CPC_BG);
    g_cv.setCursor(x, cp_row_y(row++)); g_cv.printf("bcn  %d", g_rx.deploy.beacons);
    g_cv.setCursor(x, cp_row_y(row++)); g_cv.printf("lnk  %d", g_mesh.n_chords);

    // FUSED contacts -- what survived cross-checking -- not raw shadowing.
    int believed = 0;
    for (int i = 0; i < MANTIS_FUSE_MAX; i++) if (mantis_fuse_believed(&g_rx.fusion.t[i])) believed++;
    g_cv.setTextColor(believed ? CPC_LIME : CPC_MID, CPC_BG);
    g_cv.setCursor(x, cp_row_y(row++)); g_cv.printf("cont %d", believed);
    for (int i = 0; i < MANTIS_FUSE_MAX && row < cp_rows() - 3; i++) {
        const MantisFused &t = g_rx.fusion.t[i];
        if (!mantis_fuse_believed(&t)) continue;
        g_cv.setTextColor(t.cls == MFC_STATIC ? CPC_TEAL : CPC_LIME, CPC_BG);
        g_cv.setCursor(x, cp_row_y(row++));
        g_cv.printf("%s %u", mantis_fuse_class_name(t.cls), t.witnesses);
    }
    const uint32_t now = millis();
    g_cv.setCursor(x, cp_row_y(row++));
    if (mantis_probe_anchor_live(&g_link, now)) {
        g_cv.setTextColor((g_link.st.flags & MAS_ALERT) ? CPC_ALERT : CPC_TEAL, CPC_BG);
        g_cv.printf("A %s", mantis_anchor_state_name(g_link.st.app_state));
    } else {
        g_cv.setTextColor(CPC_DIM, CPC_BG);
        g_cv.printf("A %s", mantis_probe_link_state(&g_link, now));
    }

    // Anything wrong gets the bottom rows, always.
    if (g_integ.geometry_suspect) {
        g_cv.setTextColor(CPC_ALERT, CPC_BG);
        g_cv.setCursor(x, cp_row_y(cp_rows() - 2));
        g_cv.printf("B%d MOVED", g_integ.worst_id);
    }
    const float conf = mantis_probe_confidence(&g_probe);
    if (conf >= 0.0f) {
        g_cv.setTextColor(conf > 0.6f ? CPC_MID : CPC_WARN, CPC_BG);
        g_cv.setCursor(x, cp_row_y(cp_rows() - 1));
        g_cv.printf("fit %2.0f%%", conf * 100.0f);
    }
}

// ── Screens ───────────────────────────────────────────────────
static void screen_splash() {
    draw_chrome("MANTIS");
    g_cv.setFont(&fonts::Font2);
    g_cv.setTextColor(CPC_LIME, CPC_BG);
    g_cv.setCursor(60, 45);  g_cv.print("MANTIS");
    g_cv.setTextColor(CPC_MID, CPC_BG);
    g_cv.setFont(&fonts::Font0);
    g_cv.setCursor(52, 70);  g_cv.print("Cardputer-Adv Probe");
    g_cv.setCursor(52, 84);  g_cv.print(cp_mode_name(g_mode));
    draw_footer("", "");
    flush();
    if (screen_age() > 2200) go(CPS_DISCOVERY);
}

static void screen_discovery() {
    draw_chrome("DISCOVERY");
    g_cv.setFont(&fonts::Font0);
    g_cv.setTextColor(CPC_INK, CPC_BG);
    g_cv.setCursor(6, cp_row_y(0));
    g_cv.printf("beacons heard: %d", g_rx.deploy.beacons);
    g_cv.setCursor(6, cp_row_y(1));
    g_cv.printf("anchor: %s", mantis_probe_anchor_live(&g_link, millis()) ? "linked" : "none");

    // What this deployment can do, and the single most useful thing to
    // add next.  Both from the shared capability layer, so the menu can
    // never offer something the solver cannot do.
    const MantisDeployment &dep = g_rx.deploy;
    const MantisCaps &caps = g_rx.caps;

    g_cv.setTextColor(caps.localize ? CPC_LIME : CPC_WARN, CPC_BG);
    g_cv.setCursor(6, cp_row_y(2));
    g_cv.printf("%s  %d chords", caps.localize ? "LOCATING" : "presence only",
                caps.chords);
    if (caps.blocker[0]) {
        g_cv.setTextColor(CPC_MID, CPC_BG);
        g_cv.setCursor(6, cp_row_y(4));
        g_cv.print(caps.blocker);
    }

    const bool ready = g_rx.deploy.beacons || mantis_probe_anchor_live(&g_link, millis());
    draw_footer("", ready ? "go" : "");
    flush();
    if ((cp_was_short(&g_in, BTN_RIGHT) || screen_age() > 6000) && ready) go(CPS_DASHBOARD);
}

static void screen_dashboard() {
    draw_chrome("RADAR");
    draw_map();
    g_cv.drawLine(CP_SPLIT_X, CP_CONTENT_Y, CP_SPLIT_X,
                  CP_SCREEN_H - CP_FOOTER_H, CPC_DIM);
    draw_status_col();
    if ((int32_t)(millis() - g_alarm_flash_until) < 0) {
        g_cv.drawRect(0, CP_CONTENT_Y, CP_SCREEN_W, CP_CONTENT_H, CPC_ALERT);
        g_cv.setTextColor(CPC_ALERT, CPC_BG);
        g_cv.setCursor(4, CP_CONTENT_Y + 2);
        g_cv.print(CP_ALARMS[g_last_alarm].label);
    }
    if (g_sess.recording) { g_cv.fillCircle(CP_SCREEN_W - 6, CP_CONTENT_Y + 5, 3, CPC_ALERT); }
    draw_footer("menu", g_link.anchor_seen ? "anchor" : "");
    flush();

    if (cp_was_short(&g_in, BTN_LEFT))  go(CPS_SETTINGS);
    if (cp_was_short(&g_in, BTN_RIGHT) && g_link.anchor_seen) go(CPS_LINK);
    if (g_in.tab && g_link.anchor_seen) go(CPS_LINK);
}

static void screen_alarms() {
    draw_chrome("ALARMS");
    g_cv.setFont(&fonts::Font0);
    static int sel = 1;
    if (g_in.down) sel = (sel % (CPA_COUNT - 1)) + 1;
    if (g_in.up)   sel = (sel <= 1) ? (CPA_COUNT - 1) : (sel - 1);
    if (cp_was_short(&g_in, BTN_RIGHT)) g_audio.enabled[sel] = !g_audio.enabled[sel];
    if (g_in.mute_key) g_audio.muted = !g_audio.muted;

    for (int i = 1; i < CPA_COUNT; i++) {
        const bool on = g_audio.enabled[i];
        g_cv.setTextColor(i == sel ? CPC_LIME : (on ? CPC_INK : CPC_DIM), CPC_BG);
        g_cv.setCursor(6, cp_row_y(i - 1));
        g_cv.printf("%c %-12s %-4s %s", i == sel ? '>' : ' ',
                    CP_ALARMS[i].label, on ? "ON" : "off",
                    cp_audio_source(&g_audio, (CardputerAlarm)i));
    }
    g_cv.setTextColor(CPC_MID, CPC_BG);
    g_cv.setCursor(6, CP_SCREEN_H - CP_FOOTER_H - 11);
    g_cv.printf("SD %s   M=mute", g_audio.sd_present ? "ok" : "absent");
    draw_footer("back", "toggle");
    flush();
    if (cp_was_short(&g_in, BTN_LEFT)) go(CPS_SETTINGS);
}

static void screen_sessions() {
    draw_chrome("SESSIONS");
    g_cv.setFont(&fonts::Font0);
    g_cv.setTextColor(CPC_INK, CPC_BG);
    g_cv.setCursor(6, cp_row_y(0));
    g_cv.printf("status: %s", cp_session_status(&g_sess));
    g_cv.setCursor(6, cp_row_y(1));
    g_cv.printf("frames: %lu", (unsigned long)g_sess.frames_written);
    g_cv.setCursor(6, cp_row_y(2));
    g_cv.printf("rate:   %lu B/s", (unsigned long)cp_session_bytes_per_s(30));
    if (g_sess.dropped) {
        g_cv.setTextColor(CPC_WARN, CPC_BG);
        g_cv.setCursor(6, cp_row_y(3));
        g_cv.printf("dropped %lu (card slow)", (unsigned long)g_sess.dropped);
    }
    if (g_sess.recording) {
        g_cv.setTextColor(CPC_MID, CPC_BG);
        g_cv.setCursor(6, cp_row_y(4)); g_cv.printf("%s", g_sess.path);
    }
    draw_footer("back", g_sess.recording ? "stop" : "record");
    flush();
    if (cp_was_short(&g_in, BTN_LEFT))  go(CPS_SETTINGS);
    if (cp_was_short(&g_in, BTN_RIGHT)) { if (g_sess.recording) session_stop(false); else session_start(); }
}

static void screen_mesh() {
    draw_chrome("MESH");
    g_cv.setFont(&fonts::Font0);
    int row = 0;
    for (uint8_t i = 1; i <= g_mesh.n_beacons && i < 7; i++) {
        if (!g_mesh.live[i]) continue;
        const bool sus = (g_integ.geometry_suspect && g_integ.worst_id == i);
        g_cv.setTextColor(sus ? CPC_ALERT : (g_mesh.pos_measured[i] ? CPC_LIME : CPC_MID), CPC_BG);
        g_cv.setCursor(6, cp_row_y(row++));
        g_cv.printf("B%d %s (%+.2f,%+.2f)%s", i,
                    g_mesh.pos_measured[i] ? "surveyed" : "assumed ",
                    g_mesh.bx[i], g_mesh.by[i], sus ? " MOVED" : "");
    }
    draw_footer("back", "");
    flush();
    if (cp_was_short(&g_in, BTN_LEFT)) go(CPS_SETTINGS);
}

static void screen_settings() {
    draw_chrome("SETTINGS");
    g_cv.setFont(&fonts::Font0);
    static int sel = 0;
    const char *items[] = { "Alarms", "Sessions", "Mesh health", "Radar", "Anchor remote" };
    // Calibrate needs a fixed anchor, which a hand-held device is not.
    // Shown either way, with the reason -- a greyed row that explains
    // itself is help; one that does not is a support question.
    const bool avail[] = { true, g_sess.card_present, true, true, g_link.anchor_seen };
    const int n = 5;
    if (g_in.down) sel = (sel + 1) % n;
    if (g_in.up)   sel = (sel + n - 1) % n;
    for (int i = 0; i < n; i++) {
        const bool ok = avail[i];
        g_cv.setTextColor(!ok ? CPC_DIM : (i == sel ? CPC_LIME : CPC_INK), CPC_BG);
        g_cv.setCursor(6, cp_row_y(i));
        g_cv.printf("%c %s", i == sel ? '>' : ' ', items[i]);
        if (!ok && i == sel) {
            g_cv.setTextColor(CPC_WARN, CPC_BG);
            g_cv.setCursor(cp_status_x(), cp_row_y(i));
            g_cv.print(i == 4 ? "no anchor heard" : (i == 1 ? "no SD card" : ""));
        }
    }
    draw_footer("radar", "select");
    flush();
    if (cp_was_short(&g_in, BTN_LEFT) || g_in.esc) go(CPS_DASHBOARD);
    if (cp_was_short(&g_in, BTN_RIGHT) && avail[sel]) {
        switch (sel) {
            case 0: go(CPS_ALARMS);    break;
            case 1: go(CPS_SESSIONS);  break;
            case 2: go(CPS_MESH);      break;
            case 3: go(CPS_DASHBOARD); break;
            case 4: go(CPS_LINK);      break;
        }
    }
}

// ── THE ANCHOR, IN YOUR HAND ──────────────────────────────────
// Mirrors the anchor's screen and drives its two buttons:
//   ,  or  del    = anchor LEFT        hold = long press
//   /  or  enter  = anchor RIGHT       hold = long press
//   esc / tab     = back to the radar
// Everything shown came from the anchor, about the frame it last drew.
static void screen_link() {
    const uint32_t now = millis();
    const bool live = mantis_probe_anchor_live(&g_link, now);
    const MantisAnchorStatus &st = g_link.st;
    draw_chrome("ANCHOR");
    g_cv.setFont(&fonts::Font2);
    g_cv.setTextColor(!live ? CPC_WARN : ((st.flags & MAS_ALERT) ? CPC_ALERT : CPC_LIME), CPC_BG);
    g_cv.setCursor(4, CP_CONTENT_Y + 1);
    g_cv.print(live ? mantis_anchor_state_name(st.app_state) : "NO ANCHOR");
    g_cv.setFont(&fonts::Font0);
    const char *ls = mantis_probe_link_state(&g_link, now);
    g_cv.setTextColor(g_link.key_tries ? CPC_WARN : CPC_MID, CPC_BG);
    g_cv.setCursor(CP_SCREEN_W - 4 - 6 * (int)strlen(ls), CP_CONTENT_Y + 4);
    g_cv.print(ls);

    int y = CP_CONTENT_Y + 20;
    if (live) {
        char lines[4][48];
        const int n = mantis_wrap(st.hint, 38, lines, 4);
        g_cv.setTextColor(CPC_INK, CPC_BG);
        for (int i = 0; i < n; i++) { g_cv.setCursor(4, y); g_cv.print(lines[i]); y += 10; }
        if (st.steps) {
            g_cv.setTextColor(CPC_TEAL, CPC_BG);
            g_cv.setCursor(4, y); g_cv.printf("step %u / %u", st.step, st.steps); y += 10;
        }
        if (st.progress) {
            g_cv.drawRect(4, y + 1, CP_SCREEN_W - 8, 7, CPC_DIM);
            g_cv.fillRect(5, y + 2, (CP_SCREEN_W - 10) * st.progress / 100, 5, CPC_LIME);
            y += 10;
        }
        g_cv.setTextColor(CPC_MID, CPC_BG);
        g_cv.setCursor(4, CP_SCREEN_H - CP_FOOTER_H - 10);
        g_cv.printf("bcn %u mesh %u trk %u %s", st.beacons, st.mesh_beacons, st.n_tracks,
                    st.title);
    } else {
        g_cv.setTextColor(CPC_MID, CPC_BG);
        g_cv.setCursor(4, y);
        g_cv.print(g_link.anchor_seen ? "Anchor went quiet." : "Power on the T-Display anchor.");
    }
    char l[16] = "", r[16] = "";
    if (live) { snprintf(l, sizeof(l), "%s", st.left); snprintf(r, sizeof(r), "%s", st.right); }
    draw_footer(l, r);
    flush();

    if (g_in.esc || g_in.tab) { go(CPS_DASHBOARD); return; }
    if (!live) return;
    if (cp_was_short(&g_in, BTN_LEFT))  mantis_probe_press(&g_link, MRK_LEFT, now);
    if (cp_was_short(&g_in, BTN_RIGHT)) mantis_probe_press(&g_link, MRK_RIGHT, now);
    if (cp_was_long(&g_in, BTN_LEFT))   mantis_probe_press(&g_link, MRK_LEFT_LONG, now);
    if (cp_was_long(&g_in, BTN_RIGHT))  mantis_probe_press(&g_link, MRK_RIGHT_LONG, now);
}

// ── Arduino ───────────────────────────────────────────────────
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);          // true = enable keyboard
    M5Cardputer.Display.setRotation(1);    // landscape

    // 240x135 at 16bpp is 64,800 bytes.  Same reasoning as the Core2:
    // halve the depth before giving up, and if it still fails say so on
    // the panel instead of running with a blank screen.
    g_canvas_ok = g_cv.createSprite(CP_SCREEN_W, CP_SCREEN_H);
    if (!g_canvas_ok) {
        g_cv.setColorDepth(8);
        g_canvas_ok = g_cv.createSprite(CP_SCREEN_W, CP_SCREEN_H);
    }
    if (!g_canvas_ok) {
        M5Cardputer.Display.fillScreen(CPC_BG);
        M5Cardputer.Display.setTextColor(CPC_ALERT, CPC_BG);
        M5Cardputer.Display.setCursor(6, 56);
        M5Cardputer.Display.print("display buffer alloc failed");
    }

    cp_input_begin(&g_in);
    cp_imu_begin(&g_imu, CP_STRIDE_M_DEFAULT);
    cp_audio_begin(&g_audio);
    cp_turn_begin(&g_turn, &g_imu);

    // Probe role: one receiver, no T-Display until we hear one.
    mantis_rx_begin(&g_rx, 1.2f, /*tdisplays*/0, /*cardputers*/1, /*docked*/false);
    mantis_probe_link_begin(&g_link);
    mantis_probe_radio_begin();     // without this the UI renders nothing real

    // ── SD MOUNT, WITH THE PINS IT ACTUALLY NEEDS ─────────────
    // SD.begin() with no arguments uses the default VSPI pins, which are
    // NOT this board's, so it returns false and every SD feature dies
    // with nothing on screen to explain why.  The bus has to be started
    // explicitly first.  This is the vendor's documented sequence.
    //
    // Still optional: alarms fall back to tones and sessions simply do
    // not record.  Nothing that matters depends on the card.
    SPI.begin(CP_SD_SCK_PIN, CP_SD_MISO_PIN, CP_SD_MOSI_PIN, CP_SD_CS_PIN);
    g_sess.card_present = SD.begin(CP_SD_CS_PIN, SPI, CP_SD_FREQ);
    g_audio.sd_present  = g_sess.card_present;
    if (g_sess.card_present) {
        SD.mkdir(CP_DIR_ROOT);
        SD.mkdir(CP_DIR_ALARMS);
        SD.mkdir(CP_DIR_SESSIONS);
        SD.mkdir(CP_DIR_LOGS);
        SD.mkdir(CP_DIR_CONFIG);
        // Decoder task.  If it will not start, every alarm falls back to
        // a tone -- which is why the fallback exists.
        cp_mp3_begin();
        for (int i = 1; i < CPA_COUNT; i++) {
            char p[96];
            snprintf(p, sizeof(p), "%s/%s", CP_ALARM_DIR, CP_ALARMS[i].file);
            g_audio.file_ok[i] = SD.exists(p);
        }
    }

    g_screen_entered_ms = millis();
}

void loop() {
    M5Cardputer.update();
    poll_keys();
    poll_imu();
    update_mode();

    // Mute is global and must work from ANY screen: an alarm you cannot
    // silence instantly is a liability, not a feature.
    if (g_in.mute_key && g_screen != CPS_ALARMS) g_audio.muted = !g_audio.muted;

    // Mesh solve, rate-limited well below the display rate -- the
    // tomography does not get better by running faster than the beacons
    // report.
    // Drain the radio EVERY loop, not on the solve interval: reports
    // arrive faster than the solve runs and the queue is eight deep.
    mantis_probe_pump(&g_rx, millis());
    mantis_probe_service(&g_link, MANTIS_PROBE_KIND_CARDPUTER, millis());
    tone_tick();

    // One call. It rate-limits itself, latches the baseline on coverage,
    // applies dense link quality, runs both channels and fuses them.
    static MantisAlarmWatch watch = {};
    if (mantis_rx_solve(&g_rx, millis(), 0.1f)) {
        const CardputerAlarm k = mantis_alarm_eval(&watch, &g_rx, &g_link, millis());
        if (k != CPA_NONE) play_alarm(k);
    }
    session_tick(millis());

    switch (g_screen) {
        case CPS_SPLASH:    screen_splash();    break;
        case CPS_DISCOVERY: screen_discovery(); break;
        case CPS_DASHBOARD: screen_dashboard(); break;
        case CPS_ALARMS:    screen_alarms();    break;
        case CPS_SESSIONS:  screen_sessions();  break;
        case CPS_MESH:      screen_mesh();      break;
        case CPS_LINK:      screen_link();      break;
        case CPS_SETTINGS:  screen_settings();  break;
        default:
            // Never sit on an unhandled screen: a frozen display with
            // live keys is indistinguishable from a crash.
            go(CPS_DASHBOARD);
            break;
    }
}
