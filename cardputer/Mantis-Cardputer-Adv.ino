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
#include "cardputer_imu.h"
#include "cardputer_audio.h"
#include "cardputer_mp3.h"
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

// ── State ─────────────────────────────────────────────────────
static M5Canvas          g_cv(&M5Cardputer.Display);
static CardputerInput    g_in;
static CardputerImu      g_imu;
static CardputerTurn     g_turn;
static CardputerAudio    g_audio;
static CardputerSession  g_sess;
static CardputerScreen   g_screen = CPS_SPLASH;
static CardputerMode     g_mode   = CPM_SEARCHING;

static MantisMesh            g_mesh;
static MantisPerspectiveStore g_store;
static MantisTargetSet       g_targets;
static MantisProbe           g_probe;
static MantisStaticScene     g_static;
static MantisIntegrity       g_integ;

static uint32_t g_screen_entered_ms = 0;
static uint8_t  g_dash_view = 0;
static bool     g_canvas_ok = false;

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
static void play_alarm(CardputerAlarm k) {
    const uint32_t now = millis();
    if (cp_audio_should_fire(&g_audio, k, now) != CPF_FIRE) return;
    cp_audio_mark_fired(&g_audio, k, now);

    const CardputerAlarmDef &d = CP_ALARMS[k];

    // MP3 from SD first.  cp_mp3_play() QUEUES and returns immediately --
    // the decoder task does the work, so this function never blocks the
    // sensing loop no matter how long the clip is.
    if (g_audio.sd_present && g_audio.file_ok[k] && !cp_mp3_busy()) {
        char path[96];
        snprintf(path, sizeof(path), "%s/%s", CP_ALARM_DIR, d.file);
        if (cp_mp3_play(path, g_audio.volume)) return;
    }

    // Tone fallback.  Reached when there is no card, no file, the decoder
    // never started, or a clip is already sounding.  Short and blocking
    // is acceptable here precisely because it IS short -- a few hundred
    // milliseconds against a 100 ms solve interval.
    M5Cardputer.Speaker.setVolume(g_audio.volume);
    for (uint8_t i = 0; i < d.repeats; i++) {
        M5Cardputer.Speaker.tone(d.tone_hz, d.tone_ms, CP_MP3_CHANNEL + 1);
        delay(d.tone_ms + 60);
    }
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
static uint8_t g_tdisplay_seen = 0;
static uint32_t g_last_tdisplay_ms = 0;

static void update_mode() {
    const uint32_t now = millis();
    const bool anchor_live = (now - g_last_tdisplay_ms) < 2000;
    const uint8_t nb = g_mesh.n_beacons;

    CardputerMode m;
    if (anchor_live && g_tdisplay_seen >= 2)      m = CPM_PROBE_STEREO;
    else if (anchor_live)                          m = CPM_PROBE_SOLO;
    else if (nb >= 3)                              m = CPM_STANDALONE;
    else                                           m = CPM_SEARCHING;

    if (m != g_mode) {
        g_mode = m;
        // A mode change alters what the device can do, so it is worth an
        // alarm: an operator whose anchor just died should not find out
        // by noticing the screen looks different.
        if (m == CPM_STANDALONE || m == CPM_SEARCHING) play_alarm(CPA_MESH_FAULT);
    }
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

// The map half: beacons, mesh glow, targets.
static void draw_map() {
    const int cx = CP_MAP_CX, cy = CP_MAP_CY, r = CP_MAP_R;
    g_cv.drawCircle(cx, cy, r, CPC_DIM);

    // Mesh underglow first -- backdrop, never over the tracks.
    float hi = 0;
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++)
        if (g_mesh.field[i] > hi) hi = g_mesh.field[i];
    if (hi > MANTIS_MESH_MIN_LLR * 0.5f) {
        const float step = (2.0f * g_mesh.extent) / (float)MANTIS_TOMO_DIM;
        const float sc   = (float)r / g_mesh.extent;
        for (int gy = 0; gy < MANTIS_TOMO_DIM; gy++)
            for (int gx = 0; gx < MANTIS_TOMO_DIM; gx++) {
                const float v = g_mesh.field[gy * MANTIS_TOMO_DIM + gx];
                if (v <= 0) continue;
                const float t = v / hi;
                if (t < 0.45f) continue;
                const float ux = -g_mesh.extent + (gx + 0.5f) * step;
                const float uy =  g_mesh.extent - (gy + 0.5f) * step;
                const int px = cx + (int)(ux * sc), py = cy - (int)(uy * sc);
                if ((px-cx)*(px-cx) + (py-cy)*(py-cy) > r*r) continue;
                g_cv.fillCircle(px, py, t > 0.8f ? 3 : 2, CPC_MESH);
            }
    }

    // Beacons, at MEASURED positions when the survey has run.
    for (uint8_t i = 1; i <= g_mesh.n_beacons && i < MANTIS_SLOTS; i++) {
        if (!g_mesh.have_pos[i]) continue;
        const float sc = (float)r / g_mesh.extent;
        const int px = cx + (int)(g_mesh.bx[i] * sc);
        const int py = cy - (int)(g_mesh.by[i] * sc);
        const bool suspect = (g_integ.geometry_suspect && g_integ.worst_id == i);
        g_cv.fillCircle(px, py, 3, suspect ? CPC_ALERT : CPC_LIME);
        g_cv.setFont(&fonts::Font0);
        g_cv.setTextColor(CPC_INK, CPC_BG);
        g_cv.setCursor(px - 2, py - 10);
        g_cv.printf("%d", i);
    }

    // Targets on top.
    for (int i = 0; i < g_targets.n; i++) {
        const MantisTarget &t = g_targets.t[i];
        if (!t.active) continue;
        const float sc = (float)r / g_mesh.extent;
        const int px = cx + (int)(t.x * sc), py = cy - (int)(t.y * sc);
        const uint16_t c = (t.kind == MT_STATIC) ? CPC_TEAL : CPC_LIME;
        g_cv.fillCircle(px, py, 4, c);
        // A STATIC target gets a ring, because "present but not moving"
        // is the case ordinary CSI radar cannot see and the operator
        // should be able to tell at a glance that this is that case.
        if (t.kind == MT_STATIC) g_cv.drawCircle(px, py, 7, c);
    }

    // The operator, from the probe's own known position.
    if (g_probe.known) {
        const float sc = (float)r / g_mesh.extent;
        const int px = cx + (int)(g_probe.x * sc);
        const int py = cy - (int)(g_probe.y * sc);
        g_cv.drawLine(px - 4, py, px + 4, py, CPC_VIOLET);
        g_cv.drawLine(px, py - 4, px, py + 4, CPC_VIOLET);
    }
}

static void draw_status_col() {
    const int x = cp_status_x();
    g_cv.setFont(&fonts::Font0);
    int row = 0;

    g_cv.setTextColor(CPC_MID, CPC_BG);
    g_cv.setCursor(x, cp_row_y(row++)); g_cv.printf("bcn  %d", g_mesh.n_beacons);
    g_cv.setCursor(x, cp_row_y(row++)); g_cv.printf("lnk  %d", g_mesh.n_chords);

    g_cv.setTextColor(g_targets.n ? CPC_LIME : CPC_MID, CPC_BG);
    g_cv.setCursor(x, cp_row_y(row++)); g_cv.printf("tgt  %d", g_targets.n);

    for (int i = 0; i < g_targets.n && row < cp_rows() - 3; i++) {
        const MantisTarget &t = g_targets.t[i];
        g_cv.setTextColor(t.kind == MT_STATIC ? CPC_TEAL : CPC_LIME, CPC_BG);
        g_cv.setCursor(x, cp_row_y(row++));
        g_cv.printf("%s", t.kind == MT_STATIC ? "STATIC" : "MOVING");
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
    g_cv.printf("beacons heard: %d", g_mesh.n_beacons);
    g_cv.setCursor(6, cp_row_y(1));
    g_cv.printf("anchor: %s", (millis() - g_last_tdisplay_ms) < 2000 ? "yes" : "none");

    const CardputerCaps c = cp_caps(g_mode, g_mesh.n_beacons);
    g_cv.setTextColor(CPC_WARN, CPC_BG);
    g_cv.setCursor(6, cp_row_y(3));
    g_cv.print(c.why_limited);

    draw_footer("", g_mesh.n_beacons ? "go" : "");
    flush();
    if (cp_was_short(&g_in, BTN_RIGHT) && g_mesh.n_beacons) go(CPS_DASHBOARD);
}

static void screen_dashboard() {
    draw_chrome("RADAR");
    draw_map();
    g_cv.drawLine(CP_SPLIT_X, CP_CONTENT_Y, CP_SPLIT_X,
                  CP_SCREEN_H - CP_FOOTER_H, CPC_DIM);
    draw_status_col();
    draw_footer("menu", "views");
    flush();

    if (cp_was_short(&g_in, BTN_LEFT))  go(CPS_SETTINGS);
    if (g_in.tab) g_dash_view = (uint8_t)((g_dash_view + 1) % 3);
    if (g_in.esc) go(CPS_DASHBOARD);
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
    draw_footer("back", g_sess.recording ? "stop" : "record");
    flush();
    if (cp_was_short(&g_in, BTN_LEFT))  go(CPS_SETTINGS);
    if (cp_was_short(&g_in, BTN_RIGHT)) g_sess.recording = !g_sess.recording;
}

static void screen_mesh() {
    draw_chrome("MESH");
    g_cv.setFont(&fonts::Font0);
    for (uint8_t i = 1; i <= g_mesh.n_beacons && i < 7; i++) {
        const bool sus = (g_integ.geometry_suspect && g_integ.worst_id == i);
        g_cv.setTextColor(sus ? CPC_ALERT : (g_mesh.have_pos[i] ? CPC_LIME : CPC_MID), CPC_BG);
        g_cv.setCursor(6, cp_row_y(i - 1));
        g_cv.printf("B%d %s (%+.2f,%+.2f)%s", i,
                    g_mesh.have_pos[i] ? "surveyed" : "assumed ",
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
    const char *items[] = { "Alarms", "Sessions", "Mesh health", "Radar", "Calibrate" };
    const int n = 5;
    if (g_in.down) sel = (sel + 1) % n;
    if (g_in.up)   sel = (sel + n - 1) % n;
    for (int i = 0; i < n; i++) {
        g_cv.setTextColor(i == sel ? CPC_LIME : CPC_INK, CPC_BG);
        g_cv.setCursor(6, cp_row_y(i));
        g_cv.printf("%c %s", i == sel ? '>' : ' ', items[i]);
    }
    draw_footer("radar", "select");
    flush();
    if (cp_was_short(&g_in, BTN_LEFT) || g_in.esc) go(CPS_DASHBOARD);
    if (cp_was_short(&g_in, BTN_RIGHT)) {
        switch (sel) {
            case 0: go(CPS_ALARMS);    break;
            case 1: go(CPS_SESSIONS);  break;
            case 2: go(CPS_MESH);      break;
            case 3: go(CPS_DASHBOARD); break;
            case 4: go(CPS_CAL_WALK);  break;
        }
    }
}

// The cal walk, with the IMU compass -- the screen that most justifies
// this hardware.  The red needle is MEASURED here, not estimated.
static void screen_cal_walk() {
    draw_chrome("CAL WALK");
    const int cx = CP_MAP_CX, cy = CP_MAP_CY, r = CP_MAP_R;
    g_cv.drawCircle(cx, cy, r, CPC_DIM);

    const float imu = g_imu.heading_rad;
    const int ix = cx + (int)(sinf(imu) * r * 0.85f);
    const int iy = cy - (int)(cosf(imu) * r * 0.85f);
    g_cv.drawLine(cx, cy, ix, iy, CPC_ALERT);

    g_cv.setFont(&fonts::Font0);
    const int x = cp_status_x();
    g_cv.setTextColor(CPC_INK, CPC_BG);
    g_cv.setCursor(x, cp_row_y(0)); g_cv.printf("turn %5.0f", cp_turn_degrees(&g_turn));
    g_cv.setCursor(x, cp_row_y(1)); g_cv.printf("step %5lu", (unsigned long)g_imu.steps);
    g_cv.setCursor(x, cp_row_y(2)); g_cv.printf("dist %5.1fm",
                   cp_turn_distance_m(&g_turn, &g_imu));
    g_cv.setTextColor(g_imu.bias_ready ? CPC_MID : CPC_WARN, CPC_BG);
    g_cv.setCursor(x, cp_row_y(4));
    g_cv.print(g_imu.bias_ready ? "imu ready" : "imu warming");

    draw_footer("back", "mark");
    flush();
    if (cp_was_short(&g_in, BTN_LEFT))  go(CPS_SETTINGS);
    if (cp_was_short(&g_in, BTN_RIGHT)) cp_turn_begin(&g_turn, &g_imu);
}

// ── Arduino ───────────────────────────────────────────────────
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);          // true = enable keyboard
    M5Cardputer.Display.setRotation(1);    // landscape

    g_canvas_ok = g_cv.createSprite(CP_SCREEN_W, CP_SCREEN_H);

    cp_input_begin(&g_in);
    cp_imu_begin(&g_imu, CP_STRIDE_M_DEFAULT);
    cp_audio_begin(&g_audio);
    cp_turn_begin(&g_turn, &g_imu);

    g_mesh = MantisMesh{};
    g_mesh.extent = 1.2f;
    g_probe = MantisProbe{};

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
    static uint32_t last_solve = 0;
    if (millis() - last_solve >= 100) {
        last_solve = millis();
        if (g_mesh.base_ready) {
            mantis_mesh_solve(&g_mesh);
            mantis_targets_extract(&g_targets, &g_mesh, &g_store,
                                   g_mesh.last_seq, 0.1f, millis());
            mantis_integrity_check(&g_integ, &g_mesh, 0.10f);
        }
    }

    switch (g_screen) {
        case CPS_SPLASH:    screen_splash();    break;
        case CPS_DISCOVERY: screen_discovery(); break;
        case CPS_DASHBOARD: screen_dashboard(); break;
        case CPS_ALARMS:    screen_alarms();    break;
        case CPS_SESSIONS:  screen_sessions();  break;
        case CPS_MESH:      screen_mesh();      break;
        case CPS_CAL_WALK:  screen_cal_walk();  break;
        case CPS_SETTINGS:  screen_settings();  break;
        default:
            // Never sit on an unhandled screen: a frozen display with
            // live keys is indistinguishable from a crash.
            go(CPS_DASHBOARD);
            break;
    }
}
