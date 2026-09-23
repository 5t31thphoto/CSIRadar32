// ═══════════════════════════════════════════════════════════════
//  MANTIS — M5Stack CORE2 PROBE
//  The primary in-hand device
// ═══════════════════════════════════════════════════════════════
//
//  Same role as the Cardputer: a FULL receiver that stays in the
//  operator's hand and is never an anchor.  Same mantis inference, same
//  shared capability gating, so a deployment does not care which
//  hand-held device is present.
//
//  ── EVERYTHING GOES THROUGH M5Unified ────────────────────────
//
//  M5Unified abstracts the parts that differ between Core2 revisions,
//  and they differ in ways that matter:
//
//      IMU     MPU6886 on v1.0/v1.1, BMI270 on v1.3   -> M5.Imu
//      PMIC    AXP192 on v1.0/v1.3, AXP2101 on v1.1   -> M5.Power
//      touch   FT6336U                                 -> M5.Touch
//
//  Talking to those chips directly -- which a lot of Core2 code does --
//  gives firmware that works on one revision of the same product and
//  silently fails on another.  Nothing in this file names a chip.
//
//  ── THE "BUTTONS" ARE TOUCH ZONES ────────────────────────────
//
//  Core2 has no A/B/C buttons.  It has three dot-shaped capacitive
//  zones printed below the glass, and M5Unified maps them to
//  M5.BtnA/BtnB/BtnC so they behave like buttons.  That mapping is the
//  documented way to use them and it is what this firmware consumes --
//  the zones and the touchscreen are the same sensor, read two ways.
//
//  ── ALERTS ───────────────────────────────────────────────────
//
//  Speaker AND vibration motor, both configurable per alarm in the
//  menu.  The motor matters more than it sounds: a device in a pocket
//  or a loud room is exactly where an audible alert fails, and haptics
//  are the only channel that still works with the speaker muted.
//
//  Build: board = m5stack-core2, partitions default_16MB,
//         M5Unified + M5GFX, -DBOARD_HAS_PSRAM
// ═══════════════════════════════════════════════════════════════
#include <M5Unified.h>
#include <SD.h>
#include <SPI.h>

#include "core2_platform.h"
#include "core2_input.h"
#include "mantis_imu.h"      // shared: pure arithmetic, no Cardputer in it
#include "mantis_alarms.h"    // shared: alarm table and debounce
#include "mantis_mp3.h"      // shared: decoder task

#include "mantis_air.h"
#include "mantis_receiver.h"
#include "mantis_caps.h"
#include "mantis_fuse.h"
#include "mantis_control.h"
#include "mantis_probe_radio.h"

// ── State ─────────────────────────────────────────────────────
static M5Canvas       g_cv(&M5.Display);
static Core2Input     g_in;
static CardputerImu   g_imu;
static CardputerTurn  g_turn;
static CardputerAudio g_audio;
static MantisReceiver g_rx;
static bool           g_canvas_ok = false;
static MantisProbeLink g_link;

// Haptics, per alarm.  Stored alongside the audio enables so the menu
// can offer both on one row.
static bool g_vibe[CPA_COUNT];
static bool g_vibe_master = true;

typedef enum : uint8_t {
    S_SPLASH = 0, S_DISCOVERY, S_RADAR, S_MENU, S_ALARMS, S_MESH, S_CAL,
    S_COUNT
} Screen;
static Screen   g_screen = S_SPLASH;
static uint32_t g_entered = 0;
static int      g_sel = 0;

static uint32_t age() { return millis() - g_entered; }
static void go(Screen s) { if (s != g_screen) { g_screen = s; g_entered = millis(); g_sel = 0; } }

// ── Palette ───────────────────────────────────────────────────
// Same MEANING as the other two platforms, so an operator moving
// between devices does not have to relearn it.
#define C_BG     0x0000
#define C_INK    0xFFFF
#define C_MID    0x8410
#define C_DIM    0x4208
#define C_LIME   0x07E0
#define C_TEAL   0x0679
#define C_VIOLET 0xB01F
#define C_ALERT  0xF800
#define C_WARN   0xFD20
#define C_MESH   0xFD60

// ── Alerts ────────────────────────────────────────────────────
static void fire_alarm(CardputerAlarm k) {
    const uint32_t now = millis();
    if (cp_audio_should_fire(&g_audio, k, now) != CPF_FIRE) {
        // Even when the SOUND is suppressed, haptics may still be
        // wanted: muting a speaker in a quiet room should not also
        // disable the alert.  Debounce is shared, the channels are not.
        if (g_vibe_master && g_vibe[k] && !g_audio.muted) M5.Power.setVibration(180);
        return;
    }
    cp_audio_mark_fired(&g_audio, k, now);

    if (g_vibe_master && g_vibe[k]) M5.Power.setVibration(180);

    const CardputerAlarmDef &d = CP_ALARMS[k];
    if (g_audio.sd_present && g_audio.file_ok[k] && !cp_mp3_busy()) {
        char path[96];
        snprintf(path, sizeof(path), "%s/%s", C2_ALARM_DIR, d.file);
        if (cp_mp3_play(path, g_audio.volume)) return;
    }
    M5.Speaker.setVolume(g_audio.volume);
    for (uint8_t i = 0; i < d.repeats; i++) {
        M5.Speaker.tone(d.tone_hz, d.tone_ms);
        delay(d.tone_ms + 60);
    }
}

// The motor has no auto-off, so it has to be stopped explicitly or it
// runs until the next call.  Easy to miss, and the symptom is a device
// that buzzes forever after one alert.
static uint32_t g_vibe_until = 0;
static void vibe_tick() {
    if (g_vibe_until && millis() > g_vibe_until) {
        M5.Power.setVibration(0);
        g_vibe_until = 0;
    }
}

// ── Input ─────────────────────────────────────────────────────
static void poll_input() {
    c2_input_new_frame(&g_in);
    // M5Unified maps the three capacitive zones to these.  They are not
    // physical buttons; the mapping is the documented way to read them.
    c2_input_buttons(&g_in, M5.BtnA.isPressed(), M5.BtnB.isPressed(),
                     M5.BtnC.isPressed(), millis());
    auto t = M5.Touch.getDetail();   // non-const: portable across M5Unified versions
    c2_input_touch(&g_in, t.isPressed(), (int16_t)t.x, (int16_t)t.y, millis());
}

static void poll_imu() {
    if (!M5.Imu.update()) return;       // abstracts MPU6886 and BMI270
    auto d = M5.Imu.getImuData();
    cp_imu_update(&g_imu, d.accel.x, d.accel.y, d.accel.z, d.gyro.z, millis());
    cp_turn_update(&g_turn, &g_imu);
}

// ── Drawing ───────────────────────────────────────────────────
static void chrome(const char *title) {
    g_cv.fillSprite(C_BG);
    g_cv.fillRect(0, 0, C2_SCREEN_W, C2_HEADER_H, C_DIM);
    g_cv.setFont(&fonts::Font2);
    g_cv.setTextColor(C_INK, C_DIM);
    g_cv.setCursor(4, 4);
    g_cv.print(title);

    char sum[32];
    mantis_caps_summary(&g_rx.deploy, sum, sizeof(sum));
    g_cv.setTextColor(C_MESH, C_DIM);
    g_cv.setCursor(C2_SCREEN_W - 150, 4);
    g_cv.print(sum);

    const int b = M5.Power.getBatteryLevel();
    g_cv.setTextColor(b < 20 ? C_ALERT : C_MID, C_DIM);
    g_cv.setCursor(C2_SCREEN_W - 34, 4);
    g_cv.printf("%3d%%", b);

    if (g_audio.muted) {
        g_cv.setTextColor(C_WARN, C_DIM);
        g_cv.setCursor(C2_SCREEN_W - 190, 4);
        g_cv.print("MUTE");
    }
}

// Footer labels sit OVER the three capacitive zones they describe.
// Centring them on the zone centres rather than spacing them evenly is
// the difference between a label that points at its dot and one that
// merely sits near it.
static void footer(const char *a, const char *b, const char *c) {
    const int y = C2_SCREEN_H - C2_FOOTER_H;
    g_cv.fillRect(0, y, C2_SCREEN_W, C2_FOOTER_H, C_DIM);
    g_cv.setFont(&fonts::Font2);
    g_cv.setTextColor(C_INK, C_DIM);
    const char *L[3] = { a, b, c };
    const int cx[3] = { C2_BTN_A_CX, C2_BTN_B_CX, C2_BTN_C_CX };
    for (int i = 0; i < 3; i++) {
        if (!L[i] || !*L[i]) continue;
        g_cv.setCursor(cx[i] - (int)strlen(L[i]) * 5, y + 6);
        g_cv.print(L[i]);
    }
}

static void flush() { if (g_canvas_ok) g_cv.pushSprite(0, 0); }

static void draw_map() {
    const int cx = C2_MAP_CX, cy = C2_MAP_CY, r = C2_MAP_R;
    g_cv.drawCircle(cx, cy, r, C_DIM);
    const MantisMesh &m = g_rx.mesh;
    if (m.extent <= 0) return;
    const float sc = (float)r / m.extent;

    // Mesh underglow first: backdrop, never over the tracks.
    float hi = 0;
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++) if (m.field[i] > hi) hi = m.field[i];
    if (hi > MANTIS_MESH_MIN_LLR * 0.5f) {
        const float step = (2.0f * m.extent) / (float)MANTIS_TOMO_DIM;
        for (int gy = 0; gy < MANTIS_TOMO_DIM; gy++)
            for (int gx = 0; gx < MANTIS_TOMO_DIM; gx++) {
                const float v = m.field[gy * MANTIS_TOMO_DIM + gx];
                if (v <= 0) continue;
                const float t = v / hi;
                if (t < 0.40f) continue;
                const int px = cx + (int)((-m.extent + (gx + 0.5f) * step) * sc);
                const int py = cy - (int)(( m.extent - (gy + 0.5f) * step) * sc);
                if ((px-cx)*(px-cx) + (py-cy)*(py-cy) > r*r) continue;
                g_cv.fillCircle(px, py, t > 0.75f ? 4 : 3, C_MESH);
            }
    }

    // Beacons.  A SURVEYED position is drawn solid; an assumed one
    // hollow, because presenting a guess with the same weight as a
    // measurement is the mistake this whole survey exists to correct.
    for (uint8_t i = 1; i <= m.n_beacons && i < MANTIS_SLOTS; i++) {
        if (!m.have_pos[i]) continue;
        const int px = cx + (int)(m.bx[i] * sc), py = cy - (int)(m.by[i] * sc);
        const bool sus = (g_rx.integrity.geometry_suspect && g_rx.integrity.worst_id == i);
        if (m.pos_measured[i]) g_cv.fillCircle(px, py, 4, sus ? C_ALERT : C_LIME);
        else                   g_cv.drawCircle(px, py, 4, sus ? C_ALERT : C_MID);
        g_cv.setFont(&fonts::Font0);
        g_cv.setTextColor(C_INK, C_BG);
        g_cv.setCursor(px - 3, py - 13);
        g_cv.printf("%d", i);
    }

    // Contacts, on top, from the FUSED view rather than raw shadowing --
    // the fused one is what survived cross-checking.
    for (int i = 0; i < MANTIS_FUSE_MAX; i++) {
        const MantisFused &t = g_rx.fusion.t[i];
        if (!t.active) continue;
        const int px = cx + (int)(t.x * sc), py = cy - (int)(t.y * sc);
        const bool believed = mantis_fuse_believed(&t);
        const uint16_t col = !believed ? C_VIOLET
                           : (t.cls == MFC_STATIC ? C_TEAL : C_LIME);
        if (believed) g_cv.fillCircle(px, py, 6, col);
        else          g_cv.drawCircle(px, py, 6, col);
        // A ring marks a STATIC contact -- present but not moving is the
        // case ordinary CSI radar cannot see, and it should be obvious.
        if (t.cls == MFC_STATIC) g_cv.drawCircle(px, py, 10, col);
    }

    if (g_rx.probe.known) {
        const int px = cx + (int)(g_rx.probe.x * sc);
        const int py = cy - (int)(g_rx.probe.y * sc);
        g_cv.drawLine(px-6, py, px+6, py, C_VIOLET);
        g_cv.drawLine(px, py-6, px, py+6, C_VIOLET);
    }
}

static void draw_status() {
    const int x = C2_SPLIT_X + 6;
    g_cv.setFont(&fonts::Font2);
    int row = 0;
    auto line = [&](uint16_t c, const char *fmt, ...) {
        char buf[40]; va_list ap; va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
        g_cv.setTextColor(c, C_BG);
        g_cv.setCursor(x, C2_CONTENT_Y + 4 + row * C2_ROW_H);
        g_cv.print(buf); row++;
    };
    line(C_MID,  "bcn  %d", g_rx.mesh.n_beacons);
    line(C_MID,  "link %d", g_rx.mesh.n_chords);

    int believed = 0;
    for (int i = 0; i < MANTIS_FUSE_MAX; i++)
        if (mantis_fuse_believed(&g_rx.fusion.t[i])) believed++;
    line(believed ? C_LIME : C_MID, "cont %d", believed);

    for (int i = 0; i < MANTIS_FUSE_MAX && row < C2_ROWS - 3; i++) {
        const MantisFused &t = g_rx.fusion.t[i];
        if (!t.active) continue;
        line(t.cls == MFC_STATIC ? C_TEAL : C_LIME, "%s %d/%d",
             mantis_fuse_class_name(t.cls), t.witnesses, g_rx.mesh.n_beacons);
    }

    // Geometry provenance, always visible.  An operator should never
    // have to wonder whether the map is measured or assumed.
    const float gc = mantis_mesh_geometry_confidence(&g_rx.mesh);
    g_cv.setTextColor(gc > 0.5f ? C_MID : C_WARN, C_BG);
    g_cv.setCursor(x, C2_CONTENT_Y + 4 + (C2_ROWS - 2) * C2_ROW_H);
    g_cv.print(gc > 0.5f ? "surveyed" : "assumed layout");

    if (g_rx.integrity.geometry_suspect) {
        g_cv.setTextColor(C_ALERT, C_BG);
        g_cv.setCursor(x, C2_CONTENT_Y + 4 + (C2_ROWS - 1) * C2_ROW_H);
        g_cv.printf("B%d MOVED", g_rx.integrity.worst_id);
    }
}

// ── Screens ───────────────────────────────────────────────────
static void s_splash() {
    chrome("MANTIS");
    g_cv.setFont(&fonts::Font4);
    g_cv.setTextColor(C_LIME, C_BG);
    g_cv.setCursor(96, 80); g_cv.print("MANTIS");
    g_cv.setFont(&fonts::Font2);
    g_cv.setTextColor(C_MID, C_BG);
    g_cv.setCursor(84, 118); g_cv.print("Core2 Probe");
    footer("", "", "");
    flush();
    if (age() > 1800) go(S_DISCOVERY);
}

static void s_discovery() {
    chrome("DISCOVERY");
    g_cv.setFont(&fonts::Font2);
    g_cv.setTextColor(C_INK, C_BG);
    g_cv.setCursor(8, C2_CONTENT_Y + 8);
    g_cv.printf("beacons heard: %d", g_rx.mesh.n_beacons);
    g_cv.setCursor(8, C2_CONTENT_Y + 8 + C2_ROW_H);
    g_cv.printf("links: %d", g_rx.caps.chords);

    g_cv.setTextColor(g_rx.caps.localize ? C_LIME : C_WARN, C_BG);
    g_cv.setCursor(8, C2_CONTENT_Y + 8 + 3 * C2_ROW_H);
    g_cv.print(g_rx.caps.localize ? "LOCATING" : "presence only");
    if (g_rx.caps.blocker[0]) {
        g_cv.setTextColor(C_MID, C_BG);
        g_cv.setCursor(8, C2_CONTENT_Y + 8 + 4 * C2_ROW_H);
        g_cv.print(g_rx.caps.blocker);
    }
    footer("", "", g_rx.mesh.n_beacons ? "GO" : "");
    flush();
    if (c2_was_short(&g_in, BTN_RIGHT) && g_rx.mesh.n_beacons) go(S_RADAR);
}

static void s_radar() {
    chrome("RADAR");
    draw_map();
    g_cv.drawLine(C2_SPLIT_X, C2_CONTENT_Y, C2_SPLIT_X,
                  C2_SCREEN_H - C2_FOOTER_H, C_DIM);
    draw_status();
    footer("MENU", g_audio.muted ? "UNMUTE" : "MUTE", "");
    flush();
    if (c2_was_short(&g_in, BTN_LEFT)) go(S_MENU);
    if (g_in.select) g_audio.muted = !g_audio.muted;
}

static const char *MENU[] = { "Alarms", "Mesh health", "Calibrate", "Radar" };
static const int   MENU_N = 4;

static void s_menu() {
    chrome("MENU");
    const bool avail[MENU_N] = { true, g_rx.caps.localize, g_rx.caps.tactical, true };
    // Touch selects a row directly; the zones still step through it, so
    // nothing here needs the screen.
    if (g_in.touched_row >= 0 && g_in.touched_row < MENU_N) g_sel = g_in.touched_row;
    if (g_in.gesture == C2T_SWIPE_DOWN) g_sel = (g_sel + 1) % MENU_N;
    if (g_in.gesture == C2T_SWIPE_UP)   g_sel = (g_sel + MENU_N - 1) % MENU_N;
    if (g_in.select)                    g_sel = (g_sel + 1) % MENU_N;

    g_cv.setFont(&fonts::Font2);
    for (int i = 0; i < MENU_N; i++) {
        const int y = C2_CONTENT_Y + 4 + i * C2_ROW_H;
        if (i == g_sel) g_cv.fillRect(0, y - 2, C2_SPLIT_X, C2_ROW_H, C_DIM);
        g_cv.setTextColor(!avail[i] ? C_DIM : (i == g_sel ? C_LIME : C_INK),
                          i == g_sel ? C_DIM : C_BG);
        g_cv.setCursor(10, y);
        g_cv.print(MENU[i]);
        if (!avail[i] && i == g_sel) {
            g_cv.setTextColor(C_WARN, C_DIM);
            g_cv.setCursor(C2_SPLIT_X + 6, y);
            g_cv.print(mantis_caps_why(&g_rx.deploy, &g_rx.caps,
                       i == 2 ? "tactical" : "localize"));
        }
    }
    footer("BACK", "NEXT", "SELECT");
    flush();
    if (c2_was_short(&g_in, BTN_LEFT)) go(S_RADAR);
    if (c2_was_short(&g_in, BTN_RIGHT) && avail[g_sel]) {
        switch (g_sel) {
            case 0: go(S_ALARMS); break;
            case 1: go(S_MESH);   break;
            case 2: go(S_CAL);    break;
            default: go(S_RADAR); break;
        }
    }
}

// The alarms submenu.  Each row is one alarm with THREE independent
// settings, because they fail in different situations: the sound, the
// haptic, and where the sound comes from.
static void s_alarms() {
    chrome("ALARMS");
    const int n = CPA_COUNT - 1;
    if (g_in.touched_row >= 0 && g_in.touched_row < n) g_sel = g_in.touched_row;
    if (g_in.gesture == C2T_SWIPE_DOWN) g_sel = (g_sel + 1) % n;
    if (g_in.gesture == C2T_SWIPE_UP)   g_sel = (g_sel + n - 1) % n;
    if (g_in.select)                    g_sel = (g_sel + 1) % n;

    g_cv.setFont(&fonts::Font2);
    for (int i = 0; i < n; i++) {
        const CardputerAlarm k = (CardputerAlarm)(i + 1);
        const int y = C2_CONTENT_Y + 4 + i * C2_ROW_H;
        if (i == g_sel) g_cv.fillRect(0, y - 2, C2_SCREEN_W, C2_ROW_H, C_DIM);
        const uint16_t bg = (i == g_sel) ? C_DIM : C_BG;
        g_cv.setTextColor(g_audio.enabled[k] ? C_INK : C_DIM, bg);
        g_cv.setCursor(10, y);
        g_cv.print(CP_ALARMS[k].label);
        g_cv.setTextColor(g_audio.enabled[k] ? C_LIME : C_DIM, bg);
        g_cv.setCursor(150, y);
        g_cv.print(g_audio.enabled[k] ? "SND" : "---");
        g_cv.setTextColor(g_vibe[k] ? C_TEAL : C_DIM, bg);
        g_cv.setCursor(196, y);
        g_cv.print(g_vibe[k] ? "VIB" : "---");
        g_cv.setTextColor(C_MID, bg);
        g_cv.setCursor(244, y);
        g_cv.print(cp_audio_source(&g_audio, k));
    }
    footer("BACK", "NEXT", "TOGGLE");
    flush();

    const CardputerAlarm sel = (CardputerAlarm)(g_sel + 1);
    if (c2_was_short(&g_in, BTN_LEFT)) go(S_MENU);
    // RIGHT cycles the row through off -> sound -> sound+vibe -> vibe,
    // which is one control for two settings and avoids a second level of
    // menu for something an operator changes in the field.
    if (c2_was_short(&g_in, BTN_RIGHT)) {
        const bool s = g_audio.enabled[sel], v = g_vibe[sel];
        if (!s && !v)      { g_audio.enabled[sel] = true;  g_vibe[sel] = false; }
        else if (s && !v)  { g_vibe[sel] = true; }
        else if (s && v)   { g_audio.enabled[sel] = false; }
        else               { g_vibe[sel] = false; }
        fire_alarm(sel);   // preview it, so the choice is audible
    }
    if (g_in.select_long) g_vibe_master = !g_vibe_master;
}

static void s_mesh() {
    chrome("MESH");
    g_cv.setFont(&fonts::Font2);
    for (uint8_t i = 1; i <= g_rx.mesh.n_beacons && i < 7; i++) {
        const bool sus = (g_rx.integrity.geometry_suspect && g_rx.integrity.worst_id == i);
        g_cv.setTextColor(sus ? C_ALERT : (g_rx.mesh.pos_measured[i] ? C_LIME : C_MID), C_BG);
        g_cv.setCursor(10, C2_CONTENT_Y + 4 + (i - 1) * C2_ROW_H);
        g_cv.printf("B%d %-9s (%+.2f,%+.2f)%s", i,
                    g_rx.mesh.pos_measured[i] ? "surveyed" : "assumed",
                    g_rx.mesh.bx[i], g_rx.mesh.by[i], sus ? "  MOVED" : "");
    }
    g_cv.setTextColor(C_MID, C_BG);
    g_cv.setCursor(10, C2_SCREEN_H - C2_FOOTER_H - C2_ROW_H);
    g_cv.printf("geom adopted %lu  rebuilds %lu",
                (unsigned long)g_rx.geom_in, (unsigned long)g_rx.geom_rebuilds);
    footer("BACK", "", "");
    flush();
    if (c2_was_short(&g_in, BTN_LEFT)) go(S_MENU);
}

static void s_cal() {
    chrome("CAL WALK");
    // Mirror the step to the anchor so it renders what the operator is
    // actually doing.  A hint, so it is re-sent rather than retried.
    static uint32_t s_hint_ms = 0;
    if (millis() - s_hint_ms > 500) {
        s_hint_ms = millis();
        mantis_probe_send(&g_link, MPC_CAL_STEP_HINT, (uint8_t)g_sel, 0, 0,
                          millis(), true);
    }
    const int cx = C2_MAP_CX, cy = C2_MAP_CY, r = C2_MAP_R;
    g_cv.drawCircle(cx, cy, r, C_DIM);
    const float h = g_imu.heading_rad;
    g_cv.drawLine(cx, cy, cx + (int)(sinf(h) * r * 0.85f),
                          cy - (int)(cosf(h) * r * 0.85f), C_ALERT);
    g_cv.setFont(&fonts::Font2);
    const int x = C2_SPLIT_X + 6;
    g_cv.setTextColor(C_INK, C_BG);
    g_cv.setCursor(x, C2_CONTENT_Y + 8);
    g_cv.printf("turn %5.0f", cp_turn_degrees(&g_turn));
    g_cv.setCursor(x, C2_CONTENT_Y + 8 + C2_ROW_H);
    g_cv.printf("step %5lu", (unsigned long)g_imu.steps);
    g_cv.setCursor(x, C2_CONTENT_Y + 8 + 2 * C2_ROW_H);
    g_cv.printf("dist %5.1fm", cp_turn_distance_m(&g_turn, &g_imu));
    g_cv.setTextColor(g_imu.bias_ready ? C_MID : C_WARN, C_BG);
    g_cv.setCursor(x, C2_CONTENT_Y + 8 + 4 * C2_ROW_H);
    g_cv.print(g_imu.bias_ready ? "imu ready" : "imu warming");
    // Link state, always visible.  A silent failure here wastes the
    // whole walk and the operator finds out when the model comes out
    // wrong -- which is far too late to do anything about.
    g_cv.setTextColor(mantis_probe_anchor_live(&g_link, millis()) ? C_MID : C_WARN, C_BG);
    g_cv.setCursor(x, C2_CONTENT_Y + 8 + 5 * C2_ROW_H);
    g_cv.print(mantis_probe_link_state(&g_link, millis()));

    footer("BACK", "", "MARK");
    flush();
    if (c2_was_short(&g_in, BTN_LEFT)) {
        // Leaving the walk closes the anchor's capture window.  Without
        // this it keeps recording an empty room into the training set.
        mantis_probe_send(&g_link, MPC_CAL_END, 0, 0, 0, millis(), true);
        go(S_MENU);
    }
    if (c2_was_short(&g_in, BTN_RIGHT)) {
        cp_turn_begin(&g_turn, &g_imu);
        // The operator is holding this device, so THIS device advances
        // the script.  Walking back to the anchor to press a button
        // would put body motion into the walk that is not part of it.
        mantis_probe_send(&g_link, MPC_CAL_BEGIN, 0, 0, 0, millis(), true);
    }
}

// ── Arduino ───────────────────────────────────────────────────
void setup() {
    auto cfg = M5.config();
    cfg.internal_imu = true;
    cfg.internal_spk = true;
    M5.begin(cfg);
    M5.Display.setRotation(1);          // 320x240 landscape

    g_canvas_ok = g_cv.createSprite(C2_SCREEN_W, C2_SCREEN_H);

    c2_input_begin(&g_in);
    cp_imu_begin(&g_imu, C2_STRIDE_M_DEFAULT);
    cp_turn_begin(&g_turn, &g_imu);
    cp_audio_begin(&g_audio);
    for (int i = 1; i < CPA_COUNT; i++) g_vibe[i] = true;

    // This device is a PROBE and never an anchor, so it declares itself
    // as one receiver with no T-Display until it hears otherwise.
    mantis_rx_begin(&g_rx, 1.2f, /*tdisplays*/0, /*cardputers*/1, /*docked*/false);
    mantis_probe_link_begin(&g_link);
    mantis_probe_radio_begin();     // without this the UI renders nothing real

    // M5Unified brings up the SD bus on the Core2, so the plain form
    // works here -- unlike the Cardputer, which needs explicit pins.
    g_audio.sd_present = SD.begin(C2_SD_CS_PIN);
    if (g_audio.sd_present) {
        SD.mkdir("/mantis");
        SD.mkdir(C2_ALARM_DIR);
        cp_mp3_begin();
        for (int i = 1; i < CPA_COUNT; i++) {
            char p[96];
            snprintf(p, sizeof(p), "%s/%s", C2_ALARM_DIR, CP_ALARMS[i].file);
            g_audio.file_ok[i] = SD.exists(p);
        }
    }
    g_entered = millis();
}

void loop() {
    M5.update();
    poll_input();
    poll_imu();
    vibe_tick();

    // Drain the radio EVERY loop, not on the solve interval: reports
    // arrive at up to 180 Hz across six beacons and the queue is eight
    // deep, so anything slower loses them.
    mantis_probe_pump(&g_rx, millis());
    mantis_probe_retry_tick(&g_link, millis());

    static uint32_t last = 0;
    if (millis() - last >= 100) {
        last = millis();
        mantis_rx_solve(&g_rx, millis(), 0.1f);
    }

    switch (g_screen) {
        case S_SPLASH:    s_splash();    break;
        case S_DISCOVERY: s_discovery(); break;
        case S_RADAR:     s_radar();     break;
        case S_MENU:      s_menu();      break;
        case S_ALARMS:    s_alarms();    break;
        case S_MESH:      s_mesh();      break;
        case S_CAL:       s_cal();       break;
        default:
            // Never sit on an unhandled screen: a frozen display with
            // live controls is indistinguishable from a crash.
            go(S_RADAR);
            break;
    }
}
