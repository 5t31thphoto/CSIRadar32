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
#include "mantis_probe_app.h"  // shared: alarm policy, map, anchor remote

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
    S_SPLASH = 0, S_DISCOVERY, S_RADAR, S_MENU, S_ALARMS, S_MESH, S_ANCHOR,
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
// The motor has no auto-off.  Every path that starts it sets the stop
// time -- the previous version started it and never stopped it, so one
// alert buzzed until the next one.
static uint32_t g_vibe_until = 0;
static void vibe_start(uint32_t ms) {
    M5.Power.setVibration(180);
    g_vibe_until = millis() + ms;
}
static void vibe_tick() {
    if (g_vibe_until && (int32_t)(millis() - g_vibe_until) >= 0) {
        M5.Power.setVibration(0);
        g_vibe_until = 0;
    }
}

// Tone fallback, NON-BLOCKING.  The old loop called delay() between
// repeats, freezing the radio pump for up to a second on every alarm --
// exactly when reports matter most.
static uint16_t g_tone_hz = 0, g_tone_ms = 0;
static uint8_t  g_tone_left = 0;
static uint32_t g_tone_next = 0;
static void tone_tick() {
    if (!g_tone_left || (int32_t)(millis() - g_tone_next) < 0) return;
    M5.Speaker.tone(g_tone_hz, g_tone_ms);
    g_tone_left--;
    g_tone_next = millis() + g_tone_ms + 60;
}

static uint32_t g_alarm_flash_until = 0;
static CardputerAlarm g_last_alarm = CPA_NONE;

static void fire_alarm(CardputerAlarm k) {
    if (k <= CPA_NONE || k >= CPA_COUNT) return;
    const uint32_t now = millis();
    g_last_alarm = k;
    g_alarm_flash_until = now + 1500;
    const bool vib = g_vibe_master && g_vibe[k];
    if (cp_audio_should_fire(&g_audio, k, now) != CPF_FIRE) {
        // Sound suppressed (muted, disabled, debounced): haptics are a
        // separate channel and a muted speaker must not disable them.
        if (vib && g_audio.muted) vibe_start(350);
        return;
    }
    cp_audio_mark_fired(&g_audio, k, now);
    if (vib) vibe_start(350);
    const CardputerAlarmDef &d = CP_ALARMS[k];
    if (g_audio.sd_present && g_audio.file_ok[k] && !cp_mp3_busy()) {
        char path[96];
        snprintf(path, sizeof(path), "%s/%s", C2_ALARM_DIR, d.file);
        if (cp_mp3_play(path, g_audio.volume)) return;
    }
    M5.Speaker.setVolume(g_audio.volume);
    g_tone_hz = d.tone_hz; g_tone_ms = d.tone_ms;
    g_tone_left = d.repeats; g_tone_next = now;
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

static void flush() {
    // No canvas means the frame cannot be presented.  Reported once at
    // boot on the panel itself; silently skipping here is correct at
    // this point BECAUSE the operator was already told.
    if (g_canvas_ok) g_cv.pushSprite(0, 0);
}

static const MantisProbePalette PAL = {
    C_BG, C_INK, C_MID, C_DIM, C_LIME, C_TEAL, C_VIOLET, C_ALERT, C_WARN, C_MESH };

static void draw_map() {
    g_cv.setFont(&fonts::Font0);
    mantis_probe_draw_map(g_cv, g_rx, g_link, millis(),
                          C2_MAP_CX, C2_MAP_CY, C2_MAP_R, PAL);
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
    line(C_MID,  "bcn  %d", g_rx.deploy.beacons);
    line(C_MID,  "link %d", g_rx.mesh.n_chords);

    int believed = 0;
    for (int i = 0; i < MANTIS_FUSE_MAX; i++)
        if (mantis_fuse_believed(&g_rx.fusion.t[i])) believed++;
    line(believed ? C_LIME : C_MID, "cont %d", believed);

    for (int i = 0; i < MANTIS_FUSE_MAX && row < C2_ROWS - 3; i++) {
        const MantisFused &t = g_rx.fusion.t[i];
        if (!t.active) continue;
        line(t.cls == MFC_STATIC ? C_TEAL : C_LIME, "%s %d/%d",
             mantis_fuse_class_name(t.cls), t.witnesses, g_rx.deploy.beacons);
    }
    // The anchor, always one line: what it is doing, and whether it sees
    // anyone.  "no anchor" is information too.
    const uint32_t now = millis();
    if (mantis_probe_anchor_live(&g_link, now))
        line((g_link.st.flags & MAS_ALERT) ? C_ALERT : C_TEAL, "A:%s",
             mantis_anchor_state_name(g_link.st.app_state));
    else
        line(C_DIM, "A:%s", mantis_probe_link_state(&g_link, now));

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
    g_cv.printf("beacons heard: %d", g_rx.deploy.beacons);
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
    const bool anchor = mantis_probe_anchor_live(&g_link, millis());
    g_cv.setTextColor(anchor ? C_TEAL : C_DIM, C_BG);
    g_cv.setCursor(8, C2_CONTENT_Y + 8 + 6 * C2_ROW_H);
    g_cv.print(anchor ? "anchor: linked" : "anchor: not heard");
    const bool ready = g_rx.deploy.beacons || anchor;
    footer("", "", ready ? "GO" : "");
    flush();
    // Automatically onward once there is anything to show: the operator
    // should not have to press GO to be told someone is in the room.
    if ((c2_was_short(&g_in, BTN_RIGHT) || age() > 6000) && ready) go(S_RADAR);
}

static void s_radar() {
    chrome("RADAR");
    draw_map();
    g_cv.drawLine(C2_SPLIT_X, C2_CONTENT_Y, C2_SPLIT_X,
                  C2_SCREEN_H - C2_FOOTER_H, C_DIM);
    draw_status();
    if ((int32_t)(millis() - g_alarm_flash_until) < 0) {
        g_cv.drawRect(0, C2_CONTENT_Y, C2_SCREEN_W, C2_CONTENT_H, C_ALERT);
        g_cv.drawRect(1, C2_CONTENT_Y + 1, C2_SCREEN_W - 2, C2_CONTENT_H - 2, C_ALERT);
        g_cv.setTextColor(C_ALERT, C_BG);
        g_cv.setCursor(6, C2_CONTENT_Y + 4);
        g_cv.print(CP_ALARMS[g_last_alarm].label);
    }
    const bool anchor = g_link.anchor_seen;
    footer("MENU", g_audio.muted ? "UNMUTE" : "MUTE", anchor ? "ANCHOR" : "");
    flush();
    if (c2_was_short(&g_in, BTN_LEFT)) go(S_MENU);
    if (c2_was_short(&g_in, BTN_RIGHT) && anchor) go(S_ANCHOR);
    if (g_in.select) g_audio.muted = !g_audio.muted;
}

static const char *MENU[] = { "Alarms", "Mesh health", "Anchor remote", "Radar" };
static const int   MENU_N = 4;

static void s_menu() {
    chrome("MENU");
    const bool avail[MENU_N] = { true, true, g_link.anchor_seen, true };
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
            g_cv.print(i == 2 ? "no anchor heard" : "");
        }
    }
    footer("BACK", "NEXT", "SELECT");
    flush();
    if (c2_was_short(&g_in, BTN_LEFT)) go(S_RADAR);
    if (c2_was_short(&g_in, BTN_RIGHT) && avail[g_sel]) {
        switch (g_sel) {
            case 0: go(S_ALARMS); break;
            case 1: go(S_MESH);   break;
            case 2: go(S_ANCHOR); break;
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
        if (!g_rx.mesh.live[i]) continue;
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

// ── THE ANCHOR, IN YOUR HAND ──────────────────────────────────
// A mirror of the anchor's own screen and a remote for its two buttons.
// Nothing here is inferred: the state, the instruction and the button
// labels are what the anchor broadcast about the frame it just DREW, and
// a press is shown as landed only when the anchor reports having applied
// it.  The probe cannot drift from the anchor because it keeps no copy of
// the anchor's state machine to drift.
static void s_anchor() {
    const uint32_t now = millis();
    const bool live = mantis_probe_anchor_live(&g_link, now);
    const MantisAnchorStatus &st = g_link.st;
    chrome("ANCHOR");
    g_cv.setFont(&fonts::Font4);
    g_cv.setTextColor(!live ? C_WARN : ((st.flags & MAS_ALERT) ? C_ALERT : C_LIME), C_BG);
    g_cv.setCursor(8, C2_CONTENT_Y + 6);
    g_cv.print(live ? mantis_anchor_state_name(st.app_state) : "NO ANCHOR");

    g_cv.setFont(&fonts::Font2);
    g_cv.setTextColor(g_link.key_tries ? C_WARN : C_MID, C_BG);
    const char *ls = mantis_probe_link_state(&g_link, now);
    g_cv.setCursor(C2_SCREEN_W - 8 - g_cv.textWidth(ls), C2_CONTENT_Y + 10);
    g_cv.print(ls);

    int y = C2_CONTENT_Y + 38;
    if (live) {
        if (st.title[0]) {
            g_cv.setTextColor(C_MID, C_BG);
            g_cv.setCursor(8, y); g_cv.printf("screen: %s", st.title); y += C2_ROW_H;
        }
        char lines[5][48];
        const int n = mantis_wrap(st.hint, 36, lines, 5);
        g_cv.setTextColor(C_INK, C_BG);
        for (int i = 0; i < n; i++) { g_cv.setCursor(8, y); g_cv.print(lines[i]); y += C2_ROW_H; }
        if (st.steps) {
            g_cv.setTextColor(C_TEAL, C_BG);
            g_cv.setCursor(8, y); g_cv.printf("step %u of %u", st.step, st.steps); y += C2_ROW_H;
        }
        if (st.progress) {
            g_cv.drawRect(8, y + 2, C2_SCREEN_W - 16, 10, C_DIM);
            g_cv.fillRect(9, y + 3, (C2_SCREEN_W - 18) * st.progress / 100, 8, C_LIME);
            y += C2_ROW_H;
        }
        g_cv.setTextColor(C_MID, C_BG);
        g_cv.setCursor(8, C2_SCREEN_H - C2_FOOTER_H - C2_ROW_H);
        g_cv.printf("bcn %u  mesh %u  tracks %u  mesh contacts %u",
                    st.beacons, st.mesh_beacons, st.n_tracks, st.mesh_contacts);
    } else {
        g_cv.setTextColor(C_MID, C_BG);
        g_cv.setCursor(8, y);
        g_cv.print(g_link.anchor_seen ? "Anchor went quiet - is it on?" : "Power the T-Display anchor on.");
        g_cv.setCursor(8, y + C2_ROW_H);
        g_cv.print("Hold a zone: long press.");
    }

    // Footer: the ANCHOR's labels on A and C, ours on B.
    char la[16] = "", lc[16] = "";
    if (live && st.left[0])  snprintf(la, sizeof(la), "<%s", st.left);
    if (live && st.right[0]) snprintf(lc, sizeof(lc), "%s>", st.right);
    footer(la, "BACK", lc);
    flush();

    if (g_in.select) { go(S_RADAR); return; }
    if (!live) return;
    if (c2_was_short(&g_in, BTN_LEFT))  mantis_probe_press(&g_link, MRK_LEFT, now);
    if (c2_was_short(&g_in, BTN_RIGHT)) mantis_probe_press(&g_link, MRK_RIGHT, now);
    if (c2_was_long(&g_in, BTN_LEFT))   mantis_probe_press(&g_link, MRK_LEFT_LONG, now);
    if (c2_was_long(&g_in, BTN_RIGHT))  mantis_probe_press(&g_link, MRK_RIGHT_LONG, now);
}

// ── Arduino ───────────────────────────────────────────────────
void setup() {
    auto cfg = M5.config();
    cfg.internal_imu = true;
    cfg.internal_spk = true;
    M5.begin(cfg);
    M5.Display.setRotation(1);          // 320x240 landscape

    // ── CANVAS, WITH A REAL FALLBACK ──────────────────────────
    //
    // 320x240 at 16bpp is 153,600 bytes.  The Core2 has PSRAM and
    // M5Canvas will normally use it, but if that allocation fails the
    // previous code simply set g_canvas_ok = false and every flush()
    // became a no-op -- a device that boots, runs, responds to touch
    // and shows a BLANK SCREEN, with nothing anywhere saying why.
    //
    // Half the depth before giving up: 8bpp is 76,800 bytes and the UI
    // uses a dozen flat colours, so the loss is invisible.
    g_canvas_ok = g_cv.createSprite(C2_SCREEN_W, C2_SCREEN_H);
    if (!g_canvas_ok) {
        g_cv.setColorDepth(8);
        g_canvas_ok = g_cv.createSprite(C2_SCREEN_W, C2_SCREEN_H);
    }
    if (!g_canvas_ok) {
        // Still nothing.  Say so ON THE PANEL rather than presenting a
        // black screen: the device is otherwise working and the
        // operator needs to know it is a memory problem, not a dead
        // unit.
        M5.Display.fillScreen(C_BG);
        M5.Display.setTextColor(C_ALERT, C_BG);
        M5.Display.setCursor(8, 100);
        M5.Display.print("display buffer alloc failed");
        M5.Display.setCursor(8, 120);
        M5.Display.print("sensing continues; UI degraded");
    }

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
    tone_tick();

    // Drain the radio EVERY loop, not on the solve interval: reports
    // arrive at up to 180 Hz across six beacons and the queue is eight
    // deep, so anything slower loses them.
    mantis_probe_pump(&g_rx, millis());
    mantis_probe_service(&g_link, MANTIS_PROBE_KIND_CORE2, millis());

    static uint32_t last = 0;
    static MantisAlarmWatch watch = {};
    if (millis() - last >= 100) {
        last = millis();
        mantis_rx_solve(&g_rx, millis(), 0.1f);
        // The anchor counts as a display in the capability picture.
        g_rx.deploy.tdisplays = mantis_probe_anchor_live(&g_link, millis()) ? 1 : 0;
        g_rx.caps = mantis_caps(&g_rx.deploy);
        const CardputerAlarm k = mantis_alarm_eval(&watch, &g_rx, &g_link, millis());
        if (k != CPA_NONE) fire_alarm(k);
    }

    switch (g_screen) {
        case S_SPLASH:    s_splash();    break;
        case S_DISCOVERY: s_discovery(); break;
        case S_RADAR:     s_radar();     break;
        case S_MENU:      s_menu();      break;
        case S_ALARMS:    s_alarms();    break;
        case S_MESH:      s_mesh();      break;
        case S_ANCHOR:    s_anchor();    break;
        default:
            // Never sit on an unhandled screen: a frozen display with
            // live controls is indistinguishable from a crash.
            go(S_RADAR);
            break;
    }
}
