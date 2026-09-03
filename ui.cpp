// ═══════════════════════════════════════════════════════════════
//  CSI-Radar-S3 — ui.cpp
// ═══════════════════════════════════════════════════════════════
#include "ui.h"
#include "lgfx_tdisplay_s3.h"
#include "wizard.h"
#include "scene.h"
#include <math.h>

// ── Display globals ─────────────────────────────────────────────
static LGFX_TDisplayS3 g_lcd;
static LGFX_Sprite     g_canvas(&g_lcd);
static bool            g_canvas_ok = false;

// Palette — dark, high-contrast, radar-y
// ── Colors: legacy names bound to the MantisSec palette ────────
// v0.3 unifies the visual identity under the MantisSec palette in
// config.h.  These aliases keep the 200+ existing color references
// throughout ui.cpp working while making every screen adopt the new
// look automatically.  Prefer COL_MS_* directly in new code.
static constexpr uint16_t COL_BG        = COL_MS_BG;
static constexpr uint16_t COL_GRID_DARK = COL_MS_DIM;
static constexpr uint16_t COL_GRID      = COL_MS_MID;
static constexpr uint16_t COL_FG        = COL_MS_LIME;           // "signal on"
static constexpr uint16_t COL_DIM       = COL_MS_LIME_DIM;       // dim-lime
static constexpr uint16_t COL_ACCENT    = COL_MS_TEAL_BRIGHT;    // stereo / peer accent
static constexpr uint16_t COL_WARN      = COL_MS_WARN;
static constexpr uint16_t COL_ALERT     = COL_MS_ALERT;
static constexpr uint16_t COL_MUTED     = COL_MS_MID;
static constexpr uint16_t COL_TEXT      = COL_MS_INK;

// ── Helpers ─────────────────────────────────────────────────────
static LovyanGFX &gfx() {
    return g_canvas_ok ? (LovyanGFX&)g_canvas : (LovyanGFX&)g_lcd;
}

// Public accessor for render.cpp — draws into the same sprite/lcd.
LovyanGFX &gfx_sprite() { return gfx(); }

static void flush() {
    if (g_canvas_ok) g_canvas.pushSprite(0, 0);
}

static void clear() {
    if (g_canvas_ok) g_canvas.fillSprite(COL_BG);
    else             g_lcd.fillScreen(COL_BG);
}

static void draw_header(const char *title) {
    auto &g = gfx();
    g.fillRect(0, 0, SCREEN_W, HEADER_H, COL_MS_CHROME);
    // Two-pixel MantisSec accent line: violet on top of thin teal
    g.drawFastHLine(0, HEADER_H,     SCREEN_W, COL_MS_CHROME_LINE);
    g.drawFastHLine(0, HEADER_H + 1, SCREEN_W, COL_MS_TEAL);
    g.setTextColor(COL_MS_INK, COL_MS_CHROME);
    g.setTextSize(1);
    g.setFont(&fonts::Font2);
    g.setCursor(4, 4);
    g.print(title);

    // Right side: beacon count icons
    int x = SCREEN_W - 6;
    for (int i = MAX_BEACONS - 1; i >= 0; i--) {
        if (!g_app.beacon[i].active) continue;
        uint16_t c = COL_DIM;
        if (g_app.beacon[i].status == LS_MOTION)   c = COL_WARN;
        if (g_app.beacon[i].status == LS_PRESENCE) c = COL_ALERT;
        else if (g_app.beacon[i].baseline_valid)   c = COL_FG;
        g.fillCircle(x - 4, HEADER_H / 2, 3, c);
        g.drawCircle(x - 4, HEADER_H / 2, 3, COL_TEXT);
        x -= 10;
    }
}

static void draw_footer(const char *left_label, const char *right_label) {
    auto &g = gfx();
    int y0 = SCREEN_H - FOOTER_H;
    g.fillRect(0, y0, SCREEN_W, FOOTER_H, COL_MS_CHROME);
    g.drawFastHLine(0, y0,     SCREEN_W, COL_MS_TEAL);
    g.drawFastHLine(0, y0 + 1, SCREEN_W, COL_MS_CHROME_LINE);
    g.setTextColor(COL_MS_INK, COL_MS_CHROME);
    g.setFont(&fonts::Font2);
    g.setCursor(4, y0 + 4);
    if (left_label && *left_label) {
        g.print("< ");
        g.print(left_label);
    }
    if (right_label && *right_label) {
        int tw = g.textWidth(right_label) + g.textWidth("> ");
        g.setCursor(SCREEN_W - tw - 4, y0 + 4);
        g.print(right_label);
        g.print(" >");
    }
}

static void progress_bar(int x, int y, int w, int h, float p, uint16_t fg, uint16_t bg = COL_GRID_DARK) {
    auto &g = gfx();
    g.drawRect(x, y, w, h, COL_DIM);
    g.fillRect(x + 1, y + 1, w - 2, h - 2, bg);
    int fill = (int)((w - 2) * p);
    if (fill < 0) fill = 0;
    if (fill > w - 2) fill = w - 2;
    g.fillRect(x + 1, y + 1, fill, h - 2, fg);
}

// ── begin ───────────────────────────────────────────────────────
void ui_begin() {
    // Power on the LCD rail
    pinMode(PIN_LCD_POWER_ON, OUTPUT);
    digitalWrite(PIN_LCD_POWER_ON, HIGH);
    delay(50);

    g_lcd.init();
    g_lcd.setRotation(0);           // portrait, ribbon at bottom = buttons at bottom
    g_lcd.setBrightness(200);
    g_lcd.fillScreen(COL_BG);

    // Full-screen off-screen sprite for flicker-free rendering
    g_canvas.setColorDepth(16);
    g_canvas_ok = g_canvas.createSprite(SCREEN_W, SCREEN_H);
    if (!g_canvas_ok) {
        // Insufficient memory — fall back to direct-draw. Not ideal, but works.
        g_lcd.setCursor(4, 4);
        g_lcd.setTextColor(COL_WARN);
        g_lcd.print("[ui] no sprite RAM");
    }
}

// ── Splash ─────────────────────────────────────────────────────
// MantisSec animated splash:
//   1. Dark BG with a subtle violet grid backdrop
//   2. Stylized mantis silhouette (line-art via primitive geometry)
//      draws in from the head down, one segment per animation phase
//   3. "MANTISSEC" wordmark reveals character-by-character with a
//      scan-line sweep highlighting each letter as it lands
//   4. Subtitle "CSI SITUATIONAL AWARENESS" in dim teal
//   5. Version string + status text at the bottom
//   6. A single pulsing "eye" dot on the mantis
//
// Total reveal takes ~2.5s.  After that the whole splash pulses
// gently until the user advances or timeout expires.
static void draw_mantis_silhouette(int cx, int cy, uint16_t col,
                                   uint16_t glow_col, float reveal /*0..1*/) {
    auto &g = gfx();
    // Line art built from primitives.  Coordinates are relative to
    // (cx, cy) which is the head-center.  Reveal draws in this order:
    //   0.00-0.25  head + antennae + eyes
    //   0.25-0.55  thorax + first pair of legs (raptorial arms)
    //   0.55-0.85  abdomen + second/third leg pairs
    //   0.85-1.00  glow accents (a couple of highlight strokes)
    auto R = [reveal](float t){ return reveal >= t; };
    auto part = [reveal](float from, float to)->float{
        if (reveal < from) return 0.0f;
        if (reveal >= to)  return 1.0f;
        return (reveal - from) / (to - from);
    };

    // Head (triangular)
    if (R(0.00f)) {
        int hy = cy;
        g.drawLine(cx - 8, hy, cx + 8, hy, col);
        g.drawLine(cx - 8, hy, cx,    hy + 10, col);
        g.drawLine(cx + 8, hy, cx,    hy + 10, col);
        // Eyes
        g.fillCircle(cx - 5, hy + 3, 2, glow_col);
        g.fillCircle(cx + 5, hy + 3, 2, glow_col);
    }
    // Antennae — sweep out as reveal grows
    if (R(0.10f)) {
        float t = part(0.10f, 0.28f);
        int ex = (int)(14 * t);
        int ey = (int)(10 * t);
        g.drawLine(cx - 8, cy, cx - 8 - ex, cy - ey, col);
        g.drawLine(cx + 8, cy, cx + 8 + ex, cy - ey, col);
    }
    // Thorax — a narrow angled body
    if (R(0.28f)) {
        float t = part(0.28f, 0.55f);
        int by = cy + 10 + (int)(24 * t);
        g.drawLine(cx - 3, cy + 10, cx - 3, by, col);
        g.drawLine(cx + 3, cy + 10, cx + 3, by, col);
        g.drawLine(cx - 3, by, cx + 3, by, col);
    }
    // Raptorial arms (the mantis's iconic folded blades)
    if (R(0.30f)) {
        float t = part(0.30f, 0.55f);
        int reach = (int)(18 * t);
        // Right arm: upper segment out and slightly up, then folded down
        g.drawLine(cx + 3, cy + 14, cx + 3 + reach, cy + 8, col);
        if (t > 0.5f) g.drawLine(cx + 3 + reach, cy + 8,
                                  cx + 3 + reach - 4, cy + 20, col);
        // Left arm: mirrored
        g.drawLine(cx - 3, cy + 14, cx - 3 - reach, cy + 8, col);
        if (t > 0.5f) g.drawLine(cx - 3 - reach, cy + 8,
                                  cx - 3 - reach + 4, cy + 20, col);
    }
    // Abdomen — tapered tail
    if (R(0.55f)) {
        float t = part(0.55f, 0.85f);
        int ay = cy + 34;
        int aend = ay + (int)(22 * t);
        g.drawLine(cx - 3, ay, cx - 1, aend, col);
        g.drawLine(cx + 3, ay, cx + 1, aend, col);
    }
    // Middle legs
    if (R(0.60f)) {
        float t = part(0.60f, 0.85f);
        int reach = (int)(14 * t);
        g.drawLine(cx - 3, cy + 22, cx - 3 - reach, cy + 22 + reach, col);
        g.drawLine(cx + 3, cy + 22, cx + 3 + reach, cy + 22 + reach, col);
    }
    // Rear legs
    if (R(0.65f)) {
        float t = part(0.65f, 0.85f);
        int reach = (int)(16 * t);
        g.drawLine(cx - 3, cy + 30, cx - 3 - reach, cy + 30 + reach + 4, col);
        g.drawLine(cx + 3, cy + 30, cx + 3 + reach, cy + 30 + reach + 4, col);
    }
    // Glow accents — a highlight running down the thorax
    if (R(0.85f)) {
        g.drawFastVLine(cx, cy + 12, 22, glow_col);
    }
}

void ui_splash() {
    clear();
    auto &g = gfx();

    uint32_t elapsed = millis() - g_app.state_enter_ms;
    // Reveal phases
    float reveal_mantis = elapsed / 1400.0f;       if (reveal_mantis > 1) reveal_mantis = 1;
    float reveal_word   = (elapsed - 800) / 1200.0f;
    if (reveal_word < 0) reveal_word = 0;
    if (reveal_word > 1) reveal_word = 1;
    float reveal_sub    = (elapsed - 1800) / 600.0f;
    if (reveal_sub < 0) reveal_sub = 0;
    if (reveal_sub > 1) reveal_sub = 1;
    bool  fully_revealed = elapsed > 2600;

    // Backdrop grid — subtle violet 8px grid
    for (int x = 0; x < SCREEN_W; x += 8) g.drawFastVLine(x, 0, SCREEN_H, COL_MS_DIM);
    for (int y = 0; y < SCREEN_H; y += 8) g.drawFastHLine(0, y, SCREEN_W, COL_MS_DIM);

    // Thin teal frame
    g.drawRect(2, 2, SCREEN_W - 4, SCREEN_H - 4, COL_MS_TEAL);
    g.drawRect(3, 3, SCREEN_W - 6, SCREEN_H - 6, COL_MS_VIOLET);

    // Mantis silhouette — draws in over reveal_mantis
    int mx = SCREEN_W / 2;
    int my = 44;
    // Pulsing eye brightness
    uint16_t eye_col;
    if (fully_revealed) {
        uint32_t phase = (elapsed / 40) % 40;
        eye_col = phase < 20 ? COL_MS_LIME : COL_MS_LIME_DIM;
    } else {
        eye_col = COL_MS_LIME;
    }
    draw_mantis_silhouette(mx, my, COL_MS_TEAL_BRIGHT, eye_col, reveal_mantis);

    // MANTISSEC wordmark — character-by-character reveal with scan sweep
    g.setFont(&fonts::Font4);
    const char *word = "MANTISSEC";
    int total_w = g.textWidth(word);
    int start_x = (SCREEN_W - total_w) / 2;
    int word_y = 156;
    int chars = (int)(reveal_word * 9.0f + 0.5f);
    if (chars > 9) chars = 9;
    // Draw revealed characters
    for (int i = 0; i < chars; i++) {
        char c[2] = {word[i], 0};
        // Compute this char's x position by measuring the prefix
        char prefix[10]; strncpy(prefix, word, i); prefix[i] = 0;
        int px = start_x + g.textWidth(prefix);
        // Latest char draws in lime, older chars settle to violet-bright
        uint16_t cc = (i == chars - 1 && chars < 9) ? COL_MS_LIME : COL_MS_VIOLET_BRIGHT;
        g.setTextColor(cc, COL_MS_BG);
        g.setCursor(px, word_y);
        g.print(c);
    }
    // Once fully spelled, wordmark stays violet-bright with a lime underline
    if (chars == 9) {
        g.setTextColor(COL_MS_VIOLET_BRIGHT, COL_MS_BG);
        g.setCursor(start_x, word_y);
        g.print(word);
        // Underline sweeps in from left to right during reveal_sub phase
        int uw = (int)(total_w * (reveal_sub > 0 ? reveal_sub : 1.0f));
        g.drawFastHLine(start_x, word_y + 22, uw, COL_MS_LIME);
        g.drawFastHLine(start_x, word_y + 23, uw, COL_MS_LIME_DIM);
    }

    // Subtitle (fades in during reveal_sub)
    if (reveal_sub > 0.1f) {
        g.setFont(&fonts::Font2);
        // Fade by picking dim vs bright teal based on reveal_sub
        uint16_t sub_col = reveal_sub > 0.5f ? COL_MS_TEAL_BRIGHT : COL_MS_TEAL;
        g.setTextColor(sub_col, COL_MS_BG);
        const char *sub1 = "CSI SITUATIONAL";
        const char *sub2 = "AWARENESS ENGINE";
        int w1 = g.textWidth(sub1);
        int w2 = g.textWidth(sub2);
        g.setCursor((SCREEN_W - w1) / 2, 194);
        g.print(sub1);
        g.setCursor((SCREEN_W - w2) / 2, 210);
        g.print(sub2);
    }

    // Version + prompt (only after everything else is in place)
    if (fully_revealed) {
        g.setFont(&fonts::Font0);
        g.setTextColor(COL_MS_MID, COL_MS_BG);
        int vw = g.textWidth(FW_VERSION);
        g.setCursor((SCREEN_W - vw) / 2, 240);
        g.print(FW_VERSION);

        g.setFont(&fonts::Font2);
        // Prompt pulses via lime<->violet
        uint32_t p = (millis() / 400) % 2;
        g.setTextColor(p ? COL_MS_LIME : COL_MS_VIOLET_BRIGHT, COL_MS_BG);
        const char *prompt = "> hold RIGHT to arm";
        int pw = g.textWidth(prompt);
        g.setCursor((SCREEN_W - pw) / 2, 268);
        g.print(prompt);
    }

    // Scan-line effect during reveal (a bright teal line sweeping down)
    if (!fully_revealed) {
        int scan_y = (int)((elapsed % 900) * SCREEN_H / 900);
        g.drawFastHLine(4, scan_y,     SCREEN_W - 8, COL_MS_TEAL_BRIGHT);
        g.drawFastHLine(4, scan_y + 1, SCREEN_W - 8, COL_MS_TEAL);
    }

    flush();
}

// ── Discovery ──────────────────────────────────────────────────
void ui_discovery(int found, uint32_t elapsed_ms, uint32_t deadline_ms) {
    clear();
    draw_header("DISCOVERY");
    auto &g = gfx();
    g.setTextColor(COL_TEXT, COL_BG);
    g.setFont(&fonts::Font2);
    g.setCursor(6, CONTENT_Y + 8);
    g.print("Listening on ch 11");
    g.setCursor(6, CONTENT_Y + 26);
    g.setTextColor(COL_MUTED, COL_BG);
    g.print("for CSI-Beacon TX...");

    g.setFont(&fonts::Font7);
    g.setTextColor(found > 0 ? COL_FG : COL_WARN, COL_BG);
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", found);
    int tw = g.textWidth(buf);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 60);
    g.print(buf);

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    const char *lbl = "beacon(s) found";
    tw = g.textWidth(lbl);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 130);
    g.print(lbl);

    // Beacon MAC list
    int y = CONTENT_Y + 160;
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        char line[40];
        snprintf(line, sizeof(line), "#%d  ID=%u  f=%lu",
                 i + 1, (unsigned)g_app.beacon[i].id,
                 (unsigned long)g_app.beacon[i].frames);
        g.setTextColor(COL_FG, COL_BG);
        g.setCursor(8, y);
        g.print(line);
        y += 14;
    }

    // Timer
    float p = (float)elapsed_ms / (float)deadline_ms;
    if (p > 1.0f) p = 1.0f;
    progress_bar(8, CONTENT_H + HEADER_H - 20, SCREEN_W - 16, 8, p, COL_ACCENT);

    draw_footer(found > 0 ? "recount" : "wait",
                found > 0 ? "accept"  : "-");
    flush();
}

// ── Geometry guide ─────────────────────────────────────────────
void ui_geometry_guide() {
    clear();
    draw_header("PLACE BEACONS");
    auto &g = gfx();
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 8);

    const int cx = SCREEN_W / 2;
    const int cy = CONTENT_Y + 130;
    const int R  = 55;

    switch (g_app.mode) {
        case RM_TRIANGLE_3: {
            g.print("3 beacons detected.");
            g.setCursor(6, CONTENT_Y + 26);
            g.setTextColor(COL_MUTED, COL_BG);
            g.print("Arrange as equilateral");
            g.setCursor(6, CONTENT_Y + 42);
            g.print("triangle, sides ~3 m.");
            g.setCursor(6, CONTENT_Y + 58);
            g.print("T-Display goes in the");
            g.setCursor(6, CONTENT_Y + 74);
            g.print("centre, facing up.");

            // Draw triangle — labels use the ACTUAL discovered beacon IDs
            // (slot 0/1/2), which may be any of 1..8 depending on which
            // beacons happen to be online.
            char lb0[6], lb1[6], lb2[6];
            snprintf(lb0, sizeof(lb0), "B%u",
                     (unsigned)(g_app.beacon[0].active ? g_app.beacon[0].id : 0));
            snprintf(lb1, sizeof(lb1), "B%u",
                     (unsigned)(g_app.beacon[1].active ? g_app.beacon[1].id : 0));
            snprintf(lb2, sizeof(lb2), "B%u",
                     (unsigned)(g_app.beacon[2].active ? g_app.beacon[2].id : 0));
            int x0 = cx,       y0 = cy - R;
            int x1 = cx - 48,  y1 = cy + 30;
            int x2 = cx + 48,  y2 = cy + 30;
            g.drawLine(x0, y0, x1, y1, COL_DIM);
            g.drawLine(x1, y1, x2, y2, COL_DIM);
            g.drawLine(x2, y2, x0, y0, COL_DIM);
            g.fillCircle(x0, y0, 6, COL_FG); g.setCursor(x0 - 4, y0 - 20); g.setTextColor(COL_FG); g.print(lb0);
            g.fillCircle(x1, y1, 6, COL_FG); g.setCursor(x1 - 22, y1);      g.print(lb1);
            g.fillCircle(x2, y2, 6, COL_FG); g.setCursor(x2 + 8, y2);       g.print(lb2);
            g.fillRect(cx - 6, cy - 4, 12, 20, COL_ACCENT);
            g.setTextColor(COL_ACCENT); g.setCursor(cx + 10, cy);            g.print("YOU");
            break;
        }
        case RM_LINE_2: {
            g.print("2 beacons detected.");
            g.setCursor(6, CONTENT_Y + 26);
            g.setTextColor(COL_MUTED, COL_BG);
            g.print("Place ~3 m apart facing");
            g.setCursor(6, CONTENT_Y + 42);
            g.print("across the guarded area.");
            g.setCursor(6, CONTENT_Y + 58);
            g.print("T-Display at midpoint,");
            g.setCursor(6, CONTENT_Y + 74);
            g.print("slightly offset if needed.");

            int y = cy;
            char lb0[6], lb1[6];
            snprintf(lb0, sizeof(lb0), "B%u",
                     (unsigned)(g_app.beacon[0].active ? g_app.beacon[0].id : 0));
            snprintf(lb1, sizeof(lb1), "B%u",
                     (unsigned)(g_app.beacon[1].active ? g_app.beacon[1].id : 0));
            g.drawLine(20, y, SCREEN_W - 20, y, COL_DIM);
            g.fillCircle(20, y, 6, COL_FG);           g.setTextColor(COL_FG); g.setCursor(6,  y - 20); g.print(lb0);
            g.fillCircle(SCREEN_W - 20, y, 6, COL_FG); g.setCursor(SCREEN_W - 28, y - 20); g.print(lb1);
            g.fillRect(cx - 6, y - 10, 12, 20, COL_ACCENT);
            g.setTextColor(COL_ACCENT); g.setCursor(cx + 10, y + 4); g.print("YOU");
            break;
        }
        case RM_TRIPWIRE_1: {
            g.print("1 beacon detected.");
            g.setCursor(6, CONTENT_Y + 26);
            g.setTextColor(COL_MUTED, COL_BG);
            g.print("Place across the doorway,");
            g.setCursor(6, CONTENT_Y + 42);
            g.print("~2-3 m from T-Display.");
            g.setCursor(6, CONTENT_Y + 58);
            g.print("Anything crossing the");
            g.setCursor(6, CONTENT_Y + 74);
            g.print("link will trip the wire.");

            char lb0[6];
            snprintf(lb0, sizeof(lb0), "B%u",
                     (unsigned)(g_app.beacon[0].active ? g_app.beacon[0].id : 0));
            g.drawLine(cx, cy - R, cx, cy + R, COL_DIM);
            g.fillCircle(cx, cy - R, 6, COL_FG); g.setTextColor(COL_FG); g.setCursor(cx + 10, cy - R - 4); g.print(lb0);
            g.fillRect(cx - 6, cy + R - 6, 12, 20, COL_ACCENT);
            g.setTextColor(COL_ACCENT); g.setCursor(cx + 10, cy + R); g.print("YOU");
            break;
        }
        default: {
            g.setTextColor(COL_ALERT, COL_BG);
            g.print("No beacons.");
            g.setCursor(6, CONTENT_Y + 26);
            g.setTextColor(COL_MUTED, COL_BG);
            g.print("Power on a CSI-Beacon,");
            g.setCursor(6, CONTENT_Y + 42);
            g.print("then long-press LEFT.");
            break;
        }
    }

    draw_footer("back", "ready");
    flush();
}

// ── Center T-Display ──────────────────────────────────────────
void ui_center_tdisplay() {
    clear();
    draw_header("PLACE T-DISPLAY");
    auto &g = gfx();
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 8);
    g.print("Set the T-Display at");
    g.setCursor(6, CONTENT_Y + 24);
    g.print("the centre of the beacon");
    g.setCursor(6, CONTENT_Y + 40);
    g.print("layout, screen facing up.");

    g.setTextColor(COL_MUTED, COL_BG);
    g.setCursor(6, CONTENT_Y + 64);
    g.print("This defines the coordinate");
    g.setCursor(6, CONTENT_Y + 80);
    g.print("origin used for spatial");
    g.setCursor(6, CONTENT_Y + 96);
    g.print("estimation.");

    // Little iso of T-Display
    int cx = SCREEN_W / 2;
    int cy = CONTENT_Y + 170;
    g.fillRect(cx - 22, cy - 30, 44, 60, COL_MS_DIM);          // device chassis
    g.drawRect(cx - 22, cy - 30, 44, 60, COL_ACCENT);
    g.fillRect(cx - 18, cy - 24, 36, 40, COL_MS_VIOLET);       // screen (dark violet)
    // ping ring
    uint32_t t = millis() / 30;
    int r = 6 + (int)(t % 34);
    g.drawCircle(cx, cy, r, COL_DIM);

    g.setTextColor(COL_FG, COL_BG);
    g.setFont(&fonts::Font2);
    const char *msg = "confirm centred";
    int tw = g.textWidth(msg);
    g.setCursor((SCREEN_W - tw) / 2, SCREEN_H - FOOTER_H - 22);
    g.print(msg);

    draw_footer("back", "OK");
    flush();
}

// ── Baseline countdown ────────────────────────────────────────
void ui_baseline_countdown(int seconds_remaining) {
    clear();
    draw_header("LEAVE THE ROOM");
    auto &g = gfx();
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 8);
    g.print("Empty-room baseline");
    g.setCursor(6, CONTENT_Y + 24);
    g.setTextColor(COL_MUTED, COL_BG);
    g.print("starts in...");

    g.setFont(&fonts::Font7);
    g.setTextColor(seconds_remaining <= 3 ? COL_WARN : COL_FG, COL_BG);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", seconds_remaining);
    int tw = g.textWidth(buf);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 60);
    g.print(buf);

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    const char *msg = "seconds";
    tw = g.textWidth(msg);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 140);
    g.print(msg);

    g.setTextColor(COL_ACCENT, COL_BG);
    g.setCursor(4, CONTENT_Y + 180);
    g.print("Move out of the area,");
    g.setCursor(4, CONTENT_Y + 196);
    g.print("close doors, keep still.");

    draw_footer("cancel", "-");
    flush();
}

// ── Baseline capture ──────────────────────────────────────────
void ui_baseline_capture(float progress) {
    clear();
    draw_header("BASELINE");
    auto &g = gfx();
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 8);
    g.print("Recording empty room...");

    g.setFont(&fonts::Font4);
    char buf[16];
    snprintf(buf, sizeof(buf), "%3d%%", (int)(progress * 100));
    int tw = g.textWidth(buf);
    g.setTextColor(COL_FG, COL_BG);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 50);
    g.print(buf);

    progress_bar(10, CONTENT_Y + 100, SCREEN_W - 20, 14, progress, COL_FG);

    // Per-beacon count-in
    int y = CONTENT_Y + 130;
    g.setFont(&fonts::Font2);
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        char line[40];
        snprintf(line, sizeof(line), "B%u  %d/%d", (unsigned)b.id, b.cal_count, BASELINE_FRAMES);
        g.setTextColor(b.cal_count >= BASELINE_FRAMES ? COL_FG : COL_MUTED, COL_BG);
        g.setCursor(10, y);
        g.print(line);
        y += 14;
    }

    g.setCursor(4, SCREEN_H - FOOTER_H - 40);
    g.setTextColor(COL_ACCENT, COL_BG);
    g.print("Stay OUT of the area.");

    draw_footer("cancel", "-");
    flush();
}

// ── Walk guide ────────────────────────────────────────────────
void ui_walk_guide() {
    clear();
    draw_header("WALK CALIBRATE");
    auto &g = gfx();
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 8);
    g.print("Optional but recommended.");
    g.setTextColor(COL_MUTED, COL_BG);
    g.setCursor(6, CONTENT_Y + 30);
    g.print("On confirm, walk a slow");
    g.setCursor(6, CONTENT_Y + 46);
    g.print("figure-8 between beacons");
    g.setCursor(6, CONTENT_Y + 62);
    g.print("for ~20 seconds. This");
    g.setCursor(6, CONTENT_Y + 78);
    g.print("scales per-link metrics.");

    // Small figure-8 illustration
    int cx = SCREEN_W / 2, cy = CONTENT_Y + 170, rx = 40, ry = 20;
    for (int a = 0; a < 360; a += 6) {
        float t = a * (float)M_PI / 180.0f;
        int x = cx + (int)(rx * sinf(2 * t));
        int y = cy + (int)(ry * sinf(t));
        g.drawPixel(x, y, COL_FG);
    }
    g.fillCircle(cx - rx, cy, 5, COL_ACCENT);
    g.fillCircle(cx + rx, cy, 5, COL_ACCENT);

    draw_footer("skip", "start");
    flush();
}

void ui_walk_capture(float progress) {
    clear();
    draw_header("WALK");
    auto &g = gfx();
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 8);
    g.print("Walk the pattern...");

    progress_bar(10, CONTENT_Y + 40, SCREEN_W - 20, 14, progress, COL_ACCENT);

    int y = CONTENT_Y + 70;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        char line[40];
        snprintf(line, sizeof(line), "B%u peak %.4f", (unsigned)b.id, b.walk_peak);
        g.setTextColor(b.walk_calibrated ? COL_FG : COL_MUTED, COL_BG);
        g.setCursor(10, y);
        g.print(line);
        y += 16;

        float p = b.threshold > 0 ? (b.walk_peak / (b.threshold * 6.0f)) : 0;
        if (p > 1.0f) p = 1.0f;
        progress_bar(10, y, SCREEN_W - 20, 6, p, b.walk_calibrated ? COL_FG : COL_WARN);
        y += 14;
    }

    draw_footer("stop", "-");
    flush();
}

// ── Dashboard views ───────────────────────────────────────────

// ═══════════════════════════════════════════════════════════════
//  RADAR VIEW — the main event.
//
//  Full-screen top-down map with the tracker's target estimate as
//  a dot plus covariance ellipse.  Beacons drawn at their calibrated
//  positions, RX at the origin (paired bar in stereo mode).  Fading
//  trail behind the target so motion is visible even between updates.
//  When stereo is active, per-beacon AoA rays are drawn faintly from
//  the RX at the measured angle.  A small strip at the bottom shows
//  each beacon's link disturbance as a mini oscilloscope trace — the
//  "raw signal" moved from center stage to a diagnostic sidebar.
// ═══════════════════════════════════════════════════════════════

// Mini oscilloscope: ring buffer of recent link metrics per beacon.
// Pushed every time the radar view is drawn, so at UI redraw rate.
#define RADAR_SCOPE_LEN 160
static float s_scope[MAX_BEACONS][RADAR_SCOPE_LEN];
static int   s_scope_head = 0;
static bool  s_scope_init = false;

// Draw one point of the covariance ellipse.  For a 2×2 symmetric
// covariance [[vxx, vxy], [vxy, vyy]], eigendecompose to get the
// principal axes, then draw a 2σ ellipse as a polyline.  Cheap: 24
// segments, one atan2, one sqrt per axis.
static void draw_confidence_ellipse(int cx, int cy, float pix_per_cm,
                                    float vxx, float vyy, float vxy,
                                    uint16_t col) {
    // Eigenvalues
    float tr = vxx + vyy;
    float det = vxx * vyy - vxy * vxy;
    float disc = tr * tr * 0.25f - det;
    if (disc < 0) disc = 0;
    float sd = sqrtf(disc);
    float l1 = tr * 0.5f + sd;   // major
    float l2 = tr * 0.5f - sd;   // minor
    if (l1 < 1) l1 = 1;
    if (l2 < 1) l2 = 1;
    // Eigenvector angle for major axis
    float ang;
    if (fabsf(vxy) < 1e-6f) ang = (vxx >= vyy) ? 0.0f : (float)M_PI * 0.5f;
    else                    ang = atan2f(l1 - vxx, vxy);
    float ca = cosf(ang), sa = sinf(ang);
    // 2σ semi-axes in pixels
    float ax = 2.0f * sqrtf(l1) * pix_per_cm;
    float ay = 2.0f * sqrtf(l2) * pix_per_cm;
    // Cap to keep pathological ellipses from swamping the screen
    if (ax > 90) ax = 90;
    if (ay > 90) ay = 90;

    const int N = 24;
    int prev_px = 0, prev_py = 0;
    for (int i = 0; i <= N; i++) {
        float t = (float)i / (float)N * 2.0f * (float)M_PI;
        float ux = ax * cosf(t);
        float uy = ay * sinf(t);
        // Rotate + place at (cx, cy).  Screen Y is inverted.
        int px = cx + (int)(ca * ux - sa * uy);
        int py = cy - (int)(sa * ux + ca * uy);
        if (i > 0) gfx().drawLine(prev_px, prev_py, px, py, col);
        prev_px = px; prev_py = py;
    }
}

static void draw_view_radar() {
    auto &g = gfx();

    // ── Layout ──
    // 22 header (already drawn by ui_dashboard) + 14 status band +
    // 190 radar box + 50 mini scope band + 22 footer = 298.
    const int status_y  = CONTENT_Y + 2;
    const int map_y     = CONTENT_Y + 18;
    const int map_h     = 190;
    const int scope_y   = map_y + map_h + 4;
    const int scope_h   = 44;

    // ── Status band: mode badge + confidence % ──
    g.setFont(&fonts::Font2);
    const char *mode_lbl = "?";
    uint16_t mode_col = COL_MUTED;
    switch (g_app.peer.role) {
        case ROLE_PRIMARY:
            mode_lbl = g_app.peer.peer_present ? "STEREO" : "STEREO?";
            mode_col = g_app.peer.peer_present ? COL_ACCENT : COL_WARN;
            break;
        case ROLE_SECONDARY: mode_lbl = "SECONDARY"; mode_col = COL_WARN; break;
        case ROLE_SOLO:      mode_lbl = "SOLO";      mode_col = COL_FG;   break;
        default:             mode_lbl = "..";        mode_col = COL_MUTED; break;
    }
    g.setTextColor(mode_col, COL_BG);
    g.setCursor(4, status_y);
    g.print(mode_lbl);
    // Confidence %
    char cbuf[16];
    snprintf(cbuf, sizeof(cbuf), "%3d%%",
             (int)(g_app.tracker.confidence * 100.0f));
    int cw = g.textWidth(cbuf);
    uint16_t cc = g_app.tracker.confidence > 0.5f ? COL_FG
                : g_app.tracker.confidence > 0.2f ? COL_WARN : COL_MUTED;
    g.setTextColor(cc, COL_BG);
    g.setCursor(SCREEN_W - cw - 4, status_y);
    g.print(cbuf);

    // ── Radar map ──
    const int cx = SCREEN_W / 2;
    const int cy = map_y + map_h / 2;
    // Fit the room half-extent into the smaller of (map width/2, map height/2)
    const int usable = (SCREEN_W / 2) - 6;   // leave 6 px padding
    const int usable_v = (map_h / 2) - 6;
    const int R = (usable < usable_v) ? usable : usable_v;
    const float pix_per_cm = (float)R / TRACKER_ROOM_HALF_CM;

    // Map frame + range rings
    g.drawRect(0, map_y, SCREEN_W, map_h, COL_GRID_DARK);
    for (int r_cm = 100; r_cm <= (int)TRACKER_ROOM_HALF_CM; r_cm += 100) {
        int rp = (int)(r_cm * pix_per_cm);
        g.drawCircle(cx, cy, rp, COL_GRID_DARK);
    }
    // Axes
    g.drawFastHLine(cx - R, cy, 2 * R, COL_GRID_DARK);
    g.drawFastVLine(cx, cy - R, 2 * R, COL_GRID_DARK);
    // North label ("forward" from the RX bar in stereo)
    g.setTextColor(COL_DIM, COL_BG);
    g.setCursor(cx + 2, cy - R + 1);
    g.print("N");

    // ── Beacons ──
    // Draw links (dim lines RX→beacon) first so beacon dots overpaint them.
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        int bx = cx + (int)(b.pos_x * pix_per_cm);
        int by = cy - (int)(b.pos_y * pix_per_cm);
        uint16_t lc = COL_GRID;
        if (b.link_metric_ema > 0.4f) lc = COL_WARN;
        if (b.link_metric_ema > 0.7f) lc = COL_ALERT;
        g.drawLine(cx, cy, bx, by, lc);
    }
    // AoA rays (stereo primary only) — draw faint colored rays from RX at
    // each beacon's measured angle.  Convention: theta = atan2(x, y) with
    // "north" = +Y, so ray endpoint = (R sin θ, R cos θ) from RX.
    if (g_app.peer.role == ROLE_PRIMARY && g_app.peer.peer_present) {
        uint16_t ray_cols[3] = {COL_MS_CH_A, COL_MS_CH_B, COL_MS_CH_C};
        uint32_t now = millis();
        int col = 0;
        for (int i = 0; i < MAX_BEACONS; i++) {
            BeaconState &b = g_app.beacon[i];
            if (!b.active) { col++; continue; }
            if (b.aoa_conf < 0.05f) { col++; continue; }
            if ((now - b.last_aoa_ms) > 500) { col++; continue; }
            float a = b.aoa_rad;
            int rx_end = cx + (int)(R * sinf(a));
            int ry_end = cy - (int)(R * cosf(a));
            g.drawLine(cx, cy, rx_end, ry_end, ray_cols[col % 3]);
            col++;
        }
    }
    // Beacon dots on top
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        int bx = cx + (int)(b.pos_x * pix_per_cm);
        int by = cy - (int)(b.pos_y * pix_per_cm);
        uint16_t bc = COL_FG;
        if (b.status == LS_MOTION)   bc = COL_WARN;
        if (b.status == LS_PRESENCE) bc = COL_ALERT;
        g.fillCircle(bx, by, 4, bc);
        g.drawCircle(bx, by, 4, COL_TEXT);
        g.setFont(&fonts::Font2);
        g.setTextColor(COL_MUTED, COL_BG);
        char lbl[8]; snprintf(lbl, sizeof(lbl), "%u", (unsigned)b.id);
        g.setCursor(bx + 6, by - 8);
        g.print(lbl);
    }

    // ── RX icon ──
    // Solo: single small blue square.  Stereo primary: two squares 6cm apart
    // to show the actual antenna baseline the tracker is using.
    if (g_app.peer.role == ROLE_PRIMARY && g_app.peer.peer_present) {
        int half = (int)(STEREO_BASELINE_CM * 0.5f * pix_per_cm);
        if (half < 2) half = 2;
        g.fillRect(cx - half - 2, cy - 2, 4, 5, COL_ACCENT);
        g.fillRect(cx + half - 2, cy - 2, 4, 5, COL_ACCENT);
        g.drawLine(cx - half, cy, cx + half, cy, COL_ACCENT);
    } else {
        g.fillRect(cx - 3, cy - 3, 6, 7, COL_ACCENT);
    }

    // ── Trail (fading) ──
    if (g_app.tracker.trail_count > 0) {
        uint32_t now = millis();
        int start = g_app.tracker.trail_head;
        int total = g_app.tracker.trail_count;
        for (int k = 0; k < total; k++) {
            // Walk from oldest to newest
            int idx = (start + TRACKER_TRAIL_LEN - total + k) % TRACKER_TRAIL_LEN;
            auto &p = g_app.tracker.trail[idx];
            if (p.t_ms == 0) continue;
            uint32_t age = now - p.t_ms;
            if (age > 3000) continue;
            int px = cx + (int)(p.x_cm * pix_per_cm);
            int py = cy - (int)(p.y_cm * pix_per_cm);
            // Age fade: newest = warm color, oldest = dim
            uint16_t tc = (age < 500)  ? COL_WARN
                        : (age < 1500) ? COL_DIM
                                       : COL_GRID_DARK;
            int rad = (age < 500) ? 2 : 1;
            g.fillCircle(px, py, rad, tc);
        }
    }

    // ── Target + confidence ellipse ──
    if (g_app.tracker.valid && g_app.tracker.confidence > 0.05f) {
        int tx = cx + (int)(g_app.tracker.x_cm * pix_per_cm);
        int ty = cy - (int)(g_app.tracker.y_cm * pix_per_cm);
        uint16_t tcol = g_app.tracker.confidence > 0.5f ? COL_ALERT : COL_WARN;
        draw_confidence_ellipse(tx, ty, pix_per_cm,
                                g_app.tracker.cov_xx,
                                g_app.tracker.cov_yy,
                                g_app.tracker.cov_xy,
                                COL_DIM);
        // Concentric target rings — makes the dot pop even at low conf
        g.drawCircle(tx, ty, 6, tcol);
        g.drawCircle(tx, ty, 3, tcol);
        g.fillCircle(tx, ty, 2, COL_TEXT);
    }

    // ── Alert border ──
    if (g_app.alert_latched && (millis() - g_app.last_alert_ms) < 4000) {
        g.drawRect(0, map_y, SCREEN_W, map_h, COL_ALERT);
        g.drawRect(1, map_y + 1, SCREEN_W - 2, map_h - 2, COL_ALERT);
    }

    // ── Mini oscilloscope strip ──
    // Push current values
    if (!s_scope_init) {
        for (int i = 0; i < MAX_BEACONS; i++)
            for (int k = 0; k < RADAR_SCOPE_LEN; k++) s_scope[i][k] = 0;
        s_scope_init = true;
    }
    for (int i = 0; i < MAX_BEACONS; i++) {
        s_scope[i][s_scope_head] = g_app.beacon[i].active
                                 ? g_app.beacon[i].link_metric_ema : 0;
    }
    s_scope_head = (s_scope_head + 1) % RADAR_SCOPE_LEN;

    // Frame
    g.drawRect(0, scope_y, SCREEN_W, scope_h, COL_GRID_DARK);
    // Threshold line at 0.5
    int mid = scope_y + scope_h / 2;
    g.drawFastHLine(1, mid, SCREEN_W - 2, COL_GRID_DARK);

    uint16_t bcols[3] = {COL_MS_CH_A, COL_MS_CH_B, COL_MS_CH_C};
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        uint16_t c = bcols[i % 3];
        int prev_x = 0, prev_y = 0;
        int n = RADAR_SCOPE_LEN < SCREEN_W ? RADAR_SCOPE_LEN : SCREEN_W;
        for (int k = 0; k < n; k++) {
            int idx = (s_scope_head + k) % RADAR_SCOPE_LEN;
            float v = s_scope[i][idx];
            if (v > 1) v = 1;
            int px = k * SCREEN_W / n;
            int py = scope_y + scope_h - 2 - (int)(v * (scope_h - 4));
            if (k > 0) g.drawLine(prev_x, prev_y, px, py, c);
            prev_x = px; prev_y = py;
        }
    }
    // Label + LO drift readout tucked in the strip corner
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    g.setCursor(4, scope_y + 2);
    g.print("links");
    if (g_app.peer.role == ROLE_PRIMARY && g_app.peer.peer_present) {
        char lo[16];
        snprintf(lo, sizeof(lo), "LO %+.2f", (double)g_app.peer.lo_drift_ema);
        int lw = g.textWidth(lo);
        g.setCursor(SCREEN_W - lw - 4, scope_y + 2);
        g.print(lo);
    }
}

static void draw_view_tripwire() {
    auto &g = gfx();

    // Determine composite state
    bool any_motion = false, any_presence = false;
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        if (g_app.beacon[i].status == LS_MOTION)   any_motion = true;
        if (g_app.beacon[i].status == LS_PRESENCE) any_presence = true;
    }
    const char *state = "SECURE";
    uint16_t c = COL_FG;
    if (any_motion)   { state = "MOTION";   c = COL_WARN; }
    if (any_presence) { state = "ALERT";    c = COL_ALERT; }

    // Full-screen wash — dim variants of the state color so text on top pops
    // Secure = dim lime, motion = dim amber, alert = dim hot pink
    uint16_t fill;
    if      (c == COL_FG)   fill = COL_MS_WASH_OK;
    else if (c == COL_WARN) fill = COL_MS_WASH_WARN;
    else                    fill = COL_MS_WASH_ALERT;
    g.fillRect(0, CONTENT_Y, SCREEN_W, CONTENT_H, fill);
    // Font4 supports letters (Font7 is 7-segment numeric-only)
    g.setFont(&fonts::Font4);
    g.setTextColor(c, fill);
    int tw = g.textWidth(state);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 60);
    g.print(state);

    // Latch banner
    if (g_app.alert_latched) {
        g.setFont(&fonts::Font2);
        g.setTextColor(COL_ALERT, COL_BG);
        uint32_t age = (millis() - g_app.last_alert_ms) / 1000;
        char msg[32]; snprintf(msg, sizeof(msg), "ALERTED  %us ago", (unsigned)age);
        tw = g.textWidth(msg);
        g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 130);
        g.print(msg);
    }

    // Per-beacon dots
    int y = CONTENT_Y + 170;
    g.setFont(&fonts::Font2);
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        uint16_t dc = COL_FG;
        if (b.status == LS_MOTION)   dc = COL_WARN;
        if (b.status == LS_PRESENCE) dc = COL_ALERT;
        g.fillCircle(20 + i * 40, y, 10, dc);
        g.drawCircle(20 + i * 40, y, 10, COL_TEXT);
        g.setTextColor(COL_TEXT, COL_BG);
        char lbl[8]; snprintf(lbl, sizeof(lbl), "B%u", (unsigned)g_app.beacon[i].id);
        g.setCursor(12 + i * 40, y + 14);
        g.print(lbl);
    }
}

static void draw_view_links() {
    auto &g = gfx();
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 4);
    g.print("Per-link metrics");

    int y = CONTENT_Y + 24;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;

        char hdr[32]; snprintf(hdr, sizeof(hdr), "B%u", (unsigned)b.id);
        g.setTextColor(COL_ACCENT, COL_BG);
        g.setCursor(6, y);
        g.print(hdr);

        // MAC
        char mac[24];
        snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 b.mac[0], b.mac[1], b.mac[2], b.mac[3], b.mac[4], b.mac[5]);
        g.setTextColor(COL_MUTED, COL_BG);
        g.setFont(&fonts::Font0);
        g.setCursor(6, y + 14);
        g.print(mac);
        g.setFont(&fonts::Font2);

        // Turbulence / MV / metric
        char l1[40]; snprintf(l1, sizeof(l1), "turb %.3f  MV %.4f", b.feat_turbulence, b.moving_variance);
        g.setTextColor(COL_TEXT, COL_BG);
        g.setCursor(6, y + 26);
        g.print(l1);

        uint16_t c = COL_FG;
        if (b.status == LS_MOTION)   c = COL_WARN;
        if (b.status == LS_PRESENCE) c = COL_ALERT;
        progress_bar(6, y + 42, SCREEN_W - 40, 10, b.link_metric_ema, c);
        char pct[8]; snprintf(pct, sizeof(pct), "%3d%%", (int)(b.link_metric_ema * 100));
        g.setTextColor(COL_MUTED, COL_BG);
        g.setCursor(SCREEN_W - 30, y + 40);
        g.print(pct);

        y += 66;
        if (y > SCREEN_H - FOOTER_H - 20) break;
    }
}

static uint8_t s_csi_sel = 0;   // which beacon's CSI to show

void ui_csi_next_beacon() {
    // Advance to next active beacon slot, wrap around.
    for (int i = 1; i <= MAX_BEACONS; i++) {
        int idx = (s_csi_sel + i) % MAX_BEACONS;
        if (g_app.beacon[idx].active) { s_csi_sel = (uint8_t)idx; return; }
    }
}

static void draw_view_csi() {
    auto &g = gfx();
    // Find first active beacon at s_csi_sel or after
    int slot = -1;
    int scanned = 0;
    int idx = s_csi_sel;
    while (scanned < MAX_BEACONS) {
        if (g_app.beacon[idx].active) { slot = idx; break; }
        idx = (idx + 1) % MAX_BEACONS;
        scanned++;
    }

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 4);
    if (slot < 0) {
        g.setTextColor(COL_ALERT);
        g.print("no beacon");
        return;
    }
    BeaconState &b = g_app.beacon[slot];
    char hdr[24]; snprintf(hdr, sizeof(hdr), "CSI  B%u", (unsigned)b.id);
    g.print(hdr);

    // Find max amp for scaling
    float maxA = 1.0f;
    for (int i = 0; i < CSI_NUM_SUBCARRIERS; i++)
        if (b.amplitude[i] > maxA) maxA = b.amplitude[i];

    // Plot area
    const int px = 6, py = CONTENT_Y + 28, pw = SCREEN_W - 12, ph = 120;
    g.drawRect(px, py, pw, ph, COL_GRID_DARK);
    // Bars for each subcarrier
    int bw = pw / CSI_NUM_SUBCARRIERS;
    if (bw < 1) bw = 1;
    for (int i = 0; i < CSI_NUM_SUBCARRIERS; i++) {
        float v = b.amplitude[i] / maxA;
        int   h = (int)(v * (ph - 2));
        int   x = px + i * bw;
        // Highlight selected subcarriers
        bool sel = false;
        for (int k = 0; k < CSI_SEL_COUNT; k++) if (CSI_SEL_SC[k] == i) { sel = true; break; }
        uint16_t c = sel ? COL_ACCENT : COL_FG;
        g.fillRect(x + 1, py + ph - 1 - h, bw - 1, h, c);
    }
    // Baseline overlay
    if (b.baseline_valid) {
        int prev_y = -1, prev_x = -1;
        for (int i = 0; i < CSI_NUM_SUBCARRIERS; i++) {
            float v = b.baseline[i] / maxA;
            int y = py + ph - 1 - (int)(v * (ph - 2));
            int x = px + i * bw + bw / 2;
            if (prev_y >= 0) g.drawLine(prev_x, prev_y, x, y, COL_WARN);
            prev_x = x; prev_y = y;
        }
    }

    // Legend
    g.setFont(&fonts::Font0);
    g.setTextColor(COL_ACCENT, COL_BG);
    g.setCursor(6, py + ph + 4);
    g.print("selected sc");
    g.setTextColor(COL_WARN, COL_BG);
    g.setCursor(80, py + ph + 4);
    g.print("baseline");

    // Features
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    int y = py + ph + 20;
    char l[40];
    snprintf(l, sizeof(l), "turb  %.4f", b.feat_turbulence);        g.setCursor(6, y); g.print(l); y += 14;
    snprintf(l, sizeof(l), "mean  %.2f", b.feat_mean_amp);          g.setCursor(6, y); g.print(l); y += 14;
    snprintf(l, sizeof(l), "d(base) %.2f", b.feat_delta_baseline);  g.setCursor(6, y); g.print(l); y += 14;
    snprintf(l, sizeof(l), "d(t)  %.2f", b.feat_temporal_delta);    g.setCursor(6, y); g.print(l); y += 14;
}

// Forward declarations for stereo-specific views defined further down
static void draw_view_aoa();
static void draw_view_peer();
// v0.3: primary radar view + field debug view live in render.cpp.
extern void render_radar_view();
extern void render_field_view();

void ui_dashboard() {
    clear();
    // Header shows current view name
    const char *hdrs[] = {"RADAR", "FIELD", "AOA", "TRIPWIRE", "LINKS", "CSI", "PEER"};
    const char *h = (g_app.dash_view < DV_COUNT) ? hdrs[g_app.dash_view] : "?";
    draw_header(h);

    switch (g_app.dash_view) {
        case DV_RADAR:    render_radar_view();  break;   // v0.3 primary view
        case DV_FIELD:    render_field_view();  break;   // v0.3 debug view
        case DV_AOA:      draw_view_aoa();      break;
        case DV_TRIPWIRE: draw_view_tripwire(); break;
        case DV_LINKS:    draw_view_links();    break;
        case DV_CSI:      draw_view_csi();      break;
        case DV_PEER:     draw_view_peer();     break;
        default: break;
    }

    // Footer: navigation
    const char *rlabel = "action";
    if (g_app.dash_view == DV_TRIPWIRE && g_app.alert_latched) rlabel = "ack";
    if (g_app.dash_view == DV_CSI)  rlabel = "next B";
    if (g_app.dash_view == DV_AOA)  rlabel = "-";
    if (g_app.dash_view == DV_PEER) rlabel = "-";
    if (g_app.dash_view == DV_FIELD) rlabel = "-";
    draw_footer("view", rlabel);
    flush();
}

// ── Settings ──────────────────────────────────────────────────
void ui_settings(int selected_row) {
    clear();
    draw_header("SETTINGS");
    auto &g = gfx();
    g.setFont(&fonts::Font2);

    const int rows = 5;
    const char *labels[rows] = {
        "Sensitivity",
        "Recalibrate baseline",
        "Redo walk-cal",
        "Change mode",
        "Exit",
    };
    char values[rows][24] = {};
    snprintf(values[0], 24, "%.1fx", g_app.sensitivity);
    strcpy(values[1], "");
    strcpy(values[2], "");
    const char *mode_names[] = {"none", "1-tripwire", "2-line", "3-triangle"};
    snprintf(values[3], 24, "%s", mode_names[g_app.mode]);
    strcpy(values[4], "");

    int y = CONTENT_Y + 10;
    for (int i = 0; i < rows; i++) {
        bool sel = (i == selected_row);
        uint16_t c = sel ? COL_ACCENT : COL_TEXT;
        if (sel) g.fillRect(0, y - 2, SCREEN_W, 18, COL_MS_CHROME);
        g.setTextColor(c, sel ? COL_MS_CHROME : COL_BG);
        g.setCursor(8, y);
        g.print(labels[i]);
        if (values[i][0]) {
            int tw = g.textWidth(values[i]);
            g.setCursor(SCREEN_W - tw - 8, y);
            g.print(values[i]);
        }
        y += 22;
    }

    draw_footer("select", "adjust");
    flush();
}

// ── Generic message ──────────────────────────────────────────
void ui_message(const char *title, const char *line1, const char *line2, uint16_t title_color) {
    clear();
    draw_header(title);
    auto &g = gfx();
    g.setFont(&fonts::Font4);
    g.setTextColor(title_color, COL_BG);
    int tw = g.textWidth(line1);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 40);
    g.print(line1);

    if (line2) {
        g.setFont(&fonts::Font2);
        g.setTextColor(COL_MUTED, COL_BG);
        tw = g.textWidth(line2);
        g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 90);
        g.print(line2);
    }
    draw_footer(nullptr, nullptr);
    flush();
}

// ── Sleep-arm ──────────────────────────────────────────────────
// Both buttons are being held.  Draws a full-screen dark screen with
// a big "GOING TO SLEEP" label and a filling bar at the bottom.  If
// the user releases before the bar fills, the state machine cancels
// and returns to whatever screen they were on.
void ui_sleep_arm(float progress) {
    if (progress < 0) progress = 0;
    if (progress > 1) progress = 1;

    clear();
    auto &g = gfx();

    // Title
    g.setFont(&fonts::Font4);
    g.setTextColor(COL_WARN, COL_BG);
    const char *t1 = "GOING TO";
    const char *t2 = "SLEEP";
    int tw1 = g.textWidth(t1);
    int tw2 = g.textWidth(t2);
    g.setCursor((SCREEN_W - tw1) / 2, 80);  g.print(t1);
    g.setCursor((SCREEN_W - tw2) / 2, 115); g.print(t2);

    // Hint
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    const char *hint = "release to cancel";
    int hw = g.textWidth(hint);
    g.setCursor((SCREEN_W - hw) / 2, 175);
    g.print(hint);

    // Progress bar (thicker than usual for emphasis)
    int bar_w = SCREEN_W - 40;
    int bar_h = 18;
    int bar_x = 20;
    int bar_y = SCREEN_H - 60;
    progress_bar(bar_x, bar_y, bar_w, bar_h, progress, COL_WARN);

    // Big percent under the bar
    char pct[8];
    snprintf(pct, sizeof(pct), "%d%%", (int)(progress * 100.0f));
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    int pw = g.textWidth(pct);
    g.setCursor((SCREEN_W - pw) / 2, bar_y + bar_h + 8);
    g.print(pct);

    flush();
}

// Shown for ~300–500 ms just before esp_deep_sleep_start().
void ui_going_to_sleep() {
    clear();
    auto &g = gfx();
    g.setFont(&fonts::Font4);
    g.setTextColor(COL_ACCENT, COL_BG);
    const char *t = "Sleeping";
    int tw = g.textWidth(t);
    g.setCursor((SCREEN_W - tw) / 2, SCREEN_H / 2 - 20);
    g.print(t);

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    const char *hint = "press any button";
    int hw = g.textWidth(hint);
    g.setCursor((SCREEN_W - hw) / 2, SCREEN_H / 2 + 20);
    g.print(hint);

    const char *hint2 = "to wake";
    int hw2 = g.textWidth(hint2);
    g.setCursor((SCREEN_W - hw2) / 2, SCREEN_H / 2 + 40);
    g.print(hint2);

    flush();
}

// ═══════════════════════════════════════════════════════════════
//  Stereo-related setup screens + dashboard sub-views
// ═══════════════════════════════════════════════════════════════

static void draw_role_pill(int x, int y) {
    // Small pill in the header showing our role.  Called from setup screens
    // where the standard header doesn't already carry this info.
    auto &g = gfx();
    const char *lbl = "?";
    uint16_t col = COL_MUTED;
    switch (g_app.peer.role) {
        case ROLE_PRIMARY:   lbl = "PRIMARY";   col = COL_ACCENT; break;
        case ROLE_SECONDARY: lbl = "SECONDARY"; col = COL_WARN;   break;
        case ROLE_SOLO:      lbl = "SOLO";      col = COL_FG;     break;
        default:             lbl = "..";        col = COL_MUTED;  break;
    }
    g.setFont(&fonts::Font2);
    g.setTextColor(col, COL_BG);
    int w = g.textWidth(lbl);
    g.drawRect(x - 3, y - 2, w + 6, 15, col);
    g.setCursor(x, y);
    g.print(lbl);
}

// ── Peer discovery ─────────────────────────────────────────────
// Shows a spinner + "looking for peer T-Display" + elapsed time.
void ui_peer_discovery(uint32_t elapsed_ms, uint32_t deadline_ms) {
    clear();
    draw_header("PEER LINK");
    auto &g = gfx();

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 10);
    g.print("Looking for a second");
    g.setCursor(6, CONTENT_Y + 26);
    g.print("T-Display-S3 on ch 11.");

    g.setTextColor(COL_MUTED, COL_BG);
    g.setCursor(6, CONTENT_Y + 52);
    g.print("If you're running solo,");
    g.setCursor(6, CONTENT_Y + 68);
    g.print("just wait or press RIGHT.");

    // Big status text
    g.setFont(&fonts::Font4);
    if (g_app.peer.peer_present) {
        g.setTextColor(COL_ACCENT, COL_BG);
        const char *m = "FOUND";
        int tw = g.textWidth(m);
        g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 110);
        g.print(m);
        // Show peer MAC last 3 bytes
        g.setFont(&fonts::Font2);
        g.setTextColor(COL_MUTED, COL_BG);
        char mac[24];
        snprintf(mac, sizeof(mac), "peer %02X:%02X:%02X",
                 g_app.peer.peer_mac[3], g_app.peer.peer_mac[4], g_app.peer.peer_mac[5]);
        int mw = g.textWidth(mac);
        g.setCursor((SCREEN_W - mw) / 2, CONTENT_Y + 150);
        g.print(mac);
    } else {
        g.setTextColor(COL_MUTED, COL_BG);
        // Simple animated dots based on elapsed_ms
        int dots = (elapsed_ms / 400) % 4;
        char anim[8] = "listen";
        char full[12];
        snprintf(full, sizeof(full), "%s%.*s", anim, dots, "...");
        int tw = g.textWidth(full);
        g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 115);
        g.print(full);
    }

    // Progress bar
    float p = (float)elapsed_ms / (float)deadline_ms;
    if (p > 1.0f) p = 1.0f;
    progress_bar(8, CONTENT_H + HEADER_H - 20, SCREEN_W - 16, 8, p, COL_ACCENT);

    draw_footer("-", "skip");
    flush();
}

// ── Role confirmation ──────────────────────────────────────────
// Shows the resolved role for a moment before continuing the wizard.
// User can press LEFT to force SOLO or RIGHT to confirm.
void ui_role_confirm() {
    clear();
    draw_header("ROLE");
    auto &g = gfx();

    g.setFont(&fonts::Font4);
    const char *big = "?";
    uint16_t bigc = COL_TEXT;
    switch (g_app.peer.role) {
        case ROLE_PRIMARY:   big = "PRIMARY";   bigc = COL_ACCENT; break;
        case ROLE_SECONDARY: big = "SECONDARY"; bigc = COL_WARN;   break;
        case ROLE_SOLO:      big = "SOLO";      bigc = COL_FG;     break;
        default:             big = "UNKNOWN";   bigc = COL_MUTED;  break;
    }
    g.setTextColor(bigc, COL_BG);
    int tw = g.textWidth(big);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 60);
    g.print(big);

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    const char *desc1 = "";
    const char *desc2 = "";
    switch (g_app.peer.role) {
        case ROLE_PRIMARY:
            desc1 = "This unit drives the";
            desc2 = "setup + main display.";
            break;
        case ROLE_SECONDARY:
            desc1 = "This unit streams to";
            desc2 = "the primary.";
            break;
        case ROLE_SOLO:
            desc1 = "No peer found — single";
            desc2 = "RX mode. No stereo AoA.";
            break;
        default: break;
    }
    int w1 = g.textWidth(desc1);
    int w2 = g.textWidth(desc2);
    g.setCursor((SCREEN_W - w1) / 2, CONTENT_Y + 130); g.print(desc1);
    g.setCursor((SCREEN_W - w2) / 2, CONTENT_Y + 150); g.print(desc2);

    // Show MAC last 3 bytes for both
    g.setTextColor(COL_DIM, COL_BG);
    char line[32];
    snprintf(line, sizeof(line), "me:  %02X:%02X:%02X",
             g_app.peer.own_mac[3], g_app.peer.own_mac[4], g_app.peer.own_mac[5]);
    g.setCursor(10, CONTENT_Y + 200); g.print(line);
    if (g_app.peer.peer_present) {
        snprintf(line, sizeof(line), "peer:%02X:%02X:%02X",
                 g_app.peer.peer_mac[3], g_app.peer.peer_mac[4], g_app.peer.peer_mac[5]);
        g.setCursor(10, CONTENT_Y + 218); g.print(line);
    }

    draw_footer("solo", "next");
    flush();
}

// ── RX bar assembly ────────────────────────────────────────────
// Explicit, dictated positioning: 6 cm between screens, same orientation,
// USB on same side.  Shown only when we're PRIMARY (peer exists).
void ui_rx_assembly() {
    clear();
    draw_header("MOUNT RXs");
    auto &g = gfx();

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 6);
    g.print("Two T-Displays side");
    g.setCursor(6, CONTENT_Y + 22);
    g.print("by side on a rigid bar:");

    g.setTextColor(COL_ACCENT, COL_BG);
    char line[32];
    snprintf(line, sizeof(line), " %.0f cm between screens",
             (double)STEREO_BASELINE_CM);
    g.setCursor(6, CONTENT_Y + 44); g.print(line);
    g.setCursor(6, CONTENT_Y + 60); g.print(" screens face SAME way");
    g.setCursor(6, CONTENT_Y + 76); g.print(" USB ports on SAME side");

    // Diagram: two rectangles side by side with a "6 cm" label
    int cx  = SCREEN_W / 2;
    int cy  = CONTENT_Y + 130;
    int box_w = 34, box_h = 60;
    int gap = 46;   // pixels representing 6cm on screen
    int lx = cx - gap/2 - box_w/2;
    int rx = cx + gap/2 - box_w/2;
    g.drawRect(lx, cy, box_w, box_h, COL_FG);
    g.drawRect(rx, cy, box_w, box_h, COL_FG);
    // Small mark at bottom for USB
    g.fillRect(lx + box_w/2 - 3, cy + box_h - 3, 6, 4, COL_ACCENT);
    g.fillRect(rx + box_w/2 - 3, cy + box_h - 3, 6, 4, COL_ACCENT);
    // Labels
    g.setTextColor(COL_MUTED, COL_BG);
    g.setCursor(lx + 8, cy + 20); g.print("A");
    g.setCursor(rx + 8, cy + 20); g.print("B");
    // Distance line
    g.drawLine(lx + box_w, cy + box_h/2, rx, cy + box_h/2, COL_DIM);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(cx - 10, cy + box_h/2 - 12);
    g.print("6cm");

    g.setTextColor(COL_MUTED, COL_BG);
    g.setCursor(6, CONTENT_Y + 210);
    g.print("Center the bar over");
    g.setCursor(6, CONTENT_Y + 226);
    g.print("the beacon geometry.");

    draw_footer("back", "next");
    flush();
}

// ── Secondary active ──────────────────────────────────────────
// Minimal screen shown on the SECONDARY unit while it's streaming.  Shows
// health at a glance so the operator can see it's working.
void ui_secondary_active() {
    clear();
    draw_header("SECONDARY");
    auto &g = gfx();

    // Big role indicator
    g.setFont(&fonts::Font4);
    g.setTextColor(COL_WARN, COL_BG);
    const char *big = "STREAMING";
    int tw = g.textWidth(big);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 12);
    g.print(big);

    // Rate / uptime
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);

    uint32_t now = millis();
    bool peer_ok = g_app.peer.peer_present
                   && (now - g_app.peer.last_peer_seen_ms) < PEER_TIMEOUT_MS;
    uint16_t link_col = peer_ok ? COL_FG : COL_ALERT;
    const char *link_lbl = peer_ok ? "primary OK" : "primary LOST";
    g.setTextColor(link_col, COL_BG);
    int lw = g.textWidth(link_lbl);
    g.setCursor((SCREEN_W - lw) / 2, CONTENT_Y + 60);
    g.print(link_lbl);

    // Per-beacon frame rate table
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 90);
    g.print("beacons:");

    int y = CONTENT_Y + 110;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        char line[40];
        float age = (now - b.last_frame_ms) / 1000.0f;
        snprintf(line, sizeof(line), "B%u  %lu  %.1fs",
                 (unsigned)b.id, (unsigned long)b.frames, (double)age);
        g.setTextColor(age < 0.5f ? COL_FG : COL_ALERT, COL_BG);
        g.setCursor(10, y);
        g.print(line);
        y += 14;
    }

    // Cal status
    y = CONTENT_Y + 190;
    g.setTextColor(COL_MUTED, COL_BG);
    int cal_ok = 0, cal_total = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        cal_total++;
        if (g_app.beacon[i].baseline_valid) cal_ok++;
    }
    char cline[32];
    snprintf(cline, sizeof(cline), "cal: %d/%d valid", cal_ok, cal_total);
    g.setCursor(6, y); g.print(cline);

    draw_footer("-", "-");
    flush();
}

// ══════════════════════════════════════════════════════════════
//  Dashboard sub-views: AoA and Peer
// ══════════════════════════════════════════════════════════════

// ── AoA view ──────────────────────────────────────────────────
// For each active beacon, draw a compass arc with a needle pointing at
// the current AoA estimate.  Arc opacity/brightness proportional to
// aoa_conf.  A latched-per-beacon arrow lingers for ~1 s after motion.
static void draw_view_aoa() {
    auto &g = gfx();
    // Solo/secondary have no meaningful AoA yet — say so.
    if (g_app.peer.role != ROLE_PRIMARY) {
        g.setFont(&fonts::Font2);
        g.setTextColor(COL_MUTED, COL_BG);
        const char *m1 = "AoA needs stereo";
        const char *m2 = "(PRIMARY + peer)";
        int w1 = g.textWidth(m1);
        int w2 = g.textWidth(m2);
        g.setCursor((SCREEN_W - w1) / 2, CONTENT_Y + 80); g.print(m1);
        g.setCursor((SCREEN_W - w2) / 2, CONTENT_Y + 100); g.print(m2);
        return;
    }
    if (!g_app.peer.peer_present) {
        g.setFont(&fonts::Font2);
        g.setTextColor(COL_ALERT, COL_BG);
        const char *m = "peer link LOST";
        int w = g.textWidth(m);
        g.setCursor((SCREEN_W - w) / 2, CONTENT_Y + 90); g.print(m);
        return;
    }

    // Layout: one wide compass on top, three narrow rows below (one per
    // beacon slot) showing angle + confidence bar.
    const int cx = SCREEN_W / 2;
    const int cy = CONTENT_Y + 70;
    const int R  = 60;

    // Compass background arc (top half)
    g.drawCircle(cx, cy, R,      COL_GRID);
    g.drawCircle(cx, cy, R - 20, COL_GRID_DARK);
    // Baseline (array normal points UP on screen = "forward" from user)
    g.drawLine(cx, cy - R - 4, cx, cy + 4, COL_DIM);
    g.drawLine(cx - R - 4, cy, cx + R + 4, cy, COL_DIM);
    // ±30/60/90 tick marks
    for (int deg = -90; deg <= 90; deg += 30) {
        float rad = deg * (float)M_PI / 180.0f;
        int x1 = cx + (int)((R - 4) * sinf(rad));
        int y1 = cy - (int)((R - 4) * cosf(rad));
        int x2 = cx + (int)((R + 2) * sinf(rad));
        int y2 = cy - (int)((R + 2) * cosf(rad));
        g.drawLine(x1, y1, x2, y2, COL_MUTED);
    }

    // Per-beacon needles
    uint16_t bcols[3] = {COL_MS_CH_A, COL_MS_CH_B, COL_MS_CH_C};
    int label_y = cy + R + 12;
    int col = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        if (b.aoa_conf < 0.02f) continue;
        uint16_t c = bcols[col % 3];
        // Draw needle
        float a = b.aoa_rad;
        // Clamp to ±90° for compass display
        if (a >  (float)M_PI / 2) a =  (float)M_PI / 2;
        if (a < -(float)M_PI / 2) a = -(float)M_PI / 2;
        int nx = cx + (int)((R - 8) * sinf(a));
        int ny = cy - (int)((R - 8) * cosf(a));
        g.drawLine(cx, cy, nx, ny, c);
        // Confidence dot at tip
        int rr = 2 + (int)(3.0f * b.aoa_conf);
        g.fillCircle(nx, ny, rr, c);
        col++;
    }

    // Per-beacon rows: id, angle, confidence bar
    g.setFont(&fonts::Font2);
    int y = label_y + 4;
    col = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        uint16_t c = bcols[col % 3];
        char line[32];
        float deg = b.aoa_rad * 180.0f / (float)M_PI;
        snprintf(line, sizeof(line), "B%u %+4.0fdeg", (unsigned)b.id, (double)deg);
        g.setTextColor(c, COL_BG);
        g.setCursor(6, y);
        g.print(line);
        // Confidence bar
        int bx = 92, by = y + 3;
        int bw = SCREEN_W - bx - 8;
        int bh = 8;
        progress_bar(bx, by, bw, bh, b.aoa_conf, c);
        y += 16;
        col++;
    }

    // LO drift readout at very bottom
    g.setTextColor(COL_DIM, COL_BG);
    char ld[32];
    snprintf(ld, sizeof(ld), "LO drift %+5.2f rad",
             (double)g_app.peer.lo_drift_ema);
    g.setCursor(6, CONTENT_H + HEADER_H - 30);
    g.print(ld);
}

// ── Peer view ─────────────────────────────────────────────────
// Diagnostic screen: role, peer MAC, packet counts, LO drift, ages.
static void draw_view_peer() {
    auto &g = gfx();
    g.setFont(&fonts::Font2);

    int y = CONTENT_Y + 4;
    // Role
    const char *role = "?";
    uint16_t rc = COL_MUTED;
    switch (g_app.peer.role) {
        case ROLE_PRIMARY:   role = "PRIMARY";   rc = COL_ACCENT; break;
        case ROLE_SECONDARY: role = "SECONDARY"; rc = COL_WARN;   break;
        case ROLE_SOLO:      role = "SOLO";      rc = COL_FG;     break;
        default: break;
    }
    g.setTextColor(COL_TEXT, COL_BG); g.setCursor(6, y); g.print("role: ");
    g.setTextColor(rc, COL_BG);       g.print(role);
    y += 16;

    // Own MAC (last 3)
    char line[40];
    snprintf(line, sizeof(line), "self %02X:%02X:%02X",
             g_app.peer.own_mac[3], g_app.peer.own_mac[4], g_app.peer.own_mac[5]);
    g.setTextColor(COL_MUTED, COL_BG); g.setCursor(6, y); g.print(line); y += 14;

    // Peer MAC
    if (g_app.peer.peer_present) {
        snprintf(line, sizeof(line), "peer %02X:%02X:%02X",
                 g_app.peer.peer_mac[3], g_app.peer.peer_mac[4], g_app.peer.peer_mac[5]);
        g.setTextColor(COL_FG, COL_BG);
    } else {
        snprintf(line, sizeof(line), "peer LOST");
        g.setTextColor(COL_ALERT, COL_BG);
    }
    g.setCursor(6, y); g.print(line); y += 20;

    // Packet counters (PRIMARY perspective)
    g.setTextColor(COL_TEXT, COL_BG);
    snprintf(line, sizeof(line), "rx summ  %lu", (unsigned long)g_app.peer.peer_frames_rx);
    g.setCursor(6, y); g.print(line); y += 14;
    snprintf(line, sizeof(line), "pairs    %lu", (unsigned long)g_app.peer.peer_pairs_ok);
    g.setCursor(6, y); g.print(line); y += 14;
    snprintf(line, sizeof(line), "dropped  %lu", (unsigned long)g_app.peer.peer_frames_dropped);
    g.setTextColor(g_app.peer.peer_frames_dropped > 20 ? COL_WARN : COL_MUTED, COL_BG);
    g.setCursor(6, y); g.print(line); y += 20;

    // Pair success rate
    uint32_t tot = g_app.peer.peer_pairs_ok + g_app.peer.peer_frames_dropped;
    float rate = tot ? (100.0f * g_app.peer.peer_pairs_ok / tot) : 0;
    snprintf(line, sizeof(line), "success  %.0f%%", (double)rate);
    g.setTextColor(rate > 80 ? COL_FG : (rate > 40 ? COL_WARN : COL_ALERT), COL_BG);
    g.setCursor(6, y); g.print(line); y += 14;

    // LO drift
    snprintf(line, sizeof(line), "LO ema   %+.3f", (double)g_app.peer.lo_drift_ema);
    g.setTextColor(COL_ACCENT, COL_BG); g.setCursor(6, y); g.print(line); y += 20;

    // Last pair age
    uint32_t age = millis() - g_app.peer.last_pair_ms;
    snprintf(line, sizeof(line), "last pair %lums",
             (unsigned long)(g_app.peer.last_pair_ms ? age : 0));
    g.setTextColor(age < 500 ? COL_FG : COL_MUTED, COL_BG);
    g.setCursor(6, y); g.print(line); y += 14;

    // Per-beacon disparity baseline status
    g.setTextColor(COL_DIM, COL_BG);
    g.setCursor(6, y); g.print("phase base:"); y += 14;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        snprintf(line, sizeof(line), " B%u %s", (unsigned)b.id,
                 b.phase_baseline_valid ? "OK" : "--");
        g.setTextColor(b.phase_baseline_valid ? COL_FG : COL_MUTED, COL_BG);
        g.setCursor(10, y); g.print(line); y += 12;
    }
}

// ═══════════════════════════════════════════════════════════════
//  v0.3 CAL CEREMONY SCREENS
//
//  Six one-shot screens that walk the user through the new
//  landmark+transit calibration ceremony.  ui_cal_landmark_walk
//  reads live wizard + scene state and re-renders every frame; the
//  others are static enough to redraw only when should_redraw fires.
// ═══════════════════════════════════════════════════════════════

// Helper: draw a multiline instruction block, honoring '\n'.
static void draw_multiline(int x, int y, int line_h, uint16_t col,
                           const char *text) {
    auto &g = gfx();
    g.setTextColor(col, COL_BG);
    if (!text) return;
    const char *p = text;
    int cy = y;
    char buf[64];
    int bi = 0;
    while (*p) {
        if (*p == '\n' || bi >= (int)sizeof(buf) - 1) {
            buf[bi] = 0;
            g.setCursor(x, cy);
            g.print(buf);
            cy += line_h;
            bi = 0;
            if (*p == '\n') p++;
            continue;
        }
        buf[bi++] = *p++;
    }
    if (bi > 0) { buf[bi] = 0; g.setCursor(x, cy); g.print(buf); }
}

// Draw a small map (radar mini-view) into a rect showing beacon
// triangle + RX + a highlighted landmark position.  Used inside
// ui_cal_landmark_walk to help the user visualize where to walk.
static void draw_mini_landmark_map(int box_x, int box_y, int box_w, int box_h,
                                   LandmarkId highlight,
                                   LandmarkId next) {
    auto &g = gfx();
    g.drawRect(box_x, box_y, box_w, box_h, COL_MS_DIM);
    int cx = box_x + box_w / 2;
    int cy = box_y + box_h / 2;
    int R  = (box_w < box_h ? box_w : box_h) / 2 - 6;
    float pix_per_unit = (float)R / SCENE_EXTENT;
    // Rings
    for (float r = 0.5f; r <= SCENE_EXTENT; r += 0.5f) {
        g.drawCircle(cx, cy, (int)(r * pix_per_unit), COL_MS_DIM);
    }
    // Beacons
    struct { LandmarkId id; } bs[3] = {{LM_BEACON_1}, {LM_BEACON_2}, {LM_BEACON_3}};
    for (int i = 0; i < 3; i++) {
        float bx, by; scene_landmark_pos(bs[i].id, &bx, &by);
        int px = cx + (int)(bx * pix_per_unit);
        int py = cy - (int)(by * pix_per_unit);
        g.fillCircle(px, py, 2, COL_MS_LIME);
    }
    // RX
    g.fillRect(cx - 2, cy - 2, 4, 4, COL_MS_TEAL);
    // Highlighted (current) landmark - bright ring
    if (highlight < LM_COUNT) {
        float hx, hy; scene_landmark_pos(highlight, &hx, &hy);
        int px = cx + (int)(hx * pix_per_unit);
        int py = cy - (int)(hy * pix_per_unit);
        g.drawCircle(px, py, 4, COL_MS_WARN);
        g.drawCircle(px, py, 5, COL_MS_WARN);
    }
    // Next landmark - arrow from highlight to next
    if (next < LM_COUNT && highlight != next) {
        float hx, hy; scene_landmark_pos(highlight, &hx, &hy);
        float nx, ny; scene_landmark_pos(next, &nx, &ny);
        int px1 = cx + (int)(hx * pix_per_unit);
        int py1 = cy - (int)(hy * pix_per_unit);
        int px2 = cx + (int)(nx * pix_per_unit);
        int py2 = cy - (int)(ny * pix_per_unit);
        g.drawLine(px1, py1, px2, py2, COL_MS_INK);
        // Small arrowhead at target end
        g.fillCircle(px2, py2, 2, COL_MS_INK);
    }
}

// ── ui_cal_intro ───────────────────────────────────────────────
// Explains the walk ceremony before it begins.  Different copy for
// stereo vs solo.
void ui_cal_intro() {
    clear();
    draw_header("CAL WALK");
    auto &g = gfx();

    // Big title
    g.setFont(&fonts::Font4);
    g.setTextColor(COL_MS_LIME, COL_BG);
    const char *t = (g_app.cal_mode == CAL_MODE_STEREO) ? "STEREO" : "SOLO";
    int tw = g.textWidth(t);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 8);
    g.print(t);

    g.setFont(&fonts::Font2);
    const char *body_stereo =
        "You will walk\n"
        "a scripted path\n"
        "through the room\n"
        "to teach the model\n"
        "the room's radio\n"
        "response.\n"
        "\n"
        "Carry PROBE.\n"
        "ANCHOR stays put.\n"
        "~100 seconds.";
    const char *body_solo =
        "Hold the T-Display\n"
        "against your chest\n"
        "for the walk.\n"
        "\n"
        "Solo cal is a\n"
        "degraded fallback.\n"
        "Two receivers give\n"
        "much better results.\n"
        "\n"
        "~90 seconds.";
    draw_multiline(6, CONTENT_Y + 44, 14, COL_MS_INK,
                   (g_app.cal_mode == CAL_MODE_STEREO) ? body_stereo : body_solo);

    draw_footer("cancel", "begin");
    flush();
}

// ── ui_cal_anchor_place (STEREO only) ─────────────────────────
// User is instructed to place the ANCHOR unit at the geometric
// center of the beacon triangle.  Only PROBE runs this screen —
// ANCHOR shows a "I am ANCHOR, don't move me" status.
void ui_cal_anchor_place() {
    clear();
    draw_header("PLACE ANCHOR");
    auto &g = gfx();

    const bool is_probe = (g_app.peer.cal_role == CAL_ROLE_PROBE);
    if (is_probe) {
        g.setFont(&fonts::Font4);
        g.setTextColor(COL_MS_LIME, COL_BG);
        const char *t = "PROBE";
        int tw = g.textWidth(t);
        g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 4);
        g.print(t);

        g.setFont(&fonts::Font2);
        draw_multiline(6, CONTENT_Y + 36, 14, COL_MS_INK,
            "1. Take ANCHOR\n"
            "   (the other unit)\n"
            "2. Place it at the\n"
            "   center of the\n"
            "   beacon triangle\n"
            "3. Leave it there\n"
            "4. Come back here\n"
            "   with THIS unit");
        // Illustration: small triangle with a dot at center
        int cx = SCREEN_W / 2, cy = CONTENT_Y + 190, R = 30;
        for (int i = 0; i < 3; i++) {
            float a1 = (i * 2.0f * (float)M_PI / 3.0f) - (float)M_PI / 2.0f;
            float a2 = ((i + 1) % 3 * 2.0f * (float)M_PI / 3.0f) - (float)M_PI / 2.0f;
            int x1 = cx + (int)(R * cosf(a1)), y1 = cy + (int)(R * sinf(a1));
            int x2 = cx + (int)(R * cosf(a2)), y2 = cy + (int)(R * sinf(a2));
            g.drawLine(x1, y1, x2, y2, COL_MS_DIM);
            g.fillCircle(x1, y1, 3, COL_MS_LIME);
        }
        g.fillRect(cx - 3, cy - 3, 6, 6, COL_MS_TEAL);   // ANCHOR at center
        draw_footer("back", "placed");
    } else {
        // ANCHOR side: passive status
        g.setFont(&fonts::Font4);
        g.setTextColor(COL_MS_TEAL, COL_BG);
        int tw = g.textWidth("ANCHOR");
        g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 30);
        g.print("ANCHOR");
        g.setFont(&fonts::Font2);
        g.setTextColor(COL_MS_INK, COL_BG);
        draw_multiline(6, CONTENT_Y + 80, 14, COL_MS_INK,
            "Place me at\n"
            "the center of\n"
            "the beacon\n"
            "triangle.\n"
            "\n"
            "Then don't move\n"
            "me until cal\n"
            "is complete.");
        draw_footer("", "");
    }
    flush();
}

// ── ui_cal_empty_room ─────────────────────────────────────────
// User is asked to leave the room (with PROBE if stereo) so we
// can capture the empty-room baseline.  Uses the existing baseline
// capture UI feel, plus a countdown and progress ring.
void ui_cal_empty_room(uint32_t elapsed_ms) {
    clear();
    draw_header("EMPTY ROOM");
    auto &g = gfx();
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MS_INK, COL_BG);
    draw_multiline(6, CONTENT_Y + 8, 14, COL_MS_INK,
        "Leave the room\n"
        "with the PROBE.\n"
        "\n"
        "ANCHOR is\n"
        "learning what\n"
        "the room looks\n"
        "like empty.");

    // Big countdown / progress
    float p = csi_baseline_progress();
    if (p > 1.0f) p = 1.0f;
    progress_bar(10, CONTENT_Y + 130, SCREEN_W - 20, 16, p, COL_MS_LIME);

    g.setFont(&fonts::Font4);
    g.setTextColor(COL_MS_LIME, COL_BG);
    char buf[16];
    snprintf(buf, sizeof(buf), "%d%%", (int)(p * 100));
    int tw = g.textWidth(buf);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 160);
    g.print(buf);

    // Also show seconds since we entered this screen
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MS_MID, COL_BG);
    snprintf(buf, sizeof(buf), "%lus", (unsigned long)(elapsed_ms / 1000));
    g.setCursor(4, CONTENT_Y + CONTENT_H - 20);
    g.print(buf);

    draw_footer("", "cancel");
    flush();
}

// ── ui_cal_landmark_walk ──────────────────────────────────────
// The star of the cal ceremony.  Reads live wizard state each
// frame and renders the current step:
//   - Big instruction text from the WizardStep
//   - Mini map showing current landmark + next landmark arrow
//   - Overall progress bar across the whole script
//   - Per-step timer (min_duration countdown when relevant)
void ui_cal_landmark_walk() {
    clear();
    draw_header("CAL WALK");
    auto &g = gfx();

    const WizardStep *step = wizard_current_step();
    int cur = wizard_current_index();
    int total = wizard_total_steps();
    float overall = wizard_overall_progress();

    // Overall progress bar right below header
    progress_bar(4, CONTENT_Y + 2, SCREEN_W - 8, 6, overall, COL_MS_TEAL);
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MS_MID, COL_BG);
    char sbuf[16];
    snprintf(sbuf, sizeof(sbuf), "%d/%d", cur + 1, total);
    int sw = g.textWidth(sbuf);
    g.setCursor(SCREEN_W - sw - 4, CONTENT_Y + 12);
    g.print(sbuf);

    if (!step) {
        g.setTextColor(COL_MS_WARN, COL_BG);
        g.setCursor(6, CONTENT_Y + 30);
        g.print("(no step)");
        draw_footer("redo", "next");
        flush();
        return;
    }

    // Step title (big)
    g.setFont(&fonts::Font4);
    uint16_t title_col = COL_MS_LIME;
    if (step->kind == STEP_WALK)   title_col = COL_MS_TEAL;
    if (step->kind == STEP_ROTATE) title_col = COL_MS_WARN;
    g.setTextColor(title_col, COL_BG);
    int tw = g.textWidth(step->title);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 24);
    g.print(step->title);

    // Step instruction body
    g.setFont(&fonts::Font2);
    draw_multiline(6, CONTENT_Y + 56, 14, COL_MS_INK, step->instruction);

    // Mini landmark map (bottom half)
    LandmarkId highlight = step->landmark_a;
    LandmarkId next = (step->kind == STEP_WALK) ? step->landmark_b : step->landmark_a;
    // Look ahead one step for the "next" arrow if we're standing
    if (step->kind == STEP_STAND && cur + 1 < total) {
        const WizardStep *next_step = &step[1];
        if (next_step->kind == STEP_WALK) next = next_step->landmark_b;
        else if (next_step->kind != STEP_END) next = next_step->landmark_a;
    }
    int map_y = CONTENT_Y + 154;
    int map_h = 96;
    draw_mini_landmark_map(4, map_y, SCREEN_W - 8, map_h, highlight, next);

    // Timer / status line right above footer
    uint32_t elapsed = wizard_step_elapsed_ms();
    uint32_t min_rem = wizard_step_min_remaining_ms();
    char tbuf[24];
    if (min_rem > 0) {
        snprintf(tbuf, sizeof(tbuf), "hold %.1fs", (double)min_rem / 1000.0);
        g.setTextColor(COL_MS_WARN, COL_BG);
    } else {
        snprintf(tbuf, sizeof(tbuf), "ready %.1fs", (double)elapsed / 1000.0);
        g.setTextColor(COL_MS_LIME, COL_BG);
    }
    g.setFont(&fonts::Font2);
    int ttw = g.textWidth(tbuf);
    g.setCursor((SCREEN_W - ttw) / 2, map_y + map_h + 2);
    g.print(tbuf);

    const char *rlabel = (min_rem > 0) ? "wait" : "next";
    if (step->kind == STEP_INTRO) rlabel = "begin";
    draw_footer("redo", rlabel);
    flush();
}

// ── ui_cal_finalize ────────────────────────────────────────────
// Shown briefly while scene_finalize_cal() crunches numbers.
// Progress is faked (the actual finalize is a one-shot; we just
// show a "training your model..." moment for user feedback).
void ui_cal_finalize(float progress) {
    clear();
    draw_header("TRAINING");
    auto &g = gfx();

    g.setFont(&fonts::Font4);
    g.setTextColor(COL_MS_LIME, COL_BG);
    const char *t = "MODEL";
    int tw = g.textWidth(t);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 30);
    g.print(t);

    g.setFont(&fonts::Font2);
    draw_multiline(6, CONTENT_Y + 70, 14, COL_MS_INK,
        "Building kernel...\n"
        "Fitting per-beacon\n"
        "response weights...\n"
        "Cross-validating\n"
        "landmark predictions...\n"
        "Checking loop\n"
        "closure...");

    progress_bar(10, CONTENT_Y + 190, SCREEN_W - 20, 16, progress, COL_MS_TEAL);
    // Little counter
    char buf[16];
    snprintf(buf, sizeof(buf), "%d%%", (int)(progress * 100));
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MS_MID, COL_BG);
    int bw = g.textWidth(buf);
    g.setCursor((SCREEN_W - bw) / 2, CONTENT_Y + 214);
    g.print(buf);

    // Kernel sample count as a diagnostic
    snprintf(buf, sizeof(buf), "N=%d", scene_kernel_sample_count());
    g.setCursor(4, CONTENT_Y + CONTENT_H - 20);
    g.print(buf);

    draw_footer("", "");
    flush();
}

// ── ui_cal_results ─────────────────────────────────────────────
// Post-cal quality report.  Shows CalReport in a compact table
// with green/amber/red coloring for interpreted quality.  User can
// accept (RIGHT) or redo (LEFT).
//
// The report is fetched fresh each render from scene_get_last_report()
// — but we don't have that accessor.  Instead, we cache a copy at
// the moment of finalize.  For simplicity of this initial cut, we
// read the pieces that are cheap to re-derive.
static CalReport s_last_report = {};
void ui_cal_stash_report(const CalReport &r) { s_last_report = r; }

void ui_cal_results() {
    clear();
    draw_header("CAL RESULT");
    auto &g = gfx();

    g.setFont(&fonts::Font4);
    const CalReport &r = s_last_report;
    // Verdict now factors in cross-val, loop closure, observability,
    // and alias count.  All four must be reasonable for OK.
    bool xval_ok  = r.cross_val_error < 0.35f;
    bool loop_ok  = r.loop_closure_error < 0.30f;
    bool obs_ok   = r.mean_observability > 0.05f;
    bool alias_ok = r.alias_pair_count == 0;
    bool good = r.valid && xval_ok && loop_ok && obs_ok && alias_ok;
    bool marginal = r.valid && (xval_ok || loop_ok);
    const char *verdict = good     ? "OK"
                        : marginal ? "MARGINAL"
                        : r.valid  ? "POOR"
                                   : "FAILED";
    uint16_t vcol = good     ? COL_MS_LIME
                  : marginal ? COL_MS_WARN
                             : COL_MS_ALERT;
    g.setTextColor(vcol, COL_MS_BG);
    int tw = g.textWidth(verdict);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 4);
    g.print(verdict);

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MS_INK, COL_MS_BG);
    int y = CONTENT_Y + 40;
    char line[40];

    snprintf(line, sizeof(line), "mode  : %s",
             r.mode == CAL_MODE_STEREO ? "STEREO" : "SOLO");
    g.setCursor(6, y); g.print(line); y += 14;

    snprintf(line, sizeof(line), "kernel: %u samples", (unsigned)r.total_kernel_samples);
    g.setCursor(6, y); g.print(line); y += 14;

    snprintf(line, sizeof(line), "lm/tr : %u/%u",
             (unsigned)r.landmarks_captured, (unsigned)r.transit_samples);
    g.setCursor(6, y); g.print(line); y += 14;

    // Cross-val error
    uint16_t ccol = xval_ok
                  ? (r.cross_val_error < 0.2f ? COL_MS_LIME : COL_MS_TEAL_BRIGHT)
                  : COL_MS_WARN;
    g.setTextColor(ccol, COL_MS_BG);
    snprintf(line, sizeof(line), "xval  : %.3f", (double)r.cross_val_error);
    g.setCursor(6, y); g.print(line); y += 14;

    // Loop closure
    uint16_t lcol = loop_ok
                  ? (r.loop_closure_error < 0.15f ? COL_MS_LIME : COL_MS_TEAL_BRIGHT)
                  : COL_MS_WARN;
    g.setTextColor(lcol, COL_MS_BG);
    snprintf(line, sizeof(line), "loop  : %.3f", (double)r.loop_closure_error);
    g.setCursor(6, y); g.print(line); y += 14;

    // Observability — how sensitive the room's RF field is to position changes
    uint16_t ocol = r.mean_observability > 0.15f ? COL_MS_LIME
                  : r.mean_observability > 0.05f ? COL_MS_TEAL_BRIGHT
                                                  : COL_MS_WARN;
    g.setTextColor(ocol, COL_MS_BG);
    snprintf(line, sizeof(line), "obs   : %.3f", (double)r.mean_observability);
    g.setCursor(6, y); g.print(line); y += 14;

    // Alias pairs
    uint16_t acol = r.alias_pair_count == 0 ? COL_MS_LIME
                  : r.alias_pair_count < 3  ? COL_MS_WARN
                                             : COL_MS_ALERT;
    g.setTextColor(acol, COL_MS_BG);
    snprintf(line, sizeof(line), "alias : %u pairs", (unsigned)r.alias_pair_count);
    g.setCursor(6, y); g.print(line); y += 16;

    // Per-beacon effective weight = SNR × orientation reliability
    g.setTextColor(COL_MS_MID, COL_MS_BG);
    g.setCursor(6, y); g.print("beacon wt / orient:"); y += 12;
    for (int b = 0; b < MAX_BEACONS; b++) {
        if (!g_app.beacon[b].active) continue;
        float s   = r.per_beacon_snr[b];
        float rel = r.per_beacon_orient_reliability[b];
        uint16_t sc = s > 1.0f ? COL_MS_LIME
                    : s > 0.3f ? COL_MS_TEAL_BRIGHT : COL_MS_WARN;
        g.setTextColor(sc, COL_MS_BG);
        snprintf(line, sizeof(line), " B%u %.2f (r=%.2f)",
                 (unsigned)g_app.beacon[b].id, (double)s, (double)rel);
        g.setCursor(10, y); g.print(line); y += 12;
    }

    // Worst landmark (if any)
    if (r.worst_landmark_error > 0.3f) {
        g.setTextColor(COL_MS_WARN, COL_MS_BG);
        snprintf(line, sizeof(line), "worst lm: %u (%.2f)",
                 (unsigned)r.worst_landmark, (double)r.worst_landmark_error);
        g.setCursor(6, y); g.print(line); y += 14;
    }

    // v0.5: geometry-validation warnings (PROBE at beacon lm should
    // observe that beacon strongest).  If any bit set, tell the user
    // which beacon(s) might be mispositioned or misidentified.
    if (r.geometry_validation_fail_mask) {
        g.setTextColor(COL_MS_ALERT, COL_MS_BG);
        snprintf(line, sizeof(line), "geom warn: 0x%02X",
                 (unsigned)r.geometry_validation_fail_mask);
        g.setCursor(6, y); g.print(line); y += 12;
    }

    // v0.5: alias-pair preview (first pair only — space-limited screen)
    if (r.alias_pair_count > 0 && y < SCREEN_H - FOOTER_H - 12) {
        g.setTextColor(COL_MS_WARN, COL_MS_BG);
        snprintf(line, sizeof(line), "alias: lm%u↔lm%u",
                 (unsigned)r.alias_pairs[0].lm_a,
                 (unsigned)r.alias_pairs[0].lm_b);
        g.setCursor(6, y); g.print(line); y += 12;
    }

    draw_footer("redo", good ? "accept" : "accept?");
    flush();
}
