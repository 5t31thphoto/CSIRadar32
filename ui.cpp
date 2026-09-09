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
        // The `else` here used to bind to the LS_PRESENCE test, so a
        // beacon in LS_MOTION had its amber overwritten by lime as soon
        // as baseline_valid went true -- motion never showed in the
        // header. Explicit chain.
        uint16_t c = COL_DIM;
        if      (g_app.beacon[i].status == LS_PRESENCE) c = COL_ALERT;
        else if (g_app.beacon[i].status == LS_MOTION)   c = COL_WARN;
        else if (g_app.beacon[i].baseline_valid)        c = COL_FG;
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

    // Full-screen off-screen sprite for flicker-free rendering.
    //
    // Deliberately INTERNAL DRAM.  This is 106 KB, the largest single
    // allocation in the firmware, and it is also the one thing that must
    // work on every boot -- so it does not get placed behind PSRAM, which
    // has never successfully initialized on this hardware across many
    // attempts.  The v0.85 static-memory reductions (capture decimator,
    // exact top-K percentile, right-sized kernel) recovered ~37 KB of
    // headroom, which is what makes MAX_BEACONS=6 fit here WITHOUT PSRAM.
    // Do not "optimize" this into SPIRAM.
    g_canvas.setColorDepth(16);
    g_canvas_ok = g_canvas.createSprite(SCREEN_W, SCREEN_H);
    if (!g_canvas_ok) {
        // Insufficient memory — fall back to direct-draw. Not ideal, but works.
        g_lcd.setCursor(4, 4);
        g_lcd.setTextColor(COL_WARN);
        g_lcd.print("[ui] no sprite RAM");
    }
    // ── Read-only report.  Reports what IS; nothing depends on it. ──
    MSLOG("[ui] canvas=%s  free heap %u (min ever %u)\n",
          g_canvas_ok ? "sprite" : "DIRECT-DRAW",
          (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
    const PsramProbe &pp = psram_probe();
    MSLOG("[psram] %s  size=%u  %s\n",
          pp.status == PSRAM_OK ? "OK" :
          pp.status == PSRAM_NOT_COMPILED ? "NOT COMPILED" : "NO RESPONSE",
          (unsigned)pp.size_bytes, pp.hint);
}

// Evaluated once, at first call.  PSRAM is enabled in the build but used
// by nothing -- this exists purely to answer "did the experiment work",
// and to say WHY when it did not, because the two failure modes need
// completely different fixes.
const PsramProbe &psram_probe() {
    static PsramProbe p;
    static bool done = false;
    if (done) return p;
    done = true;
#if !defined(BOARD_HAS_PSRAM)
    p.status = PSRAM_NOT_COMPILED;
    p.size_bytes = 0;
    p.hint = "build flag missing: fqbn needs PSRAM=opi";
#else
    if (psramFound()) {
        p.status = PSRAM_OK;
        p.size_bytes = (uint32_t)ESP.getPsramSize();
        p.hint = "available for future use";
    } else {
        p.status = PSRAM_NO_RESPONSE;
        p.size_bytes = 0;
        // Compiled in and still silent: the flag reached the build, so
        // this is the chip/mode side.  T-Display-S3 is an S3R8 (octal),
        // and octal PSRAM only initializes with QIO flash -- a mismatch
        // here is the usual cause.
        p.hint = "enabled but chip silent: check OPI vs QUAD, or FlashMode=qio";
    }
#endif
    return p;
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
    // elapsed is uint32_t: (elapsed - 800) underflows to ~4.29e9 for the
    // first 800 ms, which the clamp below turns into 1.0 -- so the
    // wordmark appeared instantly instead of after the mantis. Same for
    // the subtitle. Signed arithmetic before the divide.
    float reveal_word   = ((float)((int32_t)elapsed - 800)) / 1200.0f;
    if (reveal_word < 0) reveal_word = 0;
    if (reveal_word > 1) reveal_word = 1;
    float reveal_sub    = ((float)((int32_t)elapsed - 1800)) / 600.0f;
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
        // No button does anything on this screen -- it is not skippable
        // -- so the prompt must not name one.  It previously read
        // "> hold RIGHT to arm", which was wrong on both counts: RIGHT
        // did not arm, it skipped, and the screen advanced on its own
        // anyway.  Show what is actually happening instead.
        uint32_t p = (millis() / 400) % 2;
        g.setTextColor(p ? COL_MS_LIME : COL_MS_VIOLET_BRIGHT, COL_MS_BG);
        const char *prompt = "> starting up";
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

    // v0.9: PSRAM probe result.  Serial is compiled out on this build, so
    // the only place this can surface is the screen.  Discovery is the
    // right place: it is on screen for seconds at every boot, before the
    // user is asked to do anything.  Nothing depends on PSRAM -- this
    // reports whether the safe experiment worked, and why if it didn't.
    {
        // Firmware version, first thing on this screen.  Without it there
        // is no way to tell which build is actually flashed -- and a
        // failed CI job silently leaves the previous build on the
        // flasher, which looks exactly like "the new code does nothing".
        g.setFont(&fonts::Font0);
        g.setTextColor(COL_ACCENT, COL_BG);
        g.setCursor(6, CONTENT_Y + 32);
        g.print("fw " FW_VERSION);

        const PsramProbe &pp = psram_probe();
        char ln[40];
        uint16_t c;
        if (pp.status == PSRAM_OK) {
            snprintf(ln, sizeof(ln), "PSRAM ok  %u MB",
                     (unsigned)(pp.size_bytes / (1024u * 1024u)));
            c = COL_FG;
        } else if (pp.status == PSRAM_NOT_COMPILED) {
            snprintf(ln, sizeof(ln), "PSRAM off (not built in)");
            c = COL_MUTED;
        } else {
            snprintf(ln, sizeof(ln), "PSRAM FAIL (chip silent)");
            c = COL_WARN;
        }
        g.setFont(&fonts::Font0);
        g.setTextColor(c, COL_BG);
        g.setCursor(6, CONTENT_Y + 42);
        g.print(ln);
        // The reason matters more than the fact -- the two failure modes
        // need completely different fixes.
        if (pp.status != PSRAM_OK) {
            g.setTextColor(COL_MUTED, COL_BG);
            g.setCursor(6, CONTENT_Y + 52);
            g.print(pp.hint);
        }
        g.setFont(&fonts::Font2);
    }

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

    // v0.9: say what mode this count will produce, and flag a short set.
    // A silent downgrade to tripwire because two beacons were asleep is
    // the failure this is meant to make impossible to miss.
    {
        g.setFont(&fonts::Font2);
        char m[36];
        if (found >= 3)      snprintf(m, sizeof(m), "-> %d beacons, full", found);
        else if (found == 2) snprintf(m, sizeof(m), "-> 2: LINE mode only");
        else if (found == 1) snprintf(m, sizeof(m), "-> 1: TRIPWIRE only");
        else                 snprintf(m, sizeof(m), "-> none yet");
        g.setTextColor(found >= 3 ? COL_FG : COL_WARN, COL_BG);
        g.setCursor(6, SCREEN_H - FOOTER_H - 34);
        g.print(m);
        if (found > 0 && found < 3) {
            g.setTextColor(COL_MUTED, COL_BG);
            g.setCursor(6, SCREEN_H - FOOTER_H - 18);
            g.print("press to accept");
        }
    }

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
            // v0.8: RM_TRIANGLE_3 now covers 3..6 beacons.  The 3-beacon
            // branch below is the v0.7 drawing VERBATIM so an existing
            // install's setup screen is pixel-identical; the N-gon branch
            // is only reached at 4+ beacons, which v0.7 could not do.
            const int n_disc = g_app.beacon_count;
            if (n_disc <= 3) {
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

            // ── 4..6 beacons: regular N-gon ──
            const int n = (n_disc > MAX_BEACONS) ? MAX_BEACONS : n_disc;
            char l0[28];
            snprintf(l0, sizeof(l0), "%d beacons detected.", n);
            g.print(l0);
            g.setCursor(6, CONTENT_Y + 26);
            g.setTextColor(COL_MUTED, COL_BG);
            char l1[32];
            snprintf(l1, sizeof(l1), "Space evenly in a ring,");
            g.print(l1);
            g.setCursor(6, CONTENT_Y + 42);
            snprintf(l1, sizeof(l1), "%d roughly equal gaps.", n);
            g.print(l1);
            g.setCursor(6, CONTENT_Y + 58);
            g.print("T-Display goes in the");
            g.setCursor(6, CONTENT_Y + 74);
            g.print("centre, facing up.");

            // Vertices from the same formula csi_assign_default_geometry
            // uses, so the picture always matches the geometry actually
            // assigned: first vertex at the top, then counter-clockwise.
            int vx[MAX_BEACONS], vy[MAX_BEACONS];
            for (int i = 0; i < n; i++) {
                float th = (float)M_PI / 2.0f
                         + (2.0f * (float)M_PI * (float)i) / (float)n;
                vx[i] = cx + (int)(R * cosf(th));
                vy[i] = cy - (int)(R * sinf(th));
            }
            for (int i = 0; i < n; i++) {
                int j = (i + 1) % n;
                g.drawLine(vx[i], vy[i], vx[j], vy[j], COL_DIM);
            }
            for (int i = 0; i < n; i++) {
                char lb[6];
                snprintf(lb, sizeof(lb), "B%u",
                         (unsigned)(g_app.beacon[i].active ? g_app.beacon[i].id : 0));
                g.fillCircle(vx[i], vy[i], 6, COL_FG);
                g.setTextColor(COL_FG);
                // Nudge each label outward from the centre so it never
                // lands on its own dot or a polygon edge.
                g.setCursor(vx[i] + (vx[i] - cx) / 5 - 6,
                            vy[i] + (vy[i] - cy) / 5 - 6);
                g.print(lb);
            }
            g.fillRect(cx - 6, cy - 4, 12, 20, COL_ACCENT);
            g.setTextColor(COL_ACCENT); g.setCursor(cx + 10, cy); g.print("YOU");
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

    // v0.4: RX-side beacon rate inference.  Averages the inter-arrival
    // EMA across active beacons, converts to Hz, picks a mode label:
    //   >30Hz → "50Hz"           (beacons in normal fast mode)
    //   4-15Hz → "5Hz+sleep"     (beacons in extended feature envelope)
    //   otherwise → raw Hz shown so the user sees what's actually happening
    int n_active = 0;
    float sum_ms = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        const BeaconState &b = g_app.beacon[i];
        if (b.active && b.inter_arrival_ms_ema > 0) {
            sum_ms += b.inter_arrival_ms_ema;
            n_active++;
        }
    }
    char rbuf[24];
    if (n_active > 0) {
        float avg_ms = sum_ms / n_active;
        float hz     = 1000.0f / avg_ms;
        const char *tag;
        uint16_t rcol;
        if (hz >= 30.0f)       { tag = "50Hz";        rcol = COL_ACCENT; }
        else if (hz >= 4.0f && hz <= 15.0f)
                               { tag = "5Hz+sleep";   rcol = COL_MS_LIME; }
        else                   { tag = nullptr;       rcol = COL_MUTED; }
        if (tag) snprintf(rbuf, sizeof(rbuf), "%dB %s", n_active, tag);
        else     snprintf(rbuf, sizeof(rbuf), "%dB %.0fHz", n_active, (double)hz);
        g.setTextColor(rcol, COL_BG);
        int mw = g.textWidth(mode_lbl);
        g.setCursor(4 + mw + 8, status_y);
        g.print(rbuf);
    }

    // Confidence % — pulled from primary scene track (was g_app.tracker,
    // now owned by scene module).  Zero if no active track.
    const TargetTrack *tr0 = scene_get_track(0);
    float track_conf = (tr0 && tr0->active) ? tr0->confidence : 0.0f;
    char cbuf[16];
    snprintf(cbuf, sizeof(cbuf), "%3d%%", (int)(track_conf * 100.0f));
    int cw = g.textWidth(cbuf);
    uint16_t cc = track_conf > 0.5f ? COL_FG
                : track_conf > 0.2f ? COL_WARN : COL_MUTED;
    g.setTextColor(cc, COL_BG);
    g.setCursor(SCREEN_W - cw - 4, status_y);
    g.print(cbuf);

    // ── Radar map ──
    // Room extent in cm — used for beacon coords (which are stored in cm)
    // and for scaling normalized scene positions to display pixels.
    const float ROOM_HALF_CM = 200.0f;   // 4m × 4m room
    const int cx = SCREEN_W / 2;
    const int cy = map_y + map_h / 2;
    const int usable = (SCREEN_W / 2) - 6;
    const int usable_v = (map_h / 2) - 6;
    const int R = (usable < usable_v) ? usable : usable_v;
    const float pix_per_cm = (float)R / ROOM_HALF_CM;
    // Scene module works in normalized units (±SCENE_EXTENT = room bounds).
    // Convert to cm on read: cm = norm * ROOM_HALF_CM / SCENE_EXTENT.
    const float norm_to_cm = ROOM_HALF_CM / SCENE_EXTENT;

    g.drawRect(0, map_y, SCREEN_W, map_h, COL_GRID_DARK);
    for (int r_cm = 100; r_cm <= (int)ROOM_HALF_CM; r_cm += 100) {
        int rp = (int)(r_cm * pix_per_cm);
        g.drawCircle(cx, cy, rp, COL_GRID_DARK);
    }
    g.drawFastHLine(cx - R, cy, 2 * R, COL_GRID_DARK);
    g.drawFastVLine(cx, cy - R, 2 * R, COL_GRID_DARK);
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

    // ── Trail (fading) ── from primary scene track
    if (tr0 && tr0->active && tr0->trail_count > 0) {
        uint32_t now = millis();
        int start = tr0->trail_head;
        int total = tr0->trail_count;
        for (int k = 0; k < total; k++) {
            // Walk from oldest to newest
            int idx = (start + TRACK_TRAIL_LEN - total + k) % TRACK_TRAIL_LEN;
            const TargetTrack::TrailPoint &p = tr0->trail[idx];
            if (p.t_ms == 0) continue;
            uint32_t age = now - p.t_ms;
            if (age > 3000) continue;
            // Convert normalized position to cm, then to pixels
            int px = cx + (int)(p.pos[0] * norm_to_cm * pix_per_cm);
            int py = cy - (int)(p.pos[1] * norm_to_cm * pix_per_cm);
            uint16_t tc = (age < 500)  ? COL_WARN
                        : (age < 1500) ? COL_DIM
                                       : COL_GRID_DARK;
            int rad = (age < 500) ? 2 : 1;
            g.fillCircle(px, py, rad, tc);
        }
    }

    // ── Target + confidence ellipse ── from primary scene track
    if (tr0 && tr0->active && tr0->confidence > 0.05f) {
        int tx = cx + (int)(tr0->pos[0] * norm_to_cm * pix_per_cm);
        int ty = cy - (int)(tr0->pos[1] * norm_to_cm * pix_per_cm);
        uint16_t tcol = tr0->confidence > 0.5f ? COL_ALERT : COL_WARN;
        // Covariance is in normalized-units² — convert scaling for ellipse:
        // one-sigma pixel radius = sqrt(cov) * norm_to_cm * pix_per_cm
        float cov_scale = norm_to_cm * pix_per_cm;
        draw_confidence_ellipse(tx, ty, cov_scale,
                                tr0->cov_xx,
                                tr0->cov_yy,
                                tr0->cov_xy,
                                COL_DIM);
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

    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        uint16_t c = beacon_color(i);
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
    // v0.9 — WHOSE tripwire is this?
    //
    // In TW_REMOTE the tripwire is the beacon <-> ANCHOR link, because
    // the anchor is the unit that stays put and therefore the only one
    // whose link geometry means anything.  The probe is a remote display
    // for it.  Previously the probe evaluated its OWN link, so carrying
    // it around tripped its own wire constantly.
    const bool mirroring = (g_app.tripwire_mode == TW_REMOTE)
                        && (g_app.peer.cal_role == CAL_ROLE_PROBE)
                        && g_app.peer.peer_present;

    bool any_motion = false, any_presence = false;
    if (mirroring) {
        if (peer_tripwire_fresh()) {
            any_motion   = (g_app.tw_remote_status == LS_MOTION);
            any_presence = (g_app.tw_remote_status == LS_PRESENCE);
        }
    } else
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
    // v0.9: this used slot index i with 40 px spacing, so the 5th and 6th
    // beacons landed at x=180 and 220 on a 170 px panel -- off-screen.
    // It also spaced by SLOT, so beacons in slots 0/2/4 drew with gaps.
    // Pack by active order and divide the width by the count.
    // Source line: never leave the user guessing which link tripped.
    g.setFont(&fonts::Font2);
    if (mirroring) {
        bool fresh = peer_tripwire_fresh();
        g.setTextColor(fresh ? COL_MS_TEAL : COL_ALERT, COL_BG);
        g.setCursor(6, CONTENT_Y + 148);
        if (fresh) {
            char l[32];
            snprintf(l, sizeof(l), "anchor link B%u", (unsigned)g_app.tw_remote_beacon);
            g.print(l);
        } else {
            g.print("anchor link LOST");
        }
    } else if (g_app.peer.peer_present) {
        g.setTextColor(COL_MUTED, COL_BG);
        g.setCursor(6, CONTENT_Y + 148);
        g.print("this unit's link");
    }

    int y = CONTENT_Y + 170;
    g.setFont(&fonts::Font2);
    int n_act = 0;
    for (int i = 0; i < MAX_BEACONS; i++) if (g_app.beacon[i].active) n_act++;
    if (mirroring) n_act = 0;    // local dots mean nothing when mirroring
    if (n_act > 0) {
        int step = (SCREEN_W - 24) / n_act;
        if (step > 40) step = 40;
        int r = (step < 26) ? 7 : 10;
        int k = 0;
        for (int i = 0; i < MAX_BEACONS; i++) {
            BeaconState &b = g_app.beacon[i];
            if (!b.active) continue;
            int x = 14 + step / 2 + k * step;
            uint16_t dc = COL_FG;
            if (b.status == LS_MOTION)   dc = COL_WARN;
            if (b.status == LS_PRESENCE) dc = COL_ALERT;
            g.fillCircle(x, y, r, dc);
            g.drawCircle(x, y, r, COL_TEXT);
            g.setTextColor(COL_TEXT, COL_BG);
            char lbl[8]; snprintf(lbl, sizeof(lbl), "B%u", (unsigned)b.id);
            g.setCursor(x - 8, y + r + 4);
            g.print(lbl);
            k++;
        }
    }
}

static void draw_view_links() {
    auto &g = gfx();
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_TEXT, COL_BG);
    g.setCursor(6, CONTENT_Y + 4);
    g.print("Per-link metrics");

    // v0.8: the full 66 px per-beacon block fits at most 4 beacons on a
    // 320 px screen, and the loop below used to just `break` — so at 5 or
    // 6 beacons the last ones silently vanished from the diagnostic view.
    // Beacon count is now 3..6, so pick the layout that actually fits:
    // <=4 keeps the v0.7 block verbatim (3-beacon view is unchanged),
    // 5-6 drops the MAC line and tightens spacing.
    int n_active = 0;
    for (int i = 0; i < MAX_BEACONS; i++) if (g_app.beacon[i].active) n_active++;
    const bool compact = (n_active > 4);

    int y = CONTENT_Y + 24;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;

        char hdr[32]; snprintf(hdr, sizeof(hdr), "B%u", (unsigned)b.id);
        g.setTextColor(COL_ACCENT, COL_BG);
        g.setCursor(6, y);
        g.print(hdr);

        uint16_t c = COL_FG;
        if (b.status == LS_MOTION)   c = COL_WARN;
        if (b.status == LS_PRESENCE) c = COL_ALERT;

        // v0.9: commanded rate vs what the beacon reports/behaves like.
        // The old UI only had the inter-arrival EMA, which reads the same
        // whether a command landed or was never sent -- so a beacon
        // ignoring us was indistinguishable from one obeying us.
        char rate[24];
        {
            int obs_hz = (b.inter_arrival_ms_ema > 0.5f)
                       ? (int)(1000.0f / b.inter_arrival_ms_ema + 0.5f) : 0;
            if (b.cmd_rate_hz == 0)
                snprintf(rate, sizeof(rate), "%dHz uncmd", obs_hz);
            else if (b.reported_rate_hz == 0)
                snprintf(rate, sizeof(rate), "%d/%dHz no-ack",
                         obs_hz, (int)b.cmd_rate_hz);
            else
                snprintf(rate, sizeof(rate), "%d/%dHz%s", obs_hz,
                         (int)b.reported_rate_hz, b.fw_marker ? " x" : "");
        }

        if (compact) {
            // One line of numbers + the bar, 40 px total.
            char l1[40];
            snprintf(l1, sizeof(l1), "%s t%.3f", rate, b.feat_turbulence);
            g.setTextColor(COL_TEXT, COL_BG);
            g.setCursor(30, y);
            g.print(l1);
            progress_bar(6, y + 16, SCREEN_W - 40, 10, b.link_metric_ema, c);
            char pct[8]; snprintf(pct, sizeof(pct), "%3d%%",
                                  (int)(b.link_metric_ema * 100));
            g.setTextColor(COL_MUTED, COL_BG);
            g.setCursor(SCREEN_W - 30, y + 14);
            g.print(pct);
            y += 40;
        } else {
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
            char l1[40]; snprintf(l1, sizeof(l1), "turb %.3f  %s", b.feat_turbulence, rate);
            g.setTextColor(COL_TEXT, COL_BG);
            g.setCursor(6, y + 26);
            g.print(l1);

            progress_bar(6, y + 42, SCREEN_W - 40, 10, b.link_metric_ema, c);
            char pct[8]; snprintf(pct, sizeof(pct), "%3d%%", (int)(b.link_metric_ema * 100));
            g.setTextColor(COL_MUTED, COL_BG);
            g.setCursor(SCREEN_W - 30, y + 40);
            g.print(pct);

            y += 66;
        }
        if (y > SCREEN_H - FOOTER_H - 12) break;
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
// v0.8: the settings menu grew a conditional row.  The "Undock probe"
// row only exists on the PROBE unit, and only once a walk cal has
// actually produced a kernel to localize against — undocking before
// then would put the probe into a state where it can never get a fix.
// The anchor never shows the row at all.
bool ui_settings_undock_row_visible() {
    if (g_app.peer.cal_role != CAL_ROLE_PROBE) return false;
    if (!g_app.peer.peer_present) return false;
    if (g_app.mode != RM_TRIANGLE_3) return false;   // needs 3+ beacons
    return scene_probe_kernel_ready();
}

int ui_settings_row_count() {
    // Debug row is always present; the undock row only on a probe with a
    // usable kernel, and it sits last so its presence never shifts the
    // index of anything above it.
    return ui_settings_undock_row_visible() ? UI_SETTINGS_MAX_ROWS
                                            : UI_SETTINGS_MAX_ROWS - 1;
}

void ui_settings(int selected_row) {
    clear();
    draw_header("SETTINGS");
    auto &g = gfx();
    g.setFont(&fonts::Font2);

    const int rows = ui_settings_row_count();
    const char *labels[UI_SETTINGS_MAX_ROWS] = {
        "Sensitivity",
        "Redo full cal",           // wipes kernel + baseline, walk again
        "Redo baseline only",      // keep kernel, only re-do empty room
        "Change mode",
        "Cal role",                // AUTO / force PROBE / force ANCHOR
        "Exit",
        nullptr,                   // v0.8 undock row, label set below
    };
    char values[UI_SETTINGS_MAX_ROWS][24] = {};
    snprintf(values[0], 24, "%.1fx", g_app.sensitivity);
    strcpy(values[1], "");
    strcpy(values[2], "");
    // v0.8: RM_TRIANGLE_3 covers 3..6 beacons, so the label reports the
    // live count rather than the historical "3-triangle".  The enum name
    // is deliberately unchanged; only what the user reads changes.
    const char *mode_names[] = {"none", "1-tripwire", "2-line", "multi"};
    if (g_app.mode == RM_TRIANGLE_3)
        snprintf(values[3], 24, "%d beacons", g_app.beacon_count);
    else
        snprintf(values[3], 24, "%s", mode_names[g_app.mode]);
    const char *ro_names[] = {"AUTO", "PROBE", "ANCHOR"};
    snprintf(values[4], 24, "%s", ro_names[g_app.peer.role_override]);
    strcpy(values[5], "");
    labels[UI_SETTINGS_ROW_TRIPWIRE] = "Tripwire";
    snprintf(values[UI_SETTINGS_ROW_TRIPWIRE], 24, "%s",
             g_app.tripwire_mode == TW_REMOTE ? "anchor link" : "both units");
    labels[UI_SETTINGS_ROW_DEBUG] = "Debug log";
    snprintf(values[UI_SETTINGS_ROW_DEBUG], 24, "%d", ms_log_count());
    if (rows > UI_SETTINGS_ROW_UNDOCK) {
        labels[UI_SETTINGS_ROW_UNDOCK] =
            g_app.probe_undocked ? "Return to stereo" : "Undock probe";
        strcpy(values[UI_SETTINGS_ROW_UNDOCK], "");
    }

    int y = CONTENT_Y + 10;
    for (int i = 0; i < rows; i++) {
        bool sel = (i == selected_row);
        uint16_t c = sel ? COL_ACCENT : COL_TEXT;
        if (sel) g.fillRect(0, y - 2, SCREEN_W, 18, COL_MS_CHROME);
        g.setTextColor(c, sel ? COL_MS_CHROME : COL_BG);
        g.setCursor(8, y);
        g.print(labels[i] ? labels[i] : "");
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
    // v0.9: was the literal "6cm". If STEREO_BASELINE_CM is ever changed
    // (it MUST match the physical spacing or every bearing is scaled
    // wrong) this label has to follow it, or the setup screen instructs
    // the user to build something the solver is not expecting.
    {
        char bl[12];
        snprintf(bl, sizeof(bl), "%.1fcm", (double)STEREO_BASELINE_CM);
        g.setCursor(cx - 12, cy + box_h/2 - 12);
        g.print(bl);
    }

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

    // v0.9: 14 px per row from CONTENT_Y+110 reaches y=216 at six
    // beacons, and the cal line below was pinned at CONTENT_Y+190 (=212)
    // -- they overlapped. Track the running y instead of assuming.
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

    // Cal status — below whatever the table actually used.
    y += 8;
    if (y < CONTENT_Y + 190) y = CONTENT_Y + 190;
    if (y > SCREEN_H - FOOTER_H - 16) y = SCREEN_H - FOOTER_H - 16;
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
    int label_y = cy + R + 12;
    int col = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        if (b.aoa_conf < 0.02f) continue;
        uint16_t c = beacon_color(col);
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
        uint16_t c = beacon_color(col);
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

// Per-beacon identity colour.  ONE definition, because there were three
// copies of this table and two of them only had three entries with a
// `% 3` -- so B1/B4, B2/B5 and B3/B6 rendered identically and the
// colour told the user nothing at 4+ beacons.
// Human name for a landmark.  The results screen printed raw enum
// indices ("worst lm: 14") which mean nothing to anyone standing in the
// room -- the whole point of that line is to say WHERE the model is
// weakest so it can be re-walked.
static const char *landmark_name(uint8_t lm) {
    switch (lm) {
        case LM_RX:          return "RX bar";
        case LM_BEACON_1:    return "B1";
        case LM_BEACON_2:    return "B2";
        case LM_BEACON_3:    return "B3";
        case LM_BEACON_4:    return "B4";
        case LM_BEACON_5:    return "B5";
        case LM_BEACON_6:    return "B6";
        case LM_CENTROID:    return "centre";
        case LM_MID_12:      return "B1-B2 mid";
        case LM_MID_23:      return "B2-B3 mid";
        case LM_MID_13:      return "B1-B3 mid";
        case LM_MID_34:      return "B3-B4 mid";
        case LM_MID_45:      return "B4-B5 mid";
        case LM_MID_56:      return "B5-B6 mid";
        case LM_MID_41:      return "B4-B1 mid";
        case LM_MID_51:      return "B5-B1 mid";
        case LM_MID_61:      return "B6-B1 mid";
        case LM_OPPOSITE_RX: return "outside";
        default:             return "?";
    }
}

static uint16_t beacon_color(int idx) {
    static const uint16_t C[6] = {
        COL_MS_CH_A, COL_MS_CH_B, COL_MS_CH_C,
        COL_MS_CH_D, COL_MS_CH_E, COL_MS_CH_F
    };
    if (idx < 0) idx = 0;
    return C[idx % 6];
}

// Draw a small map (radar mini-view) into a rect showing beacon
// triangle + RX + a highlighted landmark position.  Used inside
// ui_cal_landmark_walk to help the user visualize where to walk.
// ═══════════════════════════════════════════════════════════════
//  v0.9 — ROTATION COMPASS
// ═══════════════════════════════════════════════════════════════
// theta for the Fourier fit is inferred from ELAPSED TIME, assuming a
// uniform 10 s turn.  People hesitate and overshoot, and that error goes
// straight into the a1/b1 phase -- the very thing the rotation exists to
// measure.  Rather than tolerate the slop, give the user a pace to
// match: if they track the green needle, theta(t) is correct by
// construction.
//
// BODY FRAME.  The script has them face B1, so B1 starts straight ahead
// (needle up).  Turning right swings B1 to their left, so the needle
// runs anticlockwise on screen -- what they would see if they held a
// compass pointed at the beacon.
//
//   GREEN = where B1 should be, at a perfectly uniform turn rate.
//   RED   = where the radio says their body actually is, from the
//           angular centroid of per-beacon attenuation: the body blocks
//           the links behind it, so that vector rotates with them.
//           Same math the exterior bearing estimator uses.
static void draw_rotation_compass(int cx, int cy, int R,
                                  uint32_t elapsed_ms, uint32_t hold_ms) {
    auto &g = gfx();

    g.drawCircle(cx, cy, R,      COL_MS_DIM);
    g.drawCircle(cx, cy, R - 6,  COL_MS_DIM);
    // Tick every 45 deg so progress is readable at a glance.
    for (int t = 0; t < 8; t++) {
        float a = t * (float)M_PI / 4.0f;
        int x0 = cx + (int)((R - 6) * sinf(a)), y0 = cy - (int)((R - 6) * cosf(a));
        int x1 = cx + (int)(R       * sinf(a)), y1 = cy - (int)(R       * cosf(a));
        g.drawLine(x0, y0, x1, y1, COL_MS_DIM);
    }

    // ── expected (green) ──
    float frac = (hold_ms > 0) ? (float)elapsed_ms / (float)hold_ms : 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    float want = -frac * 2.0f * (float)M_PI;          // anticlockwise
    int wx = cx + (int)((R - 10) * sinf(want));
    int wy = cy - (int)((R - 10) * cosf(want));
    g.drawLine(cx, cy, wx, wy, COL_MS_LIME);
    g.fillCircle(wx, wy, 5, COL_MS_LIME);

    // ── measured (red) ──
    // Angular centroid of attenuation across beacons, in the ROOM frame,
    // then rotated into the body frame by the start heading.
    float sx = 0, sy = 0; int nb = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        const BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        float bx, by;
        scene_landmark_pos((LandmarkId)(LM_BEACON_1 + i), &bx, &by);
        float th = atan2f(by, bx);
        float w  = b.link_metric_ema;                 // 0..1, higher = more perturbed
        if (w < 0) w = 0;
        sx += w * cosf(th); sy += w * sinf(th); nb++;
    }
    if (nb > 0 && (sx * sx + sy * sy) > 1e-6f) {
        // B1's room bearing is the zero of the body frame.
        float b1x, b1y; scene_landmark_pos(LM_BEACON_1, &b1x, &b1y);
        float ref  = atan2f(b1y, b1x);
        float meas = atan2f(sy, sx) - ref;
        int mx = cx + (int)((R - 18) * sinf(-meas));
        int my = cy - (int)((R - 18) * cosf(-meas));
        g.drawLine(cx, cy, mx, my, COL_MS_ALERT);
        g.fillCircle(mx, my, 4, COL_MS_ALERT);
    }

    g.fillCircle(cx, cy, 3, COL_MS_INK);

    // Legend, because two needles with no key is worse than one needle.
    g.setFont(&fonts::Font0);
    g.setTextColor(COL_MS_LIME, COL_BG);
    g.setCursor(cx - R, cy + R + 4);
    g.print("green=pace");
    g.setTextColor(COL_MS_ALERT, COL_BG);
    g.setCursor(cx + 6, cy + R + 4);
    g.print("red=you");
}

// Top-down room map for the cal walk.
//
// v0.9: beacons were unlabelled 2 px dots, so a step saying "walk to B3"
// gave the user no way to tell WHICH physical box that was -- the walk
// could not be executed correctly, which makes the whole cal suspect.
// Each beacon is now drawn with its number and in ITS OWN colour, the
// same COL_MS_CH_* used by the links view and AoA compass, so the
// identity is consistent everywhere the beacon appears.
static void draw_mini_landmark_map(int box_x, int box_y, int box_w, int box_h,
                                   LandmarkId highlight,
                                   LandmarkId next) {
    auto &g = gfx();
    g.drawRect(box_x, box_y, box_w, box_h, COL_MS_DIM);
    int cx = box_x + box_w / 2;
    int cy = box_y + box_h / 2;
    int R  = (box_w < box_h ? box_w : box_h) / 2 - 12;   // room for labels
    float pix_per_unit = (float)R / SCENE_EXTENT;

    for (float r = 0.5f; r <= SCENE_EXTENT; r += 0.5f)
        g.drawCircle(cx, cy, (int)(r * pix_per_unit), COL_MS_DIM);

    const int n_b = g_app.beacon_count < 1 ? 0
                  : (g_app.beacon_count > MAX_BEACONS ? MAX_BEACONS
                                                      : g_app.beacon_count);

    // Where the user is being sent, resolved first so a beacon that IS
    // the target can be drawn emphasised rather than drawn twice.
    float hx = 0, hy = 0; bool have_h = false;
    if (highlight < LM_COUNT) { scene_landmark_pos(highlight, &hx, &hy); have_h = true; }

    // Route line to the next stop, under everything else.
    if (next < LM_COUNT && highlight != next && have_h) {
        float nx, ny; scene_landmark_pos(next, &nx, &ny);
        g.drawLine(cx + (int)(hx * pix_per_unit), cy - (int)(hy * pix_per_unit),
                   cx + (int)(nx * pix_per_unit), cy - (int)(ny * pix_per_unit),
                   COL_MS_DIM);
    }

    // Beacons, numbered and colour-coded.
    g.setFont(&fonts::Font0);
    for (int i = 0; i < n_b; i++) {
        float bx, by;
        scene_landmark_pos((LandmarkId)(LM_BEACON_1 + i), &bx, &by);
        int px = cx + (int)(bx * pix_per_unit);
        int py = cy - (int)(by * pix_per_unit);

        const bool is_target = have_h && (highlight == (LandmarkId)(LM_BEACON_1 + i));
        uint16_t c = beacon_color(i);         // same hue as links / compass

        if (is_target) {
            // Pulse so the eye lands on it immediately.
            if ((millis() / 350) % 2) {
                g.fillCircle(px, py, 7, COL_MS_WARN);
                g.drawCircle(px, py, 9, COL_MS_WARN);
            } else {
                g.fillCircle(px, py, 6, COL_MS_WARN);
            }
        } else {
            g.fillCircle(px, py, 4, c);
            g.drawCircle(px, py, 4, COL_MS_INK);
        }

        char lbl[4]; snprintf(lbl, sizeof(lbl), "B%d", i + 1);
        // Push the label radially outward so it never sits on the dot.
        int lx = px + (bx >= 0 ? 7 : -14);
        int ly = py + (by >= 0 ? -12 : 5);
        g.setTextColor(is_target ? COL_MS_WARN : c, COL_BG);
        g.setCursor(lx, ly);
        g.print(lbl);
    }

    // The receiver bar.
    g.fillRect(cx - 4, cy - 2, 8, 4, COL_MS_TEAL);
    g.setTextColor(COL_MS_TEAL, COL_BG);
    g.setCursor(cx + 6, cy + 2);
    g.print("RX");

    // A non-beacon target (centroid, midpoint, exterior anchor) still
    // needs marking -- those are the stops with no physical object to
    // walk to, so they matter MORE, not less.
    if (have_h && !(highlight >= LM_BEACON_1 && highlight < LM_BEACON_1 + n_b)) {
        int px = cx + (int)(hx * pix_per_unit);
        int py = cy - (int)(hy * pix_per_unit);
        uint16_t c = ((millis() / 350) % 2) ? COL_MS_WARN : COL_MS_ALERT;
        g.drawCircle(px, py, 6, c);
        g.drawCircle(px, py, 8, c);
        g.drawLine(px - 4, py, px + 4, py, c);
        g.drawLine(px, py - 4, px, py + 4, c);
        g.setTextColor(c, COL_BG);
        g.setCursor(px + 9, py - 4);
        g.print("GO");
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
    // v0.9: the duration is a function of beacon count now -- the script
    // is 4N+12 steps with 2N+4 stands and 3 rotations, so quoting a flat
    // "~100 seconds" understates a 6-beacon walk by about a minute.
    int n_cal = g_app.beacon_count;
    if (n_cal < 3) n_cal = 3;
    if (n_cal > MAX_BEACONS) n_cal = MAX_BEACONS;
    int est_s = (2 * n_cal + 4) * 4     // stands, ~4 s each
              + 3 * 10                  // three 360 rotations
              + n_cal * 8;              // walking between stops
    char tail[24];
    snprintf(tail, sizeof(tail), "~%d seconds.", est_s);

    char body[256];
    if (g_app.cal_mode == CAL_MODE_STEREO) {
        snprintf(body, sizeof(body),
            "You will walk\n"
            "a scripted path\n"
            "through the room\n"
            "to teach the model\n"
            "the room's radio\n"
            "response.\n"
            "\n"
            "Carry PROBE.\n"
            "ANCHOR stays put.\n"
            "%s", tail);
    } else {
        snprintf(body, sizeof(body),
            "Hold the T-Display\n"
            "against your chest\n"
            "for the walk.\n"
            "\n"
            "Solo cal is a\n"
            "degraded fallback.\n"
            "Two receivers give\n"
            "much better results.\n"
            "\n"
            "%s", tail);
    }
    draw_multiline(6, CONTENT_Y + 44, 14, COL_MS_INK, body);

    // v0.9: state the RADAR mode here.  It decides whether a walk cal
    // happens at all -- tripwire skips it entirely -- and it was
    // previously invisible, so a mode that did not match the beacons on
    // the floor only revealed itself by the cal behaving unexpectedly.
    {
        static const char *RADAR_NAMES[] = {"none","tripwire","line-2","multi"};
        char m[40];
        snprintf(m, sizeof(m), "mode: %s  (%d beacons)",
                 RADAR_NAMES[(int)g_app.mode <= 3 ? (int)g_app.mode : 0],
                 (int)g_app.beacon_count);
        g.setFont(&fonts::Font2);
        g.setTextColor(g_app.mode == RM_TRIPWIRE_1 ? COL_WARN : COL_MUTED, COL_BG);
        g.setCursor(6, SCREEN_H - FOOTER_H - 20);
        g.print(m);
    }

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
            "The OTHER unit\n"
            "is the anchor.\n"
            "\n"
            "Set it down in\n"
            "the middle of\n"
            "the beacon ring\n"
            "and leave it.\n"
            "\n"
            "Keep this one.");
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
            "ring.\n"
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
    // The duration depends on the beacon transmit rate, so it is
    // reported rather than hardcoded -- at 20 Hz this is 25 s, not the
    // 5 s a 100 Hz beacon would give.
    // Wording follows the CAL MODE.  This screen named "the PROBE" and
    // "ANCHOR" unconditionally, but in solo and tripwire there is only
    // one device in the user's hand and neither word means anything --
    // it read as an instruction to fetch equipment that does not exist.
    char body[224];
    if (g_app.cal_mode == CAL_MODE_STEREO) {
        snprintf(body, sizeof(body),
            "Measuring the\n"
            "empty room.\n"
            "\n"
            "Stay outside\n"
            "until this\n"
            "finishes.\n"
            "\n"
            "~%d seconds, then\n"
            "come back in.", csi_baseline_expected_seconds());
    } else {
        snprintf(body, sizeof(body),
            "Put this unit\n"
            "down and leave\n"
            "the room.\n"
            "\n"
            "It is measuring\n"
            "the room with\n"
            "nobody in it.\n"
            "\n"
            "~%d seconds.", csi_baseline_expected_seconds());
    }
    draw_multiline(6, CONTENT_Y + 8, 14, COL_MS_INK, body);

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
    WizardPhase phase = wizard_current_phase();

    // On a ROTATE step the room map is useless -- the user is standing
    // still and the only thing that matters is their turn RATE.  Show
    // the compass instead, so they have something to pace against.
    if (step->kind == STEP_ROTATE && phase == WP_CAPTURE) {
        int mh = (map_h < 110 ? 110 : map_h);
        int R  = (mh / 2) - 16;
        if (R > (SCREEN_W / 2) - 14) R = (SCREEN_W / 2) - 14;
        draw_rotation_compass(SCREEN_W / 2, map_y + mh / 2, R,
                              wizard_step_elapsed_ms(), step->hold_ms);
    } else {
        // v0.9: labels need vertical room; the map was sized for bare dots.
        draw_mini_landmark_map(4, map_y, SCREEN_W - 8,
                               map_h < 110 ? 110 : map_h, highlight, next);
    }

    // Timer / status line right above footer — driven by wizard phase.
    uint32_t hold_rem = wizard_hold_remaining_ms();
    uint32_t elapsed  = wizard_step_elapsed_ms();
    char tbuf[24];
    const char *rlabel = "next";

    switch (step->kind) {
        case STEP_WALK:
            if (phase == WP_ARM) {
                snprintf(tbuf, sizeof(tbuf), "press BEGIN");
                g.setTextColor(COL_MS_LIME, COL_BG);
                rlabel = "begin";
            } else {  // WP_CAPTURE
                snprintf(tbuf, sizeof(tbuf), "walking %.1fs",
                         (double)elapsed / 1000.0);
                g.setTextColor(COL_MS_WARN, COL_BG);
                rlabel = "arrived";
            }
            break;
        case STEP_STAND:
        case STEP_ROTATE:
            if (phase == WP_CAPTURE && hold_rem > 0) {
                snprintf(tbuf, sizeof(tbuf), "hold %.1fs",
                         (double)hold_rem / 1000.0);
                g.setTextColor(COL_MS_WARN, COL_BG);
                rlabel = "wait";
            } else {
                snprintf(tbuf, sizeof(tbuf), "ready");
                g.setTextColor(COL_MS_LIME, COL_BG);
                rlabel = "next";
            }
            break;
        case STEP_INTRO:
        default:
            snprintf(tbuf, sizeof(tbuf), "ready");
            g.setTextColor(COL_MS_LIME, COL_BG);
            rlabel = "begin";
            break;
    }
    g.setFont(&fonts::Font2);
    int ttw = g.textWidth(tbuf);
    g.setCursor((SCREEN_W - ttw) / 2, map_y + map_h + 2);
    g.print(tbuf);

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
    // TRIPWIRE HAS NO WALK, SO IT HAS NO KERNEL.
    //
    // Grading it against cross-validation, loop closure and ambiguity --
    // all structurally zero -- printed a wall of 0.000 and a MARGINAL
    // verdict for a calibration that had in fact completed correctly.
    // Tripwire only needs the empty-room baseline, so report THAT.
    if (g_app.mode == RM_TRIPWIRE_1) {
        clear();
        draw_header("CAL RESULT");
        auto &g = gfx();
        const CalReport &r = s_last_report;

        g.setFont(&fonts::Font4);
        g.setTextColor(COL_MS_LIME, COL_BG);
        const char *v = "BASELINE OK";
        int vw = g.textWidth(v);
        g.setCursor((SCREEN_W - vw) / 2, CONTENT_Y + 24);
        g.print(v);

        g.setFont(&fonts::Font2);
        draw_multiline(8, CONTENT_Y + 60, 16, COL_MS_INK,
            "Tripwire mode.\n"
            "\n"
            "One beacon, one\n"
            "link. No room\n"
            "walk is needed\n"
            "or possible.\n"
            "\n"
            "The empty-room\n"
            "reference is\n"
            "captured.");

        char line[40];
        int n_act = 0;
        for (int i = 0; i < MAX_BEACONS; i++) if (g_app.beacon[i].active) n_act++;
        snprintf(line, sizeof(line), "beacons: %d   mode: %s",
                 n_act, r.mode == CAL_MODE_SOLO ? "solo" : "stereo");
        g.setTextColor(COL_MUTED, COL_BG);
        g.setCursor(8, SCREEN_H - FOOTER_H - 20);
        g.print(line);

        draw_footer("redo", "accept");
        flush();
        return;
    }


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
    // v0.85: grade on the RATE of confusable far-pairs, not the raw
    // count.  The number of pairs tested grows quadratically with
    // landmark count (39 at 3 beacons, 98 at 6), so demanding "zero"
    // gets strictly harder as the array gets better — backwards.  The
    // rate is dimensionless and comparable across beacon counts.
    bool alias_ok = (r.far_pairs_tested == 0) ||
                    ((uint32_t)r.alias_pairs_found * 100u
                       < (uint32_t)r.far_pairs_tested * 2u);
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
    uint16_t acol = r.alias_pairs_found == 0 ? COL_MS_LIME
                  : alias_ok                  ? COL_MS_WARN
                                             : COL_MS_ALERT;
    g.setTextColor(acol, COL_MS_BG);
    // Show the margin (the permanent number: peak sidelobe of the
    // array's ambiguity function, in sigma of the sensor's own noise)
    // alongside the rate that produced the verdict.
    snprintf(line, sizeof(line), "ambig : %.1f sd  %u/%u",
             (double)r.ambiguity_margin_sigma,
             (unsigned)r.alias_pairs_found,
             (unsigned)r.far_pairs_tested);
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
        snprintf(line, sizeof(line), "weakest: %s (%.2f)",
                 landmark_name(r.worst_landmark),
                 (double)r.worst_landmark_error);
        g.setCursor(6, y); g.print(line); y += 14;
    }

    // v0.5: geometry-validation warnings (PROBE at beacon lm should
    // observe that beacon strongest).  If any bit set, tell the user
    // which beacon(s) might be mispositioned or misidentified.
    if (r.geometry_validation_fail_mask) {
        g.setTextColor(COL_MS_ALERT, COL_MS_BG);
        // This printed a raw hex bitmask ("geom warn: 0x05"), which told
        // the user nothing actionable.  The mask is one bit PER BEACON,
        // flagging beacons that did not read strongest when the probe
        // stood at them -- i.e. probably placed or numbered wrongly.
        // Name them, because the fix is physical.
        char blist[24]; int bi = 0;
        blist[0] = 0;
        for (int b = 0; b < MAX_BEACONS; b++) {
            if (!(r.geometry_validation_fail_mask & (1u << b))) continue;
            bi += snprintf(blist + bi, sizeof(blist) - bi,
                           "%sB%d", bi ? "," : "", b + 1);
            if (bi >= (int)sizeof(blist) - 4) break;
        }
        snprintf(line, sizeof(line), "check placement: %s", blist);
        g.setCursor(6, y); g.print(line); y += 12;
    }

    // v0.5: alias-pair preview (first pair only — space-limited screen)
    if (r.alias_pair_count > 0 && y < SCREEN_H - FOOTER_H - 12) {
        g.setTextColor(COL_MS_WARN, COL_MS_BG);
        snprintf(line, sizeof(line), "closest: %s/%s %.1fsd",
                 landmark_name(r.alias_pairs[0].lm_a),
                 landmark_name(r.alias_pairs[0].lm_b),
                 (double)r.alias_pairs[0].sigma_distance);
        g.setCursor(6, y); g.print(line); y += 12;
    }

    draw_footer("redo", good ? "accept" : "accept?");
    flush();
}

// ═══════════════════════════════════════════════════════════════
//  v0.8 — MOBILE PROBE VIEW (ST_MOBILE_PROBE)
// ═══════════════════════════════════════════════════════════════
//  Shown only on the undocked probe.  Two things on one screen:
//    1. Where the anchor thinks everybody is (streamed track list).
//    2. Where THIS unit thinks it is (its own self-localization).
//  Self-tagged tracks are dimmed and labelled "YOU", matching exactly
//  what the anchor's radar view draws, so the two screens agree.
void ui_mobile_probe_view(const PeerTrackStatePacket *track_state,
                          uint32_t track_state_age_ms,
                          const float probe_pos[2],
                          const float probe_cov[3],
                          float probe_conf,
                          bool acquiring) {
    clear();
    draw_header("MOBILE PROBE");
    auto &g = gfx();
    g.setFont(&fonts::Font2);

    const int map_y = CONTENT_Y + 18;
    const int map_h = 210;
    const int cx = SCREEN_W / 2;
    const int cy = map_y + map_h / 2;
    const int usable_h = (SCREEN_W / 2) - 6;
    const int usable_v = (map_h / 2) - 6;
    const int R = (usable_h < usable_v) ? usable_h : usable_v;
    const float ppu = (float)R / SCENE_EXTENT;

    // ── Status line ──
    // The link state is the first thing that matters here: a frozen
    // picture and a live picture look identical, so say which it is.
    const bool link_ok = (track_state != nullptr)
                      && (track_state_age_ms < 1500);
    const char *status;
    uint16_t status_col;
    if (acquiring)      { status = "acquiring pos..."; status_col = COL_WARN; }
    else if (!link_ok)  { status = "anchor unreachable"; status_col = COL_ALERT; }
    else                { status = "linked";           status_col = COL_FG; }
    g.setTextColor(status_col, COL_BG);
    g.setCursor(4, CONTENT_Y + 2);
    g.print(status);

    char cbuf[12];
    snprintf(cbuf, sizeof(cbuf), "%3d%%", (int)(probe_conf * 100.0f));
    int cw = g.textWidth(cbuf);
    g.setTextColor(probe_conf > 0.5f ? COL_FG
                 : probe_conf > 0.3f ? COL_WARN : COL_MUTED, COL_BG);
    g.setCursor(SCREEN_W - cw - 4, CONTENT_Y + 2);
    g.print(cbuf);

    // ── Map frame + rings ──
    g.drawRect(0, map_y, SCREEN_W, map_h, COL_GRID_DARK);
    g.drawCircle(cx, cy, R / 2, COL_GRID_DARK);
    g.drawCircle(cx, cy, R,     COL_GRID_DARK);
    g.drawFastHLine(1, cy, SCREEN_W - 2, COL_GRID_DARK);
    g.drawFastVLine(cx, map_y + 1, map_h - 2, COL_GRID_DARK);

    // ── Beacons at their calibrated positions ──
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        float lx, ly;
        scene_landmark_pos((LandmarkId)(LM_BEACON_1 + i), &lx, &ly);
        int bx = cx + (int)(lx * ppu);
        int by = cy - (int)(ly * ppu);
        g.fillCircle(bx, by, 3, COL_ACCENT);
    }

    // ── Anchor's tracks ──
    if (track_state) {
        for (int i = 0; i < track_state->n_tracks && i < TRACK_MAX; i++) {
            const auto &t = track_state->tracks[i];
            if (!t.active) continue;
            int tx = cx + (int)(t.pos_x * ppu);
            int ty = cy - (int)(t.pos_y * ppu);
            // Stale link: draw what we last knew, but greyed, so the
            // user can tell memory from measurement at a glance.
            uint16_t tc = !link_ok  ? COL_GRID
                        : t.is_self ? COL_MUTED
                                    : COL_FG;
            g.drawCircle(tx, ty, 6, tc);
            g.drawCircle(tx, ty, 3, tc);
            if (t.is_self) {
                g.setTextColor(COL_MUTED, COL_BG);
                g.setCursor(tx + 8, ty - 6);
                g.print("YOU");
            }
        }
    }

    // ── This unit's own position estimate ──
    // Drawn as a cross plus a 1-sigma ellipse.  While acquiring we draw
    // the marker hollow so a not-yet-trusted fix never looks like a
    // settled one.
    int px = cx + (int)(probe_pos[0] * ppu);
    int py = cy - (int)(probe_pos[1] * ppu);
    uint16_t self_col = acquiring ? COL_WARN : COL_ACCENT;
    g.drawLine(px - 6, py, px + 6, py, self_col);
    g.drawLine(px, py - 6, px, py + 6, self_col);
    if (!acquiring) g.fillCircle(px, py, 2, self_col);
    // Hand-drawn as a polyline rather than drawEllipse(), to match how
    // render.cpp draws its covariance ellipses — same visual weight,
    // and no dependency on a primitive this build may not expose.
    // Axis-aligned is enough here: the cross-term only skews the shape,
    // and the user reads this as "roughly this uncertain", not as a
    // precise orientation.
    {
        float ax = sqrtf(fmaxf(1e-6f, probe_cov[0])) * ppu;
        float ay = sqrtf(fmaxf(1e-6f, probe_cov[1])) * ppu;
        if (ax > 60) ax = 60;
        if (ay > 60) ay = 60;
        if (ax < 3)  ax = 3;
        if (ay < 3)  ay = 3;
        const int N = 20;
        int prev_x = 0, prev_y = 0;
        for (int i = 0; i <= N; i++) {
            float t = (float)i / (float)N * 2.0f * (float)M_PI;
            int ex = px + (int)(ax * cosf(t));
            int ey = py - (int)(ay * sinf(t));
            if (i > 0) g.drawLine(prev_x, prev_y, ex, ey, self_col);
            prev_x = ex; prev_y = ey;
        }
    }

    draw_footer("back", "");
    flush();
}

// ═══════════════════════════════════════════════════════════════
//  v0.9 — DEBUG LOG VIEW
// ═══════════════════════════════════════════════════════════════
// Renders the in-RAM ring.  Serial is compiled out on this build, so
// this is the only place log lines surface.  Font0 is used to fit a
// useful number of lines on a 170 px panel; these are diagnostics, so
// density beats legibility.
void ui_debug_log(int scroll) {
    clear();
    draw_header("DEBUG LOG");
    auto &g = gfx();
    g.setFont(&fonts::Font0);

    const int line_h  = 9;
    const int top     = CONTENT_Y + 4;
    const int usable  = (SCREEN_H - FOOTER_H) - top - 4;
    const int visible = usable / line_h;

    const int total = ms_log_count();
    if (total == 0) {
        g.setFont(&fonts::Font2);
        g.setTextColor(COL_MUTED, COL_BG);
        g.setCursor(8, top + 20);
        g.print("(no log lines)");
        draw_footer("back", "clear");
        flush();
        return;
    }

    // Clamp scroll so the last page always sits flush with the bottom.
    int max_scroll = total - visible;
    if (max_scroll < 0)      max_scroll = 0;
    if (scroll > max_scroll) scroll = max_scroll;
    if (scroll < 0)          scroll = 0;

    int y = top;
    for (int i = 0; i < visible && (scroll + i) < total; i++) {
        const char *ln = ms_log_line(scroll + i);
        // Colour by severity so a fault is findable without reading.
        uint16_t c = COL_TEXT;
        if (strstr(ln, "WARNING") || strstr(ln, "no-ack") || strstr(ln, "off-rate"))
            c = COL_WARN;
        if (strstr(ln, "FULL") || strstr(ln, "unreachable") || strstr(ln, "NO CANVAS"))
            c = COL_ALERT;
        g.setTextColor(c, COL_BG);
        g.setCursor(4, y);
        g.print(ln);
        y += line_h;
    }

    // Position indicator
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    char pos[24];
    snprintf(pos, sizeof(pos), "%d-%d/%d",
             scroll + 1,
             (scroll + visible) < total ? (scroll + visible) : total,
             total);
    int pw = g.textWidth(pos);
    g.setCursor(SCREEN_W - pw - 4, SCREEN_H - FOOTER_H - 14);
    g.print(pos);

    draw_footer("scroll", "clear");
    flush();
}

// ═══════════════════════════════════════════════════════════════
//  v0.9 — LEAVE-THE-ROOM COUNTDOWN
// ═══════════════════════════════════════════════════════════════
// Shown before any baseline sample is taken.  Big numeral, because the
// user is walking away from the device and needs to read it at a
// distance and over their shoulder.
void ui_cal_leave_countdown(uint32_t seconds_left) {
    clear();
    draw_header("EMPTY ROOM");
    auto &g = gfx();

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MS_INK, COL_BG);
    draw_multiline(6, CONTENT_Y + 10, 16, COL_MS_INK,
        (g_app.cal_mode == CAL_MODE_STEREO)
            ? "LEAVE THE ROOM\n"
              "with the PROBE.\n"
              "\n"
              "Measuring starts\n"
              "when this hits 0."
            : "PUT THIS DOWN\n"
              "and leave the\n"
              "room.\n"
              "\n"
              "Measuring starts\n"
              "when this hits 0.");

    // Countdown numeral
    g.setFont(&fonts::Font7);
    g.setTextColor(seconds_left <= 2 ? COL_ALERT : COL_WARN, COL_BG);
    char n[8]; snprintf(n, sizeof(n), "%u", (unsigned)seconds_left);
    int nw = g.textWidth(n);
    g.setCursor((SCREEN_W - nw) / 2, CONTENT_Y + 120);
    g.print(n);

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    g.setCursor(6, CONTENT_Y + 200);
    g.print("Nothing is being");
    g.setCursor(6, CONTENT_Y + 216);
    g.print("recorded yet.");

    draw_footer("back", "");
    flush();
}

// ═══════════════════════════════════════════════════════════════
//  v0.9 — PICK THE UNIT YOU CARRY
// ═══════════════════════════════════════════════════════════════
// Shown on BOTH units simultaneously.  Press the one in your hand; it
// becomes the PROBE and the other becomes the ANCHOR.  The firmware
// used to decide this by MAC order, which meant it told you to put down
// whichever device you happened to be holding.
void ui_pick_carry() {
    clear();
    draw_header("WHICH ONE?");
    auto &g = gfx();

    draw_multiline(6, CONTENT_Y + 12, 16, COL_MS_INK,
        "Press the button\n"
        "on the unit you\n"
        "are HOLDING.\n"
        "\n"
        "That one comes\n"
        "with you.\n"
        "\n"
        "The other stays\n"
        "in the room.");

    // Pulse so it is obvious this screen wants a press, not a wait.
    g.setFont(&fonts::Font4);
    uint32_t p = (millis() / 400) % 2;
    g.setTextColor(p ? COL_MS_LIME : COL_MS_VIOLET_BRIGHT, COL_BG);
    const char *cta = "PRESS ME";
    int cw = g.textWidth(cta);
    g.setCursor((SCREEN_W - cw) / 2, CONTENT_Y + 190);
    g.print(cta);

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    g.setCursor(6, CONTENT_Y + 220);
    g.print("...on THIS unit");

    draw_footer("back", "this one");
    flush();
}

// Anchor-side view of the same moment: once the other unit has been
// pressed, this one says what it is and asks for nothing.  You should
// never have to touch the anchor except to change its display.
void ui_pick_carry_anchor() {
    clear();
    draw_header("ANCHOR");
    auto &g = gfx();
    g.setFont(&fonts::Font4);
    g.setTextColor(COL_MS_TEAL, COL_BG);
    const char *t = "ANCHOR";
    int tw = g.textWidth(t);
    g.setCursor((SCREEN_W - tw) / 2, CONTENT_Y + 40);
    g.print(t);

    draw_multiline(6, CONTENT_Y + 90, 16, COL_MS_INK,
        "This unit stays\n"
        "in the room.\n"
        "\n"
        "It follows the\n"
        "one you carry -\n"
        "no need to press\n"
        "anything here.");

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    g.setCursor(6, CONTENT_Y + 210);
    g.print("...but you still can.");

    // Buttons work here even though nothing requires them: the anchor is
    // a follower, not a locked-out slave.
    draw_footer("back", "carry this");
    flush();
}

// ═══════════════════════════════════════════════════════════════
//  v0.9 — EMPTY ROOM: WALK OUT FIRST, THEN PRESS
// ═══════════════════════════════════════════════════════════════
// The old screen said "leave the room" and started measuring at once,
// never telling the user to press anything and never saying what to do
// when it finished.  You are holding the probe, so you can simply carry
// it out and start the measurement from where you are standing.
void ui_cal_empty_prompt() {
    clear();
    draw_header("EMPTY ROOM");
    auto &g = gfx();

    draw_multiline(6, CONTENT_Y + 10, 16, COL_MS_INK,
        "1. Take this unit\n"
        "   and walk out\n"
        "   of the room.\n"
        "\n"
        "2. Standing\n"
        "   outside, press\n"
        "   START.\n"
        "\n"
        "3. Wait, then come\n"
        "   back in.");

    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MUTED, COL_BG);
    g.setCursor(6, CONTENT_Y + 196);
    g.print("The anchor stays put.");
    g.setCursor(6, CONTENT_Y + 212);
    g.print("Don't touch it.");

    draw_footer("back", "START");
    flush();
}
