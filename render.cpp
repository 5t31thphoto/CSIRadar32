// ═══════════════════════════════════════════════════════════════
//  render.cpp
// ═══════════════════════════════════════════════════════════════
#include "render.h"
#include "scene.h"
#include "lgfx_tdisplay_s3.h"
#include <math.h>

// Access to the shared drawing surface owned by ui.cpp.  Returns the
// sprite when double-buffered, or the direct LCD when not.
extern LovyanGFX &gfx_sprite();

// ── Mini oscilloscope buffer (bottom strip) ────────────────────
#define RADAR_SCOPE_LEN 160
static float s_scope[MAX_BEACONS][RADAR_SCOPE_LEN];
static int   s_scope_head = 0;
static bool  s_scope_init = false;

// ── Track colors (stable per-track from the MantisSec channel palette) ─
static uint16_t track_color(uint8_t id) {
    static const uint16_t palette[] = {
        COL_MS_CH_A,   // lime
        COL_MS_CH_B,   // bright teal
        COL_MS_CH_C,   // amber
        COL_MS_CH_D,   // violet-magenta
    };
    return palette[id % 4];
}

// Occupancy heatmap: dim violet → teal → electric lime.
// Two-segment interpolation from MantisSec palette.
static uint16_t heatmap_color(float v) {
    if (v <= 0) return COL_MS_BG;
    if (v > 1)  v = 1;
    // Interpolate MS_VIOLET (dark) → MS_TEAL (mid) → MS_LIME (hot)
    if (v < 0.5f) {
        // violet → teal
        float t = v * 2.0f;
        uint8_t r = (uint8_t)((1 - t) * 0x5D + t * 0x00);
        uint8_t g = (uint8_t)((1 - t) * 0x00 + t * 0x73);
        uint8_t b = (uint8_t)((1 - t) * 0x5D + t * 0x73);
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    } else {
        // teal → lime
        float t = (v - 0.5f) * 2.0f;
        uint8_t r = (uint8_t)((1 - t) * 0x00 + t * 0xB8);
        uint8_t g = (uint8_t)((1 - t) * 0x73 + t * 0xE6);
        uint8_t b = (uint8_t)((1 - t) * 0x73 + t * 0x00);
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }
}

// Draw a rotated 2σ ellipse from a 2×2 covariance.
static void draw_covariance_ellipse(int cx, int cy, float pix_per_unit,
                                    float vxx, float vyy, float vxy,
                                    uint16_t col) {
    auto &g = gfx_sprite();
    float tr = vxx + vyy;
    float det = vxx * vyy - vxy * vxy;
    float disc = tr * tr * 0.25f - det;
    if (disc < 0) disc = 0;
    float sd = sqrtf(disc);
    float l1 = tr * 0.5f + sd, l2 = tr * 0.5f - sd;
    if (l1 < 0.001f) l1 = 0.001f;
    if (l2 < 0.001f) l2 = 0.001f;
    float ang;
    if (fabsf(vxy) < 1e-6f) ang = (vxx >= vyy) ? 0.0f : (float)M_PI * 0.5f;
    else                    ang = atan2f(l1 - vxx, vxy);
    float ca = cosf(ang), sa = sinf(ang);
    float ax = 2.0f * sqrtf(l1) * pix_per_unit;
    float ay = 2.0f * sqrtf(l2) * pix_per_unit;
    if (ax > 60) ax = 60;
    if (ay > 60) ay = 60;
    if (ax < 3)  ax = 3;
    if (ay < 3)  ay = 3;
    const int N = 20;
    int prev_x = 0, prev_y = 0;
    for (int i = 0; i <= N; i++) {
        float t = (float)i / (float)N * 2.0f * (float)M_PI;
        float ux = ax * cosf(t), uy = ay * sinf(t);
        int px = cx + (int)(ca * ux - sa * uy);
        int py = cy - (int)(sa * ux + ca * uy);
        if (i > 0) g.drawLine(prev_x, prev_y, px, py, col);
        prev_x = px; prev_y = py;
    }
}

// ═══════════════════════════════════════════════════════════════
//  LIFECYCLE
// ═══════════════════════════════════════════════════════════════
void render_begin() {
    s_scope_init = false;
    s_scope_head = 0;
}

// ═══════════════════════════════════════════════════════════════
//  PRIMARY RADAR VIEW
// ═══════════════════════════════════════════════════════════════
void render_radar_view() {
    auto &g = gfx_sprite();

    // Layout
    const int status_y = CONTENT_Y + 2;
    const int map_y    = CONTENT_Y + 18;
    const int map_h    = 190;
    const int scope_y  = map_y + map_h + 4;
    const int scope_h  = 44;

    // ── Status band: mode + track count + confidence ──
    g.setFont(&fonts::Font2);
    const char *mode_lbl = "?";
    uint16_t mode_col = COL_MS_DIM;
    switch (g_app.peer.role) {
        case ROLE_PRIMARY:
            mode_lbl = g_app.peer.peer_present ? "STEREO" : "STEREO?";
            mode_col = g_app.peer.peer_present ? COL_MS_LIME : COL_MS_WARN;
            break;
        case ROLE_SECONDARY: mode_lbl = "SECONDARY"; mode_col = COL_MS_WARN; break;
        case ROLE_SOLO:      mode_lbl = "SOLO";      mode_col = COL_MS_TEAL; break;
        default:             mode_lbl = ".."; break;
    }
    g.setTextColor(mode_col, COL_MS_BG);
    g.setCursor(4, status_y);
    g.print(mode_lbl);

    int n_tracks = scene_active_track_count();
    char tbuf[16];
    snprintf(tbuf, sizeof(tbuf), "%d tgt", n_tracks);
    g.setTextColor(n_tracks > 0 ? COL_MS_LIME : COL_MS_DIM, COL_MS_BG);
    int tw = g.textWidth(tbuf);
    g.setCursor((SCREEN_W - tw) / 2, status_y);
    g.print(tbuf);

    // Aggregate confidence = max track confidence
    float max_conf = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        const TargetTrack *t = scene_get_track(i);
        if (!t || !t->active) continue;
        if (t->confidence > max_conf) max_conf = t->confidence;
    }
    char cbuf[8];
    snprintf(cbuf, sizeof(cbuf), "%3d%%", (int)(max_conf * 100.0f));
    int cw = g.textWidth(cbuf);
    uint16_t cc = max_conf > 0.5f ? COL_MS_LIME
                : max_conf > 0.2f ? COL_MS_WARN : COL_MS_DIM;
    g.setTextColor(cc, COL_MS_BG);
    g.setCursor(SCREEN_W - cw - 4, status_y);
    g.print(cbuf);

    // ── Radar map ──
    const int cx = SCREEN_W / 2;
    const int cy = map_y + map_h / 2;
    // Fit the scene-extent range into the smaller of (map w/2, map h/2)
    const int usable_h = (SCREEN_W / 2) - 6;
    const int usable_v = (map_h / 2) - 6;
    const int R = (usable_h < usable_v) ? usable_h : usable_v;
    const float pix_per_unit = (float)R / SCENE_EXTENT;

    // Frame + range rings
    g.drawRect(0, map_y, SCREEN_W, map_h, COL_MS_DIM);
    for (float r_unit = 0.5f; r_unit <= SCENE_EXTENT; r_unit += 0.5f) {
        int rp = (int)(r_unit * pix_per_unit);
        g.drawCircle(cx, cy, rp, COL_MS_DIM);
    }
    // Axes
    g.drawFastHLine(cx - R, cy, 2 * R, COL_MS_DIM);
    g.drawFastVLine(cx, cy - R, 2 * R, COL_MS_DIM);
    g.setTextColor(COL_MS_MID, COL_MS_BG);
    g.setCursor(cx + 2, cy - R + 1);
    g.print("N");

    // ── Occupancy field heatmap ──
    // Draw sub-cells as fillRect.  Grid dim = 24; field cell size in pixels
    // = (2R) / 24.  Skip cells below a threshold to reduce visual noise.
    const OccupancyField *field = scene_get_field();
    if (field && field->valid && field->cell_max > 0.01f) {
        int cell_px = (2 * R) / FIELD_DIM;
        if (cell_px < 1) cell_px = 1;
        int origin_x = cx - R;
        int origin_y = cy - R;
        for (int gy = 0; gy < FIELD_DIM; gy++) {
            for (int gx = 0; gx < FIELD_DIM; gx++) {
                float v = field->cell[gx][gy] / field->cell_max;
                if (v < 0.15f) continue;   // suppress low-signal cells
                // Screen coords: field's +x maps to +x on screen, +y maps
                // to UP (so we invert)
                int px = origin_x + gx * cell_px;
                int py = origin_y + (FIELD_DIM - 1 - gy) * cell_px;
                g.fillRect(px, py, cell_px, cell_px, heatmap_color(v));
            }
        }
    }

    // ── Beacons ──
    // Convert normalized geometry to screen coords.  Use scene's derived
    // landmark positions (LM_BEACON_1..3) if cal has produced them, else
    // fall back to raw BeaconState.pos_x/y (which lives in unnormalized
    // physical cm).
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        float bx_n, by_n;
        // Match beacon slot i (0,1,2) to LM_BEACON_i+1
        if (i == 0) scene_landmark_pos(LM_BEACON_1, &bx_n, &by_n);
        else if (i == 1) scene_landmark_pos(LM_BEACON_2, &bx_n, &by_n);
        else if (i == 2) scene_landmark_pos(LM_BEACON_3, &bx_n, &by_n);
        else { bx_n = 0; by_n = 0; }
        int bx = cx + (int)(bx_n * pix_per_unit);
        int by = cy - (int)(by_n * pix_per_unit);

        // Link line RX→beacon, color by disturbance level
        uint16_t lc = COL_MS_DIM;
        if (b.link_metric_ema > 0.4f) lc = COL_MS_WARN;
        if (b.link_metric_ema > 0.7f) lc = COL_MS_ALERT;
        g.drawLine(cx, cy, bx, by, lc);

        // Beacon dot
        uint16_t bc = COL_MS_LIME;
        if (b.status == LS_MOTION)   bc = COL_MS_WARN;
        if (b.status == LS_PRESENCE) bc = COL_MS_ALERT;
        g.fillCircle(bx, by, 4, bc);
        g.drawCircle(bx, by, 4, COL_MS_INK);
        g.setFont(&fonts::Font2);
        g.setTextColor(COL_MS_MID, COL_MS_BG);
        char lbl[6]; snprintf(lbl, sizeof(lbl), "%u", (unsigned)b.id);
        g.setCursor(bx + 6, by - 8);
        g.print(lbl);
    }

    // ── AoA rays (stereo mode) ──
    if (g_app.peer.role == ROLE_PRIMARY && g_app.peer.peer_present) {
        uint32_t now = millis();
        for (int i = 0; i < MAX_BEACONS; i++) {
            BeaconState &b = g_app.beacon[i];
            if (!b.active) continue;
            if (b.aoa_conf < 0.05f) continue;
            if ((now - b.last_aoa_ms) > 500) continue;
            float a = b.aoa_rad;
            int ex = cx + (int)(R * sinf(a));
            int ey = cy - (int)(R * cosf(a));
            // Faint teal ray
            g.drawLine(cx, cy, ex, ey, COL_MS_TEAL);
        }
    }

    // ── RX icon ──
    if (g_app.peer.role == ROLE_PRIMARY && g_app.peer.peer_present) {
        // Stereo pair: two small squares 6cm baseline visualized as ~4px apart
        int half = 3;
        g.fillRect(cx - half - 2, cy - 2, 4, 5, COL_MS_TEAL);
        g.fillRect(cx + half - 2, cy - 2, 4, 5, COL_MS_TEAL);
        g.drawLine(cx - half, cy, cx + half, cy, COL_MS_TEAL);
    } else {
        g.fillRect(cx - 3, cy - 3, 6, 7, COL_MS_TEAL);
    }

    // ── Target tracks (dots + ellipses + trails) ──
    for (int i = 0; i < TRACK_MAX; i++) {
        const TargetTrack *t = scene_get_track(i);
        if (!t || !t->active) continue;
        uint16_t tc = track_color(t->id);

        // Trail (age-faded)
        uint32_t now = millis();
        int start = t->trail_head;
        int total = t->trail_count;
        for (int k = 0; k < total; k++) {
            int idx = (start + TRACK_TRAIL_LEN - total + k) % TRACK_TRAIL_LEN;
            uint32_t age = now - t->trail[idx].t_ms;
            if (age > 3000) continue;
            int px = cx + (int)(t->trail[idx].pos[0] * pix_per_unit);
            int py = cy - (int)(t->trail[idx].pos[1] * pix_per_unit);
            uint16_t dc = (age < 500) ? tc
                        : (age < 1500) ? COL_MS_DIM
                                       : COL_MS_MID;
            int rad = (age < 500) ? 2 : 1;
            g.fillCircle(px, py, rad, dc);
        }

        // Confidence ellipse — v0.5: derived from real EKF covariance.
        // The track's cov_xx/cov_yy/cov_xy are now maintained by the
        // Kalman update as actual position uncertainty in normalized
        // units².  Alias-flagged tracks have their covariance floor
        // inflated by scene.cpp, so ambiguous regions naturally show
        // wider ellipses without special-casing here.
        int tx = cx + (int)(t->pos[0] * pix_per_unit);
        int ty = cy - (int)(t->pos[1] * pix_per_unit);
        float cxx = fmaxf(0.005f, t->cov_xx);
        float cyy = fmaxf(0.005f, t->cov_yy);
        float cxy = t->cov_xy;
        // Ambiguity-flagged tracks get an alert-color ellipse to
        // visually communicate uncertainty.
        uint16_t ellipse_col = t->ambiguity_flag ? COL_MS_WARN : COL_MS_DIM;
        draw_covariance_ellipse(tx, ty, pix_per_unit, cxx, cyy, cxy, ellipse_col);

        // Target rings
        g.drawCircle(tx, ty, 6, tc);
        g.drawCircle(tx, ty, 3, tc);
        g.fillCircle(tx, ty, 2, COL_MS_INK);
        // Ambiguity marker — small alert dot near the target
        if (t->ambiguity_flag) {
            g.fillCircle(tx + 8, ty - 8, 2, COL_MS_ALERT);
        }
    }

    // ── Alert border ──
    if (g_app.alert_latched && (millis() - g_app.last_alert_ms) < 4000) {
        g.drawRect(0, map_y, SCREEN_W, map_h, COL_MS_ALERT);
        g.drawRect(1, map_y + 1, SCREEN_W - 2, map_h - 2, COL_MS_ALERT);
    }

    // ── Mini oscilloscope strip ──
    if (!s_scope_init) {
        for (int i = 0; i < MAX_BEACONS; i++)
            for (int k = 0; k < RADAR_SCOPE_LEN; k++) s_scope[i][k] = 0;
        s_scope_init = true;
    }
    for (int i = 0; i < MAX_BEACONS; i++)
        s_scope[i][s_scope_head] = g_app.beacon[i].active ? g_app.beacon[i].link_metric_ema : 0;
    s_scope_head = (s_scope_head + 1) % RADAR_SCOPE_LEN;

    g.drawRect(0, scope_y, SCREEN_W, scope_h, COL_MS_DIM);
    int mid = scope_y + scope_h / 2;
    g.drawFastHLine(1, mid, SCREEN_W - 2, COL_MS_DIM);
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MS_MID, COL_MS_BG);
    g.setCursor(4, scope_y + 2);
    g.print("links");
    if (g_app.peer.role == ROLE_PRIMARY && g_app.peer.peer_present) {
        char lo[16];
        snprintf(lo, sizeof(lo), "LO %+.2f", (double)g_app.peer.lo_drift_ema);
        int lw = g.textWidth(lo);
        g.setCursor(SCREEN_W - lw - 4, scope_y + 2);
        g.print(lo);
    }
    uint16_t bcols[3] = {COL_MS_CH_A, COL_MS_CH_B, COL_MS_CH_C};
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        uint16_t c = bcols[i % 3];
        int n = RADAR_SCOPE_LEN < SCREEN_W ? RADAR_SCOPE_LEN : SCREEN_W;
        int prev_x = 0, prev_y = 0;
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
}

// ═══════════════════════════════════════════════════════════════
//  FIELD DEBUG VIEW  — raw occupancy grid + kernel sample overlay
// ═══════════════════════════════════════════════════════════════
void render_field_view() {
    auto &g = gfx_sprite();
    const int map_y = CONTENT_Y + 4;
    const int map_h = 240;
    const int cx = SCREEN_W / 2;
    const int cy = map_y + map_h / 2;
    const int R = (SCREEN_W / 2) - 4;
    const float pix_per_unit = (float)R / SCENE_EXTENT;

    g.drawRect(0, map_y, SCREEN_W, map_h, COL_MS_DIM);

    // Full-opacity occupancy field
    const OccupancyField *field = scene_get_field();
    if (field && field->valid && field->cell_max > 0.001f) {
        int cell_px = (2 * R) / FIELD_DIM;
        if (cell_px < 1) cell_px = 1;
        int origin_x = cx - R;
        int origin_y = cy - R;
        for (int gy = 0; gy < FIELD_DIM; gy++) {
            for (int gx = 0; gx < FIELD_DIM; gx++) {
                float v = field->cell[gx][gy] / field->cell_max;
                if (v < 0.05f) continue;
                int px = origin_x + gx * cell_px;
                int py = origin_y + (FIELD_DIM - 1 - gy) * cell_px;
                g.fillRect(px, py, cell_px, cell_px, heatmap_color(v));
            }
        }
    }

    // Kernel sample overlay: tiny dots at each landmark/transit sample.
    // Landmarks = larger + lime, transit samples = tiny + teal.
    int n = scene_kernel_sample_count();
    for (int i = 0; i < n; i++) {
        const KernelSample *s = scene_kernel_sample(i);
        if (!s) continue;
        int px = cx + (int)(s->pos[0] * pix_per_unit);
        int py = cy - (int)(s->pos[1] * pix_per_unit);
        if (s->landmark_id != 0xFF) {
            g.fillCircle(px, py, 2, COL_MS_LIME);
        } else {
            g.fillRect(px, py, 1, 1, COL_MS_TEAL);
        }
    }

    // Diagnostics at bottom
    g.setFont(&fonts::Font2);
    g.setTextColor(COL_MS_MID, COL_MS_BG);
    char buf[32];
    snprintf(buf, sizeof(buf), "kernel %d", n);
    g.setCursor(4, map_y + map_h + 4);
    g.print(buf);
    snprintf(buf, sizeof(buf), "novelty %.2f", (double)scene_novelty_score());
    g.setCursor(4, map_y + map_h + 18);
    g.print(buf);
}
