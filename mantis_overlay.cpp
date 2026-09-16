// ═══════════════════════════════════════════════════════════════
//  MANTIS MESH OVERLAY  —  the cowitness underglow
// ═══════════════════════════════════════════════════════════════
//
//  The mesh is a second, independent instrument.  Drawing its sight-
//  lines made the method legible and was the wrong call: a lattice of
//  beams is a DIAGRAM of how it works, it competes with the tracks for
//  attention, and it gives the operator nothing they can act on.
//
//  What matters is whether the two instruments AGREE, so the mesh
//  renders as a soft warm field beneath everything else -- an underglow.
//
//      track sitting in its own glow  -> both methods, same place
//      glow with no track             -> geometry witnesses something
//                                        the learned model never learned
//      track on unlit floor           -> the model claims something the
//                                        geometry cannot corroborate
//
//  You read all three without reading a number.
//
//  DRAWN FIRST, ALWAYS.  The glow is backdrop; the tracks are subject.
//  Anything rendered on TOP of a track competes with it, and this layer
//  is corroboration -- not a second opinion shouted over the first.
//
//  GRAMMAR
//      warm amber, soft, no hard edges    the mesh
//      lime / teal / violet, crisp orbs   the tracks
//  Different hue AND different texture, so they read as separate layers
//  at a glance instead of as one confused picture.
// ═══════════════════════════════════════════════════════════════
#include "config.h"
#include "mantis_mesh.h"
#include "lgfx_tdisplay_s3.h"
#include <math.h>

extern LovyanGFX &gfx_sprite();

// Warm ramp, reserved for this layer.  Deliberately no green or cyan
// anywhere in it -- those belong to the track layer.
#ifndef COL_MESH_C0
  #define COL_MESH_C0  0x2060   // barely-there ember
  #define COL_MESH_C1  0x6180   // dull orange
  #define COL_MESH_C2  0xB2C0   // amber
  #define COL_MESH_C3  0xFD60   // hot amber
#endif

static uint16_t mesh_ramp(float t) {
    // Piecewise, with a dead low end.  A faint glow spread everywhere
    // would read as confidence the mesh does not have.
    if (t < 0.34f) return COL_MESH_C0;
    if (t < 0.62f) return COL_MESH_C1;
    if (t < 0.85f) return COL_MESH_C2;
    return COL_MESH_C3;
}

// The underglow.  Call BEFORE drawing tracks.
//
// cx/cy/radius_px must be the same circle the host view uses, so the two
// layers share one coordinate frame and cannot drift apart.
void mantis_mesh_glow(const MantisMesh *m, int cx, int cy, int radius_px) {
    if (!m || m->n_chords <= 0 || m->extent <= 0.0f) return;
    auto &g = gfx_sprite();

    // Normalise against the LIVE peak, not a fixed scale.  Absolute
    // log-odds depend on how many chords are alive, so a fixed scale
    // would dim the whole glow every time a beacon dropped out -- the
    // display would appear to lose confidence when only the arithmetic
    // had changed.
    float hi = 0.0f;
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++)
        if (m->field[i] > hi) hi = m->field[i];
    if (hi < MANTIS_MESH_MIN_LLR * 0.5f) return;   // nothing worth showing

    const float step_units = (2.0f * m->extent) / (float)MANTIS_TOMO_DIM;
    const float scale      = (float)radius_px / m->extent;
    const int   cell_px    = (int)(step_units * scale) + 1;

    // Two passes make it bloom rather than pixelate: a wide dim pass
    // lays a halo, a tighter bright pass gives it a core.  Overlap
    // between neighbouring hot cells does the blending for free, which
    // is why this stays cheap enough for every frame.
    for (int pass = 0; pass < 2; pass++) {
        const int   r    = (pass == 0) ? (cell_px + 3) : (cell_px - 1);
        const float gate = (pass == 0) ? 0.30f : 0.58f;
        if (r <= 0) continue;

        for (int gy = 0; gy < MANTIS_TOMO_DIM; gy++) {
            for (int gx = 0; gx < MANTIS_TOMO_DIM; gx++) {
                const float v = m->field[gy * MANTIS_TOMO_DIM + gx];
                if (v <= 0.0f) continue;
                const float t = v / hi;
                if (t < gate) continue;

                const float ux = -m->extent + ((float)gx + 0.5f) * step_units;
                const float uy =  m->extent - ((float)gy + 0.5f) * step_units;
                const int   px = cx + (int)(ux * scale);
                const int   py = cy - (int)(uy * scale);

                // Stay inside the radar circle.  Glow spilling past the
                // edge would imply coverage that does not exist.
                const int dx = px - cx, dy = py - cy;
                if (dx * dx + dy * dy > radius_px * radius_px) continue;

                g.fillCircle(px, py, r, mesh_ramp(pass == 0 ? t * 0.55f : t));
            }
        }
    }
}

// A faint reticle at the mesh's own peak, ONLY when the model has
// nothing there.
//
// When both agree, the glow under the orb already says so; a second
// marker on top of a track is exactly the competition this layer exists
// to avoid.
void mantis_mesh_mark_uncorroborated(const MantisMesh *m, MantisConcord c,
                                     int cx, int cy, int radius_px) {
    if (!m || !m->peak_valid || c != MC_MESH_ONLY) return;
    auto &g = gfx_sprite();
    const float scale = (float)radius_px / m->extent;
    const int px = cx + (int)(m->peak_x * scale);
    const int py = cy - (int)(m->peak_y * scale);
    const uint16_t col = ((millis() / 500) % 2) ? COL_MESH_C3 : COL_MESH_C2;
    g.drawCircle(px, py, 9,  col);
    g.drawCircle(px, py, 10, col);
}

// One short line naming which of the five states we are in.
//
// Reported, never resolved.  "DISAGREE" is not a failure to hide -- it
// is two independent instruments contradicting each other, which is
// exactly when an operator most needs to know.
void mantis_mesh_badge(const MantisMesh *m, MantisConcord c, int x, int y) {
    auto &g = gfx_sprite();
    const char *label; uint16_t col;
    switch (c) {
        case MC_AGREE:      label = "CONFIRMED";   col = COL_MS_LIME;  break;
        case MC_MESH_ONLY:  label = "MESH ONLY";   col = COL_MESH_C3;  break;
        case MC_MODEL_ONLY: label = "UNWITNESSED"; col = COL_MS_TEAL;  break;
        case MC_CONFLICT:   label = "DISAGREE";    col = COL_MS_ALERT; break;
        default:            label = "CLEAR";       col = COL_MS_MID;   break;
    }
    g.setFont(&fonts::Font0);
    g.setTextColor(col, COL_MS_BG);
    g.setCursor(x, y);
    g.print(label);

    // When the mesh has an opinion, surface the two numbers that decided
    // whether its peak was believed at all.
    if (m && m->peak_valid) {
        char s[24];
        snprintf(s, sizeof(s), "%.1fx %dch", (double)m->peak_margin,
                 (int)m->peak_coverage);
        g.setTextColor(COL_MS_MID, COL_MS_BG);
        g.setCursor(x, y + 9);
        g.print(s);
    }
}
