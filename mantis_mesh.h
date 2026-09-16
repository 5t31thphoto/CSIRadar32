#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS MESH LAYER
//  Beacon perspectives -> chords -> an independent spatial estimate
// ═══════════════════════════════════════════════════════════════
//
//  This sits between the perspective store and the tomography, and it is
//  deliberately MODE-AGNOSTIC: Full Mode and Tactical both feed it and
//  both read it.  It knows nothing about kernels, charts, ceremonies or
//  wizards -- only about which beacons heard which, how strongly, and
//  where those beacons are.
//
//  That independence is the point.  The learned model and the mesh
//  estimate share only the raw radio; they share no priors, no
//  calibration and no failure modes.  Two estimators that can fail
//  independently are worth far more than one that cannot be checked.
//
//  ── THEY ARE NOT MERGED INTO ONE ANSWER ──────────────────────
//
//  It would be easy to average them into a single "best" position, and
//  that would throw away the most useful thing the pair produces:
//  DISAGREEMENT.
//
//    both agree      -> high confidence, and the UI can say so
//    model only      -> a response the geometry cannot see; possibly a
//                       target outside chord coverage, possibly a
//                       manifold artefact
//    mesh only       -> a genuinely blocked sight-line the model has no
//                       calibration for; likely a real target in an
//                       un-walked region
//    neither         -> the room is quiet, and two independent methods
//                       say so
//
//  Collapsing that into one number destroys four distinguishable states
//  and replaces them with an average that is wrong in a different way
//  each time.  The UI overlays them instead.
// ═══════════════════════════════════════════════════════════════
#include "mantis_report.h"
#include "mantis_tomo.h"

#define MANTIS_MESH_MAX_CHORDS 32

// Acceptance thresholds for a mesh peak.  Both must pass: a tall peak
// with too few sight-lines through it is a streak crossing, which is the
// dominant artefact of a sparse sinogram.
#ifndef MANTIS_MESH_MIN_LLR
  #define MANTIS_MESH_MIN_LLR 6.0f
#endif
#ifndef MANTIS_MESH_MIN_MARGIN
  #define MANTIS_MESH_MIN_MARGIN 1.8f
#endif

typedef struct {
    // Geometry, as measured -- never assumed.  A beacon without a
    // position contributes nothing, because a chord with a guessed
    // endpoint is not a measurement and mixing one in would corrupt the
    // reconstruction while looking like extra evidence.
    float   bx[MANTIS_SLOTS];
    float   by[MANTIS_SLOTS];
    uint8_t have_pos[MANTIS_SLOTS];
    uint8_t n_beacons;
    float   extent;            // chart half-width these coords live in

    // Per-link state, indexed [hearer][heard], 1-based ids collapsed to 0-based.
    float   atten[MANTIS_SLOTS * MANTIS_SLOTS];
    float   qual [MANTIS_SLOTS * MANTIS_SLOTS];

    // Per-link quiet-room reference.  Learned the same way the rest of
    // the system learns one: by averaging while nothing is happening.
    float   base [MANTIS_SLOTS * MANTIS_SLOTS];
    uint16_t base_n[MANTIS_SLOTS * MANTIS_SLOTS];
    bool    base_ready;

    MantisChord chords[MANTIS_MESH_MAX_CHORDS];
    int         n_chords;
    float       field[MANTIS_TOMO_CELLS];

    // Latest peak, and whether it is worth believing.
    bool    peak_valid;
    float   peak_x, peak_y, peak_llr, peak_margin;
    uint8_t peak_coverage;

    uint32_t last_seq;
    uint32_t frames;
} MantisMesh;

static inline void mantis_mesh_reset(MantisMesh *m) {
    const float ex = m->extent;
    const uint8_t n = m->n_beacons;
    float bx[MANTIS_SLOTS], by[MANTIS_SLOTS]; uint8_t hp[MANTIS_SLOTS];
    for (int i = 0; i < MANTIS_SLOTS; i++) { bx[i]=m->bx[i]; by[i]=m->by[i]; hp[i]=m->have_pos[i]; }
    *m = MantisMesh{};
    m->extent = ex; m->n_beacons = n;
    for (int i = 0; i < MANTIS_SLOTS; i++) { m->bx[i]=bx[i]; m->by[i]=by[i]; m->have_pos[i]=hp[i]; }
}

// Publish measured beacon geometry.  Called by whichever survey produced
// it -- FTM ranging, a tactical deployment walk, or a Full Mode cal.
// The mesh does not care which, only that it was MEASURED.
static inline void mantis_mesh_set_pos(MantisMesh *m, uint8_t id,
                                       float x, float y, bool measured) {
    if (id == 0 || id >= MANTIS_SLOTS) return;
    m->bx[id] = x; m->by[id] = y; m->have_pos[id] = measured ? 1 : 0;
}

// Fold one beacon's perspective in.  ASSIGNMENT, never accumulation --
// the same rule the report protocol is built on, for the same reason.
static inline void mantis_mesh_ingest(MantisMesh *m,
                                      const MantisPerspective *p,
                                      bool learning_baseline) {
    if (!p || p->reporter_id == 0 || p->reporter_id >= MANTIS_SLOTS) return;
    const uint8_t h = p->reporter_id;
    for (uint8_t i = 0; i < p->n_links; i++) {
        const MantisLinkView &lv = p->links[i];
        if (lv.peer_id == 0 || lv.peer_id >= MANTIS_SLOTS) continue;
        const int idx = h * MANTIS_SLOTS + lv.peer_id;
        const float a = mantis_unq8(lv.amp_q8);
        const float q = (float)lv.quality / 255.0f;

        if (learning_baseline) {
            // Running mean of the quiet room, per link.
            const uint16_t n = m->base_n[idx];
            m->base[idx] = (n == 0) ? a : (m->base[idx] + (a - m->base[idx]) / (float)(n + 1));
            if (m->base_n[idx] < 65535) m->base_n[idx]++;
            m->atten[idx] = 0.0f;
        } else {
            // Perturbation relative to THIS link's own quiet reference,
            // converted to ATTENUATION.
            //
            // The sign matters and I had it inverted: a beacon reports
            // the fractional amplitude CHANGE, which is NEGATIVE when a
            // body blocks the path.  Clamping `d > 0` therefore threw
            // away every real blockage and kept only constructive
            // multipath -- the tomography would have seen a permanently
            // empty room and there would have been nothing in the logs
            // to say why.
            //
            // mantis_atten_from_amp() is the single definition of that
            // conversion; see the convention note in mantis_report.h.
            const float d = a - m->base[idx];
            m->atten[idx] = mantis_atten_from_amp(d);
        }
        m->qual[idx] = q;
    }
    m->last_seq = p->seq;
}

// Is the per-link baseline usable yet?
#define MANTIS_MESH_BASE_MIN 20
static inline bool mantis_mesh_baseline_ready(const MantisMesh *m) {
    int ready = 0, total = 0;
    for (uint8_t i = 1; i <= m->n_beacons; i++)
        for (uint8_t j = 1; j <= m->n_beacons; j++) {
            if (i == j) continue;
            total++;
            if (m->base_n[i * MANTIS_SLOTS + j] >= MANTIS_MESH_BASE_MIN) ready++;
        }
    // Two thirds is enough: a link that never reports is a beacon pair
    // that cannot hear each other, and waiting for it would block
    // forever on a geometry that will never improve.
    return total > 0 && ready * 3 >= total * 2;
}

// Rebuild the chord set and reconstruct.  Cheap enough to run at the
// solve rate; the cost is n_chords * cells, and n_chords is at most 15.
static inline void mantis_mesh_solve(MantisMesh *m) {
    m->n_chords  = 0;
    m->peak_valid = false;
    if (m->n_beacons < 3 || !m->base_ready) return;

    // Pack 1-based ids into the dense 0-based form the builder wants.
    float bx[MANTIS_SLOTS], by[MANTIS_SLOTS];
    uint8_t hp[MANTIS_SLOTS];
    float at[MANTIS_SLOTS * MANTIS_SLOTS], qu[MANTIS_SLOTS * MANTIS_SLOTS];
    const uint8_t n = m->n_beacons;
    for (uint8_t i = 0; i < n; i++) {
        bx[i] = m->bx[i + 1]; by[i] = m->by[i + 1]; hp[i] = m->have_pos[i + 1];
        for (uint8_t j = 0; j < n; j++) {
            at[i * n + j] = m->atten[(i + 1) * MANTIS_SLOTS + (j + 1)];
            qu[i * n + j] = m->qual [(i + 1) * MANTIS_SLOTS + (j + 1)];
        }
    }
    m->n_chords = mantis_tomo_build(bx, by, hp, n, at, qu,
                                    m->chords, MANTIS_MESH_MAX_CHORDS);
    if (m->n_chords < 3) return;    // fewer than three lines cannot intersect

    mantis_tomo_backproject(m->chords, m->n_chords, m->extent, m->field);
    float x, y, v, mg;
    if (!mantis_tomo_peak(m->field, m->extent, &x, &y, &v, &mg)) return;

    m->peak_x = x; m->peak_y = y; m->peak_llr = v; m->peak_margin = mg;
    m->peak_coverage = (uint8_t)mantis_tomo_coverage(m->chords, m->n_chords,
                                                     m->extent, x, y);
    // TWO chords is the floor, not three.
    //
    // I set this to three first and it rejected every correct fix: on a
    // six-beacon ring an off-centre point genuinely has only two chords
    // passing within the Fresnel half-width, so the gate admitted the
    // centre and nothing else.  Measured peaks at 0.05 units of error
    // with 2.5-2.8x margins were all thrown away.
    //
    // Two intersecting lines determine a point -- that IS a fix.  The
    // risk with two is that a single bad measurement manufactures a
    // crossing, and the defence against that is the MARGIN, which asks
    // whether the peak stands clear of everything outside its own lobe.
    // Coverage guards against reconstructing where nothing looked;
    // margin guards against believing an artefact.  They are different
    // jobs and the margin is the one doing the work here.
    m->peak_valid = (v > MANTIS_MESH_MIN_LLR)
                 && (mg > MANTIS_MESH_MIN_MARGIN)
                 && (m->peak_coverage >= 2);
    m->frames++;
}

// ── Concordance ───────────────────────────────────────────────
// How the two independent estimates relate.  This is reported, not
// resolved: the UI draws both and says which case it is in.
typedef enum : uint8_t {
    MC_QUIET     = 0,   // neither sees anything
    MC_AGREE     = 1,   // both, same place -> strongest possible evidence
    MC_MODEL_ONLY= 2,   // learned model only -> outside chord coverage, or artefact
    MC_MESH_ONLY = 3,   // geometry only -> real blockage the model never learned
    MC_CONFLICT  = 4,   // both, different places -> one of them is wrong
} MantisConcord;

static inline MantisConcord mantis_mesh_concord(const MantisMesh *m,
                                                bool model_has, float mx, float my,
                                                float agree_radius) {
    if (!m->peak_valid && !model_has) return MC_QUIET;
    if (!m->peak_valid)               return MC_MODEL_ONLY;
    if (!model_has)                   return MC_MESH_ONLY;
    const float dx = mx - m->peak_x, dy = my - m->peak_y;
    return (dx * dx + dy * dy <= agree_radius * agree_radius) ? MC_AGREE : MC_CONFLICT;
}
