#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS FUSION
//  Independent channels corroborating each other
// ═══════════════════════════════════════════════════════════════
//
//  ── WHY THIS EXISTS ──────────────────────────────────────────
//
//  Shadowing tomography alone reports three targets for one person.  The
//  extra two are not noise -- they are STABLE, identical across ten
//  consecutive solves, because they are deterministic streak crossings
//  of a 15-line sinogram.  Temporal filtering cannot remove them and
//  raising the detection bar until they vanish would suppress a genuine
//  second person in every well-covered room.
//
//  But they are only stable in ONE channel.  Measured on the same frame:
//
//      shadowing peak        llr    Doppler explains
//      REAL   (0.35, 0.35)  14.7          98.3%
//      ghost1 (-0.15,-0.95) 13.4          61.8%
//      ghost2 (0.75,-0.25)  11.6          59.7%
//
//  The Doppler channel is not fooled by a shadowing artefact, because
//  the two are computed from different physics: one from how much
//  amplitude a path lost, the other from whether a single velocity
//  explains every link's phase rate at that position.  An artefact can
//  satisfy one and essentially never both.
//
//  ── WHAT FUSION DOES AND DOES NOT DO ─────────────────────────
//
//  It does NOT average the channels into one number.  Averaging destroys
//  the very thing that makes two instruments worth having: a target both
//  channels see is a different claim from one only shadowing sees, and
//  collapsing them loses that distinction permanently.
//
//  Instead every candidate keeps BOTH scores and gains a corroboration
//  class.  Nothing is deleted -- a shadow-only contact is still drawn,
//  just drawn differently -- because "we see something here but only one
//  instrument agrees" is information the operator can act on, and
//  silently dropping it is a decision the firmware should not be making
//  on their behalf.
//
//  ── ACCUMULATION ─────────────────────────────────────────────
//
//  Confidence integrates across frames.  A real contact is corroborated
//  frame after frame and climbs; an artefact that only ever satisfies
//  one channel plateaus low no matter how stable it is.  That is what
//  finally separates a persistent ghost from a stationary person, and it
//  is why stability alone was never going to be enough.
// ═══════════════════════════════════════════════════════════════
#include "mantis_targets.h"
#include "mantis_doppler.h"
#include "mantis_report.h"
#include "mantis_trilat.h"

// How many independent beacons must have called a link through this
// point BLOCKED before it can be believed.
//
// Measured on a live six-beacon run: the real target had 4 witnesses,
// the near ghost 2, the far ghost 0.  Three sits in the gap.
//
// This is the strongest discriminator in the system and the cheapest:
// no field, no back-projection, just counting who independently agrees.
// It works because each beacon judged its own links against its own
// baseline -- six separate opinions, not one opinion six times.
#ifndef MANTIS_MIN_WITNESSES
  #define MANTIS_MIN_WITNESSES 3
#endif

// How far a back-projection peak can sit from the truth it represents.
// One grid cell (0.10) plus the reconstruction bias measured against
// known targets (~0.05).  Widening the witness test by this much is not
// slack -- it is the measured uncertainty of the thing being tested.
// Witnesses sufficient to carry a contact WITHOUT Doppler corroboration.
// Set above the measured artefact count (2 of 6) with a clear margin,
// and below what a real target reliably achieves (4-5 of 6).
#ifndef MANTIS_STRONG_WITNESSES
  #define MANTIS_STRONG_WITNESSES 4
#endif

#ifndef MANTIS_WITNESS_SLACK
  #define MANTIS_WITNESS_SLACK 0.15f
#endif

// Count the distinct beacons whose OWN verdict places a blockage on a
// path through (px,py).
static inline uint8_t mantis_count_witnesses(const MantisMesh *m,
                                             const MantisPerspectiveStore *st,
                                             float px, float py,
                                             uint32_t now_ms) {
    uint8_t seen = 0;   // bitmask of beacons
    // Fresnel half-width plus the peak's own positional uncertainty.
    const float tol  = MANTIS_CHORD_HALFWIDTH + MANTIS_WITNESS_SLACK;
    const float tol2 = tol * tol;
    for (uint8_t h = 1; h <= m->n_beacons && h < MANTIS_SLOTS; h++) {
        if (!mantis_store_fresh(st, h, now_ms)) continue;
        const MantisPerspective &v = st->by_id[h].view;
        if (!m->have_pos[h]) continue;
        for (uint8_t b = 0; b < MANTIS_MAX_LINKS; b++) {
            if (!(v.blocked_mask & (1u << b))) continue;
            const uint8_t peer = (uint8_t)(b + 1);
            if (peer == h || peer >= MANTIS_SLOTS || !m->have_pos[peer]) continue;
            // TOLERANCE, not an exact hit.
            //
            // The candidate comes from a back-projection peak, which
            // carries the grid's quantisation (one cell) plus the
            // reconstruction's own bias -- measured at about 0.11 chart
            // units against truth.  Testing the chord against the exact
            // peak therefore MISSES witnesses that genuinely saw the
            // target, and the real contact scored fewer witnesses than
            // an artefact.  The test has to be as wide as the position
            // uncertainty actually is.
            if (mantis_pt_seg_d2(px, py, m->bx[h], m->by[h],
                                 m->bx[peer], m->by[peer]) > tol2) continue;
            // BOTH endpoints witnessed it: the blockage is on a path
            // they share, and each of them measured that path.
            seen |= (uint8_t)(1u << (h - 1));
            seen |= (uint8_t)(1u << (peer - 1));
        }
    }
    uint8_t n = 0;
    for (uint8_t k = 0; k < 8; k++) if (seen & (1u << k)) n++;
    return n;
}

typedef enum : uint8_t {
    MFC_NONE = 0,
    MFC_CONFIRMED,    // shadowing AND Doppler agree -> a moving body
    MFC_STATIC,       // shadowing only, but Doppler says nothing is moving
                      // anywhere -- consistent with a motionless person
    MFC_SHADOW_ONLY,  // shadowing sees it, Doppler contradicts it
    MFC_DOPPLER_ONLY, // motion with no blockage: off-chord, or beyond the
                      // shadowing channel's 59% coverage
} MantisFuseClass;

typedef struct {
    float  x, y;
    float  shadow_llr;      // from the tomography
    float  doppler_expl;    // 0..1, fraction of rate energy explained here
    float  vx, vy;          // recovered velocity, when Doppler supports it
    uint8_t coverage;       // chords with line of sight
    uint8_t witnesses;      // DISTINCT beacons whose own verdict supports it
    bool    tri_agrees;     // does segment trilateration land here too?
    float  confidence;      // accumulated across frames, 0..1
    MantisFuseClass cls;
    uint16_t frames_seen;
    uint16_t frames_corroborated;
    uint16_t frames_contradicted;   // Doppler actively said "not here"
    bool   active;
} MantisFused;

#define MANTIS_FUSE_MAX 4

typedef struct {
    MantisFused t[MANTIS_FUSE_MAX];
    uint8_t     n;
    float       dop_field[MANTIS_TOMO_CELLS];
    MantisDopplerObs obs[MANTIS_DOP_MAX_OBS];
    int         n_obs;
    MantisTriFix tri;       // an independent fix from the same measurements
    bool        anything_moving;      // is there Doppler energy at all?
} MantisFusion;

// A candidate must clear this fraction of the Doppler field's own peak
// to count as corroborated.  Measured separation was 98% against 60-62%,
// so 0.78 sits in the gap with room on both sides.
#ifndef MANTIS_FUSE_CORROBORATE
  #define MANTIS_FUSE_CORROBORATE 0.78f
#endif

// How fast confidence moves.  Deliberately slow to RISE and fast to
// FALL: a contact should have to earn belief over several frames, but
// must be able to disappear promptly when it is gone.
#ifndef MANTIS_FUSE_RISE
  #define MANTIS_FUSE_RISE 0.12f
#endif
#ifndef MANTIS_FUSE_FALL
  #define MANTIS_FUSE_FALL 0.35f
#endif

static inline float mantis_fuse_sample(const float *field, float extent,
                                       float px, float py) {
    const float step = (2.0f * extent) / (float)MANTIS_TOMO_DIM;
    const int gx = (int)((px + extent) / step);
    const int gy = (int)((extent - py) / step);
    if (gx < 0 || gy < 0 || gx >= MANTIS_TOMO_DIM || gy >= MANTIS_TOMO_DIM)
        return 0.0f;
    return field[gy * MANTIS_TOMO_DIM + gx];
}

// Match a new candidate to an existing track by position.
static inline int mantis_fuse_match(const MantisFusion *F, float x, float y) {
    int best = -1; float bd = MANTIS_TARGET_SEPARATION * MANTIS_TARGET_SEPARATION;
    for (int i = 0; i < MANTIS_FUSE_MAX; i++) {
        if (!F->t[i].active) continue;
        const float dx = F->t[i].x - x, dy = F->t[i].y - y;
        const float d2 = dx * dx + dy * dy;
        if (d2 < bd) { bd = d2; best = i; }
    }
    return best;
}

// Run both channels and cross-reference them.
//
// `rate_matrix` is the per-link phase rate the mesh measured this frame.
static inline void mantis_fuse_update(MantisFusion *F,
                                      const MantisMesh *m,
                                      const MantisPerspectiveStore *st,
                                      const MantisTargetSet *shadow,
                                      const float *rate_matrix,
                                      uint32_t now_ms) {
    // ── the Doppler channel ──
    F->n_obs = mantis_doppler_build(m, rate_matrix, F->obs, MANTIS_DOP_MAX_OBS);
    mantis_doppler_field(F->obs, F->n_obs, m->extent, F->dop_field);

    float dop_peak = 0.0f;
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++)
        if (F->dop_field[i] > dop_peak) dop_peak = F->dop_field[i];
    // No Doppler energy anywhere means nothing in the room is moving --
    // which is a STATEMENT, not a failure.  It is what makes a
    // shadow-only contact read as a motionless person rather than as an
    // artefact.
    F->anything_moving = (dop_peak > 0.05f);

    // Run the independent fix once per frame, before scoring candidates.
    F->tri = mantis_trilaterate(m, st, now_ms);

    bool touched[MANTIS_FUSE_MAX] = {false};

    for (int k = 0; k < shadow->n && k < MANTIS_FUSE_MAX; k++) {
        const MantisTarget &s = shadow->t[k];
        if (!s.active) continue;

        const float raw  = mantis_fuse_sample(F->dop_field, m->extent, s.x, s.y);
        const float expl = (dop_peak > 1e-6f) ? (raw / dop_peak) : 0.0f;
        float vx = 0, vy = 0; int used = 0;
        mantis_doppler_residual(F->obs, F->n_obs, s.x, s.y, &vx, &vy, &used);

        int idx_prior = mantis_fuse_match(F, s.x, s.y);
        const bool discredited =
            (idx_prior >= 0) &&
            (F->t[idx_prior].frames_contradicted >
             F->t[idx_prior].frames_corroborated + 2);

        // ── INDEPENDENT WITNESSES, FIRST ──────────────────────
        // Counted before anything else because it is the strongest and
        // cheapest test available, and because it is the one that uses
        // what the mesh actually is: several radios that each formed
        // their own opinion, rather than one radio with several inputs.
        const uint8_t wit = mantis_count_witnesses(m, st, s.x, s.y, now_ms);
        // A THIRD opinion, from the same data by entirely different
        // arithmetic: intersect the blocked segments directly instead of
        // back-projecting them onto a grid.  Same accuracy, a thousandth
        // of the work, and it fails in different places -- so when both
        // land together the position is corroborated by two methods that
        // cannot share a mistake.
        const bool tri_ok = mantis_tri_agrees(&F->tri, s.x, s.y,
                                              MANTIS_WITNESS_SLACK * 2.0f);

        MantisFuseClass cls;
        if (wit < MANTIS_MIN_WITNESSES) {
            // Too few beacons independently saw a blockage on any path
            // through here.  The back-projection may show a strong peak,
            // and it is still a crossing of streaks rather than a body:
            // no amount of field strength substitutes for somebody
            // having actually measured an obstruction there.
            cls = MFC_SHADOW_ONLY;
        } else if (!F->anything_moving) {
            // Nothing is moving anywhere, so the Doppler channel has
            // nothing to say and cannot discriminate.  A shadowing
            // contact is then consistent with a person holding still --
            // the case ordinary CSI radar cannot see at all.
            //
            // BUT NOT IF IT WAS ALREADY DISCREDITED.
            //
            // Falling back to trusting shadowing whenever motion stops
            // resurrected the two streak artefacts the moving phase had
            // just rejected: all three contacts became STATIC and
            // believed, and a person standing still read as three
            // people.  Evidence has to accumulate in both directions --
            // a contact the Doppler channel contradicted twenty times
            // does not become credible the moment that channel goes
            // quiet.
            cls = discredited ? MFC_SHADOW_ONLY : MFC_STATIC;
        } else if (expl >= MANTIS_FUSE_CORROBORATE) {
            cls = MFC_CONFIRMED;
        } else if (tri_ok && wit >= MANTIS_MIN_WITNESSES) {
            // Two independent position methods agree and enough beacons
            // witnessed a blockage.  That is corroboration from a
            // different direction than Doppler, and it is worth the same.
            cls = MFC_CONFIRMED;
        } else if (wit >= MANTIS_STRONG_WITNESSES) {
            // STRONG INDEPENDENT AGREEMENT OUTWEIGHS ONE CHANNEL'S DOUBT.
            //
            // Five of six beacons each measured a blockage on a path
            // through here, against their own baselines, with their own
            // radios.  That is five separate witnesses, and it is
            // stronger evidence than the Doppler channel's failure to
            // corroborate -- which can happen for honest reasons: the
            // target may be moving mostly along a bisector, or the
            // motion may be slow enough to sit under the rate floor.
            //
            // Demanding BOTH channels always would mean a room full of
            // agreeing witnesses could be overruled by one quiet
            // instrument.
            cls = MFC_CONFIRMED;
        } else {
            // Something IS moving, and it is not here, and only a couple
            // of beacons think anything is.  That is the signature of a
            // streak artefact rather than a body.
            cls = MFC_SHADOW_ONLY;
        }

        int idx = idx_prior;
        if (idx < 0) {
            for (int i = 0; i < MANTIS_FUSE_MAX; i++)
                if (!F->t[i].active) { idx = i; F->t[i] = MantisFused{}; break; }
            if (idx < 0) continue;
        }
        MantisFused &t = F->t[idx];
        touched[idx] = true;
        t.active = true;
        t.x = s.x; t.y = s.y;
        t.shadow_llr   = s.llr;
        t.doppler_expl = expl;
        t.vx = vx; t.vy = vy;
        t.coverage  = s.coverage;
        t.witnesses  = wit;
        t.tri_agrees = tri_ok;
        t.cls = cls;
        t.frames_seen++;

        // ── ACCUMULATE ──
        // Only corroborated evidence raises confidence.  A contact that
        // one channel supports and the other denies can be seen on every
        // frame forever and never become believed, which is precisely
        // the behaviour a stable artefact needs.
        const bool good = (cls == MFC_CONFIRMED || cls == MFC_STATIC);
        if (good) t.frames_corroborated++;
        else if (F->anything_moving) t.frames_contradicted++;   // an active denial
        const float target = good ? 1.0f : 0.0f;
        const float rate = good ? MANTIS_FUSE_RISE : MANTIS_FUSE_FALL;
        t.confidence += rate * (target - t.confidence);
    }

    // Tracks the shadowing channel did not report this frame decay.
    for (int i = 0; i < MANTIS_FUSE_MAX; i++) {
        if (!F->t[i].active || touched[i]) continue;
        F->t[i].confidence += MANTIS_FUSE_FALL * (0.0f - F->t[i].confidence);
        if (F->t[i].confidence < 0.05f) F->t[i].active = false;
    }

    F->n = 0;
    for (int i = 0; i < MANTIS_FUSE_MAX; i++) if (F->t[i].active) F->n++;
}

// Should this contact be presented as real?
//
// Reported, not enforced: the UI draws everything and uses the class and
// the confidence to decide HOW.  Deleting a low-confidence contact would
// hide the fact that something is there at all.
static inline bool mantis_fuse_believed(const MantisFused *t) {
    return t->active && t->confidence >= 0.55f;
}

static inline const char *mantis_fuse_class_name(MantisFuseClass c) {
    switch (c) {
        case MFC_CONFIRMED:    return "CONFIRMED";
        case MFC_STATIC:       return "STATIC";
        case MFC_SHADOW_ONLY:  return "unconfirmed";
        case MFC_DOPPLER_ONLY: return "MOTION";
        default:               return "-";
    }
}
