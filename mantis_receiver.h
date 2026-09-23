#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS RECEIVER INGEST
//  Turning beacon broadcasts into a live mesh estimate
// ═══════════════════════════════════════════════════════════════
//
//  One file, used by BOTH receivers.  The T-Display and the Cardputer
//  run the same ingest because they consume the same air protocol, and
//  two copies of this would drift apart within a week.
//
//  ── WHAT ARRIVES ─────────────────────────────────────────────
//
//  Every beacon broadcast has the same shape:
//
//      [uint32 counter][MantisAirFrame][optional payload]
//
//  The counter is what the ORIGINAL receiver reads, and it still works
//  untouched -- this parser starts after it.  The optional payload is a
//  MantisPerspective, a MantisDensePacket, or nothing, and which one it
//  is comes from the program, not from a type byte: any node can compute
//  what beacon k should be sending on frame N.
//
//  We still check magic and CRC rather than trusting that computation,
//  because a receiver that decodes a packet as whatever it EXPECTED is
//  one dropped frame away from parsing garbage with confidence.
//
//  ── THE SOLVE IS RATE-LIMITED, THE INGEST IS NOT ─────────────
//
//  Reports arrive at up to 180 Hz across six beacons.  Ingest is cheap
//  (validate, assign) and must keep up or reports are lost.  The solve
//  is expensive (back-projection over 576 cells) and gains nothing from
//  running faster than the beacons report, so it runs at 10 Hz.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"
#include "mantis_report.h"
#include "mantis_mesh.h"
#include "mantis_targets.h"
#include "mantis_dense.h"
#include "mantis_geometry.h"
#include "mantis_static.h"
#include "mantis_probe.h"
#include "mantis_fuse.h"
#include "mantis_caps.h"

// UID -> current beacon_id.  The receiver watches this and RE-KEYS
// rather than silently mixing data when a box's assignable id changes.
//
// Without it, a beacon that came back with a different id would inherit
// the surveyed position and quiet-room baseline of whichever box held
// that id before -- and nothing anywhere would look wrong.
typedef struct {
    uint16_t uid;
    uint8_t  beacon_id;
    bool     present;
} MantisUidBind;

typedef struct {
    MantisMesh             mesh;
    MantisUidBind          uid_bind[MANTIS_SLOTS];
    uint32_t               rekeys;
    MantisPerspectiveStore store;
    MantisTargetSet        targets;
    MantisStaticScene      statics;
    MantisIntegrity        integrity;
    MantisProbe            probe;
    MantisFusion           fusion;
    // Per-link phase rate, the Doppler channel's raw input.  Derived
    // from consecutive reports of the same link, which is only
    // meaningful because the slot schedule makes dt exact.
    float    rate[MANTIS_SLOTS * MANTIS_SLOTS];
    float    prev_phase[MANTIS_SLOTS * MANTIS_SLOTS];
    uint32_t prev_seq[MANTIS_SLOTS * MANTIS_SLOTS];

    // Per-link delay-spread and classification, from DENSE packets.
    uint16_t spread_q12[MANTIS_SLOTS * MANTIS_SLOTS];
    uint8_t  link_flags[MANTIS_SLOTS * MANTIS_SLOTS];

    uint32_t reports_in, dense_in, rejected;
    uint32_t geom_in, geom_rejected, geom_rebuilds;
    uint8_t  geom_stress;
    float    geom_shift;      // largest beacon move on the last adoption
    uint16_t cm_per_unit;     // absolute scale, when the survey supplies it
    bool     scale_known;
    // What this deployment can do, recomputed as hardware appears and
    // disappears.  Every screen asks THIS rather than testing beacon
    // counts itself -- gating scattered across screens is how a menu
    // ends up offering something the solver cannot do.
    MantisDeployment deploy;
    MantisCaps       caps;
    uint32_t last_solve_ms;
    bool     baseline_latched;
    bool     baseline_partial;   // latched on the backstop, not on coverage
    uint32_t first_report_ms;
} MantisReceiver;

// ── WHEN TO STOP LEARNING THE QUIET ROOM ─────────────────────
//
// COVERAGE, not a stopwatch.
//
// A fixed window from the first packet was the obvious design and it
// failed in the ordinary case: beacons spend several seconds claiming
// their ids before anyone reports anything useful, so a 12-second timer
// started at the first packet left barely five seconds of real learning
// and only 15 of 30 directed links ever got a baseline.  The receiver
// then sat at "need more links" forever, with a fully healthy mesh in
// front of it.
//
// Learning now continues until enough links actually HAVE a reference,
// which is the condition that matters, and the timer is only a backstop
// against a mesh that never converges.
//
// Minimum, so a single fast link cannot end the window early.
#ifndef MANTIS_RX_BASELINE_MIN_MS
  #define MANTIS_RX_BASELINE_MIN_MS 6000
#endif
// Backstop.  Past this we accept whatever coverage exists rather than
// refusing to work: a partial baseline is degraded, no baseline is dead.
#ifndef MANTIS_RX_BASELINE_MAX_MS
  #define MANTIS_RX_BASELINE_MAX_MS 45000
#endif

// `tdisplays` / `cardputers` describe THIS deployment's receivers,
// including ourselves.  The beacon count is learned from the air and
// filled in as they are heard.
static inline void mantis_rx_begin(MantisReceiver *r, float extent,
                                   uint8_t tdisplays, uint8_t cardputers,
                                   bool docked) {
    *r = MantisReceiver{};
    r->mesh.extent = extent;
    r->deploy.tdisplays  = tdisplays;
    r->deploy.cardputers = cardputers;
    r->deploy.docked     = docked;
    r->caps = mantis_caps(&r->deploy);
}

// Feed one raw ESP-NOW payload.  Returns true if anything was consumed.
//
// `now_ms` is the local clock; the mesh's own sequencing comes from the
// air frame, so a receiver with a wandering millis() cannot corrupt the
// ordering of reports.
static inline bool mantis_rx_adopt_geometry(MantisReceiver *r,
                                            const MantisGeomPacket *g,
                                            uint16_t len);

static inline bool mantis_rx_packet(MantisReceiver *r, uint8_t from_id,
                                    const uint8_t *data, int len,
                                    uint32_t now_ms) {
    if (!data || len < (int)(sizeof(uint32_t) + sizeof(MantisAirFrame)))
        return false;
    if (from_id == 0 || from_id >= MANTIS_SLOTS) return false;

    const MantisAirFrame *f =
        (const MantisAirFrame *)(data + sizeof(uint32_t));
    if (f->magic != MANTIS_AIR_MAGIC) return false;

    // ── UID TRACKING ──────────────────────────────────────────
    // Identity is the box (uid); the id is only a slot assignment.  If a
    // uid turns up under a NEW id, everything learned about the old id
    // describes a different physical box and must not be carried over.
    if (f->uid != 0) {
        bool seen = false;
        for (uint8_t i = 1; i < MANTIS_SLOTS; i++) {
            if (!r->uid_bind[i].present || r->uid_bind[i].uid != f->uid) continue;
            seen = true;
            if (r->uid_bind[i].beacon_id != from_id) {
                // This box moved to a different id.  Drop the learned
                // state on BOTH: the old id's data belongs to this box
                // and is now filed under the wrong name, and the new
                // id's data belongs to whoever was there before.
                const uint8_t old_id = r->uid_bind[i].beacon_id;
                for (uint8_t j = 1; j < MANTIS_SLOTS; j++) {
                    r->mesh.base_n[old_id * MANTIS_SLOTS + j] = 0;
                    r->mesh.base_n[j * MANTIS_SLOTS + old_id] = 0;
                    r->mesh.base_n[from_id * MANTIS_SLOTS + j] = 0;
                    r->mesh.base_n[j * MANTIS_SLOTS + from_id] = 0;
                }
                r->mesh.pos_measured[old_id]  = 0;
                r->mesh.pos_measured[from_id] = 0;
                r->uid_bind[i].beacon_id = from_id;
                r->rekeys++;
            }
            break;
        }
        if (!seen) {
            for (uint8_t i = 1; i < MANTIS_SLOTS; i++) {
                if (r->uid_bind[i].present) continue;
                r->uid_bind[i].uid       = f->uid;
                r->uid_bind[i].beacon_id = from_id;
                r->uid_bind[i].present   = true;
                break;
            }
        }
    }

    // Learn how many beacons exist from who actually speaks, rather than
    // from a configured count that can be wrong in either direction.
    if (from_id > r->mesh.n_beacons) r->mesh.n_beacons = from_id;
    // Capability follows the hardware, always.  Recomputed here because
    // this is the one place that learns a beacon exists.
    r->deploy.beacons        = r->mesh.n_beacons;
    r->deploy.geometry_known = (mantis_mesh_geometry_confidence(&r->mesh) > 0.5f);
    r->caps = mantis_caps(&r->deploy);
    if (r->first_report_ms == 0) r->first_report_ms = now_ms;

    const int used = (int)(sizeof(uint32_t) + sizeof(MantisAirFrame));
    const int rest = len - used;
    if (rest <= 0) return true;              // bare sounding: nothing more

    const uint8_t *body = data + used;
    // Learning ends when the SOLVE latches the baseline, not on a timer.
    // Keeping these two in step matters: if ingest stopped accumulating
    // before the solve was satisfied, coverage could never improve and
    // the receiver would wait forever for a condition it had stopped
    // working toward.
    const bool learning = !r->baseline_latched;

    // ── perspective ──
    if (rest >= (int)sizeof(MantisPerspective)) {
        const MantisPerspective *p = (const MantisPerspective *)body;
        if (mantis_report_valid(p, sizeof(MantisPerspective),
                                r->mesh.n_beacons)) {
            if (mantis_store_apply(&r->store, p, sizeof(*p),
                                   r->mesh.n_beacons, now_ms)) {
                // Phase RATE, before the mesh ingest overwrites state.
                // One frame interval per superframe, known exactly from
                // the schedule -- which is what makes this a measurement
                // rather than an estimate carrying transmitter jitter.
                for (uint8_t i = 0; i < p->n_links; i++) {
                    const uint8_t peer = p->links[i].peer_id;
                    const int idx = p->reporter_id * MANTIS_SLOTS + peer;
                    const float cur = mantis_unq12(p->links[i].phase_q12);
                    if (r->prev_seq[idx] != 0 && p->seq > r->prev_seq[idx]) {
                        const uint32_t dseq = p->seq - r->prev_seq[idx];
                        float d = cur - r->prev_phase[idx];
                        while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
                        while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
                        const float dt = (float)dseq * (float)MANTIS_FRAME_US / 1e6f;
                        if (dt > 1e-4f) r->rate[idx] = d / dt;
                    }
                    r->prev_phase[idx] = cur;
                    r->prev_seq[idx]   = p->seq;
                }
                mantis_mesh_ingest(&r->mesh, p, learning);
                r->reports_in++;
            }
            return true;
        }
    }

    // ── surveyed geometry ──
    // Checked before dense: both are large, and adopting a MEASURED
    // layout is the single most valuable thing arriving on this radio --
    // every position downstream is relative to it.
    if (rest >= (int)sizeof(MantisGeomPacket)) {
        const MantisGeomPacket *g = (const MantisGeomPacket *)body;
        if (mantis_geom_valid(g, sizeof(MantisGeomPacket))) {
            if (mantis_rx_adopt_geometry(r, g, sizeof(MantisGeomPacket))) {
                r->geom_in++;
                r->geom_stress = g->stress_q8;
            } else {
                // Valid packet, but the mesh does not trust its own
                // solve.  Counted separately so the UI can say "the mesh
                // is surveying but not confident yet" rather than
                // silently staying on the assumed ring.
                r->geom_rejected++;
                r->geom_stress = g->stress_q8;
            }
            return true;
        }
    }

    // ── dense ──
    if (rest >= (int)sizeof(MantisDensePacket)) {
        const MantisDensePacket *d = (const MantisDensePacket *)body;
        if (mantis_dense_valid(d, sizeof(MantisDensePacket),
                               r->mesh.n_beacons)) {
            for (uint8_t i = 0; i < d->n_links; i++) {
                const uint8_t peer = d->link[i].peer_id;
                const int idx = d->reporter_id * MANTIS_SLOTS + peer;
                r->spread_q12[idx] = d->link[i].delay_spread_q12;
                r->link_flags[idx] = d->link[i].flags;
            }
            r->dense_in++;
            return true;
        }
    }

    r->rejected++;
    return true;
}

// Down-weight links the DENSE reports say are reflection-dominated.
//
// A frequency-selective path is mostly multipath, so its amplitude
// changes for reasons that have nothing to do with anything blocking the
// straight line between two beacons.  Treating it as a clean shadowing
// measurement puts energy in the wrong place on the map -- and does it
// confidently, because the number itself looks fine.
static inline void mantis_rx_apply_link_quality(MantisReceiver *r) {
    for (uint8_t i = 1; i <= r->mesh.n_beacons; i++)
        for (uint8_t j = 1; j <= r->mesh.n_beacons; j++) {
            if (i == j) continue;
            const int idx = i * MANTIS_SLOTS + j;
            const uint8_t fl = r->link_flags[idx];
            if (fl == 0) continue;             // no dense report yet
            if (fl & MANTIS_DF_SELECTIVE) r->mesh.qual[idx] *= 0.45f;
            else if (fl & MANTIS_DF_LOS)  r->mesh.qual[idx] *= 1.0f;
        }
}

// Run the estimate.  Call freely; it rate-limits itself.
static inline bool mantis_rx_solve(MantisReceiver *r, uint32_t now_ms,
                                   float dt_s) {
    if ((now_ms - r->last_solve_ms) < 100) return false;
    r->last_solve_ms = now_ms;

    if (!r->baseline_latched) {
        if (!r->first_report_ms) return false;
        const uint32_t el = now_ms - r->first_report_ms;
        // Latch once, on COVERAGE -- or on the backstop, so a mesh that
        // will never fully converge still produces a working, degraded
        // estimate rather than nothing at all.
        const bool covered = (el >= MANTIS_RX_BASELINE_MIN_MS)
                           && mantis_mesh_baseline_ready(&r->mesh);
        const bool timeout = (el >= MANTIS_RX_BASELINE_MAX_MS);
        if (covered || timeout) {
            r->mesh.base_ready  = true;
            r->baseline_latched = true;
            r->baseline_partial = timeout && !covered;
            mantis_static_build(&r->statics, &r->mesh);  // the room's fixed structure
        }
        return false;
    }

    mantis_rx_apply_link_quality(r);
    // Do not run a solve the deployment cannot support.  With fewer than
    // three beacons the reconstruction has nothing to intersect and
    // would report structure that is purely its own.
    if (!r->caps.localize) return false;
    mantis_mesh_solve(&r->mesh);
    mantis_targets_extract(&r->targets, &r->mesh, &r->store,
                           r->mesh.last_seq, dt_s, now_ms);
    // CROSS-REFERENCE.  Shadowing alone reports stable artefacts that no
    // amount of temporal filtering removes, because they are not noise --
    // they are deterministic crossings of a sparse sinogram.  The
    // Doppler channel is computed from different physics and is not
    // fooled by them, so the two together resolve what neither can
    // alone.
    mantis_fuse_update(&r->fusion, &r->mesh, &r->store,
                       &r->targets, r->rate, now_ms);
    mantis_integrity_check(&r->integrity, &r->mesh, 0.10f);
    return true;
}

// Seed beacon geometry.  Until a survey runs this is the nominal ring --
// and `measured` says which it is, so nothing downstream mistakes an
// assumption for a measurement.
static inline void mantis_rx_seed_geometry(MantisReceiver *r, bool measured) {
    const uint8_t n = r->mesh.n_beacons;
    if (n == 0) return;
    for (uint8_t i = 1; i <= n; i++) {
        const float t = (float)M_PI / 2.0f + 2.0f * (float)M_PI * (i - 1) / n;
        mantis_mesh_set_pos(&r->mesh, i, cosf(t), sinf(t), measured);
    }
}

// Adopt a surveyed layout, and REBUILD WHAT RESTED ON THE OLD ONE.
//
// Adopting the positions alone is not enough, and the failure would be
// silent: the static room map is back-projected THROUGH the chords, so
// a map computed against the assumed ring describes walls that are not
// where it thinks they are.  The probe's learned body profile has the
// same problem -- it was measured against chord geometry.
//
// Invalidated IN PROPORTION to how far things moved.  A small
// refinement should not throw away a good baseline; a beacon that turns
// out to be somewhere else entirely should.
static inline bool mantis_rx_adopt_geometry(MantisReceiver *r,
                                            const MantisGeomPacket *g,
                                            uint16_t len) {
    if (!mantis_geom_valid(g, len)) return false;
    // A high-stress fit is worse than the nominal ring, because it LOOKS
    // like a measurement.
    if (mantis_geom_confidence(g->stress_q8) < 0.35f) return false;

    float moved = 0.0f;
    for (uint8_t i = 0; i < g->n_nodes && i + 1 < MANTIS_SLOTS; i++) {
        const uint8_t id = (uint8_t)(i + 1);
        const float nx = (float)g->node[i].x_q10 / 1024.0f;
        const float ny = (float)g->node[i].y_q10 / 1024.0f;
        if (r->mesh.have_pos[id]) {
            const float dx = nx - r->mesh.bx[id], dy = ny - r->mesh.by[id];
            const float d = sqrtf(dx * dx + dy * dy);
            if (d > moved) moved = d;
        }
        mantis_mesh_set_pos(&r->mesh, id, nx, ny, true);
    }
    r->geom_shift = moved;

    // ABSOLUTE SCALE, when the mesh has it.  An RSSI survey gives shape
    // without metres; an FTM one gives both.  Carrying cm_per_unit is
    // what lets the UI say "2.4 m" instead of "0.8 chart units".
    if (g->scale_known && g->cm_per_unit > 0) {
        r->cm_per_unit = g->cm_per_unit;
        r->scale_known = true;
    }

    // A shift wider than a chord means the chord set is materially
    // different, so anything back-projected through it describes the
    // wrong room.
    if (moved > MANTIS_CHORD_HALFWIDTH && r->baseline_latched) {
        mantis_static_build(&r->statics, &r->mesh);
        r->probe.learn_n    = 0;      // re-learn against the new geometry
        r->probe.body_atten = 0.0f;
        r->geom_rebuilds++;
    }
    return true;
}

static inline const char *mantis_rx_state(const MantisReceiver *r,
                                          uint32_t now_ms) {
    if (r->mesh.n_beacons == 0)  return "no beacons";
    // Below the localisation tier, say what the deployment IS doing
    // rather than reporting a failure to do something it was never
    // equipped for.  One or two beacons detect presence perfectly well.
    if (!r->caps.localize)       return r->caps.blocker;
    if (!r->baseline_latched) return "learning room";
    if (r->baseline_partial)   return "partial baseline";
    if (r->integrity.geometry_suspect) return "BEACON MOVED";
    // Report what is BELIEVED, not what shadowing merely proposed.
    int believed = 0;
    for (int i = 0; i < MANTIS_FUSE_MAX; i++)
        if (mantis_fuse_believed(&r->fusion.t[i])) believed++;
    if (believed > 0) return "CONTACT";
    if (r->fusion.n > 0) return "unconfirmed";
    return "clear";
}
