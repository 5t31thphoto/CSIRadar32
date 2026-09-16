#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS MESH GEOMETRY
//  The beacons work out their own shape and hand it to the receivers
// ═══════════════════════════════════════════════════════════════
//
//  Every beacon hears every other beacon, so the mesh holds a complete
//  pairwise measurement matrix that no receiver can obtain.  From that
//  matrix the mesh can solve its OWN layout and broadcast it -- which
//  removes the single worst assumption in the whole system.
//
//  Until now beacon positions came from csi_assign_default_geometry():
//  a perfect ring, ordered by slot index, because nothing better was
//  available.  Every downstream position was therefore relative to a
//  fiction.  The tomography, the chart, the coverage map, the "walk to
//  B2" instruction -- all of it rested on beacons being where we told
//  ourselves they were.
//
//  ── WHAT A RELATIVE SOLUTION IS, AND IS NOT ──────────────────
//
//  A distance matrix determines a configuration up to rotation,
//  reflection and translation.  It cannot determine absolute orientation
//  -- that is not a limitation of the method, it is a fact about
//  distances -- so this produces a RELATIVE geometry with a defined
//  canonical frame:
//
//      origin     the centroid of the beacons
//      rotation   beacon 1 placed on the +Y axis
//      handedness fixed so the lowest-id triple runs counter-clockwise
//
//  Anchoring that frame to the room is the receiver's job, and it has
//  the probe walk to do it with.  What the mesh supplies is the SHAPE,
//  which is the part it can actually measure.
//
//  ── WHERE THE DISTANCES COME FROM ────────────────────────────
//
//  Two sources, in order of preference:
//
//    FTM    real time-of-flight, ~0.5-2 m indoors after averaging.
//           Gives absolute scale, so the chart can be in metres.
//    RSSI   log-distance path loss.  Poor indoors -- multipath makes it
//           optimistic in some directions and pessimistic in others --
//           but it is available instantly, on every link, with no extra
//           protocol.
//
//  RSSI alone is NOT good enough to trust as a survey.  It is good
//  enough to SEED one, and the residual stress after solving says how
//  much to believe the result.  A solution with high stress is reported
//  as low confidence rather than quietly used as though it were
//  measured.
// ═══════════════════════════════════════════════════════════════
#include "mantis_report.h"
#include <math.h>

#define MANTIS_GEOM_VERSION 1
#define MANTIS_GEOM_MAGIC   0x4745   // 'GE'

// Per-beacon telemetry, carried alongside the geometry because the
// operator needs to know mesh HEALTH at the same moment they are looking
// at mesh SHAPE -- a beacon drawn in the right place but running flat is
// not a working beacon.
typedef struct __attribute__((packed)) {
    int16_t  x_q10, y_q10;     // solved position, Q6.10, canonical frame
    uint8_t  batt_pct;
    int8_t   noise_floor;
    uint8_t  heard_mask;       // which other beacons this one can hear
    uint8_t  loss_pct;         // frames missed from its expected speakers
    uint16_t uptime_s;
    uint8_t  flags;
} MantisGeomNode;

#define MANTIS_GN_FTM      0x01   // its ranges came from FTM, not RSSI
#define MANTIS_GN_LOWBATT  0x02
#define MANTIS_GN_ISOLATED 0x04   // hears fewer than two peers
#define MANTIS_GN_MOVED    0x08   // its ranges changed after the survey

typedef struct __attribute__((packed)) {
    uint32_t counter;          // FIRST, so legacy receivers still work
    uint16_t magic;
    uint8_t  version;
    uint8_t  reporter_id;
    uint32_t seq;
    uint8_t  n_nodes;
    uint8_t  stress_q8;        // residual fit error, 0 = perfect
    uint8_t  scale_known;      // 1 = cm_per_unit is meaningful (FTM)
    uint8_t  crc8;
    uint16_t cm_per_unit;      // absolute scale when scale_known
    MantisGeomNode node[MANTIS_SLOTS];
} MantisGeomPacket;

#define MANTIS_GEOM_CRC_SKIP offsetof(MantisGeomPacket, cm_per_unit)

static inline void mantis_geom_seal(MantisGeomPacket *p) {
    p->magic = MANTIS_GEOM_MAGIC; p->version = MANTIS_GEOM_VERSION;
    p->crc8 = 0;
    p->crc8 = mantis_crc8((const uint8_t *)p + MANTIS_GEOM_CRC_SKIP,
                          (uint16_t)(sizeof(*p) - MANTIS_GEOM_CRC_SKIP));
}
static inline bool mantis_geom_valid(const MantisGeomPacket *p, uint16_t len) {
    if (len != sizeof(MantisGeomPacket))    return false;
    if (p->magic   != MANTIS_GEOM_MAGIC)    return false;
    if (p->version != MANTIS_GEOM_VERSION)  return false;
    if (p->n_nodes == 0 || p->n_nodes >= MANTIS_SLOTS) return false;
    MantisGeomPacket t = *p; const uint8_t got = t.crc8; t.crc8 = 0;
    return mantis_crc8((const uint8_t *)&t + MANTIS_GEOM_CRC_SKIP,
                       (uint16_t)(sizeof(t) - MANTIS_GEOM_CRC_SKIP)) == got;
}

// ── RSSI to distance ──────────────────────────────────────────
// Log-distance path loss.  Honest about what it is: a rough, biased
// estimate that multipath makes optimistic in some directions and
// pessimistic in others.  Its job is to SEED the solver, not to survey.
#ifndef MANTIS_RSSI_AT_1M
  #define MANTIS_RSSI_AT_1M   -40.0f
#endif
#ifndef MANTIS_PATH_LOSS_N
  #define MANTIS_PATH_LOSS_N   2.6f    // indoor, between free space and heavy clutter
#endif
static inline float mantis_rssi_to_m(int8_t rssi) {
    const float d = powf(10.0f, (MANTIS_RSSI_AT_1M - (float)rssi)
                                / (10.0f * MANTIS_PATH_LOSS_N));
    return d < 0.2f ? 0.2f : (d > 30.0f ? 30.0f : d);
}

// ── The solver ────────────────────────────────────────────────
// Stress majorisation on the distance matrix (SMACOF).  Chosen over
// classical MDS because it degrades gracefully with missing entries --
// and entries WILL be missing, because two beacons at opposite corners
// may genuinely not hear each other.  Classical MDS needs a complete
// matrix and would have to invent the gaps.
//
// `dist` is n x n in metres; a non-positive entry means "not measured".
// `out_x/out_y` receive the solution; the return value is residual
// stress, normalised, where 0 is a perfect fit.
static inline float mantis_geom_solve(const float *dist, uint8_t n,
                                      float *out_x, float *out_y,
                                      int iters) {
    if (n < 3) return 1.0f;

    // Seed on a circle.  Any non-degenerate start converges; a circle
    // avoids the collapsed configuration a random seed can fall into.
    float mean_d = 0; int md_n = 0;
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++)
            if (i != j && dist[i * n + j] > 0) { mean_d += dist[i * n + j]; md_n++; }
    const float R = (md_n > 0) ? (mean_d / (float)md_n) * 0.6f : 1.0f;
    for (uint8_t i = 0; i < n; i++) {
        const float a = 2.0f * (float)M_PI * (float)i / (float)n;
        out_x[i] = R * cosf(a);
        out_y[i] = R * sinf(a);
    }

    for (int it = 0; it < iters; it++) {
        float nx[MANTIS_SLOTS], ny[MANTIS_SLOTS]; float w[MANTIS_SLOTS];
        for (uint8_t i = 0; i < n; i++) { nx[i] = 0; ny[i] = 0; w[i] = 0; }

        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < n; j++) {
                if (i == j) continue;
                const float d = dist[i * n + j];
                if (d <= 0) continue;                  // not measured: skip
                float dx = out_x[i] - out_x[j], dy = out_y[i] - out_y[j];
                float cur = sqrtf(dx * dx + dy * dy);
                if (cur < 1e-5f) { dx = 1e-3f; dy = 0; cur = 1e-3f; }
                // Move i to where j says it should be, then average.
                nx[i] += out_x[j] + d * dx / cur;
                ny[i] += out_y[j] + d * dy / cur;
                w[i]  += 1.0f;
            }
        }
        for (uint8_t i = 0; i < n; i++)
            if (w[i] > 0) { out_x[i] = nx[i] / w[i]; out_y[i] = ny[i] / w[i]; }
    }

    // Residual stress, normalised by the measured distances.
    float num = 0, den = 0;
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = (uint8_t)(i + 1); j < n; j++) {
            const float d = dist[i * n + j];
            if (d <= 0) continue;
            const float dx = out_x[i] - out_x[j], dy = out_y[i] - out_y[j];
            const float cur = sqrtf(dx * dx + dy * dy);
            num += (cur - d) * (cur - d);
            den += d * d;
        }
    return (den > 0) ? sqrtf(num / den) : 1.0f;
}

// Put a solution into the canonical frame.
//
// A distance matrix fixes shape but not pose, so without this the same
// room would be reported differently on every solve and the display
// would spin.  Centroid to origin, beacon 1 to +Y, handedness fixed.
static inline void mantis_geom_canonical(float *x, float *y, uint8_t n) {
    if (n < 2) return;
    float cx = 0, cy = 0;
    for (uint8_t i = 0; i < n; i++) { cx += x[i]; cy += y[i]; }
    cx /= (float)n; cy /= (float)n;
    for (uint8_t i = 0; i < n; i++) { x[i] -= cx; y[i] -= cy; }

    // Rotate beacon 0 (id 1) onto +Y.
    const float a = atan2f(y[0], x[0]);
    const float rot = (float)M_PI / 2.0f - a;
    const float cs = cosf(rot), sn = sinf(rot);
    for (uint8_t i = 0; i < n; i++) {
        const float px = x[i], py = y[i];
        x[i] = px * cs - py * sn;
        y[i] = px * sn + py * cs;
    }

    // Fix handedness.  Reflection is the one ambiguity a distance matrix
    // genuinely cannot resolve, so we CHOOSE a convention rather than
    // pretend to measure one: the first three beacons run
    // counter-clockwise.  The receiver resolves the real handedness
    // during the walk, when the probe supplies a second observation.
    if (n >= 3) {
        const float cross = (x[1] - x[0]) * (y[2] - y[0])
                          - (y[1] - y[0]) * (x[2] - x[0]);
        if (cross < 0) for (uint8_t i = 0; i < n; i++) x[i] = -x[i];
    }
}

// Normalise to the chart's unit conventions, returning cm per unit so
// absolute scale survives if FTM supplied it.
static inline uint16_t mantis_geom_normalise(float *x, float *y, uint8_t n) {
    float maxr = 0;
    for (uint8_t i = 0; i < n; i++) {
        const float r = sqrtf(x[i] * x[i] + y[i] * y[i]);
        if (r > maxr) maxr = r;
    }
    if (maxr < 1e-4f) return 0;
    for (uint8_t i = 0; i < n; i++) { x[i] /= maxr; y[i] /= maxr; }
    const float cm = maxr * 100.0f;
    return (uint16_t)(cm > 65535.0f ? 65535.0f : cm);
}

// Confidence from residual stress.
//
// Reported, not hidden.  A geometry solved from RSSI in a cluttered room
// will have high stress, and the correct response is to draw it faintly
// and keep asking for FTM -- not to present a guess as a survey.
static inline float mantis_geom_confidence(uint8_t stress_q8) {
    const float s = (float)stress_q8 / 255.0f;
    if (s >= 0.5f) return 0.0f;
    return 1.0f - 2.0f * s;
}
