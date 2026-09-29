#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS FRAME ALIGNMENT
//  One room, two coordinate systems, one transform between them
// ═══════════════════════════════════════════════════════════════
//
//  The anchor's scene solver lives in ITS frame: the T-Display at the
//  origin, beacons where the operator placed them relative to it.  The
//  mesh lives in the SURVEY frame: centroid at the origin, the lowest
//  beacon due north, radius normalised to 1.  Both describe the same
//  beacons, so the beacons themselves are the correspondences, and a
//  similarity transform (scale, rotation, translation, optional mirror)
//  maps one frame onto the other.
//
//  That transform is what makes the system scale opportunistically:
//
//    anchor + 1 beacon      no transform needed; anchor-only radar
//    anchor + 2 beacons     exact fit, rotation assumed (a mirror is
//                           indistinguishable from two points)
//    anchor + 3+ beacons    least squares; the mirror is DECIDED by the
//                           data, and the residual is a trust metric
//
//  Closed-form Umeyama in 2-D.  No iteration, no failure mode beyond
//  "fewer than two points", and it runs in microseconds.
// ═══════════════════════════════════════════════════════════════
#include <stdint.h>
#include <math.h>

typedef struct {
    // q = M p + t, with M = s * [[c, -sn*m], [sn, c*m]] ... stored flat.
    float m00, m01, m10, m11;
    float tx, ty;
    float scale;
    float rms;          // residual in DESTINATION units
    uint8_t n;          // correspondences used
    bool  mirror;
    bool  valid;
} MantisAlign;

static inline void mantis_align_apply(const MantisAlign *A, float px, float py,
                                      float *qx, float *qy) {
    *qx = A->m00 * px + A->m01 * py + A->tx;
    *qy = A->m10 * px + A->m11 * py + A->ty;
}

// Inverse mapping, q -> p.  M is a scaled orthogonal matrix, so its
// inverse is its transpose over scale squared.
static inline void mantis_align_invert(const MantisAlign *A, float qx, float qy,
                                       float *px, float *py) {
    const float s2 = A->scale * A->scale;
    if (s2 < 1e-12f) { *px = qx; *py = qy; return; }
    const float dx = qx - A->tx, dy = qy - A->ty;
    *px = (A->m00 * dx + A->m10 * dy) / s2;
    *py = (A->m01 * dx + A->m11 * dy) / s2;
}

// Fit one hypothesis (mirror or not).  Returns the residual.
static inline float mantis_align_fit_one(const float *px, const float *py,
                                         const float *qx, const float *qy,
                                         int n, bool mirror, MantisAlign *out) {
    float mpx = 0, mpy = 0, mqx = 0, mqy = 0;
    for (int i = 0; i < n; i++) {
        const float y = mirror ? -py[i] : py[i];
        mpx += px[i]; mpy += y; mqx += qx[i]; mqy += qy[i];
    }
    mpx /= n; mpy /= n; mqx /= n; mqy /= n;
    float sxx = 0, sxy = 0, pp = 0;
    for (int i = 0; i < n; i++) {
        const float ax = px[i] - mpx, ay = (mirror ? -py[i] : py[i]) - mpy;
        const float bx = qx[i] - mqx, by = qy[i] - mqy;
        sxx += ax * bx + ay * by;
        sxy += ax * by - ay * bx;
        pp  += ax * ax + ay * ay;
    }
    if (pp < 1e-9f) { out->valid = false; return 1e9f; }
    const float th = atan2f(sxy, sxx);
    const float s  = sqrtf(sxx * sxx + sxy * sxy) / pp;
    const float c = cosf(th) * s, sn = sinf(th) * s;
    // Rotation R = [[c,-sn],[sn,c]]; mirror multiplies column 2 by -1.
    out->m00 = c;  out->m01 = mirror ?  sn : -sn;
    out->m10 = sn; out->m11 = mirror ? -c  :  c;
    out->scale  = s;
    out->mirror = mirror;
    // t = mq - M * mp_original.  mp in the mirrored frame had y negated,
    // and M already folds the mirror in, so use the ORIGINAL centroid.
    const float opx = mpx, opy = mirror ? -mpy : mpy;
    out->tx = mqx - (out->m00 * opx + out->m01 * opy);
    out->ty = mqy - (out->m10 * opx + out->m11 * opy);
    out->n = (uint8_t)n;
    out->valid = true;
    float e = 0;
    for (int i = 0; i < n; i++) {
        float x, y; mantis_align_apply(out, px[i], py[i], &x, &y);
        e += (x - qx[i]) * (x - qx[i]) + (y - qy[i]) * (y - qy[i]);
    }
    out->rms = sqrtf(e / n);
    return out->rms;
}

// Fit p -> q.  With two points the mirror is unobservable and rotation is
// assumed; with three or more the better hypothesis wins, and it has to
// win by a margin so noise cannot flip the map back and forth.
static inline bool mantis_align_fit(const float *px, const float *py,
                                    const float *qx, const float *qy,
                                    int n, bool prev_mirror, MantisAlign *out) {
    *out = MantisAlign{};
    if (n < 2) return false;
    MantisAlign a, b;
    const float ea = mantis_align_fit_one(px, py, qx, qy, n, false, &a);
    if (n == 2) { *out = a; return a.valid; }
    const float eb = mantis_align_fit_one(px, py, qx, qy, n, true, &b);
    // Hysteresis in favour of whatever we already believed.
    const float bias = 0.8f;
    bool pick_mirror;
    if (prev_mirror) pick_mirror = !(ea < eb * bias);
    else             pick_mirror =  (eb < ea * bias);
    *out = pick_mirror ? b : a;
    return out->valid;
}
