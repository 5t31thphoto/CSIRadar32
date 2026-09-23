#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS BEACON CSI REDUCTION
//  A C3 turning its own channel estimates into a sufficient statistic
// ═══════════════════════════════════════════════════════════════
//
//  Each beacon hears every other beacon.  Shipping that raw is out of
//  the question: 64 subcarriers x I/Q x 5 peers x 30 Hz is ~200 kB/s per
//  beacon, which no control plane on this hardware will carry and which
//  would defeat the point of moving control off the measurement channel.
//
//  So the beacon reduces.  An ESP32-C3 has a 160 MHz RISC-V core doing
//  almost nothing between its own transmissions -- it can afford this
//  easily, and the reduction is the whole reason to make beacons smart
//  rather than merely two-way.
//
//  WHAT SURVIVES THE REDUCTION, AND WHY
//
//  The receiver needs to know, per link: how much the path changed, in
//  which direction, and how much to trust it.  Everything else in a raw
//  CSI vector is either constant (the static room), common-mode (the
//  radio), or noise.
//
//    amp    mean |H| over selected subcarriers, referenced to this
//           beacon's OWN quiet-room average.  Referencing locally is
//           what makes the report comparable across beacons with
//           different gains, antennas and neighbours.
//
//    phase  the LINEAR-FIT INTERCEPT across subcarriers, not a raw
//           angle.  Per-packet CFO and sampling offset appear as a SLOPE
//           across frequency; the intercept is what survives them.  A
//           raw phase would be dominated by transmitter timing jitter
//           and carry no path information at all.
//
//    quality  how much of the estimate was usable -- how many
//           subcarriers passed, how stable they were.  A link reporting
//           low quality is not reporting a small perturbation; it is
//           reporting that it does not know, and those are different
//           claims that must not be collapsed.
//
//  SUBCARRIER SELECTION
//
//  Not all of them.  DC and the guard bands carry no channel, pilots are
//  modulated, and the extreme edges are filter roll-off.  Averaging
//  those in adds variance for no signal -- the classic mistake that
//  makes a CSI system look noisier than the channel actually is.
// ═══════════════════════════════════════════════════════════════
#include "mantis_report.h"
#include <stdint.h>
#include <math.h>
#include <string.h>

// HT20 gives 64 subcarriers.  Usable data carriers, excluding DC, the
// pilots at +-7/+-21, and the outer guard.
#define MBC_SC_TOTAL   64
#define MBC_SC_LO      6
#define MBC_SC_HI      58
#define MBC_SC_MAX     64

static inline bool mbc_sc_usable(int k) {
    if (k < MBC_SC_LO || k > MBC_SC_HI) return false;   // guard / roll-off
    const int c = k - 32;                                // centre on DC
    if (c == 0) return false;                            // DC carries nothing
    if (c == 7 || c == -7 || c == 21 || c == -21) return false;  // pilots
    return true;
}

// Per-link running state, held on the beacon.
typedef struct {
    float    amp_base;      // quiet-room mean |H|
    float    phi_base;      // quiet-room fit intercept
    uint16_t base_n;        // samples folded into the baseline
    float    amp_ema;       // smoothed current
    float    phi_ema;
    uint8_t  last_quality;
    uint32_t last_seq;
    bool     ready;

    // Local noise estimate: how much this link wanders when nothing is
    // happening.  A link that is intrinsically restless needs a bigger
    // excursion to count as blocked than one that sits still, and only
    // the beacon knows which kind this is.
    float    quiet_dev;      // running mean |deviation| during baseline
    bool     blocked;        // THIS beacon's verdict on THIS link
} MantisBeaconLink;

// How many multiples of a link's own quiet deviation count as blocked.
// Per-link rather than global: a 0.15 excursion is decisive on a stable
// link and meaningless on a restless one, and a single global threshold
// has to be set for the worst link in the room.
#ifndef MANTIS_BLOCK_SIGMA
  #define MANTIS_BLOCK_SIGMA 3.5f
#endif
// Floor, so an unnaturally quiet link cannot make trivial noise look
// like a body.
#ifndef MANTIS_BLOCK_FLOOR
  #define MANTIS_BLOCK_FLOOR 0.08f
#endif

// Reduce one raw CSI buffer to amplitude, fit-intercept phase and a
// quality figure.
//
// `buf` is the ESP-IDF layout: interleaved int8 imag, real per
// subcarrier.  Returns false when too little of the estimate was usable
// to say anything, which is a real and common outcome at range.
static inline bool mantis_beacon_reduce(const int8_t *buf, int len,
                                        float *out_amp, float *out_phi,
                                        uint8_t *out_quality) {
    if (!buf || len < MBC_SC_TOTAL * 2) return false;

    float amp_sum = 0.0f;
    int   n = 0;
    // Weighted linear fit of unwrapped phase against subcarrier index.
    // Weighting by amplitude matters: a near-null subcarrier has a phase
    // that is essentially random, and letting it pull the fit equally
    // would corrupt the one number we actually want.
    float sw = 0, swx = 0, swy = 0, swxx = 0, swxy = 0;
    float prev = 0.0f, unwrapped = 0.0f;
    bool  first = true;

    for (int k = 0; k < MBC_SC_TOTAL; k++) {
        if (!mbc_sc_usable(k)) continue;
        const float im = (float)buf[k * 2];
        const float re = (float)buf[k * 2 + 1];
        const float a  = sqrtf(re * re + im * im);
        if (a < 1.0f) continue;          // below the quantiser floor
        amp_sum += a; n++;

        float ph = atan2f(im, re);
        if (first) { unwrapped = ph; first = false; }
        else {
            float d = ph - prev;
            while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
            while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
            unwrapped += d;
        }
        prev = ph;

        const float x = (float)(k - 32);
        const float w = a;
        sw += w; swx += w * x; swy += w * unwrapped;
        swxx += w * x * x; swxy += w * x * unwrapped;
    }

    // Fewer than a third of the usable carriers is not an estimate.
    if (n < 16 || sw <= 0.0f) { if (out_quality) *out_quality = 0; return false; }

    const float den = sw * swxx - swx * swx;
    // Intercept of the weighted fit: the component that does NOT vary
    // with frequency, which is the part CFO and timing offset cannot
    // fake.
    const float intercept = (fabsf(den) < 1e-6f)
                          ? (swy / sw)
                          : ((swxx * swy - swx * swxy) / den);

    if (out_amp) *out_amp = amp_sum / (float)n;
    if (out_phi) *out_phi = intercept;
    if (out_quality) {
        // Quality is coverage, plainly: what fraction of the usable
        // carriers actually contributed.  Not a confidence in the VALUE,
        // a confidence in having measured at all.
        const int usable = MBC_SC_HI - MBC_SC_LO + 1 - 5;
        int q = (n * 255) / (usable > 0 ? usable : 1);
        *out_quality = (uint8_t)(q > 255 ? 255 : (q < 1 ? 1 : q));
    }
    return true;
}

// Fold a reduction into a link's state, and emit the wire view.
//
// `learning` selects baseline accumulation over measurement.  Same rule
// as everywhere else in this system: the baseline is a running mean of
// the quiet room, and a measurement is a difference from it.
static inline void mantis_beacon_link_update(MantisBeaconLink *L,
                                             float amp, float phi,
                                             uint8_t quality, bool learning,
                                             MantisLinkView *out, uint8_t peer_id) {
    // Light smoothing only.  Heavier filtering here would hide exactly
    // the fast transient the receiver is looking for, and the receiver
    // has its own filters that can be tuned with full context.
    L->amp_ema = L->ready ? (L->amp_ema + 0.30f * (amp - L->amp_ema)) : amp;
    // Phase must be averaged on the circle, not the line.
    float dphi = phi - L->phi_ema;
    while (dphi >  (float)M_PI) dphi -= 2.0f * (float)M_PI;
    while (dphi < -(float)M_PI) dphi += 2.0f * (float)M_PI;
    L->phi_ema = L->ready ? (L->phi_ema + 0.30f * dphi) : phi;
    L->ready = true;
    L->last_quality = quality;

    if (learning) {
        const uint16_t k = L->base_n;
        L->amp_base = (k == 0) ? L->amp_ema
                               : (L->amp_base + (L->amp_ema - L->amp_base) / (float)(k + 1));
        float db = L->phi_ema - L->phi_base;
        while (db >  (float)M_PI) db -= 2.0f * (float)M_PI;
        while (db < -(float)M_PI) db += 2.0f * (float)M_PI;
        L->phi_base = (k == 0) ? L->phi_ema
                               : (L->phi_base + db / (float)(k + 1));
        if (L->base_n < 65535) L->base_n++;
    }

    // ── THE BEACON'S OWN VERDICT ──────────────────────────────
    // Made here because only here is the link's own quiet behaviour
    // known.  The receiver sees the number; the beacon knows what is
    // normal for it.
    float da_now = 0.0f;
    if (L->base_n > 0 && L->amp_base > 1e-3f)
        da_now = (L->amp_ema - L->amp_base) / L->amp_base;
    if (learning) {
        // Measure the link's restlessness only once its BASELINE has
        // settled.
        //
        // da_now is a deviation from amp_base, and amp_base is still
        // converging during the first samples -- so those deviations are
        // huge and have nothing to do with how restless the link
        // actually is.  Folding them in inflated quiet_dev, which raised
        // the blocked threshold, which made the beacons miss real
        // blockages on exactly the links carrying the most signal.  The
        // witness count then came out BACKWARDS: the true target scored
        // 2 of 6 while an artefact scored 3.
        if (L->base_n > 12) {
            const float dev = fabsf(da_now);
            const uint16_t k = (uint16_t)(L->base_n - 12);
            L->quiet_dev = (k <= 1) ? dev
                                    : (L->quiet_dev + (dev - L->quiet_dev) / (float)k);
        }
        L->blocked = false;
    } else {
        float thresh = MANTIS_BLOCK_SIGMA * L->quiet_dev;
        if (thresh < MANTIS_BLOCK_FLOOR) thresh = MANTIS_BLOCK_FLOOR;
        // Blocked means LOST amplitude.  A gain is constructive
        // multipath, not a body, and counting it would let a reflection
        // vote as a witness.
        L->blocked = (da_now < -thresh) && (quality > 60);
    }

    if (!out) return;
    out->peer_id = peer_id;
    out->quality = quality;

    // Report the PERTURBATION, normalised by the baseline so it is a
    // fractional change.  An absolute delta would depend on this
    // beacon's gain and path loss, making reports incomparable between
    // beacons -- which is precisely what the receiver needs to compare.
    float da = 0.0f;
    if (L->base_n > 0 && L->amp_base > 1e-3f)
        da = (L->amp_ema - L->amp_base) / L->amp_base;
    out->amp_q8 = mantis_q8(da);

    float dp = L->phi_ema - L->phi_base;
    while (dp >  (float)M_PI) dp -= 2.0f * (float)M_PI;
    while (dp < -(float)M_PI) dp += 2.0f * (float)M_PI;
    out->phase_q12 = mantis_q12(dp);
}
