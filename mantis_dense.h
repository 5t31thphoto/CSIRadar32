#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS DENSE PERSPECTIVE
//  What a beacon knows that no receiver can compute
// ═══════════════════════════════════════════════════════════════
//
//  ── BACKWARD COMPATIBILITY IS STRUCTURAL, NOT OPTIONAL ───────
//
//  The existing receiver inference reads a broadcast like this:
//
//      if (len >= sizeof(uint32_t)) { memcpy(&counter, data, 4); }
//
//  It takes the FIRST FOUR BYTES and ignores everything after.  So every
//  payload in this protocol leads with that counter, and the whole
//  existing pipeline -- discovery, slot assignment, the kernel, the
//  scene solver, the Rust core -- keeps working with no change at all.
//
//  That is not politeness.  It means the mesh upgrade can be deployed
//  one beacon at a time, and a receiver running old firmware degrades to
//  exactly its current behaviour instead of breaking.
//
//  ── THE THREE TIERS ──────────────────────────────────────────
//
//    SOUND   counter only.  What the current system consumes.
//    REPORT  counter + MantisPerspective: every link, summarised.
//    DENSE   counter + this: TWO links, in depth.
//
//  ── WHAT ONLY A BEACON CAN MEASURE ───────────────────────────
//
//  A receiver sees beacon-to-receiver paths.  It has no access whatever
//  to beacon-to-beacon paths -- those chords cross the room and the
//  receiver is not on them.  So everything here is information the
//  receiver cannot obtain by any amount of processing.
//
//  Four channels, each chosen because it answers a question the
//  amplitude summary cannot:
//
//  1. SUBCARRIER PROFILE -> is this blockage or a reflection change?
//     A body blocking the direct path attenuates BROADBAND, roughly
//     flatly.  A reflection changing attenuates SELECTIVELY, because
//     the interference pattern between two paths depends on frequency.
//     Sixteen bins of |H(f)| tell the two apart; a single mean cannot.
//
//  2. DELAY SPREAD -> how much multipath is on this path at all?
//     Computed as normalised variance across subcarriers, which is the
//     cheap and honest proxy: frequency-selective fading IS delay
//     spread. A path with low spread is nearly line-of-sight and its
//     blockage measurement can be trusted; a path with high spread is
//     mostly reflections and should be weighted down.
//
//  3. MICRO-DOPPLER HISTOGRAM -> what KIND of motion is this?
//     Eight bins of the per-frame phase-rate distribution over a window.
//     A walking person produces a broad spread: torso slow, limbs fast.
//     A fan produces a narrow line at one rate.  A door swing is a
//     transient.  A single "is moving" bit throws all of that away.
//
//  4. EVENT TIMING -> when exactly did this link change?
//     With slot-accurate timing, the seq of the last significant change
//     is meaningful to about a millisecond.  Across beacons, the ORDER
//     in which links broke tells you the direction of travel before any
//     tracker has run.
// ═══════════════════════════════════════════════════════════════
#include "mantis_report.h"
#include <math.h>   // sqrtf/fabsf, used by the profile and Doppler helpers

#define MANTIS_DENSE_VERSION   1
#define MANTIS_DENSE_SC_BINS   16   // subcarrier profile resolution
#define MANTIS_DENSE_DOP_BINS   8   // micro-Doppler histogram
#define MANTIS_DENSE_LINKS      2   // links carried per packet

// Delay-spread class boundaries, normalised sigma across frequency.
// See mantis_dense_classify() for where these came from.
#ifndef MANTIS_SPREAD_LOS
  #define MANTIS_SPREAD_LOS        0.10f
#endif
#ifndef MANTIS_SPREAD_SELECTIVE
  #define MANTIS_SPREAD_SELECTIVE  0.15f
#endif

// One link, in depth.
typedef struct __attribute__((packed)) {
    uint8_t peer_id;
    uint8_t quality;

    // |H(f)| across the band, normalised to this link's own mean and
    // stored as a signed deviation.  Self-normalising means the profile
    // is comparable between beacons with different gain and path loss --
    // it describes the SHAPE of the channel, not its strength.
    int8_t  sc_profile[MANTIS_DENSE_SC_BINS];

    uint16_t delay_spread_q12;   // normalised sc variance, Q4.12
    uint16_t amp_var_q12;        // temporal variance of |H| over the window
    int16_t  amp_mean_q8;        // perturbation vs baseline, Q8.8 (as REPORT)
    int16_t  phase_q12;          // fit intercept delta, Q4.12

    // Micro-Doppler: how the per-frame phase rate was distributed over
    // the window.  Bin 0 is near-zero rate, bin 7 the fastest.
    uint8_t  dop_hist[MANTIS_DENSE_DOP_BINS];

    uint32_t last_event_seq;     // superframe of the last significant change
    uint8_t  event_count;        // changes within the window
    uint8_t  flags;
} MantisDenseLink;

#define MANTIS_DF_LOS        0x01   // low delay spread: near line-of-sight
#define MANTIS_DF_SELECTIVE  0x02   // frequency-selective: reflection-dominated
#define MANTIS_DF_BLOCKED    0x04   // broadband attenuation: a body
#define MANTIS_DF_PERIODIC   0x08   // narrow Doppler line: machinery, not a person

typedef struct __attribute__((packed)) {
    uint32_t counter;            // FIRST. Keeps the existing receiver working.
    uint16_t magic;
    uint8_t  version;
    uint8_t  reporter_id;
    uint32_t seq;
    int8_t   noise_floor;
    uint8_t  n_links;
    uint8_t  crc8;
    uint8_t  _pad;
    MantisDenseLink link[MANTIS_DENSE_LINKS];
} MantisDensePacket;

#define MANTIS_DENSE_MAGIC 0x444E   // 'DN'
#define MANTIS_DENSE_CRC_SKIP offsetof(MantisDensePacket, _pad)

static inline void mantis_dense_seal(MantisDensePacket *p) {
    p->magic   = MANTIS_DENSE_MAGIC;
    p->version = MANTIS_DENSE_VERSION;
    p->crc8    = 0;
    p->crc8    = mantis_crc8((const uint8_t *)p + MANTIS_DENSE_CRC_SKIP,
                             (uint16_t)(sizeof(*p) - MANTIS_DENSE_CRC_SKIP));
}

static inline bool mantis_dense_valid(const MantisDensePacket *p, uint16_t len,
                                      uint8_t n_beacons) {
    if (len != sizeof(MantisDensePacket))   return false;
    if (p->magic   != MANTIS_DENSE_MAGIC)   return false;
    if (p->version != MANTIS_DENSE_VERSION) return false;
    MantisDensePacket t = *p;
    const uint8_t got = t.crc8; t.crc8 = 0;
    if (mantis_crc8((const uint8_t *)&t + MANTIS_DENSE_CRC_SKIP,
                    (uint16_t)(sizeof(t) - MANTIS_DENSE_CRC_SKIP)) != got) return false;
    if (p->reporter_id == 0 || p->reporter_id > n_beacons) return false;
    if (p->n_links > MANTIS_DENSE_LINKS)                   return false;
    for (uint8_t i = 0; i < p->n_links; i++) {
        const uint8_t id = p->link[i].peer_id;
        if (id == 0 || id > n_beacons || id == p->reporter_id) return false;
    }
    return true;
}

// ── Beacon-side computation ───────────────────────────────────

// Fold a raw CSI buffer into the subcarrier profile.
//
// Normalising to the link's own mean is what makes the shape portable:
// two beacons with different gain, antenna and distance produce the same
// profile for the same channel structure, so the receiver can compare
// them directly.
static inline void mantis_dense_profile(const int8_t *buf, int len,
                                        int8_t *out_bins, uint16_t *out_spread) {
    float amp[MANTIS_DENSE_SC_BINS];
    int   cnt[MANTIS_DENSE_SC_BINS];
    for (int i = 0; i < MANTIS_DENSE_SC_BINS; i++) { amp[i] = 0; cnt[i] = 0; }
    if (len < 128) { for (int i = 0; i < MANTIS_DENSE_SC_BINS; i++) out_bins[i] = 0;
                     if (out_spread) *out_spread = 0; return; }

    const int total = len / 2;
    for (int k = 0; k < total; k++) {
        const float im = (float)buf[k * 2], re = (float)buf[k * 2 + 1];
        const float a = sqrtf(re * re + im * im);
        const int b = (k * MANTIS_DENSE_SC_BINS) / total;
        if (b >= 0 && b < MANTIS_DENSE_SC_BINS) { amp[b] += a; cnt[b]++; }
    }
    float mean = 0; int n = 0;
    for (int i = 0; i < MANTIS_DENSE_SC_BINS; i++)
        if (cnt[i]) { amp[i] /= (float)cnt[i]; mean += amp[i]; n++; }
    if (n == 0 || mean <= 0) { for (int i = 0; i < MANTIS_DENSE_SC_BINS; i++) out_bins[i] = 0;
                               if (out_spread) *out_spread = 0; return; }
    mean /= (float)n;

    // The transmitted PROFILE is binned, because 16 bytes is what the
    // packet can afford.
    for (int i = 0; i < MANTIS_DENSE_SC_BINS; i++) {
        const float d = (cnt[i] ? amp[i] : mean) - mean;
        // +-100 maps to +-100% deviation from the mean, which covers a
        // deep null without clipping the common case.
        int v = (int)(100.0f * d / mean);
        out_bins[i] = (int8_t)(v > 127 ? 127 : (v < -127 ? -127 : v));
    }

    // The SPREAD is computed at FULL subcarrier resolution.
    //
    // Deriving it from the 16 binned values averages away the very
    // frequency selectivity it is supposed to measure: binning four
    // subcarriers spans a third of a typical null period, so a deep
    // 2:1 selective fade collapsed to a spread of 0.083 against 0.012
    // for a clear path -- barely separable, and the SELECTIVE flag never
    // fired at all.  The profile is a display artefact of the packet
    // budget; the statistic must not inherit that budget.
    float full_mean = 0; int full_n = 0;
    for (int k = 0; k < total; k++) {
        const float im = (float)buf[k * 2], re = (float)buf[k * 2 + 1];
        const float a = sqrtf(re * re + im * im);
        if (a < 1.0f) continue;
        full_mean += a; full_n++;
    }
    if (full_n < 8 || full_mean <= 0) { if (out_spread) *out_spread = 0; return; }
    full_mean /= (float)full_n;

    float var = 0;
    for (int k = 0; k < total; k++) {
        const float im = (float)buf[k * 2], re = (float)buf[k * 2 + 1];
        const float a = sqrtf(re * re + im * im);
        if (a < 1.0f) continue;
        const float d = a - full_mean;
        var += d * d;
    }
    var /= (float)full_n;
    // Normalised standard deviation across frequency.  Frequency-
    // selective fading IS delay spread; this is the cheap honest proxy.
    const float spread = sqrtf(var) / full_mean;
    if (out_spread) {
        const float q = spread * 4096.0f;
        *out_spread = (uint16_t)(q > 65535.0f ? 65535.0f : (q < 0 ? 0 : q));
    }
}

// Classify a link from its own profile.
//
// A body blocking the direct path attenuates BROADBAND and roughly
// flatly.  A reflection changing attenuates SELECTIVELY.  This is the
// distinction a single mean amplitude cannot make, and it is why the
// profile is worth its sixteen bytes.
static inline uint8_t mantis_dense_classify(uint16_t spread_q12, float amp_delta) {
    // Thresholds placed in the measured gap, not guessed.  On a
    // synthetic channel the four cases separate by roughly 7x:
    //
    //      clear path            0.027
    //      body blocking         0.035    <- flat loss, spread unchanged
    //      reflection changed    0.235    <- selective, similar mean amp
    //      body + rich multipath 0.211
    //
    // Note the second and third rows: nearly the same mean amplitude,
    // completely different spread.  That is the distinction a single
    // amplitude summary cannot make and this packet exists to carry.
    //
    // PROVISIONAL until measured on hardware -- a real room has more
    // multipath than a synthetic channel, so both classes will shift
    // upward together.  The SEPARATION is the robust part, not the
    // absolute values, and these should be re-fitted from the first
    // calibration walk rather than trusted as constants.
    uint8_t f = 0;
    const float spread = (float)spread_q12 / 4096.0f;
    if (spread < MANTIS_SPREAD_LOS)       f |= MANTIS_DF_LOS;
    if (spread > MANTIS_SPREAD_SELECTIVE) f |= MANTIS_DF_SELECTIVE;
    // Broadband loss on a path that is NOT reflection-dominated is a
    // body in the direct path.
    if (amp_delta < -0.10f && spread < MANTIS_SPREAD_SELECTIVE)
        f |= MANTIS_DF_BLOCKED;
    return f;
}

// Accumulate a micro-Doppler histogram.
//
// Log-spaced bins, because the interesting structure is at the low end:
// breathing and a slow torso live below 1 rad/s while a swinging limb
// reaches several, and linear bins would put almost everything in bin 0.
static inline void mantis_dense_dop_accum(uint8_t *hist, float rate_rad_s) {
    const float a = fabsf(rate_rad_s);
    int b;
    if      (a < 0.10f) b = 0;
    else if (a < 0.25f) b = 1;
    else if (a < 0.50f) b = 2;
    else if (a < 1.00f) b = 3;
    else if (a < 2.00f) b = 4;
    else if (a < 4.00f) b = 5;
    else if (a < 8.00f) b = 6;
    else                b = 7;
    if (hist[b] < 255) hist[b]++;
}

// Is this motion PERIODIC rather than human?
//
// Machinery concentrates its Doppler at one rate; a person spreads it
// across several because different body parts move at different speeds.
// A fan that trips a presence detector every time is the single most
// common false alarm in this class of system, and one narrow histogram
// bin identifies it without any training.
static inline bool mantis_dense_is_periodic(const uint8_t *hist) {
    int total = 0, peak = 0, peak_bin = -1;
    for (int i = 1; i < MANTIS_DENSE_DOP_BINS; i++) {   // skip the at-rest bin
        total += hist[i];
        if (hist[i] > peak) { peak = hist[i]; peak_bin = i; }
    }
    if (total < 12 || peak_bin < 0) return false;   // too little evidence
    // Over 80% of the moving energy in one bin is not a person.
    return (peak * 100) > (total * 80);
}
