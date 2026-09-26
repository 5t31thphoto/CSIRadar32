#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS SOUNDING PATTERNS
// ═══════════════════════════════════════════════════════════════
//
//  A beacon is not a transmitter that happens to repeat.  It is one
//  element of a distributed array, and WHEN each element transmits is as
//  much a design parameter as what it sends.
//
//  Four patterns, each measuring something the others cannot:
//
//  ── SWEEP ─────────────────────────────────────────────────────
//  One beacon transmits, every other radio listens.  Repeated across
//  all beacons this yields the full bistatic matrix: at 6 beacons, 42
//  links -- 12 beacon->receiver plus 30 beacon->beacon.
//
//  The peer links are the valuable ones.  Receiver links are SPOKES from
//  the perimeter inward; peer links are CHORDS across the room.  A
//  person standing between B2 and B5 blocks that chord and essentially
//  nothing else, which is a localised constraint the spoke topology
//  cannot produce at any rate.
//
//  And they are free: a beacon idle in someone else's slot is a receiver
//  in that slot.  Not one extra packet is transmitted to obtain them.
//
//  ── FLASH ─────────────────────────────────────────────────────
//  Every beacon transmits in the SAME slot.  The receiver sees a
//  superposition, so nothing can be attributed to any one path -- this
//  pattern localises nothing.
//
//  What it buys is energy.  N transmitters sum at the receiver:
//  +4.8 dB at 3 beacons, +7.8 dB at 6.  That is the most sensitive
//  "did anything change at all" measurement available, obtained in ONE
//  slot where a sweep spends N.
//
//  So FLASH is a trigger, not a locator: cheap enough to run often,
//  sensitive enough to catch the first frame of motion, and it hands off
//  to SWEEP and FOCUS to say where.
//
//  ── FOCUS ─────────────────────────────────────────────────────
//  Re-sound only the links whose path passes near a hypothesis.
//
//  Measured on a 6-beacon ring, of 42 links the number passing within
//  0.25 of a given point:
//      centre        18 of 42
//      off-centre     8 of 42
//      near a beacon 10 of 42
//
//  Roughly a third of the matrix carries almost all the evidence about
//  any one place.  Spending the agile slot on those links raises their
//  effective update rate ~3x exactly where a target is, at zero extra
//  airtime -- the scheduling equivalent of pointing a beam.
//
//  ── RANGE ─────────────────────────────────────────────────────
//  An FTM exchange between one pair per cycle.  Far too coarse to track
//  a person (~0.5-2 m) and exactly right for surveying where the beacons
//  ARE -- the thing the old system never knew and invented as a ring by
//  slot index.
//
//  ── THE BUDGET ────────────────────────────────────────────────
//      slots 0..N-1   SWEEP    full matrix, every superframe (30 Hz)
//      slot  6        AGILE    rotates FLASH / FOCUS / FLASH / RANGE
//      slot  7        SILENCE  true noise floor (30 Hz)
//
//  The agile slot runs at 30 Hz and cycles four patterns, so FLASH lands
//  at 15 Hz and FOCUS and RANGE at 7.5 Hz each.  Full matrix rate is
//  untouched.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"
#include "mantis_sched.h"

typedef enum : uint8_t {
    MP_SWEEP   = 0,   // this beacon's own slot: transmit alone
    MP_FLASH   = 1,   // all beacons transmit together
    MP_FOCUS   = 2,   // only the selected subset transmits
    MP_RANGE   = 3,   // FTM exchange for one pair
    MP_SILENCE = 4,   // nobody transmits
    MP_LISTEN  = 5,   // not our turn; measure whoever is speaking
} MantisPattern;

// What the agile slot does, by macro-frame index.  FLASH twice per cycle
// because motion detection is the thing that must not be slow; RANGE
// only needs to converge over minutes, not seconds.
#define MANTIS_AGILE_CYCLE 4
static inline MantisPattern mantis_agile_for(uint32_t seq) {
    switch (seq % MANTIS_AGILE_CYCLE) {
        case 0: return MP_FLASH;
        case 1: return MP_FOCUS;
        case 2: return MP_FLASH;
        default: return MP_RANGE;
    }
}

// Up to 6 beacons -> a subset fits in a byte.  Bit k = beacon id (k+1).
typedef uint8_t MantisFocusMask;

// What should THIS radio do in the slot it is currently in?
//
// `my_id` is 1..6 for a beacon, 0 for a receiver (which never transmits
// and therefore always listens).
static inline MantisPattern mantis_pattern_now(const MantisSched *s,
                                               int64_t now_us,
                                               uint8_t my_id,
                                               MantisFocusMask focus,
                                               uint8_t *out_slot,
                                               uint32_t *out_seq) {
    uint32_t seq = 0; int32_t off = 0;
    const uint8_t slot = mantis_sched_slot(s, now_us, &seq, &off);
    if (out_slot) *out_slot = slot;
    if (out_seq)  *out_seq  = seq;

    if (!s->synced) return MP_LISTEN;      // unsynced radios stay quiet
    if (slot == MANTIS_SLOT_SILENCE) return MP_SILENCE;

    if (slot == MANTIS_SLOT_GUARD) {
        const MantisPattern p = mantis_agile_for(seq);
        if (my_id == 0) return MP_LISTEN;  // receiver: always listening
        switch (p) {
            case MP_FLASH: return MP_FLASH;                 // everyone transmits
            case MP_FOCUS: return (focus & (1u << (my_id - 1)))
                                  ? MP_FOCUS : MP_LISTEN;   // only the chosen
            case MP_RANGE: return MP_RANGE;                 // the pair decides below
            default:       return MP_LISTEN;
        }
    }

    // Sweep slots: exactly one beacon speaks.
    if (my_id != 0 && slot == (uint8_t)(my_id - 1)) return MP_SWEEP;
    return MP_LISTEN;
}

// Which pair ranges this cycle?  Walks every unordered pair in turn so
// the full distance matrix fills in deterministically rather than by
// chance.  At 7.5 Hz and 15 pairs (6 beacons), one complete matrix every
// 2 seconds.
static inline void mantis_range_pair(uint32_t seq, uint8_t n_beacons,
                                     uint8_t *a, uint8_t *b) {
    if (n_beacons < 2) { *a = *b = 0; return; }
    const uint32_t pairs = (uint32_t)n_beacons * (n_beacons - 1) / 2;
    uint32_t k = (seq / MANTIS_AGILE_CYCLE) % pairs;
    for (uint8_t i = 1; i <= n_beacons; i++)
        for (uint8_t j = (uint8_t)(i + 1); j <= n_beacons; j++) {
            if (k == 0) { *a = i; *b = j; return; }
            k--;
        }
    *a = 1; *b = 2;
}

// Build a focus mask from a hypothesis.
//
// A link carries information about p when its path passes near p.  With
// beacon positions known (measured by RANGE, not assumed), this is a
// point-to-segment distance -- cheap, and it is the whole of the
// "pointing a beam" idea: select the elements whose geometry actually
// looks at the place we care about.
//
// Falls back to everything when no geometry exists yet, because focusing
// on a guessed layout would be worse than not focusing at all.
static inline MantisFocusMask mantis_focus_mask(float px, float py,
                                                const float *bx, const float *by,
                                                uint8_t n, float radius,
                                                bool geometry_known) {
    (void)radius;
    if (!geometry_known || n < 3) return (MantisFocusMask)((1u << n) - 1u);

    // RANK, do not threshold.
    //
    // A fixed radius was the obvious first attempt and it is wrong on a
    // ring: every beacon has SOME chord passing near the centre, so a
    // centre target selected all six and FOCUS collapsed into SWEEP --
    // spending the agile slot to re-measure what the sweep just measured.
    //
    // Score each beacon by how closely its BEST chord approaches the
    // hypothesis, then take the better half.  That yields a genuine
    // subset everywhere, and it degrades gracefully: when all beacons
    // score alike (a centre target) the choice barely matters, which is
    // itself the correct answer -- the centre is the best-observed place
    // in the room and needs no help.
    float best[8];
    for (uint8_t i = 0; i < n && i < 8; i++) {
        best[i] = 1e9f;
        for (uint8_t j = 0; j < n && j < 8; j++) {
            if (i == j) continue;
            const float ax = bx[i], ay = by[i];
            const float dx = bx[j] - ax, dy = by[j] - ay;
            const float L  = dx * dx + dy * dy;
            float t = (L <= 1e-9f) ? 0.0f : ((px - ax) * dx + (py - ay) * dy) / L;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            const float ex = px - (ax + t * dx), ey = py - (ay + t * dy);
            const float d2 = ex * ex + ey * ey;
            if (d2 < best[i]) best[i] = d2;
        }
    }
    // Half the array, at least two transmitters -- a one-beacon "focus"
    // is a dropout, not a beam.
    uint8_t want = (uint8_t)(n / 2); if (want < 2) want = 2;
    MantisFocusMask m = 0;
    for (uint8_t k = 0; k < want; k++) {
        int8_t pick = -1; float bd = 1e30f;
        for (uint8_t i = 0; i < n && i < 8; i++) {
            if (m & (1u << i)) continue;
            if (best[i] < bd) { bd = best[i]; pick = (int8_t)i; }
        }
        if (pick < 0) break;
        m |= (MantisFocusMask)(1u << pick);
    }
    return m;
}
