#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS SUPERFRAME PROGRAM
//  What every beacon transmits, computes and sleeps, on every frame
// ═══════════════════════════════════════════════════════════════
//
//  ── THE INSIGHT THIS IS BUILT ON ─────────────────────────────
//
//  The channel estimate comes from the PREAMBLE, not the payload.  Every
//  beacon already transmits once per superframe in its own slot, so the
//  payload bytes of that same transmission are free real estate:
//
//      payload                 airtime   slot use
//        4 B stock counter       52 us     1.25%
//       12 B bare sounding       64 us     1.54%
//       48 B + full REPORT      112 us     2.69%
//      120 B + DENSE packet     208 us     4.99%
//
//  A complete perspective report costs 68 microseconds more than a bare
//  sounding, and it REPLACES a separate control transmission that would
//  have cost a whole packet plus its own preamble.  The entire
//  distributed reporting system is close to free.
//
//  Six beacons at 112 us occupy 2.0% of the superframe.  The other 98%
//  is idle, and idle is sleep.
//
//  ── SO THE SLOT CONTENT IS PROGRAMMABLE ──────────────────────
//
//  A beacon is not "a thing that broadcasts".  Each superframe it is
//  assigned a ROLE and a PAYLOAD, and the assignment is a pure function
//  of (seq, id).  Every node computes the same program from the same
//  seq, so there is no coordination traffic at all -- no scheduling
//  messages, no negotiation, nothing to lose or resend.  The program IS
//  the protocol.
//
//  ── THE PROGRAM ──────────────────────────────────────────────
//
//  A macroframe is 16 superframes, ~533 ms, and it is engineered around
//  what the inference actually needs rather than around what is easy:
//
//    frames 0-9   SOLO sounding, rotating REPORT payloads
//                 The workhorse.  Full bistatic matrix every frame, and
//                 each beacon's perspective reaches the receivers within
//                 one macroframe without a single extra packet.
//
//    frame 10     CHORD.  Two beacons transmit in the SAME slot on
//                 purpose.  A receiver sees the coherent sum of two
//                 paths, which is a different equation from either alone
//                 -- it constrains the relative phase between two links
//                 that solo sounding can never observe.
//
//    frame 11     CHORUS.  All beacons together.  +7.8 dB of received
//                 energy in one slot, the most sensitive "did anything
//                 change" measurement available.  A trigger, not a
//                 locator.
//
//    frame 12     DENSE.  One beacon, rotating, spends a long payload on
//                 subcarrier-resolved detail instead of a summary.  Low
//                 rate by design: depth is for kernel building, not for
//                 tracking.
//
//    frames 13-14 SOLO again, so the matrix never goes stale for long.
//
//    frame 15     DEEP SLEEP window.  Nothing transmits.  Every beacon
//                 that has delivered its report may sleep through the
//                 whole frame.
//
//  ── WHY A PROGRAM AND NOT A SCHEDULER ────────────────────────
//
//  A scheduler needs to tell nodes what to do, which needs messages,
//  which can be lost, which needs retries and acknowledgements and
//  timeouts and a way to detect a node acting on a stale schedule.
//
//  A program needs none of that.  It is deterministic in seq, every node
//  already knows seq from the sync it needs anyway, and a node that
//  misses frames rejoins the program exactly where everyone else is
//  simply by knowing what time it is.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"
#include "mantis_sched.h"

#define MANTIS_MACRO_FRAMES 16

// What a beacon puts in its payload this frame.
typedef enum : uint8_t {
    MPL_SOUND  = 0,   // bare MantisAirFrame; cheapest, pure sounding
    MPL_REPORT = 1,   // + MantisPerspective: this beacon's whole view
    MPL_DENSE  = 2,   // + subcarrier-resolved detail for one link set
    MPL_ECHO   = 3,   // + a digest of what this beacon heard from others
} MantisPayloadKind;

// What the frame as a whole is doing.
typedef enum : uint8_t {
    MFR_SOLO   = 0,   // one beacon per slot, the full matrix
    MFR_CHORD  = 1,   // two beacons share a slot, coherent sum
    MFR_CHORUS = 2,   // all beacons together, maximum energy
    MFR_DENSE  = 3,   // one beacon, deep payload
    MFR_SLEEP  = 4,   // nothing transmits; sleep window
} MantisFrameRole;

static inline MantisFrameRole mantis_frame_role(uint32_t seq) {
    switch (seq % MANTIS_MACRO_FRAMES) {
        case 10: return MFR_CHORD;
        case 11: return MFR_CHORUS;
        case 12: return MFR_DENSE;
        case 15: return MFR_SLEEP;
        default: return MFR_SOLO;
    }
}

// Which beacon carries the REPORT payload this frame.
//
// Rotating one per frame rather than all at once keeps every packet
// short and spreads the receiver's parsing load evenly.  With six
// beacons and ten SOLO frames per macroframe, every beacon reports at
// least once per macroframe -- ~1.9 Hz per beacon, far above what the
// tomography needs and far below what would waste airtime.
static inline uint8_t mantis_report_turn(uint32_t seq, uint8_t n_beacons) {
    if (n_beacons == 0) return 0;
    return (uint8_t)((seq % n_beacons) + 1);
}

// Which beacon gets the DENSE slot, rotating far more slowly.
static inline uint8_t mantis_dense_turn(uint32_t seq, uint8_t n_beacons) {
    if (n_beacons == 0) return 0;
    return (uint8_t)(((seq / MANTIS_MACRO_FRAMES) % n_beacons) + 1);
}

// The CHORD pair for this frame.
//
// Walks every unordered pair so the coherent-sum observation covers all
// link pairs deterministically rather than by chance.  15 pairs at one
// per macroframe is a complete sweep every 8 seconds -- slow, but this
// observation constrains static geometry, which does not move.
static inline void mantis_chord_pair(uint32_t seq, uint8_t n_beacons,
                                     uint8_t *a, uint8_t *b) {
    if (n_beacons < 2) { *a = *b = 0; return; }
    const uint32_t pairs = (uint32_t)n_beacons * (n_beacons - 1) / 2;
    uint32_t k = (seq / MANTIS_MACRO_FRAMES) % pairs;
    for (uint8_t i = 1; i <= n_beacons; i++)
        for (uint8_t j = (uint8_t)(i + 1); j <= n_beacons; j++) {
            if (k == 0) { *a = i; *b = j; return; }
            k--;
        }
    *a = 1; *b = 2;
}

// ── What THIS beacon does, right now ──────────────────────────
typedef struct {
    bool              transmit;     // do we key the radio at all this slot?
    MantisPayloadKind payload;
    bool              listen;       // measure CSI from whoever is speaking
    bool              may_sleep;    // nothing required of us until the next slot
    MantisFrameRole   role;
    uint8_t           slot;
    uint32_t          seq;
} MantisDuty;

static inline MantisDuty mantis_duty(const MantisSched *s, int64_t now_us,
                                     uint8_t my_id, uint8_t n_beacons,
                                     bool cal_active) {
    MantisDuty d{};
    uint32_t seq = 0; int32_t off = 0;
    d.slot = mantis_sched_slot(s, now_us, &seq, &off);
    d.seq  = seq;
    d.role = mantis_frame_role(seq);

    if (!s->synced) { d.listen = true; return d; }   // unsynced: listen only

    // The silence slot is silent for everyone, always.  It is how the
    // noise floor gets measured against a band that is known quiet, and
    // one node ignoring it destroys that guarantee for every node.
    if (d.slot == MANTIS_SLOT_SILENCE) {
        d.listen    = true;       // measure the floor
        d.may_sleep = false;      // measuring IS the job
        return d;
    }

    if (d.role == MFR_SLEEP) {
        // The whole frame is a sleep window.  A beacon with nothing owed
        // may power down its radio for ~33 ms, which at 30 Hz is the
        // single largest saving available.
        d.may_sleep = !cal_active;
        d.listen    = cal_active;
        return d;
    }

    switch (d.role) {
        case MFR_CHORUS:
            // Everyone transmits in the guard slot together.
            d.transmit = (d.slot == MANTIS_SLOT_GUARD);
            d.payload  = MPL_SOUND;
            d.listen   = !d.transmit;
            break;
        case MFR_CHORD: {
            uint8_t a, b; mantis_chord_pair(seq, n_beacons, &a, &b);
            d.transmit = (d.slot == MANTIS_SLOT_GUARD) && (my_id == a || my_id == b);
            d.payload  = MPL_SOUND;
            d.listen   = !d.transmit;
            break;
        }
        case MFR_DENSE:
            d.transmit = (d.slot == (uint8_t)(my_id - 1))
                      && (my_id == mantis_dense_turn(seq, n_beacons));
            d.payload  = MPL_DENSE;
            d.listen   = !d.transmit;
            break;
        default:   // MFR_SOLO
            d.transmit = (d.slot == (uint8_t)(my_id - 1));
            // The payload rides the transmission we were making anyway.
            d.payload  = (my_id == mantis_report_turn(seq, n_beacons))
                       ? MPL_REPORT : MPL_SOUND;
            d.listen   = !d.transmit;
            break;
    }

    // ── LISTEN DECIMATION ─────────────────────────────────────
    // Receiving costs 70 mA against 90 for transmit and 0.8 for light
    // sleep.  Listening to every slot is therefore the dominant power
    // cost of the whole mesh, not transmitting:
    //
    //      listen every slot   25% sleep -> 52.7 mA
    //      listen half, rotate 60% sleep -> 28.5 mA    1.9x the runtime
    //
    // At RUNTIME each beacon listens to a rotating half of the other
    // beacons, so the full bistatic matrix completes in two frames
    // rather than one: 30 Hz per link becomes 15 Hz.  A person at
    // 1.4 m/s crosses a chord's half-width in ~0.3 s, so 15 Hz keeps
    // real margin over the ~10 Hz the tomography needs.
    //
    // The rotation is keyed on (slot + id + seq) so different beacons
    // cover different halves on the same frame -- the union across the
    // mesh stays complete every frame even though no single node hears
    // everything.  A decimation that made every node skip the SAME
    // links would lose those links entirely.
    //
    // During CAL nothing is decimated: that data is unrepeatable, and
    // power is the cheapest thing to spend during a ninety-second walk.
    if (!cal_active && d.listen && !d.transmit) {
        if (((uint32_t)d.slot + my_id + seq) & 1u) { d.listen = false; }
    }

    // SLEEP RULE.  A beacon may sleep in a slot where it neither
    // transmits nor has anything to learn.
    //
    // During calibration it never sleeps: every frame is a measurement
    // going into a model that cannot be re-derived, and a missed frame
    // is a hole in training data.  At runtime a missed frame is one
    // observation in thirty and the tracker tolerates far worse.
    //
    // Listening in OTHER beacons' slots is not optional though -- that
    // is where the bistatic matrix comes from, and it is the entire
    // reason the mesh exists.  So the sleepable slots are the ones with
    // no transmitter at all.
    const bool slot_has_speaker =
        (d.role == MFR_SOLO  && d.slot < n_beacons) ||
        (d.role == MFR_DENSE && d.slot < n_beacons) ||
        ((d.role == MFR_CHORD || d.role == MFR_CHORUS) && d.slot == MANTIS_SLOT_GUARD);
    d.may_sleep = !cal_active && !d.transmit && (!slot_has_speaker || !d.listen);
    if (d.may_sleep) d.listen = false;
    return d;
}

static inline const char *mantis_role_name(MantisFrameRole r) {
    switch (r) {
        case MFR_CHORD:  return "CHORD";
        case MFR_CHORUS: return "CHORUS";
        case MFR_DENSE:  return "DENSE";
        case MFR_SLEEP:  return "SLEEP";
        default:         return "SOLO";
    }
}
static inline const char *mantis_payload_name(MantisPayloadKind p) {
    switch (p) {
        case MPL_REPORT: return "REPORT";
        case MPL_DENSE:  return "DENSE";
        case MPL_ECHO:   return "ECHO";
        default:         return "sound";
    }
}
