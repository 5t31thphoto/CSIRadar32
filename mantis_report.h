#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS PERSPECTIVE PROTOCOL
//  How a beacon contributes its compute to the anchor and probe
// ═══════════════════════════════════════════════════════════════
//
//  Every beacon hears every other beacon.  That is a view of the room
//  no receiver can obtain: chords across the middle rather than spokes
//  from the edge.  A C3 is perfectly capable of reducing its own CSI to
//  a sufficient statistic, so it does, and it ships THAT rather than raw
//  samples nobody has the airtime to carry.
//
//  ── THE RULE THIS PROTOCOL TURNS ON ──────────────────────────
//
//      A REPORT IS STATE, NEVER A DELTA.
//
//  Every field is an absolute value for a named superframe.  The
//  receiver ASSIGNS, it never accumulates.  Consequences:
//
//    a duplicate  -> writes the same value again.  Harmless.
//    a loss       -> healed by the next report.  No resync needed.
//    reordering   -> the older seq is dropped on sight.
//    a replay     -> same seq, same values, same result.
//
//  This is not a stylistic preference.  The previous peer-baseline path
//  did `b.slope_baseline -= peer.slope` on receipt, and the sender
//  transmitted three copies because the link was unacknowledged -- so
//  the baseline was subtracted three times and the stereo phase
//  reference was silently wrong by a multiple of itself.  Idempotence
//  is the property that made that class of bug impossible, and it is
//  cheaper than any retry scheme.
//
//  ── SYNCHRONISATION: NO MASTER, NO HANDSHAKE ─────────────────
//
//  The sounding packets ARE the clock.  Every transmission carries
//  (seq, slot), so any radio hearing ANY packet can derive the frame
//  epoch:
//
//      epoch = rx_time - slot * MANTIS_SLOT_US
//
//  No election, no time-sync exchange, no round-trip estimation.  A
//  radio that just powered up listens, hears one packet, and is aligned.
//
//  Robustness comes from there being N sources of the same truth: sync
//  from whichever packet arrives.  By convention the lowest active
//  beacon id is preferred when several are available, because its slot
//  is 0 and the arithmetic is exact -- but nothing DEPENDS on it, so a
//  dead beacon 1 costs nothing.
//
//  Drift: 20 ppm relative moves neighbours ~1.2 ms per minute against a
//  4.2 ms slot.  Every packet re-anchors, so in practice drift never
//  accumulates past one frame.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"
#include <stddef.h>   // offsetof, used by the CRC coverage macro

#define MANTIS_REPORT_VERSION   1
#define MANTIS_MAX_LINKS        (MANTIS_SLOTS - 2)   // peers a beacon can hear

// ── One link, as one beacon sees it ───────────────────────────
// Fixed point, not float: the wire layout must be identical on a C3 and
// an S3, and a float is an invitation to differ.
// SIGN CONVENTION -- stated once, here, because getting it wrong is
// silent.  amp_q8 is the FRACTIONAL CHANGE in received amplitude:
//
//      amp_q8 < 0   the path lost amplitude   -> something is BLOCKING it
//      amp_q8 > 0   the path gained amplitude -> constructive multipath
//
// A body blocking a chord therefore reports NEGATIVE.  Anything
// consuming this as "attenuation" must negate it; mantis_mesh.h does,
// and there is a test that fails if it stops.
typedef struct __attribute__((packed)) {
    uint8_t peer_id;       // whose transmission this describes (1..6)
    uint8_t quality;       // 0..255; 0 = heard nothing this frame
    int16_t amp_q8;        // fractional amplitude change, Q8.8.  NEGATIVE = blocked.
    int16_t phase_q12;     // phase change, Q4.12 radians, wrapped to +-pi
} MantisLinkView;

// Convert a reported amplitude change into the ATTENUATION the
// tomography wants: positive when the path is obstructed.
static inline float mantis_atten_from_amp(float amp_change) {
    return (amp_change < 0.0f) ? -amp_change : 0.0f;
}

// ── A beacon's whole perspective for one superframe ───────────
// Fixed size.  A variable-length record would save a few bytes and cost
// a length field that has to be trusted before it can be validated --
// the exact shape of parser bug that turns a corrupt packet into an
// out-of-bounds read.
typedef struct __attribute__((packed)) {
    uint8_t  version;      // MANTIS_REPORT_VERSION
    uint8_t  reporter_id;  // which beacon is speaking (1..6)
    uint32_t seq;          // WHICH superframe.  The idempotency key.
    int8_t   noise_floor;  // measured in the silence slot, band known quiet
    int8_t   rssi_self;    // this beacon's own reference level
    uint8_t  n_links;      // how many entries of links[] are meaningful
    uint8_t  flags;        // see MANTIS_RF_* below

    // ── THIS BEACON'S OWN VERDICT ─────────────────────────────
    //
    // Bit k set = "link to beacon k+1 is BLOCKED", decided HERE, on the
    // beacon, against its own baseline, its own noise floor and its own
    // per-link history.
    //
    // The receiver could threshold the amplitudes itself, and that would
    // NOT be the same thing.  This is an independent judgement made with
    // information the receiver does not have: how quiet this particular
    // link normally is, how much it has been wandering, what the local
    // noise floor did in the silence slot.  Six beacons each making that
    // call is six witnesses, not one witness with six inputs.
    //
    // It is what turns the mesh from a sensor array into a jury: a real
    // body sits on links from MANY beacons, while a reconstruction
    // artefact is a crossing of streaks that few or none of them
    // actually saw.  Measured on a live six-beacon run:
    //
    //      real target   4 of 6 beacons had a blocked link through it
    //      ghost         2 of 6
    //      far ghost     0 of 6
    uint8_t  blocked_mask;
    uint8_t  _pad2;
    uint8_t  crc8;         // over everything after this field
    uint8_t  _pad;         // explicit: keeps links[] 2-byte aligned
    MantisLinkView links[MANTIS_MAX_LINKS];
} MantisPerspective;

// flags
#define MANTIS_RF_SYNCED       0x01   // reporter believes its slots are aligned
#define MANTIS_RF_SILENCE_OK   0x02   // silence slot was genuinely quiet
#define MANTIS_RF_INTERFERER   0x04   // something transmitted during silence
#define MANTIS_RF_BASELINE_OK  0x08   // reporter has a usable local baseline
#define MANTIS_RF_MOTION       0x10   // reporter's own links say something moved

// ── CRC-8, polynomial 0x07 ────────────────────────────────────
// Cheap, and enough for a 38-byte record.  Its job is not security, it
// is refusing to parse a record that arrived damaged -- because a
// damaged record with a plausible seq is worse than no record at all.
static inline uint8_t mantis_crc8(const uint8_t *d, uint16_t n) {
    uint8_t c = 0xFF;
    for (uint16_t i = 0; i < n; i++) {
        c ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

// Offset past the header fields the CRC does not cover.
#define MANTIS_CRC_SKIP  offsetof(MantisPerspective, _pad)

static inline void mantis_report_seal(MantisPerspective *p) {
    p->version = MANTIS_REPORT_VERSION;
    p->crc8    = 0;
    p->crc8    = mantis_crc8((const uint8_t *)p + MANTIS_CRC_SKIP,
                             (uint16_t)(sizeof(*p) - MANTIS_CRC_SKIP));
}

// Validate EVERYTHING before a single field is believed.
//
// Order matters: size, then version, then CRC, then ranges.  A record
// that fails any check is dropped whole -- never partially applied,
// never repaired.  Guessing at a damaged record is how a transient radio
// glitch becomes a permanently wrong model.
static inline bool mantis_report_valid(const MantisPerspective *p, uint16_t len,
                                       uint8_t n_beacons) {
    if (len != sizeof(MantisPerspective))      return false;
    if (p->version != MANTIS_REPORT_VERSION)   return false;

    MantisPerspective t = *p;
    const uint8_t got = t.crc8;
    t.crc8 = 0;
    if (mantis_crc8((const uint8_t *)&t + MANTIS_CRC_SKIP,
                    (uint16_t)(sizeof(t) - MANTIS_CRC_SKIP)) != got) return false;

    if (p->reporter_id == 0 || p->reporter_id > n_beacons) return false;
    if (p->n_links > MANTIS_MAX_LINKS)                     return false;
    for (uint8_t i = 0; i < p->n_links; i++) {
        const uint8_t id = p->links[i].peer_id;
        if (id == 0 || id > n_beacons)  return false;   // out of range
        if (id == p->reporter_id)       return false;   // cannot hear itself
    }
    return true;
}

// ── Receiver-side store ───────────────────────────────────────
// Latest-by-seq per reporter.  ASSIGNMENT only; there is deliberately no
// operation here that accumulates.
typedef struct {
    uint32_t last_seq;     // 0 = nothing yet
    uint32_t last_rx_ms;
    MantisPerspective view;
    bool     present;
} MantisReporterSlot;

typedef struct {
    MantisReporterSlot by_id[MANTIS_SLOTS];   // indexed by reporter_id
    uint32_t accepted, dropped_old, dropped_bad, dropped_dup;
} MantisPerspectiveStore;

// Returns true if this report changed anything.
//
// Sequence comparison is wrap-safe: (int32_t)(a - b) > 0 stays correct
// across the uint32 rollover, which at 30 Hz arrives after ~4.5 years of
// continuous running.  Unlikely, and free to handle correctly.
static inline bool mantis_store_apply(MantisPerspectiveStore *s,
                                      const MantisPerspective *p,
                                      uint16_t len, uint8_t n_beacons,
                                      uint32_t now_ms) {
    if (!mantis_report_valid(p, len, n_beacons)) { s->dropped_bad++; return false; }
    MantisReporterSlot *slot = &s->by_id[p->reporter_id];

    if (slot->present) {
        if (p->seq == slot->last_seq) { s->dropped_dup++; return false; }
        if ((int32_t)(p->seq - slot->last_seq) < 0) { s->dropped_old++; return false; }
    }
    slot->view       = *p;          // ASSIGN.  Never +=.
    slot->last_seq   = p->seq;
    slot->last_rx_ms = now_ms;
    slot->present    = true;
    s->accepted++;
    return true;
}

// A reporter is live only if it spoke recently.  Stale views are not
// deleted -- they are simply not trusted, so a beacon that returns picks
// up where it left off without a re-handshake.
#define MANTIS_REPORT_STALE_MS 400
static inline bool mantis_store_fresh(const MantisPerspectiveStore *s,
                                      uint8_t id, uint32_t now_ms) {
    if (id >= MANTIS_SLOTS || !s->by_id[id].present) return false;
    return (now_ms - s->by_id[id].last_rx_ms) < MANTIS_REPORT_STALE_MS;
}

// Look up one link as a specific beacon sees it.  Null when that beacon
// has not reported, is stale, or did not hear that peer this frame --
// three different kinds of "no", all of which mean the same thing to a
// caller: do not use this link.
static inline const MantisLinkView *mantis_store_link(const MantisPerspectiveStore *s,
                                                      uint8_t hearer, uint8_t heard,
                                                      uint32_t now_ms) {
    if (!mantis_store_fresh(s, hearer, now_ms)) return nullptr;
    const MantisPerspective *v = &s->by_id[hearer].view;
    for (uint8_t i = 0; i < v->n_links; i++)
        if (v->links[i].peer_id == heard)
            return v->links[i].quality ? &v->links[i] : nullptr;
    return nullptr;
}

// Fixed-point helpers, so both firmwares convert identically.
static inline int16_t mantis_q8(float v)   { return (int16_t)(v * 256.0f); }
static inline float   mantis_unq8(int16_t v){ return (float)v / 256.0f; }
static inline int16_t mantis_q12(float v)  { return (int16_t)(v * 4096.0f); }
static inline float   mantis_unq12(int16_t v){ return (float)v / 4096.0f; }
