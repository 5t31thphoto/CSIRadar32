#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS BEACON IDENTITY
//  One firmware for every beacon, stable ids that survive a reboot
// ═══════════════════════════════════════════════════════════════
//
//  ── THE HARD REQUIREMENT IS STABILITY, NOT UNIQUENESS ────────
//
//  Uniqueness is easy and it is the least useful of the three properties
//  this needs.  Everything downstream is KEYED TO BEACON ID:
//
//      the surveyed geometry        "beacon 3 is at (x, y)"
//      the per-link baselines       "link 3-5 is normally this quiet"
//      the static room map          built from those baselines
//      the operator's mental model  "walk to B2"
//
//  If ids shuffle on reboot, every one of those silently refers to the
//  wrong physical box.  The system would look completely healthy and be
//  wrong, which is the worst failure mode this project has.
//
//  So: an id is CLAIMED ONCE and then PERSISTED.  A beacon that has an
//  id keeps it for life unless something takes it away.
//
//  ── THREE WAYS AN ID GETS SET, IN PRIORITY ORDER ─────────────
//
//    1. FLASH-TIME   a build-time constant, if someone sets one.  The CI
//                    no longer does, but the hook stays: a deployment
//                    that wants beacon 4 to always be beacon 4 should
//                    not have to fight the firmware.
//    2. STORED       whatever was claimed or configured previously, read
//                    from NVS.  This is the normal path after first boot.
//    3. CLAIMED      listen, see which ids are in use, take the lowest
//                    free one, persist it.
//
//  ── COLLISIONS RESOLVE BY MAC, NOT BY BACKOFF ────────────────
//
//  Two beacons booting simultaneously can claim the same id.  Random
//  backoff resolves that in 1.15 rounds on average with an UNBOUNDED
//  worst case; comparing MACs resolves it in exactly one, always, and
//  cannot tie because a MAC is already unique and already in the header
//  of every packet.
//
//  Lower MAC keeps the id.  The loser re-claims, which costs it one more
//  listen window and nothing else.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"
#include <string.h>   // memcmp/memcpy on the MAC tiebreak

// HIGHEST VALID BEACON ID.
//
// slot = id - 1, and the beacon slots are 0 .. MANTIS_SLOT_GUARD-1.  So
// the top id is MANTIS_SLOTS - 2, not MANTIS_SLOTS - 1.
//
// I had the off-by-one and it was not cosmetic: id 7 maps to slot 6,
// which is the GUARD slot that CHORD and CHORUS transmit in.  A beacon
// claiming it would collide with every chord and chorus frame in the
// program, and the symptom would be those two patterns quietly
// producing garbage while ordinary sweeps looked fine.
#define MANTIS_MAX_BEACON_ID  (MANTIS_SLOTS - 2)

// Set this at build time to pin a beacon.  0 means "decide at runtime",
// which is what the shipped firmware does.
#ifndef MANTIS_FIXED_BEACON_ID
  #define MANTIS_FIXED_BEACON_ID 0
#endif

// How long to listen before claiming.
//
// Long enough that a beacon already running will certainly have
// transmitted -- it speaks once per superframe, so anything over a few
// frames is sufficient.  Two seconds is generous and only ever costs
// two seconds, once, on a beacon's first boot in its life.
#define MANTIS_CLAIM_LISTEN_MS   2000
// And how long to hold a claim unchallenged before committing it.
#define MANTIS_CLAIM_CONFIRM_MS  1200

typedef enum : uint8_t {
    MID_BOOT = 0,      // nothing decided yet
    MID_LISTENING,     // building a picture of who is out there
    MID_CLAIMING,      // announced an id, waiting to be challenged
    MID_COMMITTED,     // the id is ours and is persisted
    MID_EXHAUSTED,     // every id taken; we are a listener, not a beacon
} MantisIdState;

typedef struct {
    MantisIdState state;
    uint8_t  id;                 // 0 until decided
    uint8_t  occupied_mask;      // bit k set = id (k+1) heard in use
    uint8_t  mac[6];             // ours, the tiebreak key
    uint32_t state_since_ms;
    bool     from_storage;       // did we load it rather than claim it?
    bool     dirty;              // needs writing to NVS
    uint8_t  challenges;         // times we lost a tiebreak this boot
} MantisIdentity;

static inline void mantis_id_begin(MantisIdentity *I, const uint8_t *mac,
                                   uint8_t stored_id, uint32_t now_ms) {
    *I = MantisIdentity{};
    memcpy(I->mac, mac, 6);
    I->state_since_ms = now_ms;

    if (MANTIS_FIXED_BEACON_ID > 0) {
        // Pinned at build time.  Commit immediately and never listen:
        // someone asked for this id specifically and deserves to get it
        // even if it means a collision they can then diagnose.
        I->id = MANTIS_FIXED_BEACON_ID;
        I->state = MID_COMMITTED;
        return;
    }
    if (stored_id >= 1 && stored_id <= MANTIS_MAX_BEACON_ID) {
        // We had an id last time.  Take it back, but STILL listen --
        // hardware gets swapped, and an id we think is ours may now
        // belong to a beacon that claimed it while we were off.
        I->id = stored_id;
        I->from_storage = true;
    }
    I->state = MID_LISTENING;
}

// Note that some id is in use.  Called for every frame heard, in any
// state, because an id can be lost after it is committed.
static inline void mantis_id_saw(MantisIdentity *I, uint8_t other_id,
                                 const uint8_t *other_mac, uint32_t now_ms) {
    if (other_id == 0 || other_id > MANTIS_MAX_BEACON_ID) return;
    I->occupied_mask |= (uint8_t)(1u << (other_id - 1));

    if (other_id != I->id || I->state == MID_BOOT) return;

    // Somebody else is using OUR id.  Compare MACs: lower wins.
    //
    // Deterministic, symmetric, and both sides reach the same verdict
    // from the same data without exchanging anything further.  A beacon
    // that loses does not need to be told it lost.
    const int cmp = memcmp(I->mac, other_mac, 6);
    if (cmp < 0) return;               // we are lower: we keep it

    // We lose.  Drop back to claiming, and remember this id is taken so
    // we do not immediately re-pick it.
    I->challenges++;
    I->id = 0;
    I->from_storage = false;
    I->state = MID_LISTENING;
    I->state_since_ms = now_ms;
}

// The id this beacon would LIKE, derived from its MAC.
//
// Deterministic, so the same box prefers the same id every time even
// before anything has been persisted -- which makes a first boot in a
// known room usually land on the same layout as the last one.
static inline uint8_t mantis_id_preferred(const MantisIdentity *I) {
    // FNV-1a over the MAC: cheap, and mixes the low bytes that actually
    // differ between adjacent units from the same production run.
    uint32_t h = 2166136261u;
    for (int i = 0; i < 6; i++) { h ^= I->mac[i]; h *= 16777619u; }
    return (uint8_t)((h % MANTIS_MAX_BEACON_ID) + 1);
}

// Lowest id not known to be in use.  0 when the mesh is full.
static inline uint8_t mantis_id_lowest_free(const MantisIdentity *I) {
    for (uint8_t k = 1; k <= MANTIS_MAX_BEACON_ID; k++)
        if (!(I->occupied_mask & (1u << (k - 1)))) return k;
    return 0;
}

// Drive the state machine.  Returns true when the id CHANGED, so the
// caller knows to re-init anything keyed to it.
static inline bool mantis_id_tick(MantisIdentity *I, uint32_t now_ms) {
    const uint32_t age = now_ms - I->state_since_ms;

    switch (I->state) {
        case MID_LISTENING: {
            if (age < MANTIS_CLAIM_LISTEN_MS) return false;
            // A stored id nobody else has taken is ours again, with no
            // claim exchange at all -- the common case, and the one that
            // keeps geometry and baselines valid across a power cycle.
            if (I->from_storage && I->id &&
                !(I->occupied_mask & (1u << (I->id - 1)))) {
                I->state = MID_COMMITTED;
                I->state_since_ms = now_ms;
                return true;
            }
            // PREFER AN ID DERIVED FROM OUR MAC, not simply the lowest.
            //
            // Six blank beacons booting together all see an empty mask
            // and all reach for id 1.  One wins, five are knocked back,
            // and the next round they all reach for id 2 -- so
            // convergence SERIALISES at one beacon per listen+confirm
            // cycle, about 19 seconds for six.  Correct, and far slower
            // than it needs to be.
            //
            // Hashing the MAC into the id space spreads the initial
            // guesses so most beacons pick differently on the first
            // round.  Collisions still happen and still resolve by MAC
            // comparison; they are just rare instead of guaranteed.
            uint8_t pick = mantis_id_preferred(I);
            if (pick == 0 || (I->occupied_mask & (1u << (pick - 1))))
                pick = mantis_id_lowest_free(I);
            if (pick == 0) {
                // Every id in use.  Do NOT transmit: a seventh beacon
                // sharing a slot would corrupt that slot for everyone,
                // and being useless is far better than being harmful.
                I->id = 0;
                I->state = MID_EXHAUSTED;
                I->state_since_ms = now_ms;
                return true;
            }
            I->id = pick;
            I->state = MID_CLAIMING;
            I->state_since_ms = now_ms;
            return true;
        }
        case MID_CLAIMING:
            if (age < MANTIS_CLAIM_CONFIRM_MS) return false;
            // Held it unchallenged. mantis_id_saw() would have knocked us
            // back to LISTENING if anyone lower had objected.
            I->state = MID_COMMITTED;
            I->dirty = true;             // persist it
            I->state_since_ms = now_ms;
            return true;

        case MID_EXHAUSTED:
            // Re-check periodically: a beacon may have been switched off
            // and its id freed, and a spare that silently stays a spare
            // forever is not what anyone wants.
            if (age > 30000) {
                I->occupied_mask = 0;
                I->state = MID_LISTENING;
                I->state_since_ms = now_ms;
            }
            return false;

        default:
            return false;
    }
}

// May this beacon transmit?
//
// Only once committed.  Transmitting while listening would pollute the
// very picture being built, and transmitting while exhausted would
// corrupt a slot that belongs to someone else.
static inline bool mantis_id_may_tx(const MantisIdentity *I) {
    return I->state == MID_COMMITTED && I->id >= 1 && I->id <= MANTIS_MAX_BEACON_ID;
}

static inline const char *mantis_id_state_name(MantisIdState s) {
    switch (s) {
        case MID_LISTENING: return "listening";
        case MID_CLAIMING:  return "claiming";
        case MID_COMMITTED: return "committed";
        case MID_EXHAUSTED: return "mesh full";
        default:            return "boot";
    }
}
