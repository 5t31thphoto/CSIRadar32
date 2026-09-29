#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS BEACON IDENTITY
//  One firmware for every beacon, stable ids that survive a reboot
// ═══════════════════════════════════════════════════════════════
//
//  ── THE HARD REQUIREMENT IS STABILITY, NOT UNIQUENESS ────────
//
//  Everything downstream is keyed to beacon id: the surveyed geometry,
//  the per-link baselines, the static room map and the operator's "walk
//  to B2".  If ids shuffle on reboot all of those silently describe the
//  wrong box.  So an id is CLAIMED ONCE and then PERSISTED.
//
//  ── THREE WAYS AN ID GETS SET, IN PRIORITY ORDER ─────────────
//
//    1. FLASH-TIME   MANTIS_FIXED_BEACON_ID, if a build sets one.
//    2. STORED       the id claimed previously, from NVS.  Normal path.
//    3. CLAIMED      listen, take the LOWEST free id, persist it.
//
//  ── WHY LOWEST FREE, NOT A MAC-HASHED GUESS ──────────────────
//
//  The previous version picked a MAC-derived "preferred" id to spread
//  blank beacons apart.  It worked, and it produced HOLES: three beacons
//  landing on ids 2, 4 and 5.  Every receiver loops 1..max-id, so ids 1
//  and 3 became permanent phantom beacons that never reported, the
//  baseline-coverage test could never reach 90%, and the mesh sat in
//  "learning room" until the 45 s backstop fired.  Contiguous ids are
//  what the rest of the system assumes; lowest-free gives them.
//
//  Blank beacons booting together are separated in TIME instead: each
//  listens for a uid-derived extra 0..1.5 s, so the first to commit is
//  already on the air when the next one decides.
//
//  ── COLLISIONS RESOLVE BY UID, IN ONE EXCHANGE ───────────────
//
//  If two boxes still land on one id, the lower uid keeps it and the
//  other re-claims.  The uid travels in every air frame (and in the MAC,
//  see mantis_air.h), so both sides compute the same verdict from the
//  same bytes.  The previous version compared our FACTORY MAC against
//  the peer's SPOOFED 1A:... MAC -- a comparison both sides lose, so
//  both yielded, both re-claimed the same next id, and they could
//  chase each other up the id space.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"
#include <string.h>

// Highest valid beacon id.  slot = id-1 and slot 6 is the GUARD slot
// CHORD/CHORUS transmit in, so the top id is MANTIS_SLOTS - 2.

#ifndef MANTIS_FIXED_BEACON_ID
  #define MANTIS_FIXED_BEACON_ID 0
#endif

#define MANTIS_CLAIM_LISTEN_MS   2000
#define MANTIS_CLAIM_JITTER_MS   1500
#define MANTIS_CLAIM_CONFIRM_MS  1200

typedef enum : uint8_t {
    MID_BOOT = 0,
    MID_LISTENING,     // building a picture of who is out there
    MID_CLAIMING,      // picked an id, holding it quietly for a moment
    MID_COMMITTED,     // the id is ours and is persisted
    MID_EXHAUSTED,     // every id taken; we are a listener, not a beacon
} MantisIdState;

typedef struct {
    MantisIdState state;
    uint8_t  id;                 // 0 until decided
    uint8_t  occupied_mask;      // bit k set = id (k+1) heard in use
    uint16_t uid;                // ours, the tiebreak key
    uint32_t state_since_ms;
    uint32_t listen_ms;          // this box's listen window (base + jitter)
    bool     from_storage;
    bool     dirty;              // needs writing to NVS
    uint8_t  challenges;         // tiebreaks lost this boot
} MantisIdentity;

static inline void mantis_id_begin(MantisIdentity *I, uint16_t my_uid,
                                   uint8_t stored_id, uint32_t now_ms) {
    *I = MantisIdentity{};
    I->uid = my_uid ? my_uid : 1;
    I->state_since_ms = now_ms;
    I->listen_ms = MANTIS_CLAIM_LISTEN_MS
                 + (uint32_t)((I->uid * 2654435761u) >> 16) % MANTIS_CLAIM_JITTER_MS;
    if (MANTIS_FIXED_BEACON_ID > 0) {
        I->id = MANTIS_FIXED_BEACON_ID;
        I->state = MID_COMMITTED;
        return;
    }
    if (stored_id >= 1 && stored_id <= MANTIS_MAX_BEACON_ID) {
        I->id = stored_id;
        I->from_storage = true;
    }
    I->state = MID_LISTENING;
}

// Is our uid the winner against another box on the same id?
// Lower uid keeps it.  A tie (1 in ~13,000) keeps both on the air for
// the membership layer to flag; nothing here can break it.
static inline bool mantis_id_we_win(uint16_t ours, uint16_t theirs) {
    return ours < theirs;
}

// Call for every frame heard from a beacon.
static inline void mantis_id_saw(MantisIdentity *I, uint8_t other_id,
                                 uint16_t other_uid, uint32_t now_ms) {
    if (other_id == 0 || other_id > MANTIS_MAX_BEACON_ID) return;
    if (other_uid == I->uid) return;            // our own echo
    I->occupied_mask |= (uint8_t)(1u << (other_id - 1));
    if (other_id != I->id || I->state == MID_BOOT) return;
    if (MANTIS_FIXED_BEACON_ID > 0) return;     // pinned at build time
    if (mantis_id_we_win(I->uid, other_uid)) return;
    // We lose.  Give it up and claim again.
    I->challenges++;
    I->id = 0;
    I->from_storage = false;
    I->state = MID_LISTENING;
    I->state_since_ms = now_ms;
}

static inline uint8_t mantis_id_lowest_free(const MantisIdentity *I) {
    for (uint8_t k = 1; k <= MANTIS_MAX_BEACON_ID; k++)
        if (!(I->occupied_mask & (1u << (k - 1)))) return k;
    return 0;
}

// Advance the claim.  Returns true when the state changed.
static inline bool mantis_id_tick(MantisIdentity *I, uint32_t now_ms) {
    const uint32_t age = now_ms - I->state_since_ms;
    switch (I->state) {
        case MID_LISTENING: {
            if (age < I->listen_ms) return false;
            if (I->from_storage && I->id &&
                !(I->occupied_mask & (1u << (I->id - 1)))) {
                I->state = MID_COMMITTED;
                I->state_since_ms = now_ms;
                return true;
            }
            const uint8_t pick = mantis_id_lowest_free(I);
            if (pick == 0) {
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
            // Someone committed to our pick while we held it: back off.
            if (I->occupied_mask & (1u << (I->id - 1))) {
                I->id = 0;
                I->state = MID_LISTENING;
                I->state_since_ms = now_ms;
                I->listen_ms = 300;              // we already know the room
                return true;
            }
            if (age < MANTIS_CLAIM_CONFIRM_MS) return false;
            I->state = MID_COMMITTED;
            I->dirty = true;
            I->state_since_ms = now_ms;
            return true;
        case MID_EXHAUSTED:
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

// The occupancy picture ages: a beacon powered off must not hold its id
// forever in the minds of the others.  Call periodically (every few s).
static inline void mantis_id_forget_occupancy(MantisIdentity *I) {
    I->occupied_mask = 0;
}

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
