#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS MESH MEMBERSHIP & TIMEKEEPING
//  The beacons organise themselves.  Receivers are optional.
// ═══════════════════════════════════════════════════════════════
//
//  A supported deployment is "one mobile probe and three beacons, no
//  anchor".  So the mesh cannot depend on a receiver for slot
//  assignment, for time, or for anything else -- a receiver is a
//  LISTENER that may or may not be present, and the mesh must behave
//  identically whether zero, one or two of them are switched on.
//
//  Three problems, each with a deliberately boring answer, because
//  boring is what survives a cold start in a room with nobody watching.
//
//  ── 1. WHICH SLOT DO I TRANSMIT IN? ──────────────────────────
//
//  slot = id - 1.  That is the entire algorithm.
//
//  No negotiation, no claim/defend exchange, no convergence time.  The
//  id is set once at flash time and is unique by construction, so
//  deriving the slot from it cannot fail to converge -- it never
//  converges, it is simply true from the first packet.
//
//  A negotiated scheme would be strictly worse: it needs a claim
//  protocol, a conflict rule, and a settling period during which the
//  schedule cannot be trusted -- and it would STILL be wrong if two
//  beacons were flashed with the same id.  Which brings us to:
//
//  ── 2. WHAT IF TWO BEACONS SHARE AN ID? ──────────────────────
//
//  They transmit in the same slot and collide, permanently.  It is a
//  deployment error, invisible from the air unless you look for it, and
//  it degrades the system in a way that reads as poor RF rather than as
//  a misconfiguration.
//
//  So we look for it.  A beacon hearing ITS OWN id from elsewhere raises
//  a conflict flag and STOPS TRANSMITTING.  Yielding rather than
//  fighting means one of the pair keeps working, the fault is reported
//  instead of silently degrading everything, and the operator is told
//  exactly which id to re-flash.
//
//  ── 3. WHO DEFINES SUPERFRAME ZERO? ──────────────────────────
//
//  The lowest-id beacon currently alive.  Also not an election: every
//  node computes it from its own membership view, so there are no
//  election messages, no term numbers and no split brain.
//
//  Failover preserves continuity because the successor adopts ITS OWN
//  epoch estimate, which was already synced to the departed timekeeper.
//  Frame boundaries do not move when leadership does -- and they must
//  not, because a jump in seq would invalidate every in-flight report at
//  once.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"
#include "mantis_sched.h"

// Alive if heard within three superframes: long enough to ride out two
// lost transmissions, short enough that a dead beacon's slot is not
// wasted for long.
#define MANTIS_MEMBER_TIMEOUT_US  (3 * (int64_t)MANTIS_FRAME_US)

// How long to wait after the incumbent timekeeper goes quiet before
// assuming the role.  Staggered by id so two candidates never take over
// on the same frame: the lower id acts first and the higher one sees it
// and stands down.
#define MANTIS_TK_BASE_US         (5 * (int64_t)MANTIS_FRAME_US)
#define MANTIS_TK_STAGGER_US      (2 * (int64_t)MANTIS_FRAME_US)

#define MANTIS_SLOT_TK_BIT 0x80
static inline uint8_t mantis_frame_slot(const MantisAirFrame *f) {
    return (uint8_t)(f->slot & ~MANTIS_SLOT_TK_BIT);
}
static inline bool mantis_frame_is_tk(const MantisAirFrame *f) {
    return (f->slot & MANTIS_SLOT_TK_BIT) != 0;
}
typedef struct {
    int64_t  last_heard_us;
    uint32_t last_seq;
    int8_t   rssi;
    bool     present;
    bool     claims_timekeeper;
} MantisMember;

typedef struct {
    uint8_t      my_id;             // 1..6, set at flash time
    MantisMember member[MANTIS_SLOTS];
    MantisSched  sched;

    bool     id_conflict;           // heard our own id from elsewhere
    int64_t  conflict_seen_us;

    bool     is_timekeeper;
    uint8_t  tk_id;                 // who we believe it is; 0 = nobody yet
    int64_t  tk_last_heard_us;

    uint32_t frames_tx, frames_rx, id_conflicts;
} MantisMembership;

static inline void mantis_mesh_init(MantisMembership *M, uint8_t my_id) {
    *M = MantisMembership{};
    M->my_id = my_id;
    M->sched.my_slot = (uint8_t)(my_id - 1);   // identity IS the schedule
}

// How many beacons are alive right now, including us.
static inline uint8_t mantis_mesh_live(const MantisMembership *M, int64_t now) {
    uint8_t n = M->id_conflict ? 0 : 1;        // a yielded node is not live
    for (uint8_t i = 1; i < MANTIS_SLOTS; i++) {
        if (i == M->my_id || !M->member[i].present) continue;
        if (now - M->member[i].last_heard_us <= MANTIS_MEMBER_TIMEOUT_US) n++;
    }
    return n;
}

// Lowest live id.  Computed, never elected.
static inline uint8_t mantis_mesh_lowest_live(const MantisMembership *M, int64_t now) {
    uint8_t best = M->id_conflict ? 0xFF : M->my_id;
    for (uint8_t i = 1; i < MANTIS_SLOTS; i++) {
        if (i == M->my_id || !M->member[i].present) continue;
        if (now - M->member[i].last_heard_us > MANTIS_MEMBER_TIMEOUT_US) continue;
        if (i < best) best = i;
    }
    return (best == 0xFF) ? 0 : best;
}

// Fold in a received air frame.  This is the ONLY place membership,
// sync and conflict state are updated, so there is one path to reason
// about rather than three that can disagree.
static inline void mantis_mesh_on_frame(MantisMembership *M,
                                        const MantisAirFrame *f,
                                        int64_t rx_us, int8_t rssi,
                                        bool sender_is_timekeeper) {
    if (!f || f->magic != MANTIS_AIR_MAGIC) return;
    if (f->beacon_id == 0 || f->beacon_id >= MANTIS_SLOTS) return;
    M->frames_rx++;

    // ── our own id, from someone else ──
    // Yield, do not fight.  Both beacons transmitting in one slot means
    // neither is usable; one standing down leaves a working system and a
    // diagnosable fault.
    if (f->beacon_id == M->my_id) {
        if (!M->id_conflict) M->id_conflicts++;
        M->id_conflict      = true;
        M->conflict_seen_us = rx_us;
        M->is_timekeeper    = false;
        return;
    }

    MantisMember *m = &M->member[f->beacon_id];
    m->present           = true;
    m->last_heard_us     = rx_us;
    m->last_seq          = f->seq;
    m->rssi              = rssi;
    m->claims_timekeeper = sender_is_timekeeper;

    // ── sync from ANY packet ──
    // Every transmission carries (seq, slot), so the epoch is derivable
    // from whoever happens to be heard.  Preferring the timekeeper keeps
    // everyone on one origin; accepting anyone means a node with a bad
    // link to the timekeeper still stays aligned via its neighbours.
    const bool prefer = sender_is_timekeeper || !M->sched.synced;
    if (prefer) {
        // MASK THE FLAG OFF.  The timekeeper bit rides in the slot
        // field's high bit, and passing the raw byte here made slot 0
        // read as 128 -- so every synced node computed an epoch ~128
        // slots in the past, its own slot never came round, and it never
        // transmitted at all.  One unmasked byte silently disabled the
        // entire mesh: no conflict detection, no failover, no reports.
        mantis_sched_sync(&M->sched, rx_us, f->seq, mantis_frame_slot(f));
        if (sender_is_timekeeper) {
            M->tk_id            = f->beacon_id;
            M->tk_last_heard_us = rx_us;
            // Someone lower than us is keeping time: stand down.
            if (M->is_timekeeper && f->beacon_id < M->my_id)
                M->is_timekeeper = false;
        }
    }
}

// Decide whether we should be keeping time.  Called every loop; cheap.
//
// Two conditions, both necessary: we are the lowest live id, and the
// incumbent has been quiet long enough that this is not a transient.
static inline void mantis_mesh_tick(MantisMembership *M, int64_t now) {
    // Retire stale members so `lowest_live` reflects reality.
    for (uint8_t i = 1; i < MANTIS_SLOTS; i++)
        if (M->member[i].present &&
            (now - M->member[i].last_heard_us) > MANTIS_MEMBER_TIMEOUT_US)
            M->member[i].present = false;

    if (M->id_conflict) { M->is_timekeeper = false; return; }

    const uint8_t lowest = mantis_mesh_lowest_live(M, now);
    if (lowest != M->my_id) {
        // Not our job.  If the incumbent is gone the lower node will take
        // over first; we only act if IT does not.
        if (M->tk_id != 0 && (now - M->tk_last_heard_us) > MANTIS_MEMBER_TIMEOUT_US)
            M->tk_id = 0;
        return;
    }

    // We are the lowest live id.  Take over once the incumbent has been
    // silent past our staggered deadline.
    //
    // The stagger is by id so two candidates cannot assume the role on
    // the same frame; the lower acts first, transmits, and the higher
    // sees that and stands down before its own deadline expires.
    const int64_t wait = MANTIS_TK_BASE_US
                       + (int64_t)M->my_id * MANTIS_TK_STAGGER_US;
    const bool incumbent_gone =
        (M->tk_id == 0) || ((now - M->tk_last_heard_us) > wait);

    if (incumbent_gone && !M->is_timekeeper) {
        // CONTINUITY: adopt our own epoch estimate, which was synced to
        // the departed timekeeper.  Starting a fresh epoch here would
        // jump seq and invalidate every in-flight report simultaneously.
        if (!M->sched.synced) {
            M->sched.epoch_us     = now;
            M->sched.seq          = 0;
            M->sched.synced       = true;
            M->sched.last_sync_us = now;
        }
        M->is_timekeeper    = true;
        M->tk_id            = M->my_id;
        M->tk_last_heard_us = now;
    }
    if (M->is_timekeeper) M->tk_last_heard_us = now;
}

// May we transmit in this instant?
//
// Three independent reasons not to, and every one of them is a case
// where transmitting would actively make things worse rather than
// merely be useless.
static inline bool mantis_mesh_may_tx(const MantisMembership *M, int64_t now) {
    if (M->id_conflict)              return false;  // would collide, forever
    if (!M->sched.synced)            return false;  // would land anywhere
    if (mantis_sched_stale(&M->sched, now) && !M->is_timekeeper)
        return false;                                // drifted; listen instead
    return mantis_sched_may_tx(&M->sched, now);
}

// Populate an outgoing frame.  The timekeeper flag travels in the slot
// field's high bit -- there is no spare byte, and a slot only needs
// three bits for eight slots.
static inline void mantis_mesh_fill_frame(const MantisMembership *M,
                                          MantisAirFrame *f, int64_t now) {
    uint32_t seq = 0; int32_t off = 0;
    const uint8_t slot = mantis_sched_slot(&M->sched, now, &seq, &off);
    f->magic     = MANTIS_AIR_MAGIC;
    f->slot      = (uint8_t)(slot | (M->is_timekeeper ? MANTIS_SLOT_TK_BIT : 0));
    f->beacon_id = M->my_id;
    f->seq       = seq;
    f->epoch_us  = (uint32_t)(M->sched.epoch_us & 0xFFFFFFFF);
}

