#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS SLOT SCHEDULER
// ═══════════════════════════════════════════════════════════════
//
//  One superframe, MANTIS_SLOTS slots, absolute-deadline scheduling.
//  Compiled into BOTH firmwares so the two cannot disagree about where
//  a slot boundary is.
//
//  ABSOLUTE DEADLINES, NOT ELAPSED TIME.
//
//  The old beacon cadence did `last_tx = now` after every send, so each
//  period became (period + however late the poll was) and the error was
//  never corrected -- it random-walked.  Measured: 446 ms of drift in 30
//  seconds, against a slot that is 4.2 ms wide.  Slots were gone inside
//  a second, which is why staggering transmits had achieved nothing.
//
//  Here every boundary is computed from a frame epoch:
//
//      slot_start(k) = epoch + k * MANTIS_SLOT_US
//
//  so lateness on one slot never displaces the next.  What remains is
//  poll latency alone, ~1 ms against a 4.2 ms slot with a 3.7 ms
//  tolerance.
//
//  THE EPOCH IS SHARED, NOT GUESSED.
//
//  Beacons anchor their epoch to a received sync, so all radios agree on
//  slot boundaries without any of them owning a clock.  Relative crystal
//  drift at 20 ppm moves neighbours ~1.2 ms per minute against a 4.2 ms
//  slot, so a re-sync every ~30 s holds alignment with large margin.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"

// Microsecond clock.  esp_timer_get_time() on target; the host harness
// substitutes its own so the scheduler can be tested off-device.
#ifndef MANTIS_NOW_US
  #if defined(ARDUINO) || defined(ESP32)
    #include <esp_timer.h>
    #define MANTIS_NOW_US() ((int64_t)esp_timer_get_time())
  #else
    #include <stdint.h>
    int64_t mantis_host_now_us(void);
    #define MANTIS_NOW_US() mantis_host_now_us()
  #endif
#endif

typedef struct {
    int64_t  epoch_us;      // start of superframe 0
    uint32_t seq;           // current superframe number
    uint8_t  my_slot;       // this radio's transmit slot; 0xFF = never transmits
    uint8_t  n_slots_used;  // how many beacon slots are occupied
    bool      synced;       // has an epoch been established?
    int64_t  last_sync_us;  // for staleness
} MantisSched;

// Establish or re-anchor the epoch.  `rx_us` is the local time at which
// the sync reference arrived; `their_epoch_us` is the sender's frame
// start, carried in MantisAirFrame.
//
// Anchoring to the RECEPTION of a shared RF event means every radio
// derives the same boundaries from the same instant, with no clock
// master and no round-trip estimation.
static inline void mantis_sched_sync(MantisSched *s, int64_t rx_us,
                                     uint32_t seq, uint8_t slot_of_sender) {
    // Wind back to the start of superframe ZERO, not merely to the start
    // of the superframe the sender was in.
    //
    // mantis_sched_slot() derives the frame number from (now - epoch),
    // so an epoch anchored only to the current frame boundary makes
    // every follower compute frame 0 forever while the timekeeper counts
    // up.  Sequence numbers are the idempotency key for every report, so
    // a follower stuck at 0 would have its reports rejected as duplicates
    // from the second frame onward -- the mesh would look alive on the
    // air and deliver nothing.
    //
    // Subtracting seq * FRAME_US puts every radio on ONE origin, which
    // is what makes seq globally meaningful rather than per-node.
    s->epoch_us     = rx_us
                    - (int64_t)slot_of_sender * MANTIS_SLOT_US
                    - (int64_t)seq * MANTIS_FRAME_US;
    s->seq          = seq;
    s->synced       = true;
    s->last_sync_us = rx_us;
}

// Which slot are we in right now, and how far into it?
static inline uint8_t mantis_sched_slot(const MantisSched *s,
                                        int64_t now_us,
                                        uint32_t *out_seq,
                                        int32_t *out_offset_us) {
    if (!s->synced) { if (out_seq) *out_seq = 0;
                      if (out_offset_us) *out_offset_us = 0; return 0; }
    const int64_t d = now_us - s->epoch_us;
    if (d < 0) { if (out_seq) *out_seq = 0;
                 if (out_offset_us) *out_offset_us = 0; return 0; }
    const uint32_t frame = (uint32_t)(d / MANTIS_FRAME_US);
    const int64_t  into  = d - (int64_t)frame * MANTIS_FRAME_US;
    const uint8_t  slot  = (uint8_t)(into / MANTIS_SLOT_US);
    if (out_seq)       *out_seq = frame;
    if (out_offset_us) *out_offset_us = (int32_t)(into - (int64_t)slot * MANTIS_SLOT_US);
    return slot;
}

// Absolute start time of a given slot in a given superframe.
static inline int64_t mantis_sched_slot_start(const MantisSched *s,
                                              uint32_t frame, uint8_t slot) {
    return s->epoch_us + (int64_t)frame * MANTIS_FRAME_US
                       + (int64_t)slot  * MANTIS_SLOT_US;
}

// Is it this radio's turn, and is there still room to transmit?
//
// The tail guard matters: a transmission started too late in the slot
// would bleed into the next beacon's, which is the collision the frame
// exists to prevent.  Better to miss one sounding than corrupt someone
// else's.
static inline bool mantis_sched_may_tx(const MantisSched *s, int64_t now_us) {
    if (!s->synced || s->my_slot >= MANTIS_SLOTS) return false;
    int32_t off; uint32_t seq;
    if (mantis_sched_slot(s, now_us, &seq, &off) != s->my_slot) return false;
    return off < (int32_t)(MANTIS_SLOT_US - MANTIS_TX_BUDGET_US);
}

// Is the band meant to be quiet right now?
//
// Anything heard here is either a radio that has lost sync or an
// external interferer -- both worth knowing about, and neither
// distinguishable from a measurement without a slot that guarantees
// silence.
static inline bool mantis_sched_is_silence(const MantisSched *s, int64_t now_us) {
    if (!s->synced) return false;
    return mantis_sched_slot(s, now_us, nullptr, nullptr) == MANTIS_SLOT_SILENCE;
}

// Sync staleness.  Past this, slots can no longer be trusted to align
// and the radio should fall back to listening until it hears a frame.
#define MANTIS_SYNC_STALE_US  (30ll * 1000000ll)
static inline bool mantis_sched_stale(const MantisSched *s, int64_t now_us) {
    return !s->synced || (now_us - s->last_sync_us) > MANTIS_SYNC_STALE_US;
}
