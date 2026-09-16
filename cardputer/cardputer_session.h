#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CARDPUTER SESSIONS
//  Recording RF so it can be replayed against changed inference
// ═══════════════════════════════════════════════════════════════
//
//  ── WHY THIS IS WORTH THE CARD SLOT ──────────────────────────
//
//  A room is never twice the same.  Furniture moves, doors open, people
//  walk past outside.  So comparing two versions of the inference by
//  running them live is comparing them on two different problems, and
//  the result tells you almost nothing.
//
//  Recording the observations lets the SAME RF be pushed through both.
//  That is the only way to answer "did that change help", and no amount
//  of live testing substitutes for it.
//
//  It also means a field failure can come home.  A session that produced
//  a wrong answer is reproducible on the bench, forever, instead of
//  being a story about something that happened once.
//
//  ── WHAT IS RECORDED, AND WHY THAT LEVEL ─────────────────────
//
//  Observations, not raw CSI.  Raw would be ~200 kB/s per beacon and
//  would fill a card in minutes; the FrameObservation is what every
//  solver actually consumes, so replaying it exercises the entire
//  inference stack faithfully while costing a few hundred bytes a frame.
//
//  What is deliberately NOT captured is anything derived: no tracks, no
//  positions, no chart. Those are OUTPUTS.  Recording them would make a
//  replay agree with the original by construction, which is exactly the
//  thing a replay must be able to disagree about.
//
//  ── FAILURE BEHAVIOUR ────────────────────────────────────────
//
//  Recording never blocks sensing.  A slow card, a full card, a card
//  yanked mid-session -- every one of these stops the recording and
//  leaves the live system untouched.  A device that stutters its
//  tracking because the SD write queue backed up has traded the thing it
//  is for the thing it was merely noting down.
// ═══════════════════════════════════════════════════════════════
#include "cardputer_platform.h"

#define CP_SESSION_MAGIC   0x4D53   // 'MS'
#define CP_SESSION_VERSION 1

// File header, written once.
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  version;
    uint8_t  n_beacons;
    uint32_t start_ms;
    uint32_t frame_count;     // patched on close; 0 means "still open"
    uint16_t frame_bytes;     // size of each record that follows
    uint8_t  mode;            // CardputerMode at capture time
    uint8_t  flags;
    float    beacon_x[8];     // geometry AS MEASURED at capture time
    float    beacon_y[8];
    char     note[32];        // operator's own label
} CardputerSessionHeader;

#define CP_SF_GEOMETRY_MEASURED 0x01   // geometry came from a survey, not a ring
#define CP_SF_HAS_IMU           0x02
#define CP_SF_TRUNCATED         0x04   // closed by card removal or full card

// One recorded frame.  Fixed size, so seeking to frame N is arithmetic
// rather than a scan -- which is what makes scrubbing a replay usable.
typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint32_t t_ms;
    // Per-beacon observation: what the solvers consume.
    int16_t  amp_q8[8];
    int16_t  phase_q12[8];
    uint8_t  quality[8];
    int8_t   rssi[8];
    int8_t   noise[8];
    uint8_t  fresh_mask;
    // The probe's own state, which is what makes a replay honest: the
    // operator was somewhere, and where they were changes what the RF
    // means.  Without it a replay has lost the known-target advantage.
    int16_t  probe_x_q10, probe_y_q10;
    int16_t  imu_heading_q12;
    uint16_t steps;
    uint8_t  flags;
} CardputerSessionFrame;

typedef struct {
    bool     recording;
    bool     replaying;
    uint32_t frames_written;
    uint32_t frames_total;     // when replaying
    uint32_t replay_pos;
    uint32_t bytes_written;
    uint32_t dropped;          // frames the card could not keep up with
    bool     card_present;
    bool     card_full;
    char     path[64];
} CardputerSession;

// Bytes per second at a given frame rate -- so the UI can tell the
// operator how long their card will last BEFORE they start, rather than
// discovering it when the card fills mid-deployment.
static inline uint32_t cp_session_bytes_per_s(uint8_t hz) {
    return (uint32_t)sizeof(CardputerSessionFrame) * hz;
}

static inline uint32_t cp_session_minutes_free(uint64_t free_bytes, uint8_t hz) {
    const uint32_t bps = cp_session_bytes_per_s(hz);
    if (bps == 0) return 0;
    const uint64_t secs = free_bytes / bps;
    return (uint32_t)(secs / 60);
}

// Should we record this frame?
//
// Decimation is offered because 30 Hz is far more than a replay needs to
// reproduce a walk, and halving the rate doubles the recording time on
// the same card.  The rate is stored in the header so a replay knows
// what it is looking at rather than assuming.
static inline bool cp_session_should_write(const CardputerSession *s,
                                           uint32_t seq, uint8_t decimate) {
    if (!s->recording || s->card_full || !s->card_present) return false;
    if (decimate <= 1) return true;
    return (seq % decimate) == 0;
}

// A recording that stopped for a reason the operator did not choose must
// SAY so.  A truncated session that looks complete is worse than no
// session, because it will be trusted.
static inline const char *cp_session_status(const CardputerSession *s) {
    if (s->replaying)      return "REPLAY";
    if (!s->card_present)  return "NO CARD";
    if (s->card_full)      return "CARD FULL";
    if (s->recording)      return s->dropped ? "REC (dropping)" : "REC";
    return "idle";
}
