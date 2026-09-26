#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS PROBE CONTROL
//  The hand-held device drives the fixed receiver
// ═══════════════════════════════════════════════════════════════
//
//  ── WHY THE PROBE HAS TO BE THE ONE IN CHARGE ────────────────
//
//  During setup and calibration the operator is holding the probe and
//  walking around the room.  The anchor is on a shelf somewhere,
//  possibly behind them, possibly a docked pair they deliberately
//  placed and do not want to touch.
//
//  If the anchor owns the script, the operator has to walk back to it
//  at every step.  That is not a usability nuisance -- it corrupts the
//  measurement, because the walk cal is measuring where the operator's
//  BODY is, and a trip back to the shelf is body motion that is not
//  part of the script.
//
//  So the probe advances the script and the anchor follows.
//
//  ── THIS IS THE SAME WIRE FORMAT THE T-DISPLAYS ALREADY USE ──
//
//  PeerCmd / PEER_OP_* is the existing docked-pair protocol.  The M5
//  probes emit exactly those, so the anchor needs NO new code to obey a
//  Core2 or a Cardputer -- it cannot tell the difference between being
//  driven by a T-Display probe and being driven by an M5 one.
//
//  Reusing the format rather than inventing a parallel one is the whole
//  point: a second control protocol would need the anchor to implement
//  and test both, and the two would drift.
//
//  ── A DOCKED PAIR IS ONE ADDRESSEE ───────────────────────────
//
//  The pair shares state over its wired link, so the probe broadcasts
//  and whichever unit hears it propagates to the other.  The probe does
//  not track which of the two it is talking to, and should not: from
//  outside the bar they are one instrument.
// ═══════════════════════════════════════════════════════════════
#include <stdint.h>
#include <string.h>

// Mirrors the PeerCmd in config.h.  Declared here so the M5 firmwares
// do not have to pull in the whole T-Display config, and asserted
// against the real size at the one place both are visible.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  op;
    uint8_t  arg_u8;
    uint16_t arg_u16;
    uint32_t arg_u32;
} MantisPeerCmd;

#ifndef PEER_CMD_MAGIC
  #define PEER_CMD_MAGIC 0xC5CDC0DEUL
#endif

// Ops the probe may send.  Values MUST match config.h; a mismatch here
// would make the anchor act on the wrong instruction rather than
// rejecting it, which is far worse than a dropped packet.
typedef enum : uint8_t {
    MPC_ENTER_STREAMING = 1,
    MPC_RECALIBRATE     = 2,
    MPC_SLEEP           = 3,
    MPC_STATE_HINT      = 4,
    MPC_CAL_STEP_HINT   = 5,
    MPC_CAL_BEGIN       = 6,
    MPC_CAL_END         = 7,
    MPC_UNDOCK_PROBE    = 8,
    MPC_REDOCK_PROBE    = 9,
    MPC_WIRE_PING       = 10,
    MPC_CAL_ROLE_ANCHOR = 11,
    MPC_TAC_STEP        = 12,
} MantisProbeOp;

typedef struct {
    bool     anchor_seen;
    uint32_t last_anchor_ms;
    uint32_t sent, acked;
    uint8_t  last_op;
    uint32_t last_send_ms;
    // The probe repeats an unacknowledged command rather than assuming
    // it arrived.  A lost CAL_BEGIN means the anchor never opens its
    // capture window, and the operator walks an entire leg that records
    // nothing -- with no indication until the model comes out wrong.
    uint8_t  retries;
} MantisProbeLink;

#define MANTIS_PROBE_RETRY_MS   250
#define MANTIS_PROBE_MAX_RETRY  4
// An anchor quiet for this long is treated as absent, and the probe
// falls back to standalone rather than waiting for a partner that is
// not coming.
#define MANTIS_ANCHOR_TIMEOUT_MS 2500

static inline void mantis_probe_link_begin(MantisProbeLink *L) {
    *L = MantisProbeLink{};
}

// Fill a command for transmission.  The caller sends the bytes with
// whatever transport it has (ESP-NOW broadcast on the M5 probes).
static inline uint16_t mantis_probe_cmd(MantisPeerCmd *c, MantisProbeOp op,
                                        uint8_t a8, uint16_t a16, uint32_t a32) {
    c->magic   = PEER_CMD_MAGIC;
    c->op      = (uint8_t)op;
    c->arg_u8  = a8;
    c->arg_u16 = a16;
    c->arg_u32 = a32;
    return (uint16_t)sizeof(*c);
}

// Should we repeat the last command?
//
// Commands that CHANGE STATE are worth repeating; a hint that merely
// mirrors the display is not, because a stale repeat of it would drag
// the anchor backwards to a step the operator has already left.
static inline bool mantis_probe_op_is_critical(MantisProbeOp op) {
    switch (op) {
        case MPC_CAL_BEGIN:
        case MPC_CAL_END:
        case MPC_RECALIBRATE:
        case MPC_UNDOCK_PROBE:
        case MPC_REDOCK_PROBE:
        case MPC_CAL_ROLE_ANCHOR:
            return true;
        default:
            return false;   // hints are idempotent and re-sent anyway
    }
}

static inline bool mantis_probe_should_retry(const MantisProbeLink *L,
                                             uint32_t now_ms) {
    if (L->retries == 0 || L->retries > MANTIS_PROBE_MAX_RETRY) return false;
    if (!mantis_probe_op_is_critical((MantisProbeOp)L->last_op)) return false;
    return (now_ms - L->last_send_ms) >= MANTIS_PROBE_RETRY_MS;
}

static inline void mantis_probe_note_sent(MantisProbeLink *L, MantisProbeOp op,
                                          uint32_t now_ms, bool first) {
    L->last_op      = (uint8_t)op;
    L->last_send_ms = now_ms;
    L->sent++;
    L->retries = first ? 1 : (uint8_t)(L->retries + 1);
}

// The anchor confirms by reporting the state it reached.  Matching on
// the ACHIEVED STATE rather than on an ack byte means a command that
// arrived but could not be obeyed does not read as success.
static inline void mantis_probe_note_anchor(MantisProbeLink *L,
                                            uint8_t anchor_state,
                                            uint8_t expected_state,
                                            uint32_t now_ms) {
    L->anchor_seen    = true;
    L->last_anchor_ms = now_ms;
    if (expected_state != 0xFF && anchor_state == expected_state) {
        L->retries = 0;
        L->acked++;
    }
}

static inline bool mantis_probe_anchor_live(const MantisProbeLink *L,
                                            uint32_t now_ms) {
    return L->anchor_seen &&
           (now_ms - L->last_anchor_ms) < MANTIS_ANCHOR_TIMEOUT_MS;
}

// What to show the operator about the link.  A silent failure here
// wastes a whole calibration walk, so it gets a line on screen.
static inline const char *mantis_probe_link_state(const MantisProbeLink *L,
                                                  uint32_t now_ms) {
    if (!L->anchor_seen)                    return "no anchor";
    if (!mantis_probe_anchor_live(L, now_ms)) return "ANCHOR LOST";
    if (L->retries > MANTIS_PROBE_MAX_RETRY) return "not responding";
    if (L->retries > 0)                      return "confirming...";
    return "linked";
}
