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

// Ops the probe may send.  Values MUST match config.h (asserted in
// tools/opcode_check.cpp).
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
    MPC_REMOTE_KEY      = 13,
    MPC_PROBE_HELLO     = 14,
} MantisProbeOp;

// REMOTE_KEY arg_u8 values: the anchor's two buttons.
typedef enum : uint8_t {
    MRK_LEFT = 1, MRK_RIGHT = 2, MRK_LEFT_LONG = 3, MRK_RIGHT_LONG = 4,
} MantisRemoteKey;

#define MANTIS_PROBE_KIND_CORE2      1
#define MANTIS_PROBE_KIND_CARDPUTER  2

// ── Anchor status: mirror of PeerAnchorStatus in config.h ─────
#define MANTIS_ANCHOR_STATUS_MAGIC 0xC5A57A75UL
#define MANTIS_PEER_HELLO_MAGIC    0xC51EE511UL
#define MANTIS_AS_TRACKS 6
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t uptime_ms;
    uint8_t  app_state;
    uint8_t  dash_view;
    uint16_t remote_ack;
    uint8_t  flags;
    uint8_t  beacons;
    uint8_t  mesh_beacons;
    uint8_t  n_tracks;
    uint8_t  step, steps;
    uint8_t  progress;
    uint8_t  mesh_contacts;
    int16_t  anchor_x_q10, anchor_y_q10;
    char     title[16];
    char     hint[40];
    char     left[12];
    char     right[12];
    struct __attribute__((packed)) {
        int16_t x_q10, y_q10;
        uint8_t conf;
        uint8_t flags;
    } track[MANTIS_AS_TRACKS];
} MantisAnchorStatus;
#define MAS_CAL_COMPLETE   0x01
#define MAS_TACTICAL       0x02
#define MAS_ALERT          0x04
#define MAS_FRAME_MESH     0x08
#define MAS_MESH_BASELINE  0x10
#define MAS_REMOTE_DRIVES  0x20
#define MAS_STEREO         0x40
#define MAT_ACTIVE         0x01
#define MAT_SELF           0x02
#define MAT_CORROBORATED   0x04
#define MAT_MESH_ONLY      0x08

// ── The link, as the probe sees it ────────────────────────────
//
// State is never INFERRED here.  The anchor broadcasts what it is doing;
// the probe shows exactly that.  A key press is confirmed when the anchor
// reports having APPLIED its sequence number, so "sent" and "done" are
// never confused on screen.
typedef struct {
    bool     anchor_seen;
    uint32_t last_anchor_ms;
    MantisAnchorStatus st;       // the latest status, verbatim
    uint32_t sent, acked;
    uint16_t key_seq;            // last key sequence we issued
    uint8_t  key_code;           // ...and which key it was
    uint32_t key_sent_ms;
    uint8_t  key_tries;          // 0 = nothing outstanding
    uint32_t last_hello_ms;
} MantisProbeLink;

#define MANTIS_PROBE_RETRY_MS    180
#define MANTIS_PROBE_MAX_RETRY   6
#define MANTIS_ANCHOR_TIMEOUT_MS 2500
#define MANTIS_PROBE_HELLO_MS    1000

static inline void mantis_probe_link_begin(MantisProbeLink *L) {
    *L = MantisProbeLink{};
    L->key_seq = 1;
}

static inline uint16_t mantis_probe_cmd(MantisPeerCmd *c, MantisProbeOp op,
                                        uint8_t a8, uint16_t a16, uint32_t a32) {
    c->magic   = PEER_CMD_MAGIC;
    c->op      = (uint8_t)op;
    c->arg_u8  = a8;
    c->arg_u16 = a16;
    c->arg_u32 = a32;
    return (uint16_t)sizeof(*c);
}

// A status packet arrived.  Returns true if it confirmed our key.
static inline bool mantis_probe_on_status(MantisProbeLink *L,
                                          const MantisAnchorStatus *s,
                                          uint32_t now_ms) {
    L->st = *s;
    L->st.title[sizeof(L->st.title) - 1] = 0;
    L->st.hint[sizeof(L->st.hint) - 1]   = 0;
    L->st.left[sizeof(L->st.left) - 1]   = 0;
    L->st.right[sizeof(L->st.right) - 1] = 0;
    L->anchor_seen    = true;
    L->last_anchor_ms = now_ms;
    if (L->key_tries && s->remote_ack == L->key_seq) {
        L->key_tries = 0;
        L->acked++;
        return true;
    }
    return false;
}

static inline bool mantis_probe_anchor_live(const MantisProbeLink *L,
                                            uint32_t now_ms) {
    return L->anchor_seen &&
           (now_ms - L->last_anchor_ms) < MANTIS_ANCHOR_TIMEOUT_MS;
}

// Start a key press.  A newer press replaces an unconfirmed older one:
// the operator's latest intent is the one that matters.
static inline uint16_t mantis_probe_key_begin(MantisProbeLink *L,
                                              MantisRemoteKey k, uint32_t now_ms) {
    L->key_seq  = (uint16_t)(L->key_seq + 1);
    if (L->key_seq == 0) L->key_seq = 1;
    L->key_code = (uint8_t)k;
    L->key_tries = 1;
    L->key_sent_ms = now_ms;
    L->sent++;
    return L->key_seq;
}

static inline bool mantis_probe_key_due(const MantisProbeLink *L, uint32_t now_ms) {
    return L->key_tries > 0 && L->key_tries <= MANTIS_PROBE_MAX_RETRY
        && (now_ms - L->key_sent_ms) >= MANTIS_PROBE_RETRY_MS;
}

static inline const char *mantis_probe_link_state(const MantisProbeLink *L,
                                                  uint32_t now_ms) {
    if (!L->anchor_seen)                      return "no anchor";
    if (!mantis_probe_anchor_live(L, now_ms)) return "ANCHOR LOST";
    if (L->key_tries > MANTIS_PROBE_MAX_RETRY) return "not responding";
    if (L->key_tries > 0)                     return "sending...";
    return "linked";
}

// Anchor AppState names, for the probe's screen.  Values mirror config.h's
// AppState enum, which is append-only by contract (it travels on the air).
static inline const char *mantis_anchor_state_name(uint8_t s) {
    static const char *const N[] = {
        "SPLASH", "PEER SEARCH", "ROLE", "DISCOVERY", "GEOMETRY",
        "CAL INTRO", "ANCHOR PLACE", "EMPTY ROOM", "CAL WALK", "TRAINING",
        "CAL RESULTS", "RX ASSEMBLY", "RADAR", "SETTINGS", "SLEEP",
        "SECONDARY", "MOBILE PROBE", "DEBUG LOG", "PICK CARRY",
        "TAC INTRO", "TAC DEPLOY", "TAC RETURN", "TAC CIRCUIT",
        "TAC BASELINE", "TAC CRITIQUE", "TAC CHECK" };
    return (s < sizeof(N) / sizeof(N[0])) ? N[s] : "?";
}
