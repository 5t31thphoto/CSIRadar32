#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS AIR PROTOCOL  —  shared contract, receiver AND beacon
// ═══════════════════════════════════════════════════════════════
//
//  This file is the ONE definition of how the radio is used.  It is
//  compiled into both the receiver and the beacon firmware and the two
//  must agree byte for byte; there is a static check at the bottom.
//
//  WHY A FRAME AT ALL
//
//  The previous design had no structure: beacons free-ran at a commanded
//  rate, control traffic shared the channel with the measurement, and
//  the receiver inferred everything from arrival times.  Measured on the
//  old scheme at 6 beacons:
//
//      sounding  ....................  7.2% airtime
//      control   .................... 16.9% airtime   <-- 2.3x the signal
//
//  Control was using more of the channel than the measurement, and every
//  control packet perturbed the very channel being measured.  A frame
//  fixes that by giving every transmission a reserved place, and by
//  moving control OFF this radio entirely (see MANTIS_CTRL_BLE).
//
//  THE FRAME
//
//      superframe = MANTIS_SLOTS slots, one beacon per slot
//
//      slot 0..N-1   beacon k transmits.  EVERY other radio -- the other
//                    beacons AND both receivers -- is listening.
//      slot N..6     guard (unused beacon slots at N<6)
//      slot 7        SILENCE.  Nothing transmits, anywhere.
//
//  Two things fall out of this for free:
//
//  1. BISTATIC LINKS AT NO COST.  A beacon idle in slot k is a receiver
//     in slot k.  Six beacons go from 12 links to 42, and the new ones
//     are chords ACROSS the room rather than spokes from the edge -- a
//     person between B2 and B5 blocks that chord and nothing else.  Not
//     one extra packet is transmitted to get them.
//
//  2. A REAL NOISE FLOOR.  In the silence slot the receiver measures the
//     band while KNOWING nothing is being sent.  Previously noise_floor
//     was whatever the PHY happened to report during someone else's
//     packet, which is why it was expected to carry no information.
// ═══════════════════════════════════════════════════════════════
#include <stdint.h>

// ── Frame geometry ────────────────────────────────────────────
// 8 slots keeps the arithmetic to shifts and masks, leaves room for the
// 6-beacon maximum, and still affords a guard and a silence slot.
#define MANTIS_SLOTS            8
#define MANTIS_SLOT_SILENCE     (MANTIS_SLOTS - 1)   // slot 7
#define MANTIS_SLOT_GUARD       (MANTIS_SLOTS - 2)   // slot 6

// Superframe period in microseconds.  30 Hz nominal; the silence and
// guard slots mean the EFFECTIVE per-beacon sounding rate is
//   30 * (MANTIS_SLOTS - 2) / MANTIS_SLOTS = 22.5 Hz
// which is still comfortably above the 20 Hz capture decimation and far
// above the ~5 Hz of human gross motion.
#define MANTIS_FRAME_US         33333u
#define MANTIS_SLOT_US          (MANTIS_FRAME_US / MANTIS_SLOTS)   // 4166 us

// A transmission occupies roughly 400 us at MCS0.  The slot is 4166 us,
// so a beacon may be up to ~3.7 ms late before it bleeds into the next
// slot.  That is the jitter budget the absolute-deadline scheduler has
// to hold, and it holds it to ~1 ms.
#define MANTIS_TX_BUDGET_US     500u

// ── Fixed PHY ─────────────────────────────────────────────────
// Rate adaptation is DISABLED by design.  With a fixed rate and fixed
// power, RSSI becomes comparable across time and across beacons, and the
// PHY-rate feature stops being a record of the rate controller's mood.
// We give up a little robustness and gain a calibrated amplitude axis.
#define MANTIS_CHANNEL          11
#define MANTIS_TX_POWER_QDBM    60      // 15 dBm in 0.25 dBm units

// ── Control plane ─────────────────────────────────────────────
// 1 = control (schedule, commands, telemetry, peer summaries) rides on
// BLE, and the Wi-Fi channel carries ONLY the sounding waveform.
// 0 = legacy: control shares the measurement channel.
//
// This is the change that makes the others trustworthy: it removes the
// 16.9% of contaminating airtime, which is more than the sounding uses.
#define MANTIS_CTRL_BLE         1

// ── Air frame payload ─────────────────────────────────────────
// Deliberately tiny.  The CHANNEL ESTIMATE comes from the preamble
// (L-LTF / HT-LTF), not from the payload, so payload bytes buy nothing
// and cost airtime.  These fields exist only to identify the sounding.
#define MANTIS_AIR_MAGIC        0x4D41ul     // 'MA'
// TWO DIFFERENT KINDS OF NAME, and keeping them separate matters.
//
//   beacon_id  a SCHEDULING RESOURCE.  It picks the transmit slot
//              (slot = id-1) and it is assignable, so it can change when
//              hardware is swapped or a collision resolves.
//
//   uid        the PHYSICAL BOX.  Derived from the MAC, so it is unique
//              by construction, permanent, and needs no protocol, no
//              storage and no claim exchange to establish.
//
// Everything learned about a location -- surveyed geometry, per-link
// quiet-room baselines, the static room map -- is keyed to UID, never to
// beacon_id.  That was the trap in keying it to the assignable name: an
// id reshuffle would leave every one of those silently describing the
// wrong physical box, and the system would look completely healthy while
// being wrong.
//
// The operator never sees either of these.  The UI names beacons by the
// SLOT BINDING established during the walk -- B1 is whichever box you
// walked to first -- so the assignable name and the permanent name are
// both implementation details.
typedef struct __attribute__((packed)) {
    uint16_t magic;        // MANTIS_AIR_MAGIC
    uint8_t  slot;         // which slot this was sent in
    uint8_t  beacon_id;    // 1..6, the scheduling resource
    uint16_t uid;          // MAC-derived, permanent, identifies the BOX
    uint32_t seq;          // superframe counter, the RF event identity
    uint32_t epoch_us;     // sender's frame-start time, for drift tracking
} MantisAirFrame;

// 16 bits of MAC.  Collision probability across six beacons is about
// 1 in 13,000 -- and a collision is DETECTABLE (two boxes reporting the
// same uid from different ids), where a silent re-key is not.
static inline uint16_t mantis_uid_from_mac(const uint8_t *mac) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < 6; i++) { h ^= mac[i]; h *= 16777619u; }
    h ^= h >> 16;
    const uint16_t u = (uint16_t)(h & 0xFFFF);
    return u ? u : 1;      // 0 means "unknown", so never hand it out
}

// ── Beacon-to-beacon report ───────────────────────────────────
// A beacon that heard another beacon reports a COMPACT summary, never
// raw CSI.  Sent on the control plane (BLE when enabled), so it never
// competes with the sounding.
typedef struct __attribute__((packed)) {
    uint8_t  hearer_id;    // which beacon measured this
    uint8_t  heard_id;     // which beacon it heard
    uint32_t seq;          // the superframe it refers to
    int8_t   rssi;
    int8_t   noise;
    uint16_t amp_q12;      // mean |H| over selected subcarriers, Q4.12
    uint16_t var_q12;      // dispersion across those subcarriers, Q4.12
    uint8_t  n_sc;         // how many subcarriers contributed
    uint8_t  flags;
} MantisBistaticReport;

// ── Silence-slot noise report ─────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  reporter_id;  // 0 = receiver, else beacon id
    uint32_t seq;
    int8_t   noise_floor;  // measured with the band known-quiet
    int8_t   rssi_max;     // loudest thing heard during SILENCE
    uint8_t  flags;        // bit0: something WAS heard -> external interferer
} MantisSilenceReport;

// ── FTM survey ────────────────────────────────────────────────
// 802.11mc Fine Timing Measurement.  Available in ESP-IDF for the S3 and
// C3 (esp_wifi_ftm_initiate_session), using PHY-level timestamps rather
// than the microsecond TSF -- roughly 0.5-2 m indoors after averaging.
//
// Far too coarse to track a person.  Exactly right for SURVEYING where
// the beacons are, which is the thing the old system never knew and
// instead invented as a ring by slot index.
//
// At 6 beacons: 12 receiver->beacon ranges + 15 beacon->beacon ranges =
// 27 constraints for 12 unknowns.  Over-determined by 15, so the solve
// has a residual and the residual is a trust metric.
#define MANTIS_FTM_BURSTS       16      // per session; more = less variance
#define MANTIS_FTM_MIN_VALID    6       // fewer than this -> range unusable
typedef struct __attribute__((packed)) {
    uint8_t  from_id;      // 0 = receiver
    uint8_t  to_id;
    uint16_t dist_cm;      // measured distance
    uint16_t sigma_cm;     // spread across bursts -- the trust signal
    uint8_t  n_valid;
    uint8_t  flags;
} MantisRange;

// ── Compile-time contract ─────────────────────────────────────
// Both firmwares include this header; if either side edits a struct
// without the other, these fire at build time rather than producing
// silently misaligned reads on the air.
#ifdef __cplusplus
static_assert(sizeof(MantisAirFrame)        == 14, "MantisAirFrame layout changed");
static_assert(sizeof(MantisBistaticReport)  == 14, "MantisBistaticReport layout changed");
static_assert(sizeof(MantisSilenceReport)   ==  8, "MantisSilenceReport layout changed");
static_assert(sizeof(MantisRange)           ==  8, "MantisRange layout changed");
static_assert(MANTIS_SLOTS >= 8,            "need a guard and a silence slot");
static_assert(MANTIS_SLOT_US > MANTIS_TX_BUDGET_US * 4,
              "slot must be comfortably longer than a transmission");
#endif
