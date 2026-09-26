// ═══════════════════════════════════════════════════════════════
//  peer.h — RX↔RX peer link over ESP-NOW.
//
//  Sits alongside the beacon-side ESP-NOW rx path in csi.cpp.
//  Peer-related broadcasts and unicasts are demultiplexed by magic
//  number in the payload, so a beacon that happens to broadcast a
//  4-byte counter can't be mistaken for a peer packet (and vice
//  versa — the magic values are chosen with no overlap in the low
//  bytes to make sniffing on the wire clearer).
//
//  Same firmware runs on both units.  During PEER_DISCOVERY both
//  units broadcast a HELLO for ~4 s while listening; lower MAC
//  wins → ROLE_PRIMARY, other → ROLE_SECONDARY.  If no HELLO is
//  ever received from another unit, we fall through to ROLE_SOLO
//  and behave like the old single-RX firmware.
// ═══════════════════════════════════════════════════════════════
#pragma once

#include "config.h"

void peer_begin();                      // register unicast peer (broadcast)
void peer_start_discovery();            // reset counters, begin broadcasting HELLO
void peer_tick();                       // called from loop; heartbeats + timeouts
bool peer_discovery_done();             // discovery window elapsed
void peer_resolve_role();               // decide PRIMARY/SECONDARY/SOLO from what we heard

// ── v0.9: wired transport for docked mode ────────────────────
// Docked, the receivers are 6 cm apart and physically joined, so the
// link belongs on a wire rather than on the radio that is busy
// sniffing CSI.  Undocked, the probe falls back to ESP-NOW
// automatically.  Every packet struct and handler is shared: the wire
// feeds the same peer_try_consume() demux.
void peer_wire_begin();                 // open the UART (call once, in setup)
void peer_wire_poll();                  // called from loop; reassembles frames
bool peer_wire_link_up();               // a valid frame arrived recently
PeerTransport peer_active_transport();  // what the next packet will use
void peer_wire_stats(uint32_t *frames, uint32_t *crc_errors);

// Sending — no-ops if peer isn't present.
void peer_send_hello();                                    // broadcast
void peer_send_csi_summary(const PeerCsiSummary &pkt);     // unicast to peer (broadcast if no unicast)
void peer_send_command(uint8_t op, uint8_t a8 = 0,
                       uint16_t a16 = 0, uint32_t a32 = 0);
void peer_send_baseline(const PeerBaselinePacket &pkt);    // unicast to peer
void peer_send_cal_observation(const PeerCalObservation &pkt);  // PROBE→ANCHOR during cal

// ── v0.8: probe undock mode ───────────────────────────────────
void peer_send_probe_position(const PeerProbePositionPacket &pkt);  // PROBE→ANCHOR ~10 Hz
void peer_send_track_state(const PeerTrackStatePacket &pkt);
// v0.9: anchor mirrors its tripwire link to the probe (TW_REMOTE only).
void peer_send_tripwire(uint8_t status, uint8_t beacon_id, uint16_t pct);
bool peer_tripwire_fresh();        // ANCHOR→PROBE ~5 Hz

// ANCHOR side.  True while the probe has told us it is undocked AND is
// still sending fresh position fixes.  Both conditions matter: a probe
// that walks out of ESP-NOW range never gets to send a REDOCK, so
// staleness is what actually ends self-suppression.
bool peer_probe_is_undocked();
bool peer_has_recent_probe_position();
bool peer_get_probe_position(float out_pos[2], float out_cov[3], float *out_conf);

// PROBE side.  Most recent track list from the anchor, plus its age so
// the UI can show "anchor unreachable" rather than a frozen picture.
const PeerTrackStatePacket *peer_last_track_state();
uint32_t peer_track_state_age_ms();

// Clear undock/probe-position state (on redock, or when re-calibrating).
void peer_reset_probe_undock();

// Recv paths (called from ESP-NOW recv callback in csi.cpp)
void peer_handle_hello(const uint8_t *src_mac, const PeerHelloPacket &pkt);
void peer_handle_csi_summary(const PeerCsiSummary &pkt);
void peer_handle_command(const PeerCommand &cmd);
void peer_handle_baseline(const PeerBaselinePacket &pkt);
void peer_handle_cal_observation(const PeerCalObservation &pkt);

// Demultiplex: returns true if the packet was a peer packet (any type)
// and consumed by this module.  csi.cpp calls this first; if false, the
// packet is treated as a beacon ESP-NOW frame.
bool peer_try_consume(const uint8_t *src_mac, const uint8_t *data, int len);
