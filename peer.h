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

// Sending — no-ops if peer isn't present.
void peer_send_hello();                                    // broadcast
void peer_send_csi_summary(const PeerCsiSummary &pkt);     // unicast to peer (broadcast if no unicast)
void peer_send_command(uint8_t op, uint8_t a8 = 0,
                       uint16_t a16 = 0, uint32_t a32 = 0);
void peer_send_baseline(const PeerBaselinePacket &pkt);    // unicast to peer
void peer_send_cal_observation(const PeerCalObservation &pkt);  // PROBE→ANCHOR during cal

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
