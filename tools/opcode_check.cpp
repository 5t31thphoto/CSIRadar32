// ═══════════════════════════════════════════════════════════════
//  OPCODE DRIFT CHECK — compiled, not grepped
// ═══════════════════════════════════════════════════════════════
//
//  mantis_control.h mirrors PeerCommand so the M5 probes can drive an
//  anchor with no new code on the anchor side.  A mismatch does not
//  merely fail to work: it makes the anchor act on the WRONG
//  instruction, which is worse than a dropped packet.
//
//  The first version of this check grepped config.h for literal strings
//  like 'PEER_OP_CAL_STEP_HINT     = 5'.  config.h writes
//  'PEER_OP_CAL_STEP_HINT   = 5,' -- three spaces, not five -- so the
//  guard failed on WHITESPACE and broke the build while every opcode
//  was correct.
//
//  A static_assert cannot care about spacing, comments or ordering.
// ═══════════════════════════════════════════════════════════════
#include <cstddef>
#include <cstdint>
#include "config.h"
#include "mantis_control.h"

static_assert(sizeof(MantisPeerCmd) == sizeof(PeerCommand), "cmd size");
static_assert(offsetof(MantisPeerCmd, op)      == offsetof(PeerCommand, op),      "op");
static_assert(offsetof(MantisPeerCmd, arg_u8)  == offsetof(PeerCommand, arg_u8),  "arg_u8");
static_assert(offsetof(MantisPeerCmd, arg_u16) == offsetof(PeerCommand, arg_u16), "arg_u16");
static_assert(offsetof(MantisPeerCmd, arg_u32) == offsetof(PeerCommand, arg_u32), "arg_u32");

static_assert((int)MPC_ENTER_STREAMING == (int)PEER_OP_ENTER_STREAMING, "ENTER_STREAMING");
static_assert((int)MPC_RECALIBRATE     == (int)PEER_OP_RECALIBRATE,     "RECALIBRATE");
static_assert((int)MPC_SLEEP           == (int)PEER_OP_SLEEP,           "SLEEP");
static_assert((int)MPC_STATE_HINT      == (int)PEER_OP_STATE_HINT,      "STATE_HINT");
static_assert((int)MPC_CAL_STEP_HINT   == (int)PEER_OP_CAL_STEP_HINT,   "CAL_STEP_HINT");
static_assert((int)MPC_CAL_BEGIN       == (int)PEER_OP_CAL_BEGIN,       "CAL_BEGIN");
static_assert((int)MPC_CAL_END         == (int)PEER_OP_CAL_END,         "CAL_END");
static_assert((int)MPC_UNDOCK_PROBE    == (int)PEER_OP_UNDOCK_PROBE,    "UNDOCK_PROBE");
static_assert((int)MPC_REDOCK_PROBE    == (int)PEER_OP_REDOCK_PROBE,    "REDOCK_PROBE");
static_assert((int)MPC_WIRE_PING       == (int)PEER_OP_WIRE_PING,       "WIRE_PING");
static_assert((int)MPC_CAL_ROLE_ANCHOR == (int)PEER_OP_CAL_ROLE_ANCHOR, "CAL_ROLE_ANCHOR");
static_assert((int)MPC_TAC_STEP        == (int)PEER_OP_TAC_STEP,        "TAC_STEP");
static_assert((int)MPC_REMOTE_KEY      == (int)PEER_OP_REMOTE_KEY,      "REMOTE_KEY");
static_assert((int)MPC_PROBE_HELLO     == (int)PEER_OP_PROBE_HELLO,     "PROBE_HELLO");
static_assert((int)ST_COUNT == 26, "AppState grew: extend mantis_anchor_state_name()");

// The anchor status packet travels anchor -> probe; both sides must lay
// it out identically.
static_assert(MANTIS_ANCHOR_STATUS_MAGIC == PEER_ANCHOR_STATUS_MAGIC, "status magic");
static_assert(MANTIS_PEER_HELLO_MAGIC    == PEER_HELLO_MAGIC,         "hello magic");
static_assert(sizeof(MantisAnchorStatus) == sizeof(PeerAnchorStatus), "status size");
static_assert(offsetof(MantisAnchorStatus, remote_ack) == offsetof(PeerAnchorStatus, remote_ack), "ack");
static_assert(offsetof(MantisAnchorStatus, title) == offsetof(PeerAnchorStatus, title), "title");
static_assert(offsetof(MantisAnchorStatus, track) == offsetof(PeerAnchorStatus, track), "track");
static_assert(MANTIS_AS_TRACKS == ANCHOR_STATUS_TRACKS, "track count");
static_assert(MAS_FRAME_MESH == AS_FRAME_MESH && MAT_CORROBORATED == AT_CORROBORATED, "flags");
static_assert(MAS_STEREO == AS_STEREO && MAS_ALERT == AS_ALERT && MAT_MESH_ONLY == AT_MESH_ONLY, "flags");

int main() { return 0; }
