// ═══════════════════════════════════════════════════════════════
//  mesh_anchor.h — the beacon mesh, inside the anchor
// ═══════════════════════════════════════════════════════════════
//
//  The anchor has always had two independent views of the room and used
//  one.  Its OWN view is the beacon->anchor spokes: CSI captured here,
//  learned by the walk calibration, solved by scene.cpp.  The MESH view
//  is every beacon->beacon chord, measured by the beacons themselves and
//  published as perspectives -- tomography that needs no calibration walk
//  at all.  The anchor was hearing every one of those packets and
//  throwing them away.
//
//  This module keeps them.  It runs the SAME mantis_receiver.h the
//  hand-held probes run (so all three devices reach the same verdict
//  from the same packets), aligns the mesh's surveyed frame onto this
//  radar using the beacons both frames contain, and exposes:
//
//    - mesh contacts in the radar's own coordinates
//    - corroboration: does the mesh ALSO see someone where a track is?
//    - an underglow of the tomography field for the radar view
//    - the status broadcast that lets a Core2 / Cardputer ADV mirror and
//      drive this anchor
//
//  ── OPPORTUNISTIC BY CONSTRUCTION ────────────────────────────
//
//    1 beacon            no mesh; anchor radar only, nothing drawn here
//    2 beacons           mesh presence; alignment exact (no mirror info)
//    3+ beacons          mesh contacts on the radar, BEFORE calibration
//    3+ and calibrated   corroboration rings on anchor tracks
//    + a hand-held probe status broadcast, remote control
//
//  Every step works with whatever is present and says what the next
//  piece of hardware would add.
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "config.h"

void mesh_anchor_begin();

// Wi-Fi task: copy a beacon's ESP-NOW payload for the loop to parse.
void mesh_anchor_enqueue(uint8_t beacon_id, const uint8_t *data, int len);

// loop(): drain, solve (self rate-limited), realign, broadcast status.
void mesh_anchor_tick();

bool     mesh_anchor_present();     // any live mesh beacon
uint8_t  mesh_anchor_live();        // live mesh beacons
bool     mesh_anchor_aligned();     // mesh frame mapped onto this radar
float    mesh_anchor_align_rms();   // radar units
const char *mesh_anchor_state();    // one-line summary

struct MeshContact {
    float   x, y;          // RADAR (scene) frame
    float   conf;          // 0..1
    uint8_t cls;           // MantisFuseClass
    bool    believed;
};
int  mesh_anchor_contacts(MeshContact *out, int max);
int  mesh_anchor_believed();

// Is there a believed mesh contact within reach of this radar position?
bool mesh_anchor_corroborates(float sx, float sy);

// Radar view overlay: underglow + mesh contacts, drawn in the radar's
// own pixel mapping.  Called from render_radar_view().
void mesh_anchor_draw_overlay(int cx, int cy, float pix_per_unit,
                              int clip_y0, int clip_y1);

// DV_MESH dashboard view.
void mesh_anchor_draw_view();

// ── Hand-held probes ──────────────────────────────────────────
void mesh_anchor_note_probe(uint8_t kind);
bool mesh_anchor_probe_present();
uint8_t mesh_anchor_probe_kind();
// Apply a remote button press exactly once per sequence number.
void mesh_anchor_remote_key(uint8_t key, uint16_t seq);
