#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CAPABILITIES
//  What this deployment can actually do, derived from what is present
// ═══════════════════════════════════════════════════════════════
//
//  ── WHY THIS IS ONE FILE ─────────────────────────────────────
//
//  Every firmware needs the same answer to "can I offer this?", and
//  every place that answers it separately is a place the answer can
//  drift.  Capability gating scattered across screens is how a menu ends
//  up offering something the solver cannot do, or hiding something it
//  can.
//
//  It also has to explain itself.  "Tactical" greyed out with no reason
//  is a support question; "needs 3+ beacons, 2 present" is not.
//
//  ── THE TIERS ────────────────────────────────────────────────
//
//  Capability grows with hardware, and the steps are geometric rather
//  than arbitrary:
//
//    1 beacon    ONE link.  Presence and motion on that path, and
//                nothing else -- a single line cannot say where along
//                itself something is.  This is classic single-link CSI
//                sensing and it genuinely works; it is the floor, not
//                nothing.
//
//    2 beacons   ONE chord plus the receiver links.  Still no fix:
//                trilateration needs two segments to intersect.  What it
//                adds is a second independent witness, so a disagreement
//                between them is detectable.
//
//    3 beacons   THREE chords.  The first configuration that can
//                LOCALISE: tomography, Doppler and trilateration all
//                become possible, and the geometry can be surveyed.
//                This is the sensible minimum and where the product is
//                centred.
//
//    4 beacons   SIX chords.  Integrity checking becomes possible: with
//                three, one beacon's links are most of the matrix, so a
//                moved beacon and a person look alike.
//
//    6 beacons   FIFTEEN chords, 105 independent segment pairs.  Ghost
//                rejection by witness count works properly here.
//
//  ── RECEIVERS ARE A SEPARATE AXIS ────────────────────────────
//
//    0 receivers   the mesh still runs -- it keeps its own time and
//                  surveys itself.  Nobody is watching, which is a
//                  display problem, not a sensing one.
//    1 receiver    a fixed anchor: full calibration becomes possible.
//    2 docked      a rigid pair: stereo AoA.
//    + cardputer   a KNOWN MOBILE TARGET: self-supervision, live
//                  validation and self-cancellation.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"
#include <string.h>
#include <stdio.h>

typedef struct {
    uint8_t beacons;        // beacons actually heard
    uint8_t tdisplays;      // T-Displays present
    uint8_t cardputers;     // 0 or 1
    bool    docked;         // the two T-Displays are on the stereo bar
    bool    geometry_known; // surveyed, not assumed
} MantisDeployment;

typedef struct {
    // sensing
    bool link_presence;     // 1 beacon: something is on this path
    bool motion_detect;     // any beacon: change over time
    bool localize;          // 3+ beacons: an actual position
    bool doppler;           // velocity vector
    bool trilaterate;       // independent segment fix
    bool witness_vote;      // ghost rejection by independent agreement
    // mesh services
    bool geometry_survey;
    bool integrity_check;
    bool static_map;
    // receiver features
    bool stereo_aoa;
    bool full_cal;
    bool tactical;
    bool probe_cancel;      // subtract the operator, find everyone else
    bool live_validation;
    // derived numbers, for the UI to show rather than the user to guess
    uint8_t chords;
    uint8_t max_witnesses;
    const char *blocker;    // the single most useful thing to add next
} MantisCaps;

static inline MantisCaps mantis_caps(const MantisDeployment *d) {
    MantisCaps c{};
    const uint8_t nb = d->beacons;
    const uint8_t rx = (uint8_t)(d->tdisplays + d->cardputers);

    c.chords        = (uint8_t)(nb >= 2 ? nb * (nb - 1) / 2 : 0);
    c.max_witnesses = nb;

    // ── sensing tiers ──
    // A single link is a real capability, not a degenerate case: it
    // detects presence and motion on that path.  Refusing to offer
    // anything below three beacons would throw away a configuration that
    // genuinely works.
    // SENSING IS SEPARATE FROM DISPLAYING.
    //
    // Two beacons hear each other, so the mesh detects presence with no
    // receiver at all.  Gating this on rx conflated "can the system
    // sense" with "is anyone watching", and produced the nonsense of a
    // deployment that could localise but not detect presence.
    //
    // A single beacon does need a receiver, because one radio alone has
    // no second endpoint to form a link with.
    c.link_presence = (nb >= 2) || (nb >= 1 && rx >= 1);
    c.motion_detect = c.link_presence;

    // Localisation needs three chords to intersect meaningfully.
    c.localize    = (nb >= 3);
    c.doppler     = (c.chords >= 3);
    c.trilaterate = (c.chords >= 2);
    // Witness voting only discriminates once there are more beacons than
    // a single artefact can accidentally involve.
    c.witness_vote = (nb >= 4);

    c.geometry_survey = (nb >= 3);
    c.static_map      = (nb >= 3) && d->geometry_known;
    // With three beacons, one beacon touches two of three chords -- a
    // moved beacon and a person are indistinguishable.  Four is the
    // first count where a row stands out from the matrix.
    c.integrity_check = (nb >= 4);

    // ── receiver features ──
    c.stereo_aoa = (d->tdisplays >= 2) && d->docked;
    // A fixed anchor is what a Full Mode walk is measured against, and a
    // Cardputer is never fixed -- it is in the operator's hand by
    // definition.
    c.full_cal   = (d->tdisplays >= 1);
    c.tactical   = (nb >= 3) && (rx >= 1);

    // THE OPERATOR AS A KNOWN TARGET.
    // Needs a mobile receiver AND something for it to be known relative
    // to.  With fewer than two chords there is nothing to cancel the
    // operator out OF, so offering it would be offering an empty
    // subtraction.
    const bool mobile_probe = (d->cardputers >= 1)
                           || (d->tdisplays >= 1 && !d->docked);
    c.probe_cancel    = mobile_probe && (c.chords >= 2);
    c.live_validation = mobile_probe && c.localize;

    // ── what to add next ──
    // Ordered by how much the next item unlocks, so the advice is
    // actionable rather than a list of everything missing.
    if (nb == 0)                  c.blocker = "add a beacon";
    else if (rx == 0)             c.blocker = "add a display";
    else if (nb < 3)              c.blocker = (nb == 1) ? "add 2 beacons to locate"
                                                        : "add 1 beacon to locate";
    else if (nb < 4)              c.blocker = "add a beacon for fault detection";
    else if (nb < 6)              c.blocker = "add beacons for ghost rejection";
    else if (!d->geometry_known)  c.blocker = "survey the beacon layout";
    else if (!mobile_probe)       c.blocker = "add a probe to cancel yourself out";
    else if (d->tdisplays >= 2 && !d->docked) c.blocker = "dock the pair for AoA";
    else                          c.blocker = "";
    return c;
}

// Why a specific feature is unavailable.  Returned as text the UI shows
// verbatim, because a greyed-out control with no explanation is a
// support question.
static inline const char *mantis_caps_why(const MantisDeployment *d,
                                          const MantisCaps *c,
                                          const char *feature) {
    if (!feature) return "";
    if (!strcmp(feature, "localize") && !c->localize)
        return (d->beacons == 0) ? "no beacons" : "needs 3+ beacons";
    if (!strcmp(feature, "stereo") && !c->stereo_aoa)
        return (d->tdisplays < 2) ? "needs 2 T-Displays" : "dock the pair";
    if (!strcmp(feature, "cal") && !c->full_cal)
        return "needs a fixed anchor (T-Display)";
    if (!strcmp(feature, "tactical") && !c->tactical)
        return (d->beacons < 3) ? "needs 3+ beacons" : "needs a display";
    if (!strcmp(feature, "integrity") && !c->integrity_check)
        return "needs 4+ beacons";
    if (!strcmp(feature, "cancel") && !c->probe_cancel)
        return (c->chords < 2) ? "needs 3+ beacons" : "needs a mobile probe";
    return "";
}

// A one-line description of the deployment, for the header.
static inline void mantis_caps_summary(const MantisDeployment *d, char *out,
                                       int cap) {
    const char *shape =
        (d->tdisplays >= 2 && d->docked) ? "STEREO"
      : (d->tdisplays >= 1)              ? "ANCHOR"
      : (d->cardputers >= 1)             ? "STANDALONE"
                                         : "MESH";
    snprintf(out, cap, "%s %db%s", shape, d->beacons,
             d->cardputers ? "+P" : "");
}
