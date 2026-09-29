// ═══════════════════════════════════════════════════════════════
//  LINK SIMULATION — probe remote keys against a lossy radio
// ═══════════════════════════════════════════════════════════════
//  The probe presses the anchor's buttons with mantis_control.h's
//  sequence/ack protocol; the anchor side below applies each sequence
//  number ONCE, exactly as mesh_anchor_remote_key() does, and echoes it
//  in its 4 Hz status (sent immediately after a key, as the anchor does).
//
//  Invariants, under 30% loss in BOTH directions:
//    - every press the probe confirms was applied exactly once
//    - no press is ever applied twice (retries are idempotent)
//    - the probe never reports "linked" while a press is outstanding
// ═══════════════════════════════════════════════════════════════
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "../mantis_control.h"

static uint32_t rng = 777;
static bool lost() { rng = rng * 1664525u + 1013904223u; return (rng >> 24) < 77; }  // ~30%

int main() {
    MantisProbeLink L; mantis_probe_link_begin(&L);
    uint16_t anchor_seq = 0; int applied[5] = {0}; int applied_total = 0;
    uint32_t status_due = 0; int dup = 0;
    int pressed = 0, confirmed = 0, failed = 0;
    for (uint32_t t = 0; t < 120000; t += 10) {
        // The operator presses a key every 1.5 s.
        if (t % 1500 == 0) {
            const MantisRemoteKey k = (MantisRemoteKey)(1 + (pressed % 4));
            mantis_probe_key_begin(&L, k, t); pressed++;
            if (!lost()) { if (L.key_seq != anchor_seq) { anchor_seq = L.key_seq; applied[k]++; applied_total++; status_due = t; } else dup++; }
        }
        // Probe retry path (same as mantis_probe_service).
        if (mantis_probe_key_due(&L, t)) {
            L.key_tries++; L.key_sent_ms = t;
            if (!lost()) {
                if (L.key_seq != anchor_seq) { anchor_seq = L.key_seq; applied[L.key_code]++; applied_total++; }
                status_due = t;   // anchor answers at once either way
            }
        }
        // Anchor status: every 250 ms, or immediately after a key.
        if (t >= status_due) {
            status_due = t + 250;
            MantisAnchorStatus st = {}; st.magic = MANTIS_ANCHOR_STATUS_MAGIC; st.remote_ack = anchor_seq;
            if (!lost()) {
                const bool was = L.key_tries > 0;
                if (mantis_probe_on_status(&L, &st, t)) confirmed++;
                (void)was;
            }
        }
        if (L.key_tries > MANTIS_PROBE_MAX_RETRY) { failed++; L.key_tries = 0; }
        if (L.key_tries && !strcmp(mantis_probe_link_state(&L, t), "linked")) { printf("FAIL: linked while sending\n"); return 1; }
    }
    printf("pressed %d  applied %d  confirmed %d  gave up %d  duplicate applies %d\n",
           pressed, applied_total, confirmed, failed, dup);
    int bad = 0;
    if (dup) { printf("FAIL: a press was applied twice\n"); bad++; }
    if (applied_total > pressed) { printf("FAIL: more applies than presses\n"); bad++; }
    if (confirmed > applied_total) { printf("FAIL: confirmed a press that never landed\n"); bad++; }
    if (confirmed + failed < pressed - 1) { printf("FAIL: presses neither confirmed nor reported failed\n"); bad++; }
    if (confirmed < pressed * 95 / 100) { printf("FAIL: under 95%% of presses confirmed at 30%% loss\n"); bad++; }
    printf(bad ? "LINK SIM: FAILED\n" : "LINK SIM: every press landed once, confirmation is honest\n");
    return bad ? 1 : 0;
}
