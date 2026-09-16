// ═══════════════════════════════════════════════════════════════
//  MantisSec — RF OBSERVATION ARCHIVE  (PSRAM cold tier)
//  See rfarchive.h for why this exists.
// ═══════════════════════════════════════════════════════════════
#include "rfarchive.h"
#include "config.h"
#include <string.h>

#if defined(ARDUINO) || defined(ESP32)
  #include <esp_heap_caps.h>
  #define RF_ARCHIVE_HAS_PSRAM_API 1
#else
  #define RF_ARCHIVE_HAS_PSRAM_API 0
  #include <stdlib.h>
#endif

static MantisRfNode *s_arc      = nullptr;
static int           s_count    = 0;
static bool          s_tried    = false;

bool rf_archive_begin() {
    if (s_arc) return true;
    if (s_tried) return false;      // one attempt; do not thrash the heap
    s_tried = true;

    const size_t bytes = (size_t)RF_ARCHIVE_MAX_NODES * sizeof(MantisRfNode);

#if RF_ARCHIVE_HAS_PSRAM_API
    // MALLOC_CAP_SPIRAM explicitly: this must NEVER fall back to internal
    // DRAM.  Half a megabyte of internal heap would strand the canvas,
    // the kernel and the peer buffers, and the archive is a nice-to-have.
    // Better to have no cold tier than to break the hot path for it.
    s_arc = (MantisRfNode *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (!s_arc) {
        MSLOG("[rfarc] PSRAM alloc of %u B failed - cold tier disabled\n",
              (unsigned)bytes);
        return false;
    }
#else
    s_arc = (MantisRfNode *)malloc(bytes);   // host builds only
    if (!s_arc) return false;
#endif

    s_count = 0;
    MSLOG("[rfarc] cold tier up: %d nodes x %u B = %u KB PSRAM (%d feats)\n",
          RF_ARCHIVE_MAX_NODES, (unsigned)sizeof(MantisRfNode),
          (unsigned)(bytes / 1024), MANTIS_RF_FEATS);
    return true;
}

bool rf_archive_available() { return s_arc != nullptr; }
int  rf_archive_count()     { return s_count; }
int  rf_archive_capacity()  { return s_arc ? RF_ARCHIVE_MAX_NODES : 0; }

size_t rf_archive_bytes() {
    return s_arc ? (size_t)RF_ARCHIVE_MAX_NODES * sizeof(MantisRfNode) : 0;
}

void rf_archive_add(const MantisRfNode *n) {
    if (!s_arc || !n) return;
    if (s_count >= RF_ARCHIVE_MAX_NODES) {
        // Full.  Do NOT evict: the hot tier already does novelty-based
        // eviction and is authoritative for runtime.  An archive that
        // silently reshaped itself would make a re-fit unreproducible.
        static bool warned = false;
        if (!warned) {
            warned = true;
            MSLOGLN("[rfarc] archive FULL - later samples not retained");
        }
        return;
    }
    // memcpy rather than assignment: this crosses into PSRAM and the
    // struct is 260 B, so a burst copy is what we want.
    memcpy(&s_arc[s_count], n, sizeof(MantisRfNode));
    s_count++;
}

const MantisRfNode *rf_archive_get(int idx) {
    if (!s_arc || idx < 0 || idx >= s_count) return nullptr;
    return &s_arc[idx];
}

void rf_archive_clear() {
    s_count = 0;
}

int rf_archive_refit() {
    if (!s_arc || s_count == 0) return 0;

    // Replay EVERY archived sample through the core.  The core applies
    // its own novelty test and keeps the best coreset it can -- but now
    // it chooses from the full population instead of from whatever
    // happened to arrive before it filled up.
    mantis_rf_reset();
    mantis_rf_begin();
    for (int i = 0; i < s_count; i++) mantis_rf_add_node(&s_arc[i]);
    const uint8_t ok = mantis_rf_finalize();

    MSLOG("[rfarc] refit from %d archived samples -> %s\n",
          s_count, ok ? "ready" : "NOT ready");
    return s_count;
}
