#pragma once
// ═══════════════════════════════════════════════════════════════
//  MantisSec — RF OBSERVATION ARCHIVE  (PSRAM cold tier)
// ═══════════════════════════════════════════════════════════════
//
//  WHY THIS EXISTS
//
//  The Rust inference core keeps a bounded coreset of MANTIS_RF_MAX_NODES
//  response samples in internal DRAM.  That bound is not a storage
//  decision, it is a SPEED decision: observe() scores every node against
//  every feature, so the hot set has to live where random access is
//  single-cycle.
//
//  When a walk produces more samples than the coreset can hold, the core
//  keeps the most NOVEL ones and discards the rest.  That is the right
//  call for a real-time scorer, and it throws away real measurements.
//
//  8 MB of PSRAM means we no longer have to throw them away -- but PSRAM
//  is 40-80 MB/s through a cache, not 600+ MB/s single-cycle, so it
//  cannot host the scoring loop.  Hence two tiers:
//
//      HOT  (internal DRAM, in Rust)  the coreset scored every frame
//      COLD (PSRAM, here)             every observation ever taken
//
//  Nothing in the runtime path touches the cold tier.  It is written
//  during calibration and read only when re-fitting, which happens while
//  the operator is standing still and a few milliseconds do not matter.
//
//  WHAT IT BUYS
//    - re-fit the chart over ALL samples instead of the surviving 72
//    - re-learn feature weights from the full distribution
//    - replay a walk after changing a parameter, without re-walking it
//    - honest coverage reporting: how much did we actually keep?
//
//  RESOURCE RULE
//  This adds ZERO internal DRAM.  If PSRAM is absent the archive simply
//  reports unavailable and every caller degrades to the coreset-only
//  behaviour that exists today -- the tactical path must never depend on
//  PSRAM being present.
// ═══════════════════════════════════════════════════════════════
#include <stdint.h>
#include <stddef.h>
#include "rfcore.h"

// Ceiling on archived samples.  2000 x 260 B = ~508 KB of 8 MB.
// Chosen so a very long deployment plus several re-walks still fits with
// room to spare, not because 2000 is a target to fill.
#define RF_ARCHIVE_MAX_NODES  2000

// Bring the archive up.  Safe to call more than once.  Returns false if
// PSRAM is unavailable or the allocation failed; callers must treat that
// as "cold tier disabled", never as an error.
bool rf_archive_begin();

// True once storage is actually held.
bool rf_archive_available();

// Append one observation.  Silently ignored when unavailable or full --
// the hot tier is authoritative for runtime, so a full archive degrades
// re-fit quality and nothing else.
void rf_archive_add(const MantisRfNode *n);

// How many samples are held, and the ceiling.
int  rf_archive_count();
int  rf_archive_capacity();

// Read one back.  Returns null if idx is out of range.
const MantisRfNode *rf_archive_get(int idx);

// Drop everything (a new deployment).  Keeps the allocation.
void rf_archive_clear();

// Push the archive back through the Rust core: reset, replay every
// archived sample, finalize.  This is the re-fit.  Returns the number of
// samples replayed, or 0 when unavailable.
//
// Costs real time (hundreds of ms at full capacity) and must only be
// called when nothing is being measured.
int  rf_archive_refit();

// Bytes actually held, for the resource readout.
size_t rf_archive_bytes();
