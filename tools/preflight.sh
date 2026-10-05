#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
#  PRE-FLIGHT — every firmware, every translation unit, plus the mesh
# ═══════════════════════════════════════════════════════════════
#
#  Runs in seconds with nothing but g++.  CI runs it before spending ten
#  minutes installing toolchains; run it locally before every drop.
#
#   1. SKETCHES compiled exactly as arduino-cli lays them out: flat
#      directory, ARDUINO defined, vendor header SHAPES from tools/ardshim.
#   2. EVERY RECEIVER .cpp, not just the .ino.  The previous pre-flight
#      only syntax-checked the sketch file, so csi.cpp / scene.cpp /
#      ui.cpp -- 90% of the anchor -- were never looked at.
#   3. A HOST LINK of the whole receiver: undefined or duplicate symbols
#      across translation units fail here instead of in the Xtensa link.
#   4. The probe control opcodes, compiled against config.h.
#   5. THE MESH SIMULATION, across 1..6 beacons and an id conflict: the
#      shared headers running together, proving the radios AGREE, not
#      merely that they compile.
#
#  What this cannot prove: that the real vendor headers match the shims
#  exactly, or anything about RF.  Those are CI's and the bench's jobs.
# ═══════════════════════════════════════════════════════════════
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT=$(pwd)
SHIM="$ROOT/tools/ardshim"
CXX="${CXX:-g++}"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
fail=0
FLAGS="-std=gnu++17 -DARDUINO=200 -DESP32 -Wno-psabi"

# ── 1. sketches ────────────────────────────────────────────────
# name : sketch : extra header dir
SKETCHES="
receiver:CSI-Radar-S3.ino:.
beacon:CSI-Beacon-Mantis/CSI-Beacon-Mantis.ino:
cardputer:cardputer/Mantis-Cardputer-Adv.ino:cardputer
core2:core2/Mantis-Core2.ino:core2
"
for entry in $SKETCHES; do
  name="${entry%%:*}"; rest="${entry#*:}"
  sketch="${rest%%:*}"; extra="${rest#*:}"
  d="$TMP/$name"; mkdir -p "$d"
  cp "$ROOT"/mantis_*.h "$d"/
  if [ "$extra" = "." ]; then
    cp "$ROOT"/*.h "$ROOT"/*.cpp "$d"/
    cp "$ROOT"/rust/rfcore/*.h "$d"/
  elif [ -n "$extra" ]; then
    cp "$ROOT/$extra"/*.h "$d"/
  fi
  cp "$ROOT/$sketch" "$d/sketch_main.cpp"
  sed -i '1i #include <Arduino.h>' "$d/sketch_main.cpp"

  for inc in $(grep -ho '#include "[^"]*"' "$d"/*.cpp "$d"/*.h 2>/dev/null \
               | sed 's/.*"\(.*\)"/\1/' | sort -u); do
    if [ ! -f "$d/$inc" ]; then
      echo "::error::$name: '$inc' is not in the sketch directory"; fail=1
    fi
  done

  if out=$("$CXX" -fsyntax-only $FLAGS -I "$SHIM" -I "$d" "$d/sketch_main.cpp" 2>&1); then
    echo "  ok   $name sketch"
  else
    echo "::error::$name sketch fails to compile"; echo "$out" | grep -E "error" | head -15; fail=1
  fi
done

# ── 2 + 3. every receiver translation unit, then a host link ────
d="$TMP/receiver"
objs=()
for f in "$d"/*.cpp; do
  o="${f%.cpp}.o"
  if out=$("$CXX" -c $FLAGS -w -I "$SHIM" -I "$d" "$f" -o "$o" 2>&1); then
    objs+=("$o")
  else
    echo "::error::receiver: $(basename "$f") fails to compile"; echo "$out" | grep -E "error" | head -15; fail=1
  fi
done
n_tu=$(ls "$d"/*.cpp | wc -l)
if [ "${#objs[@]}" -eq "$n_tu" ]; then echo "  ok   receiver: all $n_tu translation units"; fi
cat > "$TMP/hostmain.cpp" <<'CPP'
#include <LovyanGFX.hpp>
namespace lgfx { namespace fonts { const IFont Font0, Font2, Font4, Font6, Font7, Font8,
  FreeMono9pt7b, FreeMonoBold9pt7b, FreeSans9pt7b, FreeSansBold9pt7b, FreeSansBold12pt7b, TomThumb; } }
int main() { return 0; }
CPP
"$CXX" -c $FLAGS -w -I "$SHIM" "$TMP/hostmain.cpp" -o "$TMP/hostmain.o"
# The Rust core's functions, for the HOST link only (the firmware links
# the real libmantis_rfcore.a).
"$CXX" -c $FLAGS -w -I "$SHIM" -I "$ROOT/rust/rfcore" "$ROOT/tools/hoststub_rfcore.cpp" -o "$TMP/rfstub.o"
if out=$("$CXX" "${objs[@]}" "$TMP/hostmain.o" "$TMP/rfstub.o" -o "$TMP/rxlink" 2>&1); then
  echo "  ok   receiver links (no undefined or duplicate symbols)"
else
  echo "::error::receiver does not link"; echo "$out" | grep -E "undefined|multiple" | head -15; fail=1
fi

# ── 4. probe opcodes vs config.h ───────────────────────────────
if "$CXX" -std=c++17 -fsyntax-only -I . -I tools/hostshim tools/opcode_check.cpp; then
  echo "  ok   probe control contract matches config.h"
else
  echo "::error::probe control contract drifted from config.h"; fail=1
fi

# ── 5. the mesh, simulated ─────────────────────────────────────
if "$CXX" -std=c++17 -O2 -w -o "$TMP/mesh_sim" tools/mesh_sim.cpp; then
  for args in "6" "6 1" "5" "4" "3" "2" "1"; do
    if out=$("$TMP/mesh_sim" $args 2>&1); then
      echo "  ok   mesh sim [$args]: $(echo "$out" | grep -E 'survey vs|MESH SIM' | tr '\n' ' ')"
    else
      echo "::error::mesh simulation failed [$args]"; echo "$out" | grep -E "FAIL"; fail=1
    fi
  done
else
  echo "::error::mesh simulation does not compile"; fail=1
fi

# ── 6. the probe <-> anchor link, simulated ────────────────────
if "$CXX" -std=c++17 -O2 -w -I tools/hostshim -o "$TMP/link_sim" tools/link_sim.cpp \
   && out=$("$TMP/link_sim" 2>&1); then
  echo "  ok   link sim: $(echo "$out" | head -1)"
else
  echo "::error::probe/anchor link simulation failed"; echo "${out:-}"; fail=1
fi

# ── 7a. the workflow itself ────────────────────────────────────
if out=$(python3 tools/lint_workflow.py 2>&1); then echo "  ok   $out"
else echo "$out"; fail=1; fi

# ── 7. the ESP8266Audio prune, against a mock library ──────────
if out=$(bash tools/test_prune.sh 2>&1); then echo "  ok   $out"
else echo "::error::ESP8266Audio prune test failed: $out"; fail=1; fi

[ $fail -eq 0 ] && echo "pre-flight: all firmware compile, receiver links, mesh and link agree"
exit $fail
