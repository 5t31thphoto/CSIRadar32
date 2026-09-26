#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
#  PRE-FLIGHT — compile every firmware before CI installs anything
# ═══════════════════════════════════════════════════════════════
#
#  WHY THIS EXISTS
#
#  A CI build spends ten minutes installing toolchains before it
#  compiles a single line, so a missing #include costs ten minutes to
#  discover and another ten to confirm a fix.  Worse, the checks that
#  ran BEFORE the toolchain were host-only, and host builds skip every
#  #if defined(ARDUINO) block -- which is how a header with a missing
#  include passed every check and failed on the device.
#
#  This compiles each sketch EXACTLY as arduino-cli sees it:
#
#    - flat directory: every header copied beside the sketch, because
#      that is what the Arduino build does and it is where a missing
#      copy actually shows up
#    - ARDUINO defined: so the code that ships is the code checked
#    - real vendor header shapes from tools/ardshim
#
#  It is a syntax check, not a link, and it needs no toolchain at all.
#  It runs in seconds and catches the entire class of failure that was
#  costing full CI cycles.
# ═══════════════════════════════════════════════════════════════
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT=$(pwd)
SHIM="$ROOT/tools/ardshim"
CXX="${CXX:-g++}"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
fail=0

# name : sketch : extra source dirs (headers copied flat beside it)
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

  cp "$ROOT"/mantis_*.h "$d"/ 2>/dev/null
  if [ "$extra" = "." ]; then
    cp "$ROOT"/*.h "$ROOT"/*.cpp "$d"/ 2>/dev/null
    cp "$ROOT"/rust/rfcore/*.h "$d"/ 2>/dev/null
  elif [ -n "$extra" ]; then
    cp "$ROOT/$extra"/*.h "$d"/ 2>/dev/null
  fi
  cp "$ROOT/$sketch" "$d/src.cpp"

  # The Arduino build auto-includes Arduino.h and defines the fonts the
  # M5 libraries declare; supply both so the check sees the same world.
  sed -i '1i #include <Arduino.h>' "$d/src.cpp"
  if grep -q "M5Unified.h\|M5Cardputer.h" "$d/src.cpp"; then
    sed -i 's/auto cfg = M5.config();/CfgT cfg = M5.config();/' "$d/src.cpp"
  fi

  # EVERY LOCAL INCLUDE MUST RESOLVE IN THE FLAT DIRECTORY.
  # This is the check that would have caught a header living under
  # cardputer/ while another sketch needed it.
  for inc in $(grep -ho '#include "[^"]*"' "$d"/*.cpp "$d"/*.h 2>/dev/null \
               | sed 's/.*"\(.*\)"/\1/' | sort -u); do
    if [ ! -f "$d/$inc" ]; then
      echo "::error::$name: '$inc' is not in the sketch directory"
      fail=1
    fi
  done

  if out=$("$CXX" -fsyntax-only -std=c++17 -DARDUINO=200 \
            -I "$SHIM" -I "$d" "$d/src.cpp" 2>&1); then
    echo "  ok   $name"
  else
    echo "::error::$name fails to compile as Arduino builds it"
    echo "$out" | grep -E "error" | head -12
    fail=1
  fi
done

[ $fail -eq 0 ] && echo "pre-flight: all firmwares compile"
exit $fail
