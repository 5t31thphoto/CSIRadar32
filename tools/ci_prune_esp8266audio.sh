#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
#  Make ESP8266Audio 1.9.7 build on arduino-esp32 3.x (IDF 5.x)
# ═══════════════════════════════════════════════════════════════
#
#  WHAT FAILS, VERBATIM FROM CI (m5stack core 3.3.9):
#
#    AudioOutputI2S.cpp:168   'esp_chip_info_t' was not declared
#    AudioOutputI2S.cpp:232   'I2S_MCLK_MULTIPLE_DEFAULT' was not declared
#    AudioOutputSPDIF.cpp:108 'I2S_MCLK_MULTIPLE_DEFAULT' was not declared
#    AudioOutputSPDIF.cpp:186 'rtc_clk_apll_enable' was not declared
#
#  Those are the library's OWN hardware output drivers, written against
#  the IDF 4 legacy I2S driver.  IDF 5 removed those symbols.  Arduino
#  compiles every .cpp in a library whether or not it is used, so an
#  unused driver breaks the build.
#
#  WE DO NOT USE THEM.  mantis_mp3.h decodes with AudioGeneratorMP3 and
#  plays through its own AudioOutput subclass into M5.Speaker.  So the
#  fix is to remove the drivers, not to patch them -- and removing them
#  is ALSO what keeps the legacy I2S driver out of the link.  IDF aborts
#  at boot ("CONFLICT! driver_ng is not allowed to be used with the
#  legacy driver") when the legacy driver and M5Unified's new one are
#  both linked, which is a silent boot loop, not a build error.
#
#  Removal is transitive: anything that references a removed class goes
#  too, until nothing does.  The files we DO need are then asserted
#  present, so a future library change fails HERE with a clear message.
# ═══════════════════════════════════════════════════════════════
set -euo pipefail

LIBROOT="$(arduino-cli config get directories.user 2>/dev/null || echo "$HOME/Arduino")/libraries"
SRC="$LIBROOT/ESP8266Audio/src"
if [ ! -d "$SRC" ]; then
  echo "::error::ESP8266Audio not installed at $SRC"
  exit 1
fi

removed=()
remove_stem() {           # remove foo.cpp / foo.h for a class stem
  local stem="$1"
  for f in "$SRC/$stem.cpp" "$SRC/$stem.h"; do
    if [ -f "$f" ]; then rm -f "$f"; removed+=("$(basename "$f")"); fi
  done
}

# The hardware drivers that cannot compile on IDF 5, and their
# direct subclasses.
for stem in AudioOutputI2S AudioOutputI2SNoDAC AudioOutputSPDIF AudioOutputULP; do
  remove_stem "$stem"
done

# Transitive closure: drop anything still naming a removed class.
changed=1
while [ "$changed" -eq 1 ]; do
  changed=0
  for f in "$SRC"/*.cpp "$SRC"/*.h; do
    [ -f "$f" ] || continue
    if grep -qE '\b(AudioOutputI2S|AudioOutputI2SNoDAC|AudioOutputSPDIF|AudioOutputULP)\b' "$f"; then
      rm -f "$f"; removed+=("$(basename "$f")"); changed=1
    fi
  done
done

echo "pruned from ESP8266Audio: ${removed[*]:-(nothing)}"

# What mantis_mp3.h actually uses must still be there.
for need in AudioOutput.h AudioGenerator.h AudioGeneratorMP3.h AudioGeneratorMP3.cpp \
            AudioFileSource.h AudioFileSourceSD.h AudioFileSourceID3.h AudioFileSourceID3.cpp; do
  if [ ! -f "$SRC/$need" ]; then
    echo "::error::ESP8266Audio no longer ships $need -- the MP3 path in mantis_mp3.h needs it"
    exit 1
  fi
done
echo "ESP8266Audio: MP3 decode path intact, legacy I2S drivers removed"
