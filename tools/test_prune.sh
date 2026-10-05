#!/usr/bin/env bash
# Regression test for ci_prune_esp8266audio.sh against a mock of the
# ESP8266Audio 1.9.7 layout: the MP3 closure must survive, everything that
# has broken a CI build must not.
set -euo pipefail
cd "$(dirname "$0")/.."
M=$(mktemp -d)/src; trap 'rm -rf "$(dirname "$M")"' EXIT
w(){ mkdir -p "$(dirname "$M/$1")"; printf '%b' "$2" > "$M/$1"; }
w AudioStatus.h '#pragma once\n'
w AudioLogger.h '#pragma once\n'
w AudioLogger.cpp '#include "AudioLogger.h"\n'
w AudioOutput.h '#include "AudioStatus.h"\n#include "AudioLogger.h"\n'
w AudioFileSource.h '#include "AudioStatus.h"\n'
w AudioFileSourceFS.h '#include "AudioFileSource.h"\n#include <FS.h>\n'
w AudioFileSourceFS.cpp '#include "AudioFileSourceFS.h"\n'
w AudioFileSourceSD.h '#include "AudioFileSourceFS.h"\n#include <SD.h>\n'
w AudioFileSourceID3.h '#include "AudioFileSource.h"\n'
w AudioFileSourceID3.cpp '#include "AudioFileSourceID3.h"\n'
w AudioGenerator.h '#include "AudioOutput.h"\n#include "AudioFileSource.h"\n'
w AudioGeneratorMP3.h '#include "AudioGenerator.h"\n#include "libmad/mad.h"\n'
w AudioGeneratorMP3.cpp '#include "AudioGeneratorMP3.h"\n'
w libmad/mad.h '#include "global.h"\n'
w libmad/global.h '#pragma once\n'
w libmad/layer3.c '#include "mad.h"\n'
w AudioFileSourceHTTPStream.h '#include <HTTPClient.h>\nWiFiClient client;\n'
w AudioFileSourceHTTPStream.cpp '#include "AudioFileSourceHTTPStream.h"\n'
w AudioFileSourceICYStream.cpp '#include "AudioFileSourceHTTPStream.h"\n'
w AudioOutputI2S.cpp 'i2s_driver_install();\n'
w AudioOutputSPDIF.cpp '\n'
w AudioGeneratorAAC.cpp '#include "libhelix-aac/aacdec.h"\n'
w libhelix-aac/aacdec.h '\n'
bash tools/ci_prune_esp8266audio.sh "$M" >/dev/null
for need in AudioOutput.h AudioFileSourceSD.h AudioFileSourceFS.cpp AudioFileSourceID3.cpp \
            AudioGeneratorMP3.cpp AudioLogger.cpp libmad/layer3.c libmad/global.h; do
  [ -f "$M/$need" ] || { echo "prune removed $need, which the MP3 path needs"; exit 1; }
done
for gone in AudioFileSourceHTTPStream.cpp AudioFileSourceICYStream.cpp AudioOutputI2S.cpp \
            AudioOutputSPDIF.cpp AudioGeneratorAAC.cpp libhelix-aac; do
  [ ! -e "$M/$gone" ] || { echo "prune kept $gone"; exit 1; }
done
echo "prune keeps the MP3 closure and drops HTTP/ICY/I2S/SPDIF/AAC"
