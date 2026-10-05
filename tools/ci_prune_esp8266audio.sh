#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
#  Trim ESP8266Audio 1.9.7 to exactly what the probes use
# ═══════════════════════════════════════════════════════════════
#
#  Arduino compiles EVERY source file in a library, used or not.
#  ESP8266Audio ships a dozen decoders, sources and outputs, and on
#  arduino-esp32 3.x several no longer compile.  CI met them one by one:
#
#    run 1  AudioOutputI2S / AudioOutputSPDIF        IDF 4 legacy I2S
#    run 2  AudioFileSourceHTTPStream / ICYStream    'WiFiClient' undeclared
#
#  Chasing them one CI run at a time is the wrong game.  This is an
#  ALLOW-LIST: start from the four headers mantis_mp3.h includes, follow
#  #include transitively inside the library, keep that closure, delete the
#  rest.  A file we never use can no longer break this build, and the
#  legacy I2S driver -- which aborts at boot next to M5Unified's new one --
#  can never be linked in.
#
#  ROOTS is the only thing to update if mantis_mp3.h ever includes another
#  ESP8266Audio header.
#
#  Usage: ci_prune_esp8266audio.sh [library_src_dir]
# ═══════════════════════════════════════════════════════════════
# NEVER fails the build: any surprise prints a warning and leaves the
# library untouched, exactly as installed.
set -uo pipefail

if [ $# -ge 1 ]; then
  SRC="$1"
else
  LIBROOT="$(arduino-cli config get directories.user 2>/dev/null || echo "$HOME/Arduino")/libraries"
  SRC="$LIBROOT/ESP8266Audio/src"
fi
[ -d "$SRC" ] || { echo "::warning::ESP8266Audio src not found at $SRC; nothing pruned"; exit 0; }

python3 - "$SRC" <<'PY'
import os, re, sys, shutil

src = os.path.abspath(sys.argv[1])
ROOTS = ["AudioOutput.h", "AudioFileSourceSD.h", "AudioFileSourceID3.h", "AudioGeneratorMP3.h"]
INC = re.compile(r'^\s*#\s*include\s*["<]([^">]+)[">]', re.M)

def resolve(name, from_dir):
    for base in (from_dir, src):
        p = os.path.normpath(os.path.join(base, name))
        if (p == src or p.startswith(src + os.sep)) and os.path.isfile(p):
            return p
    return None

keep_files, keep_dirs, todo = set(), set(), []
for r in ROOTS:
    p = resolve(r, src)
    if not p:
        print(f"::warning::ESP8266Audio no longer ships {r}; leaving the library as installed"); sys.exit(0)
    todo.append(p)

while todo:
    f = todo.pop()
    if f in keep_files: continue
    keep_files.add(f)
    d = os.path.dirname(f)
    if d != src:
        # A decoder's own directory (libmad/...) is kept WHOLE: it is
        # self-contained and its files include each other freely.
        top = os.path.join(src, os.path.relpath(d, src).split(os.sep)[0])
        if top not in keep_dirs:
            keep_dirs.add(top)
            for dp, _, fns in os.walk(top):
                todo.extend(os.path.join(dp, fn) for fn in fns)
    stem, ext = os.path.splitext(f)
    if ext in (".h", ".hpp"):                 # a header brings its implementation
        for e in (".cpp", ".c"):
            if os.path.isfile(stem + e): todo.append(stem + e)
    try:
        text = open(f, encoding="utf-8", errors="ignore").read()
    except OSError:
        continue
    for m in INC.finditer(text):
        p = resolve(m.group(1), d)
        if p: todo.append(p)

removed = []
for entry in sorted(os.listdir(src)):
    p = os.path.join(src, entry)
    if os.path.isdir(p):
        if p not in keep_dirs: shutil.rmtree(p); removed.append(entry + "/")
    elif p not in keep_files:
        os.remove(p); removed.append(entry)

top_kept = sorted(os.path.relpath(f, src) for f in keep_files if os.path.dirname(f) == src)
print("ESP8266Audio kept     :", " ".join(top_kept))
print("ESP8266Audio kept dirs:", " ".join(os.path.relpath(x, src) + "/" for x in sorted(keep_dirs)) or "(none)")
print("ESP8266Audio removed  :", " ".join(removed) or "(nothing)")

bad = re.compile(r'\b(WiFiClient|HTTPClient|AudioOutputI2S|AudioOutputSPDIF|i2s_driver_install)\b')
for f in keep_files:
    if bad.search(open(f, encoding="utf-8", errors="ignore").read()):
        print(f"::warning::kept {os.path.relpath(f, src)} still references a removed dependency")
PY
exit 0
