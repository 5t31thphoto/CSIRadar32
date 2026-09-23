# MantisSec — Cardputer-Adv Probe

A full receiver that stays in your hand. Never an anchor.

## Which binary do I want?

**`mantis-cardputer-adv-merged.bin`** — use this one unless you have a
reason not to. It works with:

- the web flasher at <https://5t31thphoto.github.io/CSIRadar32/flash.html>
- `esptool` written to flash offset `0x0`
- **M5Launcher**, which parses the merged image and extracts the
  application from it

**`mantis-cardputer-adv-app.bin`** — the bare application image. M5Launcher
accepts this form too, and it is a smaller file to keep on an SD card. It
**cannot** be flashed on its own over USB: it contains no bootloader and no
partition table.

### Why the merged image works with the launcher

This surprised me too, so it is worth stating plainly: a merged factory
image is a valid M5Launcher input. The launcher reads the partition table
inside it and installs just the app. The property that matters is that the
merged image is **truncated after the application payload** rather than
padded out to the full flash size — `esptool merge_bin` does not pad unless
you ask it to, and CI asserts the result stays under 4 MB so a future
change cannot quietly start shipping an 8 MB file.

## Installing via M5Launcher

1. Copy `mantis-cardputer-adv-merged.bin` to the SD card.
2. Boot into M5Launcher.
3. Pick the file from the SD list and install.

## SD card layout

The firmware creates these on first run:

```
/mantis/alarms/     your own MP3s for each alarm (optional)
/mantis/sessions/   recorded sessions, for replay
/mantis/logs/
/mantis/config/
```

**MP3.** Decoded with ESP8266Audio bridged onto `m5::Speaker_Class`
(the ES8311 codec), the same chain the working Cardputer-Adv MP3 players
use. Drop your own clips in `/mantis/alarms` — existing MP3s from other
firmware work as-is.

Decoding runs on **its own FreeRTOS task**, pinned to core 1. Sounding an
alarm never stalls the sensing loop: a device that briefly stops watching
the room in order to announce something is exactly backwards.

Alarm files are optional. Every alarm has a compiled-in tone fallback, so a
missing card or a missing file still makes a noise — it just makes a
plainer one, and the alarm screen shows `tone` instead of `SD`.

| alarm | file | fallback |
|---|---|---|
| perimeter crossed | `perimeter.mp3` | 1760 Hz x3 |
| new contact | `presence.mp3` | 1320 Hz x2 |
| motion | `motion.mp3` | 880 Hz (off by default) |
| tripwire | `tripwire.mp3` | 2093 Hz x4 |
| mesh fault | `meshfault.mp3` | 440 Hz x2 |

General motion is **off by default**: in a busy room it fires constantly and
trains you to ignore the speaker, taking the alarms that matter with it.

## A note on the SD card

`SD.begin()` with no arguments does **not** work on this board — it uses the
default VSPI pins, which are not the Cardputer's, so it returns `false` and
every SD feature dies with nothing on screen to explain why. The bus must be
started explicitly first:

```cpp
SPI.begin(40, 39, 14, 12);          // SCK, MISO, MOSI, CS
SD.begin(12, SPI, 25000000);
```

Those are the pins from the M5Stack microSD documentation. This is the one
place in the firmware where pin numbers are hardcoded, and it is commented
as such.

## Build requirements

From the M5Stack documentation for this board:

- board manager >= 3.2.3, board = `M5Cardputer`
- M5Cardputer >= 1.1.1, M5Unified >= 0.2.10, M5GFX >= 0.2.10
- **ESP8266Audio == 1.9.7** (that exact version; later releases change the
  `AudioOutput` interface and produce no sound)

## Download mode

Set the side power switch to **OFF**, hold **G0**, apply power, release.
