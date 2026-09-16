#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CARDPUTER MP3
//  ESP8266Audio -> m5::Speaker_Class (ES8311)
// ═══════════════════════════════════════════════════════════════
//
//  Chain, matching the M5Unified MP3_with_ESP8266Audio example and the
//  Cardputer-Adv MP3 players built on it:
//
//      SD -> AudioFileSourceSD -> AudioFileSourceID3
//         -> AudioGeneratorMP3 -> AudioOutputM5Speaker
//         -> M5Cardputer.Speaker (ES8311)
//
//  ── ESP8266Audio MUST BE 1.9.7 ───────────────────────────────
//
//  Not "1.9.7 or later".  The working Cardputer-Adv MP3 players pin this
//  exact version, and CI pins it too.  Taking latest is how a build that
//  worked last month stops producing sound with no source change to
//  explain it.
//
//  ── DECODING RUNS IN ITS OWN TASK ────────────────────────────
//
//  This is the part that matters for THIS firmware specifically.
//
//  An MP3 decoder needs to be fed steadily.  Our loop() runs CSI
//  processing, the mesh solve, the tomography and a full screen redraw --
//  if the decoder shared that loop, every heavy frame would starve it and
//  the alarm would stutter.  Worse, the inverse is also true: a blocking
//  decode would stall the sensing loop, so sounding an alarm would make
//  the device briefly stop watching the room.  That is precisely
//  backwards.
//
//  So the decoder owns a task pinned to the core the radio does NOT use,
//  and the loop only ever posts a request to it.  M5Unified's own example
//  arrived at the same structure for the same reason, and it notes three
//  buffers in rotation so the one being refilled is never the one the
//  speaker task is reading.
//
//  ── AN ALARM STILL CANNOT BE SILENT ──────────────────────────
//
//  Everything here is best-effort on top of the tone fallback in
//  cardputer_audio.h.  Missing file, unreadable card, decoder failure,
//  task busy with an earlier alarm -- every path ends in a tone rather
//  than in silence.
// ═══════════════════════════════════════════════════════════════
#include "cardputer_platform.h"
#include "cardputer_audio.h"

// Virtual speaker channel.  m5::Speaker_Class mixes 0-7 internally, so
// keeping MP3 on its own channel means a tone can still be emitted for a
// DIFFERENT alarm while a clip is playing, instead of one cutting the
// other off.
//
// Declared OUTSIDE the Arduino guard: the sketch uses it on the tone
// path too, and burying it in the hardware branch made the host build
// fail on a symbol that has nothing to do with hardware.
static constexpr uint8_t CP_MP3_CHANNEL = 0;

#if defined(ARDUINO)
  #include <M5Cardputer.h>
  #include <SD.h>
  #include <AudioOutput.h>
  #include <AudioFileSourceSD.h>
  #include <AudioFileSourceID3.h>
  #include <AudioGeneratorMP3.h>
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <freertos/queue.h>

// Bridge ESP8266Audio's sample sink onto the M5 speaker.
//
// Structure follows M5Unified's AudioOutputM5Speaker: accumulate samples
// into a buffer, hand the full buffer to the speaker, rotate to the next
// one.  Three buffers rather than two, so there is a buffer of slack
// between the decoder and the speaker task -- with two, any hiccup in
// either one is immediately audible.
class AudioOutputM5Speaker : public AudioOutput {
public:
    AudioOutputM5Speaker(m5::Speaker_Class *spk, uint8_t ch = 0)
        : _spk(spk), _ch(ch) {}
    virtual ~AudioOutputM5Speaker() {}

    bool begin() override { return true; }

    bool ConsumeSample(int16_t sample[2]) override {
        _buf[_idx][_pos++] = sample[0];
        _buf[_idx][_pos++] = sample[1];
        if (_pos >= kBufSamples) {
            flush();
        }
        return true;
    }

    void flush() override {
        if (_pos == 0) return;
        _spk->playRaw(_buf[_idx], _pos, hertz, true /* stereo */, 1, _ch);
        _idx = (uint8_t)((_idx + 1) % kBufCount);
        _pos = 0;
    }

    bool stop() override {
        flush();
        _spk->stop(_ch);
        return true;
    }

private:
    static constexpr size_t kBufCount   = 3;
    static constexpr size_t kBufSamples = 512;   // interleaved int16
    m5::Speaker_Class *_spk;
    uint8_t  _ch;
    int16_t  _buf[kBufCount][kBufSamples];
    uint8_t  _idx = 0;
    size_t   _pos = 0;
};

// ── The decoder task ──────────────────────────────────────────
struct CpMp3Request { char path[96]; uint8_t volume; };

static QueueHandle_t      s_mp3_q   = nullptr;
static volatile bool      s_mp3_busy = false;

static void cp_mp3_task(void *) {
    CpMp3Request req;
    for (;;) {
        if (xQueueReceive(s_mp3_q, &req, portMAX_DELAY) != pdTRUE) continue;
        s_mp3_busy = true;

        AudioFileSourceSD   *file = new AudioFileSourceSD(req.path);
        AudioFileSourceID3  *id3  = nullptr;
        AudioGeneratorMP3   *mp3  = nullptr;
        AudioOutputM5Speaker *out = nullptr;

        if (file && file->isOpen()) {
            // ID3 wrapper: alarm clips very often carry tags, and feeding
            // tag bytes to the decoder as if they were audio produces a
            // burst of noise before the sound starts.
            id3 = new AudioFileSourceID3(file);
            out = new AudioOutputM5Speaker(&M5Cardputer.Speaker, CP_MP3_CHANNEL);
            mp3 = new AudioGeneratorMP3();
            M5Cardputer.Speaker.setVolume(req.volume);
            if (mp3->begin(id3, out)) {
                while (mp3->isRunning()) {
                    if (!mp3->loop()) { mp3->stop(); break; }
                    // Yield so the task cannot monopolise its core; the
                    // decoder is not the most important thing running.
                    vTaskDelay(1);
                }
            }
        }

        if (mp3)  { if (mp3->isRunning()) mp3->stop(); delete mp3; }
        if (out)  { out->stop(); delete out; }
        if (id3)  delete id3;
        if (file) delete file;

        s_mp3_busy = false;
    }
}

// Bring the decoder up.  Returns false if the task could not start, in
// which case every alarm silently degrades to tones -- which is the
// whole point of having them.
static inline bool cp_mp3_begin() {
    if (s_mp3_q) return true;
    s_mp3_q = xQueueCreate(2, sizeof(CpMp3Request));
    if (!s_mp3_q) return false;
    // Core 0 runs the Wi-Fi driver and our CSI callback.  The decoder
    // goes on core 1 with the rest of the application so it cannot add
    // latency to the radio path.
    //
    // 8 KB of stack: libhelix's MP3 frame decode is stack-hungry and a
    // too-small stack here shows up as a reboot mid-alarm rather than as
    // an error anyone can read.
    const BaseType_t ok = xTaskCreatePinnedToCore(
        cp_mp3_task, "mantis_mp3", 8192, nullptr, 2, nullptr, 1);
    return ok == pdPASS;
}

// Queue a clip.  NON-BLOCKING by contract: this returns immediately and
// the caller keeps sensing.
//
// Returns false when the clip could not be queued, and the caller must
// then fall back to a tone.  A full queue means an alarm is already
// sounding, and stacking a second on top of it would produce a mess
// rather than more information.
static inline bool cp_mp3_play(const char *path, uint8_t volume) {
    if (!s_mp3_q) return false;
    CpMp3Request r{};
    snprintf(r.path, sizeof(r.path), "%s", path);
    r.volume = volume;
    return xQueueSend(s_mp3_q, &r, 0) == pdTRUE;
}

static inline bool cp_mp3_busy() { return s_mp3_busy; }

#else   // host build: the logic above is hardware-bound, so stub it out
static inline bool cp_mp3_begin() { return false; }
static inline bool cp_mp3_play(const char *, uint8_t) { return false; }
static inline bool cp_mp3_busy() { return false; }
#endif
