#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS PROBE CONFIG
//  Constants shared by every hand-held receiver
// ═══════════════════════════════════════════════════════════════
//
//  The IMU, alarm and MP3 layers are used by BOTH probes, and they each
//  pulled these from cardputer_platform.h -- which meant three "shared"
//  headers each depended on one platform's header.  The Core2 build
//  failed on exactly that.
//
//  Anything both probes need lives here.  Anything one of them needs
//  stays in its own platform header.
//
//  Every value is overridable, so a device with a genuinely different
//  requirement can say so at its own build without editing shared code.
// ═══════════════════════════════════════════════════════════════
#include <stdint.h>

// ── Step detection ────────────────────────────────────────────
// Peak above the gravity baseline that counts as a footfall, and the
// refractory period that stops one step ringing into three.
#ifndef CP_STEP_ACCEL_THRESH
  #define CP_STEP_ACCEL_THRESH   1.25f
#endif
#ifndef CP_STEP_MIN_INTERVAL_MS
  #define CP_STEP_MIN_INTERVAL_MS 250
#endif
// Adult average stride.  User-adjustable; only ever a cross-check on
// the RF distance, never an authority over it.
#ifndef CP_STRIDE_M_DEFAULT
  #define CP_STRIDE_M_DEFAULT    0.72f
#endif

// ── Alarm files ───────────────────────────────────────────────
// Same path on both devices, so one SD card works in either.
#ifndef CP_ALARM_DIR
  #define CP_ALARM_DIR "/mantis/alarms"
#endif

// ── SD layout ─────────────────────────────────────────────────
#ifndef CP_DIR_ROOT
  #define CP_DIR_ROOT     "/mantis"
  #define CP_DIR_ALARMS   "/mantis/alarms"
  #define CP_DIR_SESSIONS "/mantis/sessions"
  #define CP_DIR_LOGS     "/mantis/logs"
  #define CP_DIR_CONFIG   "/mantis/config"
#endif
