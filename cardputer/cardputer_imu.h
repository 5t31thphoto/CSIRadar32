#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CARDPUTER IMU
//  A second, independent witness to the operator's own motion
// ═══════════════════════════════════════════════════════════════
//
//  The calibration walk has always had a blind spot: it asks the
//  operator to turn in place and to walk between landmarks, then
//  ESTIMATES from RF how much they turned and how far they went.  The
//  estimate is checked against nothing, so an operator who turns 250
//  degrees instead of 360, or paces out four metres instead of six,
//  produces a confidently wrong model.
//
//  The BMI270 measures both directly, and its errors have nothing in
//  common with RF's errors.  That independence is the whole value: two
//  wrong-in-the-same-way estimates are worth little, two wrong-in-
//  different-ways estimates can check each other.
//
//    rotation  gyro integration.  Drifts slowly, is exact over the
//              seconds a turn takes.  RF bearing is noisy instantly but
//              does not drift.  Opposite failure modes.
//
//    steps     accelerometer peaks.  A step count times an assumed
//              stride is a crude distance -- but crude in a completely
//              different way from RF path-loss, so agreement is real
//              evidence and disagreement is a warning worth showing
//              BEFORE the model is built on it.
//
//  ── WHAT THIS DELIBERATELY DOES NOT DO ───────────────────────
//
//  No dead reckoning.  Integrating accelerometer twice to get position
//  diverges in seconds on a consumer MEMS part, and a position that
//  looks plausible for ten seconds and is metres wrong at thirty is far
//  more dangerous than no position at all.  Heading and step count are
//  the two quantities this sensor can actually sustain.
// ═══════════════════════════════════════════════════════════════
#include "cardputer_platform.h"   // CP_STEP_* thresholds, stride default
#include <stdint.h>
#include <math.h>

typedef struct {
    // Heading, integrated from the gyro's vertical axis.
    float    heading_rad;      // 0 = where begin() was called
    float    gyro_bias_dps;    // learned while stationary
    uint32_t bias_n;
    bool     bias_ready;

    // Step detection.
    uint32_t steps;
    uint32_t last_step_ms;
    float    grav_lp;          // SLOW baseline: gravity plus posture
    float    dev_lp;           // FAST: deviation from that baseline
    bool     armed;            // above threshold, waiting for the fall

    // Stillness, which is what makes bias learning safe.
    bool     is_still;
    uint32_t still_since_ms;

    float    stride_m;
    uint32_t last_update_ms;
} CardputerImu;

static inline void cp_imu_begin(CardputerImu *m, float stride_m) {
    *m = CardputerImu{};
    m->stride_m = stride_m;
    m->grav_lp = 1.0f;         // resting at 1 g
}

// Feed one IMU sample.
//
// `gz_dps` is rotation about the screen-normal axis -- the one that
// changes when the operator turns on the spot.  `ax/ay/az` are in g.
static inline void cp_imu_update(CardputerImu *m,
                                 float ax, float ay, float az,
                                 float gz_dps, uint32_t now_ms) {
    const uint32_t dt_ms = (m->last_update_ms == 0) ? 0 : (now_ms - m->last_update_ms);
    m->last_update_ms = now_ms;
    if (dt_ms == 0 || dt_ms > 500) return;      // first sample, or a stall
    const float dt = (float)dt_ms / 1000.0f;

    // ── stillness ──
    const float mag = sqrtf(ax * ax + ay * ay + az * az);
    // TWO time constants, not one.
    //
    // A single low-pass cannot do both jobs: slow enough to track
    // gravity and posture is far too slow to survive a footfall, and my
    // first attempt at alpha 0.15 attenuated a 1.35 g peak to 1.09 so
    // the threshold was never crossed and NO steps were counted at all.
    //
    // The slow filter tracks the baseline (gravity, and how the device
    // is being held).  The fast one tracks deviation FROM it, which is
    // what a step actually is.  Standard pedometer practice, and the
    // reason it is standard.
    m->grav_lp += 0.02f * (mag - m->grav_lp);
    const float dev = mag - m->grav_lp;
    m->dev_lp += 0.45f * (dev - m->dev_lp);
    const float wobble = fabsf(mag - 1.0f);
    const bool still_now = (wobble < 0.06f) && (fabsf(gz_dps) < 3.0f);
    if (still_now) { if (!m->is_still) m->still_since_ms = now_ms; m->is_still = true; }
    else             m->is_still = false;

    // ── gyro bias, learned ONLY while genuinely still ──
    //
    // A MEMS gyro has a bias of a few degrees per second that changes
    // with temperature.  Integrated over a 30-second walk that is tens
    // of degrees of pure error, which would make the measured heading
    // worse than the RF estimate it is supposed to check.
    //
    // Learning it requires knowing the device is stationary, and the
    // accelerometer says so independently -- which is exactly why a
    // 6-axis part can self-calibrate and a bare gyro cannot.
    if (m->is_still && (now_ms - m->still_since_ms) > 400) {
        const uint32_t n = m->bias_n;
        m->gyro_bias_dps = (n == 0) ? gz_dps
                                    : (m->gyro_bias_dps + (gz_dps - m->gyro_bias_dps) / (float)(n + 1));
        if (m->bias_n < 5000) m->bias_n++;
        if (m->bias_n > 60) m->bias_ready = true;
    }

    // ── heading ──
    // Deadband below the residual bias noise: integrating sub-threshold
    // noise is how a stationary device slowly acquires an imaginary turn.
    float rate = gz_dps - (m->bias_ready ? m->gyro_bias_dps : 0.0f);
    if (fabsf(rate) < 0.8f) rate = 0.0f;
    m->heading_rad += rate * (float)M_PI / 180.0f * dt;
    while (m->heading_rad >  (float)M_PI) m->heading_rad -= 2.0f * (float)M_PI;
    while (m->heading_rad < -(float)M_PI) m->heading_rad += 2.0f * (float)M_PI;

    // ── steps ──
    // Peak-and-fall on the low-passed magnitude, with a refractory
    // period.  Counting every threshold crossing would turn one step
    // into three, because a footfall rings.
    // Hysteresis on the DEVIATION, with a refractory period.  A
    // footfall rings, so a bare threshold crossing turns one step into
    // three; requiring a rise then a fall then a quiet interval counts
    // it once.
    const float rise = CP_STEP_ACCEL_THRESH - 1.0f;   // threshold above baseline
    if (!m->armed && m->dev_lp > rise) {
        m->armed = true;
    } else if (m->armed && m->dev_lp < rise * 0.35f) {
        m->armed = false;
        if (now_ms - m->last_step_ms >= CP_STEP_MIN_INTERVAL_MS) {
            m->steps++;
            m->last_step_ms = now_ms;
        }
    }
}

// Total turn since a mark, unwrapped, so a 360-degree turn reads as 360
// rather than wrapping to zero.
typedef struct {
    float    start_heading;
    float    accum_rad;
    float    last_heading;
    uint32_t start_steps;
    bool     active;
} CardputerTurn;

static inline void cp_turn_begin(CardputerTurn *t, const CardputerImu *m) {
    t->start_heading = m->heading_rad;
    t->last_heading  = m->heading_rad;
    t->accum_rad     = 0.0f;
    t->start_steps   = m->steps;
    t->active        = true;
}

static inline void cp_turn_update(CardputerTurn *t, const CardputerImu *m) {
    if (!t->active) return;
    float d = m->heading_rad - t->last_heading;
    // Unwrap: crossing pi is a small step, not a full revolution.
    while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
    while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
    t->accum_rad    += d;
    t->last_heading  = m->heading_rad;
}

static inline float cp_turn_degrees(const CardputerTurn *t) {
    return t->accum_rad * 180.0f / (float)M_PI;
}

// Distance walked since the mark, from steps.
static inline float cp_turn_distance_m(const CardputerTurn *t, const CardputerImu *m) {
    return (float)(m->steps - t->start_steps) * m->stride_m;
}

// ── Cross-check against the RF estimate ───────────────────────
//
// Returns a disagreement ratio: 0 means the two agree, 1 means they
// differ by as much as the measurement itself.
//
// This is REPORTED, never used to silently correct the RF.  The IMU is a
// witness, not an authority: a device held loosely, set down mid-walk,
// or swung while turning gives a heading the operator's BODY did not
// follow.  The right response to disagreement is to tell the operator,
// because only they know which one lied.
static inline float cp_turn_disagreement(float imu_deg, float rf_deg) {
    const float denom = fabsf(imu_deg) > 20.0f ? fabsf(imu_deg) : 20.0f;
    float d = imu_deg - rf_deg;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    const float r = fabsf(d) / denom;
    return r > 1.0f ? 1.0f : r;
}
