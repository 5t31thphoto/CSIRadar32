// ═══════════════════════════════════════════════════════════════
//  MantisSec CSI-Radar-S3 — config.h  (v0.3)
//
//  Shared vocabulary: constants, enums, and POD types every module
//  depends on.  Keep this header ZERO-DEPENDENCY beyond Arduino.h so
//  every other header can include it without cycles.
//
//  Design principles baked in:
//    - Positions are RELATIVE (normalized geometry-frame coords),
//      never metric.  Beacon triangle spans a unit region.
//    - Measurements are RELATIVE to the empty-room baseline captured
//      during calibration.  No absolute physics constants appear.
//    - The kernel + occupancy field ARE the model.  See scene.h.
// ═══════════════════════════════════════════════════════════════
#pragma once
#include <Arduino.h>
#include <stdint.h>

// ═══════════════════════════════════════════════════════════════
//  SERIAL LOGGING — OFF
// ═══════════════════════════════════════════════════════════════
// This project does not use a serial console.  Leaving it compiled in
// costs three ways:
//
//   1. CONTENTION.  U0TXD/U0RXD are GPIO43/44 -- the same pins as the
//      4-pin JST the wired peer link uses.  A console on UART0 fights
//      the peer link for the pins, and the build flag that moves it to
//      USB-CDC becomes load-bearing for something we do not want.
//
//   2. FLASH.  Every printf format string is stored, and pulling in
//      the float-formatting path (%f / %.2f) drags a large chunk of
//      newlib into the image.
//
//   3. CPU.  Several of these sit in the cal and scene paths and format
//      floats, which is expensive on a chip with no hardware divider
//      for the conversion.
//
// Set to 1 only for bench debugging, and only with the console on
// USB-CDC (CDCOnBoot=cdc) so it cannot touch the peer-link pins.
#define MS_SERIAL_LOG   0

// ── In-RAM log ring ───────────────────────────────────────────
// With serial off, log lines go to a small circular buffer that the
// Debug view (Settings -> Debug log) renders.  This is also the source
// any future sink reads from -- a Wi-Fi dead-drop serving the lines as
// a text file needs no changes here, only a new reader.  Note there is
// no filesystem in this firmware, so such a file is generated from this
// buffer on demand, never stored.
//
// 32 x 80 = 2.5 KB of the ~37 KB headroom.  Lines are truncated rather
// than wrapped: these are diagnostics, not prose.
#define MS_LOG_RING       1
#define MS_LOG_LINES      32
#define MS_LOG_LINE_LEN   80

#if MS_LOG_RING
void ms_log_printf(const char *fmt, ...);
int  ms_log_count();                 // lines currently held
const char *ms_log_line(int idx);    // 0 = oldest held line
void ms_log_clear();
#endif

#if MS_SERIAL_LOG
  #define MSLOG(...)    Serial.printf(__VA_ARGS__)
  #define MSLOGLN(x)    Serial.println(x)
  #define MSLOG_BEGIN() Serial.begin(115200)
#elif MS_LOG_RING
  #define MSLOG(...)    ms_log_printf(__VA_ARGS__)
  #define MSLOGLN(x)    ms_log_printf("%s", (x))
  #define MSLOG_BEGIN() ((void)0)
#else
  #define MSLOG(...)    ((void)0)
  #define MSLOGLN(x)    ((void)0)
  #define MSLOG_BEGIN() ((void)0)
#endif

// ═══════════════════════════════════════════════════════════════
//  v0.9 SUBSYSTEM FLAGS
// ═══════════════════════════════════════════════════════════════
// All ON.  These exist so a single subsystem can be isolated on
// hardware if it misbehaves -- not as a way to ship with features
// switched off.
#define MS_BEACON_CONTROL   1   // command beacon rate after discovery
#define MS_CSI_SEQLOCK      1   // callback publishes raw I/Q, core 1 decodes
#define MS_PEER_WIRE        1   // wired UART peer transport

// ── Stereo tripwire topology ──────────────────────────────────
// With one beacon and two receivers there are two sensible wirings, and
// they are genuinely different instruments:
//
//   TW_REMOTE (default) — the tripwire is the beacon <-> ANCHOR link.
//     The anchor is the one that stays put, so it is the one whose link
//     geometry is fixed and meaningful.  The PROBE is a remote display
//     and remote control for it: it shows the ANCHOR's alert, not a
//     second one of its own.  Walking around with the probe does not
//     create phantom trips.
//
//   TW_DUAL — two independent tripwires watching the same beacon.  Each
//     receiver reports its own link.  Useful for covering two lines at
//     once, or for comparing a fixed link against a hand-held one.
//
// Previously only the second behaviour existed, implicitly: each unit
// evaluated its own beacon link and the probe showed its own trip,
// which is wrong when the probe is in your hand and moving.
enum TripwireMode : uint8_t {
    TW_REMOTE = 0,   // probe mirrors the anchor's link
    TW_DUAL   = 1,   // both receivers run their own
};

// Anchor -> probe tripwire state, sent only in TW_REMOTE.
#define PEER_TRIPWIRE_MAGIC     0xC5B0741FUL
struct PeerTripwirePacket {
    uint32_t magic;
    uint8_t  status;        // LinkStatus of the anchor's strongest link
    uint8_t  beacon_id;
    uint16_t metric_pct;    // 0..100, for the bar
    uint32_t stamp_ms;
};
#define PEER_TRIPWIRE_STALE_MS  1200

// ── PSRAM probe result ────────────────────────────────────────
// PSRAM is enabled in the build but deliberately UNUSED (the canvas
// stays in internal DRAM).  This is a safe experiment: if it initializes
// we know 8 MB is available for future work; if it does not, the reason
// matters more than the fact, so the result is classified rather than
// reduced to a yes/no.
enum PsramStatus : uint8_t {
    PSRAM_OK = 0,           // initialized, size known
    PSRAM_NOT_COMPILED,     // fqbn missing PSRAM=opi -> never even tried
    PSRAM_NO_RESPONSE,      // compiled in, chip did not answer
};
struct PsramProbe {
    PsramStatus status;
    uint32_t    size_bytes;
    const char *hint;       // short human-readable reason
};
const PsramProbe &psram_probe();   // evaluated once at boot

// ── Version & identity ─────────────────────────────────────────
#define FW_NAME     "MantisSec"
#define FW_VERSION  "0.9.0"

// ── Wi-Fi / CSI (unchanged from v0.2) ─────────────────────────
#define CSI_CHANNEL             11
#define CSI_NUM_SUBCARRIERS     64
#define CSI_CENTER_FREQ_HZ      2462000000.0f
#define CSI_WAVELENGTH_M        0.1218f
#define CSI_SUBCARRIER_SPACING_HZ 312500.0f
static const uint8_t BEACON_MAC_PREFIX[5] = {0x1A, 0x00, 0x00, 0x00, 0x00};
// v0.8: raised 4 → 6.  Beacon slots are allocated dynamically by
// slot_for_mac() in csi.cpp, so discovery scales with this alone.
// See docs note in scene.cpp about the .bss cost of the raise.
#define MAX_BEACONS             6

#define CSI_SEL_COUNT           12
static const int CSI_SEL_SC[CSI_SEL_COUNT] =
    {12, 14, 16, 18, 20, 24, 28, 36, 40, 44, 48, 52};

// ── Beacon transmit rate ───────────────────────────────────────
// 100 Hz is the beacon's STOCK-COMPATIBLE boot default: the extended
// firmware header is explicit that a stock receiver (Cardputer, v0.1
// T-Display) must see no difference until commanded otherwise.  So it
// is a compatibility floor, not a design target.
//
// The extended firmware can be commanded to any rate 1..200 Hz.  Its
// stated purpose is lower ambient channel utilisation during long
// stereo runs, and the best value is an OPEN QUESTION to be settled by
// measurement -- so nothing below may assume a fixed rate.
//
// SAMPLE_RATE_HZ is therefore only the MAXIMUM, used to size buffers.
// The live filter coefficients are computed per beacon from that
// beacon's OWN measured inter-arrival time, so mixed-rate deployments
// (a stock beacon alongside commanded ones) work correctly.
#define SAMPLE_RATE_HZ          100.0f   // max / buffer sizing only

// The rate THIS project runs at.  Commanded to every beacon after
// discovery, and re-commanded whenever a beacon is seen off-rate.
//
// The beacon's 100 Hz boot default exists ONLY so a stock receiver from
// a different project sees an unmodified beacon.  It is never an
// operating point here.
//
// 30 Hz is derived, not picked.  Three constraints bracket it:
//
//   1. scene_update() runs at SCENE_UPDATE_MS (40 ms = 25 Hz), so the
//      rate must be >= 25 Hz or some updates see no fresh frame.
//
//   2. The beacon only light-sleeps when its period exceeds 30 ms
//      (CSI-Beacon-Extended.ino: `s_tx_period_ms > 30`), and it computes
//      period as the integer 1000/hz.  That caps the rate at 32 Hz --
//      33 Hz gives exactly 30 ms and sleep silently stops working.
//
//   3. The information the system actually consumes is bounded by the
//      0.5 s moving-variance window (~2 Hz) and human gross-motion
//      energy (~0.5-5 Hz).  30 Hz gives 15 Hz of Nyquist, a 3x margin.
//
// 30 Hz is the top of the feasible window: fastest sampling that still
// permits power saving, at 30% of stock channel utilisation.
#define BEACON_REQUEST_RATE_HZ  30

// Beacon light sleep between transmits.  ON.
//
// The fault was in the beacon, not in asking for sleep: it called
// esp_light_sleep_start() while Wi-Fi power save was WIFI_PS_NONE, which
// does not keep the radio coherent across a wake -- so beacons woke with
// a dead TX path and stayed silent until power-cycled.
//
// Beacon-side now: PS switches to MIN_MODEM when sleep is armed, the
// sleep return value is checked and disarms itself after 5 rejections,
// the channel is re-asserted after every wake, and a 2 s TX-stall
// watchdog restores the radio if a wake still goes wrong.
#define BEACON_REQUEST_SLEEP    1

// Re-command a beacon whose reported/observed rate has drifted from
// what we asked for (it rebooted, or missed the original broadcast).
#define BEACON_RATE_RECHECK_MS  5000
#define BEACON_RATE_TOLERANCE   3
// ESP-NOW broadcasts are unacknowledged, so the initial command is sent
// several times: one send that collides with beacon traffic is simply
// lost and the beacon would stay at 100 Hz until enforcement noticed.
// 5 repeats, not 3.  Once a beacon is running at 30 Hz WITH light sleep
// it is only listening ~30% of the time (awake ~10 ms of every 33 ms),
// so a single broadcast has ~30% odds: 3 repeats = 66%, 5 = 84%.  The
// 25 ms spacing is deliberately coprime with the 33 ms beacon period so
// the repeats sample different phases of its sleep cycle instead of
// landing on the same one every time.  Costs 100 ms, once, at a state
// transition.
// Gap between individually-addressed pings.  Beacons reply immediately,
// so simultaneous replies to a broadcast ping collide and only one
// survives -- which is why the field log showed PONGs from b1 only.
#define BEACON_PING_STAGGER_MS  12
// Closed-loop config retry.  A beacon is retried only while it has not
// CONFIRMED the requested rate by PONG.  No blind repeats, no delay()
// in a state handler.
#define BEACON_CFG_RETRY_MS     400
#define BEACON_CFG_MAX_TRIES    6
// Suppress enforcement for this long after commanding a rate: the
// inter-arrival EMA (alpha 0.15) needs ~17 frames to converge, so
// judging sooner re-commands a beacon that already obeyed.
#define BEACON_RATE_SETTLE_MS   2000

// ── Signal processing (per-beacon filter chain) ────────────────
// Runs on EVERY beacon frame; not decimated.  Windows are expressed
// in SECONDS and converted per beacon using its measured rate.
#define HAMPEL_WIN              7
#define HAMPEL_THRESH           5.0f
#define MAD_SCALE               1.4826f

// Lowpass cutoff.
//
// Expressed as an ABSOLUTE frequency, clamped to a safe fraction of the
// beacon's own Nyquist.  A fixed fraction-of-Nyquist was wrong: this is
// a DIGITAL filter, so it cannot prevent aliasing (that is decided at
// sampling) -- its only job is choosing how much of the sampled band to
// keep.  Shrinking it proportionally with the rate therefore threw away
// motion bandwidth for no benefit: at 30 Hz it gave a 3.3 Hz cutoff,
// clipping the top of the 0.5-5 Hz band where human gross motion lives.
//
// LP_CUTOFF_HZ 11.0 is the original design intent and is preserved
// wherever the rate can carry it.  The 0.45 clamp keeps the cutoff off
// Nyquist so the 1st-order response is still meaningful there.
//   100 Hz -> 11.00 Hz   (identical to the original filter)
//    50 Hz -> 11.00 Hz
//    30 Hz ->  6.75 Hz   (still above all human motion)
//    20 Hz ->  4.50 Hz
#define LP_CUTOFF_HZ            11.0f
#define LP_CUTOFF_MAX_FRAC_NYQ  0.45f

// Motion-detection window in SECONDS -- a property of people, not of
// the radio.  Buffer is sized for the max rate; the live window is
// min(MOVVAR_WIN, seconds * measured_rate).
#define MOVVAR_SECONDS          0.5f
#define MOVVAR_WIN              ((int)(MOVVAR_SECONDS * SAMPLE_RATE_HZ))

// Empty-room baseline: a fixed SAMPLE COUNT.  This is the number the
// p95 presence threshold was tuned against, so it is held constant at
// every beacon rate; the DURATION is what varies (5 s at 100 Hz, 25 s
// at 20 Hz) and the UI reports the real figure.
#define BASELINE_FRAMES         500

// Number of largest samples that must be retained to compute an EXACT
// p95 over BASELINE_FRAMES observations: the p95 is the
// (n - floor(0.95*(n-1)))-th largest, so only the top few matter.
// Verified bit-exact against a full-buffer percentile over 200 trials.
#define P95_TOPK  (BASELINE_FRAMES - (int)(0.95f * (BASELINE_FRAMES - 1)))

// Advisory only -- the empty-room screen shows the real duration from
// csi_baseline_expected_seconds(), which depends on the beacon rate.
#define CAL_LEAVE_ROOM_SECONDS  10

// Grace period after the empty-room screen appears, BEFORE any sample is
// taken.  The baseline used to start the instant the screen came up, so
// the first samples included the person still standing in the room --
// and that reading is the reference every later measurement is compared
// against.  Solo and tripwire have nobody to press a button, so this is
// on a timer rather than a confirmation.
#define CAL_LEAVE_GRACE_MS      5000

#define MOTION_MULT             1.0f
#define PRESENCE_MULT           3.0f
#define LINK_METRIC_EMA_ALPHA   0.15f

// ── Stereo (dual-RX, v0.2 mechanics unchanged) ────────────────
#define STEREO_BASELINE_CM      6.0f
#define STEREO_MAG_PACK_MAX     8
#define STEREO_PAIR_WINDOW_MS   80
#define STEREO_MIN_BEACONS      2

// ── Peer link (v0.2 mechanics unchanged) ──────────────────────
#define PEER_HELLO_MAGIC        0xC51EE511UL
#define PEER_FRAME_MAGIC        0xC5F1F00DUL
#define PEER_CMD_MAGIC          0xC5CDC0DEUL
#define PEER_BASE_MAGIC         0xC5BA5E10UL
#define PEER_CAL_MAGIC          0xC5CA1DA7UL   // NEW: cal-mode probe→anchor stream
#define PEER_PROBE_POS_MAGIC    0xC5B0B0EEUL   // v0.8: probe→anchor self-position
#define PEER_TRACK_STATE_MAGIC  0xC5715757UL   // v0.8: anchor→probe track list
#define PEER_DISCOVERY_MS       4000
#define PEER_HEARTBEAT_MS       500
#define PEER_TIMEOUT_MS         2500

// ── Hardware (T-Display-S3) ────────────────────────────────────
#define PIN_BTN_LEFT            0
#define PIN_BTN_RIGHT           14
#define PIN_LCD_POWER_ON        15
#define BTN_DEBOUNCE_MS         20
#define BTN_LONG_PRESS_MS       650
#define BTN_COMBO_MIN_MS        30
#define SLEEP_HOLD_MS           1800

// ── Display geometry ──────────────────────────────────────────
#define SCREEN_W                170
#define SCREEN_H                320
#define HEADER_H                22
#define FOOTER_H                22
#define CONTENT_Y               (HEADER_H)
#define CONTENT_H               (SCREEN_H - HEADER_H - FOOTER_H)

// ═══════════════════════════════════════════════════════════════
//  SCENE / RECONSTRUCTION CONSTANTS (v0.3 additions)
// ═══════════════════════════════════════════════════════════════

// Normalized geometry-frame extent.  Beacons live inside [-1, +1] on
// each axis after normalization (whatever the physical triangle size).
// The occupancy field spans a slightly larger box to include the
// "opposite RX" region.
#define SCENE_EXTENT            1.4f    // half-width of field region

// v0.85: normalized radius of the exterior anchor landmark
// (LM_OPPOSITE_RX).  Beacons sit at 1.0 and perimeter midpoints at
// cos(pi/N) <= 0.87, so 1.30 is clearly outside the ring while staying
// inside the field box.  This is the only landmark that samples the
// one-sided-illumination regime the perimeter detector works in.
#define LM_EXTERIOR_RADIUS      1.30f

// Occupancy field grid (2D).  24×24 = 576 cells fits comfortably in
// RAM and renders as ~7×8 px per cell on the 170px screen width.
#define FIELD_DIM               24
#define FIELD_CELL_COUNT        (FIELD_DIM * FIELD_DIM)

// Kernel database (learned during walk cal).
//   Landmarks: 9 (STAND capture, high sample count)
//   Transits:  ~9 legs × 10 resampled points = 90 (WALK capture)
//   Rotation:  1 landmark × N orientation slots = up to 8
// Storage budget: 128 samples × ~64 bytes = 8 KB — comfortable.
// v0.85: right-sized from 160 using the measured worst case.  Samples
// per walk = stands(2N+4) + rotates(3) + walks(2N+3)*N_SLICES, which at
// N=6 is 139.  144 leaves only 5 spare, so any new landmark will need
// this raised -- write_kernel_sample_from_range now warns on saturation
// rather than dropping samples silently.
#define KERNEL_MAX_SAMPLES      144

// Matching-pursuit sparse reconstruction — max simultaneous targets.
// v0.85: raised 4 -> 6.  This is a CEILING on storage, not the number
// the solver will actually fit: scene_update() derives a per-frame
// K_max from how many observation channels are genuinely valid, because
// each target costs DIMS_PER_TARGET unknowns and fitting more targets
// than the data supports just produces damping artifacts.  See
// scene_max_targets_for_frame().
#define SCENE_MAX_TARGETS       6
#define PURSUIT_MAX_ITER        6
#define PURSUIT_MIN_GAIN        0.05f   // stop if residual reduction below this

// Reconstruction scoring weights (adopted from v0.4 ideas ChatGPT tried).
// The score fused across observation channels is
//   score = Σ_channel W_ch · beacon_snr · (channel_match_term_ch)
// Weights control how much each channel gets to bully the pick.
#define AMP_SCORE_WEIGHT        1.00f   // amplitude regression term
#define PHASE_SCORE_WEIGHT      0.18f   // cos(Δphase) — cheap, uses info we already capture
#define AOA_SCORE_WEIGHT        0.55f   // cos(Δbearing) — cheaper than a strong exp gate
#define AOA_MIN_CONF            0.12f   // gate — don't fuse AoA below this confidence

// Sub-cell continuous refinement: after the coarse grid picks a cell,
// iterate REFINE_ITERS times.  Each iteration tests 4 axial candidates at
// `step`, halves `step`, keeps best.  Converges to ~1/(2^ITERS)-cell precision.
#define REFINE_ITERS            4
#define REFINE_STEP_FRAC        0.50f   // initial step as fraction of cell size

// Matching-pursuit spatial exclusion: cells within this Chebyshev
// distance (in grid cells) of an already-picked peak can't be picked
// on subsequent iterations.  Prevents same-target re-extraction.
#define PEAK_EXCLUSION_CELLS    2

// ═══════════════════════════════════════════════════════════════
//  v0.5 — everything captured must reach inference
// ═══════════════════════════════════════════════════════════════
// Robust residual clipping.  Huber loss switches from quadratic to
// linear beyond this many normalized-sigma units.  Prevents a single
// pathological beacon from dominating the score.
#define HUBER_K                 1.5f

// Sigma floors — used when the IDW-interpolated per-cell sigma comes
// back below a sensible physical minimum.  These prevent 0-sigma
// blowups from super-clean landmarks with only a few samples.
#define SIGMA_AMP_FLOOR         0.02f
#define SIGMA_PHASE_FLOOR       0.18f    // ~10°
#define SIGMA_AOA_FLOOR         0.25f    // ~15°

// Sample-count weighting in IDW.  Landmarks with fewer than this many
// frames get down-weighted proportional to sqrt(count/floor).  Prevents
// undersampled landmarks from having outsized voice.
#define IDW_COUNT_FLOOR         20

// Adaptive background — kernel-baseline-relative amp perturbation
// slowly tracks the residual, BUT only when three gates are satisfied
// simultaneously:
//   |residual_b| < BACKGROUND_GATE  (small perturbation only)
//   no active tracks                (scene is empty)
//   novelty < BACKGROUND_QUIET      (residual energy is low)
// This is the "stationary person doesn't vanish" guard.
#define BACKGROUND_ALPHA        0.001f
#define BACKGROUND_GATE         0.05f
#define BACKGROUND_QUIET        0.15f

// EKF-style tracker.  Process noise Q_pos grows position covariance per
// dt² between updates; velocity EMA now weighted by measurement R.
#define TRACK_PROCESS_NOISE     0.02f    // per second, per axis
#define TRACK_R_FLOOR           0.03f    // measurement covariance floor
#define TRACK_GATE_SIGMA        3.0f     // Mahalanobis gating in sigmas

// Alias-flag runtime handling.  A track landing within this normalized
// distance of any alias-pair endpoint gets ambiguity_flag set, its
// confidence multiplied by ALIAS_CONF_MULT, and its covariance floor
// widened by ALIAS_COV_INFLATE.
#define ALIAS_PROXIMITY         0.20f
#define ALIAS_CONF_MULT         0.65f
#define ALIAS_COV_INFLATE       1.75f

// Geometry validation: PROBE at "beacon N landmark" must observe
// beacon N amp perturbation at least this fraction of its max.
#define GEOM_VALIDATION_MIN_FRAC 0.55f

// ═══════════════════════════════════════════════════════════════
//  v0.6 — probabilistic inverse sensor model
// ═══════════════════════════════════════════════════════════════
// The observation vector at each frame is stacked per beacon:
//   y = [amp_0..B-1, phase_0..B-1, aoa_0..B-1]
// so D_OBS = 3 * MAX_BEACONS.  Not all channels are always valid
// (AoA is stereo-only, some beacons may have dropped this frame);
// per-channel `valid` bits gate their contribution to the likelihood.
#define D_OBS                   (3 * MAX_BEACONS)

// State layout per target: (p_x, p_y, alpha).  alpha is a per-target
// scalar strength that absorbs body-size / posture variation across
// people vs the aspect-averaged mean captured in cal.
#define DIMS_PER_TARGET         3
#define MAX_STATE_DIM           (DIMS_PER_TARGET * SCENE_MAX_TARGETS)

// Gauss-Newton solver — bounded steps, trust-region halving.
// Worst-case state is DIMS_PER_TARGET * SCENE_MAX_TARGETS = 18 dims
// (was 12 before SCENE_MAX_TARGETS went 4 -> 6).
#define GN_MAX_ITERS            8
#define GN_MIN_STEP             1e-4f
#define GN_MAX_STEP_POS         0.15f    // normalized units
#define GN_MAX_STEP_ALPHA       0.5f

// Numerical Jacobian step (central-difference finite-difference).
// Fraction of a grid cell — small enough to be linear, large enough
// to be numerically stable given the noise floor.
#define JACOBIAN_STEP_FRAC      0.05f

// Target birth/death.  A target costs 3 free parameters, so the BIC
// penalty is 0.5*3*ln(D_valid) = 1.5*ln(D_valid), computed per frame in
// scene_update() from the channels actually valid that frame.
//
// This constant is the FLOOR under that: at low channel counts the BIC
// term falls below the value this system was tuned around (1.5*ln(6) =
// 2.69 at three beacons with no AoA), and the solver takes the stricter
// of the two.  So 3-beacon birth behaviour matches v0.7 exactly and the
// bar only rises when extra channels justify it.  Still live -- edit it
// and 3..5 beacon behaviour moves.
#define MIN_LOG_LIK_GAIN_TO_ADD 4.0f
#define MIN_ALPHA_TO_KEEP       0.15f
#define BIRTH_SEARCH_MIN_SCORE  0.08f

// Alpha initialization + bounds
#define ALPHA_INIT              1.0f
#define ALPHA_MIN               0.02f
#define ALPHA_MAX               4.0f

// Post-MAP posterior covariance floor & inflation.  The Fisher
// information at the MAP gives the posterior covariance directly;
// we clamp against a floor so we never claim sub-cm precision, and
// alias-region flag inflates the floor as in v0.5.
#define POST_COV_FLOOR          0.010f
#define POST_COV_ALIAS_INFLATE  2.0f

// ═══════════════════════════════════════════════════════════════
//  v0.85 — EXTERIOR REGIME ("perimeter wire")
// ═══════════════════════════════════════════════════════════════
// Outside the walked region there is no learned h(p), so the inverse
// sensor model cannot honestly produce a position.  It can still
// produce a BEARING, from geometry alone: each beacon sits at a known
// angle from the receivers, so the residual amplitude pattern across
// beacons has an angular centroid.  Detections out here are reported
// as a sector, never as a dot — a different claim, not a weaker one.
//
// Nothing is discarded: a residual that the interior solve cannot
// explain becomes an exterior contact rather than silence.

// Max simultaneous exterior sectors tracked.
#define EXT_MAX_CONTACTS        4

// A cell further than this from any kernel sample has no learned
// response and belongs to the exterior regime.  Same value the probe
// self-localizer uses; see PROBE_COVERAGE_RADIUS in scene.cpp.
#define EXT_COVERAGE_RADIUS     0.45f

// Residual energy (normalized amp units, RMS across beacons) below
// which we declare the outside quiet.  Scaled at runtime by the
// aspect-envelope floor measured during the exterior ROTATE, so a
// body presenting its narrowest cross-section still trips it.
#define EXT_TRIP_RMS            0.06f

// Angular concentration below which a bearing is too diffuse to name a
// sector; the contact is still reported, just as "bearing unknown".
#define EXT_MIN_CONCENTRATION   0.25f

// Contact lifetime: how long a sector stays lit after its last trip.
#define EXT_HOLD_MS             2500

// A contact must trip this many consecutive updates before it is
// announced, so a single noisy frame doesn't raise an alert.
#define EXT_CONFIRM_FRAMES      3

// One exterior contact: a direction and a confidence, with no position
// and deliberately no covariance — claiming a covariance would imply a
// localization we have not earned.
struct ExteriorContact {
    bool     active;
    uint8_t  id;
    float    bearing_rad;      // 0 = +Y (toward B1 side), CCW positive
    float    sector_half_rad;  // angular uncertainty, floored at pi/N
    float    energy;           // residual RMS that raised it
    float    confidence;       // 0..1 from angular concentration
    uint8_t  is_self;          // probe operator standing outside the ring
    uint16_t confirm_count;
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
};

// Field rasterization: how strongly each track's Gaussian bump
// contributes to a cell in the display field.  Purely visual.
#define FIELD_RASTER_GAIN       1.0f

// Temporal tracker
#define TRACK_MAX               SCENE_MAX_TARGETS
#define TRACK_MISS_DECAY        20      // frames a track can miss before pruning
#define TRACK_SPAWN_MIN         3       // frames of consistent detection before spawn
#define TRACK_TRAIL_LEN         48

// Update rates
#define SCENE_UPDATE_MS         40      // 25 Hz cap on reconstruction

// ═══════════════════════════════════════════════════════════════
//  APPLICATION STATE MACHINE
// ═══════════════════════════════════════════════════════════════
enum AppState : uint8_t {
    ST_SPLASH,
    ST_PEER_DISCOVERY,
    ST_ROLE_CONFIRM,
    ST_DISCOVERY,             // beacon discovery
    ST_GEOMETRY_GUIDE,
    ST_CAL_INTRO,             // NEW: explain the walk ceremony
    ST_CAL_ANCHOR_PLACE,      // NEW: place ANCHOR unit at origin (stereo)
    ST_CAL_EMPTY_ROOM,        // capture background (user leaves w/ PROBE)
    ST_CAL_LANDMARK_WALK,     // NEW: wizard-driven landmark+transit script
    ST_CAL_FINALIZE,          // NEW: "training your model..." compute
    ST_CAL_RESULTS,           // NEW: quality report + accept/redo
    ST_RX_ASSEMBLY,           // (stereo only, AFTER cal now)
    ST_DASHBOARD,
    ST_SETTINGS,
    ST_SLEEP_ARM,
    ST_SECONDARY_ACTIVE,      // legacy hold — not used in v0.3 primary flow
    // v0.8.  APPENDED AT THE END ON PURPOSE: AppState values travel on
    // the wire as PEER_OP_STATE_HINT arg_u8, so inserting mid-enum would
    // renumber every later state and desync a mixed-firmware pair.
    ST_MOBILE_PROBE,          // probe undocked: self-locates, shows anchor tracks
    ST_DEBUG_LOG,             // v0.9: in-RAM log viewer (Settings -> Debug log)
    ST_PICK_CARRY,            // v0.9: user presses the unit they will carry
};

// Dashboard sub-views.  RADAR is the star; the others are diagnostic.
enum DashView : uint8_t {
    DV_RADAR,        // full-screen occupancy field + tracks (MAIN)
    DV_FIELD,        // raw occupancy grid heatmap + kernel coverage
    DV_AOA,          // per-beacon AoA compass (stereo diagnostic)
    DV_TRIPWIRE,     // simple armed/idle alert
    DV_LINKS,        // per-beacon amplitude/phase perturbation bars
    DV_CSI,          // per-beacon subcarrier plot (raw)
    DV_PEER,         // peer link diagnostics
    DV_COUNT
};

enum RadarMode : uint8_t {
    RM_NONE,
    RM_TRIPWIRE_1,
    RM_LINE_2,
    RM_TRIANGLE_3,
};

enum LinkStatus : uint8_t {
    LS_IDLE,
    LS_MOTION,
    LS_PRESENCE,
};

// Peer role — determined by MAC comparison after discovery.
enum RxRole : uint8_t {
    ROLE_UNKNOWN,
    ROLE_SOLO,
    ROLE_PRIMARY,     // lower MAC — during cal: ANCHOR
    ROLE_SECONDARY,   // higher MAC — during cal: PROBE
};

// Cal mode drives which script variant we run and whether we
// exchange PROBE→ANCHOR observations during the walk.
enum CalMode : uint8_t {
    CAL_MODE_UNKNOWN,
    CAL_MODE_STEREO,   // ANCHOR at origin, PROBE with user
    CAL_MODE_SOLO,     // single unit held by user (degraded fallback)
};

// During cal we tag each RX with its ceremonial role.  In runtime the
// tag is irrelevant (both are just stereo receivers on the bar).
enum CalRxRole : uint8_t {
    CAL_ROLE_NONE,
    CAL_ROLE_ANCHOR,
    CAL_ROLE_PROBE,
};

// User override of the auto MAC-based cal-role tiebreaker.  Toggled
// from settings row 4.  AUTO uses lower MAC = ANCHOR; the FORCE_*
// options let the user pin this unit's role when the auto-pick gets
// confused (e.g. both units end up trying to be ANCHOR after a
// discovery hiccup).  Persists per-boot only.
enum RoleOverride : uint8_t {
    RO_AUTO,
    RO_FORCE_PROBE,
    RO_FORCE_ANCHOR,
};

// Landmarks the walk visits.  Positions computed from the beacon
// geometry during cal_begin (they're not fixed here).
// v0.8: extended to 6 beacons.  Perimeter edge midpoints are named
// by the beacon pair they connect.  For an N-gon the walk visits
// LM_MID_i(i+1) for i = 1..N-1 plus LM_MID_N1 to close the loop, so
// the stop count stays linear in N.  Diagonals (e.g. LM_MID_13 in a
// 4-beacon layout) are NOT visited — LM_MID_13 is a perimeter edge
// only in the 3-beacon triangle, where it is kept for back-compat.
//
// IMPORTANT: LM_BEACON_1..6 must stay contiguous and in order —
// scene.cpp indexes them as (LM_BEACON_1 + i).
enum LandmarkId : uint8_t {
    LM_RX = 0,
    LM_BEACON_1,
    LM_BEACON_2,
    LM_BEACON_3,
    LM_BEACON_4,        // v0.8
    LM_BEACON_5,        // v0.8
    LM_BEACON_6,        // v0.8
    LM_CENTROID,
    LM_MID_12,          // edge B1-B2
    LM_MID_23,          // edge B2-B3
    LM_MID_13,          // edge B1-B3 — perimeter in 3-beacon only
    LM_MID_34,          // v0.8: edge B3-B4  (4/5/6-beacon)
    LM_MID_45,          // v0.8: edge B4-B5  (5/6-beacon)
    LM_MID_56,          // v0.8: edge B5-B6  (6-beacon)
    LM_MID_41,          // v0.8: edge B4-B1  — closes 4-beacon perimeter
    LM_MID_51,          // v0.8: edge B5-B1  — closes 5-beacon perimeter
    LM_MID_61,          // v0.8: edge B6-B1  — closes 6-beacon perimeter
    LM_OPPOSITE_RX,
    LM_COUNT
};

// Wizard script step kinds.
enum StepKind : uint8_t {
    STEP_INTRO,           // splash text only
    STEP_PLACE_ANCHOR,    // stereo: user places ANCHOR at origin
    STEP_EMPTY_ROOM,      // user leaves; capture background
    STEP_STAND,           // stand at landmark_a; capture kernel sample
    STEP_WALK,            // walk landmark_a → landmark_b; capture transit
    STEP_ROTATE,          // stand at landmark_a and rotate 360°
    STEP_END,
};

// ═══════════════════════════════════════════════════════════════
//  PER-BEACON STATE
// ═══════════════════════════════════════════════════════════════
struct BeaconState {
    bool     active;
    uint8_t  mac[6];
    uint8_t  id;

    // Decoded amplitude + phase.
    //
    // v0.9: these were `volatile` because the core-0 CSI callback wrote
    // them directly while core 1 read them.  volatile stopped the
    // compiler caching the loads but gave no atomicity, so a reader
    // could still see half of one frame and half of the next -- which is
    // what the iq_raw seqlock above now prevents.  The callback no
    // longer touches these at all: decode_iq_snapshot() fills them on
    // core 1, and only core 1 reads them.  Dropping volatile lets the
    // compiler keep the filter chain in registers instead of reloading
    // every subcarrier from memory on each access.
    float amplitude[CSI_NUM_SUBCARRIERS];
    float phase[CSI_NUM_SUBCARRIERS];
    // dirty IS still cross-core: set by the callback, cleared on core 1.
    volatile bool  dirty;

    // Beacon-payload counter, extracted from ESP-NOW rx callback.
    // Used to pair stereo observations by frame.
    volatile uint32_t last_counter;

    // ── v0.9: cross-core handoff ─────────────────────────────
    // csi_rx_cb() runs in the Wi-Fi task on CORE 0; csi_process_frames()
    // runs in loop() on CORE 1.  Previously the callback wrote
    // amplitude[]/phase[] directly and core 1 read them with NO
    // synchronisation, so a reader could observe a TORN array -- half
    // frame N, half frame N+1.  The stereo line fit snapshots all 64
    // phases at once, so a torn read fits a line through two different
    // frames and corrupts the bearing intermittently.
    //
    // Fix: the callback only copies raw I/Q into an alternating slot and
    // publishes it with a sequence counter.  All float work moved to
    // core 1, which also removes ~128 sqrtf/atan2f calls per frame from
    // the Wi-Fi task.  iq_seq is odd while a write is in flight.
    volatile uint32_t iq_seq;
    int8_t            iq_raw[2][CSI_NUM_SUBCARRIERS * 2];
    volatile uint8_t  iq_slot;
    volatile uint16_t iq_pairs;

    // Amplitude baseline (empty-room)
    float baseline[CSI_NUM_SUBCARRIERS];
    bool  baseline_valid;

    // Phase baseline for the stereo LO-drift solve.  NOT dead: it is
    // passed by name to stereo_fit_line() in stereo_snapshot_baseline(),
    // which is why a grep for indexed access misses it.
    float phase_baseline[CSI_NUM_SUBCARRIERS];
    float slope_baseline, intercept_baseline;
    bool  phase_baseline_valid;

    // Filter-chain features (used to derive link_metric_ema)
    float feat_turbulence;
    float feat_mean_amp;
    float feat_delta_baseline;
    float feat_energy;
    float feat_temporal_delta;
    float prev_amplitude[CSI_NUM_SUBCARRIERS];

    float hampel_buf[HAMPEL_WIN]; int hampel_idx, hampel_count;
    float lp_x_prev, lp_y_prev;
    // v0.9: per-beacon filter design. Coefficients were static/shared,
    // so one design served every beacon regardless of its actual rate.
    float lp_b0, lp_a1, lp_design_fs;
    float mv_buf[MOVVAR_WIN]; int mv_idx, mv_count;
    float filtered;
    float moving_variance;

    // Calibration accumulators (empty-room)
    // v0.85: was float cal_values[BASELINE_FRAMES] (2000 B/beacon =
    // 12 KB across six) held solely to compute one p95 at the end of
    // baseline.  The p95 of n samples is the (n - floor(0.95*(n-1)))-th
    // LARGEST, which for n=500 is just the top 26 -- so retaining only
    // those gives the identical answer in 108 B.  Verified bit-exact
    // against the full-buffer percentile across 200 trials with n
    // varying 30..500.  This is not an approximation.
    float cal_top[P95_TOPK];   // ascending; cal_top[0] is the smallest kept
    int   cal_top_n;
    int   cal_count;
    // Circular accumulators that produce phase_baseline. Live.
    float cal_phase_i[CSI_NUM_SUBCARRIERS];
    float cal_phase_q[CSI_NUM_SUBCARRIERS];

    float threshold;

    // Legacy walk-cal tracking (kept for compatibility with pre-cal
    // status displays; v0.3's real per-beacon weighting lives in the
    // scene module as walk-derived SNR).
    float    walk_peak;
    bool     walk_calibrated;

    // The scalar per-beacon perturbation signal that scene::observe
    // consumes.  In v0.3 this is the beacon's "brightness perturbation"
    // — how much the beacon→RX radio link is currently being disturbed.
    float link_metric_raw;
    float link_metric_ema;

    LinkStatus status;
    uint32_t frames;
    uint32_t last_frame_ms;
    uint32_t last_cal_frame;
    uint32_t last_proc_frame;

    // v0.4: rate inference — EMA of inter-arrival delta in ms, updated
    // on each frame ingest.  UI translates the number into a mode label
    // (e.g. <30ms → "50Hz normal"; >100ms → "5Hz+sleep extended").
    float    inter_arrival_ms_ema;

    // v0.9: what we actually COMMANDED, and what the beacon reported
    // back.  Previously the UI only had inter_arrival_ms_ema and
    // inferred a mode label from it -- which reads the same whether a
    // command landed or was never sent, so a beacon that ignored us
    // looked identical to one that obeyed.  Showing commanded vs
    // observed makes a failed command visible.
    uint16_t cmd_rate_hz;        // 0 = never commanded
    uint16_t reported_rate_hz;   // from PONG, 0 = never answered
    uint8_t  fw_marker;          // 1 = extended firmware (from PONG)
    uint8_t  sleep_enabled;      // from PONG
    uint32_t last_pong_ms;
    bool     rate_mismatch_logged;   // one-shot, so the log is not spammed
    // v0.9 closed-loop config: a beacon is "pending" until its PONG
    // confirms the requested rate.  Only pending beacons are retried.
    bool     cfg_pending;
    uint8_t  cfg_attempts;
    uint32_t cfg_last_try_ms;

    // Nominal position in normalized geometry frame (set during cal).
    // Beacons are placed such that their triangle centroid is at origin
    // and the triangle inscribes roughly into [-1, +1].
    float pos_x, pos_y;

    // Per-frame stereo line fit (v0.2 stereo math; unchanged)
    float slope_cur, intercept_cur;
    struct StereoRingEntry {
        uint32_t counter;
        uint32_t stamp_ms;
        float    slope, intercept;
        float    mean_amp;
        bool     paired;
    } stereo_ring[STEREO_MAG_PACK_MAX];
    uint8_t stereo_ring_head;

    // Most recent AoA estimate from stereo pairing.  In v0.3 this
    // becomes a triangulation ray input to scene::observe.
    float aoa_rad;
    float aoa_conf;
    uint32_t last_aoa_ms;
    float residual_intercept, residual_slope;
};

// ═══════════════════════════════════════════════════════════════
//  PEER LINK STATE
// ═══════════════════════════════════════════════════════════════
struct PeerState {
    RxRole   role;
    bool     peer_present;
    uint8_t  own_mac[6];
    uint8_t  peer_mac[6];
    uint32_t last_peer_seen_ms;
    uint32_t last_hello_tx_ms;

    // Stereo pairing diagnostics
    uint32_t peer_frames_rx;
    uint32_t peer_frames_dropped;
    uint32_t peer_pairs_ok;
    float    lo_drift_rad;
    float    lo_drift_ema;
    uint32_t last_pair_ms;

    // State-hint channel (PRIMARY→SECONDARY sync)
    uint8_t  primary_state_hint;

    // Cal ceremony role — set during ST_CAL_INTRO transition.
    CalRxRole cal_role;

    // User-picked override for the MAC-based tiebreaker.  RO_AUTO is
    // the default; the FORCE_* variants pin this unit's role.  Toggled
    // from Settings.  Not persisted across reboots.
    RoleOverride role_override;
};

// Peer packet formats (POD, magic-word demuxed).
struct PeerHelloPacket {
    uint32_t magic;
    uint8_t  fw_version[8];
    uint8_t  own_mac[6];
    uint32_t uptime_ms;
    uint8_t  role_wanted;
    uint8_t  _pad[1];
};

struct PeerCsiSummary {
    uint32_t magic;
    uint8_t  beacon_id;
    uint8_t  _pad0[3];
    uint32_t counter;
    uint32_t rx_stamp_ms;
    float    slope, intercept;
    float    mean_amp;
    uint16_t frame_snr_q8;
    uint8_t  baseline_valid;
    uint8_t  _pad1;
};

struct PeerCommand {
    uint32_t magic;
    uint8_t  op;
    uint8_t  arg_u8;
    uint16_t arg_u16;
    uint32_t arg_u32;
};

enum : uint8_t {
    PEER_OP_ENTER_STREAMING = 1,
    PEER_OP_RECALIBRATE     = 2,
    PEER_OP_SLEEP           = 3,
    PEER_OP_STATE_HINT      = 4,
    PEER_OP_CAL_STEP_HINT   = 5,   // NEW: PROBE tells ANCHOR which script step
    PEER_OP_CAL_BEGIN       = 6,   // NEW: begin cal capture window
    PEER_OP_CAL_END         = 7,   // NEW: end cal capture window
    // v0.8 — probe undock mode.
    PEER_OP_UNDOCK_PROBE    = 8,   // PROBE→ANCHOR: probe is now mobile
    PEER_OP_REDOCK_PROBE    = 9,   // PROBE→ANCHOR: probe is back on the bar
    // v0.9: wired-link keepalive.  Carries no meaning beyond "the cable
    // is still there" — an idle but connected wire must not look dead.
    PEER_OP_WIRE_PING       = 10,
    // v0.9: the unit the user pressed tells the other one it is the
    // anchor.  Roles are a user decision, not a MAC-order guess.
    PEER_OP_CAL_ROLE_ANCHOR = 11,
};

struct PeerBaselinePacket {
    uint32_t magic;
    uint8_t  beacon_id;
    uint8_t  valid;
    uint8_t  _pad[2];
    float    slope_baseline;
    float    intercept_baseline;
    uint32_t cal_frames;
};

// NEW in v0.3: PROBE→ANCHOR cal-window observation stream.  Sent by
// PROBE at ~10 Hz during walk cal so ANCHOR builds a joint dataset.
struct PeerCalObservation {
    uint32_t magic;             // PEER_CAL_MAGIC
    uint32_t rx_stamp_ms;
    uint8_t  cur_step_idx;      // wizard script position (from PROBE)
    uint8_t  cur_landmark;      // LandmarkId being visited (255 = transit)
    uint8_t  n_beacons;
    uint8_t  _pad;
    struct PerBeacon {
        uint8_t beacon_id;
        uint8_t have_aoa;
        uint8_t _p[2];
        float   amp_perturbation;
        float   phase_perturbation;
        float   aoa_rad;
        float   aoa_conf;
    } b[MAX_BEACONS];
};

// ═══════════════════════════════════════════════════════════════
//  v0.9 — BEACON CONTROL (RX → beacon)
// ═══════════════════════════════════════════════════════════════
// The extended beacon firmware has always listened for these.  The
// receiver never sent them, so every beacon has been running its
// compiled-in default of 100 Hz with no sleep for the entire life of
// the project, and the "mode" shown in the UI was inferred from
// measured inter-arrival time rather than confirmed.
//
// Structs must match CSI-Beacon-Extended.ino byte for byte.  They are
// duplicated rather than shared because the two sketches build
// separately; if either side changes, change both.
#define BEACON_CMD_MAGIC        0xBCC1D01AUL   // RX → beacon
#define BEACON_PONG_MAGIC       0xBCF0F0AAUL   // beacon → RX

enum : uint8_t {
    BEACON_OP_PING             = 1,   // reply with PONG
    BEACON_OP_SET_RATE         = 2,   // arg_u16 = Hz (1..200)
    BEACON_OP_SET_SLEEP        = 3,   // arg_u16 = 0/1 light sleep
    BEACON_OP_RESTORE_DEFAULTS = 4,   // back to 100 Hz, no sleep
};

struct BeaconCommand {
    uint32_t magic;
    uint8_t  target_id;      // 0 = all beacons, else specific ID
    uint8_t  op;
    uint16_t arg_u16;
    uint32_t arg_u32;
    uint8_t  sender_mac[6];  // where to send the PONG
    uint8_t  _pad[2];
};

struct BeaconPong {
    uint32_t magic;
    uint8_t  beacon_id;
    uint8_t  fw_marker;      // 1 = extended firmware
    uint16_t current_rate_hz;
    uint32_t uptime_ms;
    uint32_t total_tx_count;
    uint8_t  sleep_enabled;
    uint8_t  fw_version[7];
    uint8_t  _pad[4];
};

// (Beacon rate policy lives at the top of this file: SAMPLE_RATE_HZ is
// the stock-compatible maximum used for buffer sizing, and
// BEACON_REQUEST_RATE_HZ is the optional rate the receiver asks
// extended-firmware beacons to adopt.)

// ═══════════════════════════════════════════════════════════════
//  v0.9 — WIRED PEER TRANSPORT (docked mode)
// ═══════════════════════════════════════════════════════════════
// When the two receivers are docked on the bar they are 6 cm apart and
// physically joined, so there is no reason to put their link on the
// air.  Two problems with using ESP-NOW there:
//
//   1. Contention.  The peer link transmits on the SAME radio that is
//      sniffing CSI.  Every peer packet is airtime stolen from the
//      measurement we are actually trying to make.
//
//   2. Time base.  AoA comes from phase disparity across the baseline,
//      so the two units' observations have to be aligned in time.
//      Today they are matched by beacon counter inside an 80 ms
//      window, and any residual skew lands directly in the bearing.
//      A wire gives deterministic, sub-millisecond delivery.
//
// The undocked probe still uses ESP-NOW — the wire is unplugged, and
// that is exactly the case the radio link is for.  Transport is chosen
// per-packet; the packet structs and every handler are unchanged.
//
// PINS — the 4-pin JST beside the USB-C on T-Display-S3.
// Vendor pinout: GND / 3V3 / GPIO43 / GPIO44.  It is wired as the
// STEMMA-QT / Qwiic I2C port (SDA=43, SCL=44), but those are the
// ESP32-S3's U0TXD/U0RXD and can be repurposed as a UART, which is
// what we do here.  43 = TX, 44 = RX.
//
// Two things that follow from these being UART0:
//
//  1. BOOT CHATTER.  The ROM bootloader prints on U0TXD at reset,
//     regardless of firmware.  That garbage lands in the peer's RX.
//     Harmless here only because the framer resynchronises on the sync
//     word and CRC-checks every frame — boot noise cannot be mistaken
//     for a packet.  Do not "simplify" the framing away.
//
//  2. CROSSOVER REQUIRED.  A stock Qwiic/STEMMA cable is straight
//     through, which would join TX to TX and RX to RX — silence in both
//     directions.  The link needs 43 -> 44 and 44 -> 43.  Share GND.
//     Do NOT bridge 3V3: each unit has its own supply, and tying two
//     regulator outputs together is a good way to lose one.
//
//  3. THE CONSOLE MUST BE ON USB.  GPIO43/44 ARE U0TXD/U0RXD, so if
//     the build leaves CDCOnBoot off, `Serial` is UART0 on these exact
//     pins and the debug console fights the peer link for the GPIOs.
//     The CI fqbn therefore carries CDCOnBoot=cdc.  If you build this
//     by hand, set USB CDC On Boot = Enabled or the wire will not work
//     and the serial log will be corrupted.
#define PEER_WIRE_UART_NUM      1
#define PEER_WIRE_TX_PIN        43     // U0TXD, JST pin 3 (SDA position)
#define PEER_WIRE_RX_PIN        44     // U0RXD, JST pin 4 (SCL position)
#define PEER_WIRE_BAUD          1000000

// Framing.  The ESP-NOW path gets whole datagrams; a UART gives a byte
// stream, so packets need explicit boundaries and integrity checking.
// SYNC | len(2, LE) | payload | crc16(2, LE)
#define PEER_WIRE_SYNC0         0xA5
#define PEER_WIRE_SYNC1         0x5A
#define PEER_WIRE_MAX_PAYLOAD   256
// No frame for this long → assume the cable is out, fall back to radio.
#define PEER_WIRE_STALE_MS      400
// Keepalive cadence so an idle-but-connected wire still looks alive.
#define PEER_WIRE_PING_MS       150

enum PeerTransport : uint8_t {
    PEER_TX_NONE = 0,
    PEER_TX_ESPNOW,
    PEER_TX_WIRE,
};

// ── v0.8: probe undock mode packets ───────────────────────────
// These two carry float payloads, so (unlike UNDOCK/REDOCK, which are
// pure signals and ride on PeerCommand) they are their own packet
// types, demuxed by magic in peer_try_consume() exactly like
// PeerCalObservation.
//
// Sent by PROBE at ~10 Hz while it is in ST_MOBILE_PROBE.
struct PeerProbePositionPacket {
    uint32_t magic;             // PEER_PROBE_POS_MAGIC
    uint32_t rx_stamp_ms;
    float    pos_x, pos_y;      // normalized frame (same as scene track pos)
    float    cov_xx, cov_yy, cov_xy;
    float    confidence;        // 0..1 — probe's confidence in its own fix
};

// Sent by ANCHOR at ~5 Hz while the probe is undocked.  Carries the
// full active track list including the is_self flags the anchor
// computed, so the probe renders exactly what the anchor renders.
struct PeerTrackStatePacket {
    uint32_t magic;             // PEER_TRACK_STATE_MAGIC
    uint32_t rx_stamp_ms;
    uint8_t  n_tracks;
    uint8_t  _pad[3];
    struct Track {
        uint8_t  active;
        uint8_t  is_self;
        uint8_t  _pad[2];
        float    pos_x, pos_y;
        float    cov_xx, cov_yy, cov_xy;
        float    confidence;
    } tracks[TRACK_MAX];
};

// ═══════════════════════════════════════════════════════════════
//  FRAME OBSERVATION — single interface between sensor & scene
// ═══════════════════════════════════════════════════════════════
struct FrameObservation {
    uint32_t frame_ms;
    struct PerBeacon {
        uint8_t beacon_id;
        bool    fresh;                 // received new frame this cycle
        bool    have_aoa;              // stereo mode + valid pairing
        float   amp_perturbation;      // link_metric_ema (already bg-subtracted)
        float   phase_perturbation;    // wrap-π delta from phase baseline
        float   aoa_rad;
        float   aoa_conf;
    } beacon[MAX_BEACONS];
    uint8_t n_beacons;
};

// ═══════════════════════════════════════════════════════════════
//  APP CONTEXT
// ═══════════════════════════════════════════════════════════════
struct AppContext {
    AppState    state;
    DashView    dash_view;
    RadarMode   mode;

    BeaconState beacon[MAX_BEACONS];
    int         beacon_count;

    float       sensitivity;

    // Legacy point-estimate fields — kept nominal for compatibility
    // with tripwire/status code that hasn't been ported yet.  The
    // scene module owns the real spatial state now.
    float       est_x, est_y, est_confidence;

    bool        alert_latched;
    uint32_t    last_alert_ms;

    uint32_t    total_csi_frames;
    uint32_t    boot_ms;

    AppState    pre_sleep_state;
    bool        woke_from_deep_sleep;
    uint32_t    state_enter_ms;

    PeerState   peer;

    // Cal mode — set at ST_CAL_INTRO based on peer presence
    CalMode     cal_mode;

    // v0.8 — set on THIS unit when it is the PROBE and has been
    // undocked into ST_MOBILE_PROBE.  Always false on the anchor;
    // the anchor tracks the probe's undock state separately (it
    // learns of it via PEER_OP_UNDOCK_PROBE).
    bool        probe_undocked;
};

extern AppContext g_app;

// ═══════════════════════════════════════════════════════════════
//  MANTISSEC PALETTE  (RGB565)
//
//  Dark, edgy cyber-security aesthetic — deep teal + dark violet
//  structural, electric lime as the "signal detected" accent, hot
//  pink/red for alerts.  Backgrounds sit at near-black with a faint
//  violet undertone so the palette reads as one family.
//
//  Every screen (splash / setup / cal / dashboard / diagnostics)
//  draws from these tokens; changing a token changes every screen.
//
//  Semantic names → concrete hex → RGB565:
//    #007373 teal            (structural primary)     → 0x038E
//    #00A5A5 teal bright     (highlights)             → 0x0534
//    #5D005D violet          (structural secondary)   → 0x580B
//    #8F00A5 violet bright   (accents in violet)      → 0x8814
//    #A8FF00 lime            (electric lime accent)   → 0xAFE0
//    #558000 lime dim        (dim lime for low-int)   → 0x5400
//    #050510 background      (near-black w/ violet)   → 0x0022
//    #E8E8F0 ink             (off-white text)         → 0xEF5E
//    #303045 mid neutral                              → 0x3188
//    #181828 dim neutral                              → 0x18C5
//    #FF8800 warn amber                               → 0xFC40
//    #FF2255 alert hot pink                           → 0xF90A
// ═══════════════════════════════════════════════════════════════
#define COL_MS_TEAL          0x038E    // #007373  primary structural
#define COL_MS_TEAL_BRIGHT   0x0534    // #00A5A5  brighter accent teal
#define COL_MS_VIOLET        0x580B    // #5D005D  secondary structural
#define COL_MS_VIOLET_BRIGHT 0x8814    // #8F00A5  brighter accent violet
#define COL_MS_LIME          0xAFE0    // #A8FF00  ELECTRIC LIME — "signal detected"
#define COL_MS_LIME_DIM      0x5400    // #558000  dim lime for low-intensity
#define COL_MS_BG            0x0022    // #050510  near-black w/ violet undertone
#define COL_MS_INK           0xEF5E    // #E8E8F0  off-white text
#define COL_MS_MID           0x3188    // #303045  mid neutral
#define COL_MS_DIM           0x18C5    // #181828  deep dim
#define COL_MS_WARN          0xFC40    // #FF8800  amber warning
#define COL_MS_ALERT         0xF90A    // #FF2255  hot pink/red alert

// Header/footer band background — sits between BG and MID so text
// pops.  Uses a slightly desaturated violet so the "chrome" reads
// as cyber-sec rather than generic dark UI.
#define COL_MS_CHROME        0x2004    // dark violet chrome band
#define COL_MS_CHROME_LINE   0x580B    // violet separator line

// Full-screen state washes for tripwire — dim variants that let
// bright text on top read cleanly.
#define COL_MS_WASH_OK       COL_MS_LIME_DIM   // secure wash (dim lime)
#define COL_MS_WASH_WARN     0x5240            // motion wash (dim amber)
#define COL_MS_WASH_ALERT    0x5001            // alert wash (dim hot pink)

// Per-beacon / per-channel colors — distinct hues that all live in
// the MantisSec family.  Used for AoA rays, link bars, oscilloscope
// traces, and track ID coloring.  SIX slots, since beacon count and
// simultaneous track count can both reach 6.
//
// A..D are unchanged, so a 3- or 4-beacon install looks exactly as it
// always did.
#define COL_MS_CH_A          COL_MS_LIME          // #A8FF00 yellow-green
#define COL_MS_CH_B          COL_MS_TEAL_BRIGHT   // #00A5A5 cyan
#define COL_MS_CH_C          COL_MS_WARN          // #FF8800 orange
#define COL_MS_CH_D          COL_MS_VIOLET_BRIGHT // #8F00A5 purple
// E and F are NEW hues rather than aliases of existing tokens.  The
// first attempt reused COL_MS_TEAL and COL_MS_ALERT, which was wrong
// twice over: COL_MS_TEAL is the structural primary and is too dark to
// read as a trace on the #050510 background, and COL_MS_ALERT is the
// alarm colour — a routine beacon trace must never render in it.
// These two fill the remaining gaps on the wheel (blue, mint) at
// luminance comparable to A..D.
#define COL_MS_CH_E          0x433F               // #4466FF blue
#define COL_MS_CH_F          0x67F9               // #66FFCC mint
