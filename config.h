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

// ── Version & identity ─────────────────────────────────────────
#define FW_NAME     "MantisSec"
#define FW_VERSION  "0.6.0-model"

// ── Wi-Fi / CSI (unchanged from v0.2) ─────────────────────────
#define CSI_CHANNEL             11
#define CSI_NUM_SUBCARRIERS     64
#define CSI_CENTER_FREQ_HZ      2462000000.0f
#define CSI_WAVELENGTH_M        0.1218f
#define CSI_SUBCARRIER_SPACING_HZ 312500.0f
static const uint8_t BEACON_MAC_PREFIX[5] = {0x1A, 0x00, 0x00, 0x00, 0x00};
#define MAX_BEACONS             4

#define CSI_SEL_COUNT           12
static const int CSI_SEL_SC[CSI_SEL_COUNT] =
    {12, 14, 16, 18, 20, 24, 28, 36, 40, 44, 48, 52};

// ── Signal processing (per-beacon filter chain) ────────────────
#define HAMPEL_WIN              7
#define HAMPEL_THRESH           5.0f
#define MAD_SCALE               1.4826f
#define LP_CUTOFF_HZ            11.0f
#define SAMPLE_RATE_HZ          100.0f
#define MOVVAR_WIN              50

#define BASELINE_FRAMES         500
#define CAL_LEAVE_ROOM_SECONDS  10

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

// Occupancy field grid (2D).  24×24 = 576 cells fits comfortably in
// RAM and renders as ~7×8 px per cell on the 170px screen width.
#define FIELD_DIM               24
#define FIELD_CELL_COUNT        (FIELD_DIM * FIELD_DIM)

// Kernel database (learned during walk cal).
//   Landmarks: 9 (STAND capture, high sample count)
//   Transits:  ~9 legs × 10 resampled points = 90 (WALK capture)
//   Rotation:  1 landmark × N orientation slots = up to 8
// Storage budget: 128 samples × ~64 bytes = 8 KB — comfortable.
#define KERNEL_MAX_SAMPLES      160

// Matching-pursuit sparse reconstruction — max simultaneous targets.
#define SCENE_MAX_TARGETS       4
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
// Iterations are cheap on this state size (12-dim worst case),
// so we can afford several.
#define GN_MAX_ITERS            8
#define GN_MIN_STEP             1e-4f
#define GN_MAX_STEP_POS         0.15f    // normalized units
#define GN_MAX_STEP_ALPHA       0.5f

// Numerical Jacobian step (central-difference finite-difference).
// Fraction of a grid cell — small enough to be linear, large enough
// to be numerically stable given the noise floor.
#define JACOBIAN_STEP_FRAC      0.05f

// Target birth/death — BIC-style penalty for adding a target.
// Adding a target costs 3 free parameters, so the BIC penalty is
// 3·ln(D_OBS·k_frames_worth) where k = ~1 for per-frame BIC.
// We use a simpler tunable: only add a target if the joint MAP
// improves data-fit by at least MIN_LOG_LIK_GAIN_TO_ADD.
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

// Landmarks the walk visits.  Positions computed from the beacon
// geometry during cal_begin (they're not fixed here).
enum LandmarkId : uint8_t {
    LM_RX = 0,
    LM_BEACON_1,
    LM_BEACON_2,
    LM_BEACON_3,
    LM_CENTROID,
    LM_MID_12,
    LM_MID_23,
    LM_MID_13,
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

    // Raw amplitude + phase (updated in RX cb, read in main loop).
    volatile float amplitude[CSI_NUM_SUBCARRIERS];
    volatile float phase[CSI_NUM_SUBCARRIERS];
    volatile bool  dirty;

    // Beacon-payload counter, extracted from ESP-NOW rx callback.
    // Used to pair stereo observations by frame.
    volatile uint32_t last_counter;

    // Amplitude baseline (empty-room)
    float baseline[CSI_NUM_SUBCARRIERS];
    float baseline_std[CSI_NUM_SUBCARRIERS];
    bool  baseline_valid;

    // Phase baseline for stereo LO-drift solve
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
    float mv_buf[MOVVAR_WIN]; int mv_idx, mv_count;
    float filtered;
    float moving_variance;

    // Calibration accumulators (empty-room)
    float cal_values[BASELINE_FRAMES];
    int   cal_count;
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
// traces, and track ID coloring.  Four slots so up to 4 beacons or 4
// simultaneous tracks each get their own hue.
#define COL_MS_CH_A          COL_MS_LIME          // lime
#define COL_MS_CH_B          COL_MS_TEAL_BRIGHT   // bright teal
#define COL_MS_CH_C          COL_MS_WARN          // amber
#define COL_MS_CH_D          COL_MS_VIOLET_BRIGHT // violet-magenta
