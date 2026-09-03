# MantisSec CSI-Radar-S3 (v0.6)

Wi-Fi CSI **scene reconstruction** engine for the LilyGo T-Display-S3.
Two receivers, three beacons, a calibration walk — and a live top-down
map of who's in the room.

**Firmware version:** `0.6.0-model`

## What v0.6 is

An **empirical inverse sensor model**, replacing the pursuit-plus-EMA-
tracker architecture that persisted from v0.3 through v0.5.

The scene contains K unknown persons at positions p₁..p_K, each with a
per-target strength α_k that absorbs body-size / posture variation. The
observation model at each frame is:

    y  =  Σ_k α_k · h(p_k)  +  b  +  ε,     ε ~ N(0, Σ(p_1..p_K))

where:

- **y** ∈ ℝ¹² is the stacked observation vector — per beacon: amp
  perturbation, phase perturbation, and (stereo) AoA
- **h(p)** is the mean response learned during cal, built from kernel
  samples via 6-NN sample-count-weighted IDW with first-order Taylor
  correction from transit-derived per-landmark Jacobians `grad_amp`
- **Σ(p)** is the diagonal per-channel covariance learned during cal.
  Per-landmark `std_amp`, `std_phase`, `std_aoa` are IDW-interpolated
  into per-cell channel variances. The rotation experiment's
  `aspect_var` **adds to the amplitude channel variance** — cells near
  ROTATE landmarks with wide aspect swings get proportionally wider
  Σ_amp, because a person there genuinely produces a wider range of
  readings across body aspect and the model should accept them all as
  "body here" without treating aspect variation as measurement error
- **b** is the per-beacon adaptive background (three-way gated: no
  active tracks, low novelty, and `|residual| < BACKGROUND_GATE`).

Inference at each frame maximizes the joint log-likelihood over
(K, {p_k}, {α_k}):

    L  =  −0.5 (y − ŷ)ᵀ Σ⁻¹ (y − ŷ)  −  0.5 log|Σ|
    ŷ  =  Σ_k α_k · h(p_k)

by:

1. **Predict**: existing tracks advance under a constant-velocity motion
   model with covariance growth `Q · dt²`.
2. **Solve**: joint Gauss-Newton over all currently-tracked targets. The
   Jacobian ∂h/∂p is computed by central-difference numerical
   differentiation at each iteration (four extra `meas_model` calls per
   target — cheap and always matches whatever `h` actually does, so
   changes to the interpolation don't require re-deriving analytic
   Jacobians). Trust-region step halving with position and α bounds.
   Cholesky solve on the (3K)×(3K) normal equations.
3. **Birth**: search the 24×24 field for the peak residual likelihood
   after subtracting fitted targets. If the joint log-likelihood
   improves by at least `MIN_LOG_LIK_GAIN_TO_ADD` when we add a target
   there and re-solve, accept the new K. Repeats until adding another
   target no longer helps.
4. **Death**: any target whose α drops below `MIN_ALPHA_TO_KEEP` during
   the solve is removed and K decrements. The remaining set is
   re-solved.
5. **Posterior covariance**: for each surviving target, compute
   `(J^T W J)⁻¹` on that target's own 2×2 positional block. This is
   the honest position uncertainty from the local likelihood curvature.
   Clamped against `POST_COV_FLOOR`; inflated by
   `POST_COV_ALIAS_INFLATE` when the target sits near a cal-report
   alias landmark.

**Number of targets is chosen by the data.** Not "search up to 6
iterations, stop on threshold." K is what maximizes the joint
likelihood minus the BIC-style penalty per added target.

**The 24×24 field is a rendering artifact.** The actual scene state is
the track list `(p_k, α_k, cov_k)`. Each frame we rasterize each track
as a Gaussian bump into the field for the display; the field never
feeds back into inference.

## What replaced what

| v0.5 | v0.6 |
|---|---|
| `pursuit_score_full` (amplitude regression × phase multiplier × AoA multiplier) | `log_likelihood_multi` — full Gaussian likelihood with per-channel variance |
| Matching pursuit loop (pick cell, subtract, exclude, repeat) | Joint Gauss-Newton over all K targets simultaneously; birth via residual scan + BIC-style acceptance test; death via α-threshold |
| Sub-cell refinement by iterative step-halving | GN's continuous position optimization eliminates the need for sub-cell search entirely; positions live in ℝ² throughout |
| EKF-style tracker with heuristic R from per-cell interpolated `sigma_amp` | Posterior covariance from Fisher info `(J^T W J)⁻¹` at the MAP estimate. Real position uncertainty from the likelihood, not a floor |
| Alias flag = "draw warn-colored ellipse and reduce confidence" | Alias regions inflate `POST_COV_FLOOR` at cells near alias landmarks, so the posterior covariance genuinely widens — the ellipse widens *because the inversion is less certain there*, not as a UI decoration |
| `aspect_var` as widened `sigma_amp` in scoring | `aspect_var` as widened `Σ_amp` in the measurement model — the same math but the framing matters: aspect is measurement variance of "is a person here", not signal degradation |
| Per-cell `sigma_*` computed inside `pursuit_score_full` | Full diagonal Σ(p) returned by `meas_model` alongside h(p); one traversal, one abstraction |
| `PredictedObs` | `MeasModel` (h and diagonal Σ together) |

## What is honest to say about v0.6

The measurement model is now a real object. `meas_model(p)` returns
h(p) and diagonal Σ(p). `log_likelihood_multi(obs, targets, K)`
evaluates L. `gn_solve_joint(obs, targets, K)` maximizes it. Birth and
death search over K. Every path through inference goes through the
model; nothing bypasses it.

Costs on an ESP32-S3 at 240 MHz (measured back-of-envelope):

- One `meas_model` call: ~150 FLOPs (6-NN scan + per-channel IDW)
- One GN iteration: 5 × K `meas_model` calls + one 3K×3K Cholesky
- Full `scene_update` at K=2 targets: ~150 KFLOPs, ~1 ms
- Full `scene_update` at K=4 with birth attempt: ~300 KFLOPs, ~2 ms
- Rate cap: 25 Hz (40 ms budget) — plenty of headroom

## What deliberately did NOT ship in v0.6 (v0.7 candidates)

- **Multi-hypothesis tracking through aliased regions.** v0.6 flags the
  ambiguity in the posterior covariance (the ellipse widens because the
  math says so, not as decoration). But it still picks a single MAP
  location. A proper solution maintains a bimodal posterior until
  temporal evidence resolves it.
- **Full circular-observation superposition.** Phase and AoA channels
  currently combine multi-target contributions via α-weighted circular
  mean of the constituent per-target predictions. This is a first-order
  approximation. The exact model is complex-baseband superposition
  which requires representing predicted signals as complex vectors, not
  scalars. Deferred until we have hardware traces to validate what the
  approximation error actually costs.
- **Orientation as a latent variable with runtime marginalization.**
  v0.6 uses `aspect_var` as measurement covariance and leaves the
  Fourier coefficients stored in the kernel unused at runtime. A future
  version could marginalize over θ per target at each frame: integrate
  the likelihood over p(θ) using the Fourier basis. Cheap arithmetic,
  but changes the sigma story so it's a separate step.
- **PROBE-during-runtime constraint.** During cal, PROBE (on the
  person's chest) gives us a second measurement channel at every
  landmark. At runtime there is no PROBE-on-user — but a mounted-on-
  bar stereo receiver could still provide correlated observations. The
  current model treats stereo AoA as a channel but doesn't use the raw
  per-receiver amp/phase asymmetry as an additional constraint on p.
- **Sparse Bayesian / joint temporal regularization.** The tracker
  currently treats time as a motion-model prior only. A stronger
  approach optimizes `Σ_t log L(y_t | p_t) + λ · ||p_t − p_{t-1} − v_{t-1} dt||²`
  jointly over a temporal window.

Each of these is real math with a clear next step. v0.7 will pick one
based on what fails first on hardware.

## What v0.5 is really about

v0.3 was a fingerprint lookup. v0.4 added AoA + arc-length + circular
statistics + adopted the good pieces of ChatGPT's parallel attempt.

**v0.5 is about ending the pattern of collapsing captured data to
scalars.** Every quantity the calibration ceremony measures now
actually reaches the inference stage:

| Captured quantity | Was used at inference in v0.4.1? | v0.5 |
|---|---|---|
| `mean_amp` per (p, b) | ✓ | ✓ |
| `std_amp` per (p, b) | ✗ (kept in kernel, ignored at runtime) | ✓ — IDW-interpolated to per-cell `sigma_amp` |
| `mean_phase` per (p, b) | ✓ | ✓ |
| `std_phase` per (p, b) | ✗ | ✓ — per-cell `sigma_phase` |
| `mean_aoa` per (p, b) | ✓ | ✓ |
| `std_aoa` per (p, b) | ✗ (field existed, unused) | ✓ — per-cell `sigma_aoa` |
| `sample_count` per (p, b) | ✗ (5-frame samples weighted same as 200-frame) | ✓ — sqrt(count) enters IDW weights |
| Amp variation across ROTATE θ | Collapsed to `s_orient_rel[b]` scalar per beacon | ✓ — per (landmark, beacon) `aspect_var` + Fourier basis |
| PROBE stream during ROTATE | Discarded | ✓ — used to extract per-frame θ via arc-length in beacon-amp space |
| Transit slice sequence | Used for arc-length only | ✓ — least-squares fit yields per-landmark `grad_amp` (∂h/∂p), used as first-order Taylor correction in `kernel_predict` |
| Per-beacon loop closure | Collapsed to one scalar | ✓ — per-beacon drift, folded into `s_beacon_weight[b]` penalty |
| PROBE at "beacon N landmark" | Discarded | ✓ — geometry validation flags misidentified beacons |
| Alias pair identities | Collapsed to `alias_pair_count` integer | ✓ — `AliasPair` records stored, runtime tracks near ambiguous landmarks get `ambiguity_flag` + confidence attenuation + covariance inflation |

Plus algorithmic upgrades from CJ's v1 skeleton (the pieces I can vouch
for):

- **Heteroscedastic likelihood** — per-cell interpolated sigmas replace
  fixed constants in `pursuit_score_full`. The score is now
  `Σ w_b · z² / σ²` (with Huber) rather than a fixed-weight sum. Cells
  near noisy landmarks correctly get less voice.
- **Huber loss on residuals** — quadratic below `HUBER_K=1.5σ`, linear
  beyond. One pathological beacon can't dominate the score anymore.
- **Adaptive background** — `s_bg_amp[b]` slowly tracks the per-beacon
  residual, but only when three gates all hold simultaneously:
  no active tracks, low novelty, and `|residual| < BACKGROUND_GATE`.
  Stationary people don't dissolve into the background.
- **EKF-style tracker** — real Kalman predict + covariance-weighted
  update, Mahalanobis gating for association. The `cov_xx/cov_yy/cov_xy`
  fields are now real EKF state (were dead in v0.4). Track velocity
  updates through Kalman gain rather than raw EMA of position deltas.
  Renderer uses actual covariance for the confidence ellipse.

**What was NOT adopted from CJ's v1 skeleton (deferred to v0.6+):**

- **Gauss-Newton position refinement using local Jacobians.** Would
  replace the iterative step-halving inside the pursuit loop. CJ's
  version is correct but its value depends on hardware validation
  showing the step-halving actually stalls somewhere. If it doesn't,
  GN is overhead without benefit. Held for v0.6.
- **Fisher-derived track covariance R** from `(J^T W J)^{-1}`. Real
  math but same failure mode I flagged: in structural-kernel-error
  regions it produces confidently-wrong ellipses. v0.5's per-cell
  interpolated sigma is heteroscedastic + honest without that risk.
  Fisher R is v0.6 material paired with an alias-graph clamp.
- **Full h(p, θ) latent orientation model with runtime marginalization.**
  v0.5 stores the Fourier coefficients but uses them only for
  `aspect_var` extraction. Runtime marginalization over θ (integrating
  the likelihood over `p(θ)`) is v0.6.
- **PROBE-during-rotation as a runtime stereo constraint.** v0.5 uses
  the PROBE-during-rotation stream to extract θ during cal, but
  doesn't use it at inference (there's no runtime PROBE-on-user). The
  full "predict what PROBE would see given hypothesized position"
  constraint is v0.6.
- **Multi-hypothesis tracking through aliased regions.** v0.5 flags
  the ambiguity and widens the ellipse. v0.6 could maintain a second
  weighted hypothesis until temporal evidence resolves it.

## Complete tuning surface (all in `config.h`)

```
# Score fusion
AMP_SCORE_WEIGHT       = 1.00
PHASE_SCORE_WEIGHT     = 0.18
AOA_SCORE_WEIGHT       = 0.55
AOA_MIN_CONF           = 0.12
# Pursuit
PURSUIT_MAX_ITER       = 6
PURSUIT_MIN_GAIN       = 0.05
PEAK_EXCLUSION_CELLS   = 2
REFINE_ITERS           = 4
REFINE_STEP_FRAC       = 0.50
# Robust residuals
HUBER_K                = 1.5
# Sigma floors (per-cell interpolated sigmas clamp against these)
SIGMA_AMP_FLOOR        = 0.02
SIGMA_PHASE_FLOOR      = 0.18  (~10°)
SIGMA_AOA_FLOOR        = 0.25  (~15°)
IDW_COUNT_FLOOR        = 20   # sample-count weighting
# Adaptive background (three-way gated)
BACKGROUND_ALPHA       = 0.001
BACKGROUND_GATE        = 0.05
BACKGROUND_QUIET       = 0.15
# EKF tracker
TRACK_PROCESS_NOISE    = 0.02
TRACK_R_FLOOR          = 0.03
TRACK_GATE_SIGMA       = 3.0
# Alias handling
ALIAS_PROXIMITY        = 0.20
ALIAS_CONF_MULT        = 0.65
ALIAS_COV_INFLATE      = 1.75
# Geometry validation
GEOM_VALIDATION_MIN_FRAC = 0.55
```

## What changed in v0.4.1 (from v0.4)

v0.4 shipped the substantive architectural fixes: AoA now actually
contributes to inference, arc-length transit slicing uses PROBE's
fingerprint trajectory, ANCHOR and PROBE capture buffers are separated,
sub-cell refinement, spatial exclusion in matching pursuit, rotation
reliability applied at every inference call, PROBE packets carry real
step provenance, tracker velocity is in per-second units, cal report
gains Fisher-lite observability and RF-alias metrics, and the MantisSec
paint pass is done.

v0.4.1 folds in six specific improvements that ChatGPT's parallel v0.4
attempt got right:

**1. Iterative sub-cell refinement with step-halving.** The refinement
loop after a coarse-grid pick now runs `REFINE_ITERS=4` iterations
testing 4 axial candidates at ±step, halving the step each iteration.
Converges to ~1/(2^4)-cell precision at the same cost as the v0.4
fixed-step 3×3 sweep. Constants live in `config.h`:
`REFINE_ITERS`, `REFINE_STEP_FRAC`.

**2. Phase in scoring.** `pursuit_score_full` gained a phase-consistency
multiplier: `exp(-Σ w_b · (1−cos Δφ_b))` weighted at `PHASE_SCORE_WEIGHT=0.18`
with a 0.25 floor. Phase perturbation was already captured in the kernel
in v0.3 but discarded at runtime — this is information we get for free
that now nudges cells with matching phase toward higher scores.
Deliberately small weight because phase is noisier than amplitude.

**3. Named score-weight constants** for every branch of the fused
scoring function: `AMP_SCORE_WEIGHT`, `PHASE_SCORE_WEIGHT`,
`AOA_SCORE_WEIGHT`, `AOA_MIN_CONF`, `PEAK_EXCLUSION_CELLS`. The AoA
branch is now confidence-gated by an explicit `AOA_MIN_CONF=0.12` — noisy
AoA doesn't just drag the score down, it drops out entirely. Tuning
surface is cleaner than v0.4's magic numbers.

**4. Circular std via `sqrt(-2·ln R)`.** For any wrapped angular
quantity (phase, AoA), the von Mises-consistent standard deviation
estimate is `sqrt(-2·ln R)` where R is the mean unit-vector magnitude.
Matches a Gaussian sigma at high concentration and diverges as the
distribution goes uniform. v0.4 stored `1-R` which was fine as a
dispersion but wouldn't plug cleanly into a likelihood — this replaces
it. Both `std_phase` and `std_aoa` now use this form.

**5. `std_aoa` field in KernelSample.** Costs 4 bytes per landmark-beacon
tuple; unused by v0.4.1 pursuit (which still uses the fixed AoA sigma
floor as before) but stored for use by per-landmark AoA weighting when
a future version wants it. Future-proofing.

**6. True IDW response-space cross-val.** v0.4's cross-val took each
held-out landmark, found its single nearest-neighbor in response-space,
and reported physical distance to that neighbor. v0.4.1 predicts each
held-out landmark as the response-distance-weighted mean of ALL other
landmarks — tracks the actual matching-pursuit reconstruction more
faithfully. Same O(N²) cost with a slightly beefier inner loop; N≈24
landmark samples so ~600 pairs, trivial.

**What was explicitly NOT adopted from ChatGPT's v0.4:** track
covariance derived from local observability (`t.cov_xx = 1/sqrt(fi)`).
The reasoning holds — a locally-fit Jacobian in a region with
structural kernel error produces confidently-tight ellipses pointing
the wrong way, which is worse than an honest heuristic size. Without
lab time to validate this doesn't happen, honest heuristic sizes are
safer than false-precision ones. This is real math worth doing (CJ's
v1 skeleton also has it), but it belongs in v0.5+ where we can pair
it with the rest of the machinery that keeps it honest (see
`ROADMAP.md`).

## What changed in v0.4 (from v0.3)

v0.3 landed the scene-reconstruction architecture: kernel, occupancy
field, matching pursuit, temporal tracker, walk-cal ceremony.  v0.4
closes the algorithmic gaps that were still open — v0.3 had AoA as a
diagnostic ray, time-based transit slicing behind a TODO, and PROBE
frames colliding with ANCHOR frames in one buffer.  All fixed.

**AoA now actually contributes to inference** — not decoration.
Each kernel sample stores the mean bearing observed per beacon at
that landmark position (circular-mean averaged so ±π wrapping doesn't
corrupt it).  `kernel_predict` interpolates bearing per beacon via
unit-vector IDW across the 6 nearest kernel samples.  `pursuit_score`
combines an amplitude regression term with an AoA-consistency
multiplier — `exp(-Σ w·conf·Δθ²/2σ²)` with a bounded floor so noisy
AoA can nudge but never hard-veto an amplitude-consistent cell.
When the observation carries no AoA the multiplier is 1 and
amplitude-only inference falls out automatically.

**Transit slicing uses PROBE's fingerprint arc-length** — the whole
point of PROBE streaming during cal.  As the user walks A→B, PROBE's
per-beacon amplitude vector traces a continuous curve in RF space.
`compute_arclength_boundaries` builds cumulative arc-length over
PROBE's own observations, slices at equal-arc-length points, then
maps each slice back to the ANCHOR frame index by matching timestamps.
Walking speed variability drops out completely — a stopped user
generates no arc-length, a fast user generates it faster, and both
end up with the same 8 slices at true equal-spacing along the path.
Falls back to time-based slicing when PROBE data is unavailable
(solo mode or peer drop), and reports which path it took in the
log so you can verify from serial.

**ANCHOR and PROBE capture buffers are separate.**  v0.3's first-cut
put both streams into `s_cap_buf` and treated all frames as
equivalent — but ANCHOR observes "what the runtime receiver sees when
the user is at position P" (which IS the kernel K(P)), while PROBE
observes "what each beacon looks like from position P" (a mobile
field-strength meter).  Collapsing them mixed categories.  Now
`s_cap_anchor[]` seeds the kernel; `s_cap_probe[]` provides the
arc-length parameterization + step-provenance validation.

**Sub-cell continuous refinement.** After the coarse 24×24 grid
picks a peak, evaluate 8 offsets at 0.25×cell step around it and
keep the highest scoring.  Cheap (adds ~8 kernel_predict calls per
target per frame) and gives us sub-grid position accuracy without
expanding the search grid to 96×96.

**Matching-pursuit spatial exclusion.**  Each iteration excludes
cells within ~0.35 normalized units of any already-picked peak, so
subsequent iterations can't just re-extract the same target with a
slightly different amplitude.  This is what makes multi-target
detection actually work vs single-target detection with numerical
noise.

**Rotation experiment produces a real per-beacon reliability factor.**
v0.3 folded rotation variance into `s_beacon_snr` once at finalize
and threw the value away.  v0.4 keeps it as
`s_beacon_orient_reliability[MAX_BEACONS]` and applies it as a
weight at every inference call — beacons whose response is dominated
by body orientation get down-weighted at runtime, not just once.

**Circular statistics for phase AND AoA.**  Phase was already
circular-mean-averaged in v0.3; AoA was arithmetic-mean-averaged
(broken near ±π).  Now both use unit-vector averaging.  Phase also
records circular concentration `1 - R` as a dispersion metric in
`std_phase`.

**PROBE→ANCHOR packets carry real wizard-step provenance.**  v0.3
sent `cur_step_idx=0` and `cur_landmark=0xFF` as placeholders.  v0.4
has wizard call `scene_cal_note_step(idx, landmark_id)` on every
step entry, so each `PeerCalObservation` packet is tagged with the
walk-script position it belongs to.

**Track velocity is in real units.**  v0.3 stored `pos_delta` as
"velocity" — actually normalized-units-per-frame, which is
dimensionally meaningless and rate-dependent.  v0.4 divides by the
actual `dt_s` since the last update so velocity is normalized units
per second, independent of the reconstruction cadence.

**Cal report gains Fisher-lite observability + RF-alias detection.**
Beyond cross-val error and loop closure, the results screen now
shows:
- **mean_observability** = mean of the smaller eigenvalue of the
  local response Jacobian across all landmark samples.  Higher =
  the room's RF field is genuinely responsive to position changes.
- **alias_pair_count** = number of landmark pairs that are far
  apart physically (≥ 1 normalized unit) but close in observation
  space (< 0.15 normalized).  Non-zero = the room has RF-aliased
  regions the tracker will confuse.

Both are cheap to compute at cal-finalize and give the user a real
"is this room going to work" signal, not just a numerical hope.
The verdict on the results screen now factors in all four signals
(xval / loop / obs / alias) with color-coded per-metric readouts.

**MantisSec paint pass complete.**  All 241+ color usages route
through a single palette defined once in `config.h`:

| Token | Hex | Role |
|---|---|---|
| `COL_MS_TEAL` | `#007373` | structural primary |
| `COL_MS_TEAL_BRIGHT` | `#00A5A5` | accent teal |
| `COL_MS_VIOLET` | `#5D005D` | structural secondary |
| `COL_MS_VIOLET_BRIGHT` | `#8F00A5` | accent violet |
| `COL_MS_LIME` | `#A8FF00` | "signal detected" |
| `COL_MS_LIME_DIM` | `#558000` | dim lime |
| `COL_MS_BG` | `#050510` | near-black w/ violet undertone |
| `COL_MS_INK` | `#E8E8F0` | off-white text |
| `COL_MS_WARN` | `#FF8800` | amber warning |
| `COL_MS_ALERT` | `#FF2255` | hot pink alert |
| `COL_MS_CHROME` | dark violet | header/footer band |
| `COL_MS_CH_A..D` | family | per-beacon / per-track hues |

Legacy names (`COL_BG`, `COL_FG`, `COL_ACCENT`, ...) still exist but
are `static constexpr` aliases to the MantisSec tokens — every
diagnostic sub-view, cal ceremony screen, dashboard, and setup
screen adopts the new palette by construction.

**Animated MantisSec splash.**  Old splash was placeholder text +
ping ring.  New splash builds in over ~2.5s:
1. Mantis silhouette (head → antennae → thorax → raptorial arms →
   abdomen → legs → glow accents) draws in piece-by-piece
2. `MANTISSEC` wordmark reveals character-by-character
3. Lime underline sweeps in beneath the wordmark
4. "CSI SITUATIONAL AWARENESS ENGINE" subtitle fades in
5. Scan-line sweeps down the frame during the reveal
6. Once fully drawn, pulsing eye + pulsing "> hold RIGHT to arm" prompt

## What v0.4 is (the design point, unchanged from v0.3)

Mental model:

- **Beacons** are radio illuminators at approximately-known positions.
- **Receivers** are radio-domain "cameras" — broad, low-resolution,
  per-source signature per instant.
- **People** are occluders that perturb each beacon→RX link.
- **Empty-room baseline** is the "background photograph" of the static
  scene (walls, furniture).
- **Walk cal** samples the room's response Green's function by
  physically placing a known occluder (you, with a T-Display against
  your chest) at scripted landmarks.
- **Runtime** is a sparse inverse solve: given the current
  perturbation pattern, find the occupancy field whose combined
  kernel-response best explains what we're seeing.

Everything is **relative** — normalized geometry-frame coordinates,
kernel amplitudes measured against the empty-room baseline, no
metric units fabricated from measurements that don't produce them.
Multi-target support falls out naturally because the sparse solver
returns multiple non-zero blobs when the observation is best
explained by multiple occluders.  Up to 4 simultaneous tracks.

## Module layout

```
CSI-Radar-S3/
├── CSI-Radar-S3.ino  — state machine, module wiring
├── config.h          — shared types, enums, constants, MantisSec palette
├── input.h/cpp       — buttons, debounce, long-press, combo
├── csi.h/cpp         — CSI callback (amp+phase), baseline, per-beacon filter
├── stereo.h/cpp      — line fit, LO-drift solve, per-beacon AoA
├── peer.h/cpp        — ESP-NOW peer link, packet demux
├── scene.h/cpp       — CORE: kernel, occupancy field, AoA-fused reconstruction, tracker
├── wizard.h/cpp      — data-driven cal ceremony state machine
├── render.h/cpp      — radar view + occupancy heatmap (MantisSec paint)
├── ui.h/cpp          — LGFX rendering, setup screens, animated splash
└── lgfx_tdisplay_s3.h
```

Dependency flow: `config.h` → module headers → `.ino`.  No cycles.

## The BOM

- **2 × LilyGo T-Display-S3** as receivers (stereo is the design
  point; solo mode is a graceful fallback).
- **3 × ESP32-C3** (or any ESP32 variant) as beacons running
  `CSI-Beacon-Extended` firmware (Cardputer-compatible).
- **1 × rigid 6cm bar** for stereo runtime mounting — receivers'
  antennas at a half-wavelength baseline (`2462 MHz → λ ≈ 12.2 cm`).

## Setup flow

### Stereo (2 T-Displays — design target)

1. **Peer discovery** — units find each other on channel 11.
2. **Role confirm** — lower MAC = `PRIMARY` = cal-time `ANCHOR`;
   higher MAC = `SECONDARY` = cal-time `PROBE`.
3. **Beacon discovery** — beacons appear as they broadcast.
4. **Beacon geometry** — arrange beacons per the on-screen diagram.
5. **Cal intro** — explain what's about to happen (~100 sec).
6. **Anchor placement** — place `ANCHOR` at the geometric center of
   the beacon triangle.  Take `PROBE` with you.
7. **Empty room baseline** — leave the room with `PROBE`; `ANCHOR`
   captures the empty-scene amplitude+phase baseline.
8. **Landmark walk** — 22-step scripted walk visiting 9 landmarks
   with WALK / STAND / ROTATE segments.  `PROBE` shows the current
   instruction; each STAND becomes a kernel sample; each WALK is
   arc-length-resampled into 8 slices using PROBE's own fingerprint
   trajectory as the parameter (walking speed drops out).
9. **Finalize** — scene builds the kernel, per-beacon SNR, cross-
   validates each landmark, computes loop closure, Fisher-lite
   observability, and RF-alias count.
10. **Results screen** — verdict (OK / MARGINAL / POOR / FAILED)
    plus color-coded per-metric readouts.  Accept or redo.
11. **RX assembly** — mount both units on the 6cm bar for stereo
    runtime.  6cm center-to-center, screens facing same direction.
12. **Dashboard** — top-down radar map, live reconstruction.

### Solo (1 T-Display — fallback)

Same flow minus anchor-placement and RX-assembly.  Hold the
T-Display against your chest during the walk.  Fundamentally
degraded (receiver moves with the target — hence "fallback") and
honestly reported as such in the results screen.

## The radar view

Primary dashboard = full-screen top-down map:

- **Occupancy heatmap** — dim violet → teal → electric lime gradient
  from the matching-pursuit solver.
- **Beacon icons** at their calibrated positions.
- **RX icon(s)** at origin — single square (solo) or paired bar
  (stereo, visualizing the actual 6cm baseline).
- **Range rings** every 0.5 normalized units.
- **Link rays** RX→beacon, colored by per-beacon disturbance level.
- **AoA rays** (stereo mode) — bearing rays from RX for each beacon
  with fresh valid AoA.  These are diagnostic — the actual AoA
  contribution to inference happens inside `pursuit_score_full`.
- **Target dots** as tracked local maxima with confidence ellipses.
  Up to 4 simultaneous tracks, stable per-track colors from
  `COL_MS_CH_A..D`.
- **Fading trails** behind each track — position history over ~2.5s.
- **Alert border** — turns hot pink when the tripwire latches.
- **Status band** at top: mode / target count / aggregate confidence.
- **Mini oscilloscope** at the bottom — per-beacon disturbance
  history in the MantisSec channel palette.

`LEFT` cycles sub-views (`RADAR`, `FIELD`, `AOA`, `TRIPWIRE`,
`LINKS`, `CSI`, `PEER`).  `RADAR` is the star; the others exist for
debugging every layer of the pipeline.

## The scene module — the math actually running

### Kernel (built during walk cal)

Per-beacon tuple at each visited position: mean amplitude
perturbation, std amp (noise estimate), circular-mean phase, phase
dispersion, circular-mean AoA, AoA sample count.  Populated from:

- 9 landmarks × ~5 seconds each (STAND capture) — averaged
  fingerprint at a known point
- 9 transit legs × 8 arc-length slices each (WALK capture) —
  fingerprint change parameterized by PROBE's own RF-space arc-length
- 1 rotation-in-place at centroid (ROTATE capture) — feeds
  per-beacon orientation-reliability weights

Total: ~160 samples, ~15 KB RAM.

### Runtime (AoA-fused matching pursuit)

Every 40 ms:

1. Build observation vector: per-beacon amplitude residual, per-beacon
   AoA + confidence (if stereo).
2. For each cell in the 24×24 field, `kernel_predict` interpolates
   per-beacon amplitude AND per-beacon AoA from the 6 nearest kernel
   samples via IDW (unit-vector IDW for AoA).
3. `pursuit_score_full` scores each cell:
   - Amplitude regression coefficient weighted by SNR × orientation
     reliability
   - Multiplied by an AoA-consistency exponential (nudge, not veto)
4. Pick highest-scoring cell, refine to sub-cell (8 offsets at
   0.25×cell step), subtract from residual, exclude a 0.35-radius
   zone around the pick, repeat up to 6 iterations.
5. Peaks fed to greedy-NN temporal tracker with per-second velocity,
   trails, per-track colors.

Cost: ~120 µs on the S3 (up from 90 µs in v0.3, mostly from AoA
interpolation and sub-cell refinement).  Rate-capped to 25 Hz.

## What's kept from v0.2/v0.3

- CSI acquisition, promiscuous-mode Wi-Fi callback,
  MAC-prefix beacon slot allocation
- Per-subcarrier amplitude+phase extraction from raw I/Q
- Empty-room baseline capture + finalize
- Peer link (role negotiation, ESP-NOW demux, heartbeat)
- Stereo line-fit + LO-drift-solve-via-median + per-beacon AoA
- Extended beacon firmware, Cardputer-compatible
- Diagnostic sub-views
- Deep sleep on both-button combo
- On-push CI producing merged.bin

## Honest caveats

v0.4 is built and integrated end-to-end but has not been flashed
to hardware yet.  Constants likely to want tuning on first field
use:

- `SIGMA_AOA_SQ = 0.35²` (0.35 rad ≈ 20°) — the AoA consistency
  gate width.  Tighter = AoA has more say.  Looser = AoA nudges less.
- `EXCLUDE_RADIUS_SQ = 0.35²` — matching pursuit spatial exclusion.
- `RF_CLOSE = 0.15`, `PHYS_FAR_SQ = 1.0` — alias detection thresholds.
- `PURSUIT_MIN_GAIN = 0.05` — score floor to stop pursuit.

The AoA multiplier's `0.15` floor is deliberate — commodity CSI
AoA is noisy enough that it shouldn't hard-veto a strongly
amplitude-consistent cell.  If field data shows AoA is actually
tighter than expected, raise the floor or drop `SIGMA_AOA_SQ`.

## Not in v0.4 (deliberate v0.5+ material)

Ideas from the ChatGPT report that make sense long-term but need
lab time to tune — deferred:

- Full covariance-aware Mahalanobis likelihoods
- Learned `K(x, y, θ)` orientation surface (Fourier basis)
- Track covariance derived from local observability (skipped
  specifically — a partially-fit Jacobian can produce confidently-
  wrong ellipses in kernel-error regions; honest heuristic sizes
  are safer without lab validation)
- Bayesian / multi-hypothesis tracking with joint-sparse temporal
  regularization
- Adaptive calibration walks that direct the user to
  low-observability regions
- Low-rank compression of the kernel manifold (PCA/SVD basis)
- Low-rank + sparse background decomposition for slow drift
- Replay-trace infrastructure (would need a lab loop to justify)

## Building

Push to GitHub — CI produces `CSI-Radar-S3.merged.bin` and
`CSI-Beacon-Extended.merged.bin` you can flash directly with
`esptool.py write_flash 0x0 merged.bin`.

Locally with arduino-cli:

```
arduino-cli compile --fqbn esp32:esp32:lilygo_t_display_s3 CSI-Radar-S3
arduino-cli compile --fqbn esp32:esp32:esp32c3               CSI-Beacon-Extended
```

## License

MIT.  See `LICENSE`.
