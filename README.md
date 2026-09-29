# MantisSec CSI Radar (v1.1)

Wi-Fi CSI **scene reconstruction** on ESP32: a T-Display-S3 anchor, a
self-organising mesh of smart beacons, and hand-held probes (M5Stack
Core2, Cardputer ADV), all built from one repo and flashed from the
browser.

please start here: https://5t31thphoto.github.io/CSIRadar32/
Please get your hardware from my affiliate links on that site too!

## The system, and how it scales

Every piece works with whatever else is present, and says what the next
piece of hardware would add.

| Deployment | What you get |
|---|---|
| anchor + 1 beacon | tripwire / link presence |
| anchor + 2 beacons | presence on two links; mesh presence |
| anchor + 3+ beacons | **mesh contacts on the radar before any calibration** (beacon-to-beacon tomography), surveyed beacon layout |
| + calibration walk | anchor tracks from the learned inverse model, **corroborated** by the mesh (double lime ring = both instruments agree) |
| + 4-6 beacons | witness voting (ghost rejection), beacon-moved integrity check |
| + a hand-held probe | the anchor's screen and buttons in your hand, alarms, its own mesh map with the anchor's tracks overlaid |
| + a second T-Display | stereo AoA |

**Beacons** (`CSI-Beacon-Mantis/`, one firmware per chip) claim the lowest
free id (1..6) and persist it, elect the lowest id as timekeeper, share a
33 ms / 8-slot TDMA frame, measure every beacon-to-beacon link, and
publish perspectives, dense link detail and - from the timekeeper - a
surveyed layout solved from the full pairwise RSSI matrix. Each box has
a unique MAC (`1A:00:uid:uid:00:id`), so an id collision is heard and
resolved by uid in one exchange.

**The anchor** (`CSI-Radar-S3.ino` + `*.cpp`) runs its calibrated scene
solver and the Rust RF chart as before, and now also hosts the same
`mantis_receiver.h` the probes run (`mesh_anchor.cpp`). The mesh frame is
aligned onto the radar through the beacons both frames know
(`mantis_align.h`), so mesh contacts and tomography glow are drawn in the
right place; dashboard view **MESH** shows beacons, survey, alignment
and contacts.

**Probes** (`core2/`, `cardputer/`) share `mantis_probe_app.h`: one alarm
policy, one map renderer, one anchor remote. The anchor broadcasts its
state, what its screen is asking for and what its two buttons do; a probe
press is delivered through the anchor's own input path with a sequence
number and shown as landed only when the anchor echoes it. Nothing about
the anchor's state is guessed on the probe.

## Building and checking

    ./tools/preflight.sh

compiles every firmware and every anchor translation unit against
vendor-header shims, host-links the anchor, checks the probe/anchor wire
contract, and runs two simulations:

* `tools/mesh_sim.cpp` - 1..6 blank beacons booting together (and two
  boxes with the same stored id): ids, timekeeping, survey accuracy and
  person detection through the real receiver.
* `tools/link_sim.cpp` - remote key presses under 30% loss both ways:
  every press lands exactly once and confirmation is honest.

CI (`.github/workflows/build.yml`) runs pre-flight, then builds the anchor
(Rust core built with `-Z build-std=core`), Core2, Cardputer ADV and both
beacon chips, and publishes the web flasher. Zip drops: put the workflow
under `github/workflows` (no dot) in the zip and rename it after the drop
lands; `tools/retired.txt` lists files a drop deletes.

## What it is

An **empirical inverse sensor model**, 

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


## License

MIT.  See `LICENSE`.
