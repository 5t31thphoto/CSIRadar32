# MantisSec CSI-Radar-S3 (v0.6)

Wi-Fi CSI **scene reconstruction** engine for the LilyGo T-Display-S3.
Two receivers, three beacons, a calibration walk — and a live top-down
map of who's in the room.



## What it is

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


## License

MIT.  See `LICENSE`.
