# Stochastic-integrator efficiency study

Does using a **stochastic integrator** to propagate noisy flight dynamics
recover the moments (mean, standard deviation) of a figure of merit *more
cheaply* than the common alternative — **pre-generating a noise profile and
replaying it through a fine-step deterministic integrator**?

This study answers that as a statistical accuracy-vs-wall-time integrator
comparison, based on `examples/mujoco/scenarioStochasticDrag.py`: a 250 km
cannonball orbit decaying under drag, where the atmospheric density carries an
Ornstein–Uhlenbeck (OU) correction. The figure of merit (FoM) is the **final
semi-major axis** `a(tf)`; the statistic of interest is its **standard
deviation** across realizations (the part the noise drives — the mean is
dominated by deterministic drag).

> **Running on CU Boulder Alpine (HPC):** see [`slurm/README_ALPINE.md`](slurm/README_ALPINE.md).
> The whole study is embarrassingly parallel; `slurm/run_all.sh` submits the
> reference job + a sharded grid job array + a finalize job with the right
> dependencies. Sample caching is shard-aware (`sampleStore.py`) so array tasks
> never contend and a half-finished sweep still analyzes.

## The two arms

| Arm | What it does | Cost characteristic |
|-----|--------------|---------------------|
| `sde` | Stochastic integrator (SRA1, SOSRA, W2Ito2) evolves the OU density state **inline**. One run = one realization. | Large `dt` allowed (order 1.5–2) → cheap per sample |
| `profile` | OU path generated **in advance** (exact sampling), replayed by `ProfileAtmDensity` and integrated with a deterministic RK (RK2/RK4). | Needs small `dt` to resolve the rough forcing → many stage evaluations |

The three SDE methods are (see `SDE_METHODS` in `stochasticDragModel.py`):

* **SRA1** and **SOSRA** — Roessler stochastic Runge–Kutta methods for
  **additive** noise, **strong order 1.5 / weak order 2**. They advance the OU
  state with the *continuous* Wiener increments `dW`, `dZ`, so they track a
  Brownian path (strong sense). SOSRA is the stability-optimised variant of the
  same order. These are valid here because the OU density correction is additive
  (its diffusion `σ_st·√(2/τ)` does not depend on the state).
* **W2Ito2** — Tang & Xiao **weak-order-2** method for general Itô noise. It
  drives the step with discrete three-point random variables
  (`Iₖ ∈ {±√(3h), 0}`, quantised from `dW`), so it reproduces the *moments* of
  the SDE solution without tracking any individual Brownian path.

### Fair-speed requirement: the profile replay module is a Numba module

The stochastic arm's density correction (`StochasticAtmDensity`) is compiled
C++. To compare wall times fairly, the profile arm's replay module
(`ProfileAtmDensity`) is a [`NumbaModel`](../../../docs/source/Learn/makingModules/numbaModules.rst):
its `UpdateStateImpl` JIT-compiles once (at `Reset`, outside the timed region)
to a C-callable the scheduler invokes with **zero per-tick Python overhead**.
The two arms then differ in wall time only because of their *algorithms*, not
the implementation language. A pure-Python version is retained as
`ProfileAtmDensityPy` for cross-checking (bit-identical results) but **must not
be used for timing** — measured ~2.7× slower purely from interpreter overhead,
which would unfairly inflate the stochastic arm's apparent speedup.

**The profile arm is also charged for generating its OU path.** The SDE arm
draws its noise *inline*, inside the timed `ExecuteSimulation`; the profile arm
must pre-generate a full OU path per realization, a cost it would otherwise dodge
if only integration were timed. So `runRealization` times the OU-path generation
and adds it to the profile arm's reported wall (the SDE arm's stays untouched).
To keep that charge fair, the generation's sequential recurrence is
**numba-compiled** (`_ouRecurrence` in `stochasticDragModel.py`) — a production
noise generator would be compiled, just as the SDE arm's is C++ — and the JIT is
warmed up once outside the timed region so no realization pays the compile cost.
The compiled charge is small (sub-millisecond for a ~20 k-node path); timing the
*interpreted* loop instead would over-penalise the profile arm ~100×. **numba is
required** for any timed profile-arm run: if it is unavailable, `_ouRecurrence`
raises rather than silently fall back to the slow interpreted loop (which would
corrupt the comparison). The numpy `default_rng(seed)` draws are unchanged, so
the generated path stays bit-identical and shareable with the reference for CRN.

A third arm, `reference`, is an **independent scipy `solve_ivp`** integration of
the same planar problem with the same prescribed OU path. It shares none of
Basilisk's integrators or scheduling, so it is the unbiased ground truth both
arms are measured against (no circularity).

## Scripts

| Script | Role |
|--------|------|
| `stochasticDragModel.py` | Builds & runs one realization of either Basilisk arm; OU path generator; `ProfileAtmDensity`. |
| `referenceScipy.py` | Independent scipy ground-truth realization. |
| `runComparison.py` | Monte-Carlo sampler. **Caches every FoM sample to disk**; resumable; one file per config → embarrassingly parallel on a cluster. |
| `statsTools.py` | Bootstrap estimators: moment CI, paired-CRN / unpaired bias, wall-time CI, total-error band. Shared by analysis and plotting. |
| `analyzeResults.py` | Semi-analytic accuracy-vs-cost with full uncertainty: CRN bias for the profile arm, unpaired bias for the SDE arm, MC error ∝ 1/√N. Writes `results/pareto_<moment>.json`. |
| `plotResults.py` | Plots from JSON/cache only — never simulates. |

## Typical workflow

```bash
PY=../../../.venv/bin/python          # Basilisk's venv (MuJoCo built there)

# 1. Ground truth (run once; more samples = tighter truth)
$PY runComparison.py --reference --nSamples 4000

# 2. The competing configurations (resumable; re-run to extend)
$PY runComparison.py --nSamples 500

# 3. Analyze + plot (cheap, no simulation)
$PY analyzeResults.py --moment std
$PY plotResults.py --moment std
```

`--list` prints the config grid and cache hashes. `--only <key>` runs a single
config (e.g. `--only sde:SRA1:dt=20`) — ideal for a cluster task array, since
each config writes its own `results/samples/<hash>.npz` and the manifest update
is atomic. Re-invoking with a larger `--nSamples` only simulates the missing
realizations (seed = `baseSeed + i`, so the sequence continues).

Scenario knobs: `--orbits` (propagation length), `--stationaryStd` (noise
strength). These are part of the cache hash, so different settings never
collide.

## Caching & resume

* `results/samples/<hash>.npz` — FoM samples, per-run wall times, step counts,
  seeds. Written atomically (`tmp`+rename) every `--saveEvery` samples, so an
  interrupted run loses nothing.
* `results/manifest.json` — index of every cached config.
* `results/pareto_<moment>.json` — analysis output consumed by the plotter.

## Weak vs strong convergence (why the comparison is set up this way)

The SDE arm now mixes **two kinds** of stochastic integrator, and the
distinction matters throughout:

* **SRA1 / SOSRA are strong-order-1.5 methods.** They advance the OU state with
  the *continuous* Wiener increments and converge *pathwise* — given a Brownian
  path, they approximate that specific realization, not just its distribution.
* **W2Ito2 is a weak-order-2 method.** It reproduces the *moments / distribution*
  of the SDE solution and drives the step with discrete three-point random
  variables (`Iₖ ∈ {±√(3h), 0}`, quantised from the Gaussian increment), so it
  tracks no individual Brownian path.

Because **the figure of merit is a moment** (the standard deviation of the final
semi-major axis, a distributional functional), weak convergence is already the
*relevant* notion of accuracy: weak order 2 means the error in `E[g(a)]` shrinks
like `O(h²)`, and both strong and weak methods here are weak order 2 or better.
The strong methods simply give *more* — pathwise tracking — which turns out to
matter for how cheaply their bias can be measured (next point).

This explains the study's design choices:

1. **Common random numbers (CRN) — available for the strong methods, not the
   weak one.** CRN pairs the profile arm with the reference because both
   integrate the *same* OU path, so the huge path-to-path variance cancels and a
   sub-metre bias is resolvable at only ~3000 samples.
   - **SRA1/SOSRA** track a path, so they *can* be paired the same way: driven by
     the same Brownian increments as the reference, their per-realization FoM
     tracks it to ~0.1 m out of a ~50–150 m spread (measured correlation
     ≈ 0.99999, ≈700× smaller bias-estimate variance than unpaired). See
     `PrescribedGaussianNoiseGenerator` in
     `src/simulation/dynamics/_GeneralModuleFiles/stochasticNoiseGenerator.h`,
     which lets a caller feed the integrator a prescribed increment sequence.
     **This is a genuine option to slash the SDE arm's bias-resolution cost**;
     the shipped analysis still routes the SDE arm through the *unpaired*
     estimator (see below), so exploiting it is a concrete next step, not yet
     wired into `statsTools`/`analyzeResults`.
   - **W2Ito2** quantises `dW` to a three-point RV before using it, so even a
     prescribed same-path increment is collapsed and the pairing correlation
     drops to ≈ 0.85 (≈2.5× reduction only). For the weak method there is
     genuinely no shared path to pair against, so its bias remains an *unpaired*
     moment difference with the wider CI that implies — a property of the weak
     scheme, not of the implementation.

2. **Why the stochastic arm can be cheaper.** A large-`dt` SDE step still
   captures the distribution (weak order 2) or the path (strong order 1.5) of the
   forcing, whereas the profile arm must resolve the piecewise-linear replayed
   path with a *small* `dt` (many stage evaluations). The win is "less work per
   realization for the same accuracy", not "fewer realizations".

3. **Why this result is figure-of-merit-specific.** For a *pathwise* quantity (an
   actual trajectory, a worst-case excursion, a collision check against a
   specific realization) the weak method (W2Ito2) would be inappropriate; the
   strong SDE methods (SRA1/SOSRA) and the profile arm would qualify. The
   efficiency conclusion here is stated for *moment* estimation.

Note on the measured order: for the strong methods the std bias should fall as a
high power of `dt`, but on the coarse grid the bias for `dt ≤ 5–10` already sits
at the Monte-Carlo noise floor of the *unpaired* estimator, so the order cannot
be cleanly extracted from an unpaired sweep. CRN-pairing the strong methods (per
point 1) would resolve the sub-floor biases and make a clean order fit possible —
a good dedicated cluster task, out of scope for the efficiency Pareto itself.

## Statistical estimators and margins of error

All uncertainty is quantified by **bootstrap** (in `statsTools.py`), because the
headline statistic is a sample *standard deviation*, whose sampling distribution
is non-Gaussian at affordable N. Everything is reported as a 95% percentile
interval:

| Quantity | Estimator | Notes |
|----------|-----------|-------|
| Monte-Carlo margin on std[FoM] | resample the N realizations, recompute std, take percentiles | this is the irreducible "how many samples" error; shrinks as 1/√N |
| Profile-arm discretization bias | **paired (CRN) bootstrap** of `std(profile) − std(ref)` over shared seeds | profile & reference integrate the *same* OU path (corr ≈ 1.000), so path variance cancels → sub-metre bias resolved |
| SDE-arm discretization bias | **unpaired** bootstrap difference (MC errors add in quadrature) | the SDE draws noise internally, so no CRN; small biases only show as "consistent with zero" |
| Per-run wall time | bootstrap CI of the **median** per-realization wall | median is robust to OS scheduling jitter |
| Total error at N | `√(bias² + se_pilot²·N_pilot/N)`, bias CI propagated through | combines fixed bias and shrinking MC error into a band |

**Why CRN matters (concrete):** for `profile:RK4:dt=2` the bias is −0.03 m with a
95% CI of about [−0.07, +0.003] m (paired); the *same* quantity estimated without
CRN is −4.9 m with CI [−19, +8] m — a ~360× wider interval, dominated by
path-to-path variance. CRN is what makes the profile arm's near-zero bias
floor a *measured* fact rather than an unresolved guess.

**Are the intervals statistically meaningful?** Yes, with one honest caveat:
the realizations within a config are i.i.d. (independent seeds), so the moment,
wall, and CRN-bias bootstraps are all sound. The *unpaired* SDE bias, however,
cannot resolve a true bias below ≈ the combined MC standard error (~10 m at
N≈400); below that the CI straddles zero and we report `biasConsistentZero`
rather than a point value. Each config's JSON therefore carries `momentLo/Hi`,
`biasLo/Hi`, `biasMethod`, `biasConsistentZero`, and `perRunWallLo/Hi`.

## Figures

`plotResults.py` writes the SVGs below (all from cached JSON, no simulation).

- `*_biasPareto.svg` — **the paper figure.** Integrator *discretization bias*
  (y) vs *per-realization* wall time (x). Both axes are intrinsic to
  (integrator, dt) and **exclude the finite-sample Monte-Carlo noise**, which is
  a property of our experiment, not of the integrator. The MC noise appears only
  as the whisker (uncertainty) on the bias. Configs whose bias is statistically
  indistinguishable from zero at the sample size run are drawn as **hollow
  markers with a downward arrow** (an upper bound, "bias no larger than this"),
  not as a misleading point value. Use this when the question is "how biased is
  each integrator, and what does one run cost?"
- `*_pareto.svg` — the semi-analytic *total-error*-vs-wall curves + per-arm
  frontier. Here the y-axis is `√(bias² + MC²)`, i.e. it **includes** the
  sampling error and depends on the sample budget `N`. Use this for the
  practical "how much wall time to estimate the moment to within X" question,
  **not** for the intrinsic-bias claim.
- `*_paretoWhisker.svg` — **each config at its measured budget (N = nPilot) with
  95% error bars on both axes**: horizontal = wall-time CI, vertical = total-error
  band (bias ⊕ MC). This is the statistically honest view, since both
  coordinates are measured rather than extrapolated. Coloured by method so the
  strong additive methods (SRA1/SOSRA) are distinct from the weak-order-2 W2Ito2.
- `*_bias.svg` — signed discretization bias vs dt with 95% CI whiskers on a
  symlog axis (profile CIs are tight via CRN; SDE CIs are wide and often cross
  zero).
- `*_speedup.svg` — speedup of the stochastic arm over the profile arm vs the
  accuracy target.

## How much compute to characterise each integrator's bias

For an accuracy *comparison* the right bar is to know each integrator's
discretization bias to a fixed **relative** precision (default 10%), so the
integrators can be ranked. `analyzeResults.py --biasCost`-style output (printed
by default, also in the `biasResolutionCost` JSON block) reports, per config, the
samples and wall time to reach it:

```
analyzeResults.py --moment std --relTarget 0.10 --negligibleFrac 0.005 --cores 32
```

`N_req = N_pilot · (SE_bias / (relTarget·|bias|))²`, `time = N_req · wall/run`,
divided by `--cores`. Because `N_req` blows up as `(SE/|bias|)²` when the true
bias shrinks, demanding 10% of an arbitrarily small bias is both impossible and
pointless (a bias far below the truth changes no ranking). So we use a **mixed
target**: resolve to 10% relative where `|bias| > negligibleFrac·truth` (default
0.5% of the reference std); otherwise only *upper-bound* the bias at that
negligibility floor — a tight, honest "bias < X" statement that is **cheaper**
than over-resolving a negligible number, and a stronger accuracy claim than a
noisy point value.

Representative result (with the *unpaired* SDE estimator the shipped analysis
uses): the **profile arm is essentially free** (~seconds total — its bias SE is
sub-metre via CRN), while the **SDE arm costs core-days**, dominated by configs
whose small bias must be pinned against the σ≈80–150 m unpaired floor. 10%
relative accuracy on the larger-bias configs is a few cluster-hours; sub-0.5%-of-
truth biases are reported as tight upper bounds rather than over-resolved.

Two levers reduce the SDE cost, both floored today by the *unpaired* estimator:
a larger reference `N` (amortised once across all SDE configs), and — new with
this integrator set — **CRN-pairing the strong methods (SRA1/SOSRA)** via a
prescribed same-path increment sequence, which the prototype shows cuts their
bias-estimate variance ~700× (see "Weak vs strong convergence"). Wiring the
strong-method CRN estimator into `statsTools`/`analyzeResults` would move
SRA1/SOSRA from core-days to the same ~seconds regime as the profile arm; it is
not yet implemented. W2Ito2 (weak) cannot be paired and stays unpaired.

## How the Pareto front is built (numerically efficient)

We only ever *simulate* a pilot number of realizations per config. For each, the
total error in the estimated moment is decomposed as

    err(N) = sqrt( bias(dt)^2 + se_pilot^2 * N_pilot / N )

— a fixed discretization **bias** (distance from the scipy reference moment) plus
a **Monte-Carlo** term that shrinks as 1/√N (bootstrap-estimated at the pilot
size). Sweeping `N` traces each config's accuracy-vs-wall-time curve without
simulating every point; the per-arm Pareto frontier is the lower-left envelope.

## Validation notes (important)

These were established while building the study and are baked into the defaults:

1. **The base `exponentialAtmosphere` must run in the scene's dynamics task**, not
   the outer task. In the outer task the density is only refreshed once per macro
   step, so the profile arm injected fresh stage-time noise onto a stale base
   density and inflated `std(a)` by ~2.6×. With the fix, both arms and the scipy
   reference agree (`std(a)≈150 m` at `orbits=2, stationaryStd=0.3`).
2. **The OU profile uses a fixed fine master grid** (`profileGridDt`, default
   0.25 s) independent of `dt`, so refining `dt` is a genuine integrator
   convergence study, not a moving noise target.
3. **The SDE methods must have a high-order deterministic core to be viable at
   large `dt`.** A first-order-drift scheme (e.g. plain Euler–Maruyama, which
   this study no longer includes) gets even the *deterministic* orbit wrong by
   hundreds of km at `dt=2 s`. SRA1/SOSRA/W2Ito2 embed high-order deterministic
   Runge–Kutta stages, so they are accurate on the deterministic orbit at large
   `dt` and match the truth to MC noise. (Observed exception at the extreme: at
   `dt=40 s` SOSRA can leave its stability region for this problem and the FoM
   blows up — a large-`dt` stability limit, not a bias floor.)
4. Adaptive deterministic RK (RKF45/RKF78) is excluded from the default profile
   grid: its embedded error controller misbehaves on the piecewise-linear
   (kinked) replayed forcing.

All runs use Basilisk's venv (`.venv/bin/python`); keep a Python reference to the
integrator alive for the lifetime of a sim (Basilisk GC quirk) — `runRealization`
already does this.
