# Statistical methods for the stochastic-integrator efficiency study

*A self-contained explanation, aimed at an undergraduate statistics level.*

This document explains every statistical quantity we compute, the formula behind
it, and **why it is valid**. The running example is the one from the study: a
spacecraft orbit perturbed by random atmospheric density, where we want the
**standard deviation of the final semi-major axis** `a(t_f)` and how cheaply each
integrator can estimate it.

---

## 0. The setup in statistics language

Each simulation "realization" draws one random noise history and returns a single
number, the figure of merit (FoM)

$$ A = a(t_f), $$

which is a **random variable**: run it again with a different random seed and you
get a different `A`. We do not care about one value of `A`; we care about a
**property of its distribution** — chiefly its standard deviation

$$ \sigma = \operatorname{sd}(A) = \sqrt{\operatorname{Var}(A)}, $$

and sometimes its mean `μ = E[A]`. A quantity like `σ` is called an
**estimand** — a fixed (non-random) number we are trying to learn. (We sharpen
*which* `σ` immediately below: the truth `σ_true`, an integrator's converged value
`σ_dt`, or our finite-sample estimate `σ̂` are three different things.)

We cannot compute any of these by brute force alone, so we draw `N` independent
realizations `A_1, …, A_N` and form an **estimator** `σ̂`.

**Three different numbers — keep them separate.** Most confusion in this kind of
study comes from conflating them:

| symbol | what it is | depends on |
|---|---|---|
| `σ_true` | the std of the *exact* (continuous-time) SDE solution | nothing — it is the truth |
| `σ_dt` | the std an integrator produces **with infinite samples** at step `dt` | the integrator **and** `dt` (its *property*) |
| `σ̂`   | the std we actually compute from `N` finite runs | the integrator, `dt`, **and** our sample budget `N` |

So there are two, conceptually different, gaps:

1. **Discretization bias** `b(dt) = σ_dt − σ_true` — a *fixed property of the
   integrator at step `dt`*. It does **not** shrink with more samples; only with
   smaller `dt`. **This is the quantity an accuracy comparison is about.**
2. **Monte-Carlo (sampling) error** `σ̂ − σ_dt` — `σ̂` wobbles only because `N`
   is finite. It is a property of *our experiment*, not of the integrator, and
   shrinks as `1/√N`.

> **What we actually care about (the paper's estimand) is `σ_dt`** — i.e. the
> bias `b(dt)`. When we rank integrators by accuracy, the Monte-Carlo error is
> *not* part of the thing being compared; it is merely the measurement noise on
> our estimate of it, which we report as a confidence interval (a whisker), never
> add into the value. Section 3 estimates `b(dt)` directly; Section 5 explains
> the *separate* practical question (how many samples to budget) where the two
> gaps do get combined — but that combination answers "how expensive is it to
> measure", not "how accurate is the integrator". Do not confuse the two plots:
> the **bias-vs-cost** figure (`*_biasPareto.svg`) is the accuracy comparison;
> the **total-error-vs-cost** figure (`*_pareto.svg`) is the sampling-budget view.

---

## 1. Point estimators: sample mean and sample standard deviation

Given samples `A_1, …, A_N`:

$$ \hat\mu = \frac{1}{N}\sum_{i=1}^N A_i,
\qquad
\hat\sigma = \sqrt{\frac{1}{N-1}\sum_{i=1}^N (A_i-\hat\mu)^2}. $$

The `N−1` (not `N`) is **Bessel's correction**; it makes the sample *variance* an
unbiased estimator of `Var(A)`. In code this is `np.std(..., ddof=1)`
(`ddof` = "delta degrees of freedom" = the 1 we subtract).

> **Caveat that drives everything below.** Even though the sample *variance* is
> unbiased, the sample *standard deviation* `σ̂` (its square root) is slightly
> **biased low**, and — more importantly — its sampling distribution is **not
> Gaussian**. For Gaussian data, `(N-1)\hat\sigma^2/\sigma^2` follows a
> chi-squared distribution with `N−1` degrees of freedom, so `σ̂` is skewed. This
> is why we do **not** use the textbook "`±1.96·SE`" normal interval for `σ̂`;
> we use the **bootstrap** instead, which makes no normality assumption.

---

## 2. The bootstrap: standard error and confidence interval

We want the **standard error** `SE(σ̂)` — the standard deviation of the estimator
across hypothetical repetitions of the whole experiment — and a **95% confidence
interval (CI)**. We get both with the **nonparametric bootstrap** (Efron, 1979).

**Idea (the plug-in principle).** We do not know the true distribution of `A`, but
our `N` samples *are* a sample from it. So we treat the observed sample as a
stand-in for the population and "re-run the experiment" by resampling **from our
own data, with replacement**.

**Algorithm** (this is exactly `statsTools.momentCI`):

1. For `b = 1, …, B` (we use `B = 4000`):
   - Draw `N` indices uniformly **with replacement** from `{1, …, N}`.
   - Form the resample `A^{*b}_1, …, A^{*b}_N` and compute the statistic
     `σ̂^{*b}` (the sample std of the resample).
2. The bootstrap distribution is `{σ̂^{*1}, …, σ̂^{*B}}`. From it:
   - **Standard error:** `SE(σ̂) ≈ sd(σ̂^{*1}, …, σ̂^{*B})` (just the spread of
     the bootstrap values).
   - **95% percentile CI:** the 2.5th and 97.5th percentiles of the bootstrap
     values, `[\hat\sigma^{*}_{(2.5\%)},\ \hat\sigma^{*}_{(97.5\%)}]`.

**Why it works (intuition).** Resampling with replacement mimics the randomness of
drawing a fresh sample of size `N` from the population. The variability *between*
resamples approximates the variability *between* real experiments. Crucially, it
reproduces the **skew** of `σ̂`'s distribution automatically, so the CI can be
asymmetric — exactly what a standard deviation needs. It is valid because our
realizations are **i.i.d.** (independent seeds, identical setup), which is the one
assumption the bootstrap leans on.

**Worked example (reference, `N = 600`):**
`σ̂ = 146.2 m`, bootstrap `SE = 4.1 m`, 95% CI `[138, 154] m`.
Read as: "with 600 samples we have pinned the true std to within about ±4 m."

**The 1/√N law.** The standard error of a moment estimate scales like

$$ \mathrm{SE}(N) \;=\; \mathrm{SE}(N_{\text{pilot}})\,
   \sqrt{\frac{N_{\text{pilot}}}{N}}\;\propto\; \frac{1}{\sqrt N}. $$

This is the ordinary Monte-Carlo rate. It is why we only *simulate* a pilot of
`N_pilot` runs and then **predict** the error at any larger `N` analytically (no
need to actually run a million sims to know what a million sims would give). To
halve the MC error you need 4× the samples.

---

## 3. Estimating the discretization bias — *this is the estimand*

This is the section that measures what the paper actually wants: `σ_dt`, the
integrator's converged std at step `dt`, expressed as its gap from the truth. The
Monte-Carlo error of Section 2 enters here only as the *uncertainty* on that gap,
never as part of it.

The **bias** of an integrator at step size `dt` is

$$ b(dt) \;=\; \sigma_{dt} - \sigma_{\text{true}}, $$

where `σ_dt` is what that integrator's `σ̂` would converge to with infinite
samples (its accuracy property), and `σ_true` is the truth. We estimate `σ_true`
with an **independent reference** (a high-accuracy `scipy` integration that shares
none of Basilisk's machinery — see the README). So the natural estimator is

$$ \hat b = \hat\sigma_{\text{config}} - \hat\sigma_{\text{ref}}. $$

But there are **two ways** to compute its uncertainty, and they differ by orders
of magnitude. This is the heart of the analysis.

### 3a. Unpaired estimate (used for the stochastic / SDE arm)

The config and the reference are run with **independent** randomness. Then the two
std estimates are independent, and the variance of their difference adds:

$$ \operatorname{Var}(\hat\sigma_{\text{cfg}} - \hat\sigma_{\text{ref}})
   = \operatorname{Var}(\hat\sigma_{\text{cfg}}) + \operatorname{Var}(\hat\sigma_{\text{ref}}). $$

This is the rule **"independent errors add in quadrature"**:
`SE_diff = √(SE_cfg² + SE_ref²)`. In code (`biasUnpaired`) we bootstrap each side
separately and difference the bootstrap statistics. We report a **signed** value
with a CI, so the interval can **straddle zero** — that is how we say "this bias
is too small to distinguish from zero at this sample size."

**Worked example (SDE W2Ito2, `dt = 5`):** `b̂ ≈ +4 m`, 95% CI `[−8, +16]`.
The interval contains 0, so the bias is *consistent with zero* — undetectable
beneath the Monte-Carlo noise. This unpaired estimator is the one the shipped
analysis uses for **all** SDE methods; for the strong methods (SRA1/SOSRA) it is
conservative — they *can* be CRN-paired (see the callout in §3b), which would
shrink the interval by ~700×.

### 3b. Paired estimate with Common Random Numbers (used for the profile arm)

The profile arm and the reference both integrate the **same** pre-generated noise
path for a given seed (they share the random history exactly — verified
correlation `≈ 1.0000`). This is the variance-reduction trick called **Common
Random Numbers (CRN)**. Now the two estimates are strongly *positively
correlated*, and the general variance-of-a-difference formula has a third term:

$$ \operatorname{Var}(X - Y) = \operatorname{Var}(X) + \operatorname{Var}(Y) - 2\operatorname{Cov}(X,Y). $$

When `X` and `Y` are nearly identical (`Cov(X,Y) ≈ Var(X) ≈ Var(Y)`), the
right-hand side **nearly cancels to zero**. The huge run-to-run spread (driven by
*which* noise path was drawn) is common to both arms and subtracts out, leaving
only the tiny systematic gap due to the time step.

In code (`biasPaired`) we implement this as a **paired (block) bootstrap**: we
resample **seeds**, and for each resampled seed set we recompute *both*
`σ̂_profile` and `σ̂_ref` on the **same** seeds, then difference. Using identical
indices for both arms is what preserves the cancellation.

**Worked example (profile RK4, `dt = 2`):**
- Paired (CRN): `b̂ = −0.034 m`, 95% CI `[−0.070, +0.003]`.
- Unpaired (same data, pairing ignored): `b̂ = −4.9 m`, 95% CI `[−19, +8]`.

The paired CI is about **370× narrower** (0.073 m wide vs 27 m wide). Same
estimand, same samples — the only difference is exploiting the shared randomness. CRN is what lets us resolve a
sub-metre bias that is otherwise invisible.

> **Can the SDE arm use CRN?** It depends on the method, and this is the one
> place the two kinds of SDE integrator diverge sharply:
> - **W2Ito2 (weak): no.** A weak method reproduces the *distribution*, not any
>   specific noise path — W2Ito2 even quantises the Gaussian increment to a
>   discrete three-point jump before using it. There is no continuous shared path
>   to align against the reference, so its bias is stuck with the wide unpaired
>   interval. (Measured pairing correlation ≈ 0.85 even when fed a prescribed
>   same-path increment → only ~2.5× narrower, not worth it.)
> - **SRA1 / SOSRA (strong): yes.** These advance the state with the *continuous*
>   Wiener increments and converge pathwise. Driving them with the **same**
>   Brownian increments as the reference (via
>   `PrescribedGaussianNoiseGenerator`) makes their per-realization FoM track the
>   reference to ~0.1 m out of a ~50–150 m spread (measured correlation
>   ≈ 0.99999), i.e. the same CRN cancellation the profile arm enjoys — a ~700×
>   narrower bias interval at the same sample count.
>
> The shipped `analyzeResults`/`statsTools` still routes **all** SDE methods
> through the unpaired estimator, so today the whisker sizes reflect that choice.
> Adding a strong-method CRN estimator (build a shared-path realization in
> `stochasticDragModel`, route SRA1/SOSRA through `biasPaired`) is a concrete,
> high-value upgrade the new integrator set makes possible.

---

## 4. Margin of error on the runtime

Wall-clock time per realization is itself a noisy measurement (OS scheduling,
caches, turbo clocking). We summarize it with the **median** (robust to the
occasional slow run, unlike the mean) and bootstrap a CI **for the median**
(`wallCI`): resample the per-run times with replacement `B` times, take the median
of each resample, then take percentiles. Total wall time for `N` runs is
`N ×` (per-run median).

These intervals are very tight (e.g. per-run median `0.0129 s`, CI
`[0.0129, 0.0130]`) because timing noise is small relative to the median and `N`
is large.

---

## 5. A *separate* question: total error and the sampling budget

Section 3 gave us the quantity the accuracy comparison is about — the bias
`b(dt) = σ_dt − σ_true`. This section answers a **different, practical** question:
*if I just want a usable estimate `σ̂` and don't care to separate the two gaps,
how close to the truth is it for a given compute budget?* This is the right lens
for **sizing a run**, not for **ranking integrators** — do not read the
total-error curve as an integrator-accuracy statement (that is the bias plot).

For that budgeting question the user cares about **how close `σ̂` is to `σ_true`
in total**. The error of `σ̂` relative to the truth has two independent parts —
the fixed bias `σ_dt − σ_true` and the sampling error `σ̂ − σ_dt` — so the
**mean squared error** is their sum and the total RMS error is (`totalErrorAtN`):

$$ \mathrm{err}(N) \;=\; \sqrt{\,b(dt)^2 \;+\; \mathrm{SE}(N)^2\,}
   \;=\; \sqrt{\,b(dt)^2 \;+\; \mathrm{SE}_{\text{pilot}}^2\,\frac{N_{\text{pilot}}}{N}\,}. $$

This is just the **bias–variance decomposition**: `MSE = bias² + variance`. Note
the bias floor `b(dt)` here is exactly `σ_dt − σ_true` from Section 3 — the same
integrator property; the MC term is the only part that the sample budget moves.
Two regimes fall out of it:

- **Few samples (`N` small):** the `SE(N)` term dominates → error is MC-limited →
  add samples (cheap for the fast arm).
- **Many samples (`N` large):** `SE(N) → 0` and the error floors at `|b(dt)|` →
  the only way down is a smaller `dt` (more cost per sample).

We propagate the **bias CI** through this same formula to get an error *band*
(`errorLo`, `errorHi`). If the bias CI straddles zero, the achievable floor is
taken as zero (the bias might genuinely be negligible).

**The Pareto plot.** For each config we trace `(wall(N), err(N))` as `N` varies
and keep the lower-left envelope per arm: the best accuracy reachable for each
time budget. The whisker version plots each config at its **measured** budget
`N = N_pilot` with 95% bars on both axes (vertical = total-error band, horizontal
= wall CI), so neither coordinate is an extrapolation.

---

## 6. Why the whole comparison is statistically sound

| Estimator | Assumption it relies on | Holds here? |
|---|---|---|
| Sample mean / std | finite second/fourth moments | yes (bounded orbit) |
| Bootstrap CI of a moment | realizations i.i.d. | **yes** — independent seeds, identical config |
| Unpaired bias CI | the two arms' estimates independent | yes — different RNG streams |
| Paired (CRN) bias CI | seeds i.i.d. **and** pairing exact | yes — verified path correlation ≈ 1 |
| Wall median CI | timings i.i.d. given the machine | approximately (same hardware, isolated runs) |
| Total-error formula | bias and MC error independent | yes — different mechanisms |
| Independent reference | reference unbiased for `σ_true` | tight tolerance + cross-checked vs both arms |

**Honest limitations we report rather than hide:**

1. **Unpaired bias has a resolution floor.** The SDE bias cannot be distinguished
   from zero below roughly the combined MC standard error (~10 m at `N ≈ 400`).
   We flag this with `biasConsistentZero` instead of pretending a point value is
   significant. To resolve smaller SDE biases you must raise `N` (the CI shrinks
   as `1/√N`) — a cluster-scale job.
2. **The bootstrap is asymptotic.** Percentile intervals are approximate at very
   small `N`; with `N` in the hundreds-to-thousands here, the approximation is
   good. (One could use BCa intervals for a small-sample correction; the
   percentile method suffices at our `N`.)
3. **The conclusion is figure-of-merit specific.** Everything above measures a
   *moment* (a distributional property). For a *pathwise* goal (a specific
   trajectory), the weak SDE methods would not even be valid, and only the strong
   profile arm would qualify.
4. **Multiple comparisons.** We compare many configs; individual 95% CIs are
   marginal, not simultaneous. We use them descriptively (to read off where each
   arm sits), not to make a single family-wise significance claim, so no
   Bonferroni-type correction is applied.

---

## 7. Symbol glossary

| Symbol | Meaning |
|---|---|
| `A = a(t_f)` | figure of merit (final semi-major axis), a random variable |
| `σ_true` | std of the exact continuous-time SDE solution (the ultimate truth) |
| `σ_dt` | std an integrator converges to at step `dt` with `N=∞` — **the estimand of the accuracy comparison** |
| `σ̂` | sample std from `N` finite runs (estimates `σ_dt`, with MC error) |
| `μ, σ` | shorthand for the mean / `σ_dt` when the `dt` context is fixed |
| `μ̂, σ̂` | sample estimators |
| `N`, `N_pilot` | number of realizations; the pilot count actually simulated |
| `B` | number of bootstrap resamples (4000) |
| `SE` | standard error = std of an estimator across experiments |
| `CI` | confidence interval (here: bootstrap percentile, 95%) |
| `b(dt)` | discretization bias of a config at step size `dt` |
| `dt` | integrator time step |
| CRN | Common Random Numbers (paired variance reduction) |
| MSE | mean squared error = `bias² + variance` |

---

### One-paragraph summary

The quantity we are comparing is each integrator's **converged** std `σ_dt` — i.e.
its discretization bias `σ_dt − σ_true` against an independent reference — *not*
the finite-sample noise of any one run. Each integrator produces noisy estimates
of `σ_dt`; we quantify that *sampling* uncertainty with a nonparametric bootstrap
(valid because runs are i.i.d., and necessary because a standard deviation is
non-Gaussian) and carry it as a confidence interval on the bias, never folded into
the bias itself. For the strong/profile arm we share the random path with the
reference (Common Random Numbers), which cancels the dominant variance and resolves
a sub-metre bias; the weak SDE arm cannot be paired, so its bias is only resolvable
when large. The accuracy comparison is therefore the **bias-vs-cost** front (bias
`b(dt)` on the y-axis, per-run wall on the x-axis, bootstrap CIs as whiskers).
*Separately*, for sizing a run, the bias and sampling error combine by the
bias–variance decomposition into a total-error band traded off against wall-time —
but that is a budgeting aid, not the accuracy statement.
