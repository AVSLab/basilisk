#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
r"""
Statistical estimators for the stochastic efficiency study.

Everything here is *resampling-based* (bootstrap), because the figure of merit
is a random variable whose **standard deviation** is the headline statistic, and
the sampling distribution of a sample standard deviation is non-Gaussian at the
sample sizes we can afford. Bootstrap percentile intervals make no normality
assumption and apply uniformly to the std, the mean, biases, and speedups.

Three kinds of uncertainty are quantified, all reported as 95% intervals:

1. **Monte-Carlo margin of error on the moment estimate** (``momentCI``):
   how well N realizations pin down std[FoM] (or mean[FoM]).  Bootstrap: resample
   the N samples with replacement B times, recompute the moment, take percentiles.

2. **Discretization bias of the moment** (``biasPaired`` / ``biasUnpaired``):
   how far a config's moment sits from the truth.
   * Profile arm — *common random numbers* (CRN): the profile config and the
     scipy reference integrate the **same** OU path for each seed (verified
     correlation ≈ 1).  We resample *seeds* (paired) and recompute
     ``moment(config) - moment(reference)`` on the shared seeds, so the huge
     path-to-path variance cancels and a sub-metre bias is resolvable.
   * SDE arm — no CRN is possible (the integrator draws its Wiener increments
     internally), so the bias is the unpaired difference of the two moment
     estimates, with the two MC errors added in quadrature.

3. **Runtime margin of error** (``wallCI``): the per-realization wall time is
   itself a noisy measurement (OS scheduling, cache, etc.).  We report the median
   and a bootstrap interval for the median; total wall for N samples is N x that.

A note on *statistical meaningfulness*: an interval is only meaningful if the
estimator is (near) unbiased and the resamples are i.i.d.  The realizations
within a config are i.i.d. by construction (independent seeds), so the moment
and wall bootstraps are sound.  The CRN bias bootstrap is sound because seeds are
i.i.d. and pairing is exact.  The one caveat we flag explicitly: the unpaired SDE
bias cannot resolve a true bias below ~the combined MC standard error, so small
SDE biases are reported as "consistent with zero" rather than as a point value.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict, Optional

import numpy as np


def _momentFn(moment: str):
    if moment == "mean":
        return lambda a: np.mean(a)
    elif moment == "std":
        return lambda a: np.std(a, ddof=1)
    raise ValueError(f"Unknown moment '{moment}'")


@dataclass
class Interval:
    """A point estimate with a (percentile) confidence interval and its SE."""
    value: float
    lo: float
    hi: float
    se: float

    def asdict(self):
        return asdict(self)


def momentEstimate(samples: np.ndarray, moment: str) -> float:
    """Point estimate of the requested moment."""
    return float(_momentFn(moment)(np.asarray(samples, dtype=float)))


def momentCI(samples: np.ndarray, moment: str, nBoot: int = 4000,
             alpha: float = 0.05, seed: int = 12345) -> Interval:
    """Bootstrap point estimate, SE, and (1-alpha) percentile CI of the moment.

    This is the Monte-Carlo margin of error: the uncertainty in std[FoM]
    (or mean[FoM]) that comes from having only ``len(samples)`` realizations.
    """
    samples = np.asarray(samples, dtype=float)
    n = samples.size
    point = momentEstimate(samples, moment)
    if n < 2:
        return Interval(point, float("nan"), float("nan"), float("inf"))
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, n, size=(nBoot, n))
    stats = _bootStats(samples, idx, moment)
    lo, hi = np.percentile(stats, [100 * alpha / 2, 100 * (1 - alpha / 2)])
    return Interval(point, float(lo), float(hi), float(np.std(stats, ddof=1)))


def _bootStats(samples: np.ndarray, idx: np.ndarray, moment: str) -> np.ndarray:
    """Vectorized bootstrap statistic over resample-index rows."""
    resampled = samples[idx]            # (nBoot, n)
    if moment == "mean":
        return resampled.mean(axis=1)
    return resampled.std(axis=1, ddof=1)


def biasUnpaired(samples: np.ndarray, refSamples: np.ndarray, moment: str,
                 nBoot: int = 4000, alpha: float = 0.05,
                 seed: int = 23456) -> Interval:
    """Bias of a config's moment vs the reference, with NO common random numbers.

    Used for the SDE arm.  The bias point estimate is ``moment(config) -
    moment(ref)``; its CI comes from independently bootstrapping both samples and
    differencing (their MC errors add in quadrature).  A signed value is
    returned (not absolute) so the CI can straddle zero, which is exactly how we
    detect "bias consistent with zero".
    """
    samples = np.asarray(samples, dtype=float)
    refSamples = np.asarray(refSamples, dtype=float)
    fn = _momentFn(moment)
    point = float(fn(samples) - fn(refSamples))
    rng = np.random.default_rng(seed)
    bi = _bootStats(samples, rng.integers(0, samples.size, (nBoot, samples.size)), moment)
    br = _bootStats(refSamples, rng.integers(0, refSamples.size, (nBoot, refSamples.size)), moment)
    diffs = bi - br
    lo, hi = np.percentile(diffs, [100 * alpha / 2, 100 * (1 - alpha / 2)])
    return Interval(point, float(lo), float(hi), float(np.std(diffs, ddof=1)))


def biasPaired(samples: np.ndarray, seeds: np.ndarray,
               refSamples: np.ndarray, refSeeds: np.ndarray, moment: str,
               nBoot: int = 4000, alpha: float = 0.05,
               seed: int = 34567) -> Optional[Interval]:
    """Bias of a config's moment vs the reference using common random numbers.

    Used for the profile arm, which integrates the *same* OU path as the scipy
    reference for each seed.  We align the two records by seed, then bootstrap
    over the shared seeds (a paired/blocked bootstrap): for each resample we
    recompute ``moment(config) - moment(ref)`` on the SAME resampled seed set.
    Because the dominant path-to-path variation is common to both arms it
    cancels, so the bias of the moment is resolved with far smaller variance than
    the unpaired estimate.

    Returns ``None`` if there are too few shared seeds to pair.
    """
    seeds = np.asarray(seeds)
    refSeeds = np.asarray(refSeeds)
    samples = np.asarray(samples, dtype=float)
    refSamples = np.asarray(refSamples, dtype=float)

    refPos = {int(s): i for i, s in enumerate(refSeeds)}
    common = [s for s in seeds if int(s) in refPos]
    if len(common) < 8:
        return None
    cfgPos = {int(s): i for i, s in enumerate(seeds)}
    a = np.array([samples[cfgPos[int(s)]] for s in common])
    r = np.array([refSamples[refPos[int(s)]] for s in common])

    fn = _momentFn(moment)
    point = float(fn(a) - fn(r))
    rng = np.random.default_rng(seed)
    m = a.size
    idx = rng.integers(0, m, size=(nBoot, m))       # SAME indices for a and r
    diffs = _bootStats(a, idx, moment) - _bootStats(r, idx, moment)
    lo, hi = np.percentile(diffs, [100 * alpha / 2, 100 * (1 - alpha / 2)])
    return Interval(point, float(lo), float(hi), float(np.std(diffs, ddof=1)))


def wallCI(wallSamples: np.ndarray, nBoot: int = 4000, alpha: float = 0.05,
           seed: int = 45678) -> Interval:
    """Median per-realization wall time with a bootstrap CI for the median.

    Wall time is a noisy measurement; the median is robust to occasional OS
    hiccups.  Total wall for N realizations is N x this median.
    """
    wallSamples = np.asarray(wallSamples, dtype=float)
    n = wallSamples.size
    point = float(np.median(wallSamples))
    if n < 2:
        return Interval(point, float("nan"), float("nan"), float("inf"))
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, n, size=(nBoot, n))
    meds = np.median(wallSamples[idx], axis=1)
    lo, hi = np.percentile(meds, [100 * alpha / 2, 100 * (1 - alpha / 2)])
    return Interval(point, float(lo), float(hi), float(np.std(meds, ddof=1)))


def totalErrorAtN(biasInterval: Interval, momentSEPilot: float, nPilot: int,
                  N: int) -> Dict[str, float]:
    r"""Total RMS error of the moment estimate at sample count N, with a band.

    Combines the (fixed) discretization bias and the (shrinking) Monte-Carlo
    error:

        err(N) = sqrt( bias^2 + se_pilot^2 * nPilot / N )

    The band propagates the bias CI through the same formula.  We use
    ``|bias|`` clamped at zero for the point estimate and the CI endpoints, so a
    bias consistent with zero yields a lower band that is essentially the pure
    MC term.
    """
    seN = momentSEPilot * np.sqrt(nPilot / N)
    def comb(b):
        return float(np.sqrt(max(b, 0.0) ** 2 + seN ** 2))
    # Use the larger-magnitude CI endpoint for the upper error band and the
    # smaller (possibly zero) for the lower band.
    bAbs = abs(biasInterval.value)
    bLo = min(abs(biasInterval.lo), abs(biasInterval.hi))
    bHi = max(abs(biasInterval.lo), abs(biasInterval.hi))
    # If the CI straddles zero, the achievable bias floor is zero.
    if biasInterval.lo <= 0.0 <= biasInterval.hi:
        bLo = 0.0
    return {"error": comb(bAbs), "errorLo": comb(bLo), "errorHi": comb(bHi),
            "seN": float(seN)}
