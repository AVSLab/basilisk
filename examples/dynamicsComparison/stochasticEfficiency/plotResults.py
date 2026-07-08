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
Plot the results of the stochastic efficiency study from cached JSON.

This script *only* reads ``results/pareto_<moment>.json`` (written by
``analyzeResults.py``) and the per-config sample caches; it never runs a
simulation.  That separation means plots can be regenerated instantly and the
expensive sampling can live entirely on a cluster.

It produces:

* ``stochasticEfficiency_pareto.svg`` -- the headline accuracy-vs-wall-time
  scatter of every config's err(N)/wall(N) curve, coloured by arm, with the
  per-arm Pareto frontiers overlaid.  This is the figure that proves or
  disproves the efficiency claim.
* ``stochasticEfficiency_bias.svg`` -- discretization bias of the targeted
  moment vs step size, per method (shows each method's convergence order).
* ``stochasticEfficiency_speedup.svg`` -- speedup factor of the stochastic arm
  over the profile arm as a function of the accuracy target.
"""
from __future__ import annotations

import argparse
import json
import os
from typing import Optional

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(_HERE, "results")

ARM_COLORS = {"sde": "#aa0000", "profile": "#0044aa"}
ARM_LABELS = {"sde": "Stochastic integrator (inline noise)",
              "profile": "Pre-generated profile + deterministic RK"}


def _loadPareto(moment: str) -> dict:
    path = os.path.join(RESULTS_DIR, f"pareto_{moment}.json")
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} not found; run analyzeResults.py first.")
    with open(path) as f:
        return json.load(f)


def _curveAtN(curve, N):
    """Return the curve point nearest to sample-count N."""
    return min(curve, key=lambda p: abs(p["N"] - N))


def plotPareto(data: dict, fileName: str) -> plt.Figure:
    """Accuracy vs wall-time, every config curve + per-arm frontier."""
    fig, ax = plt.subplots(figsize=(7, 5))

    for c in data["configs"]:
        walls = [p["wall"] for p in c["curve"]]
        errs = [p["error"] for p in c["curve"]]
        ax.plot(walls, errs, color=ARM_COLORS.get(c["arm"], "gray"),
                alpha=0.2, lw=0.8, zorder=1)

    for arm, front in data["fronts"].items():
        if not front:
            continue
        walls = [p["wall"] for p in front]
        errs = [p["error"] for p in front]
        ax.plot(walls, errs, color=ARM_COLORS.get(arm, "gray"), lw=2.4,
                marker="o", ms=3, label=ARM_LABELS.get(arm, arm), zorder=3)

    ref = data.get("reference")
    if ref and ref.get("momentSE"):
        ax.axhline(ref["momentSE"], color="gray", ls=":", lw=1,
                   label="reference moment SE (MC floor)")

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Wall time to estimate the moment [s]")
    ax.set_ylabel(f"Error in {data['moment']} of FoM [m]")
    ax.set_title("Accuracy vs cost: stochastic integrator vs noise profile")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, which="both", alpha=0.25)
    fig.tight_layout()
    return fig


def plotParetoWhisker(data: dict, fileName: str) -> plt.Figure:
    """Accuracy-vs-wall with 95% error bars on BOTH axes, at the pilot budget.

    Each config is plotted at ``N = nPilot`` -- the number of realizations
    actually simulated -- so *both* coordinates are measured, not extrapolated:

    * x error bar = 95% bootstrap CI on total wall time (per-run wall-time CI
      x nPilot);
    * y error bar = 95% band on the total error of the moment estimate, i.e. the
      discretization bias CI combined in quadrature with the Monte-Carlo SE at
      nPilot.  For the profile arm the bias CI is the tight common-random-numbers
      estimate; for the SDE arm it is the (wider) unpaired estimate.

    Plotting at the measured budget is what makes the whiskers statistically
    meaningful: the semi-analytic 1/sqrt(N) curves elsewhere are extrapolations,
    whereas here every point and its bars come directly from the cached samples.
    """
    fig, ax = plt.subplots(figsize=(7.6, 5.4))

    # Colour by (arm, method) so the three SDE methods are distinguishable -- in
    # particular Euler-Maruyama, whose large bias would otherwise be confused
    # with the accurate weak-order-2 SRK methods.
    methodStyle = _METHOD_STYLE
    labelled = set()
    for c in data["configs"]:
        N = c["nPilot"]
        p = _curveAtN(c["curve"], N)
        wall = c["perRunWall"] * N
        wallLo = c["perRunWallLo"] * N
        wallHi = c["perRunWallHi"] * N
        err = max(p["error"], 1e-9)
        errLo = max(p.get("errorLo", err), 1e-9)
        errHi = max(p.get("errorHi", err), 1e-9)

        color, marker, base = methodStyle.get(
            (c["arm"], c["method"]), ("gray", "o", f"{c['arm']}:{c['method']}"))
        lab = base if base not in labelled else None
        labelled.add(base)
        ax.errorbar(
            wall, err,
            xerr=[[max(wall - wallLo, 0)], [max(wallHi - wall, 0)]],
            yerr=[[max(err - errLo, 0)], [max(errHi - err, 0)]],
            fmt=marker, ms=4, color=color, ecolor=color, elinewidth=1.0,
            capsize=2.5, alpha=0.85, label=lab, zorder=3)
        # annotate dt
        ax.annotate(f"{c['dt']:g}", (wall, err), fontsize=6, color=color,
                    xytext=(3, 3), textcoords="offset points", zorder=4)

    ref = data.get("reference")
    if ref and ref.get("momentSE"):
        ax.axhline(ref["momentSE"], color="gray", ls=":", lw=1,
                   label="reference moment MC SE")

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel(f"Total wall time at N={{pilot}} samples [s]")
    ax.set_ylabel(f"Total error in {data['moment']} of FoM [m]  (bias ⊕ MC)")
    ax.set_title("Accuracy vs cost with 95% margins (labelled by dt [s])")
    ax.legend(loc="lower left", fontsize=8)
    ax.grid(True, which="both", alpha=0.25)
    fig.tight_layout()
    return fig


# Colour/marker per (arm, method); shared by the bias-centric figures.
_METHOD_STYLE = {
    ("sde", "W2Ito2"): ("#aa0000", "o", "SDE W2Ito2"),
    ("sde", "SOSRA"): ("#dd6600", "s", "SDE SOSRA"),
    ("sde", "SRA1"): ("#dd0088", "^", "SDE SRA1"),
    ("profile", "RK4"): ("#0044aa", "o", "profile RK4"),
    ("profile", "RK2"): ("#00aaaa", "s", "profile RK2"),
}


def plotBiasParetoWhisker(data: dict, fileName: str) -> plt.Figure:
    r"""Pareto of *integrator bias* vs *per-realization cost* -- the paper figure.

    This is the figure that answers the paper's actual question: how much
    discretization **bias** does each integrator carry, and what does one
    realization cost?  Neither axis depends on the Monte-Carlo sample budget, so
    the finite-sample sampling noise -- which is a property of *our experiment*,
    not of the integrator -- is deliberately excluded from the plotted value and
    appears only as the uncertainty (whisker) on the bias.

    * y-axis: ``|bias|`` of the moment vs the independent reference.  Whisker =
      95% CI of the bias (tight common-random-numbers estimate for the profile
      arm; wider unpaired estimate for the SDE arm).
    * x-axis: median wall time for **one** realization, with its 95% CI.

    Honest handling of unresolved bias: a config whose bias CI **straddles zero**
    has a bias that is statistically indistinguishable from zero at the sample
    size run.  For those we do not draw a (meaningless) point at the noisy
    magnitude; instead we draw a **downward arrow** at the upper end of the bias
    CI -- i.e. "the bias is no larger than this".  Points whose CI excludes zero
    are drawn normally with symmetric error bars.
    """
    fig, ax = plt.subplots(figsize=(7.6, 5.4))

    labelled = set()
    for c in data["configs"]:
        color, marker, base = _METHOD_STYLE.get(
            (c["arm"], c["method"]), ("gray", "o", f"{c['arm']}:{c['method']}"))
        x = c["perRunWall"]
        xerr = [[max(x - c["perRunWallLo"], 0)], [max(c["perRunWallHi"] - x, 0)]]

        bAbs = abs(c["biasSigned"])
        # |CI| endpoints in magnitude
        loMag = min(abs(c["biasLo"]), abs(c["biasHi"]))
        hiMag = max(abs(c["biasLo"]), abs(c["biasHi"]))
        lab = base if base not in labelled else None
        labelled.add(base)

        if c["biasConsistentZero"]:
            # Upper bound only: arrow pointing down at the CI's upper magnitude.
            yub = max(hiMag, 1e-3)
            ax.errorbar(x, yub, xerr=xerr, yerr=[[yub * 0.55], [0]],
                        uplims=False, lolims=True, fmt=marker, ms=5, mfc="none",
                        color=color, ecolor=color, elinewidth=1.0, capsize=2.5,
                        alpha=0.9, label=lab, zorder=3)
            ax.annotate(f"{c['dt']:g}", (x, yub), fontsize=6, color=color,
                        xytext=(3, 2), textcoords="offset points")
        else:
            y = max(bAbs, 1e-3)
            ax.errorbar(x, y, xerr=xerr,
                        yerr=[[max(y - loMag, 0)], [max(hiMag - y, 0)]],
                        fmt=marker, ms=5, color=color, ecolor=color,
                        elinewidth=1.0, capsize=2.5, alpha=0.9, label=lab, zorder=3)
            ax.annotate(f"{c['dt']:g}", (x, y), fontsize=6, color=color,
                        xytext=(3, 3), textcoords="offset points")

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Wall time per realization [s]")
    ax.set_ylabel(f"|discretization bias| of {data['moment']}[FoM] [m]")
    ax.set_title("Integrator bias vs per-run cost (95% CI; hollow+arrow = upper bound)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, which="both", alpha=0.25)
    fig.tight_layout()
    return fig


def plotBias(data: dict, fileName: str) -> plt.Figure:
    """Discretization bias of the moment vs step size, with 95% CI whiskers.

    Bias is shown on a signed, symmetric-log axis so a CI straddling zero is
    visible. The profile arm's bars are the tight common-random-numbers estimate
    (often resolving a sub-metre bias); the SDE arm's are the wider unpaired
    estimate, many of which straddle zero (bias unresolved below the MC floor).
    """
    fig, ax = plt.subplots(figsize=(7.2, 5))

    byMethod = {}
    for c in data["configs"]:
        if not np.isfinite(c["biasSigned"]):
            continue
        byMethod.setdefault((c["arm"], c["method"]), []).append(c)

    for (arm, method), cs in sorted(byMethod.items()):
        cs.sort(key=lambda c: c["dt"])
        dts = [c["dt"] for c in cs]
        bias = [c["biasSigned"] for c in cs]
        lo = [c["biasSigned"] - c["biasLo"] for c in cs]
        hi = [c["biasHi"] - c["biasSigned"] for c in cs]
        ls = "-" if arm == "sde" else "--"
        crn = " (CRN)" if any(c["biasMethod"] == "paired-crn" for c in cs) else ""
        ax.errorbar(dts, bias, yerr=[lo, hi], ls=ls, marker="o", ms=4,
                    capsize=2.5, color=ARM_COLORS.get(arm, "gray"),
                    label=f"{arm}:{method}{crn}")

    ax.axhline(0.0, color="black", lw=0.8, alpha=0.5)
    ax.set_xscale("log")
    ax.set_yscale("symlog", linthresh=0.05)
    ax.set_xlabel("Macro step size dt [s]")
    ax.set_ylabel(f"signed bias of {data['moment']} of FoM [m]")
    ax.set_title("Discretization bias vs step size (95% CI)")
    ax.legend(loc="best", fontsize=7, ncol=2)
    ax.grid(True, which="both", alpha=0.25)
    fig.tight_layout()
    return fig


def plotSpeedup(data: dict, fileName: str) -> Optional[plt.Figure]:
    """Speedup of the stochastic arm over the profile arm vs accuracy target."""
    crossover = data.get("crossover")
    if not crossover or not crossover.get("rows"):
        return None
    rows = crossover["rows"]
    fig, ax = plt.subplots(figsize=(7, 5))
    targets = [r["targetError"] for r in rows]
    speedups = [r["speedup"] for r in rows]
    ax.plot(targets, speedups, "o-", color="#006600")
    ax.axhline(1.0, color="gray", ls=":", lw=1)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel(f"Target error in {data['moment']} of FoM [m]")
    ax.set_ylabel("Speedup (profile wall / stochastic wall)")
    ax.set_title("How much cheaper is the stochastic integrator?")
    ax.grid(True, which="both", alpha=0.25)
    fig.tight_layout()
    return fig


def run(moment: str = "std", showPlots: bool = False, save: bool = True) -> dict:
    """Generate all figures from cached analysis JSON."""
    data = _loadPareto(moment)
    figures = {}
    figures["stochasticEfficiency_biasPareto"] = plotBiasParetoWhisker(data, "biasPareto")
    figures["stochasticEfficiency_pareto"] = plotPareto(data, "pareto")
    figures["stochasticEfficiency_paretoWhisker"] = plotParetoWhisker(data, "paretoWhisker")
    figures["stochasticEfficiency_bias"] = plotBias(data, "bias")
    speedup = plotSpeedup(data, "speedup")
    if speedup is not None:
        figures["stochasticEfficiency_speedup"] = speedup

    if save:
        for name, fig in figures.items():
            path = os.path.join(RESULTS_DIR, f"{name}.svg")
            fig.savefig(path)
            print(f"Wrote {path}")
    if showPlots:
        plt.show()
    return figures


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--moment", choices=["std", "mean"], default="std")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args(argv)
    run(moment=args.moment, showPlots=args.show, save=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
