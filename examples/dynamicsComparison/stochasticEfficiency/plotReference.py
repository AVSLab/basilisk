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
Plot the reference physics of the stochastic-efficiency study, as a sanity check
that the problem is "interesting" -- i.e. that the atmospheric-density noise
produces a meaningful, resolvable spread in the orbit's figure of merit.

This mirrors ``plotOrbits`` in ``examples/mujoco/scenarioStochasticDrag.py`` (the
orbit, altitude, density-vs-altitude, density-correction, and drag panels) but
adds the part that actually matters for a *statistical* study: an **ensemble**
view showing how a(tf) and the altitude spread across many noise realizations,
overlaid trajectories from several seeds, and the histogram of the final
semi-major axis with its mean +/- std (which is the figure of merit).

It uses the independent scipy reference (``referenceScipy``): a few seeds are
re-integrated densely for the time-history overlays, and -- if a reference sample
cache exists -- the full cached a(tf) ensemble is used for the histogram so the
spread shown matches exactly what the study measured.

Usage:
    python plotReference.py                 # default scenario, save SVGs
    python plotReference.py --show          # also display
    python plotReference.py --orbits 4 --stationaryStd 0.3 --nTraj 12
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import stochasticDragModel as model  # noqa: E402
import referenceScipy as ref  # noqa: E402
import sampleStore  # noqa: E402

RESULTS_DIR = os.environ.get("STOCHEFF_RESULTS_DIR", os.path.join(_HERE, "results"))
SAMPLES_DIR = os.path.join(RESULTS_DIR, "samples")


def _loadReferenceEnsemble(params: model.ScenarioParams):
    """Return cached reference a(tf) samples for THIS scenario, or None.

    Matches by the same content hash the runner uses, so we only plot an ensemble
    that actually corresponds to the requested orbits/stationaryStd.
    """
    manifest = os.path.join(RESULTS_DIR, "manifest.json")
    if not os.path.exists(manifest):
        return None
    try:
        with open(manifest) as f:
            m = json.load(f)
    except (OSError, json.JSONDecodeError):
        return None
    for cfgHash, info in m.items():
        if info.get("arm") != "reference":
            continue
        p = info.get("params", {})
        if (abs(p.get("orbits", -1) - params.orbits) < 1e-9 and
                abs(p.get("stationaryStd", -1) - params.stationaryStd) < 1e-12):
            rec = sampleStore.loadMerged(SAMPLES_DIR, cfgHash)
            if rec["fomA"].size:
                return rec
    return None


def run(params: model.ScenarioParams, nTraj: int = 8, baseSeed: int = 1_000_000,
        showPlots: bool = False, save: bool = True) -> dict:
    """Generate the reference sanity-check figures."""
    atmo = ref._readAtmoParams()
    planet, oe, rN, vN, orbitPeriod, mu = params.deriveOrbit()

    # Re-integrate a handful of seeds densely for the time-history overlays.
    trajs = [ref.runReferenceTrajectory(params, baseSeed + i, _atmo=atmo)
             for i in range(nTraj)]
    t0 = trajs[0]
    tHours = t0["t"] / 3600.0

    figures = {}

    # --- 1. orbit in the plane (overlaid realizations) ---
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.axis("equal")
    ax.add_artist(plt.Circle((0, 0), t0["planetRadius"] / 1000, color="#008800"))
    for tr in trajs:
        ax.plot(tr["pos"][:, 0] / 1000, tr["pos"][:, 1] / 1000,
                lw=0.7, alpha=0.6, color="#aa0000")
    ax.set_xlabel("in-plane X [km]")
    ax.set_ylabel("in-plane Y [km]")
    ax.set_title(f"Decaying orbit, {nTraj} noise realizations")
    fig.tight_layout()
    figures["stochasticReference_orbit"] = fig

    # --- 2. altitude vs time (spread across realizations) ---
    fig, ax = plt.subplots(figsize=(7, 5))
    altStack = np.array([tr["alt"] for tr in trajs])
    for a in altStack:
        ax.plot(tHours, a, lw=0.6, alpha=0.4, color="#aa0000")
    ax.plot(tHours, altStack.mean(0), lw=2, color="black", label="ensemble mean")
    ax.fill_between(tHours, altStack.mean(0) - altStack.std(0),
                    altStack.mean(0) + altStack.std(0), color="gray", alpha=0.3,
                    label=r"$\pm 1\sigma$")
    ax.set_xlabel("time [h]")
    ax.set_ylabel("altitude [km]")
    ax.set_title("Altitude decay and its noise-driven spread")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    figures["stochasticReference_altitude"] = fig

    # --- 3. density vs altitude (deterministic vs stochastic) ---
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.semilogy(t0["alt"], t0["rho"], lw=0.8, color="#aa0000", label="stochastic")
    ax.semilogy(t0["alt"], t0["rhoDet"], lw=1.2, color="#0044aa", label="exponential")
    ax.set_xlabel("altitude [km]")
    ax.set_ylabel(r"$\rho$ [kg/m$^3$]")
    ax.set_title("Atmospheric density (one realization)")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    figures["stochasticReference_density"] = fig

    # --- 4. density correction x(t) (the OU noise) ---
    fig, ax = plt.subplots(figsize=(7, 5))
    for tr in trajs:
        ax.plot(tr["t"] / 3600.0, tr["x"], lw=0.6, alpha=0.5, color="#aa0000")
    ax.axhline(0, color="black", lw=0.8)
    ax.axhline(params.stationaryStd, color="gray", ls="--", lw=1,
               label=rf"$\pm\sigma_{{st}}={params.stationaryStd}$")
    ax.axhline(-params.stationaryStd, color="gray", ls="--", lw=1)
    ax.set_xlabel("time [h]")
    ax.set_ylabel(r"density correction $x(t)=\rho_{stoch}/\rho_{exp}-1$ [-]")
    ax.set_title("Ornstein-Uhlenbeck density noise")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    figures["stochasticReference_densityDiff"] = fig

    # --- 5. drag magnitude vs time ---
    fig, ax = plt.subplots(figsize=(7, 5))
    for tr in trajs:
        ax.semilogy(tr["t"] / 3600.0, tr["dragMag"], lw=0.6, alpha=0.5, color="#aa0000")
    ax.set_xlabel("time [h]")
    ax.set_ylabel(r"$|F_{drag}|$ [N]")
    ax.set_title("Drag force magnitude")
    fig.tight_layout()
    figures["stochasticReference_drag"] = fig

    # --- 6. THE interesting-ness check: distribution of the figure of merit ---
    ens = _loadReferenceEnsemble(params)
    if ens is not None:
        aFom = ens["fomA"]
        nLabel = f"cached reference, N={aFom.size}"
    else:
        # Fall back to the densely-integrated trajectories' final a.
        aFom = np.array([tr["a"][-1] for tr in trajs])
        nLabel = f"re-integrated, N={aFom.size} (run the reference for more)"

    aMean, aStd = float(np.mean(aFom)), float(np.std(aFom, ddof=1))
    altSpreadKm = aStd / 1000.0  # ~ how many km the final SMA spreads
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.hist((aFom - aMean), bins=40, color="#aa0000", alpha=0.6, density=True)
    ax.axvline(0, color="black", lw=1)
    for k in (1, 2):
        ax.axvline(k * aStd, color="gray", ls="--", lw=1)
        ax.axvline(-k * aStd, color="gray", ls="--", lw=1)
    ax.set_xlabel(r"$a(t_f) - \overline{a(t_f)}$ [m]")
    ax.set_ylabel("density")
    ax.set_title(f"Figure of merit spread: std = {aStd:.1f} m "
                 f"({altSpreadKm*1000:.0f} m SMA)\n{nLabel}")
    fig.tight_layout()
    figures["stochasticReference_fomDistribution"] = fig

    # --- console verdict ---
    print(f"Scenario: orbits={params.orbits}, stationaryStd={params.stationaryStd}, "
          f"tau={params.timeConstant}s")
    print(f"  mean a(tf)      = {aMean:.1f} m  (alt mean ~ "
          f"{np.mean([tr['alt'][-1] for tr in trajs]):.2f} km)")
    print(f"  std  a(tf)      = {aStd:.2f} m   <- the figure of merit (noise signal)")
    print(f"  relative spread = {aStd/aMean*1e6:.1f} ppm of a")
    # A crude "interesting" heuristic: the noise-driven SMA spread vs a single
    # orbit's natural decay over the run.
    aDecay = abs(trajs[0]["a"][0] - trajs[0]["a"][-1])
    print(f"  deterministic SMA decay over run ~ {aDecay:.1f} m; "
          f"noise std / decay = {aStd/max(aDecay,1e-9):.3f}")
    print("  => 'interesting' if the std is well above the integrators' bias floor")
    print("     (profile ~0.03 m, SDE bias resolvable ~ a few m). Here std >> those.")

    if save:
        for name, fig in figures.items():
            path = os.path.join(RESULTS_DIR, f"{name}.svg")
            os.makedirs(RESULTS_DIR, exist_ok=True)
            fig.savefig(path)
            print(f"Wrote {path}")
    if showPlots:
        plt.show()
    return figures


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--orbits", type=float, default=2.0)
    parser.add_argument("--stationaryStd", type=float, default=0.3)
    parser.add_argument("--nTraj", type=int, default=8,
                        help="number of densely-integrated realizations to overlay")
    parser.add_argument("--baseSeed", type=int, default=1_000_000)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args(argv)

    params = model.ScenarioParams(orbits=args.orbits,
                                  stationaryStd=args.stationaryStd)
    run(params, nTraj=args.nTraj, baseSeed=args.baseSeed, showPlots=args.show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
