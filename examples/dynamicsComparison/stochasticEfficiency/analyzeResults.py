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
Semi-analytic accuracy-vs-wall-time analysis for the stochastic efficiency study.

Reads the cached FoM samples produced by ``runComparison.py`` and turns them
into an accuracy-vs-cost trade-off, treating the whole thing as a *statistical*
integrator comparison.

The figure of merit is a random variable (the noisy final semi-major axis ``a``);
what we actually want is a *moment* of it -- here primarily its standard
deviation across realizations (the part that the noise model drives), and
secondarily its mean.  For a given integrator configuration with step size
``dt`` we decompose the total error in the estimated moment into two independent
pieces:

* **Discretization bias** ``b(dt)`` -- how far the configuration's moment
  (estimated from many samples) sits from the *true* moment.  We estimate the
  true moment from a dedicated high-fidelity ``reference`` config and take
  ``b(dt) = |moment(dt) - moment_ref|``.  The bias is a property of ``dt`` and
  the method, essentially independent of the sample count.

* **Monte-Carlo (sampling) error** ``s(N)`` -- the statistical uncertainty of
  the moment estimate from a *finite* number ``N`` of realizations.  This scales
  as ``1/sqrt(N)``; we measure its size at the pilot sample count with a
  bootstrap, giving ``s(N) = se_pilot * sqrt(N_pilot / N)``.

The total RMS error of the moment estimate at sample count ``N`` is then

    err(N) = sqrt( b(dt)^2 + s(N)^2 )

and the wall time to obtain it is ``wall(N) = N * perRunWall``.  Sweeping ``N``
traces an accuracy-vs-cost curve for each ``(method, dt)`` *without simulating
every point* -- only the pilot samples are ever run.  The Pareto frontier is the
lower-left envelope of all these curves; comparing the SDE-arm envelope against
the profile-arm envelope answers the study's question.

Outputs ``results/pareto_<moment>.json`` consumed by ``plotResults.py``.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Optional

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import statsTools as st  # noqa: E402
import sampleStore  # noqa: E402

RESULTS_DIR = os.environ.get("STOCHEFF_RESULTS_DIR", os.path.join(_HERE, "results"))
SAMPLES_DIR = os.path.join(RESULTS_DIR, "samples")
MANIFEST_PATH = os.path.join(RESULTS_DIR, "manifest.json")


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
def loadManifest() -> dict:
    if not os.path.exists(MANIFEST_PATH):
        raise FileNotFoundError(
            f"No manifest at {MANIFEST_PATH}. Run runComparison.py first.")
    with open(MANIFEST_PATH) as f:
        return json.load(f)


def loadRecord(cfgHash: str) -> dict:
    # Use the shard-aware merge so analysis works on a raw sharded cluster sweep
    # (many <hash>__shardNNNN.npz) as well as a consolidated single file.
    return sampleStore.loadMerged(SAMPLES_DIR, cfgHash)


# All statistical estimators (moment CI, paired/unpaired bias, wall CI, total
# error band) live in statsTools.py so analysis and plotting share one source.


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def analyze(moment: str = "std", fom: str = "fomA",
            nGrid: Optional[List[int]] = None) -> dict:
    """Build the semi-analytic accuracy-vs-cost data for every config.

    Args:
        moment: which moment of the FoM to target ("std" or "mean").
        fom: which figure of merit array to use ("fomA" or "fomAlt").
        nGrid: sample-count grid over which to trace each config's err(N)/wall(N)
            curve. Defaults to a geometric sweep.

    Returns:
        A dict ready to be serialized to JSON, including per-config curves, the
        reference moment, and the precomputed Pareto frontier per arm.
    """
    manifest = loadManifest()
    if nGrid is None:
        nGrid = [int(round(x)) for x in np.geomspace(5, 200000, 40)]

    # Identify the reference config: the independent scipy ground truth
    # (arm == "reference", or the explicit isReference flag).  Match on parsed
    # fields, never a string prefix.
    refHash = None
    for cfgHash, info in manifest.items():
        if info.get("arm") == "reference" or info.get("isReference"):
            refHash = cfgHash
            break
    refMoment = None
    refInfo = None
    refRec = None
    if refHash is not None:
        refRec = loadRecord(refHash)
        refCI = st.momentCI(refRec[fom], moment)
        refMoment = refCI.value
        refInfo = {"hash": refHash, "key": manifest[refHash]["key"],
                   "nSamples": int(refRec[fom].size), "moment": refMoment,
                   "momentSE": refCI.se, "momentLo": refCI.lo, "momentHi": refCI.hi}

    configs = []
    for cfgHash, info in manifest.items():
        # The reference is the truth, not a competitor; keep it out of the
        # per-config curves and Pareto fronts (it is reported via refInfo).
        if info.get("arm") == "reference" or info.get("isReference"):
            continue
        rec = loadRecord(cfgHash)
        samples = rec[fom]
        nPilot = samples.size
        if nPilot < 2:
            continue

        # --- moment estimate + Monte-Carlo margin of error ---
        momCI = st.momentCI(samples, moment)
        sePilot = momCI.se

        # --- discretization bias of the moment vs the reference ---
        # Profile arm shares the OU path with the reference (common random
        # numbers) -> a paired bootstrap resolves a sub-metre bias.  The SDE arm
        # draws noise internally, so only the unpaired estimate is possible.
        biasInterval = None
        biasMethod = "none"
        if refRec is not None:
            if info["arm"] == "profile":
                biasInterval = st.biasPaired(samples, rec["seeds"],
                                             refRec[fom], refRec["seeds"], moment)
                biasMethod = "paired-crn"
            if biasInterval is None:   # SDE arm, or too few shared seeds
                biasInterval = st.biasUnpaired(samples, refRec[fom], moment)
                biasMethod = "unpaired"

        # --- per-run wall time + its margin of error ---
        wCI = st.wallCI(rec["wall"])
        perRunWall = wCI.value

        # --- total error(N) with a band, over the sample grid ---
        curve = []
        for N in nGrid:
            if biasInterval is not None:
                e = st.totalErrorAtN(biasInterval, sePilot, nPilot, N)
            else:
                seN = sePilot * np.sqrt(nPilot / N)
                e = {"error": float(seN), "errorLo": float(seN),
                     "errorHi": float(seN), "seN": float(seN)}
            curve.append({"N": int(N), "error": e["error"],
                          "errorLo": e["errorLo"], "errorHi": e["errorHi"],
                          "wall": perRunWall * N})

        biasConsistentZero = (biasInterval is not None
                              and biasInterval.lo <= 0.0 <= biasInterval.hi)
        configs.append({
            "hash": cfgHash, "key": info["key"], "arm": info["arm"],
            "method": info["method"], "dt": info["dt"], "relTol": info.get("relTol"),
            "nPilot": int(nPilot),
            "moment": momCI.value, "momentLo": momCI.lo, "momentHi": momCI.hi,
            "momentSE": momCI.se,
            "bias": abs(biasInterval.value) if biasInterval else float("nan"),
            "biasSigned": biasInterval.value if biasInterval else float("nan"),
            "biasLo": biasInterval.lo if biasInterval else float("nan"),
            "biasHi": biasInterval.hi if biasInterval else float("nan"),
            "biasMethod": biasMethod, "biasConsistentZero": biasConsistentZero,
            "sePilot": sePilot, "perRunWall": perRunWall,
            "perRunWallLo": wCI.lo, "perRunWallHi": wCI.hi,
            "curve": curve,
        })

    # Pareto frontier per arm: pool all (wall, error) points across the arm's
    # configs and keep the lower-left envelope (min error achievable at or below
    # each wall budget).
    fronts = {}
    for arm in ("sde", "profile"):
        pts = []
        for c in configs:
            if c["arm"] != arm:
                continue
            for p in c["curve"]:
                pts.append((p["wall"], p["error"], c["key"], p["N"]))
        pts.sort(key=lambda t: t[0])
        front = []
        bestErr = float("inf")
        for wall, err, key, N in pts:
            if err < bestErr - 1e-18:
                bestErr = err
                front.append({"wall": wall, "error": err, "key": key, "N": N})
        fronts[arm] = front

    return {
        "moment": moment, "fom": fom, "reference": refInfo,
        "nGrid": nGrid, "configs": configs, "fronts": fronts,
    }


def crossoverWall(fronts: dict) -> Optional[dict]:
    """Find, for a range of accuracy targets, which arm is cheaper.

    Returns a summary that makes the headline claim explicit: at a given target
    error in the moment, the wall time each arm needs, and the speedup factor of
    the stochastic arm over the profile arm.
    """
    sde = fronts.get("sde", [])
    prof = fronts.get("profile", [])
    if not sde or not prof:
        return None

    def wallForError(front, targetErr):
        # front is sorted by increasing wall, decreasing error; find the
        # cheapest point achieving error <= targetErr.
        best = None
        for p in front:
            if p["error"] <= targetErr:
                best = p["wall"]
                break
        return best

    # Build a set of achievable targets common to both arms.
    minErr = max(min(p["error"] for p in sde), min(p["error"] for p in prof))
    maxErr = min(max(p["error"] for p in sde), max(p["error"] for p in prof))
    if not (np.isfinite(minErr) and np.isfinite(maxErr)) or minErr >= maxErr:
        return None
    targets = np.geomspace(minErr * 1.01, maxErr * 0.99, 12)

    rows = []
    for te in targets:
        ws = wallForError(sde, te)
        wp = wallForError(prof, te)
        if ws and wp:
            rows.append({"targetError": float(te), "wallSde": ws,
                         "wallProfile": wp, "speedup": wp / ws})
    return {"rows": rows} if rows else None


def biasResolutionCost(data: dict, relTarget: float = 0.10,
                       negligibleFracOfTruth: float = 0.005,
                       cores: float = 1.0) -> dict:
    r"""Compute the compute time to characterise each integrator's bias.

    For an accuracy *comparison* the right requirement is to know each
    integrator's discretization bias to a fixed *relative* precision -- by
    default 10% (``relTarget``) -- so that integrators can be ranked.  The
    Monte-Carlo error on the bias estimate scales as ``1/sqrt(N)``, so to reach
    ``SE_bias(N) <= relTarget*|bias|`` from a pilot of ``nPilot`` runs needs

        N_req = nPilot * (SE_pilot / (relTarget*|bias|))^2,

    and (single-core) ``time = N_req * perRunWall``.  Because ``N_req`` blows up
    as ``(SE/|bias|)^2`` when the true bias shrinks, demanding 10% of an
    arbitrarily small bias is both impossible and pointless: a bias far below the
    truth does not change any ranking.  We therefore use a **mixed target**:

    * if ``|bias|`` is large enough to matter -- i.e. above
      ``negligibleFracOfTruth * truth`` (default 0.5% of the reference std) --
      resolve it to ``relTarget`` relative precision (a genuine point estimate);
    * otherwise the integrator is essentially exact for this purpose, so we only
      need to *upper-bound* the bias at the negligibility threshold, i.e. reach
      ``SE_bias <= relTarget * (negligibleFracOfTruth*truth)``.  This yields a
      tight, honest "bias < X" statement at a tiny fraction of the cost.

    ``cores`` divides the single-core time to give an estimated wall time on a
    parallel machine (realizations are i.i.d. and embarrassingly parallel).

    Returns per-config rows plus per-arm totals, ready to serialise/print.
    """
    truth = data.get("reference", {}).get("moment")
    rows = []
    totals = {}
    negFloor = (negligibleFracOfTruth * truth) if truth else None

    for c in data["configs"]:
        nP = c["nPilot"]
        seBias = c["sePilot"] if c["biasMethod"] == "unpaired" else None
        # For the bias SE we need the SE of the *bias estimator*, which is the
        # bias CI half-width / 1.96 (already computed, method-correct: paired for
        # profile, unpaired for SDE).
        biasSE = (abs(c["biasHi"] - c["biasLo"]) / 2.0) / 1.96
        bias = abs(c["biasSigned"])
        if not np.isfinite(biasSE) or biasSE <= 0:
            continue

        matters = (negFloor is None) or (bias > negFloor)
        if matters:
            # Resolve the bias to relTarget relative precision.
            targetSE = relTarget * bias
            mode = "estimate-10pct"
        else:
            # Bias is negligible for ranking; we only need to *demonstrate* that,
            # i.e. place the CI upper edge at the negligibility floor (SE ~ floor
            # gives a ~2-sigma "bias < floor" statement).  This is the CHEAPER of
            # the two requirements -- we never demand more precision on a bias
            # that does not affect the comparison.
            targetSE = negFloor
            mode = "upper-bound"

        if targetSE <= 0:
            continue
        nReq = nP * (biasSE / targetSE) ** 2
        timeSingle = nReq * c["perRunWall"]
        timeCores = timeSingle / max(cores, 1.0)
        rows.append({
            "key": c["key"], "arm": c["arm"], "method": c["method"], "dt": c["dt"],
            "bias": bias, "relBiasPct": (100 * bias / truth) if truth else float("nan"),
            "biasSE": biasSE, "biasMethod": c["biasMethod"], "mode": mode,
            "nReq": nReq, "timeSingleCore": timeSingle, "timeCores": timeCores,
        })
        totals.setdefault(c["arm"], 0.0)
        totals[c["arm"]] += timeSingle

    rows.sort(key=lambda r: -r["timeSingleCore"])
    return {"relTarget": relTarget, "negligibleFracOfTruth": negligibleFracOfTruth,
            "cores": cores, "truth": truth, "rows": rows,
            "totalSingleCoreByArm": totals,
            "grandTotalSingleCore": sum(totals.values()),
            "grandTotalCores": sum(totals.values()) / max(cores, 1.0)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--moment", choices=["std", "mean"], default="std")
    parser.add_argument("--fom", choices=["fomA", "fomAlt"], default="fomA")
    parser.add_argument("--out", type=str, default=None,
                        help="output JSON path (default results/pareto_<moment>.json)")
    parser.add_argument("--relTarget", type=float, default=0.10,
                        help="relative precision target on the bias (default 0.10 = 10%%)")
    parser.add_argument("--negligibleFrac", type=float, default=0.005,
                        help="bias below this fraction of the truth is only upper-bounded "
                             "(default 0.005 = 0.5%% of the reference std)")
    parser.add_argument("--cores", type=float, default=1.0,
                        help="number of cores for the parallel wall-time estimate")
    args = parser.parse_args(argv)

    data = analyze(moment=args.moment, fom=args.fom)
    data["crossover"] = crossoverWall(data["fronts"])
    data["biasResolutionCost"] = biasResolutionCost(
        data, relTarget=args.relTarget,
        negligibleFracOfTruth=args.negligibleFrac, cores=args.cores)

    out = args.out or os.path.join(RESULTS_DIR, f"pareto_{args.moment}.json")
    with open(out, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Wrote {out}")

    # Console summary
    ref = data["reference"]
    if ref:
        print(f"Reference moment ({args.moment} of {args.fom}): "
              f"{ref['moment']:.6g} +/- {ref['momentSE']:.3g}  "
              f"(N={ref['nSamples']}, {ref['key']})")
    for arm in ("sde", "profile"):
        biases = [c["bias"] for c in data["configs"]
                  if c["arm"] == arm and np.isfinite(c["bias"])]
        if biases:
            print(f"  {arm}: {sum(1 for c in data['configs'] if c['arm']==arm)} configs, "
                  f"bias range [{min(biases):.3g}, {max(biases):.3g}]")
    if data["crossover"]:
        print("Speedup of stochastic arm over profile arm (by accuracy target):")
        for row in data["crossover"]["rows"]:
            print(f"  err<={row['targetError']:.4g}: "
                  f"wall_sde={row['wallSde']:.4g}s  wall_profile={row['wallProfile']:.4g}s  "
                  f"speedup={row['speedup']:.2f}x")

    brc = data.get("biasResolutionCost")
    if brc and brc["rows"]:
        def fmtT(s):
            if s < 90: return f"{s:.0f}s"
            if s < 5400: return f"{s/60:.1f}min"
            if s < 2 * 86400: return f"{s/3600:.1f}h"
            return f"{s/86400:.1f}d"
        print(f"\nCompute to characterise bias at {brc['relTarget']*100:.0f}% relative "
              f"(or upper-bound below {brc['negligibleFracOfTruth']*100:.1f}% of truth), "
              f"on {brc['cores']:g} core(s):")
        print(f"  {'config':22s} {'dt':>5s} {'|bias|':>8s} {'rel%':>6s} "
              f"{'mode':>14s} {'N_req':>9s} {'t_1core':>9s} {'t_cores':>9s}")
        for r in brc["rows"]:
            print(f"  {r['key']:22s} {r['dt']:5g} {r['bias']:8.3f} {r['relBiasPct']:5.2f}% "
                  f"{r['mode']:>14s} {r['nReq']:9.2g} "
                  f"{fmtT(r['timeSingleCore']):>9s} {fmtT(r['timeCores']):>9s}")
        for arm, tot in sorted(brc["totalSingleCoreByArm"].items()):
            print(f"  total {arm:8s}: {fmtT(tot):>9s} single-core  "
                  f"({fmtT(tot/max(brc['cores'],1)):>9s} on {brc['cores']:g} cores)")
        print(f"  GRAND TOTAL: {fmtT(brc['grandTotalSingleCore'])} single-core  "
              f"({fmtT(brc['grandTotalCores'])} on {brc['cores']:g} cores)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
