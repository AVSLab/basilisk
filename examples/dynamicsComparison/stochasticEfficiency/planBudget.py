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
Turn a cheap *pilot* run into a precision-targeted sampling **plan**.

The study's goal is to know each integrator's discretization bias `b(dt) =
sigma_dt - sigma_true` well enough to rank integrators. Two precision
requirements drive the budget:

  (A) The Monte-Carlo error on each config's bias must be a small fraction
      (default 10%) of that bias, so the bias is actually *resolved*:
          SE_bias <= relTarget * |bias|.
  (B) The reference must be precise enough that it does not itself limit (A) for
      ANY config we choose to resolve. Since the reference is shared, its error
      `SE_ref` must satisfy (A) for the *smallest* bias we resolve.

For the standard deviation estimand, the bootstrap SE of a config's std scales as
`SE(std) ~ sigma / sqrt(2 N)` (sigma = the FoM spread, ~equal for every config
here). The bias is a difference of two std estimates, so its variance is the sum:

    SE_bias^2 = SE_cfg^2 + SE_ref^2.

We split the budget so the config and the reference each contribute at most
`relTarget*|bias| / sqrt(2)` to `SE_bias` (equal halves in quadrature). That
gives, per config,

    N_cfg = sigma^2 / (2 * (relTarget*|bias| / sqrt(2))^2)
          = sigma^2 / (relTarget*|bias|)^2,

and the reference must meet the tightest of these:

    N_ref = sigma^2 / (relTarget * b_min_resolved)^2,

where `b_min_resolved` is the smallest bias among the configs we choose to
resolve. The **profile arm is special**: it shares the OU path with the
reference (common random numbers), so its bias SE is tiny and a few thousand
samples suffice regardless of `|bias|` -- we cap its N at `profileN`.

Configs whose |bias| is below `negligibleFrac * sigma` (default 0.5%) are *not*
worth resolving to 10% (they would each need millions of samples to pin a
quantity that does not change any ranking). For those we set a modest N that
*upper-bounds* the bias at the negligibility floor, and mark them "upper-bound".

The output `plan.json` lists, per config (including the reference), the target N
and a shard count chosen so each shard's wall time fits a target per-task budget.
``runComparison.py --planFile plan.json`` then runs exactly this.

Usage:
    # after a pilot sweep exists (e.g. nSamples ~ 200-400 per config + a pilot ref)
    python planBudget.py --relTarget 0.10 --negligibleFrac 0.005 \
        --perTaskSeconds 36000 --out results/plan.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import statsTools as st  # noqa: E402
import sampleStore  # noqa: E402

RESULTS_DIR = os.environ.get("STOCHEFF_RESULTS_DIR", os.path.join(_HERE, "results"))
SAMPLES_DIR = os.path.join(RESULTS_DIR, "samples")
MANIFEST_PATH = os.path.join(RESULTS_DIR, "manifest.json")


def _load():
    with open(MANIFEST_PATH) as f:
        manifest = json.load(f)
    refHash = next((h for h, v in manifest.items()
                    if v.get("arm") == "reference" or v.get("isReference")), None)
    if refHash is None:
        raise RuntimeError("No reference config in the pilot. Run a pilot reference first.")
    refRec = sampleStore.loadMerged(SAMPLES_DIR, refHash)
    return manifest, refHash, refRec


def buildPlan(relTarget=0.10, negligibleFrac=0.005, fom="fomA",
              profileN=3000, perTaskSeconds=36000.0, maxShards=512,
              minRefN=None):
    """Compute the precision-targeted plan from the pilot cache.

    Args:
        relTarget: target SE_bias / |bias| (0.10 = resolve bias to 10%).
        negligibleFrac: biases below this fraction of sigma are upper-bounded,
            not resolved (their N is set to bound them at the floor).
        profileN: sample count for every profile-arm config (CRN -> cheap).
        perTaskSeconds: target wall time per array task; shard counts are chosen
            so N_config / nShards * perRunWall <= perTaskSeconds.
        maxShards: hard cap on shards per config.
        minRefN: optional floor on the reference N.

    Returns a plan dict (also the thing serialized to plan.json).
    """
    manifest, refHash, refRec = _load()
    sigma = float(np.std(refRec[fom], ddof=1))
    negFloor = negligibleFrac * sigma

    # 1) per-config |bias| (paired/CRN for profile, unpaired for SDE) and per-run wall
    info = {}
    for h, v in manifest.items():
        if v.get("arm") == "reference" or v.get("isReference"):
            continue
        rec = sampleStore.loadMerged(SAMPLES_DIR, h)
        if rec[fom].size < 2:
            continue
        if v["arm"] == "profile":
            bi = st.biasPaired(rec[fom], rec["seeds"], refRec[fom], refRec["seeds"], "std")
        else:
            bi = st.biasUnpaired(rec[fom], refRec[fom], "std")
        info[v["key"]] = {
            "arm": v["arm"], "method": v["method"], "dt": v["dt"],
            "bias": abs(bi.value), "perRunWall": float(np.median(rec["wall"])),
            "pilotN": int(rec[fom].size),
        }

    # 2) which SDE biases are big enough to resolve (set the reference precision)
    resolvedSdeBias = [d["bias"] for d in info.values()
                       if d["arm"] == "sde" and d["bias"] > negFloor]
    bMinResolved = min(resolvedSdeBias) if resolvedSdeBias else negFloor

    # 3) reference N so SE_ref <= relTarget*bMinResolved/sqrt2  (half the budget)
    refTargetSE = relTarget * bMinResolved / np.sqrt(2.0)
    nRef = int(np.ceil(sigma ** 2 / (2.0 * refTargetSE ** 2)))
    if minRefN:
        nRef = max(nRef, int(minRefN))
    refPerRun = float(np.median(refRec["wall"]))

    # 4) per-config target N (SDE arm; profile handled separately as CRN-cheap)
    def nForBias(bias):
        resolve = bias > negFloor
        if resolve:
            # resolve the bias to relTarget relative precision
            targetSE = relTarget * bias / np.sqrt(2.0)
        else:
            # bias is negligible for ranking: we only need to SHOW it sits below
            # the floor, i.e. SE ~ negFloor (a ~1-sigma "bias < floor" bound) --
            # vastly cheaper than resolving a bias *at* the floor to relTarget.
            targetSE = negFloor / np.sqrt(2.0)
        n = sigma ** 2 / (2.0 * targetSE ** 2)
        # never fewer than the pilot; cap absurd tails at the reference N (no
        # point resolving a config better than the reference that anchors it)
        return int(min(max(np.ceil(n), 50), nRef)), resolve

    def shardsFor(n, perRun):
        if n * perRun <= perTaskSeconds:
            return 1
        return int(min(maxShards, np.ceil(n * perRun / perTaskSeconds)))

    configs = {}
    # reference first
    refShards = shardsFor(nRef, refPerRun)
    configs["reference:scipy:dt=2"] = {
        "arm": "reference", "nSamples": int(nRef), "nShards": int(refShards),
        "mode": "reference", "targetSE": float(refTargetSE),
        "estWallCoreHours": nRef * refPerRun / 3600.0,
    }
    totalCoreSec = nRef * refPerRun
    for key, d in sorted(info.items()):
        if d["arm"] == "profile":
            n, resolve = profileN, True
        else:
            n, resolve = nForBias(d["bias"])
        nsh = shardsFor(n, d["perRunWall"])
        configs[key] = {
            "arm": d["arm"], "method": d["method"], "dt": d["dt"],
            "pilotBias": d["bias"], "nSamples": int(n), "nShards": int(nsh),
            "mode": "resolve" if resolve else "upper-bound",
            "estWallCoreHours": n * d["perRunWall"] / 3600.0,
        }
        totalCoreSec += n * d["perRunWall"]

    nTasks = sum(c["nShards"] for c in configs.values())
    return {
        "sigma": sigma, "relTarget": relTarget, "negligibleFrac": negligibleFrac,
        "negFloor": negFloor, "bMinResolved": bMinResolved,
        "refN": int(nRef), "refTargetSE": float(refTargetSE),
        "profileN": int(profileN), "perTaskSeconds": float(perTaskSeconds),
        "configs": configs,
        "summary": {
            "nConfigs": len(configs), "nArrayTasks": int(nTasks),
            "totalCoreHours": totalCoreSec / 3600.0,
            "hoursOn64cores": totalCoreSec / 3600.0 / 64.0,
        },
    }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--relTarget", type=float, default=0.10,
                   help="resolve each bias to this relative MC precision (0.10=10%%)")
    p.add_argument("--negligibleFrac", type=float, default=0.005,
                   help="biases below this fraction of sigma are upper-bounded")
    p.add_argument("--profileN", type=int, default=3000,
                   help="samples per profile-arm config (CRN -> cheap)")
    p.add_argument("--perTaskSeconds", type=float, default=36000.0,
                   help="target wall seconds per array task (sets shard counts; "
                        "default 10 h, comfortably under the 24 h 'normal' QOS)")
    p.add_argument("--minRefN", type=int, default=None)
    p.add_argument("--out", type=str, default=os.path.join(RESULTS_DIR, "plan.json"))
    args = p.parse_args(argv)

    plan = buildPlan(relTarget=args.relTarget, negligibleFrac=args.negligibleFrac,
                     profileN=args.profileN, perTaskSeconds=args.perTaskSeconds,
                     minRefN=args.minRefN)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(plan, f, indent=2)

    s = plan["summary"]
    print(f"Plan written to {args.out}")
    print(f"  sigma(FoM)        = {plan['sigma']:.1f} m")
    print(f"  resolve bias to   = {plan['relTarget']*100:.0f}% (down to "
          f"{plan['bMinResolved']:.2f} m; below {plan['negFloor']:.2f} m -> upper-bound)")
    print(f"  reference N       = {plan['refN']:.3g}  in {plan['configs']['reference:scipy:dt=2']['nShards']} shards "
          f"(SE_ref={plan['refTargetSE']:.3f} m, {plan['configs']['reference:scipy:dt=2']['estWallCoreHours']:.1f} core-h)")
    print(f"  array tasks       = {s['nArrayTasks']}")
    print(f"  TOTAL compute     = {s['totalCoreHours']:.1f} core-hours "
          f"(~{s['hoursOn64cores']:.1f} h on 64 cores)")
    print("\n  per-config (sorted by cost):")
    rows = sorted(plan["configs"].items(),
                  key=lambda kv: -kv[1]["estWallCoreHours"])
    print(f"    {'config':24s} {'mode':11s} {'N':>9s} {'shards':>6s} {'core-h':>8s}")
    for k, c in rows:
        print(f"    {k:24s} {c['mode']:11s} {c['nSamples']:9g} {c['nShards']:6d} "
              f"{c['estWallCoreHours']:8.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
