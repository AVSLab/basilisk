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
Sampler / driver for the stochastic-integrator efficiency study.

This script runs Monte-Carlo realizations of the stochastic-drag scenario for a
grid of integrator configurations (arm x method x step-size/tolerance) and
**caches every figure-of-merit sample to disk** so that an interrupted run can
be resumed and so that analysis / plotting never has to re-simulate.

Design for cluster use and numerical efficiency:

* **One cache file per configuration** (``results/samples/<hash>.npz``) holding
  the FoM samples, per-run wall times, step counts, and the seeds used.  Runs
  are *appended*: re-invoking with a larger ``--nSamples`` only simulates the
  missing realizations (continuing the seed sequence ``baseSeed + i``).
* **Embarrassingly parallel by config and by seed.**  Different configurations
  write different files, so many copies of this script can run concurrently on a
  cluster (e.g. one ``--only`` config per task array index) without contention.
  A per-file lock guards the rare same-config concurrent case.
* A human-readable ``results/manifest.json`` indexes every cached config.

We deliberately sample only a *pilot* number of realizations per config here;
the accuracy-vs-wall-time Pareto front is then computed **semi-analytically** in
``analyzeResults.py`` (bias from the pilot mean/std, Monte-Carlo error scaled as
``1/sqrt(N)``), so we never have to brute-force every point on the front.

Usage examples
--------------
    # List the configuration grid without running anything
    python runComparison.py --list

    # Run/extend all configs to 200 samples each (resumable)
    python runComparison.py --nSamples 200

    # Run a single config (handy for a cluster task array)
    python runComparison.py --nSamples 500 --only sde:W2Ito2:dt=20

    # Build the dedicated high-fidelity reference moments
    python runComparison.py --reference --nSamples 4000
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import List, Optional

import numpy as np

# Allow running from anywhere
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import stochasticDragModel as model  # noqa: E402
import referenceScipy  # noqa: E402
import sampleStore  # noqa: E402

# Results dir is overridable via STOCHEFF_RESULTS_DIR so cluster runs can write to
# fast/large scratch (e.g. /scratch/alpine/$USER/...) instead of the repo tree.
RESULTS_DIR = os.environ.get("STOCHEFF_RESULTS_DIR", os.path.join(_HERE, "results"))
SAMPLES_DIR = os.path.join(RESULTS_DIR, "samples")
MANIFEST_PATH = os.path.join(RESULTS_DIR, "manifest.json")

# Default base seed; seed for realization i is BASE_SEED + i, shared by arms via
# the config (so the SDE RNG stream and the numpy profile stream are independent
# but reproducible).
BASE_SEED = 1_000_000


# ---------------------------------------------------------------------------
# Configuration grid
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Config:
    """One integrator configuration to be sampled.

    ``arm`` is one of:
      * "sde"       -- stochastic integrator, inline noise (Basilisk)
      * "profile"   -- pre-generated noise + deterministic integrator (Basilisk)
      * "reference" -- independent scipy ground truth (see referenceScipy.py)
    """
    arm: str                 # "sde", "profile", or "reference"
    method: str              # integrator method name (or "scipy" for reference)
    dt: float                # macro step [s] (max_step for the scipy reference)
    relTol: Optional[float] = None    # adaptive tolerance (profile arm only)
    label: str = ""          # human label

    def key(self) -> str:
        """Stable human-readable identifier for this config."""
        base = f"{self.arm}:{self.method}:dt={self.dt:g}"
        if self.relTol is not None:
            base += f":tol={self.relTol:g}"
        return base

    def hash(self, params: model.ScenarioParams) -> str:
        """Content hash combining config + scenario params (cache filename)."""
        payload = json.dumps(
            {"cfg": {"arm": self.arm, "method": self.method, "dt": self.dt,
                     "relTol": self.relTol},
             "params": asdict(params)},
            sort_keys=True,
        )
        return hashlib.sha1(payload.encode()).hexdigest()[:16]


def defaultConfigGrid() -> List[Config]:
    """The standard configuration grid for the study.

    * Arm "sde": each stochastic method over a sweep of (large) macro steps.
      These should reach a given moment-accuracy cheaply.
    * Arm "profile": fixed-step deterministic methods over a sweep of (small)
      steps.  Adaptive RK (RKF45/RKF78) is intentionally excluded from the
      default grid because its embedded error controller misbehaves on the
      piecewise-linear (kinked) replayed forcing; it can still be requested
      explicitly via --only for experimentation.
    """
    configs: List[Config] = []

    # The SDE sweep deliberately reaches down to small dt (1.0, 0.5) as well as
    # up to large dt, so the stochastic arm can be driven into its (tiny)
    # discretization-bias-limited regime and compared against the profile arm on
    # equal footing at tight accuracy targets.
    sdeSteps = [40.0, 20.0, 10.0, 5.0, 2.0, 1.0, 0.5]
    for method in model.SDE_METHODS:
        for dt in sdeSteps:
            configs.append(Config("sde", method, dt))

    detSteps = [8.0, 4.0, 2.0, 1.0, 0.5, 0.25]
    for method in ("RK2", "RK4"):
        for dt in detSteps:
            configs.append(Config("profile", method, dt))

    return configs


def referenceConfig() -> Config:
    """Ground-truth configuration used to estimate the 'true' moments.

    The reference is an *independent* scipy ``solve_ivp`` integration of the
    same planar drag problem with the same prescribed OU density path (see
    ``referenceScipy.py``).  Being completely outside Basilisk -- sharing none of
    its integrators, state plumbing, or task scheduling -- it cannot be fooled by
    an error common to the two Basilisk arms, so both the "sde" and "profile"
    arms are measured against it without circularity.  ``dt`` here denotes the
    scipy ``max_step``; the real accuracy is set by the tight rtol/atol in
    ``referenceScipy.runReferenceRealization``.
    """
    return Config("reference", "scipy", dt=2.0, label="reference")


# ---------------------------------------------------------------------------
# Cache I/O  (delegated to sampleStore, which is shard-aware for cluster runs)
# ---------------------------------------------------------------------------
def _ensureDirs():
    os.makedirs(SAMPLES_DIR, exist_ok=True)


def loadSamples(cfgHash: str) -> dict:
    """Load ALL cached samples for a config (consolidated + every shard)."""
    return sampleStore.loadMerged(SAMPLES_DIR, cfgHash)


def _updateManifest(cfgHash: str, cfg: Config, params: model.ScenarioParams, n: int,
                    isReference: bool = False):
    """Record this config in the manifest index (best-effort, single-writer).

    NOT safe to call from many concurrent array tasks (it is a read-modify-write
    of one shared file).  Sharded cluster runs therefore skip it and instead
    rebuild the manifest once, serially, from the sample files' embedded meta via
    ``rebuildManifest`` (run by the consolidation / analyze step).
    """
    manifest = {}
    if os.path.exists(MANIFEST_PATH):
        try:
            with open(MANIFEST_PATH) as f:
                manifest = json.load(f)
        except (json.JSONDecodeError, OSError):
            manifest = {}
    manifest[cfgHash] = {
        "key": cfg.key(), "arm": cfg.arm, "method": cfg.method,
        "dt": cfg.dt, "relTol": cfg.relTol, "nSamples": n,
        "isReference": bool(isReference or cfg.label == "reference"),
        "params": asdict(params),
    }
    tmp = MANIFEST_PATH + f".tmp.{os.getpid()}"
    with open(tmp, "w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
    os.replace(tmp, MANIFEST_PATH)


def rebuildManifest(params: model.ScenarioParams) -> dict:
    """Reconstruct manifest.json from the sample files on disk.

    Each sample file embeds its config ``meta``; we scan every config hash, merge
    its shards, and write one manifest entry with the merged sample count.  This
    is the concurrency-safe way to get a manifest after a sharded cluster sweep:
    array tasks never touch the shared manifest, this runs once at the end.
    """
    import re
    _ensureDirs()
    # Discover all config hashes from sample filenames.
    hashes = set()
    for p in glob.glob(os.path.join(SAMPLES_DIR, "*.npz")):
        name = os.path.basename(p)
        mobj = re.match(r"([0-9a-f]{16})(?:__shard\d+)?\.npz$", name)
        if mobj:
            hashes.add(mobj.group(1))

    manifest = {}
    for cfgHash in sorted(hashes):
        merged = sampleStore.loadMerged(SAMPLES_DIR, cfgHash)
        if merged["seeds"].size == 0:
            continue
        meta = _readMetaForHash(cfgHash)
        if meta is None:
            continue
        manifest[cfgHash] = {
            "key": meta.get("key"), "arm": meta.get("arm"),
            "method": meta.get("method"), "dt": meta.get("dt"),
            "relTol": meta.get("relTol"), "nSamples": int(merged["seeds"].size),
            "isReference": bool(meta.get("arm") == "reference"),
            "params": meta.get("params", asdict(params)),
        }
    tmp = MANIFEST_PATH + f".tmp.{os.getpid()}"
    with open(tmp, "w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
    os.replace(tmp, MANIFEST_PATH)
    return manifest


def _readMetaForHash(cfgHash: str) -> Optional[dict]:
    """Read the embedded meta JSON from any sample file for this hash."""
    candidates = [sampleStore.consolidatedPath(SAMPLES_DIR, cfgHash)]
    candidates += sampleStore.shardGlob(SAMPLES_DIR, cfgHash)
    for p in candidates:
        if not os.path.exists(p):
            continue
        try:
            with np.load(p) as d:
                if "meta" in d.files:
                    return json.loads(str(d["meta"]))
        except (OSError, ValueError, EOFError):
            continue
    return None


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------
def runConfig(cfg: Config, params: model.ScenarioParams, nSamples: int,
              baseSeed: int = BASE_SEED, saveEvery: int = 10,
              verbose: bool = True, shardIndex: Optional[int] = None,
              nShards: int = 1) -> dict:
    """Run/extend one config to ``nSamples`` realizations, caching as we go.

    Resumable: existing cached samples for this shard are kept; only the missing
    realizations are simulated.  Seeds are always ``baseSeed + i`` for the global
    index ``i`` (regardless of sharding), so sharded and non-sharded caches are
    interchangeable and shards never overlap.

    Sharding (for Slurm job arrays): when ``nShards > 1`` and ``shardIndex`` is
    set, this task only computes its contiguous block of the ``[0, nSamples)``
    index range and writes to its own ``<hash>__shardNNNN.npz`` file -- no write
    contention with sibling tasks, and the shared manifest is NOT touched (rebuild
    it once afterward with ``rebuildManifest``).
    """
    _ensureDirs()
    cfgHash = cfg.hash(params)

    # Which global indices is this task responsible for?
    if nShards > 1 and shardIndex is not None:
        idxRange = sampleStore.seedRangeForShard(0, nSamples, shardIndex, nShards)
        targetIndices = list(idxRange)             # global indices i
    else:
        targetIndices = list(range(nSamples))

    # Load only THIS shard's prior samples (so resume is per-shard).
    record = sampleStore.loadShard(SAMPLES_DIR, cfgHash, shardIndex if nShards > 1 else None)
    haveSeeds = set(int(s) for s in record["seeds"])
    todo = [i for i in targetIndices if (baseSeed + i) not in haveSeeds]

    label = cfg.key() + (f" [shard {shardIndex}/{nShards}]" if nShards > 1 else "")
    if not todo:
        if verbose:
            print(f"[skip] {label} already complete ({len(haveSeeds)} samples)")
        return record
    if verbose:
        print(f"[run ] {label}  (+{len(todo)} samples)  hash={cfgHash}")

    seeds = list(record["seeds"]); fomA = list(record["fomA"])
    fomAlt = list(record["fomAlt"]); wall = list(record["wall"])
    nSteps = list(record["nSteps"])

    meta = {"key": cfg.key(), "arm": cfg.arm, "method": cfg.method,
            "dt": cfg.dt, "relTol": cfg.relTol, "params": asdict(params)}

    # The scipy reference reads its atmosphere parameters once (avoids rebuilding
    # the Basilisk model every realization).
    refAtmo = referenceScipy._readAtmoParams() if cfg.arm == "reference" else None

    def flush():
        rec = {"seeds": np.array(seeds, dtype=np.int64),
               "fomA": np.array(fomA), "fomAlt": np.array(fomAlt),
               "wall": np.array(wall), "nSteps": np.array(nSteps, dtype=np.int64)}
        sampleStore.saveShard(SAMPLES_DIR, cfgHash,
                              shardIndex if nShards > 1 else None, rec, meta)
        # Only single-shard (single-machine) runs maintain the manifest live;
        # sharded array tasks rebuild it once at the end to avoid write races.
        if nShards == 1:
            _updateManifest(cfgHash, cfg, params, len(seeds),
                            isReference=(cfg.label == "reference"))
        return rec

    for done, i in enumerate(todo):
        seed = baseSeed + i
        if cfg.arm == "reference":
            res = referenceScipy.runReferenceRealization(
                params, seed, maxStep=cfg.dt, _atmo=refAtmo)
        else:
            res = model.runRealization(
                params, cfg.arm, cfg.method, cfg.dt, seed, relTol=cfg.relTol)
        seeds.append(seed); fomA.append(res.fomSemiMajorAxis)
        fomAlt.append(res.fomAltitude); wall.append(res.wallSeconds)
        nSteps.append(res.nSteps)

        if (done + 1) % saveEvery == 0 or (done + 1) == len(todo):
            rec = flush()
            if verbose:
                med = np.median(rec["wall"])
                print(f"       {done + 1}/{len(todo)}  n={len(seeds)}  "
                      f"a_std={np.std(rec['fomA']):.4g} m  median wall={med:.4g} s")

    return record if not todo else flush()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nSamples", type=int, default=200,
                        help="target number of realizations per config")
    parser.add_argument("--baseSeed", type=int, default=BASE_SEED)
    parser.add_argument("--orbits", type=float, default=None,
                        help="override number of orbital periods (smaller = faster)")
    parser.add_argument("--stationaryStd", type=float, default=None,
                        help="override OU stationary std (larger = stronger noise)")
    parser.add_argument("--only", type=str, default=None,
                        help="run a single config by its key, e.g. 'sde:W2Ito2:dt=20'")
    parser.add_argument("--reference", action="store_true",
                        help="run the high-fidelity reference config")
    parser.add_argument("--includeReference", action="store_true",
                        help="include the reference config in the task list / grid run")
    parser.add_argument("--list", action="store_true",
                        help="print the config grid and exit")
    parser.add_argument("--saveEvery", type=int, default=10)
    # Sharding / cluster (Slurm job arrays)
    parser.add_argument("--nShards", type=int, default=1,
                        help="split each config's samples into this many shards")
    parser.add_argument("--shardIndex", type=int, default=None,
                        help="with --only: which shard [0, nShards) this task runs")
    parser.add_argument("--taskListIndex", type=int, default=None,
                        help="cluster entry point: run the single (config, shard) "
                             "pair at this flat index of the task list "
                             "(see --printTaskCount). Maps SLURM_ARRAY_TASK_ID.")
    parser.add_argument("--printTaskCount", action="store_true",
                        help="print the number of (config, shard) tasks and exit "
                             "(for sizing a Slurm --array)")
    parser.add_argument("--rebuildManifest", action="store_true",
                        help="rebuild manifest.json from sample files (run once "
                             "after a sharded sweep) and exit")
    parser.add_argument("--consolidate", action="store_true",
                        help="fold per-shard files into one file per config and exit")
    # Plan-driven mode (precision-targeted budgets; see planBudget.py)
    parser.add_argument("--planFile", type=str, default=None,
                        help="JSON plan from planBudget.py giving a per-config "
                             "target N and shard count (incl. the reference). "
                             "When set, the task list and sample counts come from "
                             "the plan, not from --nSamples/--nShards.")
    args = parser.parse_args(argv)

    paramKwargs = {}
    if args.orbits is not None:
        paramKwargs["orbits"] = args.orbits
    if args.stationaryStd is not None:
        paramKwargs["stationaryStd"] = args.stationaryStd
    params = model.ScenarioParams(**paramKwargs)

    # --- maintenance modes (serial, run once) -------------------------------
    if args.rebuildManifest:
        m = rebuildManifest(params)
        print(f"Rebuilt manifest with {len(m)} configs at {MANIFEST_PATH}")
        return 0
    if args.consolidate:
        n = 0
        for cfgHash in _allHashesOnDisk():
            meta = _readMetaForHash(cfgHash) or {}
            sampleStore.consolidate(SAMPLES_DIR, cfgHash, meta)
            n += 1
        rebuildManifest(params)
        print(f"Consolidated {n} configs and rebuilt the manifest.")
        return 0

    # --- load a precision-targeted plan if given ----------------------------
    plan = None
    if args.planFile is not None:
        with open(args.planFile) as f:
            plan = json.load(f)

    # --- determine which configs are in scope -------------------------------
    allConfigs = defaultConfigGrid() + [referenceConfig()]
    if plan is not None:
        # The plan enumerates exactly which configs to run (by key), including
        # the reference, each with its own target N and shard count.
        byKey = {c.key(): c for c in allConfigs}
        grid = [byKey[k] for k in plan["configs"].keys() if k in byKey]
    else:
        grid = defaultConfigGrid()
        if args.includeReference:
            grid = grid + [referenceConfig()]
        if args.reference:
            grid = [referenceConfig()]
        if args.only is not None:
            grid = [c for c in allConfigs if c.key() == args.only]
            if not grid:
                print(f"No config matches key '{args.only}'. Use --list to see keys.")
                return 2

    # --- the flat task list for job arrays ----------------------------------
    # Each entry is (config, shardIndex, nShards, nSamples).  In plan mode the
    # per-config N and shard count come from the plan (so the reference IS
    # shardable -- it needs millions of samples); otherwise a uniform --nShards
    # applies and the reference stays a single un-sharded task.
    def taskList():
        tasks = []
        for c in grid:
            if plan is not None:
                spec = plan["configs"][c.key()]
                nsh = int(spec["nShards"])
                nsamp = int(spec["nSamples"])
                if nsh <= 1:
                    tasks.append((c, None, 1, nsamp))
                else:
                    for s in range(nsh):
                        tasks.append((c, s, nsh, nsamp))
            else:
                nsamp = args.nSamples
                if c.arm == "reference":
                    tasks.append((c, None, 1, nsamp))
                else:
                    for s in range(args.nShards):
                        tasks.append((c, s, args.nShards, nsamp))
        return tasks

    if args.printTaskCount:
        print(len(taskList()))
        return 0

    if args.list:
        print(f"Scenario params: {params}")
        tl = taskList()
        print(f"{len(grid)} configs, {len(tl)} tasks"
              + (f" (plan {args.planFile})" if plan else f" (nShards={args.nShards})") + ":")
        for c in grid:
            extra = ""
            if plan is not None:
                spec = plan["configs"][c.key()]
                extra = f"  N={spec['nSamples']:g} x{spec['nShards']}shards"
            print(f"  {c.key():32s}  hash={c.hash(params)}{extra}")
        return 0

    t0 = time.perf_counter()
    if args.taskListIndex is not None:
        tl = taskList()
        if not (0 <= args.taskListIndex < len(tl)):
            print(f"taskListIndex {args.taskListIndex} out of range [0,{len(tl)})")
            return 2
        cfg, shard, nsh, nsamp = tl[args.taskListIndex]
        runConfig(cfg, params, nsamp, baseSeed=args.baseSeed,
                  saveEvery=args.saveEvery, shardIndex=shard, nShards=nsh)
    else:
        for cfg, shard, nsh, nsamp in taskList():
            runConfig(cfg, params, nsamp, baseSeed=args.baseSeed,
                      saveEvery=args.saveEvery, shardIndex=shard, nShards=nsh)
    print(f"Done in {time.perf_counter() - t0:.1f} s. "
          f"Samples cached under {SAMPLES_DIR}")
    return 0


def _allHashesOnDisk():
    """All config hashes that have any sample file on disk."""
    import re
    hashes = set()
    for p in glob.glob(os.path.join(SAMPLES_DIR, "*.npz")):
        mobj = re.match(r"([0-9a-f]{16})(?:__shard\d+)?\.npz$", os.path.basename(p))
        if mobj:
            hashes.add(mobj.group(1))
    return sorted(hashes)


if __name__ == "__main__":
    raise SystemExit(main())
