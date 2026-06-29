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
Shard-aware sample cache for the stochastic efficiency study.

The study caches every figure-of-merit sample to disk so runs are resumable and
analysis never re-simulates.  On a single machine one file per config is enough.
On a cluster (Alpine / Slurm), many array tasks compute *different shards of the
same config concurrently*, so we must avoid having two processes write the same
file.  This module solves that with a simple, lock-free convention:

  results/samples/<hash>.npz                 -- consolidated (single-machine, or
                                                after `consolidate`)
  results/samples/<hash>__shardNNNN.npz      -- one file per shard, written by
                                                exactly one task -> no contention

``loadMerged`` transparently concatenates the consolidated file (if any) with all
shard files for a hash, de-duplicating by seed (a seed kept once, first writer
wins).  Because every realization is keyed by a globally-unique integer seed
(``baseSeed + i``), shards never overlap and merging is exact regardless of how
the work was split.  This means:

  * the analysis/plot scripts call ``loadMerged`` and are oblivious to sharding;
  * a half-finished cluster run merges cleanly (resumable at shard granularity);
  * ``consolidate`` can fold shards into the single canonical file afterwards.

The on-disk arrays are always: ``seeds`` (int64), ``fomA``, ``fomAlt``, ``wall``
(float64), ``nSteps`` (int64), plus a ``meta`` JSON string.
"""
from __future__ import annotations

import glob
import json
import os
from typing import Dict, List, Optional

import numpy as np

_ARRAYS = ("seeds", "fomA", "fomAlt", "wall", "nSteps")


def emptyRecord() -> Dict[str, np.ndarray]:
    """A record with no samples."""
    return {"seeds": np.empty(0, dtype=np.int64),
            "fomA": np.empty(0), "fomAlt": np.empty(0),
            "wall": np.empty(0), "nSteps": np.empty(0, dtype=np.int64)}


def consolidatedPath(samplesDir: str, cfgHash: str) -> str:
    return os.path.join(samplesDir, f"{cfgHash}.npz")


def shardPath(samplesDir: str, cfgHash: str, shardIndex: int) -> str:
    return os.path.join(samplesDir, f"{cfgHash}__shard{shardIndex:04d}.npz")


def shardGlob(samplesDir: str, cfgHash: str) -> List[str]:
    return sorted(glob.glob(os.path.join(samplesDir, f"{cfgHash}__shard*.npz")))


def _loadOne(path: str) -> Optional[Dict[str, np.ndarray]]:
    if not os.path.exists(path):
        return None
    try:
        with np.load(path) as d:
            return {k: d[k] for k in _ARRAYS if k in d.files}
    except (OSError, ValueError, EOFError):
        # A shard being written by another task, or a truncated file from a
        # killed job: skip it rather than crash the whole analysis.
        return None


def _concatDedup(records: List[Dict[str, np.ndarray]]) -> Dict[str, np.ndarray]:
    """Concatenate records and keep one row per unique seed (first writer wins)."""
    recs = [r for r in records if r is not None and r["seeds"].size > 0]
    if not recs:
        return emptyRecord()
    merged = {k: np.concatenate([r[k] for r in recs]) for k in _ARRAYS}
    # Stable de-dup by seed: first occurrence wins.
    _, firstIdx = np.unique(merged["seeds"], return_index=True)
    firstIdx = np.sort(firstIdx)
    out = {k: merged[k][firstIdx] for k in _ARRAYS}
    # Sort by seed for determinism.
    order = np.argsort(out["seeds"], kind="stable")
    return {k: out[k][order] for k in _ARRAYS}


def loadMerged(samplesDir: str, cfgHash: str) -> Dict[str, np.ndarray]:
    """Load all samples for a config: consolidated file + every shard, deduped."""
    parts = [_loadOne(consolidatedPath(samplesDir, cfgHash))]
    parts += [_loadOne(p) for p in shardGlob(samplesDir, cfgHash)]
    return _concatDedup(parts)


def loadShard(samplesDir: str, cfgHash: str, shardIndex: Optional[int]) -> Dict[str, np.ndarray]:
    """Load just one shard's prior samples (for resuming that shard).

    If ``shardIndex`` is None, load the consolidated file (single-machine mode).
    """
    if shardIndex is None:
        rec = _loadOne(consolidatedPath(samplesDir, cfgHash))
    else:
        rec = _loadOne(shardPath(samplesDir, cfgHash, shardIndex))
    return rec if rec is not None else emptyRecord()


def saveShard(samplesDir: str, cfgHash: str, shardIndex: Optional[int],
              record: Dict[str, np.ndarray], meta: dict):
    """Atomically write a shard (or the consolidated file if shardIndex is None).

    Atomic = write to a unique temp name then ``os.replace`` (atomic on POSIX),
    so a reader (or a killed job) never sees a half-written file.
    """
    os.makedirs(samplesDir, exist_ok=True)
    path = (consolidatedPath(samplesDir, cfgHash) if shardIndex is None
            else shardPath(samplesDir, cfgHash, shardIndex))
    tmp = path + f".tmp.{os.getpid()}.npz"
    np.savez(tmp,
             seeds=record["seeds"], fomA=record["fomA"], fomAlt=record["fomAlt"],
             wall=record["wall"], nSteps=record["nSteps"],
             meta=np.array(json.dumps(meta)))
    os.replace(tmp, path)


def consolidate(samplesDir: str, cfgHash: str, meta: dict, removeShards: bool = True):
    """Fold all shards (+ existing consolidated) into the single canonical file.

    Run once after a sharded cluster sweep finishes, to leave a clean
    one-file-per-config layout identical to a single-machine run.
    """
    merged = loadMerged(samplesDir, cfgHash)
    saveShard(samplesDir, cfgHash, None, merged, meta)
    if removeShards:
        for p in shardGlob(samplesDir, cfgHash):
            try:
                os.remove(p)
            except OSError:
                pass
    return merged


def seedRangeForShard(baseSeed: int, nSamples: int, shardIndex: int,
                      nShards: int) -> range:
    """Contiguous seed sub-range this shard is responsible for.

    Splitting the global ``baseSeed + [0, nSamples)`` index range into ``nShards``
    near-equal contiguous blocks keeps each realization's seed identical to the
    non-sharded run (so sharded and unsharded caches are interchangeable), and
    guarantees shards never overlap.
    """
    # Block boundaries via integer arithmetic (handles non-divisible nSamples).
    lo = (shardIndex * nSamples) // nShards
    hi = ((shardIndex + 1) * nSamples) // nShards
    return range(baseSeed + lo, baseSeed + hi)
