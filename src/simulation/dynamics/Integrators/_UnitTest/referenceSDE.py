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
"""
Reconstruct a reference case's SDE from the coefficient spec carried in its JSON.

The Julia generator (juliaReference/generate_reference.jl) serializes each toy SDE as a
``drift`` + ``diffusion`` coefficient spec rather than leaving the drift/diffusion to be
re-typed in Python. This module is the Python-side evaluator of that spec: given a case
dict it returns the drift ``f(t, x)`` and the list of per-source diffusion columns
``g_k(t, x)`` that the Basilisk test feeds to the integrator. The SDE is thus defined once
(in the generator) and interpreted identically on both sides.
"""
from __future__ import annotations

import math
from typing import Callable, List

import numpy as np


def _timeFactor(name: str) -> Callable[[float], float]:
    """The tiny fixed vocabulary of time factors, matching timefactor() in the Julia
    generator. A coefficient is multiplied by the named factor evaluated at time t."""
    if name == "one":
        return lambda t: 1.0
    if name == "1+t":
        return lambda t: 1.0 + t
    if name == "1+0.5t":
        return lambda t: 1.0 + 0.5 * t
    if name == "0.05_over_sqrt_1pt":
        return lambda t: 0.1 * 0.05 / math.sqrt(1.0 + t)
    raise NotImplementedError(f"unknown time factor {name!r}")


def makeDrift(case: dict) -> Callable:
    """Return the drift f(t, x) reconstructed from the case's ``drift`` spec."""
    spec = case["drift"]
    kind = spec["kind"]
    if kind == "linear":
        A = np.array(spec["matrix"], dtype=float)
        tf = _timeFactor(spec["timeFactor"])
        return lambda t, x: tf(t) * (A @ np.asarray(x, dtype=float))
    if kind == "ou_additive_td":
        # Special Ornstein-Uhlenbeck-style additive drift (scalar):
        #   f = 0.05/sqrt(1+t) - x/(2(1+t))
        return lambda t, x: np.array(
            [0.05 / math.sqrt(1.0 + t) - np.asarray(x, dtype=float)[0] / (2.0 * (1.0 + t))]
        )
    raise NotImplementedError(f"unknown drift kind {kind!r}")


def makeDiffusionColumns(case: dict) -> List[Callable]:
    """Return the list of per-source diffusion columns g_k(t, x) from the ``diffusion``
    spec. Column k is a length-n vector; for diagonal noise only its k-th entry is
    nonzero (which is how the Basilisk test wires source k to state k)."""
    spec = case["diffusion"]
    kind = spec["kind"]
    tf = _timeFactor(spec["timeFactor"])
    columns = spec["columns"]

    if kind == "state_linear":
        # columns[k] is an n x n matrix B_k; the column is B_k @ x.
        mats = [np.array(B, dtype=float) for B in columns]
        return [
            (lambda t, x, B=B: tf(t) * (B @ np.asarray(x, dtype=float)))
            for B in mats
        ]
    if kind == "additive":
        # columns[k] is a constant length-n vector.
        vecs = [np.array(c, dtype=float) for c in columns]
        return [
            (lambda t, x, c=c: tf(t) * c)
            for c in vecs
        ]
    raise NotImplementedError(f"unknown diffusion kind {kind!r}")
