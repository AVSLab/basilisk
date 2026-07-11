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
Numerical-equivalence tests against PAPER-FAITHFUL references for the weak-order-2
integrators whose multi-noise / non-commutative path StochasticDiffEq.jl cannot
reference:

  * RS1 / RS2 (Roessler 2007, Stratonovich) - StochasticDiffEq.jl's in-place RS cache
    has an array-aliasing defect in its multi-noise cross-terms.
  * DRI1 / RI1 / RI3 / RI5 / RI6 (Debrabant-Roessler / Roessler, Ito) -
    StochasticDiffEq.jl's DRI1 and RI non-diagonal branches raise an UndefVarError and
    cannot solve a coupled-noise problem at all. The RI methods share DRI1's step but
    with different coefficient tableaus.

The references (``paperReference/*.json``) are generated directly from the primary
sources and validated against the authors' published weak-error tables / by weak-order
convergence (see paperReference/generate_rs_reference.py and generate_dri1_reference.py).
As with the Julia tests, the reference prescribes the underlying Gaussian increments and
we replay them through Basilisk and require an exact match - exercising the genuinely
non-commutative, coupled-noise paths that the scalar/diagonal Julia tests cannot.
"""
from __future__ import annotations

import json
import os
from typing import List

import numpy as np
import numpy.testing
import pytest

from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import pythonVariableLogger
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import dynParamManager

try:
    from Basilisk.simulation import mujoco

    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

REFERENCE_FILES = [
    os.path.join(os.path.dirname(__file__), "paperReference",
                 "rs_reference_trajectories.json"),
    os.path.join(os.path.dirname(__file__), "paperReference",
                 "dri1_reference_trajectories.json"),
    os.path.join(os.path.dirname(__file__), "paperReference",
                 "w2ito_reference_trajectories.json"),
]

INTEGRATOR_CLASSES = {
    "RS1": "svStochasticIntegratorRS1",
    "RS2": "svStochasticIntegratorRS2",
    "DRI1": "svStochasticIntegratorDRI1",
    "RI1": "svStochasticIntegratorRI1",
    "RI3": "svStochasticIntegratorRI3",
    "RI5": "svStochasticIntegratorRI5",
    "RI6": "svStochasticIntegratorRI6",
    "W2Ito1": "svStochasticIntegratorW2Ito1",
    "W2Ito2": "svStochasticIntegratorW2Ito2",
}


def _drift(problemName: str):
    """Drift f(t, x). MUST match the paper generators."""
    if problemName == "noncommutative2d":  # RS: Roessler SDE (6.2)
        return lambda t, x: np.array([1.25 * x[1] - 1.25 * x[0], 0.25 * x[0] - 0.25 * x[1]])
    if problemName == "commutative2d":  # RS: Roessler SDE (6.1)
        av = 299 / 200
        return lambda t, x: np.array([av * x[0], av * x[1]])
    if problemName in ("dri1_noncommutative2d", "ri1_noncommutative2d"):
        # DRI1/RI1: linear Ito, [B0,B1] != 0 (same problem, different tableau).
        A = np.array([[-0.5, 0.2], [0.1, -0.4]])
        return lambda t, x: A @ np.asarray(x)
    # W2Ito1/W2Ito2 share these problems; the trailing tag names the problem class.
    if problemName.endswith("_noncommutative2d"):  # linear Ito, [B0,B1] != 0
        A = np.array([[-0.5, 0.2], [0.1, -0.4]])
        return lambda t, x: A @ np.asarray(x)
    if problemName.endswith("_diag2d"):  # diagonal 2D linear Ito
        return lambda t, x: np.array([1.01 * x[0], -0.5 * x[1]])
    if problemName.endswith("_scalar"):  # scalar geometric Brownian motion
        return lambda t, x: np.array([1.01 * x[0]])
    raise NotImplementedError(problemName)


def _diffusionColumns(problemName: str):
    """Return the list of per-source diffusion columns b^k(t, x). MUST match the generators."""
    if problemName == "noncommutative2d":
        return [
            lambda t, x: np.array([np.sqrt(3) / 2 * (x[0] - x[1]), 0.0]),
            lambda t, x: np.array([0.5 * (x[0] + x[1]), x[0]]),
        ]
    if problemName == "commutative2d":
        bv = 0.1
        return [
            lambda t, x: np.array([bv * x[0], 0.0]),
            lambda t, x: np.array([0.0, bv * x[1]]),
        ]
    if problemName in ("dri1_noncommutative2d", "ri1_noncommutative2d"):
        B0 = np.array([[0.3, 0.0], [0.0, 0.1]])
        B1 = np.array([[0.0, 0.15], [0.25, 0.0]])
        return [lambda t, x: B0 @ np.asarray(x), lambda t, x: B1 @ np.asarray(x)]
    # W2Ito1/W2Ito2 problems (see generate_w2ito_reference.py).
    if problemName.endswith("_noncommutative2d"):
        B0 = np.array([[0.3, 0.0], [0.0, 0.1]])
        B1 = np.array([[0.0, 0.15], [0.25, 0.0]])
        return [lambda t, x: B0 @ np.asarray(x), lambda t, x: B1 @ np.asarray(x)]
    if problemName.endswith("_diag2d"):
        return [
            lambda t, x: np.array([0.3 * x[0], 0.0]),
            lambda t, x: np.array([0.0, 0.2 * x[1]]),
        ]
    if problemName.endswith("_scalar"):
        return [lambda t, x: np.array([0.87 * x[0]])]
    raise NotImplementedError(problemName)


def _buildSim(case: dict):
    from stochasticIntegratorHarness import buildPrescribedNoiseSim

    return buildPrescribedNoiseSim(
        case, INTEGRATOR_CLASSES[case["method"]],
        _drift(case["problem"]), _diffusionColumns(case["problem"])
    )


def _referenceCases():
    cases = []
    for path in REFERENCE_FILES:
        if not os.path.exists(path):
            continue
        with open(path) as f:
            data = json.load(f)
        cases.extend((f"{c['method']}-{c['problem']}", c) for c in data["cases"])
    return cases


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("caseId,case", _referenceCases())
def test_paperEquivalence(caseId: str, case: dict):
    """Basilisk RS1/RS2/DRI1 must reproduce the paper-faithful multi-noise references."""
    scSim, stateModel, integrator, stateLogger = _buildSim(case)
    scSim.ConfigureStopTime(macros.sec2nano(case["tf"]))
    scSim.ExecuteSimulation()

    n = case["n_states"]
    xBasilisk = np.array(stateLogger.x, dtype=float).reshape(-1, n)
    reference = np.array(case["trajectory"], dtype=float).reshape(-1, n)

    assert xBasilisk.shape[0] == reference.shape[0], (
        f"{caseId}: {xBasilisk.shape[0]} samples vs reference {reference.shape[0]}"
    )
    numpy.testing.assert_allclose(
        xBasilisk, reference, rtol=0, atol=1e-11,
        err_msg=f"{caseId}: Basilisk trajectory does not match the paper reference",
    )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
