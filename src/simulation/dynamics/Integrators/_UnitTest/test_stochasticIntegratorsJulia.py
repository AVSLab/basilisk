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
Numerical-equivalence tests for the native Basilisk stochastic integrators against
StochasticDiffEq.jl (SciML).

The reference trajectories in ``juliaReference/reference_trajectories.json`` were
produced by ``juliaReference/generate_reference.jl`` using StochasticDiffEq.jl. That
generator draws a fixed sequence of Wiener increments (dW, and for the Roessler methods a
second increment dZ) and solves each toy SDE with a prescribed NoiseGrid so the increments
are deterministic inputs rather than RNG outputs. It covers the full range of noise
structures: scalar, diagonal, additive, non-diagonal (coupled), and time-dependent
coefficients.

Here we replay the *identical* increments through the corresponding Basilisk integrator
(via a ``PrescribedGaussianNoiseGenerator``) and assert the resulting trajectory matches
the reference to floating-point tolerance. Because the integrators consume the same
increments, agreement shows the Basilisk implementation reproduces the reference algorithm
exactly (hence also its convergence order).

The SDE for each case is not re-typed here: the generator serializes a drift/diffusion
coefficient spec into the JSON, and ``referenceSDE`` rebuilds f and g from it, so the SDE
is defined once (in the generator).

The Julia reference is generated offline and committed; it is not run in CI. See
``juliaReference/README.md`` for how to regenerate it.
"""
from __future__ import annotations

import json
import os

import numpy as np
import numpy.testing
import pytest

from Basilisk.utilities import macros
from Basilisk.simulation import svIntegrators

try:
    from Basilisk.simulation import mujoco

    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

REFERENCE_FILE = os.path.join(
    os.path.dirname(__file__), "juliaReference", "reference_trajectories.json"
)

# Map the method name stored in the reference JSON to the Basilisk integrator class.
INTEGRATOR_CLASSES = {
    "SRIW1": "svStochasticIntegratorSRIW1",
    "SOSRI": "svStochasticIntegratorSOSRI",
    "SRA1": "svStochasticIntegratorSRA1",
    "SOSRA": "svStochasticIntegratorSOSRA",
    "EulerHeun": "svStochasticIntegratorEulerHeun",
    "RKMil": "svStochasticIntegratorRKMil",
    # Weak-order-2 Tocino & Vigo-Aguiar family
    "SIEA": "svStochasticIntegratorSIEA",
    "SMEA": "svStochasticIntegratorSMEA",
    "SIEB": "svStochasticIntegratorSIEB",
    "SMEB": "svStochasticIntegratorSMEB",
    "RDI1WM": "svStochasticIntegratorRDI1WM",
    "DRI1": "svStochasticIntegratorDRI1",
    "DRI1NM": "svStochasticIntegratorDRI1NM",
    "RI1": "svStochasticIntegratorRI1",
    "RI3": "svStochasticIntegratorRI3",
    "RI5": "svStochasticIntegratorRI5",
    "RI6": "svStochasticIntegratorRI6",
    "RS1": "svStochasticIntegratorRS1",
    "RS2": "svStochasticIntegratorRS2",
    "W2Ito1": "svStochasticIntegratorW2Ito1",
}


def _buildSim(case: dict):
    """Build a Basilisk MuJoCo simulation for one reference case.

    The drift f and diffusion columns g_k are reconstructed from the case's serialized
    coefficient spec (see referenceSDE), so they are not duplicated here. Returns
    (scSim, stateModel, integrator, stateLogger).
    """
    from stochasticIntegratorHarness import buildPrescribedNoiseSim
    from referenceSDE import makeDrift, makeDiffusionColumns

    f = makeDrift(case)
    gCols = makeDiffusionColumns(case)
    return buildPrescribedNoiseSim(case, INTEGRATOR_CLASSES[case["method"]], f, gCols)


def _referenceCases():
    """Yield (id, case) for parametrization; returns [] if reference missing."""
    if not os.path.exists(REFERENCE_FILE):
        return []
    with open(REFERENCE_FILE) as f:
        data = json.load(f)
    return [(f"{c['method']}-{c['problem']}", c) for c in data["cases"]]


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("caseId,case", _referenceCases())
def test_juliaEquivalence(caseId: str, case: dict):
    """Basilisk must reproduce the StochasticDiffEq.jl trajectory for the same noise."""
    scSim, stateModel, integrator, stateLogger = _buildSim(case)

    tf = case["tf"]
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    # Normalize both to shape (n_saved, n_states); the logger squeezes a single
    # scalar state to 1-D, while the reference is always stored 2-D.
    n = case["n_states"]
    xBasilisk = np.array(stateLogger.x, dtype=float).reshape(-1, n)
    reference = np.array(case["trajectory"], dtype=float).reshape(-1, n)

    # The logger records the state at each task step (including the initial state),
    # so it holds n_steps+1 samples matching the reference grid.
    assert xBasilisk.shape[0] == reference.shape[0], (
        f"{caseId}: Basilisk produced {xBasilisk.shape[0]} samples, "
        f"reference has {reference.shape[0]}"
    )

    numpy.testing.assert_allclose(
        xBasilisk,
        reference,
        rtol=0,
        atol=1e-11,
        err_msg=f"{caseId}: Basilisk trajectory does not match Julia reference",
    )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
