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

#
#   Integrated test for the effector branching verification scripts. Both are run at
#   reduced settings and their reported metrics are constrained, so the branched
#   equations of motion stay verified rather than merely executable.
#

import importlib
import os
import sys
import tempfile

import pytest

pytestmark = pytest.mark.scenarioTest

THIS_FOLDER = os.path.dirname(__file__)
SCENARIO_FOLDER = os.path.join(THIS_FOLDER, "..", "..", "examples", "branchingVerification")
sys.path.append(SCENARIO_FOLDER)


def test_branchingConvergence():
    """Refine the integration step and require the branched path to hold RK4's formal order.

    A stage-inconsistency defect, where the branched wrench is evaluated against the wrong
    intermediate Runge-Kutta state, degrades this order without necessarily violating any
    conservation law, so it is invisible to the companion impulse-momentum check.
    """
    module = importlib.import_module("scenarioBranchingConvergence")
    with tempfile.TemporaryDirectory(prefix="bsk-branchingConvergence-") as resultsDir:
        results = module.run(showPlots=False,
                             dtSweep=(0.05, 0.025, 0.0125, 0.00625),
                             finalTime=4.0,
                             resultsDir=resultsDir)
        assert os.path.isfile(os.path.join(resultsDir, "scenarioBranchingConvergence.svg"))

    slopes = results["slopes"]
    assert set(slopes) == {"spinningBodyNDOF", "spinningBodyTwoDOF", "linearTranslationOneDOF"}
    for host, observables in slopes.items():
        for observable in ("omega_BN_B", "v_BN_N", "sigma_BN", "r_BN_N"):
            order = observables[observable]
            assert 3.5 < order < 4.5, (
                f"{host} {observable} converged at order {order:.2f}, expected RK4's order of 4")


def test_branchingConservation():
    """Require the branched wrench to satisfy the impulse-momentum theorems to rounding.

    A wrong reference frame, moment arm, or sign anywhere in the branched force and torque
    path leaves a residual comparable to the momentum it bounds, which is roughly 1e3 N m s
    and 1e-1 m/s here. The thresholds sit far below that and far above the float64 noise.
    """
    module = importlib.import_module("scenarioBranchingConservation")
    with tempfile.TemporaryDirectory(prefix="bsk-branchingConservation-") as resultsDir:
        results = module.run(showPlots=False,
                             dt=1.0e-3,
                             finalTime=1.0,
                             resultsDir=resultsDir)
        assert os.path.isfile(os.path.join(resultsDir, "scenarioBranchingConservation.svg"))

    for host, residuals in results["maxResiduals"].items():
        assert residuals["angular"] < 1.0e-8, (
            f"{host} angular impulse residual {residuals['angular']:.3e} N m s is too large")
        assert residuals["linear"] < 1.0e-12, (
            f"{host} linear impulse residual {residuals['linear']:.3e} m/s is too large")
