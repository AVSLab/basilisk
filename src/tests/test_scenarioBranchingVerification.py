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
# Basilisk Scenario Integrated Test
#
# Purpose:  Integrated test for scenarioBranchingVerification. The scenario
#           default reproduces the journal-paper configuration; the test runs
#           a shortened step ladder and horizon. Both reported metrics are
#           constrained, so the branched equations of motion stay verified
#           rather than merely executable.
#

import inspect
import os
import sys

import pytest
from Basilisk.utilities import simHelpers

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioBranchingVerification


@pytest.mark.scenarioTest
def test_scenarioBranchingVerification(show_plots):
    """Constrain both verification metrics and persist the figures.

    The refinement study must hold RK4's formal order, which catches a stage-inconsistency
    defect that leaves each step within tolerance. The impulse-momentum residuals must stay
    near the float64 noise floor, which catches a wrong reference frame, moment arm, or sign
    anywhere in the branched force and torque path. A defect of the second kind leaves a
    residual comparable to the momentum it bounds, roughly 1e1 to 1e2 N m s and 1e-2 m/s here.
    """
    figureList, metrics = scenarioBranchingVerification.run(
        show_plots,
        dtSweep=(0.05, 0.025, 0.0125, 0.00625),
        convergenceFinalTime=4.0,
        conservationDt=1.0e-3,
        conservationFinalTime=1.0,
    )
    for pltName, plt in list(figureList.items()):
        simHelpers.saveScenarioFigure(pltName, plt, path)

    slopes = metrics["slopes"]
    assert set(slopes) == {"spinningBodyNDOF", "nHingedRigidBody", "linearTranslationNDOF"}
    for host, observables in slopes.items():
        for observable in ("omega_BN_B", "v_BN_N", "sigma_BN", "r_BN_N"):
            order = observables[observable]
            assert 3.5 < order < 4.5, (
                f"{host} {observable} converged at order {order:.2f}, expected RK4's order of 4")

    for host, residuals in metrics["maxResiduals"].items():
        assert residuals["angular"] < 1.0e-8, (
            f"{host} angular impulse residual {residuals['angular']:.3e} N m s is too large")
        assert residuals["linear"] < 1.0e-12, (
            f"{host} linear impulse residual {residuals['linear']:.3e} m/s is too large")
