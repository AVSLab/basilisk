# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#
#   Unit Test Script
#   Module Name:        spinningBodyNDOFStateEffector
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      June 24, 2026
#

"""
Regression tests for issue #469: Reset() configuration checks for effectors.

``SpinningBodyNDOFStateEffector::Reset()`` iterates over every attached spinning
body and validates that each ``dcm_S0P`` is a proper rotation and each
``ISPntSc_S`` is a symmetric, positive-definite inertia tensor. These tests
break exactly one precondition at a time on an attached body and assert
initialization raises a ``BasiliskError``. (``sHat_S`` is validated by its own
setter, so it cannot reach ``Reset()`` in an invalid state.)
"""

import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import spacecraft, spinningBodyNDOFStateEffector
from Basilisk.utilities import SimulationBaseClass, macros


def _validBody():
    """Build a single SpinningBody with a fully valid configuration."""
    body = spinningBodyNDOFStateEffector.SpinningBody()
    body.setMass(50.0)  # [kg]
    body.setISPntSc_S([[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]])  # [kg-m^2]
    body.setDCM_S0P([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])  # [-] proper rotation
    body.setSHat_S([[0.0], [0.0], [1.0]])  # [-] unit vector
    return body


# Each case breaks exactly one precondition on the attached body.
RESET_ERROR_CASES = [
    ("dcm_S0P not orthogonal", "dcm_S0P",
     lambda b: b.setDCM_S0P([[1.0, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])),
    ("dcm_S0P left-handed", "dcm_S0P",
     lambda b: b.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])),
    ("ISPntSc_S not symmetric", "ISPntSc_S",
     lambda b: b.setISPntSc_S([[100.0, 5.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]])),
    ("ISPntSc_S not positive definite", "ISPntSc_S",
     lambda b: b.setISPntSc_S([[-100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]])),
]


@pytest.mark.parametrize("brokenPrecondition, expectedMessage, breakIt", RESET_ERROR_CASES,
                         ids=[c[0] for c in RESET_ERROR_CASES])
def test_spinningBodyNDOF_resetErrors(brokenPrecondition, expectedMessage, breakIt):
    """Reset() must raise a BasiliskError naming the misconfigured quantity."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    body = _validBody()
    breakIt(body)
    spinningBodyEffector.addSpinningBody(body)

    scObject.addStateEffector(spinningBodyEffector)
    unitTestSim.AddModelToTask("testTask", spinningBodyEffector)
    unitTestSim.AddModelToTask("testTask", scObject)

    with pytest.raises(BasiliskError, match=expectedMessage):
        unitTestSim.InitializeSimulation()


def test_spinningBodyNDOF_resetAcceptsValidConfig():
    """A fully valid configuration must initialize without raising."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    spinningBodyEffector.addSpinningBody(_validBody())

    scObject.addStateEffector(spinningBodyEffector)
    unitTestSim.AddModelToTask("testTask", spinningBodyEffector)
    unitTestSim.AddModelToTask("testTask", scObject)

    unitTestSim.InitializeSimulation()


def test_spinningBodyNDOF_resetAcceptsMasslessConnector():
    """A massless connector body (mass == 0, zero inertia) followed by a massive body
    must initialize without error. This mirrors the branching modeling pattern."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()

    connector = spinningBodyNDOFStateEffector.SpinningBody()
    connector.setMass(0.0)  # [kg] massless connector
    connector.setISPntSc_S([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])  # [kg-m^2]
    connector.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  # [-]
    connector.setSHat_S([[1.0], [0.0], [0.0]])  # [-]
    spinningBodyEffector.addSpinningBody(connector)

    spinningBodyEffector.addSpinningBody(_validBody())

    scObject.addStateEffector(spinningBodyEffector)
    unitTestSim.AddModelToTask("testTask", spinningBodyEffector)
    unitTestSim.AddModelToTask("testTask", scObject)

    unitTestSim.InitializeSimulation()


if __name__ == "__main__":
    test_spinningBodyNDOF_resetAcceptsValidConfig()
    test_spinningBodyNDOF_resetAcceptsMasslessConnector()
    for case in RESET_ERROR_CASES:
        test_spinningBodyNDOF_resetErrors(*case)
