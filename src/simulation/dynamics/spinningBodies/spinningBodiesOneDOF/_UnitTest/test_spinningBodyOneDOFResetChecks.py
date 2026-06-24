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
#   Module Name:        spinningBodyOneDOFStateEffector
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      June 24, 2026
#

"""
Regression tests for issue #469: Reset() configuration checks for effectors.

``SpinningBodyOneDOFStateEffector::Reset()`` validates that the user-supplied
configuration is physically consistent. These tests start from a valid
configuration, break exactly one precondition at a time, and assert that
initialization raises a ``BasiliskError`` naming the offending quantity.
"""

import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import spacecraft, spinningBodyOneDOFStateEffector
from Basilisk.utilities import SimulationBaseClass, macros


def _validSpinningBody():
    """Build a SpinningBodyOneDOFStateEffector with a fully valid configuration."""
    spinningBody = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
    spinningBody.mass = 50.0  # [kg]
    spinningBody.IPntSc_S = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]  # [kg-m^2]
    spinningBody.dcm_S0B = [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]  # [-] proper rotation
    spinningBody.r_ScS_S = [[1.0], [0.0], [-1.0]]  # [m]
    spinningBody.r_SB_B = [[0.5], [-1.5], [-0.5]]  # [m]
    spinningBody.sHat_S = [[0.0], [-1.0], [0.0]]  # [-] unit vector
    spinningBody.ModelTag = "spinningBody"
    return spinningBody


# Each case breaks exactly one precondition from the valid baseline above.
RESET_ERROR_CASES = [
    ("dcm_S0B not orthogonal", "dcm_S0B",
     lambda sb: setattr(sb, "dcm_S0B", [[1.0, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])),
    ("dcm_S0B left-handed", "dcm_S0B",
     lambda sb: setattr(sb, "dcm_S0B", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])),
    ("IPntSc_S not symmetric", "IPntSc_S",
     lambda sb: setattr(sb, "IPntSc_S", [[50.0, 5.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]])),
    ("IPntSc_S not positive definite", "IPntSc_S",
     lambda sb: setattr(sb, "IPntSc_S", [[-50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]])),
    ("sHat_S not set", "sHat",
     lambda sb: setattr(sb, "sHat_S", [[0.0], [0.0], [0.0]])),
]


@pytest.mark.parametrize("brokenPrecondition, expectedMessage, breakIt", RESET_ERROR_CASES,
                         ids=[c[0] for c in RESET_ERROR_CASES])
def test_spinningBodyOneDOF_resetErrors(brokenPrecondition, expectedMessage, breakIt):
    """Reset() must raise a BasiliskError naming the misconfigured quantity."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]  # [m]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBody = _validSpinningBody()
    breakIt(spinningBody)

    scObject.addStateEffector(spinningBody)
    unitTestSim.AddModelToTask("testTask", spinningBody)
    unitTestSim.AddModelToTask("testTask", scObject)

    with pytest.raises(BasiliskError, match=expectedMessage):
        unitTestSim.InitializeSimulation()


def test_spinningBodyOneDOF_resetAcceptsValidConfig():
    """A fully valid configuration must initialize without raising."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBody = _validSpinningBody()
    scObject.addStateEffector(spinningBody)
    unitTestSim.AddModelToTask("testTask", spinningBody)
    unitTestSim.AddModelToTask("testTask", scObject)

    unitTestSim.InitializeSimulation()


def test_spinningBodyOneDOF_resetAcceptsMasslessZeroInertia():
    """A massless body (mass == 0) may carry a zero inertia tensor without error."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBody = _validSpinningBody()
    spinningBody.mass = 0.0  # [kg] massless connector body
    spinningBody.IPntSc_S = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]  # [kg-m^2]
    scObject.addStateEffector(spinningBody)
    unitTestSim.AddModelToTask("testTask", spinningBody)
    unitTestSim.AddModelToTask("testTask", scObject)

    unitTestSim.InitializeSimulation()


if __name__ == "__main__":
    test_spinningBodyOneDOF_resetAcceptsValidConfig()
    test_spinningBodyOneDOF_resetAcceptsMasslessZeroInertia()
    for case in RESET_ERROR_CASES:
        test_spinningBodyOneDOF_resetErrors(*case)
