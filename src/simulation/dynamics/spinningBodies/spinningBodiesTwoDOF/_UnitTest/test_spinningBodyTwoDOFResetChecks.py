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
#   Module Name:        spinningBodyTwoDOFStateEffector
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      June 24, 2026
#

"""
Regression tests for issue #469: Reset() configuration checks for effectors.

``SpinningBodyTwoDOFStateEffector::Reset()`` validates that both per-axis DCMs
are proper rotations and both inertia tensors are symmetric and positive
definite. These tests break exactly one precondition at a time from a valid
configuration and assert initialization raises a ``BasiliskError``.
"""

import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import spacecraft, spinningBodyTwoDOFStateEffector
from Basilisk.utilities import SimulationBaseClass, macros


def _validSpinningBody():
    """Build a SpinningBodyTwoDOFStateEffector with a fully valid configuration."""
    spinningBody = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody.mass1 = 100.0  # [kg]
    spinningBody.mass2 = 50.0  # [kg]
    spinningBody.IS1PntSc1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]  # [kg-m^2]
    spinningBody.IS2PntSc2_S2 = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]  # [kg-m^2]
    spinningBody.dcm_S10B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]  # [-] proper rotation
    spinningBody.dcm_S20S1 = [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]  # [-] proper rotation
    spinningBody.s1Hat_S1 = [[1.0], [0.0], [0.0]]  # [-] unit vector
    spinningBody.s2Hat_S2 = [[0.0], [-1.0], [0.0]]  # [-] unit vector
    spinningBody.ModelTag = "spinningBody"
    return spinningBody


# Each case breaks exactly one precondition from the valid baseline above.
RESET_ERROR_CASES = [
    ("dcm_S10B not orthogonal", "dcm_S10B",
     lambda sb: setattr(sb, "dcm_S10B", [[1.0, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])),
    ("dcm_S20S1 left-handed", "dcm_S20S1",
     lambda sb: setattr(sb, "dcm_S20S1", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])),
    ("IS1PntSc1_S1 not symmetric", "IS1PntSc1_S1",
     lambda sb: setattr(sb, "IS1PntSc1_S1", [[100.0, 5.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]])),
    ("IS2PntSc2_S2 not positive definite", "IS2PntSc2_S2",
     lambda sb: setattr(sb, "IS2PntSc2_S2", [[-50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]])),
    ("mass1 negative", "mass1",
     lambda sb: setattr(sb, "mass1", -1.0)),
    ("mass2 negative", "mass2",
     lambda sb: setattr(sb, "mass2", -1.0)),
]


@pytest.mark.parametrize("brokenPrecondition, expectedMessage, breakIt", RESET_ERROR_CASES,
                         ids=[c[0] for c in RESET_ERROR_CASES])
def test_spinningBodyTwoDOF_resetErrors(brokenPrecondition, expectedMessage, breakIt):
    """Reset() must raise a BasiliskError naming the misconfigured quantity."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBody = _validSpinningBody()
    breakIt(spinningBody)

    scObject.addStateEffector(spinningBody)
    unitTestSim.AddModelToTask("testTask", spinningBody)
    unitTestSim.AddModelToTask("testTask", scObject)

    with pytest.raises(BasiliskError, match=expectedMessage):
        unitTestSim.InitializeSimulation()


def test_spinningBodyTwoDOF_resetAcceptsValidConfig():
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


def test_spinningBodyTwoDOF_resetAcceptsMasslessLowerBody():
    """A massless lower body (mass1 == 0) may carry a zero inertia tensor without error."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBody = _validSpinningBody()
    spinningBody.mass1 = 0.0  # [kg] massless lower connector body
    spinningBody.IS1PntSc1_S1 = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]  # [kg-m^2]
    scObject.addStateEffector(spinningBody)
    unitTestSim.AddModelToTask("testTask", spinningBody)
    unitTestSim.AddModelToTask("testTask", scObject)

    unitTestSim.InitializeSimulation()


if __name__ == "__main__":
    test_spinningBodyTwoDOF_resetAcceptsValidConfig()
    test_spinningBodyTwoDOF_resetAcceptsMasslessLowerBody()
    for case in RESET_ERROR_CASES:
        test_spinningBodyTwoDOF_resetErrors(*case)
