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
#   Module Name:        spacecraft (hub configuration checks)
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      July 4, 2026
#

"""
Regression tests for issue #469: hub configuration checks on spacecraft reset.

``Spacecraft::Reset()`` validates the user-supplied hub properties before the
dynamics are initialized: the hub mass must be strictly positive and the hub
inertia tensor must be symmetric positive definite. Without these checks a
zero hub mass or a singular hub inertia silently produced NaN states, and a
negative hub mass silently reversed the translational response to applied
forces. These tests start from a valid configuration, break exactly one
precondition at a time, and assert that initialization raises a
``BasiliskError`` naming the offending quantity.
"""

import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass, macros


TEST_TASK_PERIOD = 0.001  # [s]


def _validSpacecraft():
    """Build a Spacecraft with a fully valid hub configuration."""
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]  # [m]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]
    return scObject


def _setZeroHubMass(scObject):
    scObject.hub.mHub = 0.0  # [kg]


def _setNegativeHubMass(scObject):
    scObject.hub.mHub = -750.0  # [kg]


def _setNonSymmetricHubInertia(scObject):
    scObject.hub.IHubPntBc_B = [  # [kg-m^2]
        [900.0, 50.0, 0.0],
        [0.0, 800.0, 0.0],
        [0.0, 0.0, 600.0],
    ]


def _setNegativeHubInertia(scObject):
    scObject.hub.IHubPntBc_B = [  # [kg-m^2]
        [-900.0, 0.0, 0.0],
        [0.0, 800.0, 0.0],
        [0.0, 0.0, 600.0],
    ]


def _setZeroHubInertia(scObject):
    scObject.hub.IHubPntBc_B = [  # [kg-m^2]
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]


# Each case breaks exactly one precondition from the valid baseline above.
RESET_ERROR_CASES = [
    ("mHub zero", "mHub", _setZeroHubMass),
    ("mHub negative", "mHub", _setNegativeHubMass),
    ("IHubPntBc_B not symmetric", "IHubPntBc_B", _setNonSymmetricHubInertia),
    ("IHubPntBc_B not positive definite", "IHubPntBc_B", _setNegativeHubInertia),
    ("IHubPntBc_B zero", "IHubPntBc_B", _setZeroHubInertia),
]


@pytest.mark.parametrize("brokenPrecondition, expectedMessage, breakIt", RESET_ERROR_CASES,
                         ids=[c[0] for c in RESET_ERROR_CASES])
def test_spacecraftHub_resetErrors(brokenPrecondition, expectedMessage, breakIt):
    """Reset() must raise a BasiliskError naming the misconfigured hub quantity."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(TEST_TASK_PERIOD)))

    scObject = _validSpacecraft()
    breakIt(scObject)
    unitTestSim.AddModelToTask("testTask", scObject)

    with pytest.raises(BasiliskError, match=expectedMessage):
        unitTestSim.InitializeSimulation()


def test_spacecraftHub_resetAcceptsValidConfig():
    """A fully valid hub configuration must initialize without raising."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(TEST_TASK_PERIOD)))

    scObject = _validSpacecraft()
    unitTestSim.AddModelToTask("testTask", scObject)

    unitTestSim.InitializeSimulation()


def test_spacecraftHub_resetAcceptsDefaultHub():
    """The constructor defaults (1 kg, identity inertia) must initialize without raising."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(TEST_TASK_PERIOD)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    unitTestSim.AddModelToTask("testTask", scObject)

    unitTestSim.InitializeSimulation()


if __name__ == "__main__":
    test_spacecraftHub_resetAcceptsValidConfig()
