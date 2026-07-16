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

"""
Unit tests for the ``Spacecraft.pointMassTranslationalOnly`` option.

The point-mass mode should reproduce the normal ``spacecraft`` translational
motion when no state effectors are attached, while omitting rotational dynamics
from the integrated equations of motion.
"""

import numpy as np
import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import GravityGradientEffector
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import hingedRigidBodyStateEffector
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeGravBody


TEST_TASK_PERIOD = 0.1  # [s]
STOP_TIME = 1.0  # [s]


def _configureHub(scObject):
    """Configure a hub state used by both full and point-mass spacecraft."""

    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # [m]
    scObject.hub.IHubPntBc_B = [  # [kg*m^2]
        [900.0, 0.0, 0.0],
        [0.0, 800.0, 0.0],
        [0.0, 0.0, 600.0],
    ]
    scObject.hub.r_CN_NInit = [[7000.0e3], [0.0], [0.0]]  # [m]
    scObject.hub.v_CN_NInit = [[0.0], [7500.0], [100.0]]  # [m/s]
    scObject.hub.sigma_BNInit = [[0.1], [-0.2], [0.05]]  # [-]
    scObject.hub.omega_BN_BInit = [[0.01], [0.02], [-0.03]]  # [rad/s]


def _runSpacecraft(pointMassMode=False, addExternalForce=False, addGravityGradient=False):
    """Run a short spacecraft simulation and return the state recorder."""

    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTaskName = "unitTask"
    unitProcessName = "testProcess"
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, macros.sec2nano(TEST_TASK_PERIOD)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.pointMassTranslationalOnly = pointMassMode
    _configureHub(scObject)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    dynamicEffectors = []
    if addExternalForce:
        forceEffector = extForceTorque.ExtForceTorque()
        forceEffector.ModelTag = "inertialForce"
        forceEffector.extForce_N = [[12.0], [-7.0], [3.0]]  # [N]
        scObject.addDynamicEffector(forceEffector)
        dynamicEffectors.append(forceEffector)

    if addGravityGradient:
        gravityGradientEffector = GravityGradientEffector.GravityGradientEffector()
        gravityGradientEffector.ModelTag = "gravityGradient"
        gravityGradientEffector.addPlanetName(earth.planetName)
        scObject.addDynamicEffector(gravityGradientEffector)
        dynamicEffectors.append(gravityGradientEffector)

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    for effector in dynamicEffectors:
        unitTestSim.AddModelToTask(unitTaskName, effector)
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(STOP_TIME))
    unitTestSim.ExecuteSimulation()

    return dataLog


def test_spacecraftPointMass_matchesStandardSpacecraftTranslation():
    """Point-mass mode must match normal spacecraft translational motion."""

    fullSpacecraftLog = _runSpacecraft(pointMassMode=False)
    pointMassLog = _runSpacecraft(pointMassMode=True)

    np.testing.assert_allclose(pointMassLog.r_BN_N, fullSpacecraftLog.r_BN_N, rtol=1e-12, atol=1e-6)
    np.testing.assert_allclose(pointMassLog.v_BN_N, fullSpacecraftLog.v_BN_N, rtol=1e-12, atol=1e-9)


def test_spacecraftPointMass_matchesStandardSpacecraftWithDynamicForce():
    """Point-mass mode must include translational dynamic-effector forces."""

    fullSpacecraftLog = _runSpacecraft(pointMassMode=False, addExternalForce=True)
    pointMassLog = _runSpacecraft(pointMassMode=True, addExternalForce=True)

    np.testing.assert_allclose(pointMassLog.r_BN_N, fullSpacecraftLog.r_BN_N, rtol=1e-12, atol=1e-6)
    np.testing.assert_allclose(pointMassLog.v_BN_N, fullSpacecraftLog.v_BN_N, rtol=1e-12, atol=1e-9)


def test_spacecraftPointMass_allowsAttitudeDependentDynamicEffector():
    """Point-mass mode must expose fixed attitude states for dynamic effectors."""

    pointMassLog = _runSpacecraft(pointMassMode=True)
    pointMassWithGravityGradientLog = _runSpacecraft(pointMassMode=True, addGravityGradient=True)

    np.testing.assert_allclose(pointMassWithGravityGradientLog.r_BN_N, pointMassLog.r_BN_N, rtol=1e-12, atol=1e-6)
    np.testing.assert_allclose(pointMassWithGravityGradientLog.v_BN_N, pointMassLog.v_BN_N, rtol=1e-12, atol=1e-9)
    np.testing.assert_allclose(pointMassWithGravityGradientLog.sigma_BN[-1],
                               pointMassWithGravityGradientLog.sigma_BN[0])
    np.testing.assert_allclose(pointMassWithGravityGradientLog.omega_BN_B[-1],
                               pointMassWithGravityGradientLog.omega_BN_B[0])


def test_spacecraftPointMass_rejectsStateEffectors():
    """Point-mass mode must reject state effectors because they add internal states."""

    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTaskName = "unitTask"
    unitProcessName = "testProcess"
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, macros.sec2nano(TEST_TASK_PERIOD)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.pointMassTranslationalOnly = True
    _configureHub(scObject)
    scObject.addStateEffector(hingedRigidBodyStateEffector.HingedRigidBodyStateEffector())
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    with pytest.raises(BasiliskError, match="state effectors"):
        unitTestSim.InitializeSimulation()


if __name__ == "__main__":
    test_spacecraftPointMass_matchesStandardSpacecraftTranslation()
    test_spacecraftPointMass_matchesStandardSpacecraftWithDynamicForce()
    test_spacecraftPointMass_allowsAttitudeDependentDynamicEffector()
    test_spacecraftPointMass_rejectsStateEffectors()
