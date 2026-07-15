# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import gc
import inspect
import os
import warnings

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import fuelTank
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import linearSpringMassDamper
from Basilisk.simulation import spacecraft
from Basilisk.simulation import thrusterDynamicEffector, thrusterStateEffector
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import deprecated
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import simHelpers

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


def _usesExpiredDeprecation(accessor, message):
    """Return true if deprecated public access has reached the urgent warning phase."""
    expectedWarnings = (deprecated.BSKDeprecationWarning, deprecated.BSKUrgentDeprecationWarning)
    with pytest.warns(expectedWarnings, match=message) as warningList:
        accessor()
    return any(issubclass(w.category, deprecated.BSKUrgentDeprecationWarning) for w in warningList)


def _configureTankGeometry(tankModel):
    if hasattr(tankModel, "radiusTankInit"):
        tankModel.radiusTankInit = 1.0  # [m]
    if hasattr(tankModel, "lengthTank"):
        tankModel.lengthTank = 1.0  # [m]


@pytest.mark.parametrize("thrusterConstructor", [thrusterDynamicEffector.ThrusterDynamicEffector,
                                                 thrusterStateEffector.ThrusterStateEffector])
def test_massDepletionTest(show_plots, thrusterConstructor):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # [s]
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # add thruster devices
    thFactory = simIncludeThruster.thrusterFactory()
    thFactory.create(
        'TEST_Thruster',
        [1, 0, 0],  # location in B-frame
        [0, 1, 0]  # direction in B-frame
    )

    # create thruster object container and tie to spacecraft object
    thrustersEffector = thrusterConstructor()
    thFactory.addToSpacecraft("Thrusters", thrustersEffector, scObject)

    unitTestSim.fuelTankStateEffector = fuelTank.FuelTank()
    tankModel = fuelTank.FuelTankModelConstantVolume()
    unitTestSim.fuelTankStateEffector.setTankModel(tankModel)
    initialFuelMass = 40.0  # [kg]
    tankModel.propMassInit = initialFuelMass
    tankModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]
    unitTestSim.fuelTankStateEffector.setR_TB_B([[0.0], [0.0], [0.0]])  # [m]
    tankModel.radiusTankInit = 46.0 / 2.0 / 3.2808399 / 12.0

    # Add tank
    scObject.addStateEffector(unitTestSim.fuelTankStateEffector)
    unitTestSim.fuelTankStateEffector.addThrusterSet(thrustersEffector)

    # set thruster commands
    ThrustMessage = messaging.THRArrayOnTimeCmdMsgPayload()
    ThrustMessage.OnTimeRequest = [9.9]
    thrCmdMsg = messaging.THRArrayOnTimeCmdMsg().write(ThrustMessage)
    thrustersEffector.cmdsInMsg.subscribeTo(thrCmdMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.fuelTankStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, thrustersEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.planetName = "earth_planet_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15  # meters
    unitTestSim.earthGravBody.isCentralBody = True

    scObject.gravField.gravBodies = spacecraft.GravBodyVector([unitTestSim.earthGravBody])

    dataLog = scObject.scStateOutMsg.recorder()
    fuelLog = unitTestSim.fuelTankStateEffector.fuelTankOutMsg.recorder()
    thrLog = thrustersEffector.thrusterOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    unitTestSim.AddModelToTask(unitTaskName, fuelLog)
    unitTestSim.AddModelToTask(unitTaskName, thrLog)

    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]

    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    unitTestSim.InitializeSimulation()

    posRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubPosition)
    sigmaRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubSigma)

    stopTime = 60.0 * 10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()
    orbAngMom_N = simHelpers.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = simHelpers.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = simHelpers.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)

    thrust = thrLog.thrustForce_B
    thrustPercentage = thrLog.thrustFactor
    fuelMass = fuelLog.fuelMass
    fuelMassDot = fuelLog.fuelMassDot

    plt.close("all")
    plt.figure(1)
    plt.plot(orbAngMom_N[:, 0] * 1e-9, orbAngMom_N[:, 1] - orbAngMom_N[0, 1], orbAngMom_N[:, 0] * 1e-9,
             orbAngMom_N[:, 2] - orbAngMom_N[0, 2], orbAngMom_N[:, 0] * 1e-9, orbAngMom_N[:, 3] - orbAngMom_N[0, 3])
    plt.title("Change in Orbital Angular Momentum")
    plt.figure(2)
    plt.plot(rotAngMom_N[:, 0] * 1e-9, rotAngMom_N[:, 1] - rotAngMom_N[0, 1], rotAngMom_N[:, 0] * 1e-9,
             rotAngMom_N[:, 2] - rotAngMom_N[0, 2], rotAngMom_N[:, 0] * 1e-9, rotAngMom_N[:, 3] - rotAngMom_N[0, 3])
    plt.title("Change in Rotational Angular Momentum")
    plt.figure(3)
    plt.plot(rotEnergy[:, 0] * 1e-9, rotEnergy[:, 1] - rotEnergy[0, 1])
    plt.title("Change in Rotational Energy")
    plt.figure(4)
    plt.plot(thrLog.times() * 1e-9, thrust[:, 0], thrLog.times() * 1e-9, thrust[:, 1], thrLog.times() * 1e-9,
             thrust[:, 2])
    plt.xlim([0, 20])
    plt.ylim([0, 1])
    plt.title("Thrust")
    plt.figure(5)
    plt.plot(thrLog.times() * 1e-9, thrustPercentage)
    plt.xlim([0, 20])
    plt.ylim([0, 1.1])
    plt.title("Thrust Percentage")
    plt.figure(6)
    plt.plot(fuelLog.times() * 1e-9, fuelMass)
    plt.xlim([0, 20])
    plt.title("Fuel Mass")
    plt.figure(7)
    plt.plot(fuelLog.times() * 1e-9, fuelMassDot)
    plt.xlim([0, 20])
    plt.title("Fuel Mass Dot")

    if show_plots:
        plt.show()
        plt.close('all')

    dataPos = posRef.getState()
    dataSigma = sigmaRef.getState()
    dataPos = [[dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    dataSigma = [[dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]]

    if thrustersEffector.__class__.__name__ == "ThrusterDynamicEffector":
        truePos = [[-6.7815933935338277e+06, 4.9468685979815889e+06, 5.4867416696776701e+06]]
        trueSigma = [[1.4401781243854264e-01, -6.4168702021364002e-02, 3.0166086824900967e-01]]
    elif thrustersEffector.__class__.__name__ == "ThrusterStateEffector":
        truePos = [[-6781593.400948599, 4946868.619447934, 5486741.690842073]]
        trueSigma = [[0.14366625871003397, -0.06488330854626220, 0.3032637107362375]]

    for i in range(0, len(truePos)):
        np.testing.assert_allclose(dataPos[i], truePos[i], rtol=1e-6, err_msg="Thruster position not equal")

    for i in range(0, len(trueSigma)):
        # check a vector values
        np.testing.assert_allclose(dataSigma[i], trueSigma[i], rtol=1e-4, err_msg="Thruster attitude not equal")

    # target value computed from MaxThrust / (EARTH_GRAV * steadyIsp)
    np.testing.assert_allclose(fuelMassDot[100], -0.000403404216123, rtol=1e-3,
                               err_msg="Thruster mass depletion not ramped up")
    np.testing.assert_allclose(fuelMassDot[-1], 0, rtol=1e-12, err_msg="Thruster mass depletion not ramped down")


def test_leakyTank():
    """Module Unit Test"""
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # [s]
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Make Fuel Tank
    unitTestSim.fuelTankStateEffector = fuelTank.FuelTank()
    tankModel = fuelTank.FuelTankModelConstantVolume()
    unitTestSim.fuelTankStateEffector.setTankModel(tankModel)
    initialFuelMass = 40.0  # [kg]
    tankModel.propMassInit = initialFuelMass

    # Add tank
    scObject.addStateEffector(unitTestSim.fuelTankStateEffector)

    # Make the tank leaky
    leakRate = 1e-5  # [kg/s]
    unitTestSim.fuelTankStateEffector.setFuelLeakRate(leakRate)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.fuelTankStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    fuelLog = unitTestSim.fuelTankStateEffector.fuelTankOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, fuelLog)
    unitTestSim.InitializeSimulation()

    stopTime = 1000.0  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    fuelMass = fuelLog.fuelMass
    fuelMassDot = fuelLog.fuelMassDot

    assert np.allclose(fuelMassDot, -leakRate, rtol=1e-6)
    assert np.isclose(fuelMass[-1], initialFuelMass - stopTime * leakRate, rtol=1e-6)


@pytest.mark.parametrize("sloshMass", [0.0, 50.0, 150.0])
def test_sloshParticlesDoNotOverDeplete(sloshMass):
    """Propellant leaving the vehicle must equal the mass flow, regardless of fuel slosh."""
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 1000.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg*m^2]

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("TestProcess")
    testProc.addTask(unitTestSim.CreateNewTask("unitTask", macros.sec2nano(0.1)))

    tank = fuelTank.FuelTank()
    tankModel = fuelTank.FuelTankModelConstantVolume()
    tankModel.propMassInit = 500.0  # [kg] non-sloshing propellant
    tankModel.radiusTankInit = 0.5  # [m]
    tankModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]  # [m]
    tank.setTankModel(tankModel)
    tank.setR_TB_B([[0.0], [0.0], [0.0]])  # [m]

    particles = []
    for pHat in ([[1.0], [0.0], [0.0]], [[0.0], [1.0], [0.0]], [[0.0], [0.0], [1.0]]):
        if sloshMass <= 0.0:
            break
        particle = linearSpringMassDamper.LinearSpringMassDamper()
        particle.k = 100.0  # [N/m]
        particle.c = 1.0  # [N*s/m]
        particle.r_PB_B = [[0.0], [0.0], [0.0]]  # [m]
        particle.pHat_B = pHat
        particle.rhoInit = 0.0  # [m]
        particle.rhoDotInit = 0.0  # [m/s]
        particle.massInit = sloshMass  # [kg]
        tank.pushFuelSloshParticle(particle)
        scObject.addStateEffector(particle)
        particles.append(particle)

    scObject.addStateEffector(tank)
    leakRate = 0.5  # [kg/s]
    tank.setFuelLeakRate(leakRate)

    unitTestSim.AddModelToTask("unitTask", scObject)
    unitTestSim.AddModelToTask("unitTask", tank)
    unitTestSim.InitializeSimulation()

    stopTime = 100.0  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    finalTankMass = scObject.dynManager.getStateObject(tank.getNameOfMassState()).getState()[0][0]
    finalSloshMass = sum(
        scObject.dynManager.getStateObject(p.nameOfMassState).getState()[0][0] for p in particles
    )

    initialPropellant = tankModel.propMassInit + len(particles) * sloshMass  # [kg]
    expelled = initialPropellant - (finalTankMass + finalSloshMass)  # [kg]

    np.testing.assert_allclose(
        expelled, leakRate * stopTime, rtol=1e-6,
        err_msg="propellant shed by the vehicle must equal mDot*t regardless of fuel slosh"
    )


def test_leakyTankInputMessageOverridesSetter():
    """Module Unit Test"""
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # [s]
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Make Fuel Tank
    unitTestSim.fuelTankStateEffector = fuelTank.FuelTank()
    tankModel = fuelTank.FuelTankModelConstantVolume()
    unitTestSim.fuelTankStateEffector.setTankModel(tankModel)
    initialFuelMass = 40.0  # [kg]
    tankModel.propMassInit = initialFuelMass

    # Add tank
    scObject.addStateEffector(unitTestSim.fuelTankStateEffector)

    # Make the tank leaky using the input message to override the configured value
    configuredLeakRate = 1.0e-5  # [kg/s]
    messageLeakRate = 2.0e-5  # [kg/s]
    unitTestSim.fuelTankStateEffector.setFuelLeakRate(configuredLeakRate)
    leakRateMsgPayload = messaging.MassFlowRateMsgPayload()
    leakRateMsgPayload.massFlowRate = messageLeakRate
    leakRateMsg = messaging.MassFlowRateMsg().write(leakRateMsgPayload)
    unitTestSim.fuelTankStateEffector.fuelLeakRateInMsg.subscribeTo(leakRateMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.fuelTankStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    fuelLog = unitTestSim.fuelTankStateEffector.fuelTankOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, fuelLog)
    unitTestSim.InitializeSimulation()

    stopTime = 1000.0  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    fuelMass = fuelLog.fuelMass
    fuelMassDot = fuelLog.fuelMassDot

    assert np.allclose(fuelMassDot, -messageLeakRate, rtol=1e-6)
    assert np.isclose(fuelMass[-1], initialFuelMass - stopTime * messageLeakRate, rtol=1e-6)


@pytest.mark.parametrize("tankModelConstructor", [fuelTank.FuelTankModelConstantVolume,
                                                  fuelTank.FuelTankModelConstantDensity,
                                                  fuelTank.FuelTankModelEmptying,
                                                  fuelTank.FuelTankModelUniformBurn,
                                                  fuelTank.FuelTankModelCentrifugalBurn])
def test_leakyTankRunsOutOfFuel(tankModelConstructor):
    """Module Unit Test"""
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # [s]
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Make Fuel Tank
    unitTestSim.fuelTankStateEffector = fuelTank.FuelTank()
    tankModel = tankModelConstructor()
    _configureTankGeometry(tankModel)
    unitTestSim.fuelTankStateEffector.setTankModel(tankModel)
    initialFuelMass = 1.0e-4  # [kg]
    tankModel.propMassInit = initialFuelMass

    # Add tank
    scObject.addStateEffector(unitTestSim.fuelTankStateEffector)

    # Make the tank leaky
    leakRate = 1.0e-5  # [kg/s]
    unitTestSim.fuelTankStateEffector.setFuelLeakRate(leakRate)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.fuelTankStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    fuelLog = unitTestSim.fuelTankStateEffector.fuelTankOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, fuelLog)
    unitTestSim.InitializeSimulation()

    stopTime = 20.0  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    fuelMass = fuelLog.fuelMass
    fuelMassDot = fuelLog.fuelMassDot
    fuelMassTolerance = 1e-14  # [kg]
    fuelMassDotTolerance = 1e-14  # [kg/s]

    assert np.all(np.isfinite(fuelMass))
    assert np.all(np.isfinite(fuelMassDot))
    assert np.all(fuelMass >= -fuelMassTolerance)
    assert np.isclose(fuelMass[-1], 0.0, atol=fuelMassTolerance)
    assert np.isclose(fuelMassDot[-1], 0.0, atol=fuelMassDotTolerance)
    assert np.any(np.isclose(fuelMassDot, -leakRate, rtol=1e-6))


@pytest.mark.parametrize("tankModelConstructor", [fuelTank.FuelTankModelConstantVolume,
                                                  fuelTank.FuelTankModelConstantDensity,
                                                  fuelTank.FuelTankModelEmptying,
                                                  fuelTank.FuelTankModelUniformBurn,
                                                  fuelTank.FuelTankModelCentrifugalBurn])
def test_tankModelOutlivesPythonReference(tankModelConstructor):
    """Regression test for issue #282.

    The tank model is created in Python and handed to the FuelTank via
    setTankModel(). Because the model is now held by a std::shared_ptr, dropping
    the only Python reference must NOT free the underlying C++ object: the tank
    co-owns it. Before the shared_ptr conversion the C++ member was a raw pointer,
    so this sequence left a dangling pointer (undefined behaviour). Here we drop
    the Python reference (del + gc.collect()) before the simulation runs and
    assert the model still drives correct, finite depletion results.
    """
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.1)  # [s]
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.fuelTankStateEffector = fuelTank.FuelTank()
    tankModel = tankModelConstructor()
    _configureTankGeometry(tankModel)
    tankModel.propMassInit = 1.0e-4  # [kg]
    unitTestSim.fuelTankStateEffector.setTankModel(tankModel)

    # Drop the only Python reference to the model before the sim runs. With a
    # raw pointer this orphaned the C++ object; with shared_ptr the tank keeps it.
    del tankModel
    gc.collect()

    scObject.addStateEffector(unitTestSim.fuelTankStateEffector)
    leakRate = 1.0e-5  # [kg/s]
    unitTestSim.fuelTankStateEffector.setFuelLeakRate(leakRate)

    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.fuelTankStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    fuelLog = unitTestSim.fuelTankStateEffector.fuelTankOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, fuelLog)
    unitTestSim.InitializeSimulation()

    stopTime = 20.0  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    fuelMass = fuelLog.fuelMass
    fuelMassDot = fuelLog.fuelMassDot

    # If the model had been freed, these would be garbage / NaN or the sim would crash.
    assert np.all(np.isfinite(fuelMass))
    assert np.all(np.isfinite(fuelMassDot))
    assert np.any(np.isclose(fuelMassDot, -leakRate, rtol=1e-6))


def test_deprecatedPublicVariables():
    """Module Unit Test"""
    tank = fuelTank.FuelTank()

    dcm_TB = np.eye(3)
    r_TB_B = [[1.0], [2.0], [3.0]]  # [m]
    fuelLeakRate = 2.0e-5  # [kg/s]
    nameOfMassState = "fuelTankMassDeprecated"
    expiredDeprecation = False

    expiredDeprecation |= _usesExpiredDeprecation(
        lambda: setattr(tank, "nameOfMassState", nameOfMassState),
        "setNameOfMassState"
    )
    assert tank.getNameOfMassState() == nameOfMassState
    expiredDeprecation |= _usesExpiredDeprecation(lambda: getattr(tank, "nameOfMassState"), "getNameOfMassState")

    expiredDeprecation |= _usesExpiredDeprecation(lambda: setattr(tank, "dcm_TB", dcm_TB), "setDcm_TB")
    np.testing.assert_allclose(tank.getDcm_TB(), dcm_TB, rtol=1e-15)
    expiredDeprecation |= _usesExpiredDeprecation(lambda: getattr(tank, "dcm_TB"), "getDcm_TB")

    expiredDeprecation |= _usesExpiredDeprecation(lambda: setattr(tank, "r_TB_B", r_TB_B), "setR_TB_B")
    np.testing.assert_allclose(tank.getR_TB_B(), r_TB_B, rtol=1e-15)
    expiredDeprecation |= _usesExpiredDeprecation(lambda: getattr(tank, "r_TB_B"), "getR_TB_B")

    expiredDeprecation |= _usesExpiredDeprecation(lambda: setattr(tank, "updateOnly", False), "setUpdateOnly")
    assert tank.getUpdateOnly() is False
    expiredDeprecation |= _usesExpiredDeprecation(lambda: getattr(tank, "updateOnly"), "getUpdateOnly")

    expiredDeprecation |= _usesExpiredDeprecation(
        lambda: setattr(tank, "fuelLeakRate", fuelLeakRate),
        "setFuelLeakRate"
    )
    assert tank.getFuelLeakRate() == pytest.approx(fuelLeakRate)
    expiredDeprecation |= _usesExpiredDeprecation(lambda: getattr(tank, "fuelLeakRate"), "getFuelLeakRate")

    if expiredDeprecation:
        warnings.warn(
            "The cutoff date for deprecated FuelTank public variable access has passed. "
            "Remove the deprecated direct Python access to nameOfMassState, dcm_TB, r_TB_B, "
            "updateOnly, and fuelLeakRate from fuelTank.i and update this compatibility test.",
            UserWarning,
            stacklevel=1
        )


if __name__ == "__main__":
    test_massDepletionTest(True, thrusterDynamicEffector.ThrusterDynamicEffector)
    test_deprecatedPublicVariables()
