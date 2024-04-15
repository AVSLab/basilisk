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

import inspect
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import fuelTank
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import spacecraft
from Basilisk.simulation import thrusterDynamicEffector, thrusterStateEffector
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


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
    testProcessRate = macros.sec2nano(0.1)
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
    tankModel.propMassInit = 40.0
    tankModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]
    unitTestSim.fuelTankStateEffector.r_TB_B = [[0.0], [0.0], [0.0]]
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
    orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)

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
    np.testing.assert_allclose(fuelMassDot[-1],0, rtol=1e-12, err_msg="Thruster mass depletion not ramped down")

if __name__ == "__main__":
    test_massDepletionTest(True, thrusterDynamicEffector.ThrusterDynamicEffector)
