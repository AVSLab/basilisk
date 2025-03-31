# ISC License
#
# Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        spinningBodiesNDOF
#   Author:             João Vaz Carneiro
#   Creation Date:      April 2, 2024
#

import inspect
import os
import pytest
import numpy as np
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import spacecraft, spinningBodyNDOFStateEffector, gravityEffector
from Basilisk.architecture import messaging


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("function", ["spinningBodyNoInput"
    , "spinningBodyLockAxis"
    , "spinningBodyCommandedTorque"])
def test_spinningBody(show_plots, function):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft with a four single-axis rotating rigid bodies attached to a rigid hub. The spinning
    body's center of mass is off-center from the spinning axis and the position of the axis is arbitrary. The scenario
    includes gravity acting on both the spacecraft and the effector.

    **Description of Variables Being Tested**

    In this file we are checking the principles of conservation of energy and angular momentum. Both the orbital and
    rotational energy and angular momentum must be maintained when conservative forces like gravity are present.
    Therefore, the values of the variables

    - ``finalOrbAngMom``
    - ``finalOrbEnergy``
    - ``finalRotAngMom``
    - ``finalRotEnergy``

    against their initial values.
    """
    func = globals().get(function)

    if func is None:
        raise ValueError(f"Function '{function}' not found in global scope")

    func(show_plots)


def spinningBodyNoInput(show_plots):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.0001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create two hinged rigid bodies
    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    spinningBodyEffector.ModelTag = "spinningBodyEffector"

    # Define properties of spinning bodies
    spinningBody1 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody1.setMass(np.random.uniform(5.0, 50.0))
    spinningBody1.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody1.setDCM_S0P([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    spinningBody1.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody1.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody1.setSHat_S([[0], [0], [1]])
    spinningBody1.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody1.setThetaDotInit(0.0 * macros.D2R)
    spinningBody1.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody1)

    spinningBody2 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody2.setMass(np.random.uniform(5.0, 50.0))
    spinningBody2.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody2.setDCM_S0P([[0.0, -1.0, 0.0], [0.0, .0, -1.0], [1.0, 0.0, 0.0]])
    spinningBody2.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody2.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody2.setSHat_S([[0], [-1], [0]])
    spinningBody2.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody2.setThetaDotInit(0.0 * macros.D2R)
    spinningBody2.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody2)

    spinningBody3 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody3.setMass(np.random.uniform(5.0, 50.0))
    spinningBody3.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody3.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    spinningBody3.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody3.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody3.setSHat_S([[np.sqrt(1/2)], [np.sqrt(1/2)], [0]])
    spinningBody3.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody3.setThetaDotInit(0.0 * macros.D2R)
    spinningBody3.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody3)

    spinningBody4 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody4.setMass(np.random.uniform(5.0, 50.0))
    spinningBody4.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody4.setDCM_S0P([[0.0, 1.0, 0.0], [0.0, .0, 1.0], [1.0, 0.0, 0.0]])
    spinningBody4.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody4.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody4.setSHat_S([[np.sqrt(1/2)], [-np.sqrt(1/2)], [0]])
    spinningBody4.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody4.setThetaDotInit(0.0 * macros.D2R)
    spinningBody4.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody4)

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBodyEffector)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, spinningBodyEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totRotEnergy", "totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Add states to log
    theta1Data = spinningBodyEffector.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBodyEffector.spinningBodyOutMsgs[1].recorder()
    theta3Data = spinningBodyEffector.spinningBodyOutMsgs[2].recorder()
    theta4Data = spinningBodyEffector.spinningBodyOutMsgs[3].recorder()
    unitTestSim.AddModelToTask(unitTaskName, theta1Data)
    unitTestSim.AddModelToTask(unitTaskName, theta2Data)
    unitTestSim.AddModelToTask(unitTaskName, theta3Data)
    unitTestSim.AddModelToTask(unitTaskName, theta4Data)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Setup and run the simulation
    stopTime = 10000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbEnergy = scObjectLog.totOrbEnergy
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    theta1 = theta1Data.theta
    theta1Dot = theta1Data.thetaDot
    theta2 = theta2Data.theta
    theta2Dot = theta2Data.thetaDot
    theta3 = theta3Data.theta
    theta3Dot = theta3Data.thetaDot
    theta4 = theta4Data.theta
    theta4Dot = theta4Data.thetaDot

    # Setup the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    initialRotEnergy = rotEnergy[0]
    finalRotEnergy = rotEnergy[-1]

    # Plotting
    plt.close("all")
    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Orbital Angular Momentum', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Orbital Energy', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Rotational Angular Momentum', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Rotational Energy', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    plt.clf()
    plt.plot(timeSec, theta1, label=r'$\theta_1$')
    plt.plot(timeSec, theta2, label=r'$\theta_2$')
    plt.plot(timeSec, theta3, label=r'$\theta_3$')
    plt.plot(timeSec, theta4, label=r'$\theta_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Angle')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, theta1Dot, label=r'$\dot{\theta}_1$')
    plt.plot(timeSec, theta2Dot, label=r'$\dot{\theta}_2$')
    plt.plot(timeSec, theta3Dot, label=r'$\dot{\theta}_3$')
    plt.plot(timeSec, theta4Dot, label=r'$\dot{\theta}_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Angle Rate')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy)
    np.testing.assert_allclose(finalRotEnergy, initialRotEnergy, rtol=accuracy)
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy)
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy)


def spinningBodyLockAxis(show_plots):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.0001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create two hinged rigid bodies
    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    spinningBodyEffector.ModelTag = "spinningBodyEffector"

    # Define properties of spinning bodies
    spinningBody1 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody1.setMass(np.random.uniform(5.0, 50.0))
    spinningBody1.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody1.setDCM_S0P([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    spinningBody1.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody1.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody1.setSHat_S([[0], [0], [1]])
    spinningBody1.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody1.setThetaDotInit(np.random.uniform(-1.0, 1.0) * macros.D2R)
    spinningBody1.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody1)

    spinningBody2 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody2.setMass(np.random.uniform(5.0, 50.0))
    spinningBody2.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody2.setDCM_S0P([[0.0, -1.0, 0.0], [0.0, .0, -1.0], [1.0, 0.0, 0.0]])
    spinningBody2.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody2.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody2.setSHat_S([[0], [-1], [0]])
    spinningBody2.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody2.setThetaDotInit(np.random.uniform(-1.0, 1.0) * macros.D2R)
    spinningBody2.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody2)

    spinningBody3 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody3.setMass(np.random.uniform(5.0, 50.0))
    spinningBody3.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody3.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    spinningBody3.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody3.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody3.setSHat_S([[np.sqrt(1/2)], [np.sqrt(1/2)], [0]])
    spinningBody3.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody3.setThetaDotInit(np.random.uniform(-1.0, 1.0) * macros.D2R)
    spinningBody3.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody3)

    spinningBody4 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody4.setMass(np.random.uniform(5.0, 50.0))
    spinningBody4.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody4.setDCM_S0P([[0.0, 1.0, 0.0], [0.0, .0, 1.0], [1.0, 0.0, 0.0]])
    spinningBody4.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody4.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody4.setSHat_S([[np.sqrt(1/2)], [-np.sqrt(1/2)], [0]])
    spinningBody4.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody4.setThetaDotInit(np.random.uniform(-1.0, 1.0) * macros.D2R)
    spinningBody4.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody4)

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBodyEffector)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # create lock message
    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.effectorLockFlag = [1, 0, 0, 1]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    spinningBodyEffector.motorLockInMsg.subscribeTo(lockMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, spinningBodyEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totRotEnergy", "totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Add states to log
    theta1Data = spinningBodyEffector.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBodyEffector.spinningBodyOutMsgs[1].recorder()
    theta3Data = spinningBodyEffector.spinningBodyOutMsgs[2].recorder()
    theta4Data = spinningBodyEffector.spinningBodyOutMsgs[3].recorder()
    unitTestSim.AddModelToTask(unitTaskName, theta1Data)
    unitTestSim.AddModelToTask(unitTaskName, theta2Data)
    unitTestSim.AddModelToTask(unitTaskName, theta3Data)
    unitTestSim.AddModelToTask(unitTaskName, theta4Data)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Setup and run the simulation
    stopTime = 10000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbEnergy = scObjectLog.totOrbEnergy
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    theta1 = theta1Data.theta
    theta1Dot = theta1Data.thetaDot
    theta2 = theta2Data.theta
    theta2Dot = theta2Data.thetaDot
    theta3 = theta3Data.theta
    theta3Dot = theta3Data.thetaDot
    theta4 = theta4Data.theta
    theta4Dot = theta4Data.thetaDot

    # Setup the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    initialRotEnergy = rotEnergy[0]
    finalRotEnergy = rotEnergy[-1]

    # Plotting
    plt.close("all")
    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Orbital Angular Momentum', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Orbital Energy', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Rotational Angular Momentum', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Rotational Energy', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    plt.clf()
    plt.plot(timeSec, theta1, label=r'$\theta_1$')
    plt.plot(timeSec, theta2, label=r'$\theta_2$')
    plt.plot(timeSec, theta3, label=r'$\theta_3$')
    plt.plot(timeSec, theta4, label=r'$\theta_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Angle')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, theta1Dot, label=r'$\dot{\theta}_1$')
    plt.plot(timeSec, theta2Dot, label=r'$\dot{\theta}_2$')
    plt.plot(timeSec, theta3Dot, label=r'$\dot{\theta}_3$')
    plt.plot(timeSec, theta4Dot, label=r'$\dot{\theta}_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Angle Rate')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy)
    np.testing.assert_allclose(finalRotEnergy, initialRotEnergy, rtol=accuracy)
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy)
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy)


def spinningBodyCommandedTorque(show_plots):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.0001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create two hinged rigid bodies
    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    spinningBodyEffector.ModelTag = "spinningBodyEffector"

    # Define properties of spinning bodies
    spinningBody1 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody1.setMass(np.random.uniform(5.0, 50.0))
    spinningBody1.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody1.setDCM_S0P([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    spinningBody1.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody1.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody1.setSHat_S([[0], [0], [1]])
    spinningBody1.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody1.setThetaDotInit(np.random.uniform(-1.0, 1.0) * macros.D2R)
    spinningBody1.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody1)

    spinningBody2 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody2.setMass(np.random.uniform(5.0, 50.0))
    spinningBody2.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody2.setDCM_S0P([[0.0, -1.0, 0.0], [0.0, .0, -1.0], [1.0, 0.0, 0.0]])
    spinningBody2.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody2.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody2.setSHat_S([[0], [-1], [0]])
    spinningBody2.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody2.setThetaDotInit(np.random.uniform(-1.0, 1.0) * macros.D2R)
    spinningBody2.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody2)

    spinningBody3 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody3.setMass(np.random.uniform(5.0, 50.0))
    spinningBody3.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody3.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    spinningBody3.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody3.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody3.setSHat_S([[np.sqrt(1/2)], [np.sqrt(1/2)], [0]])
    spinningBody3.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody3.setThetaDotInit(np.random.uniform(-1.0, 1.0) * macros.D2R)
    spinningBody3.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody3)

    spinningBody4 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody4.setMass(np.random.uniform(5.0, 50.0))
    spinningBody4.setISPntSc_S([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    spinningBody4.setDCM_S0P([[0.0, 1.0, 0.0], [0.0, .0, 1.0], [1.0, 0.0, 0.0]])
    spinningBody4.setR_ScS_S([[np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)],
                              [np.random.uniform(-1.0, 1.0)]])
    spinningBody4.setR_SP_P([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])
    spinningBody4.setSHat_S([[np.sqrt(1/2)], [-np.sqrt(1/2)], [0]])
    spinningBody4.setThetaInit(np.random.uniform(-10.0, 10.0) * macros.D2R)
    spinningBody4.setThetaDotInit(np.random.uniform(-1.0, 1.0) * macros.D2R)
    spinningBody4.setK(np.random.random())
    spinningBodyEffector.addSpinningBody(spinningBody4)

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBodyEffector)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Create the torque message
    cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    cmdArray.motorTorque = [0.1, -0.2, 0.3, -0.15]  # [Nm]
    cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    spinningBodyEffector.motorTorqueInMsg.subscribeTo(cmdMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, spinningBodyEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totRotEnergy", "totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Add states to log
    theta1Data = spinningBodyEffector.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBodyEffector.spinningBodyOutMsgs[1].recorder()
    theta3Data = spinningBodyEffector.spinningBodyOutMsgs[2].recorder()
    theta4Data = spinningBodyEffector.spinningBodyOutMsgs[3].recorder()
    unitTestSim.AddModelToTask(unitTaskName, theta1Data)
    unitTestSim.AddModelToTask(unitTaskName, theta2Data)
    unitTestSim.AddModelToTask(unitTaskName, theta3Data)
    unitTestSim.AddModelToTask(unitTaskName, theta4Data)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Setup and run the simulation
    stopTime = 10000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbEnergy = scObjectLog.totOrbEnergy
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    theta1 = theta1Data.theta
    theta1Dot = theta1Data.thetaDot
    theta2 = theta2Data.theta
    theta2Dot = theta2Data.thetaDot
    theta3 = theta3Data.theta
    theta3Dot = theta3Data.thetaDot
    theta4 = theta4Data.theta
    theta4Dot = theta4Data.thetaDot

    # Setup the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    initialRotEnergy = rotEnergy[0]

    # Plotting
    plt.close("all")
    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Orbital Angular Momentum', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Orbital Energy', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Rotational Angular Momentum', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    ax = plt.axes()
    plt.plot(timeSec, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)', fontsize='18')
    plt.ylabel('Relative Difference', fontsize='18')
    plt.title('Rotational Energy', fontsize='22')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)

    plt.figure()
    plt.clf()
    plt.plot(timeSec, theta1, label=r'$\theta_1$')
    plt.plot(timeSec, theta2, label=r'$\theta_2$')
    plt.plot(timeSec, theta3, label=r'$\theta_3$')
    plt.plot(timeSec, theta4, label=r'$\theta_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Angle')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, theta1Dot, label=r'$\dot{\theta}_1$')
    plt.plot(timeSec, theta2Dot, label=r'$\dot{\theta}_2$')
    plt.plot(timeSec, theta3Dot, label=r'$\dot{\theta}_3$')
    plt.plot(timeSec, theta4Dot, label=r'$\dot{\theta}_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Angle Rate')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy)
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy)
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy)


if __name__ == "__main__":
    # spinningBodyNoInput(True)
    # spinningBodyLockAxis(True)
    spinningBodyCommandedTorque(True)
