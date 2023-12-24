# ISC License
#
# Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        spinningBodies
#   Author:             João Vaz Carneiro
#   Creation Date:      October 17, 2022
#

import inspect
import os
import pytest
import numpy
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import spacecraft, spinningBodyTwoDOFStateEffector, gravityEffector
from Basilisk.architecture import messaging


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("cmdTorque1, lock1, theta1Ref, cmdTorque2, lock2, theta2Ref", [
    (0.0, False, 0.0, 0.0, False, 0.0)
    , (0.0, True, 0.0, 0.0, False, 0.0)
    , (0.0, False, 0.0, 0.0, True, 0.0)
    , (0.0, True, 0.0, 0.0, True, 0.0)
    , (1.0, False, 0.0, -2.0, False, 0.0)
    , (0.0, False, 10.0 * macros.D2R, 0.0, False, -5.0 * macros.D2R)
    , (0.0, False, -5.0 * macros.D2R, 0.0, False, 10.0 * macros.D2R)
])
def test_spinningBody(show_plots, cmdTorque1, lock1, theta1Ref, cmdTorque2, lock2, theta2Ref):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft with a single-axis rotating rigid body attached to a rigid hub. The spinning
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
    [testResults, testMessage] = spinningBody(show_plots, cmdTorque1, lock1, theta1Ref, cmdTorque2, lock2, theta2Ref)
    assert testResults < 1, testMessage


def spinningBody(show_plots, cmdTorque1, lock1, theta1Ref, cmdTorque2, lock2, theta2Ref):
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
    testProcessRate = macros.sec2nano(0.0002)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create two hinged rigid bodies
    spinningBody = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()

    # Define properties of spinning body
    spinningBody.mass1 = 100.0
    spinningBody.mass2 = 50.0
    spinningBody.IS1PntSc1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    spinningBody.IS2PntSc2_S2 = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    spinningBody.dcm_S10B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.dcm_S20S1 = [[0.0, -1.0, 0.0], [0.0, .0, -1.0], [1.0, 0.0, 0.0]]
    spinningBody.r_Sc1S1_S1 = [[2.0], [-0.5], [0.0]]
    spinningBody.r_Sc2S2_S2 = [[1.0], [0.0], [-1.0]]
    spinningBody.r_S1B_B = [[-2.0], [0.5], [-1.0]]
    spinningBody.r_S2S1_S1 = [[0.5], [-1.5], [-0.5]]
    spinningBody.s1Hat_S1 = [[0], [0], [1]]
    spinningBody.s2Hat_S2 = [[0], [-1], [0]]
    spinningBody.theta1Init = 0 * macros.D2R
    spinningBody.theta2Init = 5 * macros.D2R
    spinningBody.k1 = 1000.0
    spinningBody.k2 = 500.0
    if theta1Ref != 0.0 or theta2Ref != 0.0:
        spinningBody.c1 = 500
        spinningBody.c2 = 200
    if lock1:
        spinningBody.theta1DotInit = 0 * macros.D2R
    else:
        spinningBody.theta1DotInit = 2.0 * macros.D2R
    if lock2:
        spinningBody.theta2DotInit = 0 * macros.D2R
    else:
        spinningBody.theta2DotInit = -1.0 * macros.D2R
    spinningBody.ModelTag = "SpinningBody"

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBody)

    # Create the torque message
    cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    cmdArray.motorTorque = [cmdTorque1, cmdTorque2]  # [Nm]
    cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    spinningBody.motorTorqueInMsg.subscribeTo(cmdMsg)

    # Create the locking message
    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockFlag = [0, 0]
    if lock1:
        lockFlag[0] = 1
    if lock2:
        lockFlag[1] = 1
    lockArray.effectorLockFlag = lockFlag
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    spinningBody.motorLockInMsg.subscribeTo(lockMsg)

    # Create the reference messages
    angle1Ref = messaging.HingedRigidBodyMsgPayload()
    angle1Ref.theta = theta1Ref
    angle1Ref.thetaDot = 0.0
    angle1RefMsg = messaging.HingedRigidBodyMsg().write(angle1Ref)
    spinningBody.spinningBodyRefInMsgs[0].subscribeTo(angle1RefMsg)

    angle2Ref = messaging.HingedRigidBodyMsgPayload()
    angle2Ref.theta = theta2Ref
    angle2Ref.thetaDot = 0.0
    angle2RefMsg = messaging.HingedRigidBodyMsg().write(angle2Ref)
    spinningBody.spinningBodyRefInMsgs[1].subscribeTo(angle2RefMsg)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.01], [-0.01], [0.01]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, spinningBody)
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
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)
    
    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add states to log
    theta1Data = spinningBody.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBody.spinningBodyOutMsgs[1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, theta1Data)
    unitTestSim.AddModelToTask(unitTaskName, theta2Data)

    # Setup and run the simulation
    stopTime = 25000*testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)
    orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)
    theta1 = theta1Data.theta
    theta1Dot = theta1Data.thetaDot
    theta2 = theta2Data.theta
    theta2Dot = theta2Data.thetaDot

    # Setup the conservation quantities
    initialOrbAngMom_N = [[orbAngMom_N[0, 1], orbAngMom_N[0, 2], orbAngMom_N[0, 3]]]
    finalOrbAngMom = [orbAngMom_N[-1]]
    initialRotAngMom_N = [[rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]]
    finalRotAngMom = [rotAngMom_N[-1]]
    initialOrbEnergy = [[orbEnergy[0, 1]]]
    finalOrbEnergy = [orbEnergy[-1]]
    initialRotEnergy = [[rotEnergy[0, 1]]]
    finalRotEnergy = [rotEnergy[-1]]

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 1] - orbAngMom_N[0, 1]) / orbAngMom_N[0, 1],
             orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 2] - orbAngMom_N[0, 2]) / orbAngMom_N[0, 2],
             orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 3] - orbAngMom_N[0, 3]) / orbAngMom_N[0, 3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:, 0] * 1e-9, (orbEnergy[:, 1] - orbEnergy[0, 1]) / orbEnergy[0, 1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]) / rotAngMom_N[0, 1],
             rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]) / rotAngMom_N[0, 2],
             rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]) / rotAngMom_N[0, 3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:, 0] * 1e-9, (rotEnergy[:, 1] - rotEnergy[0, 1]) / rotEnergy[0, 1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    plt.figure()
    plt.clf()
    plt.plot(theta1Data.times() * 1e-9, theta1)
    plt.xlabel('time (s)')
    plt.ylabel('theta1')

    plt.figure()
    plt.clf()
    plt.plot(theta1Data.times() * 1e-9, theta1Dot)
    plt.xlabel('time (s)')
    plt.ylabel('theta1Dot')

    plt.figure()
    plt.clf()
    plt.plot(theta2Data.times() * 1e-9, theta2)
    plt.xlabel('time (s)')
    plt.ylabel('theta2')

    plt.figure()
    plt.clf()
    plt.plot(theta2Data.times() * 1e-9, theta2Dot)
    plt.xlabel('time (s)')
    plt.ylabel('theta2Dot')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12
    finalOrbAngMom = numpy.delete(finalOrbAngMom, 0, axis=1)  # remove time column
    finalRotAngMom = numpy.delete(finalRotAngMom, 0, axis=1)  # remove time column
    finalRotEnergy = numpy.delete(finalRotEnergy, 0, axis=1)  # remove time column
    finalOrbEnergy = numpy.delete(finalOrbEnergy, 0, axis=1)  # remove time column

    for i in range(0, len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i], initialOrbAngMom_N[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: Spinning Body integrated test failed orbital angular momentum unit test")

    for i in range(0, len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i], initialRotAngMom_N[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: Spinning Body integrated test failed rotational angular momentum unit test")

    if cmdTorque1 == 0 and cmdTorque2 == 0 and theta1Ref == 0.0 and theta2Ref == 0.0:
        for i in range(0, len(initialRotEnergy)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i], initialRotEnergy[i], 1, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Spinning Body integrated test failed rotational energy unit test")

    for i in range(0, len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i], initialOrbEnergy[i], 1, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spinning Body integrated test failed orbital energy unit test")

    if theta1Ref != 0.0 or theta2Ref != 0.0:
        if not unitTestSupport.isDoubleEqual(theta1[-1], theta1Ref, 0.01):
            testFailCount += 1
            testMessages.append("FAILED: Spinning Body integrated test failed angle 1 convergence unit test")

        if not unitTestSupport.isDoubleEqual(theta2[-1], theta2Ref, 0.01):
            testFailCount += 1
            testMessages.append("FAILED: Spinning Body integrated test failed angle 2 convergence unit test")

    if testFailCount == 0:
        print("PASSED: " + " Spinning Body gravity integrated test")

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    spinningBody(True, 0.0, False, 0.0 * macros.D2R, 0.0, False, 0.0 * macros.D2R)
