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

import numpy
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split("simulation")


from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import (
    unitTestSupport,
)  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.simulation import spacecraft
from Basilisk.simulation import hingedRigidBodyStateEffector
from Basilisk.utilities import macros
from Basilisk.utilities import pythonVariableLogger
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import spacecraftSystem
from Basilisk.architecture import messaging
from Basilisk.utilities import deprecated

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_


@pytest.mark.parametrize("useScPlus", [True])
def test_hingedRigidBodyMotorTorque(show_plots, useScPlus):
    """Module Unit Test"""
    [testResults, testMessage] = hingedRigidBodyMotorTorque(show_plots, useScPlus)
    assert testResults < 1, testMessage


def hingedRigidBodyMotorTorque(show_plots, useScPlus):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True
    deprecated.filterwarnings("ignore", "SpacecraftSystem.SpacecraftSystem")

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    if useScPlus:
        scObject = spacecraft.Spacecraft()
        scObject.ModelTag = "spacecraftBody"
    else:
        scObject = spacecraftSystem.SpacecraftSystem()
        scObject.ModelTag = "spacecraftBody"
        scObject.primaryCentralSpacecraft.spacecraftName = scObject.ModelTag

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.01)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unitTestSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.mass = 100.0
    unitTestSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel1.d = 1.5
    unitTestSim.panel1.k = 0.0
    unitTestSim.panel1.c = 0.0
    unitTestSim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    unitTestSim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel1.thetaInit = 0 * numpy.pi / 180.0
    unitTestSim.panel1.thetaDotInit = 0.0
    unitTestSim.panel1.ModelTag = "panel1"

    # set a fixed motor torque message
    motorMsgData = messaging.ArrayMotorTorqueMsgPayload()
    motorMsgData.motorTorque = [2.0]
    motorMsg = messaging.ArrayMotorTorqueMsg().write(motorMsgData)
    unitTestSim.panel1.motorTorqueInMsg.subscribeTo(motorMsg)

    # Define Variables for panel 2
    unitTestSim.panel2.mass = 100.0
    unitTestSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel2.d = 1.5
    unitTestSim.panel2.k = 0.0
    unitTestSim.panel2.c = 0.0
    unitTestSim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel2.thetaInit = 0.0 * macros.D2R
    unitTestSim.panel2.thetaDotInit = 0.0
    unitTestSim.panel2.ModelTag = "panel2"

    # Add panels to spaceCraft
    scObjectPrimary = scObject
    if not useScPlus:
        scObjectPrimary = scObject.primaryCentralSpacecraft

    scObjectPrimary.addStateEffector(unitTestSim.panel1)
    scObjectPrimary.addStateEffector(unitTestSim.panel2)

    # Define mass properties of the rigid part of the spacecraft
    scObjectPrimary.hub.mHub = 750.0
    scObjectPrimary.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObjectPrimary.hub.IHubPntBc_B = [
        [900.0, 0.0, 0.0],
        [0.0, 800.0, 0.0],
        [0.0, 0.0, 600.0],
    ]

    # Set the initial values for the states
    scObjectPrimary.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    scObjectPrimary.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    scObjectPrimary.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObjectPrimary.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.panel1)
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.panel2)

    if not useScPlus:
        scStateMsg = scObject.primaryCentralSpacecraft.scStateOutMsg
    else:
        scStateMsg = scObject.scStateOutMsg
    dataLog = scStateMsg.recorder()
    dataPanel1 = unitTestSim.panel1.hingedRigidBodyOutMsg.recorder()
    dataPanel2 = unitTestSim.panel2.hingedRigidBodyOutMsg.recorder()
    dataPanel1Log = unitTestSim.panel1.hingedRigidBodyConfigLogOutMsg.recorder()
    dataPanel2Log = unitTestSim.panel2.hingedRigidBodyConfigLogOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    unitTestSim.AddModelToTask(unitTaskName, dataPanel1)
    unitTestSim.AddModelToTask(unitTaskName, dataPanel2)
    unitTestSim.AddModelToTask(unitTaskName, dataPanel1Log)
    unitTestSim.AddModelToTask(unitTaskName, dataPanel2Log)

    if useScPlus:
        scLog = scObject.logger("totRotAngMomPntC_N")
    else:
        scLog = pythonVariableLogger.PythonVariableLogger(
            {
                "totRotAngMomPntC_N": lambda _: scObject.primaryCentralSpacecraft.totRotAngMomPntC_N
            }
        )
    unitTestSim.AddModelToTask(unitTaskName, scLog)

    unitTestSim.InitializeSimulation()

    stopTime = 10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    rOut_CN_N = dataLog.r_CN_N
    vOut_CN_N = dataLog.v_CN_N
    sigma_BN = dataLog.sigma_BN
    theta1 = dataPanel1.theta
    theta2 = dataPanel2.theta

    rB1N = dataPanel1Log.r_BN_N[0]
    vB1N = dataPanel1Log.v_BN_N[0]
    sB1N = dataPanel1Log.sigma_BN[0]
    oB1N = dataPanel1Log.omega_BN_B[0]
    rB2N = dataPanel2Log.r_BN_N[0]
    vB2N = dataPanel2Log.v_BN_N[0]
    sB2N = dataPanel2Log.sigma_BN[0]
    oB2N = dataPanel2Log.omega_BN_B[0]

    rotAngMom_N = unitTestSupport.addTimeColumn(scLog.times(), scLog.totRotAngMomPntC_N)

    # Get the last sigma and position
    dataPos = [rOut_CN_N[-1]]

    truePos = [[0.0, 0.0, 0.0]]

    initialRotAngMom_N = [[rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]]

    finalRotAngMom = [rotAngMom_N[-1]]

    plt.close("all")

    plt.figure()
    plt.clf()
    plt.plot(
        rotAngMom_N[:, 0] * 1e-9,
        (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]),
        rotAngMom_N[:, 0] * 1e-9,
        (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]),
        rotAngMom_N[:, 0] * 1e-9,
        (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]),
    )
    plt.xlabel("time (s)")
    plt.ylabel("Ang. Momentum Difference")

    plt.figure()
    plt.clf()
    plt.plot(
        dataLog.times() * 1e-9,
        vOut_CN_N[:, 0],
        dataLog.times() * 1e-9,
        vOut_CN_N[:, 1],
        dataLog.times() * 1e-9,
        vOut_CN_N[:, 2],
    )
    plt.xlabel("time (s)")
    plt.ylabel("m/s")

    plt.figure()
    plt.clf()
    plt.plot(
        dataLog.times() * macros.NANO2SEC,
        sigma_BN[:, 0],
        color=unitTestSupport.getLineColor(0, 3),
        label=r"$\sigma_{1}$",
    )
    plt.plot(
        dataLog.times() * macros.NANO2SEC,
        sigma_BN[:, 1],
        color=unitTestSupport.getLineColor(1, 3),
        label=r"$\sigma_{2}$",
    )
    plt.plot(
        dataLog.times() * macros.NANO2SEC,
        sigma_BN[:, 2],
        color=unitTestSupport.getLineColor(2, 3),
        label=r"$\sigma_{3}$",
    )
    plt.legend(loc="lower right")
    plt.xlabel("time (s)")
    plt.ylabel(r"MRP $\sigma_{B/N}$")

    plt.figure()
    plt.clf()
    plt.plot(
        dataPanel1.times() * macros.NANO2SEC,
        theta1 * macros.R2D,
        color=unitTestSupport.getLineColor(0, 3),
        label=r"$\theta_{1}$",
    )
    plt.plot(
        dataPanel2.times() * macros.NANO2SEC,
        theta2 * macros.R2D,
        color=unitTestSupport.getLineColor(1, 3),
        label=r"$\theta_{2}$",
    )
    plt.legend(loc="lower right")
    plt.xlabel("time (s)")
    plt.ylabel("Hinge Angles [deg]")

    if show_plots:
        plt.show()
    plt.close("all")

    accuracy = 1e-10
    for i in range(0, len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(dataPos[i], truePos[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED:  Hinged Rigid Body integrated test failed position test"
            )

    finalRotAngMom = numpy.delete(finalRotAngMom, 0, axis=1)  # remove time column
    for i in range(0, len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(
            finalRotAngMom[i], initialRotAngMom_N[i], 3, accuracy
        ):
            testFailCount += 1
            testMessages.append(
                "FAILED: Hinged Rigid Body integrated test failed rotational angular momentum unit test"
            )

    # check config log messages
    if not unitTestSupport.isArrayEqual(rB1N, [2.0, 0, 0], 3, accuracy):
        testFailCount += 1
        testMessages.append(
            "FAILED:  Hinged Rigid Body integrated test failed panel 1 r_BN_N config log test"
        )
    if not unitTestSupport.isArrayEqual(vB1N, [0.0, 0, 0], 3, accuracy):
        testFailCount += 1
        testMessages.append(
            "FAILED:  Hinged Rigid Body integrated test failed panel 1 v_BN_N config log test"
        )
    if not unitTestSupport.isArrayEqual(sB1N, [0.0, 0, 1.0], 3, accuracy):
        testFailCount += 1
        testMessages.append(
            "FAILED:  Hinged Rigid Body integrated test failed panel 1 sigma_BN config log test"
        )
    if not unitTestSupport.isArrayEqual(oB1N, [0.0, 0, 0], 3, accuracy):
        testFailCount += 1
        testMessages.append(
            "FAILED:  Hinged Rigid Body integrated test failed panel 1 omega_BN_B config log test"
        )
    if not unitTestSupport.isArrayEqual(rB2N, [-2.0, 0, 0], 3, accuracy):
        testFailCount += 1
        testMessages.append(
            "FAILED:  Hinged Rigid Body integrated test failed panel 2 r_BN_N config log test"
        )
    if not unitTestSupport.isArrayEqual(vB2N, [0.0, 0, 0], 3, accuracy):
        testFailCount += 1
        testMessages.append(
            "FAILED:  Hinged Rigid Body integrated test failed panel 2 v_BN_N config log test"
        )
    if not unitTestSupport.isArrayEqual(sB2N, [0.0, 0, 0.0], 3, accuracy):
        testFailCount += 1
        testMessages.append(
            "FAILED:  Hinged Rigid Body integrated test failed panel 2 sigma_BN config log test"
        )
    if not unitTestSupport.isArrayEqual(oB2N, [0.0, 0, 0], 3, accuracy):
        testFailCount += 1
        testMessages.append(
            "FAILED:  Hinged Rigid Body integrated test failed panel 2 omega_BN_B config log test"
        )

    if testFailCount == 0:
        print("PASSED: " + " Hinged Rigid Body integrated test with motor torques")

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, "".join(testMessages)]


def hingedRigidBodyLagrangVsBasilisk(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True
    deprecated.filterwarnings("ignore", "SpacecraftSystem.SpacecraftSystem")

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraftSystem.SpacecraftSystem()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    stepSize = 0.1
    testProcessRate = macros.sec2nano(stepSize)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unitTestSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.mass = 100.0
    unitTestSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel1.d = 1.5
    unitTestSim.panel1.k = 5000.0
    unitTestSim.panel1.c = 75
    unitTestSim.panel1.r_HB_B = [[0.5], [1.0], [0.0]]
    unitTestSim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]]
    unitTestSim.panel1.thetaInit = 0.0
    unitTestSim.panel1.thetaDotInit = 0.0

    # Define Variables for panel 2
    unitTestSim.panel2.mass = 100.0
    unitTestSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel2.d = 1.5
    unitTestSim.panel2.k = 5000.0
    unitTestSim.panel2.c = 75
    unitTestSim.panel2.r_HB_B = [[-0.5], [1.0], [0.0]]
    unitTestSim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    unitTestSim.panel2.thetaInit = 0.0
    unitTestSim.panel2.thetaDotInit = 0.0

    # Add panels to spaceCraft
    scObject.primaryCentralSpacecraft.addStateEffector(unitTestSim.panel1)
    scObject.primaryCentralSpacecraft.addStateEffector(unitTestSim.panel2)

    # Define force and torque
    momentArm1_B = numpy.array([0.05, 0.0, 0.0])
    force1_B = numpy.array([0.2, 0.7, 0.0])
    torque1_B = numpy.cross(momentArm1_B, force1_B)
    momentArm2_B = numpy.array([-0.03, 0.0, 0.0])
    force2_B = numpy.array([0.0, 1.0, 0.0])
    torque2_B = numpy.cross(momentArm2_B, force2_B)

    # Add external force and torque
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extForce_B = [[force1_B[0]], [force1_B[1]], [force1_B[2]]]
    extFTObject.extTorquePntB_B = [[torque1_B[0]], [torque1_B[1]], [torque1_B[2]]]
    scObject.primaryCentralSpacecraft.addDynamicEffector(extFTObject)
    unitTestSim.AddModelToTask(unitTaskName, extFTObject)

    # Define mass properties of the rigid part of the spacecraft
    scObject.primaryCentralSpacecraft.hub.mHub = 750.0
    scObject.primaryCentralSpacecraft.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.primaryCentralSpacecraft.hub.IHubPntBc_B = [
        [900.0, 0.0, 0.0],
        [0.0, 800.0, 0.0],
        [0.0, 0.0, 600.0],
    ]

    # Set the initial values for the states
    scObject.primaryCentralSpacecraft.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.primaryCentralSpacecraft.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.primaryCentralSpacecraft.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.primaryCentralSpacecraft.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.panel1)
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.panel2)

    dataLog = scObject.primaryCentralSpacecraft.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    stateLog = pythonVariableLogger.PythonVariableLogger(
        {
            "theta1": lambda _: scObject.dynManager.getStateObject(
                "spacecrafthingedRigidBodyTheta1"
            ).getState(),
            "theta2": lambda _: scObject.dynManager.getStateObject(
                "spacecrafthingedRigidBodyTheta2"
            ).getState(),
        }
    )
    unitTestSim.AddModelToTask(unitTaskName, stateLog)

    unitTestSim.InitializeSimulation()

    # Define times that the new forces will be applies
    force1OffTime = 5.0
    force2OnTime = 11.0
    force2OffTime = 18.0
    stopTime = 20.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(force1OffTime))
    unitTestSim.ExecuteSimulation()

    # Turn force1 off
    extFTObject.extForce_B = [[0.0], [0.0], [0.0]]
    extFTObject.extTorquePntB_B = [[0.0], [0.0], [0.0]]

    unitTestSim.ConfigureStopTime(macros.sec2nano(force2OnTime))
    unitTestSim.ExecuteSimulation()

    # Turn force2 on
    extFTObject.extForce_B = [[force2_B[0]], [force2_B[1]], [force2_B[2]]]
    extFTObject.extTorquePntB_B = [[torque2_B[0]], [torque2_B[1]], [torque2_B[2]]]

    unitTestSim.ConfigureStopTime(macros.sec2nano(force2OffTime))
    unitTestSim.ExecuteSimulation()

    # Turn force2 off and finish sim
    extFTObject.extForce_B = [[0.0], [0.0], [0.0]]
    extFTObject.extTorquePntB_B = [[0.0], [0.0], [0.0]]

    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    theta1Out = unitTestSupport.addTimeColumn(stateLog.times(), stateLog.theta1)
    theta2Out = unitTestSupport.addTimeColumn(stateLog.times(), stateLog.theta2)

    rOut_BN_N = dataLog.r_BN_N
    sigmaOut_BN = dataLog.sigma_BN
    thetaOut = 4.0 * numpy.arctan(sigmaOut_BN[:, 2])

    # Developing the lagrangian result
    # Define initial values
    spacecraft = spacecraftClass()
    spacecraft.hub.mass = scObject.primaryCentralSpacecraft.hub.mHub
    spacecraft.hub.Inertia = scObject.primaryCentralSpacecraft.hub.IHubPntBc_B[2][2]
    # Define variables for panel1
    spacecraft.panel1.mass = unitTestSim.panel1.mass
    spacecraft.panel1.Inertia = unitTestSim.panel1.IPntS_S[1][1]
    spacecraft.panel1.Rhinge = numpy.linalg.norm(
        numpy.asarray(unitTestSim.panel1.r_HB_B)
    )
    spacecraft.panel1.beta = numpy.arctan2(
        unitTestSim.panel1.r_HB_B[1][0], unitTestSim.panel1.r_HB_B[0][0]
    )
    spacecraft.panel1.thetaH = 0.0
    spacecraft.panel1.d = unitTestSim.panel1.d
    spacecraft.panel1.k = unitTestSim.panel1.k
    spacecraft.panel1.c = unitTestSim.panel1.c
    # Define variables for panel2
    spacecraft.panel2.mass = unitTestSim.panel2.mass
    spacecraft.panel2.Inertia = unitTestSim.panel2.IPntS_S[1][1]
    spacecraft.panel2.Rhinge = numpy.linalg.norm(
        numpy.asarray(unitTestSim.panel2.r_HB_B)
    )
    spacecraft.panel2.beta = numpy.arctan2(
        unitTestSim.panel2.r_HB_B[1][0], unitTestSim.panel2.r_HB_B[0][0]
    )
    spacecraft.panel2.thetaH = numpy.pi
    spacecraft.panel2.d = unitTestSim.panel2.d
    spacecraft.panel2.k = unitTestSim.panel2.k
    spacecraft.panel2.c = unitTestSim.panel2.c

    # Define initial conditions of the sim
    time = numpy.arange(0.0, stopTime + stepSize, stepSize).flatten()
    x0 = numpy.zeros(10)
    x0[3] = unitTestSim.panel1.thetaInit
    x0[4] = -unitTestSim.panel2.thetaInit

    X = numpy.zeros((len(x0), len(time)))
    X[:, 0] = x0
    for j in range(1, (len(time))):
        if time[j - 1] < force1OffTime:
            spacecraft.xThrust_B = force1_B[0]
            spacecraft.yThrust_B = force1_B[1]
            spacecraft.Torque = torque1_B[2]
        elif time[j - 1] >= force2OnTime and time[j - 1] < force2OffTime:
            spacecraft.xThrust_B = force2_B[0]
            spacecraft.yThrust_B = force2_B[1]
            spacecraft.Torque = torque2_B[2]
        else:
            spacecraft.xThrust_B = 0.0
            spacecraft.yThrust_B = 0.0
            spacecraft.Torque = 0.0
        X[:, j] = rk4(
            planarFlexFunction, X[:, j - 1], stepSize, time[j - 1], spacecraft
        )

    plt.figure()
    plt.clf()
    plt.plot(time, X[0, :], "-b", label="Lagrangian")
    plt.plot(
        dataLog.times() * 1e-9,
        (rOut_BN_N[:, 0] - rOut_BN_N[:, 0]),
        "-r",
        label="Basilisk",
    )
    plt.plot(
        [time[25], time[75], time[125], time[175]],
        [
            X[0, 25],
            X[0, 75],
            X[0, 125],
            X[0, 175],
        ],
        "ok",
        label="Test Points",
    )
    plt.xlabel("time (s)")
    plt.ylabel("x position (m)")
    plt.legend(loc="upper left", numpoints=1)
    PlotName = "XPositionLagrangianVsBasilisk"
    PlotTitle = "X Position Lagrangian Vs Basilisk"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(time, X[1, :], "-b", label="Lagrangian")
    plt.plot(
        dataLog.times() * 1e-9,
        (rOut_BN_N[:, 1] - rOut_BN_N[:, 1]),
        "r",
        label="Basilisk",
    )
    plt.plot(
        [time[25], time[75], time[125], time[175]],
        [
            X[1, 25],
            X[1, 75],
            X[1, 125],
            X[1, 175],
        ],
        "ok",
        label="Test Points",
    )
    plt.xlabel("time (s)")
    plt.ylabel("y position (m)")
    plt.legend(loc="upper left", numpoints=1)
    PlotName = "YPositionLagrangianVsBasilisk"
    PlotTitle = "Y Position Lagrangian Vs Basilisk"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(time, X[2, :], "-b", label="Lagrangian")
    plt.plot(dataLog.times() * 1e-9, thetaOut, "-r", label="Basilisk")
    plt.plot(
        [time[25], time[75], time[125], time[175]],
        [
            X[2, 25],
            X[2, 75],
            X[2, 125],
            X[2, 175],
        ],
        "ok",
        label="Test Points",
    )
    plt.xlabel("time (s)")
    plt.ylabel("theta (rad)")
    plt.legend(loc="upper left", numpoints=1)
    PlotName = "ThetaLagrangianVsBasilisk"
    PlotTitle = "Theta Lagrangian Vs Basilisk"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(time, X[3, :], "-b", label="Lagrangian")
    plt.plot(theta1Out[:, 0] * 1e-9, theta1Out[:, 1], "-r", label="Basilisk")
    plt.plot(
        [time[25], time[75], time[125], time[175]],
        [
            X[3, 25],
            X[3, 75],
            X[3, 125],
            X[3, 175],
        ],
        "ok",
        label="Test Points",
    )
    plt.xlabel("time (s)")
    plt.ylabel("theta 1 (rad)")
    plt.legend(loc="upper left", numpoints=1)
    PlotName = "Theta1LagrangianVsBasilisk"
    PlotTitle = "Theta 1 Position Lagrangian Vs Basilisk"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(time, -X[4, :], "-b", label="Lagrangian")
    plt.plot(theta2Out[:, 0] * 1e-9, theta2Out[:, 1], "-r", label="Basilisk")
    plt.plot(
        [time[25], time[75], time[125], time[175]],
        [
            -X[4, 25],
            -X[4, 75],
            -X[4, 125],
            -X[4, 175],
        ],
        "ok",
        label="Test Points",
    )
    plt.xlabel("time (s)")
    plt.ylabel("theta 2 (rad)")
    plt.legend(loc="lower left", numpoints=1)
    PlotName = "Theta2LagrangianVsBasilisk"
    PlotTitle = "Theta 2 Lagrangian Vs Basilisk"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    if show_plots:
        plt.show()
    plt.close("all")

    accuracy = 1e-10
    timeList = [25, 75, 125, 175]

    for i in timeList:
        if abs(X[0, i] - (rOut_BN_N[i, 0] - rOut_BN_N[0, 0])) > accuracy:
            print(abs(X[0, i] - (rOut_BN_N[i, 0] - rOut_BN_N[0, 0])))
            testFailCount += 1
            testMessages.append(
                "FAILED: Hinged Rigid Body integrated test Lagrangian vs. Basilisk failed x position comparison "
            )
        if abs(X[1, i] - (rOut_BN_N[i, 1] - rOut_BN_N[0, 1])) > accuracy:
            testFailCount += 1
            testMessages.append(
                "FAILED: Hinged Rigid Body integrated test Lagrangian vs. Basilisk failed y position comparison "
            )
        if abs(X[2, i] - thetaOut[i]) > accuracy:
            testFailCount += 1
            testMessages.append(
                "FAILED: Hinged Rigid Body integrated test Lagrangian vs. Basilisk failed theta comparison "
            )
        if abs(X[3, i] - theta1Out[i, 1]) > accuracy:
            testFailCount += 1
            testMessages.append(
                "FAILED: Hinged Rigid Body integrated test Lagrangian vs. Basilisk failed theta 1 comparison "
            )
        if abs(-X[4, i] - theta2Out[i, 1]) > accuracy:
            testFailCount += 1
            testMessages.append(
                "FAILED: Hinged Rigid Body integrated test Lagrangian vs. Basilisk failed theta 2 comparison "
            )

    if testFailCount == 0:
        print("PASSED: " + " Hinged Rigid Body Transient Integrated test")

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, "".join(testMessages)]


def planarFlexFunction(x, t, variables):
    theta = x[2]
    theta1 = x[3]
    theta2 = x[4]
    xHubDot = x[5]
    yHubDot = x[6]
    thetaDot = x[7]
    theta1Dot = x[8]
    theta2Dot = x[9]
    # Define variables for hub
    mHub = variables.hub.mass
    IHub = variables.hub.Inertia
    # Define variables for panel1
    mSP1 = variables.panel1.mass
    ISP1 = variables.panel1.Inertia
    Rhinge1 = variables.panel1.Rhinge
    beta1 = variables.panel1.beta
    thetaH1 = variables.panel1.thetaH
    d1 = variables.panel1.d
    k1 = variables.panel1.k
    c1 = variables.panel1.c
    # Define variables for panel2
    mSP2 = variables.panel2.mass
    ISP2 = variables.panel2.Inertia
    Rhinge2 = variables.panel2.Rhinge
    beta2 = variables.panel2.beta
    thetaH2 = variables.panel2.thetaH
    d2 = variables.panel2.d
    k2 = variables.panel2.k
    c2 = variables.panel2.c
    Tx_B = variables.xThrust_B
    Ty_B = variables.yThrust_B
    Torque = variables.Torque

    # Convert Tx_B and Ty_B to the inertial frame
    dcm_BN = numpy.array(
        [[numpy.cos(theta), numpy.sin(theta)], [-numpy.sin(theta), numpy.cos(theta)]]
    )
    Thrust_N = numpy.dot(dcm_BN.transpose(), numpy.array([[Tx_B], [Ty_B]]))
    Tx = Thrust_N[0, 0]
    Ty = Thrust_N[1, 0]

    matrixA = numpy.zeros((5, 5))
    vectorB = numpy.zeros(5)
    # Populate X Translation Equation
    matrixA[0, 0] = 1.0
    matrixA[0, 1] = 0.0
    matrixA[0, 2] = (
        -1
        / (mHub + mSP1 + mSP2)
        * (
            mSP1 * Rhinge1 * numpy.sin(beta1 + theta)
            + mSP2 * Rhinge2 * numpy.sin(beta2 + theta)
            + d1 * mSP1 * numpy.sin(thetaH1 + theta + theta1)
            + d2 * mSP2 * numpy.sin(thetaH2 + theta + theta2)
        )
    )
    matrixA[0, 3] = (
        -1 / (mHub + mSP1 + mSP2) * (d1 * mSP1 * numpy.sin(thetaH1 + theta + theta1))
    )
    matrixA[0, 4] = (
        -1 / (mHub + mSP1 + mSP2) * (d2 * mSP2 * numpy.sin(thetaH2 + theta + theta2))
    )
    vectorB[0] = (
        1
        / (mHub + mSP1 + mSP2)
        * (
            Tx
            + mSP1 * Rhinge1 * numpy.cos(beta1 + theta) * thetaDot**2
            + mSP2 * Rhinge2 * numpy.cos(beta2 + theta) * thetaDot**2
            + d1 * mSP1 * numpy.cos(thetaH1 + theta + theta1) * thetaDot**2
            + d2 * mSP2 * numpy.cos(thetaH2 + theta + theta2) * thetaDot**2
            + 2 * d1 * mSP1 * numpy.cos(thetaH1 + theta + theta1) * thetaDot * theta1Dot
            + d1 * mSP1 * numpy.cos(thetaH1 + theta + theta1) * theta1Dot**2
            + 2 * d2 * mSP2 * numpy.cos(thetaH2 + theta + theta2) * thetaDot * theta2Dot
            + d2 * mSP2 * numpy.cos(thetaH2 + theta + theta2) * theta2Dot**2
        )
    )
    # Populate Y Translation Equation
    matrixA[1, 0] = 0.0
    matrixA[1, 1] = 1.0
    matrixA[1, 2] = (
        1
        / (mHub + mSP1 + mSP2)
        * (
            mSP1 * Rhinge1 * numpy.cos(beta1 + theta)
            + mSP2 * Rhinge2 * numpy.cos(beta2 + theta)
            + d1 * mSP1 * numpy.cos(thetaH1 + theta + theta1)
            + d2 * mSP2 * numpy.cos(thetaH2 + theta + theta2)
        )
    )
    matrixA[1, 3] = (
        1 / (mHub + mSP1 + mSP2) * (d1 * mSP1 * numpy.cos(thetaH1 + theta + theta1))
    )
    matrixA[1, 4] = (
        1 / (mHub + mSP1 + mSP2) * (d2 * mSP2 * numpy.cos(thetaH2 + theta + theta2))
    )
    vectorB[1] = (
        1
        / (mHub + mSP1 + mSP2)
        * (
            Ty
            + mSP1 * Rhinge1 * numpy.sin(beta1 + theta) * thetaDot**2
            + mSP2 * Rhinge2 * numpy.sin(beta2 + theta) * thetaDot**2
            + d1 * mSP1 * numpy.sin(thetaH1 + theta + theta1) * thetaDot**2
            + d2 * mSP2 * numpy.sin(thetaH2 + theta + theta2) * thetaDot**2
            + 2 * d1 * mSP1 * numpy.sin(thetaH1 + theta + theta1) * thetaDot * theta1Dot
            + d1 * mSP1 * numpy.sin(thetaH1 + theta + theta1) * theta1Dot**2
            + 2 * d2 * mSP2 * numpy.sin(thetaH2 + theta + theta2) * thetaDot * theta2Dot
            + d2 * mSP2 * numpy.sin(thetaH2 + theta + theta2) * theta2Dot**2
        )
    )
    # Populate theta Equation
    matrixA[2, 0] = (
        -1
        / (
            IHub
            + ISP1
            + ISP2
            + d1**2 * mSP1
            + d2**2 * mSP2
            + mSP1 * Rhinge1**2
            + mSP2 * Rhinge2**2
            + 2 * d1 * mSP1 * Rhinge1 * numpy.cos(beta1 - thetaH1 - theta1)
            + 2 * d2 * mSP2 * Rhinge2 * numpy.cos(beta2 - thetaH2 - theta2)
        )
        * (
            mSP1 * Rhinge1 * numpy.sin(beta1 + theta)
            + mSP2 * Rhinge2 * numpy.sin(beta2 + theta)
            + d1 * mSP1 * numpy.sin(thetaH1 + theta + theta1)
            + d2 * mSP2 * numpy.sin(thetaH2 + theta + theta2)
        )
    )
    matrixA[2, 1] = (
        1
        / (
            IHub
            + ISP1
            + ISP2
            + d1**2 * mSP1
            + d2**2 * mSP2
            + mSP1 * Rhinge1**2
            + mSP2 * Rhinge2**2
            + 2 * d1 * mSP1 * Rhinge1 * numpy.cos(beta1 - thetaH1 - theta1)
            + 2 * d2 * mSP2 * Rhinge2 * numpy.cos(beta2 - thetaH2 - theta2)
        )
        * (
            mSP1 * Rhinge1 * numpy.cos(beta1 + theta)
            + mSP2 * Rhinge2 * numpy.cos(beta2 + theta)
            + d1 * mSP1 * numpy.cos(thetaH1 + theta + theta1)
            + d2 * mSP2 * numpy.cos(thetaH2 + theta + theta2)
        )
    )
    matrixA[2, 2] = 1.0
    matrixA[2, 3] = (
        1
        / (
            IHub
            + ISP1
            + ISP2
            + d1**2 * mSP1
            + d2**2 * mSP2
            + mSP1 * Rhinge1**2
            + mSP2 * Rhinge2**2
            + 2 * d1 * mSP1 * Rhinge1 * numpy.cos(beta1 - thetaH1 - theta1)
            + 2 * d2 * mSP2 * Rhinge2 * numpy.cos(beta2 - thetaH2 - theta2)
        )
        * (
            ISP1
            + d1**2 * mSP1
            + d1 * mSP1 * Rhinge1 * numpy.cos(beta1 - thetaH1 - theta1)
        )
    )
    matrixA[2, 4] = (
        1
        / (
            IHub
            + ISP1
            + ISP2
            + d1**2 * mSP1
            + d2**2 * mSP2
            + mSP1 * Rhinge1**2
            + mSP2 * Rhinge2**2
            + 2 * d1 * mSP1 * Rhinge1 * numpy.cos(beta1 - thetaH1 - theta1)
            + 2 * d2 * mSP2 * Rhinge2 * numpy.cos(beta2 - thetaH2 - theta2)
        )
        * (
            ISP2
            + d2**2 * mSP2
            + d2 * mSP2 * Rhinge2 * numpy.cos(beta2 - thetaH2 - theta2)
        )
    )
    vectorB[2] = (
        1
        / (
            IHub
            + ISP1
            + ISP2
            + d1**2 * mSP1
            + d2**2 * mSP2
            + mSP1 * Rhinge1**2
            + mSP2 * Rhinge2**2
            + 2 * d1 * mSP1 * Rhinge1 * numpy.cos(beta1 - thetaH1 - theta1)
            + 2 * d2 * mSP2 * Rhinge2 * numpy.cos(beta2 - thetaH2 - theta2)
        )
        * (
            Torque
            - 2
            * d1
            * mSP1
            * Rhinge1
            * numpy.sin(beta1 - thetaH1 - theta1)
            * thetaDot
            * theta1Dot
            - d1 * mSP1 * Rhinge1 * numpy.sin(beta1 - thetaH1 - theta1) * theta1Dot**2
            - 2
            * d2
            * mSP2
            * Rhinge2
            * numpy.sin(beta2 - thetaH2 - theta2)
            * thetaDot
            * theta2Dot
            - d2 * mSP2 * Rhinge2 * numpy.sin(beta2 - thetaH2 - theta2) * theta2Dot**2
        )
    )
    # Populate theta1 Equation
    matrixA[3, 0] = (
        -1 / (ISP1 + d1**2 * mSP1) * (d1 * mSP1 * numpy.sin(thetaH1 + theta + theta1))
    )
    matrixA[3, 1] = (
        1 / (ISP1 + d1**2 * mSP1) * (d1 * mSP1 * numpy.cos(thetaH1 + theta + theta1))
    )
    matrixA[3, 2] = (
        1
        / (ISP1 + d1**2 * mSP1)
        * (
            ISP1
            + d1**2 * mSP1
            + d1 * mSP1 * Rhinge1 * numpy.cos(beta1 - thetaH1 - theta1)
        )
    )
    matrixA[3, 3] = 1.0
    matrixA[3, 4] = 0.0
    vectorB[3] = (
        1
        / (ISP1 + d1**2 * mSP1)
        * (
            -k1 * theta1
            + d1 * mSP1 * Rhinge1 * numpy.sin(beta1 - thetaH1 - theta1) * thetaDot**2
            - c1 * theta1Dot
        )
    )
    # Populate theta2 Equation
    matrixA[4, 0] = (
        -1 / (ISP2 + d2**2 * mSP2) * (d2 * mSP2 * numpy.sin(thetaH2 + theta + theta2))
    )
    matrixA[4, 1] = (
        1 / (ISP2 + d2**2 * mSP2) * (d2 * mSP2 * numpy.cos(thetaH2 + theta + theta2))
    )
    matrixA[4, 2] = (
        1
        / (ISP2 + d2**2 * mSP2)
        * (
            ISP2
            + d2**2 * mSP2
            + d2 * mSP2 * Rhinge2 * numpy.cos(beta2 - thetaH2 - theta2)
        )
    )
    matrixA[4, 3] = 0.0
    matrixA[4, 4] = 1.0
    vectorB[4] = (
        1
        / (ISP2 + d2**2 * mSP2)
        * (
            -k2 * theta2
            + d2 * mSP2 * Rhinge2 * numpy.sin(beta2 - thetaH2 - theta2) * thetaDot**2
            - c2 * theta2Dot
        )
    )

    Xdot = numpy.zeros(len(x))
    # Populate Trivial derivatives
    Xdot[0] = xHubDot
    Xdot[1] = yHubDot
    Xdot[2] = thetaDot
    Xdot[3] = theta1Dot
    Xdot[4] = theta2Dot
    # Calculate nontrivial derivatives
    result = numpy.dot(numpy.linalg.inv(matrixA), vectorB)
    Xdot[5:10] = result

    return Xdot


def rk4(Fn, X, h, t, varargin):
    k1 = h * Fn(X, t, varargin)
    k2 = h * Fn(X + k1 / 2, t + h / 2, varargin)
    k3 = h * Fn(X + k2 / 2, t + h / 2, varargin)
    k4 = h * Fn(X + k3, t + h, varargin)
    Z = X + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    return Z


class solarPanel:
    mass = 0.0
    Inertia = 0.0
    Rhinge = 0.0
    beta = 0.0
    thetaH = 0.0
    d = 0.0
    k = 0.0
    c = 0.0


class hubClass:
    mass = 0.0
    Inertia = 0.0


class spacecraftClass:
    panel1 = solarPanel()
    panel2 = solarPanel()
    hub = hubClass()
    xThrust_B = 0.0
    yThrust_B = 0.0
    Torque = 0.0


def newtonRapshon(funcAndDervi, guess, tolerance, variables):
    xOld = guess
    for i in range(1, 101):
        fx, fPrimex = funcAndDervi(xOld, variables)
        xNew = xOld - fx / fPrimex
        if abs(xNew - xOld) < tolerance:
            break
        xOld = xNew
    return xNew


def boxAndWingsFandFPrime(theta, variables):
    # Define variables
    F = variables.F
    mSC = variables.mSC
    k = variables.k
    mSP = variables.mSP
    d = variables.d
    aSP = F / mSC
    fX = k * theta + mSP * aSP * d * numpy.cos(theta)
    fPrimeX = k - mSP * aSP * d * numpy.sin(theta)
    return fX, fPrimeX


class boxAndWingParameters:
    F = 0
    mSC = 0
    k = 0
    mSP = 0
    d = 0


if __name__ == "__main__":
    hingedRigidBodyMotorTorque(True, True)
