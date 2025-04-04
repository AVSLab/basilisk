# ISC License
#
# Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        spinningBodies and prescribedMotion
#   Author:             Leah Kiner
#   Creation Date:      March 20, 2025
#

import inspect
import matplotlib.pyplot as plt
import numpy
import numpy as np
import os
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import prescribedMotionStateEffector, spacecraft, spinningBodyOneDOFStateEffector, gravityEffector
from Basilisk.simulation import prescribedLinearTranslation
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.architecture import messaging
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import vizSupport


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
@pytest.mark.parametrize("cmdTorque, lock, thetaRef", [
    (0.0, False, 0.0)
    , (0.0, True, 0.0)
    , (1.0, False, 0.0)
    , (0.0, False, 20.0 * macros.D2R)
])
def test_spinningBody(show_plots, cmdTorque, lock, thetaRef):
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
    [testResults, testMessage] = spinningBody(show_plots, cmdTorque, lock, thetaRef)
    assert testResults < 1, testMessage


def spinningBody(show_plots, cmdTorque, lock, thetaRef):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the spacecraft module
    massHub = 800  # [kg]
    lengthHub = 1.0  # [m]
    widthHub = 1.0  # [m]
    depthHub = 1.0  # [m]
    IHub_11 = (1 / 12) * massHub * (lengthHub * lengthHub + depthHub * depthHub)  # [kg m^2]
    IHub_22 = (1 / 12) * massHub * (lengthHub * lengthHub + widthHub * widthHub)  # [kg m^2]
    IHub_33 = (1 / 12) * massHub * (widthHub * widthHub + depthHub * depthHub)  # [kg m^2]

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scObject"
    scObject.hub.mHub = massHub  # kg
    scObject.hub.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
    scObject.hub.IHubPntBc_B = [[IHub_11, 0.0, 0.0], [0.0, IHub_22, 0.0], [0.0, 0.0, IHub_33]]  # [kg m^2] (Hub approximated as a cube)
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.omega_BN_BInit = [[0.01], [-0.01], [0.01]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Prescribed motion parameters
    posInit = 0.0
    thetaInit = 0.0
    transAxis_M = np.array([1.0, 0.0, 0.0])
    rotAxis_M = np.array([1.0, 0.0, 0.0])
    prvInit_FM = thetaInit * rotAxis_M
    sigma_FM = rbk.PRV2MRP(prvInit_FM)

    massPlatform = 10  # [kg]
    lengthPlatform = 1.0  # [m]
    widthPlatform = 1.0  # [m]
    depthPlatform = 1.0  # [m]
    IPlatform_11 = (1 / 12) * massPlatform * (lengthPlatform * lengthPlatform + depthPlatform * depthPlatform)  # [kg m^2]
    IPlatform_22 = (1 / 12) * massPlatform * (lengthPlatform * lengthPlatform + widthPlatform * widthPlatform)  # [kg m^2]
    IPlatform_33 = (1 / 12) * massPlatform * (widthPlatform * widthPlatform + depthPlatform * depthPlatform)  # [kg m^2]
    IPlatform_Fc_F = [[IPlatform_11, 0.0, 0.0], [0.0, IPlatform_22, 0.0], [0.0, 0.0,IPlatform_33]]  # [kg m^2] (approximated as a cube)

    # Create prescribed motion object
    platform = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    platform.ModelTag = "platform"
    platform.mass = massPlatform
    platform.IPntFc_F = IPlatform_Fc_F
    platform.r_MB_B = [0.5, 0.0, 0.0]
    platform.r_FcF_F = [0.5, 0.0, 0.0]
    platform.r_FM_M = [0.0, 0.0, 0.0]
    platform.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    platform.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
    platform.omega_FM_F = np.array([0.0, 0.0, 0.0])
    platform.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
    platform.sigma_FM = sigma_FM
    platform.omega_MB_B = [0.0, 0.0, 0.0]
    platform.omegaPrime_MB_B = [0.0, 0.0, 0.0]
    platform.sigma_MB = [0.0, 0.0, 0.0]
    unitTestSim.AddModelToTask(unitTaskName, platform)
    scObject.addStateEffector(platform)

    # Create rotational motion profiler
    angAccelMax = 0.5 * macros.D2R  # [rad/s^2]
    prescribedRotation = prescribedRotation1DOF.PrescribedRotation1DOF()
    prescribedRotation.ModelTag = "prescribedRotation1DOF"
    prescribedRotation.setRotHat_M(rotAxis_M)
    prescribedRotation.setThetaDDotMax(angAccelMax)
    prescribedRotation.setThetaInit(thetaInit)
    prescribedRotation.setCoastOptionBangDuration(1.0)
    prescribedRotation.setSmoothingDuration(1.0)
    unitTestSim.AddModelToTask(unitTaskName, prescribedRotation)

    # Create the rotational motion reference message
    prescribedThetaRef = 90.0 * macros.D2R #10.0 * macros.D2R  # [rad]
    prescribedRotationMessageData = messaging.HingedRigidBodyMsgPayload()
    prescribedRotationMessageData.theta = prescribedThetaRef
    prescribedRotationMessageData.thetaDot = 0.0  # [rad/s]
    prescribedRotationMessage = messaging.HingedRigidBodyMsg().write(prescribedRotationMessageData)
    prescribedRotation.spinningBodyInMsg.subscribeTo(prescribedRotationMessage)
    platform.prescribedRotationInMsg.subscribeTo(prescribedRotation.prescribedRotationOutMsg)

    # Create translational motion profiler
    transAccelMax = 0.005  # [m/s^2]
    prescribedTranslation = prescribedLinearTranslation.PrescribedLinearTranslation()
    prescribedTranslation.ModelTag = "prescribedLinearTranslation"
    prescribedTranslation.setTransHat_M(transAxis_M)
    prescribedTranslation.setTransAccelMax(transAccelMax)
    prescribedTranslation.setTransPosInit(posInit)
    prescribedTranslation.setCoastOptionBangDuration(1.0)
    prescribedTranslation.setSmoothingDuration(1.0)
    unitTestSim.AddModelToTask(unitTaskName, prescribedTranslation)

    # Create the translational motion reference message
    posRef = 1.0  #0.1  # [m]
    prescribedTranslationMessageData = messaging.LinearTranslationRigidBodyMsgPayload()
    prescribedTranslationMessageData.rho = posRef
    prescribedTranslationMessageData.rhoDot = 0.0
    prescribedTranslationMessage = messaging.LinearTranslationRigidBodyMsg().write(prescribedTranslationMessageData)
    prescribedTranslation.linearTranslationRigidBodyInMsg.subscribeTo(prescribedTranslationMessage)
    platform.prescribedTranslationInMsg.subscribeTo(prescribedTranslation.prescribedTranslationOutMsg)

    # Create the spinning body
    spinningBody = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
    spinningBody.ModelTag = "spinningBody"
    spinningBody.mass = 50.0
    spinningBody.IPntSc_S = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    spinningBody.dcm_S0B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.r_ScS_S = [[0.0], [0.0], [0.0]]
    spinningBody.r_SB_B = [[1.0], [0.0], [0.0]]
    spinningBody.sHat_S = [[1], [0], [0]]
    spinningBody.thetaInit = 5.0 * macros.D2R
    spinningBody.thetaDotInit = -1.0 * macros.D2R
    spinningBody.k = 100.0
    if thetaRef != 0.0:
        spinningBody.c = 50
    if lock:
        spinningBody.thetaDotInit = 0.0

    # Create the torque message
    cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    cmdArray.motorTorque = [cmdTorque]  # [Nm]
    cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    spinningBody.motorTorqueInMsg.subscribeTo(cmdMsg)

    # Create the locking message
    if lock:
        lockArray = messaging.ArrayEffectorLockMsgPayload()
        lockArray.effectorLockFlag = [1]
        lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
        spinningBody.motorLockInMsg.subscribeTo(lockMsg)

    # Create the reference message
    angleRef = messaging.HingedRigidBodyMsgPayload()
    angleRef.theta = thetaRef
    angleRef.thetaDot = 0.0
    angleRefMsg = messaging.HingedRigidBodyMsg().write(angleRef)
    spinningBody.spinningBodyRefInMsg.subscribeTo(angleRefMsg)

    unitTestSim.AddModelToTask(unitTaskName, spinningBody)

    # Add the spinning body to the prescribed motion platform
    # scObject.addStateEffector(spinningBody)
    platform.addStateEffector(spinningBody)

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
    thetaData = spinningBody.spinningBodyOutMsg.recorder()
    prescribed_rot_state_data_log = platform.prescribedRotationOutMsg.recorder()
    theta_data_log = prescribedRotation.spinningBodyOutMsg.recorder()
    prescribed_trans_state_data_log = platform.prescribedTranslationOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)
    unitTestSim.AddModelToTask(unitTaskName, thetaData)
    unitTestSim.AddModelToTask(unitTaskName, prescribed_rot_state_data_log)
    unitTestSim.AddModelToTask(unitTaskName, theta_data_log)
    unitTestSim.AddModelToTask(unitTaskName, prescribed_trans_state_data_log)

    # Add Vizard
    scBodyList = [scObject]
    scBodyList.append(["platform", platform.prescribedMotionConfigLogOutMsg])
    scBodyList.append(["spinningBody", spinningBody.spinningBodyConfigLogOutMsg])

    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(unitTestSim, unitTaskName, scBodyList,
                                                  saveFile=filename
                                                  )

        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scObject.ModelTag]
                                     , modelPath="CUBE"
                                     , scale=[widthHub, lengthHub, depthHub]
                                     , color=vizSupport.toRGBA255("gray"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["platform"]
                                     , modelPath="CUBE"
                                     , scale=[widthHub, lengthHub, depthHub]
                                     , color=vizSupport.toRGBA255("green"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["spinningBody"]
                                     , modelPath="CUBE"
                                     , scale=[widthHub, lengthHub, depthHub]
                                     , color=vizSupport.toRGBA255("blue"))
        viz.settings.orbitLinesOn = -1


    # Run the simulation
    unitTestSim.InitializeSimulation()
    simTime = 120.0  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)
    orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)
    theta = thetaData.theta
    thetaDot = thetaData.thetaDot
    timespan = prescribed_rot_state_data_log.times() * macros.NANO2SEC  # [s]
    prescribed_theta = macros.R2D * theta_data_log.theta  # [deg]
    omega_fm_f = prescribed_rot_state_data_log.omega_FM_F * macros.R2D  # [deg]
    omega_prime_fm_f = prescribed_rot_state_data_log.omegaPrime_FM_F * macros.R2D  # [deg]
    r_fm_m = prescribed_trans_state_data_log.r_FM_M
    r_prime_fm_m = prescribed_trans_state_data_log.rPrime_FM_M
    r_prime_prime_fm_m = prescribed_trans_state_data_log.rPrimePrime_FM_M

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

    theta_ref_plotting = np.ones(len(timespan)) * prescribedThetaRef * macros.R2D  # [deg]
    rhoRefs = np.ones(len(timespan)) * posRef  # [m]
    rhos = np.linalg.norm(r_fm_m, axis=1)  # [m]
    rhoDots = np.linalg.norm(r_prime_fm_m, axis=1)  # [m/s]
    rhoDDots = np.linalg.norm(r_prime_prime_fm_m, axis=1)  # [m/s^2]
    thetaDots = np.linalg.norm(omega_fm_f, axis=1)  # [deg/s]
    thetaDDots = np.linalg.norm(omega_prime_fm_f, axis=1)  # [deg/s^2]

    # Plot prescribed position and angle
    fig1, ax1 = plt.subplots()
    ax1.plot(timespan, prescribed_theta, label=r"$\theta$", color="teal")
    ax1.plot(timespan, theta_ref_plotting, "--", label=r"$\theta_{\text{ref}}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("Angle (deg)", color="teal", fontsize=16)
    # ax1.set_ylim(0.0, 11.0)
    ax2 = ax1.twinx()
    ax2.plot(timespan, 100.0 * rhos, label=r"$\rho$", color="darkviolet")
    ax2.plot(timespan, 100.0 * rhoRefs, "--", label=r"$\rho_{\text{ref}}$", color="darkviolet")
    ax2.set_ylabel("Displacement (cm)", color="darkviolet", fontsize=16)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    # ax2.set_ylim(0.0, 11.0)
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.legend(handles=handles, labels=labels, loc="center left", prop={"size": 16})
    plt.grid(True)

    # Plot prescribed velocities
    fig2, ax1 = plt.subplots()
    ax1.plot(timespan, thetaDots, label=r"$\dot{\theta}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("Angle Rate (deg/s)", color="teal", fontsize=16)
    # ax1.set_ylim(0.0, 1.1)
    ax2 = ax1.twinx()
    ax2.plot(timespan, 100.0 * rhoDots, label=r"$\dot{\rho}$", color="darkviolet")
    ax2.set_ylabel("Displacement Rate (cm/s)", color="darkviolet", fontsize=16)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    # ax2.set_ylim(0.0, 1.1)
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.legend(handles=handles, labels=labels, loc="center", prop={"size": 16})
    plt.grid(True)

    # Plot prescribed accelerations
    fig3, ax1 = plt.subplots()
    ax1.plot(timespan, omega_prime_fm_f[:, 0], label=r"$\ddot{\theta}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("Angular Acceleration (deg/$s^2$)", color="teal", fontsize=16)
    # ax1.set_ylim(-0.6, 0.6)
    ax2 = ax1.twinx()
    ax2.plot(timespan, 100.0 * r_prime_prime_fm_m[:, 0], label=r"$\ddot{\rho}$", color="darkviolet")
    ax2.set_ylabel("Linear Acceleration (cm/$s^2$)", color="darkviolet", fontsize=16)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    # ax2.set_ylim(-0.6, 0.6)
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.legend(handles=handles, labels=labels, loc="upper right", prop={"size": 16})
    plt.grid(True)


    plt.figure()
    plt.clf()
    plt.plot(thetaData.times() * 1e-9, theta)
    plt.xlabel('time (s)')
    plt.ylabel('theta')

    plt.figure()
    plt.clf()
    plt.plot(thetaData.times() * 1e-9, thetaDot)
    plt.xlabel('time (s)')
    plt.ylabel('thetaDot')

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

    # Only check rotational energy if no torques and no damping are applied
    if cmdTorque == 0.0 and thetaRef == 0.0:
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

    if thetaRef != 0.0:
        if not unitTestSupport.isDoubleEqual(theta[-1], thetaRef, 0.01):
            testFailCount += 1
            testMessages.append("FAILED: Spinning Body integrated test failed angle convergence unit test")

    if testFailCount == 0:
        print("PASSED: " + " Spinning Body gravity integrated test")

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    spinningBody(True, 0.0, False, 0.0 * macros.D2R)
