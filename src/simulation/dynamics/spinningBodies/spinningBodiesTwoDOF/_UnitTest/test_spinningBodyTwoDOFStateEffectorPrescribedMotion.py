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
#   Module Name:        spinningBodyTwoDOF and prescribedMotion
#   Author:             Leah Kiner
#   Creation Date:      June 5, 2025
#

import inspect
import matplotlib
import matplotlib.pyplot as plt
import numpy
import numpy as np
import os
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import prescribedMotionStateEffector, spacecraft, spinningBodyTwoDOFStateEffector, gravityEffector
from Basilisk.simulation import prescribedLinearTranslation
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.architecture import messaging
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import vizSupport

matplotlib.rc('xtick', labelsize=16)
matplotlib.rc('ytick', labelsize=16)

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

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.0002)  # update process rate update time
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
    transAxis_M = np.array([1.0, 0.0, 0.0])
    rotAxis_M = np.array([1.0, 0.0, 0.0])
    thetaInit = 0.0 * macros.D2R
    prvInit_PM = thetaInit * rotAxis_M
    sigma_PM = rbk.PRV2MRP(prvInit_PM)

    massPlatform = 100  # [kg]
    lengthPlatform = 1.0  # [m]
    widthPlatform = 1.0  # [m]
    depthPlatform = 1.0  # [m]
    IPlatform_11 = (1 / 12) * massPlatform * (lengthPlatform * lengthPlatform + depthPlatform * depthPlatform)  # [kg m^2]
    IPlatform_22 = (1 / 12) * massPlatform * (lengthPlatform * lengthPlatform + widthPlatform * widthPlatform)  # [kg m^2]
    IPlatform_33 = (1 / 12) * massPlatform * (widthPlatform * widthPlatform + depthPlatform * depthPlatform)  # [kg m^2]
    IPlatform_Pc_P = [[IPlatform_11, 0.0, 0.0], [0.0, IPlatform_22, 0.0], [0.0, 0.0,IPlatform_33]]  # [kg m^2] (approximated as a cube)

    # Create prescribed motion object
    platform = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    platform.ModelTag = "platform"
    platform.mass = massPlatform
    platform.IPntPc_P = IPlatform_Pc_P
    platform.r_MB_B = [0.5, 0.0, 0.0]
    platform.r_PcP_P = [0.5, 0.0, 0.0]
    platform.r_PM_M = [0.0, 0.0, 0.0]
    platform.rPrime_PM_M = np.array([0.0, 0.0, 0.0])
    platform.rPrimePrime_PM_M = np.array([0.0, 0.0, 0.0])
    platform.omega_PM_P = np.array([0.0, 0.0, 0.0])
    platform.omegaPrime_PM_P = np.array([0.0, 0.0, 0.0])
    platform.sigma_PM = sigma_PM
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
    prescribedThetaRef = 10.0 * macros.D2R  # [rad]
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
    posRef = 0.1  # [m]
    prescribedTranslationMessageData = messaging.LinearTranslationRigidBodyMsgPayload()
    prescribedTranslationMessageData.rho = posRef
    prescribedTranslationMessageData.rhoDot = 0.0
    prescribedTranslationMessage = messaging.LinearTranslationRigidBodyMsg().write(prescribedTranslationMessageData)
    prescribedTranslation.linearTranslationRigidBodyInMsg.subscribeTo(prescribedTranslationMessage)
    platform.prescribedTranslationInMsg.subscribeTo(prescribedTranslation.prescribedTranslationOutMsg)

    # Create the spinning body
    spinningBody = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody.ModelTag = "spinningBody"
    spinningBody.mass1 = 100.0
    spinningBody.mass2 = 50.0
    spinningBody.IS1PntSc1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    spinningBody.IS2PntSc2_S2 = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    spinningBody.dcm_S10B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody.r_Sc2S2_S2 = [[0.0], [0.0], [0.0]]
    spinningBody.r_S1B_B = [[1.5], [0.0], [0.0]]
    spinningBody.r_S2S1_S1 = [[1.0], [0.0], [0.0]]
    spinningBody.s1Hat_S1 = [[1], [0], [0]]
    spinningBody.s2Hat_S2 = [[1], [0], [0]]
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
    theta1Data = spinningBody.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBody.spinningBodyOutMsgs[1].recorder()
    prescribed_rot_state_data_log = platform.prescribedRotationOutMsg.recorder()
    theta_data_log = prescribedRotation.spinningBodyOutMsg.recorder()
    prescribed_trans_state_data_log = platform.prescribedTranslationOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)
    unitTestSim.AddModelToTask(unitTaskName, theta1Data)
    unitTestSim.AddModelToTask(unitTaskName, theta2Data)
    unitTestSim.AddModelToTask(unitTaskName, prescribed_rot_state_data_log)
    unitTestSim.AddModelToTask(unitTaskName, theta_data_log)
    unitTestSim.AddModelToTask(unitTaskName, prescribed_trans_state_data_log)

    # Add Vizard
    scBodyList = [scObject]
    scBodyList.append(["platform", platform.prescribedMotionConfigLogOutMsg])
    scBodyList.append(["spinningBody1", spinningBody.spinningBodyConfigLogOutMsgs[0]])
    scBodyList.append(["spinningBody2", spinningBody.spinningBodyConfigLogOutMsgs[1]])

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
                                     , simBodiesToModify=["spinningBody1"]
                                     , modelPath="CUBE"
                                     , scale=[widthHub, lengthHub, depthHub]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["spinningBody2"]
                                     , modelPath="CUBE"
                                     , scale=[widthHub, lengthHub, depthHub]
                                     , color=vizSupport.toRGBA255("blue"))
        viz.settings.orbitLinesOn = -1


    # Run the simulation
    unitTestSim.InitializeSimulation()
    simTime = 15.0  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
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
    timespan = prescribed_rot_state_data_log.times() * macros.NANO2SEC  # [s]
    prescribed_theta = macros.R2D * theta_data_log.theta  # [deg]
    omega_pm_p = prescribed_rot_state_data_log.omega_PM_P * macros.R2D  # [deg]
    omega_prime_pm_p = prescribed_rot_state_data_log.omegaPrime_PM_P * macros.R2D  # [deg]
    r_pm_m = prescribed_trans_state_data_log.r_PM_M
    r_prime_pm_m = prescribed_trans_state_data_log.rPrime_PM_M
    r_prime_prime_pm_m = prescribed_trans_state_data_log.rPrimePrime_PM_M

    # Setup the conservation quantities
    initialOrbAngMom_N = [[orbAngMom_N[0, 1], orbAngMom_N[0, 2], orbAngMom_N[0, 3]]]
    finalOrbAngMom = [orbAngMom_N[-1]]
    initialRotAngMom_N = [[rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]]
    finalRotAngMom = [rotAngMom_N[-1]]
    initialOrbEnergy = [[orbEnergy[0, 1]]]
    finalOrbEnergy = [orbEnergy[-1]]
    initialRotEnergy = [[rotEnergy[0, 1]]]
    finalRotEnergy = [rotEnergy[-1]]

    # Plot conservation quantities
    plt.figure()
    plt.clf()
    plt.plot(timespan, (orbAngMom_N[:, 1] - orbAngMom_N[0, 1]) / orbAngMom_N[0, 1], color="teal", label=r'$\hat{n}_1$')
    plt.plot(timespan, (orbAngMom_N[:, 2] - orbAngMom_N[0, 2]) / orbAngMom_N[0, 2], color="darkviolet", label=r'$\hat{n}_2$')
    plt.plot(timespan, (orbAngMom_N[:, 3] - orbAngMom_N[0, 3]) / orbAngMom_N[0, 3], color="blue", label=r'$\hat{n}_3$')
    # plt.title('Orbital Angular Momentum Relative Difference', fontsize=16)
    plt.ylabel('Relative Difference (Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='lower right', prop={'size': 16})
    plt.grid(True)

    plt.figure()
    plt.clf()
    plt.plot(timespan, (orbEnergy[:, 1] - orbEnergy[0, 1]) / orbEnergy[0, 1], color="teal")
    # plt.title('Orbital Energy Relative Difference', fontsize=16)
    plt.ylabel('Relative Difference (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    plt.figure()
    plt.clf()
    plt.plot(timespan, (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]) / rotAngMom_N[0, 1], color="teal", label=r'$\hat{n}_1$')
    plt.plot(timespan, (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]) / rotAngMom_N[0, 1], color="darkviolet", label=r'$\hat{n}_2$')
    plt.plot(timespan, (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]) / rotAngMom_N[0, 2], color="blue", label=r'$\hat{n}_3$')
    # plt.title('Rotational Angular Momentum Relative Difference', fontsize=16)
    plt.ylabel('Relative Difference (Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='upper right', prop={'size': 16})
    plt.grid(True)

    plt.figure()
    plt.clf()
    plt.plot(timespan, (rotEnergy[:, 1] - rotEnergy[0, 1]), color="teal")
    # plt.title('Rotational Energy Relative Difference', fontsize=16)
    plt.ylabel('Difference (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    theta_ref_plotting = np.ones(len(timespan)) * prescribedThetaRef * macros.R2D  # [deg]
    rhoRefs = np.ones(len(timespan)) * posRef  # [m]
    rhos = np.linalg.norm(r_pm_m, axis=1)  # [m]
    rhoDots = np.linalg.norm(r_prime_pm_m, axis=1)  # [m/s]
    rhoDDots = np.linalg.norm(r_prime_prime_pm_m, axis=1)  # [m/s^2]
    thetaDots = np.linalg.norm(omega_pm_p, axis=1)  # [deg/s]
    thetaDDots = np.linalg.norm(omega_prime_pm_p, axis=1)  # [deg/s^2]

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
    ax1.plot(timespan, omega_prime_pm_p[:, 0], label=r"$\ddot{\theta}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("Angular Acceleration (deg/$s^2$)", color="teal", fontsize=16)
    # ax1.set_ylim(-0.6, 0.6)
    ax2 = ax1.twinx()
    ax2.plot(timespan, 100.0 * r_prime_prime_pm_m[:, 0], label=r"$\ddot{\rho}$", color="darkviolet")
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


if __name__ == "__main__":
    spinningBody(True, 0.0, False, 0.0 * macros.D2R, 0.0, False, 0.0 * macros.D2R)
