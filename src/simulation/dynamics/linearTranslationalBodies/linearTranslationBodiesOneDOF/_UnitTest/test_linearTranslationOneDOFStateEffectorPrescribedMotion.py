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
#   Module Name:        translatingBodies and prescribedMotion
#   Author:             Leah Kiner
#   Creation Date:      April 1, 2025
#

import inspect
import os
import matplotlib.pyplot as plt
import numpy
import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import spacecraft, linearTranslationOneDOFStateEffector, gravityEffector, prescribedMotionStateEffector
from Basilisk.simulation import prescribedLinearTranslation
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.architecture import messaging
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import vizSupport


# uncomment this line if this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail()
# provide a unique test method name, starting with test_


# tests are paramterized by four functions
@pytest.mark.parametrize("function", ["translatingBodyNoInput"
    , "translatingBodyLockFlag"
    , "translatingBodyCommandedForce"
    , "translatingBodyRhoReference"
                                      ])
def test_translatingBody(show_plots, function):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft with a single-axis translating rigid body attached to a rigid hub. The position
    of the boom axis is arbitrary. The scenario includes gravity acting on both the spacecraft and the effector.

    **Description of Variables Being Tested**

    In this file we are checking the principles of conservation of energy and angular momentum. Both the orbital and
    rotational energy and angular momentum must be maintained when conservative forces like gravity are present.
    Therefore, the values of the variables

    - ``finalOrbAngMom``
    - ``finalOrbEnergy``
    - ``finalRotAngMom``
    - ``finalRotEnergy``

    should be constant when tested against their initial values.
    """
    if function == "translatingBodyCommandedForce":
        eval(function + '(show_plots, 1.0)')
    elif function == "translatingBodyRhoReference":
        eval(function + '(show_plots, 0.5)')
    else:
        eval(function + '(show_plots)')


# rho ref and cmd force are zero, no lock flag
def translatingBodyNoInput(show_plots):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.0001)  # update process rate update time
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
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]
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

    # Create a linear translating effector
    translatingBody = linearTranslationOneDOFStateEffector.linearTranslationOneDOFStateEffector()
    translatingBody.ModelTag = "translatingBody"
    mass = 20.0
    rhoInit = 0.0
    rhoDotInit = 0.0
    fHat_B = [[1.0], [0.0], [0.0]]
    r_FcF_F = [[0.0], [0.0], [0.0]]
    r_F0B_B = [[1.0], [0.0], [0.0]]
    IPntFc_F = [[50.0, 0.0, 0.0],
                [0.0, 80.0, 0.0],
                [0.0, 0.0, 60.0]]
    dcm_FB = [[1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0]]
    k = 100.0
    c = 0

    translatingBody.setMass(mass)
    translatingBody.setK(k)
    translatingBody.setC(c)
    translatingBody.setRhoInit(rhoInit)
    translatingBody.setRhoDotInit(rhoDotInit)
    translatingBody.setFHat_B(fHat_B)
    translatingBody.setR_FcF_F(r_FcF_F)
    translatingBody.setR_F0B_B(r_F0B_B)
    translatingBody.setIPntFc_F(IPntFc_F)
    translatingBody.setDCM_FB(dcm_FB)
    unitTestSim.AddModelToTask(unitTaskName, translatingBody)

    # Add the translating body to the prescribed motion platform
    # scObject.addStateEffector(translatingBody)
    platform.addStateEffector(translatingBody)

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
    rhoData = translatingBody.translatingBodyOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)
    unitTestSim.AddModelToTask(unitTaskName, rhoData)

    # Add Vizard
    scBodyList = [scObject]
    scBodyList.append(["platform", platform.prescribedMotionConfigLogOutMsg])
    scBodyList.append(["translatingBody", translatingBody.translatingBodyConfigLogOutMsg])

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
                                     , simBodiesToModify=["translatingBody"]
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
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    orbEnergy = scObjectLog.totOrbEnergy
    rho = rhoData.rho
    rhoDot = rhoData.rhoDot

    # Set up the conservation quantities
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
    plt.clf()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rho)
    plt.xlabel('time (s)')
    plt.ylabel('rho')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rhoDot)
    plt.xlabel('time (s)')
    plt.ylabel('rhoDot')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy, err_msg="Orbital energy is not constant.")
    np.testing.assert_allclose(finalRotEnergy, initialRotEnergy, rtol=accuracy, err_msg="Rotational energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy, err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy, err_msg="Rotational angular momentum is not constant.")


# rho ref and cmd force are zero, lock flag is enabled
def translatingBodyLockFlag(show_plots):
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
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]
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
    prescribedThetaRef = 10.0 * macros.D2R  # [rad]
    prescribedRotationMessageData = messaging.HingedRigidBodyMsgPayload()
    prescribedRotationMessageData.theta = prescribedThetaRef
    prescribedRotationMessageData.thetaDot = 0.0  # [rad/s]
    prescribedRotationMessage = messaging.HingedRigidBodyMsg().write(prescribedRotationMessageData)
    prescribedRotation.translatingBodyInMsg.subscribeTo(prescribedRotationMessage)
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

    # Create a linear translating effector
    translatingBody = linearTranslationOneDOFStateEffector.linearTranslationOneDOFStateEffector()
    translatingBody.ModelTag = "translatingBody"
    mass = 20.0
    rhoInit = 0.0
    rhoDotInit = 0.0
    fHat_B = [[1.0], [0.0], [0.0]]
    r_FcF_F = [[0.0], [0.0], [0.0]]
    r_F0B_B = [[1.0], [0.0], [0.0]]
    IPntFc_F = [[50.0, 0.0, 0.0],
                [0.0, 80.0, 0.0],
                [0.0, 0.0, 60.0]]
    dcm_FB = [[1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0]]
    k = 100.0
    c = 0

    translatingBody.setMass(mass)
    translatingBody.setK(k)
    translatingBody.setC(c)
    translatingBody.setRhoInit(rhoInit)
    translatingBody.setRhoDotInit(rhoDotInit)
    translatingBody.setFHat_B(fHat_B)
    translatingBody.setR_FcF_F(r_FcF_F)
    translatingBody.setR_F0B_B(r_F0B_B)
    translatingBody.setIPntFc_F(IPntFc_F)
    translatingBody.setDCM_FB(dcm_FB)
    unitTestSim.AddModelToTask(unitTaskName, translatingBody)

    # Add the translating body to the prescribed motion platform
    # scObject.addStateEffector(translatingBody)
    platform.addStateEffector(translatingBody)

    # create lock message
    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.effectorLockFlag = [1]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    translatingBody.motorLockInMsg.subscribeTo(lockMsg)

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
    rhoData = translatingBody.translatingBodyOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)
    unitTestSim.AddModelToTask(unitTaskName, rhoData)

    # Add Vizard
    scBodyList = [scObject]
    scBodyList.append(["platform", platform.prescribedMotionConfigLogOutMsg])
    scBodyList.append(["translatingBody", translatingBody.translatingBodyConfigLogOutMsg])

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
                                     , simBodiesToModify=["translatingBody"]
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
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    orbEnergy = scObjectLog.totOrbEnergy
    rho = rhoData.rho
    rhoDot = rhoData.rhoDot

    # Set up the conservation quantities
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
    plt.clf()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rho)
    plt.xlabel('time (s)')
    plt.ylabel('rho')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rhoDot)
    plt.xlabel('time (s)')
    plt.ylabel('rhoDot')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy, err_msg="Orbital energy is not constant.")
    np.testing.assert_allclose(finalRotEnergy, initialRotEnergy, rtol=accuracy, err_msg="Rotational energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy, err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy, err_msg="Rotational angular momentum is not constant.")


# cmd force is nonzero, rho ref is zero, no lock flag
def translatingBodyCommandedForce(show_plots, cmdForce):
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
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]
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
    prescribedThetaRef = 10.0 * macros.D2R  # [rad]
    prescribedRotationMessageData = messaging.HingedRigidBodyMsgPayload()
    prescribedRotationMessageData.theta = prescribedThetaRef
    prescribedRotationMessageData.thetaDot = 0.0  # [rad/s]
    prescribedRotationMessage = messaging.HingedRigidBodyMsg().write(prescribedRotationMessageData)
    prescribedRotation.translatingBodyInMsg.subscribeTo(prescribedRotationMessage)
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

    # Create a linear translating effector
    translatingBody = linearTranslationOneDOFStateEffector.linearTranslationOneDOFStateEffector()
    translatingBody.ModelTag = "translatingBody"
    mass = 20.0
    rhoInit = 0.0
    rhoDotInit = 0.0
    fHat_B = [[1.0], [0.0], [0.0]]
    r_FcF_F = [[0.0], [0.0], [0.0]]
    r_F0B_B = [[1.0], [0.0], [0.0]]
    IPntFc_F = [[50.0, 0.0, 0.0],
                [0.0, 80.0, 0.0],
                [0.0, 0.0, 60.0]]
    dcm_FB = [[1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0]]
    k = 100.0
    c = 0

    translatingBody.setMass(mass)
    translatingBody.setK(k)
    translatingBody.setC(c)
    translatingBody.setRhoInit(rhoInit)
    translatingBody.setRhoDotInit(rhoDotInit)
    translatingBody.setFHat_B(fHat_B)
    translatingBody.setR_FcF_F(r_FcF_F)
    translatingBody.setR_F0B_B(r_F0B_B)
    translatingBody.setIPntFc_F(IPntFc_F)
    translatingBody.setDCM_FB(dcm_FB)
    unitTestSim.AddModelToTask(unitTaskName, translatingBody)

    # Add the spinning body to the prescribed motion platform
    # scObject.addStateEffector(translatingBody)
    platform.addStateEffector(translatingBody)

    # Create the force cmd force message
    cmdArray = messaging.ArrayMotorForceMsgPayload()
    cmdArray.motorForce = [cmdForce]  # [Nm]
    cmdMsg = messaging.ArrayMotorForceMsg().write(cmdArray)
    translatingBody.motorForceInMsg.subscribeTo(cmdMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBody)
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
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy"])
    rhoData = translatingBody.translatingBodyOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)
    unitTestSim.AddModelToTask(unitTaskName, rhoData)

    # Add Vizard
    scBodyList = [scObject]
    scBodyList.append(["platform", platform.prescribedMotionConfigLogOutMsg])
    scBodyList.append(["translatingBody", translatingBody.translatingBodyConfigLogOutMsg])

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
                                     , simBodiesToModify=["translatingBody"]
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
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    orbEnergy = scObjectLog.totOrbEnergy
    rho = rhoData.rho
    rhoDot = rhoData.rhoDot

    # Set up the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rho)
    plt.xlabel('time (s)')
    plt.ylabel('rho')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rhoDot)
    plt.xlabel('time (s)')
    plt.ylabel('rhoDot')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy, err_msg="Orbital energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy, err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy, err_msg="Rotational angular momentum is not constant.")


# rho ref is nonzero, cmd force is zero and lock flag is false
def translatingBodyRhoReference(show_plots, rhoRef):
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
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]
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

    # Create a linear translating effector
    translatingBody = linearTranslationOneDOFStateEffector.linearTranslationOneDOFStateEffector()
    translatingBody.ModelTag = "translatingBody"
    mass = 20.0
    rhoInit = 0.0
    rhoDotInit = 0.0
    fHat_B = [[1.0], [0.0], [0.0]]
    r_FcF_F = [[0.0], [0.0], [0.0]]
    r_F0B_B = [[1.0], [0.0], [0.0]]
    IPntFc_F = [[50.0, 0.0, 0.0],
                [0.0, 80.0, 0.0],
                [0.0, 0.0, 60.0]]
    dcm_FB = [[1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0]]
    k = 100.0
    c = 0

    translatingBody.setMass(mass)
    translatingBody.setK(k)
    translatingBody.setC(c)
    translatingBody.setRhoInit(rhoInit)
    translatingBody.setRhoDotInit(rhoDotInit)
    translatingBody.setFHat_B(fHat_B)
    translatingBody.setR_FcF_F(r_FcF_F)
    translatingBody.setR_F0B_B(r_F0B_B)
    translatingBody.setIPntFc_F(IPntFc_F)
    translatingBody.setDCM_FB(dcm_FB)
    unitTestSim.AddModelToTask(unitTaskName, translatingBody)

    # Add the spinning body to the prescribed motion platform
    # scObject.addStateEffector(translatingBody)
    platform.addStateEffector(translatingBody)

    # Create the reference message
    translationRef = messaging.LinearTranslationRigidBodyMsgPayload()
    translationRef.rho = rhoRef
    translationRef.rhoDot = 0.0
    translationRefMsg = messaging.LinearTranslationRigidBodyMsg().write(translationRef)
    translatingBody.translatingBodyRefInMsg.subscribeTo(translationRefMsg)

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
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy"])
    rhoData = translatingBody.translatingBodyOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)
    unitTestSim.AddModelToTask(unitTaskName, rhoData)

    # Add Vizard
    scBodyList = [scObject]
    scBodyList.append(["platform", platform.prescribedMotionConfigLogOutMsg])
    scBodyList.append(["translatingBody", translatingBody.translatingBodyConfigLogOutMsg])

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
                                     , simBodiesToModify=["translatingBody"]
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
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    orbEnergy = scObjectLog.totOrbEnergy
    rho = rhoData.rho
    rhoDot = rhoData.rhoDot

    # Set up the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    finalAngle = rho[-1]

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rho)
    plt.xlabel('time (s)')
    plt.ylabel('rho')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rhoDot)
    plt.xlabel('time (s)')
    plt.ylabel('rhoDot')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalAngle, rhoRef, atol=0.01, err_msg="Angle doesn't settle to reference angle.")
    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy, err_msg="Orbital energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy, err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy, err_msg="Rotational angular momentum is not constant.")


if __name__ == "__main__":
    # translatingBodyNoInput(True)
    # translatingBodyLockFlag(True)
    # translatingBodyCommandedForce(True, 1)
    translatingBodyRhoReference(True, 0.5)
