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
#   Prescribed Motion Branching Scenario
#   Author:             Leah Kiner
#   Creation Date:      April 11, 2025
#

import inspect
import matplotlib.pyplot as plt
import numpy as np
import os

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import prescribedMotionStateEffector, spacecraft, spinningBodyTwoDOFStateEffector
from Basilisk.simulation import prescribedLinearTranslation
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.simulation import gravityEffector
from Basilisk.architecture import messaging
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import vizSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

def run(show_plots):
    """
    The scenario can be run with the followings set up parameter:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    simProcessName = "simProcess"
    dynTaskName = "dynTask"
    fswTaskName = "fswTask"
    dataRecTaskName = "dataRecTask"

    #   Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    dynTimeStep = 0.01  # [s]
    fswTimeStep = 0.1  # [s]
    dataRecStep = 0.1  # [s]
    dynProcessRate = macros.sec2nano(dynTimeStep)  # [ns]
    fswProcessRate = macros.sec2nano(fswTimeStep)  # [ns]
    dataRecRate = macros.sec2nano(dataRecStep)  # [ns]
    simProc = scSim.CreateNewProcess(simProcessName)
    simProc.addTask(scSim.CreateNewTask(dynTaskName, dynProcessRate))
    simProc.addTask(scSim.CreateNewTask(fswTaskName, fswProcessRate))
    simProc.addTask(scSim.CreateNewTask(dataRecTaskName, dataRecRate))

    # Create the spacecraft module
    massHub = 15000  # [kg]
    lengthHub = 8  # [m]
    widthHub = 8  # [m]
    depthHub = 20  # [m]
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
    scObject.hub.omega_BN_BInit = [[0.001], [-0.001], [0.001]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scSim.AddModelToTask(dynTaskName, scObject)

    # Shared prescribed array boom parameters
    posInit = 0.0
    thetaInit = 0.0
    transAxis_M = np.array([1.0, 0.0, 0.0])
    rotAxis_M = np.array([1.0, 0.0, 0.0])
    prvInit_PM = thetaInit * rotAxis_M
    sigma_PM = rbk.PRV2MRP(prvInit_PM)

    massPrescribed = 6000.0  # [kg]
    lengthPrescribed = 50.0  # [m]
    widthPrescribed = 4.0  # [m]
    depthPrescribed = 4.0  # [m]
    IPrescribed_11 = (1 / 12) * massPrescribed * (lengthPrescribed * lengthPrescribed + depthPrescribed * depthPrescribed)  # [kg m^2]
    IPrescribed_22 = (1 / 12) * massPrescribed * (lengthPrescribed * lengthPrescribed + widthPrescribed * widthPrescribed)  # [kg m^2]
    IPrescribed_33 = (1 / 12) * massPrescribed * (widthPrescribed * widthPrescribed + depthPrescribed * depthPrescribed)  # [kg m^2]
    IPrescribed_Pc_P = [[IPrescribed_11, 0.0, 0.0], [0.0, IPrescribed_22, 0.0], [0.0, 0.0,IPrescribed_33]]  # [kg m^2]

    # Create prescribed array boom 1
    prescribedTruss1 = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribedTruss1.ModelTag = "prescribedTruss1"
    prescribedTruss1.mass = massPrescribed
    prescribedTruss1.IPntPc_P = IPrescribed_Pc_P
    prescribedTruss1.r_MB_B = [4.0, 0.0, 0.0]
    prescribedTruss1.r_PcP_P = [25.0, 0.0, 0.0]
    prescribedTruss1.r_PM_M = [0.0, 0.0, 0.0]
    prescribedTruss1.rPrime_PM_M = np.array([0.0, 0.0, 0.0])
    prescribedTruss1.rPrimePrime_PM_M = np.array([0.0, 0.0, 0.0])
    prescribedTruss1.omega_PM_P = np.array([0.0, 0.0, 0.0])
    prescribedTruss1.omegaPrime_PM_P = np.array([0.0, 0.0, 0.0])
    prescribedTruss1.sigma_PM = sigma_PM
    prescribedTruss1.omega_MB_B = [0.0, 0.0, 0.0]
    prescribedTruss1.omegaPrime_MB_B = [0.0, 0.0, 0.0]
    prescribedTruss1.sigma_MB = [0.0, 0.0, 0.0]
    scSim.AddModelToTask(dynTaskName, prescribedTruss1)
    scObject.addStateEffector(prescribedTruss1)

    # Create prescribed array boom 2
    prescribedTruss2 = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribedTruss2.ModelTag = "prescribedTruss2"
    prescribedTruss2.mass = massPrescribed
    prescribedTruss2.IPntPc_P = IPrescribed_Pc_P
    prescribedTruss2.r_MB_B = [-4.0, 0.0, 0.0]
    prescribedTruss2.r_PcP_P = [25.0, 0.0, 0.0]
    prescribedTruss2.r_PM_M = [0.0, 0.0, 0.0]
    prescribedTruss2.rPrime_PM_M = np.array([0.0, 0.0, 0.0])
    prescribedTruss2.rPrimePrime_PM_M = np.array([0.0, 0.0, 0.0])
    prescribedTruss2.omega_PM_P = np.array([0.0, 0.0, 0.0])
    prescribedTruss2.omegaPrime_PM_P = np.array([0.0, 0.0, 0.0])
    prescribedTruss2.sigma_PM = sigma_PM
    prescribedTruss2.omega_MB_B = [0.0, 0.0, 0.0]
    prescribedTruss2.omegaPrime_MB_B = [0.0, 0.0, 0.0]
    dcm_MB = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
    prescribedTruss2.sigma_MB = rbk.C2MRP(dcm_MB)
    scSim.AddModelToTask(dynTaskName, prescribedTruss2)
    scObject.addStateEffector(prescribedTruss2)

    # Shared prescribed motion profiler parameters
    angAccelMax = 0.5 * macros.D2R  # [rad/s^2]
    transAccelMax = 0.005  # [m/s^2]
    prescribedThetaRef = -90.0 * macros.D2R  # [rad]
    prescribedPosRef = 0.0  # [m]

    # Create prescribed array boom 1 rotational motion profiler
    prescribedRotation1 = prescribedRotation1DOF.PrescribedRotation1DOF()
    prescribedRotation1.ModelTag = "prescribedRotation1DOF_1"
    prescribedRotation1.setRotHat_M(rotAxis_M)
    prescribedRotation1.setThetaDDotMax(angAccelMax)
    prescribedRotation1.setThetaInit(thetaInit)
    prescribedRotation1.setCoastOptionBangDuration(1.0)
    prescribedRotation1.setSmoothingDuration(1.0)
    scSim.AddModelToTask(dynTaskName, prescribedRotation1)

    # Create prescribed array boom 2 rotational motion profiler
    prescribedRotation2 = prescribedRotation1DOF.PrescribedRotation1DOF()
    prescribedRotation2.ModelTag = "prescribedRotation1DOF_2"
    prescribedRotation2.setRotHat_M(rotAxis_M)
    prescribedRotation2.setThetaDDotMax(angAccelMax)
    prescribedRotation2.setThetaInit(thetaInit)
    prescribedRotation2.setCoastOptionBangDuration(1.0)
    prescribedRotation2.setSmoothingDuration(1.0)
    scSim.AddModelToTask(dynTaskName, prescribedRotation2)

    # Create prescribed array boom 1 rotational motion reference message
    prescribedRotation1MessageData = messaging.HingedRigidBodyMsgPayload()
    prescribedRotation1MessageData.theta = prescribedThetaRef
    prescribedRotation1MessageData.thetaDot = 0.0  # [rad/s]
    prescribedRotation1Message = messaging.HingedRigidBodyMsg().write(prescribedRotation1MessageData)
    prescribedRotation1.spinningBodyInMsg.subscribeTo(prescribedRotation1Message)
    prescribedTruss1.prescribedRotationInMsg.subscribeTo(prescribedRotation1.prescribedRotationOutMsg)

    # Create prescribed array boom 2 rotational motion reference message
    prescribedRotation2MessageData = messaging.HingedRigidBodyMsgPayload()
    prescribedRotation2MessageData.theta = - prescribedThetaRef
    prescribedRotation2MessageData.thetaDot = 0.0  # [rad/s]
    prescribedRotation2Message = messaging.HingedRigidBodyMsg().write(prescribedRotation2MessageData)
    prescribedRotation2.spinningBodyInMsg.subscribeTo(prescribedRotation2Message)
    prescribedTruss2.prescribedRotationInMsg.subscribeTo(prescribedRotation2.prescribedRotationOutMsg)

    # Create prescribed array boom 1 translational motion profiler
    prescribedTranslation1 = prescribedLinearTranslation.PrescribedLinearTranslation()
    prescribedTranslation1.ModelTag = "prescribedLinearTranslation_1"
    prescribedTranslation1.setTransHat_M(transAxis_M)
    prescribedTranslation1.setTransAccelMax(transAccelMax)
    prescribedTranslation1.setTransPosInit(posInit)
    prescribedTranslation1.setCoastOptionBangDuration(1.0)
    prescribedTranslation1.setSmoothingDuration(1.0)
    scSim.AddModelToTask(dynTaskName, prescribedTranslation1)

    # Create prescribed array boom 2 translational motion profiler
    prescribedTranslation2 = prescribedLinearTranslation.PrescribedLinearTranslation()
    prescribedTranslation2.ModelTag = "prescribedLinearTranslation_2"
    prescribedTranslation2.setTransHat_M(transAxis_M)
    prescribedTranslation2.setTransAccelMax(transAccelMax)
    prescribedTranslation2.setTransPosInit(posInit)
    prescribedTranslation2.setCoastOptionBangDuration(1.0)
    prescribedTranslation2.setSmoothingDuration(1.0)
    scSim.AddModelToTask(dynTaskName, prescribedTranslation2)

    # Create prescribed array boom translational motion reference message
    prescribedTranslationMessageData = messaging.LinearTranslationRigidBodyMsgPayload()
    prescribedTranslationMessageData.rho = prescribedPosRef
    prescribedTranslationMessageData.rhoDot = 0.0
    prescribedTranslationMessage = messaging.LinearTranslationRigidBodyMsg().write(prescribedTranslationMessageData)
    prescribedTranslation1.linearTranslationRigidBodyInMsg.subscribeTo(prescribedTranslationMessage)
    prescribedTranslation2.linearTranslationRigidBodyInMsg.subscribeTo(prescribedTranslationMessage)
    prescribedTruss1.prescribedTranslationInMsg.subscribeTo(prescribedTranslation1.prescribedTranslationOutMsg)
    prescribedTruss2.prescribedTranslationInMsg.subscribeTo(prescribedTranslation2.prescribedTranslationOutMsg)

    # Shared spinning body parameters
    massSolarArray = 1000.0  # [kg]
    lengthSolarArray = 10.0  # [m]
    widthSolarArray = 0.3  # [m]
    depthSolarArray = 30.0  # [m]
    ISolarArray_11 = (1 / 12) * massSolarArray * (lengthSolarArray * lengthSolarArray + depthSolarArray * depthSolarArray)  # [kg m^2]
    ISolarArray_22 = (1 / 12) * massSolarArray * (lengthSolarArray * lengthSolarArray + widthSolarArray * widthSolarArray)  # [kg m^2]
    ISolarArray_33 = (1 / 12) * massSolarArray * (widthPrescribed * widthPrescribed + depthSolarArray * depthSolarArray)  # [kg m^2]
    ISolarArray_ScS = [[ISolarArray_11, 0.0, 0.0], [0.0, ISolarArray_22, 0.0], [0.0, 0.0, ISolarArray_33]]  # [kg m^2]
    r_ScS_S = [[0.0], [0.0], [15.0]]
    r_S1B_B = [[50.0], [0.0], [2.0]]  # [m[ r_S1P_P
    r_S2B_B = [[35.0], [0.0], [2.0]]  # [m[ r_S2P_P
    r_S3B_B = [[50.0], [0.0], [-2.0]]  # [m[ r_S3P_P
    r_S4B_B = [[35.0], [0.0], [-2.0]]  # [m[ r_S4P_P
    r_S5B_B = [[35.0], [0.0], [-2.0]]  # [m[ r_S5P_P
    r_S6B_B = [[50.0], [0.0], [-2.0]]  # [m[ r_S6P_P
    r_S7B_B = [[35.0], [0.0], [2.0]]  # [m[ r_S7P_P
    r_S8B_B = [[50.0], [0.0], [2.0]]  # [m[ r_S8P_P
    s1Hat_S = [[0.0], [0.0], [1.0]]
    s2Hat_S = [[1.0], [0.0], [0.0]]
    thetaInit = 10.0 * macros.D2R  # [rad]
    thetaDotInit = 2.0 * macros.D2R  # [rad/s]
    k = 10000.0
    c = 8000.0
    dcm_S01B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]  # dcm_S01P
    dcm_S02B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]  # dcm_S02P
    dcm_S03B = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]  # dcm_S03P
    dcm_S04B = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]  # dcm_S04P
    dcm_S05B = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]  # dcm_S05P
    dcm_S06B = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]  # dcm_S06P
    dcm_S07B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]  # dcm_S07P
    dcm_S08B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]  # dcm_S08P

    # Create the spinning bodies
    spinningBody1 = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody2 = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody3 = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody4 = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody5 = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody6 = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody7 = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody8 = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()

    spinningBody1.ModelTag = "spinningBody1"
    spinningBody2.ModelTag = "spinningBody2"
    spinningBody3.ModelTag = "spinningBody3"
    spinningBody4.ModelTag = "spinningBody4"
    spinningBody5.ModelTag = "spinningBody5"
    spinningBody6.ModelTag = "spinningBody6"
    spinningBody7.ModelTag = "spinningBody7"
    spinningBody8.ModelTag = "spinningBody8"

    spinningBody1.mass1 = 0.0
    spinningBody2.mass1 = 0.0
    spinningBody3.mass1 = 0.0
    spinningBody4.mass1 = 0.0
    spinningBody5.mass1 = 0.0
    spinningBody6.mass1 = 0.0
    spinningBody7.mass1 = 0.0
    spinningBody8.mass1 = 0.0

    spinningBody1.mass2 = massSolarArray
    spinningBody2.mass2 = massSolarArray
    spinningBody3.mass2 = massSolarArray
    spinningBody4.mass2 = massSolarArray
    spinningBody5.mass2 = massSolarArray
    spinningBody6.mass2 = massSolarArray
    spinningBody7.mass2 = massSolarArray
    spinningBody8.mass2 = massSolarArray

    spinningBody1.IS1PntSc1_S1 = [[0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0]]
    spinningBody2.IS1PntSc1_S1 = [[0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0]]
    spinningBody3.IS1PntSc1_S1 = [[0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0]]
    spinningBody4.IS1PntSc1_S1 = [[0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0]]
    spinningBody5.IS1PntSc1_S1 = [[0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0]]
    spinningBody6.IS1PntSc1_S1 = [[0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0]]
    spinningBody7.IS1PntSc1_S1 = [[0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0]]
    spinningBody8.IS1PntSc1_S1 = [[0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0]]

    spinningBody1.IS2PntSc2_S2 = ISolarArray_ScS
    spinningBody2.IS2PntSc2_S2 = ISolarArray_ScS
    spinningBody3.IS2PntSc2_S2 = ISolarArray_ScS
    spinningBody4.IS2PntSc2_S2 = ISolarArray_ScS
    spinningBody5.IS2PntSc2_S2 = ISolarArray_ScS
    spinningBody6.IS2PntSc2_S2 = ISolarArray_ScS
    spinningBody7.IS2PntSc2_S2 = ISolarArray_ScS
    spinningBody8.IS2PntSc2_S2 = ISolarArray_ScS

    spinningBody1.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody2.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody3.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody4.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody5.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody6.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody7.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody8.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    spinningBody1.dcm_S10B = dcm_S01B
    spinningBody2.dcm_S10B = dcm_S02B
    spinningBody3.dcm_S10B = dcm_S03B
    spinningBody4.dcm_S10B = dcm_S04B
    spinningBody5.dcm_S10B = dcm_S05B
    spinningBody6.dcm_S10B = dcm_S06B
    spinningBody7.dcm_S10B = dcm_S07B
    spinningBody8.dcm_S10B = dcm_S08B

    spinningBody1.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody2.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody3.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody4.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody5.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody6.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody7.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody8.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]

    spinningBody1.r_Sc2S2_S2 = r_ScS_S
    spinningBody2.r_Sc2S2_S2 = r_ScS_S
    spinningBody3.r_Sc2S2_S2 = r_ScS_S
    spinningBody4.r_Sc2S2_S2 = r_ScS_S
    spinningBody5.r_Sc2S2_S2 = r_ScS_S
    spinningBody6.r_Sc2S2_S2 = r_ScS_S
    spinningBody7.r_Sc2S2_S2 = r_ScS_S
    spinningBody8.r_Sc2S2_S2 = r_ScS_S

    spinningBody1.r_S1B_B = r_S1B_B
    spinningBody2.r_S1B_B = r_S2B_B
    spinningBody3.r_S1B_B = r_S3B_B
    spinningBody4.r_S1B_B = r_S4B_B
    spinningBody5.r_S1B_B = r_S5B_B
    spinningBody6.r_S1B_B = r_S6B_B
    spinningBody7.r_S1B_B = r_S7B_B
    spinningBody8.r_S1B_B = r_S8B_B

    spinningBody1.r_S2S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody2.r_S2S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody3.r_S2S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody4.r_S2S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody5.r_S2S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody6.r_S2S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody7.r_S2S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody8.r_S2S1_S1 = [[0.0], [0.0], [0.0]]

    spinningBody1.s1Hat_S1 = s1Hat_S
    spinningBody2.s1Hat_S1 = s1Hat_S
    spinningBody3.s1Hat_S1 = s1Hat_S
    spinningBody4.s1Hat_S1 = s1Hat_S
    spinningBody5.s1Hat_S1 = s1Hat_S
    spinningBody6.s1Hat_S1 = s1Hat_S
    spinningBody7.s1Hat_S1 = s1Hat_S
    spinningBody8.s1Hat_S1 = s1Hat_S

    spinningBody1.s2Hat_S2 = s2Hat_S
    spinningBody2.s2Hat_S2 = s2Hat_S
    spinningBody3.s2Hat_S2 = s2Hat_S
    spinningBody4.s2Hat_S2 = s2Hat_S
    spinningBody5.s2Hat_S2 = s2Hat_S
    spinningBody6.s2Hat_S2 = s2Hat_S
    spinningBody7.s2Hat_S2 = s2Hat_S
    spinningBody8.s2Hat_S2 = s2Hat_S

    spinningBody1.theta1Init = -20.0 * macros.D2R  # [rad]
    spinningBody2.theta1Init = 10.0 * macros.D2R  # [rad]
    spinningBody3.theta1Init = -15.0 * macros.D2R  # [rad]
    spinningBody4.theta1Init = 30.0 * macros.D2R  # [rad]
    spinningBody5.theta1Init = 15.0 * macros.D2R  # [rad]
    spinningBody6.theta1Init = -10.0 * macros.D2R  # [rad]
    spinningBody7.theta1Init = 20.0 * macros.D2R  # [rad]
    spinningBody8.theta1Init = -30.0 * macros.D2R  # [rad]

    spinningBody1.theta2Init = 10.0 * macros.D2R  # [rad]
    spinningBody2.theta2Init = -10.0 * macros.D2R  # [rad]
    spinningBody3.theta2Init = 5.0 * macros.D2R  # [rad]
    spinningBody4.theta2Init = 5.0 * macros.D2R  # [rad]
    spinningBody5.theta2Init = -7.0 * macros.D2R  # [rad]
    spinningBody6.theta2Init = 5.0 * macros.D2R  # [rad]
    spinningBody7.theta2Init = -5.0 * macros.D2R  # [rad]
    spinningBody8.theta2Init = 20.0 * macros.D2R  # [rad]

    spinningBody1.theta2DotInit = thetaDotInit
    spinningBody2.theta2DotInit = thetaDotInit
    spinningBody3.theta2DotInit = thetaDotInit
    spinningBody4.theta2DotInit = thetaDotInit
    spinningBody5.theta2DotInit = thetaDotInit
    spinningBody6.theta2DotInit = thetaDotInit
    spinningBody7.theta2DotInit = thetaDotInit
    spinningBody8.theta2DotInit = thetaDotInit

    spinningBody1.k1 = k
    spinningBody2.k1 = k
    spinningBody3.k1 = k
    spinningBody4.k1 = k
    spinningBody5.k1 = k
    spinningBody6.k1 = k
    spinningBody7.k1 = k
    spinningBody8.k1 = k
    spinningBody1.c1 = c
    spinningBody2.c1 = c
    spinningBody3.c1 = c
    spinningBody4.c1 = c
    spinningBody5.c1 = c
    spinningBody6.c1 = c
    spinningBody7.c1 = c
    spinningBody8.c1 = c

    spinningBody1.k2 = k
    spinningBody2.k2 = k
    spinningBody3.k2 = k
    spinningBody4.k2 = k
    spinningBody5.k2 = k
    spinningBody6.k2 = k
    spinningBody7.k2 = k
    spinningBody8.k2 = k
    spinningBody1.c2 = c
    spinningBody2.c2 = c
    spinningBody3.c2 = c
    spinningBody4.c2 = c
    spinningBody5.c2 = c
    spinningBody6.c2 = c
    spinningBody7.c2 = c
    spinningBody8.c2 = c

    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.effectorLockFlag = [1]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    # spinningBody1.motorLockInMsg.subscribeTo(lockMsg)
    # spinningBody2.motorLockInMsg.subscribeTo(lockMsg)
    # spinningBody3.motorLockInMsg.subscribeTo(lockMsg)
    # spinningBody4.motorLockInMsg.subscribeTo(lockMsg)
    # spinningBody5.motorLockInMsg.subscribeTo(lockMsg)
    # spinningBody6.motorLockInMsg.subscribeTo(lockMsg)
    # spinningBody7.motorLockInMsg.subscribeTo(lockMsg)
    # spinningBody8.motorLockInMsg.subscribeTo(lockMsg)

    scSim.AddModelToTask(dynTaskName, spinningBody1)
    scSim.AddModelToTask(dynTaskName, spinningBody2)
    scSim.AddModelToTask(dynTaskName, spinningBody3)
    scSim.AddModelToTask(dynTaskName, spinningBody4)
    scSim.AddModelToTask(dynTaskName, spinningBody5)
    scSim.AddModelToTask(dynTaskName, spinningBody6)
    scSim.AddModelToTask(dynTaskName, spinningBody7)
    scSim.AddModelToTask(dynTaskName, spinningBody8)

    prescribedTruss1.addStateEffector(spinningBody1)
    prescribedTruss1.addStateEffector(spinningBody2)
    prescribedTruss1.addStateEffector(spinningBody3)
    prescribedTruss1.addStateEffector(spinningBody4)
    prescribedTruss2.addStateEffector(spinningBody5)
    prescribedTruss2.addStateEffector(spinningBody6)
    prescribedTruss2.addStateEffector(spinningBody7)
    prescribedTruss2.addStateEffector(spinningBody8)

    # # Create the torque message
    # cmdTorque = 0.0  # [Nm]
    # cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    # cmdArray.motorTorque = [cmdTorque]
    # cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    # spinningBody1.motorTorqueInMsg.subscribeTo(cmdMsg)
    # spinningBody2.motorTorqueInMsg.subscribeTo(cmdMsg)
    # spinningBody3.motorTorqueInMsg.subscribeTo(cmdMsg)
    # spinningBody4.motorTorqueInMsg.subscribeTo(cmdMsg)
    # spinningBody5.motorTorqueInMsg.subscribeTo(cmdMsg)
    # spinningBody6.motorTorqueInMsg.subscribeTo(cmdMsg)
    # spinningBody7.motorTorqueInMsg.subscribeTo(cmdMsg)
    # spinningBody8.motorTorqueInMsg.subscribeTo(cmdMsg)

    # Create the reference message
    # spinningBodyThetaRef = 0.0 * macros.D2R
    # angleRef = messaging.HingedRigidBodyMsgPayload()
    # angleRef.theta = spinningBodyThetaRef
    # angleRef.thetaDot = 0.0
    # angleRefMsg = messaging.HingedRigidBodyMsg().write(angleRef)
    # spinningBody1.spinningBodyRefInMsg.subscribeTo(angleRefMsg)
    # spinningBody2.spinningBodyRefInMsg.subscribeTo(angleRefMsg)
    # spinningBody3.spinningBodyRefInMsg.subscribeTo(angleRefMsg)
    # spinningBody4.spinningBodyRefInMsg.subscribeTo(angleRefMsg)
    # spinningBody5.spinningBodyRefInMsg.subscribeTs[0]o(angleRefMsg)
    # spinningBody6.spinningBodyRefInMsg.subscribeTo(angleRefMsg)
    # spinningBody7.spinningBodyRefInMsg.subscribeTo(angleRefMsg)
    # spinningBody8.spinningBodyRefInMsg.subscribeTo(angleRefMsg)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Log the spacecraft state message
    scStateDataLog = scObject.scStateOutMsg.recorder()
    prescribedTruss1InertialStateDataLog = prescribedTruss1.prescribedMotionConfigLogOutMsg.recorder()
    prescribedRotStatesDataLog1 = prescribedRotation1.spinningBodyOutMsg.recorder()
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    prescribedRotStatesDataLog2 = prescribedRotation2.spinningBodyOutMsg.recorder()
    spinningBody1State1DataLog = spinningBody1.spinningBodyOutMsgs[0].recorder()
    spinningBody2State1DataLog = spinningBody2.spinningBodyOutMsgs[0].recorder()
    spinningBody3State1DataLog = spinningBody3.spinningBodyOutMsgs[0].recorder()
    spinningBody4State1DataLog = spinningBody4.spinningBodyOutMsgs[0].recorder()
    spinningBody5State1DataLog = spinningBody5.spinningBodyOutMsgs[0].recorder()
    spinningBody6State1DataLog = spinningBody6.spinningBodyOutMsgs[0].recorder()
    spinningBody7State1DataLog = spinningBody7.spinningBodyOutMsgs[0].recorder()
    spinningBody8State1DataLog = spinningBody8.spinningBodyOutMsgs[0].recorder()
    spinningBody1State2DataLog = spinningBody1.spinningBodyOutMsgs[1].recorder()
    spinningBody2State2DataLog = spinningBody2.spinningBodyOutMsgs[1].recorder()
    spinningBody3State2DataLog = spinningBody3.spinningBodyOutMsgs[1].recorder()
    spinningBody4State2DataLog = spinningBody4.spinningBodyOutMsgs[1].recorder()
    spinningBody5State2DataLog = spinningBody5.spinningBodyOutMsgs[1].recorder()
    spinningBody6State2DataLog = spinningBody6.spinningBodyOutMsgs[1].recorder()
    spinningBody7State2DataLog = spinningBody7.spinningBodyOutMsgs[1].recorder()
    spinningBody8State2DataLog = spinningBody8.spinningBodyOutMsgs[1].recorder()
    scSim.AddModelToTask(dataRecTaskName, scStateDataLog)
    scSim.AddModelToTask(dataRecTaskName, scObjectLog)
    scSim.AddModelToTask(dataRecTaskName, prescribedTruss1InertialStateDataLog)
    scSim.AddModelToTask(dataRecTaskName, prescribedRotStatesDataLog1)
    scSim.AddModelToTask(dataRecTaskName, prescribedRotStatesDataLog2)
    scSim.AddModelToTask(dataRecTaskName, spinningBody1State1DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody2State1DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody3State1DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody4State1DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody5State1DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody6State1DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody7State1DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody8State1DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody1State2DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody2State2DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody3State2DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody4State2DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody5State2DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody6State2DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody7State2DataLog)
    scSim.AddModelToTask(dataRecTaskName, spinningBody8State2DataLog)

    # Add Vizard
    scBodyList = [scObject]
    scBodyList.append(["prescribedTruss1", prescribedTruss1.prescribedMotionConfigLogOutMsg])
    scBodyList.append(["prescribedTruss2", prescribedTruss2.prescribedMotionConfigLogOutMsg])
    scBodyList.append(["spinningBody1", spinningBody1.spinningBodyConfigLogOutMsgs[1]])
    scBodyList.append(["spinningBody2", spinningBody2.spinningBodyConfigLogOutMsgs[1]])
    scBodyList.append(["spinningBody3", spinningBody3.spinningBodyConfigLogOutMsgs[1]])
    scBodyList.append(["spinningBody4", spinningBody4.spinningBodyConfigLogOutMsgs[1]])
    scBodyList.append(["spinningBody5", spinningBody5.spinningBodyConfigLogOutMsgs[1]])
    scBodyList.append(["spinningBody6", spinningBody6.spinningBodyConfigLogOutMsgs[1]])
    scBodyList.append(["spinningBody7", spinningBody7.spinningBodyConfigLogOutMsgs[1]])
    scBodyList.append(["spinningBody8", spinningBody8.spinningBodyConfigLogOutMsgs[1]])

    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, dataRecTaskName, scBodyList,
                                                  saveFile=filename
                                                  )

        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scObject.ModelTag]
                                     , modelPath="CUBE"
                                     , scale=[widthHub, lengthHub, depthHub]
                                     , color=vizSupport.toRGBA255("gray"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["prescribedTruss1"]
                                     , modelPath="CUBE"
                                     , scale=[lengthPrescribed, widthPrescribed, depthPrescribed]
                                     , color=vizSupport.toRGBA255("green"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["prescribedTruss2"]
                                     , modelPath="CUBE"
                                     , scale=[lengthPrescribed, widthPrescribed, depthPrescribed]
                                     , color=vizSupport.toRGBA255("green"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["spinningBody1"]
                                     , modelPath="CUBE"
                                     , scale=[lengthSolarArray, widthSolarArray, depthSolarArray]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["spinningBody2"]
                                     , modelPath="CUBE"
                                     , scale=[lengthSolarArray, widthSolarArray, depthSolarArray]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["spinningBody3"]
                                     , modelPath="CUBE"
                                     , scale=[lengthSolarArray, widthSolarArray, depthSolarArray]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["spinningBody4"]
                                     , modelPath="CUBE"
                                     , scale=[lengthSolarArray, widthSolarArray, depthSolarArray]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["spinningBody5"]
                                     , modelPath="CUBE"
                                     , scale=[lengthSolarArray, widthSolarArray, depthSolarArray]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["spinningBody6"]
                                     , modelPath="CUBE"
                                     , scale=[lengthSolarArray, widthSolarArray, depthSolarArray]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["spinningBody7"]
                                     , modelPath="CUBE"
                                     , scale=[lengthSolarArray, widthSolarArray, depthSolarArray]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["spinningBody8"]
                                     , modelPath="CUBE"
                                     , scale=[lengthSolarArray, widthSolarArray, depthSolarArray]
                                     , color=vizSupport.toRGBA255("blue"))

        viz.settings.orbitLinesOn = -1


    # Run the simulation
    scSim.InitializeSimulation()
    simTime = 120.0  # [s]
    scSim.ConfigureStopTime(macros.sec2nano(simTime))
    scSim.ExecuteSimulation()

    # Extract the logged variables
    timespan = scStateDataLog.times() * macros.NANO2SEC  # [s]
    orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)
    orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)
    r_BN_N = scStateDataLog.r_BN_N  # [m]
    v_BN_N = scStateDataLog.v_BN_N  # [m/s]
    sigma_BN = scStateDataLog.sigma_BN
    omega_BN_B = scStateDataLog.omega_BN_B * macros.R2D  # [deg/s]
    r_P1cN_N = prescribedTruss1InertialStateDataLog.r_BN_N  # [m]
    sigma_P1N = prescribedTruss1InertialStateDataLog.sigma_BN
    omega_P1N_P1 = prescribedTruss1InertialStateDataLog.omega_BN_B * macros.R2D  # [deg/s]
    prescribedTheta1 = prescribedRotStatesDataLog1.theta * macros.R2D  # [deg]
    prescribedTheta2 = prescribedRotStatesDataLog2.theta * macros.R2D  # [deg]
    spinningBody1Theta1 = spinningBody1State1DataLog.theta * macros.R2D  # [deg]
    spinningBody2Theta1 = spinningBody2State1DataLog.theta * macros.R2D  # [deg]
    spinningBody3Theta1 = spinningBody3State1DataLog.theta * macros.R2D  # [deg]
    spinningBody4Theta1 = spinningBody4State1DataLog.theta * macros.R2D  # [deg]
    spinningBody5Theta1 = spinningBody5State1DataLog.theta * macros.R2D  # [deg]
    spinningBody6Theta1 = spinningBody6State1DataLog.theta * macros.R2D  # [deg]
    spinningBody7Theta1 = spinningBody7State1DataLog.theta * macros.R2D  # [deg]
    spinningBody8Theta1 = spinningBody8State1DataLog.theta * macros.R2D  # [deg]
    spinningBody1Theta2 = spinningBody1State2DataLog.theta * macros.R2D  # [deg]
    spinningBody2Theta2 = spinningBody2State2DataLog.theta * macros.R2D  # [deg]
    spinningBody3Theta2 = spinningBody3State2DataLog.theta * macros.R2D  # [deg]
    spinningBody4Theta2 = spinningBody4State2DataLog.theta * macros.R2D  # [deg]
    spinningBody5Theta2 = spinningBody5State2DataLog.theta * macros.R2D  # [deg]
    spinningBody6Theta2 = spinningBody6State2DataLog.theta * macros.R2D  # [deg]
    spinningBody7Theta2 = spinningBody7State2DataLog.theta * macros.R2D  # [deg]
    spinningBody8Theta2 = spinningBody8State2DataLog.theta * macros.R2D  # [deg]
    spinningBody1ThetaDot1 = spinningBody1State1DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody2ThetaDot1 = spinningBody2State1DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody3ThetaDot1 = spinningBody3State1DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody4ThetaDot1 = spinningBody4State1DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody5ThetaDot1 = spinningBody5State1DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody6ThetaDot1 = spinningBody6State1DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody7ThetaDot1 = spinningBody7State1DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody8ThetaDot1 = spinningBody8State1DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody1ThetaDot2 = spinningBody1State2DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody2ThetaDot2 = spinningBody2State2DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody3ThetaDot2 = spinningBody3State2DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody4ThetaDot2 = spinningBody4State2DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody5ThetaDot2 = spinningBody5State2DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody6ThetaDot2 = spinningBody6State2DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody7ThetaDot2 = spinningBody7State2DataLog.thetaDot * macros.R2D  # [deg]
    spinningBody8ThetaDot2 = spinningBody8State2DataLog.thetaDot * macros.R2D  # [deg]

    # Custom color palette from teal to pink
    colors = [
        "#00bfb2",  # bright teal
        "#008b80",  # dark teal
        "#00a2ff",  # vibrant blue
        "#004fcb",  # deep blue
        "#8d7bff",  # periwinkle
        "#a066ff",  # lavender
        "#5a00c1",  # violet
        "#e600a9",  # magenta-pink
    ]

    # Plot prescribed angles
    plt.figure(1)
    plt.clf()
    plt.plot(timespan, prescribedTheta1, color=colors[1], label=r'$\theta_{\text{P}_1}$')
    plt.plot(timespan, prescribedTheta2, color=colors[6], label=r'$\theta_{\text{P}_2}$')
    # plt.title(r'Prescribed Truss Angles', fontsize=16)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend()
    plt.grid(True)


    # Plot spinning body torsional angles
    plt.figure(2)
    plt.clf()
    plt.plot(timespan, spinningBody1Theta1, color=colors[0], label=r'$\theta_{\text{S}_{1,1}}$')
    plt.plot(timespan, spinningBody2Theta1, color=colors[1], label=r'$\theta_{\text{S}_{2,1}}$')
    plt.plot(timespan, spinningBody3Theta1, color=colors[2], label=r'$\theta_{\text{S}_{3,1}}$')
    plt.plot(timespan, spinningBody4Theta1, color=colors[3], label=r'$\theta_{\text{S}_{4,1}}$')
    plt.plot(timespan, spinningBody5Theta1, color=colors[4], label=r'$\theta_{\text{S}_{5,1}}$')
    plt.plot(timespan, spinningBody6Theta1, color=colors[5], label=r'$\theta_{\text{S}_{6,1}}$')
    plt.plot(timespan, spinningBody7Theta1, color=colors[6], label=r'$\theta_{\text{S}_{7,1}}$')
    plt.plot(timespan, spinningBody8Theta1, color=colors[7], label=r'$\theta_{\text{S}_{8,1}}$')
    # plt.title(r'Solar Array Torsional Angles', fontsize=16)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend()
    plt.grid(True)

    # Plot spinning body bending angles
    plt.figure(3)
    plt.clf()
    plt.plot(timespan, spinningBody1Theta2, color=colors[0], label=r'$\theta_{\text{S}_{1,2}}$')
    plt.plot(timespan, spinningBody2Theta2, color=colors[1], label=r'$\theta_{\text{S}_{2,2}}$')
    plt.plot(timespan, spinningBody3Theta2, color=colors[2], label=r'$\theta_{\text{S}_{3,2}}$')
    plt.plot(timespan, spinningBody4Theta2, color=colors[3], label=r'$\theta_{\text{S}_{4,2}}$')
    plt.plot(timespan, spinningBody5Theta2, color=colors[4], label=r'$\theta_{\text{S}_{5,2}}$')
    plt.plot(timespan, spinningBody6Theta2, color=colors[5], label=r'$\theta_{\text{S}_{6,2}}$')
    plt.plot(timespan, spinningBody7Theta2, color=colors[6], label=r'$\theta_{\text{S}_{7,2}}$')
    plt.plot(timespan, spinningBody8Theta2, color=colors[7], label=r'$\theta_{\text{S}_{8,2}}$')
    # plt.title(r'Solar Array Bending Angles', fontsize=16)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend()
    plt.grid(True)

    # Plot spinning body torsional angle rates
    plt.figure(4)
    plt.clf()
    plt.plot(timespan, spinningBody1ThetaDot1, label=r'$\dot{\theta}_{\text{S}_{1,1}}$')
    plt.plot(timespan, spinningBody2ThetaDot1, label=r'$\dot{\theta}_{\text{S}_{2,1}}$')
    plt.plot(timespan, spinningBody3ThetaDot1, label=r'$\dot{\theta}_{\text{S}_{3,1}}$')
    plt.plot(timespan, spinningBody4ThetaDot1, label=r'$\dot{\theta}_{\text{S}_{4,1}}$')
    plt.plot(timespan, spinningBody5ThetaDot1, label=r'$\dot{\theta}_{\text{S}_{5,1}}$')
    plt.plot(timespan, spinningBody6ThetaDot1, label=r'$\dot{\theta}_{\text{S}_{6,1}}$')
    plt.plot(timespan, spinningBody7ThetaDot1, label=r'$\dot{\theta}_{\text{S}_{7,1}}$')
    plt.plot(timespan, spinningBody8ThetaDot1, label=r'$\dot{\theta}_{\text{S}_{8,1}}$')
    plt.title(r'Solar Array Torsional Angle Rates', fontsize=16)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend()
    plt.grid(True)

    # Plot spinning body bending angle rates
    plt.figure(5)
    plt.clf()
    plt.plot(timespan, spinningBody1ThetaDot2, label=r'$\dot{\theta}_{\text{S}_{1,2}}$')
    plt.plot(timespan, spinningBody2ThetaDot2, label=r'$\dot{\theta}_{\text{S}_{2,2}}$')
    plt.plot(timespan, spinningBody3ThetaDot2, label=r'$\dot{\theta}_{\text{S}_{3,2}}$')
    plt.plot(timespan, spinningBody4ThetaDot2, label=r'$\dot{\theta}_{\text{S}_{4,2}}$')
    plt.plot(timespan, spinningBody5ThetaDot2, label=r'$\dot{\theta}_{\text{S}_{5,2}}$')
    plt.plot(timespan, spinningBody6ThetaDot2, label=r'$\dot{\theta}_{\text{S}_{6,2}}$')
    plt.plot(timespan, spinningBody7ThetaDot2, label=r'$\dot{\theta}_{\text{S}_{7,2}}$')
    plt.plot(timespan, spinningBody8ThetaDot2, label=r'$\dot{\theta}_{\text{S}_{8,2}}$')
    plt.title(r'Solar Array Bending Angle Rates', fontsize=16)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend()
    plt.grid(True)

    # Plot hub inertial position
    plt.figure(6)
    plt.clf()
    plt.plot(timespan, r_BN_N[:, 0], label=r'$1$')
    plt.plot(timespan, r_BN_N[:, 1], label=r'$2$')
    plt.plot(timespan, r_BN_N[:, 2], label=r'$3$')
    plt.title(r'Hub Inertial Position ${}^\mathcal{N} r_{B/N}$', fontsize=16)
    plt.ylabel('(m)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend()
    plt.grid(True)

    # Plot hub inertial attitude
    plt.figure(7)
    plt.clf()
    plt.plot(timespan, sigma_BN[:, 0], color=colors[1], label=r'$1$')
    plt.plot(timespan, sigma_BN[:, 1], color=colors[6], label=r'$2$')
    plt.plot(timespan, sigma_BN[:, 2], color=colors[7], label=r'$3$')
    # plt.title(r'Hub Inertial Attitude $\sigma_{\mathcal{B}/\mathcal{N}}$', fontsize=16)
    plt.ylabel('', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend()
    plt.grid(True)

    # Plot hub inertial angular velocity
    plt.figure(8)
    plt.clf()
    plt.plot(timespan, omega_BN_B[:, 0], color=colors[1], label=r'$1$')
    plt.plot(timespan, omega_BN_B[:, 1], color=colors[6], label=r'$2$')
    plt.plot(timespan, omega_BN_B[:, 2], color=colors[7], label=r'$3$')
    # plt.title(r'Hub Inertial Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$', fontsize=16)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend()
    plt.grid(True)

    # # Plot prescribed truss 1 inertial position
    # plt.figure(9)
    # plt.clf()
    # plt.plot(timespan, r_P1cN_N[:, 0], label=r'$1$')
    # plt.plot(timespan, r_P1cN_N[:, 1], label=r'$2$')
    # plt.plot(timespan, r_P1cN_N[:, 2], label=r'$3$')
    # plt.title(r'Prescribed Truss 1 Inertial Position ${}^\mathcal{N} r_{P_c/N}$', fontsize=16)
    # plt.ylabel('(m)', fontsize=14)
    # plt.xlabel('Time (s)', fontsize=14)
    # plt.legend()
    # plt.grid(True)
    #
    # # Plot prescribed truss inertial attitude
    # plt.figure(10)
    # plt.clf()
    # plt.plot(timespan, sigma_P1N[:, 0], label=r'$1$')
    # plt.plot(timespan, sigma_P1N[:, 1], label=r'$2$')
    # plt.plot(timespan, sigma_P1N[:, 2], label=r'$3$')
    # plt.title(r'Prescribed Truss 1 Inertial Attitude $\sigma_{\mathcal{P}/\mathcal{N}}$', fontsize=16)
    # plt.ylabel('', fontsize=14)
    # plt.xlabel('Time (s)', fontsize=14)
    # plt.legend()
    # plt.grid(True)
    #
    # # Plot prescribed truss inertial angular velocity
    # plt.figure(11)
    # plt.clf()
    # plt.plot(timespan, omega_P1N_P1[:, 0], label=r'$1$')
    # plt.plot(timespan, omega_P1N_P1[:, 1], label=r'$2$')
    # plt.plot(timespan, omega_P1N_P1[:, 2], label=r'$3$')
    # plt.title(r'Prescribed Truss 1 Inertial Angular Velocity ${}^\mathcal{P} \omega_{\mathcal{P}/\mathcal{N}}$', fontsize=16)
    # plt.ylabel('(deg/s)', fontsize=14)
    # plt.xlabel('Time (s)', fontsize=14)
    # plt.legend()
    # plt.grid(True)
    #
    # plt.figure(12)
    # plt.clf()
    # plt.plot(timespan, (orbAngMom_N[:, 1] - orbAngMom_N[0, 1]) / orbAngMom_N[0, 1],
    #          timespan, (orbAngMom_N[:, 2] - orbAngMom_N[0, 2]) / orbAngMom_N[0, 2],
    #          timespan, (orbAngMom_N[:, 3] - orbAngMom_N[0, 3]) / orbAngMom_N[0, 3])
    # plt.xlabel('Time (s)')
    # plt.ylabel('Relative Difference')
    # plt.title('Orbital Angular Momentum')
    #
    # plt.figure(13)
    # plt.clf()
    # plt.plot(timespan, (orbEnergy[:, 1] - orbEnergy[0, 1]) / orbEnergy[0, 1])
    # plt.xlabel('Time (s)')
    # plt.ylabel('Relative Difference')
    # plt.title('Orbital Energy')
    #
    # plt.figure(14)
    # plt.clf()
    # plt.plot(timespan, (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]) / rotAngMom_N[0, 1],
    #          timespan, (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]) / rotAngMom_N[0, 2],
    #          timespan, (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]) / rotAngMom_N[0, 3])
    # plt.xlabel('Time (s)')
    # plt.ylabel('Relative Difference')
    # plt.title('Rotational Angular Momentum')
    #
    # plt.figure(15)
    # plt.clf()
    # plt.plot(rotEnergy[:, 0] * 1e-9, (rotEnergy[:, 1] - rotEnergy[0, 1]) / rotEnergy[0, 1])
    # plt.xlabel('Time (s)')
    # plt.ylabel('Relative Difference')
    # plt.title('Rotational Energy')

    if show_plots:
        plt.show()
    plt.close("all")


if __name__ == "__main__":
    run(True)
