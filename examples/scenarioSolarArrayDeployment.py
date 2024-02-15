#
# ISC License
#
# Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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
from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft
from Basilisk.simulation import prescribedMotionStateEffector
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros, RigidBodyKinematics as rbk
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import vizSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

def run(show_plots):

    simProcessName = "simProcess"
    dynTaskName = "dynTask"
    fswTaskName = "fswTask"

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    dynTimeStep = 0.5  # [s]
    fswTimeStep = 0.5  # [s]
    dataRecStep = 1.0  # [s]
    dynProcessRate = macros.sec2nano(dynTimeStep)  # [ns]
    fswProcessRate = macros.sec2nano(fswTimeStep)  # [ns]
    dataRecRate = macros.sec2nano(dataRecStep)  # [ns]
    simProc = scSim.CreateNewProcess(simProcessName)
    simProc.addTask(scSim.CreateNewTask(dynTaskName, dynProcessRate))
    simProc.addTask(scSim.CreateNewTask(fswTaskName, fswProcessRate))

    # Add the spacecraft module
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scSim.AddModelToTask(dynTaskName, scObject)

    # Define the mass properties of the rigid spacecraft hub
    scObject.hub.mHub = 2159.104862  # kg
    scObject.hub.r_BcB_B = [0.0, 0.0, 1.181]  # [m]
    scObject.hub.IHubPntBc_B = [[1073.7746365392636, -19.901891598854863, 22.174384554367514],
                                [-19.901891598854863, 2128.835652900606, 5.276914422384181],
                                [22.174384554367514, 5.276914422384181, 1989.2486219862258]]

    # Set the initial inertial hub states
    scObject.hub.r_CN_NInit = [0.0, 0.0, 0.0]
    scObject.hub.v_CN_NInit = [0.0, 0.0, 0.0]
    scObject.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]

    # Define the state effector kinematic properties
    # B frame is the hub body frame
    # S1 and S2 frames are the solar array body frames
    # M1 and M2 frames are the mount frames
    
    # DCMs representing the attitude of the hub frame with respect to the solar array frames
    dcm_BS1 = np.array([[0,  1,  0],
                        [0,  0, -1],
                        [-1,  0,  0]])
    dcm_BS2 = np.array([[0, -1,  0],
                        [0,  0, -1],
                        [1,  0,  0]])

    # Position vector of solar array frame origin points with respect to hub frame origin point B 
    # expressed in B frame components
    rArray1SB_B = np.array([0.5 * 1.53, 0.0, 0.44])   # [m]
    rArray2SB_B = np.array([-0.5 * 1.53, 0.0, 0.44])   # [m]

    # Position vector of mount frame origin points with respect to solar array frame origin points 
    # expressed in solar array frame components
    r_M1S1_S1 = [0.0, 0.0, 0.0]  # [m]
    r_M2S2_S2 = [0.0, 0.0, 0.0]  # [m]
    
    # Position vector of mount frame with respect to solar array frame in hub frame components
    r_M1S1_B = dcm_BS1 @ r_M1S1_S1  # [m]
    r_M2S2_B = dcm_BS2 @ r_M2S2_S2  # [m]

    # Position vector of mount frame origin points with respect to hub frame origin point B
    r_M1B_B = r_M1S1_B + rArray1SB_B  # [m]
    r_M2B_B = r_M2S2_B + rArray2SB_B  # [m]

    # Array element geometric parameters
    num_elements = 10
    mass_element = 82.79/10  # [kg]
    rot_hat_M = np.array([0.0, 1.0, 0.0])  # Array articulation axis in mount frame components
    radius_array = 0.5 * 7.262  # [m]
    length_element = radius_array  # [m]
    width_element = 2 * radius_array * np.cos(72 * macros.D2R)  # [m]
    thickness_element = 0.01  # [m]
    I_element_11 = (1/12) * mass_element * (length_element * length_element
                                            + thickness_element * thickness_element)  # [kg m^2]
    I_element_22 = (1/12) * mass_element * (length_element * length_element
                                            + width_element * width_element)  # [kg m^2]
    I_element_33 = (1/12) * mass_element * (width_element * width_element
                                            + thickness_element * thickness_element)  # [kg m^2]
    IElement_PntFc_F = [[I_element_11, 0.0, 0.0],
                        [0.0, I_element_22, 0.0],
                        [0.0, 0.0, I_element_33]]  # [kg m^2] (approximate as rectangular prisms)
    
    # Deployment temporal information
    ramp_duration = 1.0  # [s]
    init_deploy_duration = 5.0 * 60.0  # [s]
    main_deploy_duration = 20.0 * 60.0  # [s]
    init_coast_duration = init_deploy_duration - 2 * ramp_duration
    main_coast_duration = main_deploy_duration - 2 * ramp_duration

    # Rotation 1 initial parameters
    array1ThetaInit1 = 0.0 * macros.D2R  # [rad]
    array2ThetaInit1 = 0.0 * macros.D2R  # [rad]
    prv_FM1Init1 = array1ThetaInit1 * rot_hat_M
    prv_FM2Init1 = array2ThetaInit1 * rot_hat_M
    sigma_FM1Init1 = rbk.PRV2MRP(prv_FM1Init1)
    sigma_FM2Init1 = rbk.PRV2MRP(prv_FM2Init1)
    r_FM1_M1Init1 = [0.0, 0.0, 0.0]  # [m]
    r_FM2_M2Init1 = [0.0, 0.0, 0.0]  # [m]

    # Rotation 2 initial parameters
    array1ThetaInit2 = 108.0 * macros.D2R  # [rad]
    array2ThetaInit2 = -108.0 * macros.D2R  # [rad]
    prv_FM1Init2 = array1ThetaInit2 * rot_hat_M
    prv_FM2Init2 = array2ThetaInit2 * rot_hat_M
    sigma_FM1Init2 = rbk.PRV2MRP(prv_FM1Init2)
    sigma_FM2Init2 = rbk.PRV2MRP(prv_FM2Init2)
    r_FM1_M1Init2 = [radius_array, 0.0, 0.0]  # [m]
    r_FM2_M2Init2 = [-radius_array, 0.0, 0.0]  # [m]

    # Create the solar array elements
    array1ElementList = list()
    array2ElementList = list()
    for i in range(num_elements):
        array1ElementList.append(prescribedMotionStateEffector.PrescribedMotionStateEffector())
        array2ElementList.append(prescribedMotionStateEffector.PrescribedMotionStateEffector())
        array1ElementList[i].ModelTag = "array1Element" + str(i+1)
        array2ElementList[i].ModelTag = "array2Element" + str(i+1)
        array1ElementList[i].mass = mass_element  # [kg]
        array2ElementList[i].mass = mass_element  # [kg]
        array1ElementList[i].IPntFc_F = IElement_PntFc_F  # [kg m^2]
        array2ElementList[i].IPntFc_F = IElement_PntFc_F  # [kg m^2]
        array1ElementList[i].r_MB_B = r_M1B_B  # [m]
        array2ElementList[i].r_MB_B = r_M2B_B  # [m]
        array1ElementList[i].r_FcF_F = [- radius_array * np.cos(72 * macros.D2R),
                                        0.0,
                                        (1/3) * radius_array * np.sin(72 * macros.D2R)]  # [m] For triangular wedge
        array2ElementList[i].r_FcF_F = [radius_array * np.cos(72 * macros.D2R),
                                        0.0,
                                        (1/3) * radius_array * np.sin(72 * macros.D2R)]  # [m]
        array1ElementList[i].r_FM_M = r_FM1_M1Init1  # [m]
        array2ElementList[i].r_FM_M = r_FM2_M2Init1  # [m]
        array1ElementList[i].rPrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s]
        array2ElementList[i].rPrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s]
        array1ElementList[i].rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s^2]
        array2ElementList[i].rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s^2]
        array1ElementList[i].omega_FM_F = np.array([0.0, 0.0, 0.0])  # [rad/s]
        array2ElementList[i].omega_FM_F = np.array([0.0, 0.0, 0.0])  # [rad/s]
        array1ElementList[i].omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])  # [rad/s^2]
        array2ElementList[i].omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])  # [rad/s^2]
        array1ElementList[i].sigma_FM = sigma_FM1Init1
        array2ElementList[i].sigma_FM = sigma_FM2Init1
        array1ElementList[i].omega_MB_B = [0.0, 0.0, 0.0]  # [rad/s]
        array2ElementList[i].omega_MB_B = [0.0, 0.0, 0.0]  # [rad/s]
        array1ElementList[i].omegaPrime_MB_B = [0.0, 0.0, 0.0]  # [rad/s^2]
        array2ElementList[i].omegaPrime_MB_B = [0.0, 0.0, 0.0]  # [rad/s^2]
        array1ElementList[i].sigma_MB = [0.0, 0.0, 0.0]
        array2ElementList[i].sigma_MB = [0.0, 0.0, 0.0]

        scObject.addStateEffector(array1ElementList[i])
        scObject.addStateEffector(array2ElementList[i])
        scSim.AddModelToTask(dynTaskName, array1ElementList[i])
        scSim.AddModelToTask(dynTaskName, array2ElementList[i])

    # Create the array element reference angle messages
    array1ElementRefMsgList1 = list()
    array2ElementRefMsgList1 = list()
    for i in range(num_elements):
        array1ElementMessageData = messaging.HingedRigidBodyMsgPayload()
        array2ElementMessageData = messaging.HingedRigidBodyMsgPayload()
        array1ElementMessageData.theta = array1ThetaInit2  # [rad]
        array2ElementMessageData.theta = array2ThetaInit1  # [rad]
        array1ElementMessageData.thetaDot = 0.0  # [rad/s]
        array2ElementMessageData.thetaDot = 0.0  # [rad/s]
        array1ElementRefMsgList1.append(messaging.HingedRigidBodyMsg().write(array1ElementMessageData))
        array2ElementRefMsgList1.append(messaging.HingedRigidBodyMsg().write(array2ElementMessageData))

    # Create stand-alone element translational state messages
    array1ElementTranslationMessageData = messaging.PrescribedTranslationMsgPayload()
    array2ElementTranslationMessageData = messaging.PrescribedTranslationMsgPayload()
    array1ElementTranslationMessageData.r_FM_M = r_FM1_M1Init1  # [m]
    array2ElementTranslationMessageData.r_FM_M = r_FM2_M2Init1  # [m]
    array1ElementTranslationMessageData.rPrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s]
    array2ElementTranslationMessageData.rPrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s]
    array1ElementTranslationMessageData.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s^2]
    array2ElementTranslationMessageData.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s^2]
    array1ElementTranslationMessage = messaging.PrescribedTranslationMsg().write(array1ElementTranslationMessageData)
    array2ElementTranslationMessage = messaging.PrescribedTranslationMsg().write(array2ElementTranslationMessageData)

    # Initialize the prescribedRotation1DOF module configuration data
    array1MaxRotAccelList1 = []
    array2MaxRotAccelList1 = []
    for j in range(2):
        for i in range(num_elements):
            if j == 0:
                thetaInit = array1ThetaInit1  # [rad]
                thetaRef = array1ThetaInit2  # [rad]
                thetaDDotMax = np.abs(thetaRef - thetaInit) / ((init_coast_duration * ramp_duration)
                                                               + (ramp_duration * ramp_duration))
                array1MaxRotAccelList1.append(thetaDDotMax)
            else:
                thetaInit = array2ThetaInit1  # [rad]
                thetaRef = array2ThetaInit1  # [rad]
                thetaDDotMax = np.abs(thetaRef - thetaInit) / ((init_coast_duration * ramp_duration)
                                                               + (ramp_duration * ramp_duration))
                array2MaxRotAccelList1.append(thetaDDotMax)

    array1RotProfilerList = list()
    array2RotProfilerList = list()
    for i in range(num_elements):
        array1RotProfilerList.append(prescribedRotation1DOF.PrescribedRotation1DOF())
        array2RotProfilerList.append(prescribedRotation1DOF.PrescribedRotation1DOF())
        array1RotProfilerList[i].ModelTag = "prescribedRotation1DOFArray1Element" + str(i+1)
        array2RotProfilerList[i].ModelTag = "prescribedRotation1DOFArray2Element" + str(i+1)
        array1RotProfilerList[i].setCoastOptionRampDuration(ramp_duration)  # [s]
        array2RotProfilerList[i].setCoastOptionRampDuration(ramp_duration)  # [s]
        array1RotProfilerList[i].setRotHat_M(rot_hat_M)
        array2RotProfilerList[i].setRotHat_M(rot_hat_M)
        array1RotProfilerList[i].setThetaDDotMax(array1MaxRotAccelList1[i])  # [rad/s^2]
        array2RotProfilerList[i].setThetaDDotMax(array2MaxRotAccelList1[i])  # [rad/s^2]
        array1RotProfilerList[i].setThetaInit(array1ThetaInit1)  # [rad]
        array2RotProfilerList[i].setThetaInit(array2ThetaInit1)  # [rad]
    
        scSim.AddModelToTask(fswTaskName, array1RotProfilerList[i])
        scSim.AddModelToTask(fswTaskName, array2RotProfilerList[i])
        array1RotProfilerList[i].spinningBodyInMsg.subscribeTo(array1ElementRefMsgList1[i])
        array2RotProfilerList[i].spinningBodyInMsg.subscribeTo(array2ElementRefMsgList1[i])
        array1ElementList[i].prescribedRotationInMsg.subscribeTo(array1RotProfilerList[i].prescribedRotationOutMsg)
        array2ElementList[i].prescribedRotationInMsg.subscribeTo(array2RotProfilerList[i].prescribedRotationOutMsg)
        array1ElementList[i].prescribedTranslationInMsg.subscribeTo(array1ElementTranslationMessage)
        array2ElementList[i].prescribedTranslationInMsg.subscribeTo(array2ElementTranslationMessage)

    # Set up data logging
    energyMomentumLog = scObject.logger(["totRotAngMomPntC_N", "totRotEnergy"], dataRecRate)
    scStateData = scObject.scStateOutMsg.recorder(dataRecRate)
    array1DataLogs = list(map(lambda profiler: profiler.spinningBodyOutMsg.recorder(dataRecRate),
                              array1RotProfilerList))
    array2DataLogs = list(map(lambda profiler: profiler.spinningBodyOutMsg.recorder(dataRecRate),
                              array2RotProfilerList))
    scSim.AddModelToTask(fswTaskName, energyMomentumLog)
    scSim.AddModelToTask(fswTaskName, scStateData)
    [scSim.AddModelToTask(fswTaskName, logger) for logger in array1DataLogs + array2DataLogs]

    # Set up Vizard visualization
    if vizSupport.vizFound:
        scBodyList = [scObject]
        for i in range(num_elements):
            scBodyList.append(["Array1Element" + str(i+1), array1ElementList[i].prescribedMotionConfigLogOutMsg])
            scBodyList.append(["Array2Element" + str(i+1), array2ElementList[i].prescribedMotionConfigLogOutMsg])

        viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scBodyList, saveFile=filename)
        viz.settings.showSpacecraftAsSprites = -1
        vizSupport.createCustomModel(viz,
                                     simBodiesToModify=[scObject.ModelTag],
                                     modelPath="/Volumes/KINGSTON/ema-gnc/sim/visualization/GNC Visualization Model Spacecraft.obj",
                                     rotation=[0, 0, -np.pi / 2],
                                     offset=[0, 0, 0]
                                     )

        for i in range(num_elements):
            vizSupport.createCustomModel(viz,
                                         simBodiesToModify=["Array1Element" + str(i+1)],
                                         modelPath=path + "/dataForExamples/triangularPanel.obj",
                                         rotation=[-np.pi/2, 0, np.pi/2],
                                         scale=[1.3, 1.3, 1.3],
                                         color=vizSupport.toRGBA255("green"))
            vizSupport.createCustomModel(viz,
                                         simBodiesToModify=["Array2Element" + str(i+1)],
                                         modelPath=path + "/dataForExamples/triangularPanel.obj",
                                         rotation=[-np.pi/2, 0, np.pi/2],
                                         scale=[1.3, 1.3, 1.3],
                                         color=vizSupport.toRGBA255("blue"))

    scSim.InitializeSimulation()
    simTime1 = init_deploy_duration + 10  # [s]
    scSim.ConfigureStopTime(macros.sec2nano(simTime1))
    scSim.ExecuteSimulation()

    # Update the configuration data for the array 1 modules
    array1MaxRotAccelList2 = []
    for i in range(num_elements):
        thetaInit = array1ThetaInit2  # [rad]
        thetaRef = (36 * i * macros.D2R) + array1ThetaInit2  # [rad]
        thetaDDotMax = np.abs(thetaRef - thetaInit) / ((main_coast_duration * ramp_duration)
                                                       + (ramp_duration * ramp_duration))  # [rad/s^2]
        array1MaxRotAccelList2.append(thetaDDotMax)

    # Update the array 1 stand-alone element translational state messages
    array1ElementTranslationMessageData = messaging.PrescribedTranslationMsgPayload()
    array1ElementTranslationMessageData.r_FM_M = r_FM1_M1Init2  # [m]
    array1ElementTranslationMessageData.rPrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s]
    array1ElementTranslationMessageData.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s^2]
    array1ElementTranslationMessage = messaging.PrescribedTranslationMsg().write(array1ElementTranslationMessageData)

    array1ElementRefMsgList2 = list()
    for i in range(num_elements):
        array1ElementList[i].prescribedTranslationInMsg.subscribeTo(array1ElementTranslationMessage)
        array1ElementList[i].r_FcF_F = [0.0, 0.0, - (2/3) * radius_array * np.sin(72 * macros.D2R)]
        array1ElementList[i].r_FM_M = r_FM1_M1Init2  # [m]
        array1ElementList[i].sigma_FM = sigma_FM1Init2

        array1RotProfilerList[i].setThetaInit(array1ThetaInit2)  # [rad]
        array1RotProfilerList[i].setThetaDDotMax(array1MaxRotAccelList2[i])  # [rad/s^2]

        array1ElementMessageData = messaging.HingedRigidBodyMsgPayload()
        array1ElementMessageData.theta = (36 * i * macros.D2R) + array1ThetaInit2  # [rad]
        array1ElementMessageData.thetaDot = 0.0  # [rad/s]
        array1ElementRefMsgList2.append(messaging.HingedRigidBodyMsg().write(array1ElementMessageData))

        array1RotProfilerList[i].spinningBodyInMsg.subscribeTo(array1ElementRefMsgList2[i])

    simTime2 = main_deploy_duration + 10  # [s]
    scSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2))
    scSim.ExecuteSimulation()

    # Update the configuration data for the array 2 modules
    array2MaxRotAccelList2 = []
    for i in range(num_elements):
        thetaInit = array2ThetaInit1  # [rad]
        thetaRef = array2ThetaInit2  # [rad]
        thetaDDotMax = np.abs(thetaRef - thetaInit) / ((init_coast_duration * ramp_duration)
                                                       + (ramp_duration * ramp_duration))  # [rad/s^2]
        array2MaxRotAccelList2.append(thetaDDotMax)

    array2ElementRefMsgList2 = list()
    for i in range(num_elements):
        array2RotProfilerList[i].setThetaDDotMax(array2MaxRotAccelList2[i])  # [rad/s^2]

        array2ElementMessageData = messaging.HingedRigidBodyMsgPayload()
        array2ElementMessageData.theta = array2ThetaInit2  # [rad]
        array2ElementMessageData.thetaDot = 0.0  # [rad/s]
        array2ElementRefMsgList2.append(messaging.HingedRigidBodyMsg().write(array2ElementMessageData))

        array2RotProfilerList[i].spinningBodyInMsg.subscribeTo(array2ElementRefMsgList2[i])
        
    simTime3 = init_deploy_duration + 10  # [s]
    scSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2 + simTime3))
    scSim.ExecuteSimulation()

    # Update the configuration data for the array 2 modules
    array2MaxRotAccelList3 = []
    for i in range(num_elements):
        thetaInit = array2ThetaInit2  # [rad]
        thetaRef = (36 * i * macros.D2R) + array2ThetaInit2  # [rad]
        thetaDDotMax = np.abs(thetaRef - thetaInit) / ((main_coast_duration * ramp_duration)
                                                       + (ramp_duration * ramp_duration))  # [rad/s^2]
        array2MaxRotAccelList3.append(thetaDDotMax)

    # Update the array 2 stand-alone element translational state messages
    array2ElementTranslationMessageData = messaging.PrescribedTranslationMsgPayload()
    array2ElementTranslationMessageData.r_FM_M = r_FM2_M2Init2  # [m]
    array2ElementTranslationMessageData.rPrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s]
    array2ElementTranslationMessageData.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])  # [m/s^2]
    array2ElementTranslationMessage = messaging.PrescribedTranslationMsg().write(array2ElementTranslationMessageData)

    array2ElementRefMsgList3 = list()
    for i in range(num_elements):
        array2ElementList[i].prescribedTranslationInMsg.subscribeTo(array2ElementTranslationMessage)
        array2ElementList[i].r_FcF_F = [0.0, 0.0, - (2/3) * radius_array * np.sin(72 * macros.D2R)]
        array2ElementList[i].r_FM_M = r_FM2_M2Init2  # [m]
        array2ElementList[i].sigma_FM = sigma_FM2Init2

        array2RotProfilerList[i].setThetaInit(array2ThetaInit2)  # [rad]
        array2RotProfilerList[i].setThetaDDotMax(array2MaxRotAccelList3[i])  # [rad/s^2]

        array2ElementMessageData = messaging.HingedRigidBodyMsgPayload()
        array2ElementMessageData.theta = (36 * i * macros.D2R) + array2ThetaInit2  # [rad]
        array2ElementMessageData.thetaDot = 0.0  # [rad/s]
        array2ElementRefMsgList3.append(messaging.HingedRigidBodyMsg().write(array2ElementMessageData))

        array2RotProfilerList[i].spinningBodyInMsg.subscribeTo(array2ElementRefMsgList3[i])

    simTime4 = main_deploy_duration + 10  # [s]
    scSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2 + simTime3 + simTime4))
    scSim.ExecuteSimulation()

    # Extract the logged data
    timespan = scStateData.times() * macros.NANO2MIN  # [min]
    rotAngMom_N = unitTestSupport.addTimeColumn(energyMomentumLog.times(), energyMomentumLog.totRotAngMomPntC_N)
    # [Nms]
    rotEnergy = unitTestSupport.addTimeColumn(energyMomentumLog.times(), energyMomentumLog.totRotEnergy)  # [J]
    r_BN_N = scStateData.r_BN_N  # [m]
    omega_BN_B = scStateData.omega_BN_B * macros.R2D  # [deg/s]
    sigma_BN = scStateData.sigma_BN
    theta_array1 = list(map(lambda data: data.theta * macros.R2D, array1DataLogs))  # [deg]
    theta_array2 = list(map(lambda data: data.theta * macros.R2D, array2DataLogs))  # [deg]
    thetaDot_array1 = list(map(lambda data: data.thetaDot * macros.R2D, array1DataLogs))  # [deg/s]
    thetaDot_array2 = list(map(lambda data: data.thetaDot * macros.R2D, array2DataLogs))  # [deg/s]

    plt.close("all")

    # Plot array 1 element angles
    plt.figure()
    plt.clf()
    [plt.plot(timespan,
              array_element,
              label=r'$\theta_{' + str(i+1) + '}$') for i, array_element in enumerate(theta_array1)]
    plt.title(r'Array 1 Element Angles', fontsize=16)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='upper left', prop={'size': 12})
    plt.grid(True)

    # Plot array 2 element angles
    plt.figure()
    plt.clf()
    [plt.plot(timespan,
              array_element,
              label=r'$\theta_{' + str(i+1) + '}$') for i, array_element in enumerate(theta_array2)]
    plt.title(r'Array 2 Element Angles', fontsize=16)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center left', prop={'size': 12})
    plt.grid(True)

    # Plot array 1 element angle rates
    plt.figure()
    plt.clf()
    [plt.plot(timespan,
              array_element,
              label=r'$\dot{\theta}_{' + str(i+1) + '}$') for i, array_element in enumerate(thetaDot_array1)]
    plt.title(r'Array 1 Element Angle Rates', fontsize=16)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    # Plot array 2 element angle rates
    plt.figure()
    plt.clf()
    [plt.plot(timespan,
              array_element,
              label=r'$\dot{\theta}_{' + str(i+1) + '}$') for i, array_element in enumerate(thetaDot_array2)]
    plt.title(r'Array 2 Element Angle Rates', fontsize=16)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center left', prop={'size': 12})
    plt.grid(True)

    # Plot r_BN_N
    plt.figure()
    plt.clf()
    plt.plot(timespan, r_BN_N[:, 0], label=r'$r_{1}$')
    plt.plot(timespan, r_BN_N[:, 1], label=r'$r_{2}$')
    plt.plot(timespan, r_BN_N[:, 2], label=r'$r_{3}$')
    plt.title(r'${}^\mathcal{N} r_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial Trajectory', fontsize=16)
    plt.ylabel('(m)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='upper left', prop={'size': 12})
    plt.grid(True)

    # Plot sigma_BN
    plt.figure()
    plt.clf()
    plt.plot(timespan, sigma_BN[:, 0], label=r'$\sigma_{1}$')
    plt.plot(timespan, sigma_BN[:, 1], label=r'$\sigma_{2}$')
    plt.plot(timespan, sigma_BN[:, 2], label=r'$\sigma_{3}$')
    plt.title(r'$\sigma_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial MRP Attitude', fontsize=16)
    plt.ylabel('', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='lower left', prop={'size': 12})
    plt.grid(True)

    # Plot omega_BN_B
    plt.figure()
    plt.clf()
    plt.plot(timespan, omega_BN_B[:, 0], label=r'$\omega_{1}$')
    plt.plot(timespan, omega_BN_B[:, 1], label=r'$\omega_{2}$')
    plt.plot(timespan, omega_BN_B[:, 2], label=r'$\omega_{3}$')
    plt.title(r'Spacecraft Hub Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$', fontsize=16)
    plt.xlabel('Time (min)', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.legend(loc='lower right', prop={'size': 12})
    plt.grid(True)

    # Plot omega_BN_B norm
    omega_BN_BNorm = np.linalg.norm(omega_BN_B, axis=1)
    plt.figure()
    plt.clf()
    plt.plot(timespan, omega_BN_BNorm)
    plt.title('Hub Angular Velocity Norm', fontsize=16)
    plt.ylabel(r'(deg/s)', fontsize=14)
    plt.xlabel(r'(min)', fontsize=14)
    plt.grid(True)

    # Plotting: Conservation quantities
    plt.figure()
    plt.clf()
    plt.plot(timespan, rotAngMom_N[:, 1] - rotAngMom_N[0, 1],
             timespan, rotAngMom_N[:, 2] - rotAngMom_N[0, 2],
             timespan, rotAngMom_N[:, 3] - rotAngMom_N[0, 3])
    plt.title('Rotational Angular Momentum Difference', fontsize=16)
    plt.ylabel('(Nms)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.grid(True)

    plt.figure()
    plt.clf()
    plt.plot(timespan, rotEnergy[:, 1] - rotEnergy[0, 1])
    plt.title('Total Energy Difference', fontsize=16)
    plt.ylabel('Energy (J)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

if __name__ == "__main__":
    run(
        True,   # show_plots
    )
    