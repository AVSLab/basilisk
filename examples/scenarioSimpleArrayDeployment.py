import inspect
import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft, spinningBodyOneDOFStateEffector, prescribedLinearTranslation
from Basilisk.simulation import prescribedMotionStateEffector
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros, RigidBodyKinematics as rbk
from Basilisk.utilities import vizSupport

def run(show_plots):

    simProcessName = "simProcess"
    dynTaskName = "dynTask"
    fswTaskName = "fswTask"

    scSim = SimulationBaseClass.SimBaseClass()

    dynTimeStep = 0.05
    fswTimeStep = 0.05
    dataRecStep = 2.0
    dynProcessRate = macros.sec2nano(dynTimeStep)
    fswProcessRate = macros.sec2nano(fswTimeStep)
    dataRecRate = macros.sec2nano(dataRecStep)
    simProc = scSim.CreateNewProcess(simProcessName)
    simProc.addTask(scSim.CreateNewTask(dynTaskName, dynProcessRate))
    simProc.addTask(scSim.CreateNewTask(fswTaskName, fswProcessRate))

    # Add Spacecraft Module
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scSim.AddModelToTask(dynTaskName, scObject)

    # Define mass properties of hub
    massHub = 800 # kg
    lengthHub = 3.0 # m
    widthHub = 2.0 # m
    depthHub = 2.0 # m
    IHub_11 = (1/12) * massHub * (lengthHub * lengthHub + depthHub * depthHub) # kg m^2
    IHub_22 = (1/12) * massHub * (depthHub * depthHub + widthHub * widthHub) # kg m^2
    IHub_33 = (1/12) * massHub * (lengthHub * lengthHub + widthHub * widthHub) # kg m^2
    scObject.hub.mHub = massHub
    scObject.hub.IHubPntBc_B = [[IHub_11, 0.0, 0.0],
                                [0.0, IHub_22, 0.0],
                                [0.0, 0.0, IHub_33]]

    # Hub Initial States
    scObject.hub.r_CN_NInit = [0.0, 0.0, 0.0]
    scObject.hub.v_CN_NInit = [0.0, 0.0, 0.0]
    scObject.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]

    # Position of mount frame with respect to solar array frame
    r_M1S1_B = [0.0, 0.0, 0.0]
    r_M2S2_B = [0.0, 0.0, 0.0]

    # Position Vector of solar array frame with respect to hub body frame
    r_array1SB_B = np.array([widthHub/2, 0.0, 0.0])
    r_array2SB_B = np.array([-widthHub/2, 0.0, 0.0])

    # Position Vector of mount frame with respect to body frame origin
    r_M1B_B = r_M1S1_B + r_array1SB_B
    r_M2B_B = r_M2S2_B + r_array2SB_B

    # Create Solar Array Components
    numElements = 4
    massElement = 5.0 # [kg]
    rot_hat_M = np.array([0.0, -1.0, 0.0])
    lengthElement = 1.5
    widthElement = 0.75
    thicknessElement = 0.01
    I_element_11 = (1/12) * massElement * (widthElement**2 + thicknessElement**2)
    I_element_22 = (1/12) * massElement * (lengthElement**2 + thicknessElement**2)
    I_element_33 = (1/12) * massElement * (lengthElement**2 + widthElement**2)
    IElement_PntPc_P = [[I_element_11, 0.0, 0.0],
                        [0.0, I_element_22, 0.0],
                        [0.0, 0.0, I_element_33]]

    # Deployment Information
    rot_duration = 30.0
    translation_duration = 10.0 # for each panel

    # Rotation Parameters
    array1ThetaInit1 = 0.0 * macros.D2R
    array2ThetaInit1 = 0.0 * macros.D2R
    thetaDDotMax = 2.0 * macros.D2R
    array1ThetaRef = 90 * macros.D2R
    array2ThetaRef = -90 * macros.D2R
    r_PM1_M1Init1 = [0.0, 0.0, 0.0]  # [m]
    r_PM2_M2Init1 = [0.0, 0.0, 0.0]  # [m]
    prv_PM1Init1 = array1ThetaInit1 * rot_hat_M
    prv_PM2Init1 = array2ThetaInit1 * rot_hat_M
    sigma_PM1Init1 = rbk.PRV2MRP(prv_PM1Init1) # QUESTIONS
    sigma_PM2Init1 = rbk.PRV2MRP(prv_PM2Init1)

    # Create Array elements
    array1ElementList = list()
    array2ElementList = list()
    for i in range(numElements):
        array1ElementList.append(prescribedMotionStateEffector.PrescribedMotionStateEffector())
        array2ElementList.append(prescribedMotionStateEffector.PrescribedMotionStateEffector())
        array1ElementList[i].ModelTag = "array1Element" + str(i + 1)
        array2ElementList[i].ModelTag = "array2Element" + str(i + 1)
        array1ElementList[i].setMass(massElement)
        array2ElementList[i].setMass(massElement)
        array1ElementList[i].setIPntPc_P(IElement_PntPc_P)
        array2ElementList[i].setIPntPc_P(IElement_PntPc_P)
        array1ElementList[i].setR_MB_B(r_M1B_B)
        array2ElementList[i].setR_MB_B(r_M2B_B)
        array1ElementList[i].setR_PcP_P([lengthElement/2.0, 0.0, 0.0])
        array2ElementList[i].setR_PcP_P([-lengthElement/2.0, 0.0, 0.0])
        array1ElementList[i].setR_PM_M(r_PM1_M1Init1)
        array2ElementList[i].setR_PM_M(r_PM2_M2Init1)
        array1ElementList[i].setRPrime_PM_M(np.array([0.0, 0.0, 0.0]))
        array2ElementList[i].setRPrime_PM_M(np.array([0.0, 0.0, 0.0]))
        array1ElementList[i].setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))  # [m/s^2]
        array2ElementList[i].setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))  # [m/s^2]
        array1ElementList[i].setOmega_PM_P(np.array([0.0, 0.0, 0.0]))
        array2ElementList[i].setOmega_PM_P(np.array([0.0, 0.0, 0.0]))
        array1ElementList[i].setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))  # [rad/s^2]
        array2ElementList[i].setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))  # [rad/s^2]
        array1ElementList[i].setSigma_PM(sigma_PM1Init1)
        array2ElementList[i].setSigma_PM(sigma_PM2Init1)
        array1ElementList[i].setSigma_MB([0.0, 0.4, 0.0]) # QUESTIONS
        array2ElementList[i].setSigma_MB([0.0, -0.4, 0.0])

        scObject.addStateEffector(array1ElementList[i])
        scObject.addStateEffector(array2ElementList[i])
        scSim.AddModelToTask(dynTaskName, array1ElementList[i])
        scSim.AddModelToTask(dynTaskName, array2ElementList[i])

    # create elements for the reference angles
    array1ElementRefMsgList = list()
    array2ElementRefMsgList = list()
    for i in range(numElements):
        array1ElementMessageData = messaging.HingedRigidBodyMsgPayload()
        array2ElementMessageData = messaging.HingedRigidBodyMsgPayload()
        array1ElementMessageData.theta = array1ThetaRef
        array2ElementMessageData.theta = array2ThetaRef
        array1ElementMessageData.thetaDot = 0.0
        array2ElementMessageData.thetaDot = 0.0
        array1ElementRefMsgList.append(messaging.HingedRigidBodyMsg().write(array1ElementMessageData))
        array2ElementRefMsgList.append(messaging.HingedRigidBodyMsg().write(array2ElementMessageData))

    # Create translational message data
    array1ElementTranslationMessageData = messaging.PrescribedTranslationMsgPayload()
    array2ElementTranslationMessageData = messaging.PrescribedTranslationMsgPayload()
    array1ElementTranslationMessageData.r_PM_M = r_PM1_M1Init1  # [m]
    array2ElementTranslationMessageData.r_PM_M = r_PM2_M2Init1  # [m]
    array1ElementTranslationMessageData.rPrime_PM_M = np.array([0.0, 0.0, 0.0])  # [m/s]
    array2ElementTranslationMessageData.rPrime_PM_M = np.array([0.0, 0.0, 0.0])  # [m/s]
    array1ElementTranslationMessageData.rPrimePrime_PM_M = np.array([0.0, 0.0, 0.0])  # [m/s^2]
    array2ElementTranslationMessageData.rPrimePrime_PM_M = np.array([0.0, 0.0, 0.0])  # [m/s^2]
    array1ElementTranslationMessage = messaging.PrescribedTranslationMsg().write(array1ElementTranslationMessageData)
    array2ElementTranslationMessage = messaging.PrescribedTranslationMsg().write(array2ElementTranslationMessageData)

    # Initialize the prescribed rotation 1DOF module
    array1MaxRotAccelList1 = []
    array2MaxRotAccelList2 = []
    for j in range(2):
        for i in range(numElements):
            if j == 0:
                thetaInit = array1ThetaInit1
                thetaDDotMax = 4.0 * np.abs(array1ThetaRef - thetaInit) / (rot_duration ** 2)
                array1MaxRotAccelList1.append(thetaDDotMax)
            else:
                thetaInit = array2ThetaInit1
                thetaDDotMax = 4.0 * np.abs(array2ThetaRef - thetaInit) / (rot_duration ** 2)
                array2MaxRotAccelList2.append(thetaDDotMax)

    array1RotProfilerList = list()
    array2RotProfilerList = list()
    for i in range(numElements):
        array1RotProfilerList.append(prescribedRotation1DOF.PrescribedRotation1DOF())
        array2RotProfilerList.append(prescribedRotation1DOF.PrescribedRotation1DOF())
        array1RotProfilerList[i].ModelTag = "prescribedRotation1DOFArray1Element" + str(i + 1)
        array2RotProfilerList[i].ModelTag = "prescribedRotation1DOFArray1Element" + str(i + 1)
        array1RotProfilerList[i].setRotHat_M(rot_hat_M)
        array2RotProfilerList[i].setRotHat_M(rot_hat_M)
        array1RotProfilerList[i].setThetaDDotMax(thetaDDotMax)
        array2RotProfilerList[i].setThetaDDotMax(thetaDDotMax)
        array1RotProfilerList[i].setThetaInit(array1ThetaInit1)
        array2RotProfilerList[i].setThetaInit(array2ThetaInit1)


        scSim.AddModelToTask(fswTaskName, array1RotProfilerList[i])
        scSim.AddModelToTask(fswTaskName, array2RotProfilerList[i])
        array1RotProfilerList[i].spinningBodyInMsg.subscribeTo(array1ElementRefMsgList[i])
        array2RotProfilerList[i].spinningBodyInMsg.subscribeTo(array2ElementRefMsgList[i])
        array1ElementList[i].prescribedRotationInMsg.subscribeTo(array1RotProfilerList[i].prescribedRotationOutMsg)
        array2ElementList[i].prescribedRotationInMsg.subscribeTo(array2RotProfilerList[i].prescribedRotationOutMsg)

    # Add Translational Information
    trans_hat_M = np.array([0.0, 0.0, 1.0])
    gap = 0.1                                   # space between each array
    targetRho = lengthElement + gap             # distance needed to travel per element
    accelMax = 4.0 * np.abs(targetRho - 0.0) / (translation_duration ** 2) # max acceleration based on translation_duration

    # Define Trans Profiler (no movement for 1st sim)
    array1TransProfilerList = list()
    array2TransProfilerList = list()
    for i in range(numElements):
        array1TransProfilerList.append(prescribedLinearTranslation.PrescribedLinearTranslation())
        array2TransProfilerList.append(prescribedLinearTranslation.PrescribedLinearTranslation())
        array1TransProfilerList[i].setTransHat_M(trans_hat_M)
        array2TransProfilerList[i].setTransHat_M(trans_hat_M)
        array1TransProfilerList[i].setTransAccelMax(accelMax)
        array2TransProfilerList[i].setTransAccelMax(accelMax)
        array1TransProfilerList[i].setTransPosInit(0.0)
        array2TransProfilerList[i].setTransPosInit(0.0)

        noMoveRef = messaging.LinearTranslationRigidBodyMsgPayload()
        noMoveRef.rho = 0.0
        noMoveRef.rhoDot = 0.0
        array1TransProfilerList[i].linearTranslationRigidBodyInMsg.subscribeTo(messaging.LinearTranslationRigidBodyMsg().write(noMoveRef))
        array2TransProfilerList[i].linearTranslationRigidBodyInMsg.subscribeTo(messaging.LinearTranslationRigidBodyMsg().write(noMoveRef))

        array1ElementList[i].prescribedTranslationInMsg.subscribeTo(array1TransProfilerList[i].prescribedTranslationOutMsg)
        array2ElementList[i].prescribedTranslationInMsg.subscribeTo(array2TransProfilerList[i].prescribedTranslationOutMsg)

        scSim.AddModelToTask(fswTaskName, array1TransProfilerList[i])
        scSim.AddModelToTask(fswTaskName, array2TransProfilerList[i])

    # Set up data logging
    scStateData = scObject.scStateOutMsg.recorder(dataRecRate)
    scSim.AddModelToTask(fswTaskName, scStateData)

    array1PrescribedDataLog = list()
    array2PrescribedDataLog = list()
    for i in range(numElements):
        array1PrescribedDataLog.append(array1RotProfilerList[i].spinningBodyOutMsg.recorder(dataRecRate))
        array2PrescribedDataLog.append(array2RotProfilerList[i].spinningBodyOutMsg.recorder(dataRecRate))
        scSim.AddModelToTask(fswTaskName, array1RotProfilerList[i])
        scSim.AddModelToTask(fswTaskName, array2RotProfilerList[i])

    array1TransDataLogList = list()
    array2TransDataLogList = list()
    for i in range(numElements):
        array1TransDataLogList.append(array1TransProfilerList[i].linearTranslationRigidBodyOutMsg.recorder(dataRecRate))
        array2TransDataLogList.append(array2TransProfilerList[i].linearTranslationRigidBodyOutMsg.recorder(dataRecRate))
        scSim.AddModelToTask(fswTaskName, array1TransDataLogList[i])
        scSim.AddModelToTask(fswTaskName, array2TransDataLogList[i])


    if vizSupport.vizFound:
        scBodyList = [scObject]
        for i in range(numElements):
            scBodyList.append(["Array1Element" + str(i+1), array1ElementList[i].prescribedMotionConfigLogOutMsg])
            scBodyList.append(["Array2Element" + str(i+1), array2ElementList[i].prescribedMotionConfigLogOutMsg])

        viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scBodyList,
                                                  saveFile=r"C:\Users\mason\Desktop\_VizFiles\scenarioSimpleArrayDeployment.bin"
                                                  )
        viz.settings.showSpacecraftAsSprites = -1

        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scObject.ModelTag]
                                     , modelPath="CYLINDER"
                                     , scale=[widthHub, depthHub, lengthHub]
                                     , color=vizSupport.toRGBA255("gray"))

        for i in range(numElements):
            vizSupport.createCustomModel(viz,
                                         simBodiesToModify=["Array1Element" + str(i+1)],
                                         # Specifying relative model path is useful for sharing scenarios and resources:
                                         modelPath="CUBE",
                                         scale=[lengthElement, widthElement, thicknessElement],
                                         color=vizSupport.toRGBA255("blue"))
            vizSupport.createCustomModel(viz,
                                         simBodiesToModify=["Array2Element" + str(i+1)],
                                         # Specifying relative model path is useful for sharing scenarios and resources:
                                         modelPath="CUBE",
                                         scale=[lengthElement, widthElement, thicknessElement],
                                         color=vizSupport.toRGBA255("blue"))

    scSim.InitializeSimulation()
    simTime1 = rot_duration + 5  # [s]
    scSim.ConfigureStopTime(macros.sec2nano(simTime1))
    scSim.ExecuteSimulation()

    # Extend Each Panel
    currentTime = simTime1
    count = 1
    for i in range((numElements-1), 0, -1):
        for j in range(numElements - count):
            targetRho = count * (lengthElement + gap)

            transRef = messaging.LinearTranslationRigidBodyMsgPayload()
            transRef.rho = targetRho
            transRef.rhoDot = 0.0
            transMsg = messaging.LinearTranslationRigidBodyMsg().write(transRef)

            array1TransProfilerList[j].linearTranslationRigidBodyInMsg.subscribeTo(transMsg)
            array2TransProfilerList[j].linearTranslationRigidBodyInMsg.subscribeTo(transMsg)

        currentTime += translation_duration
        # Add time after final extension
        if count == numElements - 1:
            currentTime += 3
        scSim.ConfigureStopTime(macros.sec2nano(currentTime))
        scSim.ExecuteSimulation()
        count += 1

    timespan = array1TransDataLogList[0].times() * macros.NANO2SEC

    # Plot Element Positions
    plt.figure()
    for i in range(numElements):
        plt.plot(timespan, array1TransDataLogList[i].rho, label=f"Array1 Element {i+1}")
        plt.xlabel("Time [s]")
        plt.ylabel("rho [m]")
        plt.title("Element Translation Position vs Time")
        plt.legend(bbox_to_anchor=(1.25, 0.5), loc="center left", fontsize=8)
        plt.grid(True)
    plt.figure()
    for i in range(numElements):
        plt.plot(timespan, array2TransDataLogList[i].rho, linestyle="--", label=f"Array2 Element {i+1}")
        plt.xlabel("Time [s]")
        plt.ylabel("rho [m]")
        plt.title("Element Translation Position vs Time")
        plt.legend(bbox_to_anchor=(1.25, 0.5), loc="center left", fontsize=8)
        plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

if __name__ == "__main__":
    run(show_plots = True)