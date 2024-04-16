#
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

r"""
Overview
--------

This scenario demonstrates the multi-body prescribed motion dynamics capability of the
:ref:`prescribedMotionStateEffector` module through an illustrative rotational solar array deployment scenario.
The spacecraft in this example consists of a rigid hub and two symmetrical solar arrays. Each array is modeled as a
collection of 10 prescribed motion elements. Therefore, 20 instances of the :ref:`prescribedMotionStateEffector` module
are required to simulate the array deployments.

Note that in order to simulate hub-relative prescribed motion, kinematic profiler modules must be connected to the
prescribed motion state effector modules. The kinematic profiler modules specify the translational and rotational
states of each prescribed sub-component relative to the spacecraft hub and write the sub-component states at each time
step to the prescribed motions state effector modules using the Basilisk messaging system. Currently two kinematic
profiler modules exist in Basilisk that can be used to simulate prescribed motion. The first
:ref:`prescribedLinearTranslation` module prescribes linear translational motion of a prescribed sub-component
relative to the hub; while the second :ref:`prescribedRotation1DOF` module prescribes 1 DOF rotational motion relative
to the hub.

The type of deployment simulated in this scenario is a purely 1 DOF rotational solar array deployment; therefore only the
:ref:`prescribedRotation1DOF` module is used in this scenario to profile the array element prescribed motion. Note that
20 instances of this rotational profiler module are required to profile the array deployments. Also note that because
only rotational motion is profiled in this scenario, stand-alone :ref:`PrescribedTranslationMsgPayload` messages
are required to be written for each array element and must be connected directly to the element prescribed motion state
effector modules. This ensures that both the translational and rotational motion of each array element is defined
relative to the hub for the entire simulation.

In this deployment scenario, the solar array positioned along the +X hub-frame axis fully deploys first, followed
by deployment of the second solar array. Further, each array deploys in two stages, meaning that there are four
simulation chunks required in this scenario. The 1 DOF rotational kinematic profiler modules prescribe a bang-coast-bang
acceleration profile for each array element during both deployment phases. After first initializing all array elements
to their stowed configuration, the array elements rotate together in the initial deployment phase downward away from the
spacecraft hub. Array 1 rotates 108 degrees about +Y hub-frame axis while array 2 rotates -108 degrees about the
+Y hub-frame axis. After the initial deployment phase for each array, the main deployment phase beings where the array
elements unfurl simultaneously to their final deployment configurations. Note that each array element is given a
different acceleration profile such that all array elements lock into place together at the final time.

Finally, note that in order to exclusively use the 1 DOF rotational profiler modules to prescribe the array deployments,
the translational position of the array elements must be updated at the start of the main deployment phase for each
array. The array element frames are shifted outwards to each array's deployed center of mass location.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioDeployingSolarArrays.py

The scenario outputs eight plots. The first two plots illustrate the array element angles relative to the
hub; while the following two plots illustrate the array element rates relative to the hub. The final plots
illustrate the hub's inertial motion during the array deployment. The hub's inertial position, attitude,
angular velocity, and angular velocity magnitude are given. This scenario also creates a Vizard simulation to
visualize the solar array deployment.

Illustration of Simulation Results
----------------------------------

The following plots illustrate the solar array deployment scenario simulation results.

.. image:: /_images/Scenarios/scenarioDeployingSolarArrays_Array1ElementTheta.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDeployingSolarArrays_Array2ElementTheta.svg
    :align: center

.. image:: /_images/Scenarios/scenarioDeployingSolarArrays_Array1ElementThetaDot.svg
    :align: center

.. image:: /_images/Scenarios/scenarioDeployingSolarArrays_Array2ElementThetaDot.svg
    :align: center

.. image:: /_images/Scenarios/scenarioDeployingSolarArrays_HubInertialPosition.svg
    :align: center

.. image:: /_images/Scenarios/scenarioDeployingSolarArrays_HubInertialMRPAttitude.svg
    :align: center

.. image:: /_images/Scenarios/scenarioDeployingSolarArrays_HubInertialAngularVelocity.svg
    :align: center

.. image:: /_images/Scenarios/scenarioDeployingSolarArrays_HubInertialAngularVelocityNorm.svg
    :align: center

Visualization in Vizard
-----------------------

An image captured from the Vizard visualization of this simulation script is shown below. In this image, the arrays
are shown in their fully deployed configuration.

.. image:: /_images/static/scenarioDeployingSolarArrays.jpg
   :align: center

"""

#
#   Solar Array Deployment Scenario
#   Author:             Leah Kiner
#   Creation Date:      March 4, 2024
#

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
from Basilisk.utilities import vizSupport

filename = os.path.basename(os.path.splitext(__file__)[0])
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

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    dynTimeStep = 2.0  # [s]
    fswTimeStep = 2.0  # [s]
    dataRecStep = 2.0  # [s]
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
    massHub = 800  # [kg]
    lengthHub = 4.0  # [m]
    widthHub = 2.0  # [m]
    depthHub = 2.0  # [m]
    IHub_11 = (1/12) * massHub * (lengthHub * lengthHub + depthHub * depthHub)  # [kg m^2]
    IHub_22 = (1/12) * massHub * (lengthHub * lengthHub + widthHub * widthHub)  # [kg m^2]
    IHub_33 = (1/12) * massHub * (widthHub * widthHub + depthHub * depthHub)  # [kg m^2]
    scObject.hub.mHub = massHub  # kg
    scObject.hub.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
    scObject.hub.IHubPntBc_B = [[IHub_11, 0.0, 0.0],
                                [0.0, IHub_22, 0.0],
                                [0.0, 0.0, IHub_33]]  # [kg m^2] (Hub approximated as rectangular prism)

    # Set the initial inertial hub states
    scObject.hub.r_CN_NInit = [0.0, 0.0, 0.0]
    scObject.hub.v_CN_NInit = [0.0, 0.0, 0.0]
    scObject.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]

    # Define the array element configuration properties
    # B frame is the hub body frame
    # S1 and S2 frames are the solar array body frames
    # M1 and M2 frames are the mount frames

    # Position vector of mount frame with respect to solar array frame in hub frame components
    r_M1S1_B = [0.0, 0.0, 0.0]  # [m]
    r_M2S2_B = [0.0, 0.0, 0.0]  # [m]

    # Position vector of solar array frame origin points with respect to hub frame origin point B 
    # expressed in B frame components
    rArray1SB_B = np.array([2.0, 0.0, 0.0])   # [m]
    rArray2SB_B = np.array([-2.0, 0.0, 0.0])   # [m]

    # Position vector of mount frame origin points with respect to hub frame origin point B
    r_M1B_B = r_M1S1_B + rArray1SB_B  # [m]
    r_M2B_B = r_M2S2_B + rArray2SB_B  # [m]

    # Array element geometric parameters
    num_elements = 10
    mass_element = 5.0  # [kg]
    rot_hat_M = np.array([0.0, 1.0, 0.0])  # Array articulation axis in mount frame components
    radius_array = 4.0  # [m]
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
                        [0.0, 0.0, I_element_33]]  # [kg m^2] (Elements approximated as rectangular prisms)
    
    # Deployment temporal information
    ramp_duration = 2.0  # [s]
    init_deploy_duration = 5.0 * 60.0  # [s]
    main_deploy_duration = 30.0 * 60.0  # [s]
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
        array1ElementList[i].ModelTag = "array1Element" + str(i + 1)
        array2ElementList[i].ModelTag = "array2Element" + str(i + 1)
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
                                        (1/3) * radius_array * np.sin(72 * macros.D2R)]  # [m] For triangular wedge
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
        array1RotProfilerList[i].ModelTag = "prescribedRotation1DOFArray1Element" + str(i + 1)
        array2RotProfilerList[i].ModelTag = "prescribedRotation1DOFArray2Element" + str(i + 1)
        array1RotProfilerList[i].setCoastOptionBangDuration(ramp_duration)  # [s]
        array2RotProfilerList[i].setCoastOptionBangDuration(ramp_duration)  # [s]
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
    scStateData = scObject.scStateOutMsg.recorder(dataRecRate)
    array1Element1PrescribedDataLog = array1RotProfilerList[0].spinningBodyOutMsg.recorder(dataRecRate)
    array1Element2PrescribedDataLog = array1RotProfilerList[1].spinningBodyOutMsg.recorder(dataRecRate)
    array1Element3PrescribedDataLog = array1RotProfilerList[2].spinningBodyOutMsg.recorder(dataRecRate)
    array1Element4PrescribedDataLog = array1RotProfilerList[3].spinningBodyOutMsg.recorder(dataRecRate)
    array1Element5PrescribedDataLog = array1RotProfilerList[4].spinningBodyOutMsg.recorder(dataRecRate)
    array1Element6PrescribedDataLog = array1RotProfilerList[5].spinningBodyOutMsg.recorder(dataRecRate)
    array1Element7PrescribedDataLog = array1RotProfilerList[6].spinningBodyOutMsg.recorder(dataRecRate)
    array1Element8PrescribedDataLog = array1RotProfilerList[7].spinningBodyOutMsg.recorder(dataRecRate)
    array1Element9PrescribedDataLog = array1RotProfilerList[8].spinningBodyOutMsg.recorder(dataRecRate)
    array1Element10PrescribedDataLog = array1RotProfilerList[9].spinningBodyOutMsg.recorder(dataRecRate)
    array2Element1PrescribedDataLog = array2RotProfilerList[0].spinningBodyOutMsg.recorder(dataRecRate)
    array2Element2PrescribedDataLog = array2RotProfilerList[1].spinningBodyOutMsg.recorder(dataRecRate)
    array2Element3PrescribedDataLog = array2RotProfilerList[2].spinningBodyOutMsg.recorder(dataRecRate)
    array2Element4PrescribedDataLog = array2RotProfilerList[3].spinningBodyOutMsg.recorder(dataRecRate)
    array2Element5PrescribedDataLog = array2RotProfilerList[4].spinningBodyOutMsg.recorder(dataRecRate)
    array2Element6PrescribedDataLog = array2RotProfilerList[5].spinningBodyOutMsg.recorder(dataRecRate)
    array2Element7PrescribedDataLog = array2RotProfilerList[6].spinningBodyOutMsg.recorder(dataRecRate)
    array2Element8PrescribedDataLog = array2RotProfilerList[7].spinningBodyOutMsg.recorder(dataRecRate)
    array2Element9PrescribedDataLog = array2RotProfilerList[8].spinningBodyOutMsg.recorder(dataRecRate)
    array2Element10PrescribedDataLog = array2RotProfilerList[9].spinningBodyOutMsg.recorder(dataRecRate)
    
    scSim.AddModelToTask(fswTaskName, scStateData)
    scSim.AddModelToTask(fswTaskName, array1Element1PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array1Element2PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array1Element3PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array1Element4PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array1Element5PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array1Element6PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array1Element7PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array1Element8PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array1Element9PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array1Element10PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array2Element1PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array2Element2PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array2Element3PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array2Element4PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array2Element5PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array2Element6PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array2Element7PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array2Element8PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array2Element9PrescribedDataLog)
    scSim.AddModelToTask(fswTaskName, array2Element10PrescribedDataLog)

    # Set up Vizard visualization
    if vizSupport.vizFound:
        scBodyList = [scObject]
        for i in range(num_elements):
            scBodyList.append(["Array1Element" + str(i+1), array1ElementList[i].prescribedMotionConfigLogOutMsg])
            scBodyList.append(["Array2Element" + str(i+1), array2ElementList[i].prescribedMotionConfigLogOutMsg])

        viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scBodyList,
                                                  # saveFile=filename
                                                  )
        viz.settings.showSpacecraftAsSprites = -1
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scObject.ModelTag]
                                     , modelPath="CYLINDER"
                                     , scale=[widthHub, depthHub, lengthHub]
                                     , color=vizSupport.toRGBA255("gray"))

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
    r_BN_N = scStateData.r_BN_N  # [m]
    omega_BN_B = scStateData.omega_BN_B * macros.R2D  # [deg/s]
    sigma_BN = scStateData.sigma_BN
    theta_array1Element1 = array1Element1PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array1Element2 = array1Element2PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array1Element3 = array1Element3PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array1Element4 = array1Element4PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array1Element5 = array1Element5PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array1Element6 = array1Element6PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array1Element7 = array1Element7PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array1Element8 = array1Element8PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array1Element9 = array1Element9PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array1Element10 = array1Element10PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array2Element1 = array2Element1PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array2Element2 = array2Element2PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array2Element3 = array2Element3PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array2Element4 = array2Element4PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array2Element5 = array2Element5PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array2Element6 = array2Element6PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array2Element7 = array2Element7PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array2Element8 = array2Element8PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array2Element9 = array2Element9PrescribedDataLog.theta * macros.R2D  # [deg]
    theta_array2Element10 = array2Element10PrescribedDataLog.theta * macros.R2D  # [deg]
    thetaDot_array1Element1 = array1Element1PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array1Element2 = array1Element2PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array1Element3 = array1Element3PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array1Element4 = array1Element4PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array1Element5 = array1Element5PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array1Element6 = array1Element6PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array1Element7 = array1Element7PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array1Element8 = array1Element8PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array1Element9 = array1Element9PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array1Element10 = array1Element10PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array2Element1 = array2Element1PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array2Element2 = array2Element2PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array2Element3 = array2Element3PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array2Element4 = array2Element4PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array2Element5 = array2Element5PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array2Element6 = array2Element6PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array2Element7 = array2Element7PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array2Element8 = array2Element8PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array2Element9 = array2Element9PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]
    thetaDot_array2Element10 = array2Element10PrescribedDataLog.thetaDot * macros.R2D  # [deg/s]

    # Plot the results
    figureList = {}
    plt.close("all")

    # Plot array 1 element angles
    plt.figure(1)
    plt.clf()
    plt.plot(timespan, theta_array1Element1, label=r'$\theta_1$')
    plt.plot(timespan, theta_array1Element2, label=r'$\theta_2$')
    plt.plot(timespan, theta_array1Element3, label=r'$\theta_3$')
    plt.plot(timespan, theta_array1Element4, label=r'$\theta_4$')
    plt.plot(timespan, theta_array1Element5, label=r'$\theta_5$')
    plt.plot(timespan, theta_array1Element6, label=r'$\theta_6$')
    plt.plot(timespan, theta_array1Element7, label=r'$\theta_7$')
    plt.plot(timespan, theta_array1Element8, label=r'$\theta_8$')
    plt.plot(timespan, theta_array1Element9, label=r'$\theta_9$')
    plt.plot(timespan, theta_array1Element10, label=r'$\theta_{10}$')
    plt.title(r'Array 1 Element Angles', fontsize=16)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc='center right', prop={'size': 8})
    plt.grid(True)
    pltName = filename + "_Array1ElementTheta"
    figureList[pltName] = plt.figure(1)

    # Plot array 2 element angles
    plt.figure(2)
    plt.clf()
    plt.plot(timespan, theta_array2Element1, label=r'$\theta_1$')
    plt.plot(timespan, theta_array2Element2, label=r'$\theta_2$')
    plt.plot(timespan, theta_array2Element3, label=r'$\theta_3$')
    plt.plot(timespan, theta_array2Element4, label=r'$\theta_4$')
    plt.plot(timespan, theta_array2Element5, label=r'$\theta_5$')
    plt.plot(timespan, theta_array2Element6, label=r'$\theta_6$')
    plt.plot(timespan, theta_array2Element7, label=r'$\theta_7$')
    plt.plot(timespan, theta_array2Element8, label=r'$\theta_8$')
    plt.plot(timespan, theta_array2Element9, label=r'$\theta_9$')
    plt.plot(timespan, theta_array2Element10, label=r'$\theta_{10}$')
    plt.title(r'Array 2 Element Angles', fontsize=16)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc='center right', prop={'size': 8})
    plt.grid(True)
    pltName = filename + "_Array2ElementTheta"
    figureList[pltName] = plt.figure(2)

    # Plot array 1 element angle rates
    plt.figure(3)
    plt.clf()
    plt.plot(timespan, thetaDot_array1Element1, label=r'$\dot{\theta}_1$')
    plt.plot(timespan, thetaDot_array1Element2, label=r'$\dot{\theta}_2$')
    plt.plot(timespan, thetaDot_array1Element3, label=r'$\dot{\theta}_3$')
    plt.plot(timespan, thetaDot_array1Element4, label=r'$\dot{\theta}_4$')
    plt.plot(timespan, thetaDot_array1Element5, label=r'$\dot{\theta}_5$')
    plt.plot(timespan, thetaDot_array1Element6, label=r'$\dot{\theta}_6$')
    plt.plot(timespan, thetaDot_array1Element7, label=r'$\dot{\theta}_7$')
    plt.plot(timespan, thetaDot_array1Element8, label=r'$\dot{\theta}_8$')
    plt.plot(timespan, thetaDot_array1Element9, label=r'$\dot{\theta}_9$')
    plt.plot(timespan, thetaDot_array1Element10, label=r'$\dot{\theta}_{10}$')
    plt.title(r'Array 1 Element Angle Rates', fontsize=16)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc='center right', prop={'size': 8})
    plt.grid(True)
    pltName = filename + "_Array1ElementThetaDot"
    figureList[pltName] = plt.figure(3)

    # Plot array 2 element angle rates
    plt.figure(4)
    plt.clf()
    plt.plot(timespan, thetaDot_array2Element1, label=r'$\dot{\theta}_1$')
    plt.plot(timespan, thetaDot_array2Element2, label=r'$\dot{\theta}_2$')
    plt.plot(timespan, thetaDot_array2Element3, label=r'$\dot{\theta}_3$')
    plt.plot(timespan, thetaDot_array2Element4, label=r'$\dot{\theta}_4$')
    plt.plot(timespan, thetaDot_array2Element5, label=r'$\dot{\theta}_5$')
    plt.plot(timespan, thetaDot_array2Element6, label=r'$\dot{\theta}_6$')
    plt.plot(timespan, thetaDot_array2Element7, label=r'$\dot{\theta}_7$')
    plt.plot(timespan, thetaDot_array2Element8, label=r'$\dot{\theta}_8$')
    plt.plot(timespan, thetaDot_array2Element9, label=r'$\dot{\theta}_9$')
    plt.plot(timespan, thetaDot_array2Element10, label=r'$\dot{\theta}_{10}$')
    plt.title(r'Array 2 Element Angle Rates', fontsize=16)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc='center right', prop={'size': 8})
    plt.grid(True)
    pltName = filename + "_Array2ElementThetaDot"
    figureList[pltName] = plt.figure(4)

    # Plot r_BN_N
    plt.figure(5)
    plt.clf()
    plt.plot(timespan, r_BN_N[:, 0], label=r'$r_{1}$')
    plt.plot(timespan, r_BN_N[:, 1], label=r'$r_{2}$')
    plt.plot(timespan, r_BN_N[:, 2], label=r'$r_{3}$')
    plt.title(r'${}^\mathcal{N} r_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial Trajectory', fontsize=16)
    plt.ylabel('(m)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc='center right', prop={'size': 12})
    plt.grid(True)
    pltName = filename + "_HubInertialPosition"
    figureList[pltName] = plt.figure(5)

    # Plot sigma_BN
    plt.figure(6)
    plt.clf()
    plt.plot(timespan, sigma_BN[:, 0], label=r'$\sigma_{1}$')
    plt.plot(timespan, sigma_BN[:, 1], label=r'$\sigma_{2}$')
    plt.plot(timespan, sigma_BN[:, 2], label=r'$\sigma_{3}$')
    plt.title(r'$\sigma_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial MRP Attitude', fontsize=16)
    plt.ylabel('', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc='center right', prop={'size': 12})
    plt.grid(True)
    pltName = filename + "_HubInertialMRPAttitude"
    figureList[pltName] = plt.figure(6)

    # Plot omega_BN_B
    plt.figure(7)
    plt.clf()
    plt.plot(timespan, omega_BN_B[:, 0], label=r'$\omega_{1}$')
    plt.plot(timespan, omega_BN_B[:, 1], label=r'$\omega_{2}$')
    plt.plot(timespan, omega_BN_B[:, 2], label=r'$\omega_{3}$')
    plt.title(r'Spacecraft Hub Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$', fontsize=16)
    plt.xlabel('Time (min)', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc='center right', prop={'size': 12})
    plt.grid(True)
    pltName = filename + "_HubInertialAngularVelocity"
    figureList[pltName] = plt.figure(7)

    # Plot omega_BN_B norm
    omega_BN_BNorm = np.linalg.norm(omega_BN_B, axis=1)
    plt.figure(8)
    plt.clf()
    plt.plot(timespan, omega_BN_BNorm)
    plt.title(r'Hub Angular Velocity Norm $|{}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}|$', fontsize=16)
    plt.ylabel(r'(deg/s)', fontsize=14)
    plt.xlabel(r'(min)', fontsize=14)
    plt.grid(True)
    pltName = filename + "_HubInertialAngularVelocityNorm"
    figureList[pltName] = plt.figure(8)

    if show_plots:
        plt.show()
    plt.close("all")

    return figureList

if __name__ == "__main__":
    run(
        True,   # show_plots
    )
    