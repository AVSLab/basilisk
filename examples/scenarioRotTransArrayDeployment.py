#
# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
:ref:`prescribedMotionStateEffector` module through a sequential rotational and translational solar array deployment
scenario. The spacecraft in this example consists of a rigid hub and two symmetrical solar arrays. Each array is modeled
as a collection of 4 prescribed motion elements. Therefore, 8 instances of the :ref:`prescribedMotionStateEffector`
module are required to simulate the array deployments.

Note that in order to simulate hub-relative prescribed motion, kinematic profiler modules must be connected to the
prescribed motion state effector modules. The kinematic profiler modules specify the translational and rotational
states of each prescribed sub-component relative to the spacecraft hub and write the sub-component states at each time
step to the prescribed motions state effector modules using the Basilisk messaging system. Currently two kinematic
profiler modules exist in Basilisk that can be used to simulate prescribed motion. The first
:ref:`prescribedLinearTranslation` module prescribes linear translational motion of a prescribed sub-component
relative to the hub; while the second :ref:`prescribedRotation1DOF` module prescribes 1 DOF rotational motion relative
to the hub.

The type of deployment simulated in this scenario is a 1 DOF rotational solar array deployment followed by a linear
solar array deployment, therefore the :ref:`prescribedRotation1DOF` and :ref:`prescribedLinearTranslation` modules are
used in this scenario to profile the array element prescribed motion. Note that 8 instances of these profiler modules
are required to profile the array deployments. Also note that because only rotational motion is profiled in the first
movement, :ref:`PrescribedTranslationMsgPayload` messages are required to be written for each array element before the
translational motion is simulated and must be connected directly to the element prescribed motion state
effector modules. This ensures that both the translational and rotational motion of each array element is defined
relative to the hub for the entire simulation.

In this deployment scenario, the solar arrays on both sides of the hub are rotated at the same time. Further, each array
deploys in two stages, meaning that there are four simulation chunks required in this scenario. The 1 DOF rotational
kinematic profiler modules prescribe a bang-bang acceleration profile for each array element during the rotational
deployment phase. After first initializing all array elements to their stowed configuration, the array elements rotate
together in the initial deployment phase upward away from the spacecraft hub. Array 1 rotates -90 degrees about +Y
hub-frame axis while array 2 rotates 90 degrees about the +Y hub-frame axis. After the initial deployment phase for
each array, the main deployment phase beings where the array elements extend simultaneously to their final deployment
configurations. The array elements are extended one a time until the arrays reach their final exteded state.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioRotTransArrayDeployment.py

The scenario outputs five plots. The first two plots illustrate the array element angles relative to the
hub; while the following two plots illustrate the array element rates relative to the hub. The final plots
illustrate the hub's inertial motion during the array deployment. The hub's inertial position, attitude,
angular velocity, and angular velocity magnitude are given. This scenario also creates a Vizard simulation to
visualize the solar array deployment.

Illustration of Simulation Results
----------------------------------

The following plots illustrate the solar array deployment scenario simulation results.

.. image:: /_images/Scenarios/scenarioRotTransArrayDeployment_ArrayElementsTheta.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRotTransArrayDeployment_ArrayElementPositions.svg
    :align: center

.. image:: /_images/Scenarios/scenarioRotTransArrayDeployment_HubInertialMRPAttitude.svg
    :align: center

.. image:: /_images/Scenarios/scenarioRotTransArrayDeployment_HubInertialPosition.svg
    :align: center

.. image:: /_images/Scenarios/scenarioRotTransArrayDeployment_HubInertialAngularVelocity.svg
    :align: center

"""

#
#   Solar Array Deployment Scenario
#   Author:             Mason Drumwright
#   Creation Date:      September 14, 2026
#

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
filename = os.path.basename(os.path.splitext(__file__)[0])
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

    # Add spacecraft module
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scSim.AddModelToTask(dynTaskName, scObject)

    # Define mass properties of hub
    mass_hub = 800 # [kg]
    length_hub = 4.0 # [m]
    width_hub = 1.5 # [m]
    depth_hub = 1.5 # [m]
    IHub_11 = (1/12) * mass_hub * (length_hub * length_hub + depth_hub * depth_hub) # [kg m^2]
    IHub_22 = (1/12) * mass_hub * (length_hub * length_hub + width_hub * width_hub) # [kg m^2]
    IHub_33 = (1/12) * mass_hub * (depth_hub * depth_hub + width_hub * width_hub) # [kg m^2]
    scObject.hub.mHub = mass_hub # [kg]
    scObject.hub.IHubPntBc_B = [[IHub_11, 0.0, 0.0],
                                [0.0, IHub_22, 0.0],
                                [0.0, 0.0, IHub_33]] # [kg m^2]

    # Hub initial states
    scObject.hub.r_CN_NInit = np.array([0.0, 0.0, 0.0])
    scObject.hub.v_CN_NInit = np.array([0.0, 0.0, 0.0])
    scObject.hub.omega_BN_BInit = np.array([0.0, 0.0, 0.0])
    scObject.hub.sigma_BNInit = np.array([[0.0], [0.0], [0.0]])

    # Position of mount frame with respect to solar array frame
    r_M1S1_B = np.array([0.0, 0.0, 0.0])
    r_M2S2_B = np.array([0.0, 0.0, 0.0])

    # Position vector of solar array frame with respect to hub body frame
    r_array1SB_B = np.array([width_hub/2, 0.0, 0.0]) # [m]
    r_array2SB_B = np.array([-width_hub/2, 0.0, 0.0]) # [m]

    # Position vector of mount frame with respect to body frame origin
    r_M1B_B = r_M1S1_B + r_array1SB_B # [m]
    r_M2B_B = r_M2S2_B + r_array2SB_B # [m]

    # Create solar array components
    num_elements = 4
    mass_element = 5.0 # [kg]
    rot_hat_M = np.array([0.0, 1.0, 0.0]) # array rotation axis in mount frame
    length_element = 1.5 # [m]
    width_element = 0.75 # [m]
    thickness_element = 0.01 # [m]
    I_element_11 = (1/12) * mass_element * (width_element**2 + length_element**2) # [kg m^2]
    I_element_22 = (1/12) * mass_element * (length_element**2 + thickness_element**2) # [kg m^2]
    I_element_33 = (1/12) * mass_element * (thickness_element**2 + width_element**2) # [kg m^2]
    IElement_PntPc_P = [[I_element_11, 0.0, 0.0],
                        [0.0, I_element_22, 0.0],
                        [0.0, 0.0, I_element_33]] # [kg m^2]

    # Deployment information
    rot_duration = 30.0 # s
    translation_duration = 10.0 # for each panel (s)

    # Rotation parameters
    array_1_theta_init1 = 0.0 * macros.D2R # [rad]
    array_2_theta_init1 = 0.0 * macros.D2R # [rad]
    theta_d_dot_max = 2.0 * macros.D2R # [rad]
    array_1_theta_ref = -90 * macros.D2R # [rad]
    array_2_theta_ref = 90 * macros.D2R # [rad]
    r_PM1_M1_Init1 = np.array([0.0, 0.0, 0.0])  # [m]
    r_PM2_M2_Init1 = np.array([0.0, 0.0, 0.0])  # [m]
    prv_PM1_Init1 = array_1_theta_init1 * rot_hat_M
    prv_PM2_Init1 = array_2_theta_init1 * rot_hat_M
    sigma_PM_1Init1 = rbk.PRV2MRP(prv_PM1_Init1)
    sigma_PM2_Init1 = rbk.PRV2MRP(prv_PM2_Init1)

    # Create array elements
    array_1_element_list = list()
    array_2_element_list = list()
    for i in range(num_elements):
        array_1_element_list.append(prescribedMotionStateEffector.PrescribedMotionStateEffector())
        array_2_element_list.append(prescribedMotionStateEffector.PrescribedMotionStateEffector())
        array_1_element_list[i].ModelTag = "array1Element" + str(i + 1)
        array_2_element_list[i].ModelTag = "array2Element" + str(i + 1)
        array_1_element_list[i].setMass(mass_element) # [m]
        array_2_element_list[i].setMass(mass_element) # [m]
        array_1_element_list[i].setIPntPc_P(IElement_PntPc_P) # [kg m^2]
        array_2_element_list[i].setIPntPc_P(IElement_PntPc_P) # [kg m^2]
        array_1_element_list[i].setR_MB_B(r_M1B_B) # [m]
        array_2_element_list[i].setR_MB_B(r_M2B_B) # [m]
        array_1_element_list[i].setR_PcP_P([0.0, 0.0, -length_element/2.0]) # [m]
        array_2_element_list[i].setR_PcP_P([0.0, 0.0, -length_element/2.0]) # [m]
        array_1_element_list[i].setR_PM_M(r_PM1_M1_Init1) # [m]
        array_2_element_list[i].setR_PM_M(r_PM2_M2_Init1) # [m]
        array_1_element_list[i].setRPrime_PM_M(np.array([0.0, 0.0, 0.0])) # [m/s]
        array_2_element_list[i].setRPrime_PM_M(np.array([0.0, 0.0, 0.0])) # [m/s]
        array_1_element_list[i].setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))  # [m/s^2]
        array_2_element_list[i].setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))  # [m/s^2]
        array_1_element_list[i].setOmega_PM_P(np.array([0.0, 0.0, 0.0])) # [rad/s]
        array_2_element_list[i].setOmega_PM_P(np.array([0.0, 0.0, 0.0])) # [rad/s]
        array_1_element_list[i].setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))  # [rad/s^2]
        array_2_element_list[i].setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))  # [rad/s^2]
        array_1_element_list[i].setSigma_PM(sigma_PM_1Init1)
        array_2_element_list[i].setSigma_PM(sigma_PM2_Init1)
        array_1_element_list[i].setSigma_MB([0.0, 0.0, 0.0])
        array_2_element_list[i].setSigma_MB([0.0, 0.0, 0.0])

        scObject.addStateEffector(array_1_element_list[i])
        scObject.addStateEffector(array_2_element_list[i])
        scSim.AddModelToTask(dynTaskName, array_1_element_list[i])
        scSim.AddModelToTask(dynTaskName, array_2_element_list[i])

    # Create elements for the reference angles
    array_1_element_ref_msg_list = list()
    array_2_element_ref_msg_list = list()
    for i in range(num_elements):
        array_1_element_message_data = messaging.HingedRigidBodyMsgPayload()
        array_2_element_message_data = messaging.HingedRigidBodyMsgPayload()
        array_1_element_message_data.theta = array_1_theta_ref # [rad]
        array_2_element_message_data.theta = array_2_theta_ref # [rad]
        array_1_element_message_data.thetaDot = 0.0 # [rad/s]
        array_2_element_message_data.thetaDot = 0.0 # [rad/s]
        array_1_element_ref_msg_list.append(messaging.HingedRigidBodyMsg().write(array_1_element_message_data))
        array_2_element_ref_msg_list.append(messaging.HingedRigidBodyMsg().write(array_2_element_message_data))

    # Create translational message data
    array_1_element_translation_msg_data= messaging.PrescribedTranslationMsgPayload()
    array_2_element_translation_msg_data = messaging.PrescribedTranslationMsgPayload()
    array_1_element_translation_msg_data.r_PM_M = r_PM1_M1_Init1  # [m]
    array_2_element_translation_msg_data.r_PM_M = r_PM2_M2_Init1  # [m]
    array_1_element_translation_msg_data.rPrime_PM_M = np.array([0.0, 0.0, 0.0])  # [m/s]
    array_2_element_translation_msg_data.rPrime_PM_M = np.array([0.0, 0.0, 0.0])  # [m/s]
    array_1_element_translation_msg_data.rPrimePrime_PM_M = np.array([0.0, 0.0, 0.0])  # [m/s^2]
    array_2_element_translation_msg_data.rPrimePrime_PM_M = np.array([0.0, 0.0, 0.0])  # [m/s^2]
    array_1_element_translation_msg = messaging.PrescribedTranslationMsg().write(array_1_element_translation_msg_data)
    array_2_element_translation_msg = messaging.PrescribedTranslationMsg().write(array_2_element_translation_msg_data)

    # Initialize the prescribed rotation 1DOF module
    array1_max_rot_accel_list1 = []
    array2_max_rot_accel_list2 = []
    for j in range(2):
        for i in range(num_elements):
            if j == 0:
                theta_init = array_1_theta_init1 # [rad]
                theta_d_dot_max = 4.0 * np.abs(array_1_theta_ref - theta_init) / (rot_duration ** 2) # [rad/s^2]
                array1_max_rot_accel_list1.append(theta_d_dot_max) # [rad/s^2]
            else:
                theta_init = array_2_theta_init1 # [rad]
                theta_d_dot_max = 4.0 * np.abs(array_2_theta_ref - theta_init) / (rot_duration ** 2) # [rad/s^2]
                array2_max_rot_accel_list2.append(theta_d_dot_max) # [rad/s^2]

    array1_rot_profiler_list = list()
    array2_rot_profiler_list = list()
    for i in range(num_elements):
        array1_rot_profiler_list.append(prescribedRotation1DOF.PrescribedRotation1DOF())
        array2_rot_profiler_list.append(prescribedRotation1DOF.PrescribedRotation1DOF())
        array1_rot_profiler_list[i].ModelTag = "prescribedRotation1DOFArray1Element" + str(i + 1)
        array2_rot_profiler_list[i].ModelTag = "prescribedRotation1DOFArray1Element" + str(i + 1)
        array1_rot_profiler_list[i].setRotHat_M(rot_hat_M)
        array2_rot_profiler_list[i].setRotHat_M(rot_hat_M)
        array1_rot_profiler_list[i].setThetaDDotMax(theta_d_dot_max)
        array2_rot_profiler_list[i].setThetaDDotMax(theta_d_dot_max)
        array1_rot_profiler_list[i].setThetaInit(array_1_theta_init1)
        array2_rot_profiler_list[i].setThetaInit(array_2_theta_init1)


        scSim.AddModelToTask(fswTaskName, array1_rot_profiler_list[i])
        scSim.AddModelToTask(fswTaskName, array2_rot_profiler_list[i])
        array1_rot_profiler_list[i].spinningBodyInMsg.subscribeTo(array_1_element_ref_msg_list[i])
        array2_rot_profiler_list[i].spinningBodyInMsg.subscribeTo(array_2_element_ref_msg_list[i])
        array_1_element_list[i].prescribedRotationInMsg.subscribeTo(array1_rot_profiler_list[i].prescribedRotationOutMsg)
        array_2_element_list[i].prescribedRotationInMsg.subscribeTo(array2_rot_profiler_list[i].prescribedRotationOutMsg)

    # Add translational information
    trans_hat_M = np.array([1.0, 0.0, 0.0]) # translation axis
    gap = 0.1  # space between each array
    target_rho = length_element + gap  # distance needed to travel per element
    accel_max = 4.0 * np.abs(target_rho - 0.0) / (translation_duration ** 2) # max acceleration based on translation_duration

    # Define trans profiler (no movement for 1st sim)
    array1_trans_profiler_list = list()
    array2_trans_profiler_list = list()
    for i in range(num_elements):
        array1_trans_profiler_list.append(prescribedLinearTranslation.PrescribedLinearTranslation())
        array2_trans_profiler_list.append(prescribedLinearTranslation.PrescribedLinearTranslation())
        array1_trans_profiler_list[i].setTransHat_M(trans_hat_M)
        array2_trans_profiler_list[i].setTransHat_M(trans_hat_M)
        array1_trans_profiler_list[i].setTransAccelMax(accel_max)
        array2_trans_profiler_list[i].setTransAccelMax(accel_max)
        array1_trans_profiler_list[i].setTransPosInit(0.0)
        array2_trans_profiler_list[i].setTransPosInit(0.0)

        no_move_ref = messaging.LinearTranslationRigidBodyMsgPayload()
        no_move_ref.rho = 0.0
        no_move_ref.rhoDot = 0.0
        array1_trans_profiler_list[i].linearTranslationRigidBodyInMsg.subscribeTo(messaging.LinearTranslationRigidBodyMsg().write(no_move_ref))
        array2_trans_profiler_list[i].linearTranslationRigidBodyInMsg.subscribeTo(messaging.LinearTranslationRigidBodyMsg().write(no_move_ref))

        array_1_element_list[i].prescribedTranslationInMsg.subscribeTo(array1_trans_profiler_list[i].prescribedTranslationOutMsg)
        array_2_element_list[i].prescribedTranslationInMsg.subscribeTo(array2_trans_profiler_list[i].prescribedTranslationOutMsg)

        scSim.AddModelToTask(fswTaskName, array1_trans_profiler_list[i])
        scSim.AddModelToTask(fswTaskName, array2_trans_profiler_list[i])

    # Set up data logging
    sc_state_data = scObject.scStateOutMsg.recorder(dataRecRate)
    scSim.AddModelToTask(fswTaskName, sc_state_data)

    array1_prescribed_data_log = list()
    array2_prescribed_data_log = list()
    for i in range(num_elements):
        array1_prescribed_data_log.append(array1_rot_profiler_list[i].spinningBodyOutMsg.recorder(dataRecRate))
        array2_prescribed_data_log.append(array2_rot_profiler_list[i].spinningBodyOutMsg.recorder(dataRecRate))
        scSim.AddModelToTask(fswTaskName, array1_prescribed_data_log[i])
        scSim.AddModelToTask(fswTaskName, array2_prescribed_data_log[i])

    array1_trans_data_log_list = list()
    array2_trans_data_log_list = list()
    for i in range(num_elements):
        array1_trans_data_log_list.append(array1_trans_profiler_list[i].linearTranslationRigidBodyOutMsg.recorder(dataRecRate))
        array2_trans_data_log_list.append(array2_trans_profiler_list[i].linearTranslationRigidBodyOutMsg.recorder(dataRecRate))
        scSim.AddModelToTask(fswTaskName, array1_trans_data_log_list[i])
        scSim.AddModelToTask(fswTaskName, array2_trans_data_log_list[i])

    if vizSupport.vizFound:
        sc_body_list = [scObject]
        for i in range(num_elements):
            sc_body_list.append(["Array1Element" + str(i+1), array_1_element_list[i].prescribedMotionConfigLogOutMsg])
            sc_body_list.append(["Array2Element" + str(i+1), array_2_element_list[i].prescribedMotionConfigLogOutMsg])

        viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, sc_body_list,
                                                  saveFile=filename
                                                  )
        viz.settings.showSpacecraftAsSprites = -1

        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scObject.ModelTag]
                                     , modelPath="CUBE"
                                     , scale=[width_hub, depth_hub, length_hub]
                                     , color=vizSupport.toRGBA255("gray"))

        for i in range(num_elements):
            vizSupport.createCustomModel(viz,
                                         simBodiesToModify=["Array1Element" + str(i+1)],
                                         # Specifying relative model path is useful for sharing scenarios and resources:
                                         modelPath="CUBE",
                                         scale=[thickness_element, width_element, length_element],
                                         color=vizSupport.toRGBA255("blue"))
            vizSupport.createCustomModel(viz,
                                         simBodiesToModify=["Array2Element" + str(i+1)],
                                         # Specifying relative model path is useful for sharing scenarios and resources:
                                         modelPath="CUBE",
                                         scale=[thickness_element, width_element, length_element],
                                         color=vizSupport.toRGBA255("blue"))

    # Run simulation
    scSim.InitializeSimulation()
    simTime1 = rot_duration + 5  # [s]
    scSim.ConfigureStopTime(macros.sec2nano(simTime1))
    scSim.ExecuteSimulation()

    # Extend each panel
    current_time = simTime1
    count = 1
    for i in range((num_elements-1), 0, -1):
        for j in range(num_elements - count):
            target_rho = count * (length_element + gap)

            trans_ref_1 = messaging.LinearTranslationRigidBodyMsgPayload()
            trans_ref_1.rho = target_rho
            trans_ref_1.rhoDot = 0.0

            trans_ref_2 = messaging.LinearTranslationRigidBodyMsgPayload()
            trans_ref_2.rho = -target_rho
            trans_ref_2.rhoDot = 0.0

            trans_msg1 = messaging.LinearTranslationRigidBodyMsg().write(trans_ref_1)
            trans_msg2 = messaging.LinearTranslationRigidBodyMsg().write(trans_ref_2)

            array1_trans_profiler_list[j].linearTranslationRigidBodyInMsg.subscribeTo(trans_msg1)
            array2_trans_profiler_list[j].linearTranslationRigidBodyInMsg.subscribeTo(trans_msg2)

        current_time += translation_duration
        # Add time after final extension
        if count == num_elements - 1:
            current_time += 3
        scSim.ConfigureStopTime(macros.sec2nano(current_time))
        scSim.ExecuteSimulation()
        count += 1

    timespan = sc_state_data.times() * macros.NANO2MIN  # [min]
    r_BN_N = sc_state_data.r_BN_N  # [m]
    omega_BN_B = sc_state_data.omega_BN_B * macros.R2D  # [deg/s]
    sigma_BN = sc_state_data.sigma_BN

    # Plot the results
    figure_list = {}

    # Array element angle relative to hub
    plt.figure(1)
    for i in range(num_elements):
        plt.plot(timespan, array1_prescribed_data_log[i].theta * macros.R2D, label=f"Array 1 Element {i+1}")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle Relative to Hub [deg]")
    plt.title("Array 1 Element Angle Relative to Hub")
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc="center left", fontsize=8)
    plt.grid(True)
    # Array 2 element angle relative to hub
    for i in range(num_elements):
        plt.plot(timespan, array2_prescribed_data_log[i].theta * macros.R2D, linestyle="--", label=f"Array 2 Element {i+1}")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle Relative to Hub [deg]")
    plt.title("Array 2 Element Angle Relative to Hub")
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc="center left", fontsize=8)
    plt.grid(True)
    pltName = filename + "_ArrayElementsTheta"
    figure_list[pltName] = plt.figure(1)

    # Plot element positions
    plt.figure(2)
    for i in range(num_elements):
        plt.plot(timespan, array1_trans_data_log_list[i].rho, label=f"Array1 Element {i+1}")
        plt.xlabel("Time [s]")
        plt.ylabel("rho [m]")
        plt.title("Element Translation Position vs Time")
        plt.legend(bbox_to_anchor=(1.25, 0.5), loc="center left", fontsize=8)
        plt.grid(True)
    for i in range(num_elements):
        plt.plot(timespan, array2_trans_data_log_list[i].rho, linestyle="--", label=f"Array2 Element {i+1}")
    plt.xlabel("Time [s]")
    plt.ylabel("rho [m]")
    plt.title("Element Translation Position vs Time")
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc="center left", fontsize=8)
    plt.grid(True)
    pltName = filename + "_ArrayElementPositions"
    figure_list[pltName] = plt.figure(2)

    # Plot sigma_BN
    plt.figure(3)
    plt.plot(timespan, sigma_BN[:, 0], label=r"$\sigma_1$")
    plt.plot(timespan, sigma_BN[:, 1], label=r"$\sigma_2$")
    plt.plot(timespan, sigma_BN[:, 2], label=r"$\sigma_3$")
    plt.xlabel("Time [min]")
    plt.ylabel("MRP")
    plt.title("Hub Inertial Attitude (MRP) vs Time")
    plt.legend()
    plt.grid(True)
    pltName = filename + "_HubInertialMRPAttitude"
    figure_list[pltName] = plt.figure(3)

    # Plot r_BN_N
    plt.figure(4)
    plt.plot(timespan, r_BN_N[:, 0], label="x")
    plt.plot(timespan, r_BN_N[:, 1], label="y")
    plt.plot(timespan, r_BN_N[:, 2], label="z")
    plt.xlabel("Time [min]")
    plt.ylabel("Position [m]")
    plt.title("Hub Inertial Position vs Time")
    plt.legend()
    plt.grid(True)
    pltName = filename + "_HubInertialPosition"
    figure_list[pltName] = plt.figure(4)

    # Plot omega_BN_B
    plt.figure(5)
    plt.plot(timespan, omega_BN_B[:, 0], label=r"$\omega_1$")
    plt.plot(timespan, omega_BN_B[:, 1], label=r"$\omega_2$")
    plt.plot(timespan, omega_BN_B[:, 2], label=r"$\omega_3$")
    plt.xlabel("Time [min]")
    plt.ylabel("Angular Rate [deg/s]")
    plt.title("Hub Angular Velocity vs Time")
    plt.legend()
    plt.grid(True)
    pltName = filename + "_HubInertialAngularVelocity"
    figure_list[pltName] = plt.figure(5)

    if show_plots:
        plt.show()
    plt.close("all")

if __name__ == "__main__":
    run(show_plots = True)
