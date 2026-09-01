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
module are required to simulate the array deployments. Each element has a mass of 50 kg to make the hub's rotational
and translational response to the unequal deployment rates easier to see.

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

In this deployment scenario, the solar arrays on both sides of the hub begin rotating at the same time, with array 2
taking 25 percent longer than array 1. The 1 DOF rotational kinematic profiler modules prescribe a bang-bang acceleration
profile for each array element. After first initializing all array elements to their stowed configuration, the elements
within each array rotate together upward away from the spacecraft hub. Array 1 rotates -90 degrees about the +Y
hub-frame axis in 30 seconds, while array 2 rotates 90 degrees about the +Y hub-frame axis in 37.5 seconds. After both
arrays finish rotating, the elements extend in three sequential steps. Each extension step takes 10 seconds for array 1
and 12.5 seconds for array 2. Each step begins on both sides together, and the simulation waits for the slower array
before advancing to the next step. The rotation and three extension steps require four simulation chunks.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioRotTransArrayDeployment.py

The scenario outputs five plots. The first plot illustrates the array element angles relative to the
hub and the following plot illustrates the array element displacements relative to the hub. The last three plots
illustrate the hub's inertial attitude, position, and angular velocity. This scenario also creates a Vizard simulation to
visualize the solar array deployment.

Vizard uses its default directional lighting. To make panel shadows easier to see, use Vizard's ``Beautiful`` graphics
quality and a close spacecraft view. If needed, lower the ambient lighting and spacecraft shadow brightness manually
in Vizard's settings.

Illustration of Simulation Results
----------------------------------

The following plots illustrate the solar array deployment scenario simulation results.

.. image:: /_images/Scenarios/scenarioRotTransArrayDeployment_ArrayElementsTheta.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRotTransArrayDeployment_ArrayElementsRho.svg
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
from Basilisk.simulation import spacecraft, prescribedLinearTranslation
from Basilisk.simulation import prescribedMotionStateEffector
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros, RigidBodyKinematics as rbk
from Basilisk.utilities import vizSupport
filename = os.path.basename(os.path.splitext(__file__)[0])
def run(show_plots):

    sim_process_name = "sim_process"
    dyn_task_name = "dynTask"
    fsw_task_name = "fswTask"

    sc_sim = SimulationBaseClass.SimBaseClass()

    dyn_time_step = 0.05  # [s]
    fsw_time_step = 0.05  # [s]
    data_rec_step = 2.0  # [s]
    dyn_process_rate = macros.sec2nano(dyn_time_step)
    fsw_process_rate = macros.sec2nano(fsw_time_step)
    data_rec_rate = macros.sec2nano(data_rec_step)
    sim_proc = sc_sim.CreateNewProcess(sim_process_name)
    sim_proc.addTask(sc_sim.CreateNewTask(dyn_task_name, dyn_process_rate))
    sim_proc.addTask(sc_sim.CreateNewTask(fsw_task_name, fsw_process_rate))

    # Add spacecraft module
    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "spacecraftBody"
    sc_sim.AddModelToTask(dyn_task_name, sc_object)

    # Define mass properties of hub
    mass_hub = 800 # [kg]
    length_hub = 4.0 # [m]
    width_hub = 1.5 # [m]
    depth_hub = 1.5 # [m]
    IHub_11 = (1/12) * mass_hub * (length_hub * length_hub + depth_hub * depth_hub) # [kg m^2]
    IHub_22 = (1/12) * mass_hub * (length_hub * length_hub + width_hub * width_hub) # [kg m^2]
    IHub_33 = (1/12) * mass_hub * (depth_hub * depth_hub + width_hub * width_hub) # [kg m^2]
    sc_object.hub.mHub = mass_hub # [kg]
    sc_object.hub.IHubPntBc_B = [[IHub_11, 0.0, 0.0],
                                [0.0, IHub_22, 0.0],
                                [0.0, 0.0, IHub_33]] # [kg m^2]

    # Hub initial states
    sc_object.hub.r_CN_NInit = np.array([0.0, 0.0, 0.0])
    sc_object.hub.v_CN_NInit = np.array([0.0, 0.0, 0.0])
    sc_object.hub.omega_BN_BInit = np.array([0.0, 0.0, 0.0])
    sc_object.hub.sigma_BNInit = np.array([[0.0], [0.0], [0.0]])

    # Position of mount frame with respect to solar array frame
    r_M1S1_B = np.array([0.0, 0.0, 0.0])
    r_M2S2_B = np.array([0.0, 0.0, 0.0])

    # Position vector of solar array frame with respect to hub body frame
    r_array1SB_B = np.array([width_hub/2, 0.0, 0.0]) # [m]
    r_array2SB_B = np.array([-width_hub/2, 0.0, 0.0]) # [m]

    # Position vector of mount frame with respect to body frame origin
    r_M1B_B = np.array([width_hub/2, 0.0, 0.0]) # [m]
    r_M2B_B = np.array([-width_hub/2, 0.0, 0.0]) # [m]

    # Create solar array components
    num_elements = 4
    mass_element = 50.0  # [kg] Make the hub response to unequal deployment rates visible.
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
    array2_duration_scale = 1.25  # [-] Array 2 takes 25 percent longer for each motion.
    array1_rot_duration = 30.0  # [s]
    array2_rot_duration = array2_duration_scale * array1_rot_duration  # [s]
    array1_translation_duration = 10.0  # [s] Duration of each extension step.
    array2_translation_duration = array2_duration_scale * array1_translation_duration  # [s]

    # Rotation parameters
    array_1_theta_init1 = 0.0 * macros.D2R # [rad]
    array_2_theta_init1 = 0.0 * macros.D2R # [rad]
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
        array_1_element_list[i].setMass(mass_element)  # [kg]
        array_2_element_list[i].setMass(mass_element)  # [kg]
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

        sc_object.addStateEffector(array_1_element_list[i])
        sc_object.addStateEffector(array_2_element_list[i])
        sc_sim.AddModelToTask(dyn_task_name, array_1_element_list[i])
        sc_sim.AddModelToTask(dyn_task_name, array_2_element_list[i])

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

    # Initialize the prescribed rotation 1DOF module
    array1_rot_accel_max = (
        4.0 * np.abs(array_1_theta_ref - array_1_theta_init1) / array1_rot_duration**2
    )  # [rad/s^2]
    array2_rot_accel_max = (
        4.0 * np.abs(array_2_theta_ref - array_2_theta_init1) / array2_rot_duration**2
    )  # [rad/s^2]

    array1_rot_profiler_list = list()
    array2_rot_profiler_list = list()
    for i in range(num_elements):
        array1_rot_profiler_list.append(prescribedRotation1DOF.PrescribedRotation1DOF())
        array2_rot_profiler_list.append(prescribedRotation1DOF.PrescribedRotation1DOF())
        array1_rot_profiler_list[i].ModelTag = "prescribedRotation1DOFArray1Element" + str(i + 1)
        array2_rot_profiler_list[i].ModelTag = "prescribedRotation1DOFArray2Element" + str(i + 1)
        array1_rot_profiler_list[i].setRotHat_M(rot_hat_M)
        array2_rot_profiler_list[i].setRotHat_M(rot_hat_M)
        array1_rot_profiler_list[i].setThetaDDotMax(array1_rot_accel_max)
        array2_rot_profiler_list[i].setThetaDDotMax(array2_rot_accel_max)
        array1_rot_profiler_list[i].setThetaInit(array_1_theta_init1)
        array2_rot_profiler_list[i].setThetaInit(array_2_theta_init1)

        sc_sim.AddModelToTask(fsw_task_name, array1_rot_profiler_list[i])
        sc_sim.AddModelToTask(fsw_task_name, array2_rot_profiler_list[i])
        array1_rot_profiler_list[i].spinningBodyInMsg.subscribeTo(array_1_element_ref_msg_list[i])
        array2_rot_profiler_list[i].spinningBodyInMsg.subscribeTo(array_2_element_ref_msg_list[i])
        array_1_element_list[i].prescribedRotationInMsg.subscribeTo(array1_rot_profiler_list[i].prescribedRotationOutMsg)
        array_2_element_list[i].prescribedRotationInMsg.subscribeTo(array2_rot_profiler_list[i].prescribedRotationOutMsg)

    # Add translational information
    trans_hat_M = np.array([1.0, 0.0, 0.0]) # translation axis
    gap = 0.1  # [m] Space between adjacent elements in a deployed array.
    target_rho = length_element + gap  # [m] Distance traveled in each extension step.
    array1_trans_accel_max = 4.0 * target_rho / array1_translation_duration**2  # [m/s^2]
    array2_trans_accel_max = 4.0 * target_rho / array2_translation_duration**2  # [m/s^2]

    # Define trans profiler (no movement for 1st sim)
    array1_trans_profiler_list = list()
    array2_trans_profiler_list = list()
    for i in range(num_elements):
        array1_trans_profiler_list.append(prescribedLinearTranslation.PrescribedLinearTranslation())
        array2_trans_profiler_list.append(prescribedLinearTranslation.PrescribedLinearTranslation())
        array1_trans_profiler_list[i].setTransHat_M(trans_hat_M)
        array2_trans_profiler_list[i].setTransHat_M(trans_hat_M)
        array1_trans_profiler_list[i].setTransAccelMax(array1_trans_accel_max)
        array2_trans_profiler_list[i].setTransAccelMax(array2_trans_accel_max)
        array1_trans_profiler_list[i].setTransPosInit(0.0)
        array2_trans_profiler_list[i].setTransPosInit(0.0)

        no_move_ref = messaging.LinearTranslationRigidBodyMsgPayload()
        no_move_ref.rho = 0.0
        no_move_ref.rhoDot = 0.0
        array1_trans_profiler_list[i].linearTranslationRigidBodyInMsg.subscribeTo(messaging.LinearTranslationRigidBodyMsg().write(no_move_ref))
        array2_trans_profiler_list[i].linearTranslationRigidBodyInMsg.subscribeTo(messaging.LinearTranslationRigidBodyMsg().write(no_move_ref))

        array_1_element_list[i].prescribedTranslationInMsg.subscribeTo(array1_trans_profiler_list[i].prescribedTranslationOutMsg)
        array_2_element_list[i].prescribedTranslationInMsg.subscribeTo(array2_trans_profiler_list[i].prescribedTranslationOutMsg)

        sc_sim.AddModelToTask(fsw_task_name, array1_trans_profiler_list[i])
        sc_sim.AddModelToTask(fsw_task_name, array2_trans_profiler_list[i])

    # Set up data logging
    sc_state_data = sc_object.scStateOutMsg.recorder(data_rec_rate)
    sc_sim.AddModelToTask(fsw_task_name, sc_state_data)

    array1_prescribed_data_log = list()
    array2_prescribed_data_log = list()
    for i in range(num_elements):
        array1_prescribed_data_log.append(array1_rot_profiler_list[i].spinningBodyOutMsg.recorder(data_rec_rate))
        array2_prescribed_data_log.append(array2_rot_profiler_list[i].spinningBodyOutMsg.recorder(data_rec_rate))
        sc_sim.AddModelToTask(fsw_task_name, array1_prescribed_data_log[i])
        sc_sim.AddModelToTask(fsw_task_name, array2_prescribed_data_log[i])

    array1_trans_data_log_list = list()
    array2_trans_data_log_list = list()
    for i in range(num_elements):
        array1_trans_data_log_list.append(array1_trans_profiler_list[i].linearTranslationRigidBodyOutMsg.recorder(data_rec_rate))
        array2_trans_data_log_list.append(array2_trans_profiler_list[i].linearTranslationRigidBodyOutMsg.recorder(data_rec_rate))
        sc_sim.AddModelToTask(fsw_task_name, array1_trans_data_log_list[i])
        sc_sim.AddModelToTask(fsw_task_name, array2_trans_data_log_list[i])

    if vizSupport.vizFound:
        sc_body_list = [sc_object]
        for i in range(num_elements):
            sc_body_list.append(["Array1Element" + str(i+1), array_1_element_list[i].prescribedMotionConfigLogOutMsg])
            sc_body_list.append(["Array2Element" + str(i+1), array_2_element_list[i].prescribedMotionConfigLogOutMsg])

        viz = vizSupport.enableUnityVisualization(sc_sim, dyn_task_name, sc_body_list,
                                                  #saveFile=filename
                                                  )
        viz.settings.showSpacecraftAsSprites = -1

        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[sc_object.ModelTag]
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
    sc_sim.InitializeSimulation()
    sim_time_1 = max(array1_rot_duration, array2_rot_duration) + 5.0  # [s]
    sc_sim.ConfigureStopTime(macros.sec2nano(sim_time_1))
    sc_sim.ExecuteSimulation()

    # Extend each panel
    current_time = sim_time_1
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

        # Allow one task step for the profilers to read the new commands before timing the motion.
        current_time += max(array1_translation_duration, array2_translation_duration) + fsw_time_step
        # Add time after final extension
        if count == num_elements - 1:
            current_time += 3.0  # [s]
        sc_sim.ConfigureStopTime(macros.sec2nano(current_time))
        sc_sim.ExecuteSimulation()
        count += 1

    timespan = sc_state_data.times() * macros.NANO2MIN  # [min]
    r_BN_N = sc_state_data.r_BN_N  # [m]
    omega_BN_B = sc_state_data.omega_BN_B * macros.R2D  # [deg/s]
    sigma_BN = sc_state_data.sigma_BN

    # Plot the results
    figure_list = {}
    plt.close("all")

    # Array element angle relative to hub
    plt.figure(1)
    for i in range(num_elements):
        plt.plot(timespan, array1_prescribed_data_log[i].theta * macros.R2D, label=f"Array 1 Element {i+1}")
    plt.xlabel("Time [min]")
    plt.ylabel("Angle Relative to Hub [deg]")
    plt.title("Array 1 Element Angle Relative to Hub")
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc="center left", fontsize=8)
    plt.grid(True)
    # Array 2 element angle relative to hub
    for i in range(num_elements):
        plt.plot(timespan, array2_prescribed_data_log[i].theta * macros.R2D, linestyle="--", label=f"Array 2 Element {i+1}")
    plt.xlabel("Time [min]")
    plt.ylabel("Angle Relative to Hub [deg]")
    plt.title("Array 2 Element Angle Relative to Hub")
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc="center left", fontsize=8)
    plt.grid(True)
    plt_name = filename + "_ArrayElementsTheta"
    figure_list[plt_name] = plt.figure(1)

    # Plot element displacements
    plt.figure(2)
    for i in range(num_elements):
        plt.plot(timespan, array1_trans_data_log_list[i].rho, label=f"Array1 Element {i+1}")
        plt.xlabel("Time [min]")
        plt.ylabel("rho [m]")
        plt.title("Element Translation Displacement vs Time")
        plt.legend(bbox_to_anchor=(1.25, 0.5), loc="center left", fontsize=8)
        plt.grid(True)
    for i in range(num_elements):
        plt.plot(timespan, array2_trans_data_log_list[i].rho, linestyle="--", label=f"Array2 Element {i+1}")
    plt.xlabel("Time [min]")
    plt.ylabel("rho [m]")
    plt.title("Element Translation Displacement vs Time")
    plt.legend(bbox_to_anchor=(1.25, 0.5), loc="center left", fontsize=8)
    plt.grid(True)
    plt_name = filename + "_ArrayElementsRho"
    figure_list[plt_name] = plt.figure(2)

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
    plt_name = filename + "_HubInertialMRPAttitude"
    figure_list[plt_name] = plt.figure(3)

    # Plot r_BN_N
    plt.figure(4)
    plt.plot(timespan, r_BN_N[:, 0], label="x")
    plt.plot(timespan, r_BN_N[:, 1], label="y")
    plt.plot(timespan, r_BN_N[:, 2], label="z")
    plt.xlabel("Time [min]")
    plt.ylabel("Displacement [m]")
    plt.title("Hub Inertial Displacement vs Time")
    plt.legend()
    plt.grid(True)
    plt_name = filename + "_HubInertialPosition"
    figure_list[plt_name] = plt.figure(4)

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
    plt_name = filename + "_HubInertialAngularVelocity"
    figure_list[plt_name] = plt.figure(5)

    if show_plots:
        plt.show()
    plt.close("all")

    return figure_list


if __name__ == "__main__":
    run(show_plots = True)
