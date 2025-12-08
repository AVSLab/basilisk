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

r"""
Overview
--------

This scenario demonstrates the branching capability of the prescribed motion module using the
linearTranslationOneDOFStateEffector module. The prescribed motion module can be configured to have state effectors
attached to it rather than the spacecraft hub. To do so, the state effectors are set up normally, except their
parameters are set with respect to the prescribed motion effector rather than the hub. The parameters that were
previously required to be provided relative to the hub frame must instead be provided relative to the prescribed
motion frame.

This example configures a spacecraft lander concept, where the spacecraft system is comprised of
a small cylindrical hub, a prescribed motion platform attached underneath the hub, and three symmetrically
canted telescoping landing struts that are used as shock absorbers to assist the spacecraft in safely landing.
The landing struts are each canted by 60 degrees relative to the platform base (30 degrees relative to the
platform normal), and are evenly spaced by 120 degrees surrounding the center of the platform base. The stowed and
deployed lander configurations are shown below.

.. image:: /_images/static/scenarioPrescribedMotionWithTranslationBranching.svg
   :align: center

The spacecraft is at rest at the start of the simulation. The platform is actuated first, where it is commanded
to simultaneously translate and rotate downwards and away from primary hub structure to model
the initial phase of the lander deployment. Its reference translational and rotational displacements are chosen
arbitrarily as 10 degrees and 0.5 meters, respectively. Two smoothed bang-coast-bang acceleration profiles
are used to kinematically prescribe the required two-degree-of-freedom platform motion. After the platform finishes
actuating, the three landing struts are each commanded to deploy away from the platform along their translational
axes. Similarly, the struts are given arbitrary reference displacements of 0.2, 0.1, and 0.15 meters relative
to the platform.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioPrescribedMotionWithTranslationBranching.py

The prescribed platform motion, landing strut displacements and their rates, and the hub response to the lander
deployment is plotted in this scenario.

Illustration of Simulation Results
----------------------------------

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithTranslationBranching_1.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithTranslationBranching_2.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithTranslationBranching_3.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithTranslationBranching_4.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithTranslationBranching_5.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithTranslationBranching_6.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithTranslationBranching_7.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithTranslationBranching_8.svg
    :align: center

"""

#
#   Prescribed Motion with 1-DOF Linear Translation Branching Scenario
#   Author:             Leah Kiner
#   Creation Date:      October 1, 2025
#

import inspect
import matplotlib.pyplot as plt
import numpy as np
import os

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import prescribedMotionStateEffector, spacecraft, linearTranslationOneDOFStateEffector
from Basilisk.simulation import prescribedLinearTranslation
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.architecture import messaging
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import vizSupport

filename = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))

def run(show_plots):
    """
    The scenario can be run with the followings set up parameter:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    # Set up the simulation
    sc_sim = SimulationBaseClass.SimBaseClass()
    sim_process_name = "simProcess"
    sim_process = sc_sim.CreateNewProcess(sim_process_name)
    dyn_time_step_sec = 0.01  # [s]
    fsw_time_step_sec = 0.1  # [s]
    data_rec_time_step_sec = 0.1  # [s]
    dyn_task_name = "dynTask"
    data_rec_task_name = "dataRecTask"
    sim_process.addTask(sc_sim.CreateNewTask(dyn_task_name, macros.sec2nano(dyn_time_step_sec)))
    sim_process.addTask(sc_sim.CreateNewTask("fswTask", macros.sec2nano(fsw_time_step_sec)))
    sim_process.addTask(sc_sim.CreateNewTask(data_rec_task_name, macros.sec2nano(data_rec_time_step_sec)))

    # Create the spacecraft hub
    mass_hub = 1000.0  # [kg]
    length_hub = 1.0  # [m]
    width_hub = 1.0  # [m]
    depth_hub = 1.0  # [m]
    I_hub_11 = (1 / 12) * mass_hub * (length_hub * length_hub + depth_hub * depth_hub)  # [kg m^2]
    I_hub_22 = (1 / 12) * mass_hub * (length_hub * length_hub + width_hub * width_hub)  # [kg m^2]
    I_hub_33 = (1 / 12) * mass_hub * (width_hub * width_hub + depth_hub * depth_hub)  # [kg m^2]

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "scObject"
    sc_object.hub.mHub = mass_hub  # kg
    sc_object.hub.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
    sc_object.hub.IHubPntBc_B = [[I_hub_11, 0.0, 0.0], [0.0, I_hub_22, 0.0], [0.0, 0.0, I_hub_33]]  # [kg m^2] (Hub approximated as a cube)
    sc_object.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_sim.AddModelToTask(dyn_task_name, sc_object)

    # Prescribed platform parameters
    mass_prescribed = 200  # [kg]
    length_prescribed = 1.0  # [m]
    width_prescribed = 1.0  # [m]
    depth_prescribed = 0.2  # [m]
    I_prescribed_11 = (1 / 12) * mass_prescribed * (length_prescribed * length_prescribed + depth_prescribed * depth_prescribed)  # [kg m^2]
    I_prescribed_22 = (1 / 12) * mass_prescribed * (length_prescribed * length_prescribed + width_prescribed * width_prescribed)  # [kg m^2]
    I_prescribed_33 = (1 / 12) * mass_prescribed * (width_prescribed * width_prescribed + depth_prescribed * depth_prescribed)  # [kg m^2]
    I_prescribed_Pc_P = [[I_prescribed_11, 0.0, 0.0], [0.0, I_prescribed_22, 0.0], [0.0, 0.0,I_prescribed_33]]  # [kg m^2]
    prescribed_theta_init = 0.0
    prescribed_trans_axis_M = np.array([0.0, 0.0, -1.0])
    prescribed_rot_axis_M = np.array([0.0, 1.0, 0.0])
    prv_P0M = prescribed_theta_init * prescribed_rot_axis_M
    sigma_P0M = rbk.PRV2MRP(prv_P0M)

    # Create the prescribed platform
    prescribed_platform = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribed_platform.ModelTag = "prescribedPlatform"
    prescribed_platform.setMass(mass_prescribed)
    prescribed_platform.setIPntPc_P(I_prescribed_Pc_P)
    prescribed_platform.setR_MB_B([0.0, 0.0, -0.5 * depth_hub])
    prescribed_platform.setR_PcP_P([0.0, 0.0, 0.5 * depth_prescribed])
    prescribed_platform.setR_PM_M([0.0, 0.0, -depth_prescribed])
    prescribed_platform.setRPrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_platform.setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_platform.setOmega_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_platform.setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_platform.setSigma_PM(sigma_P0M)
    prescribed_platform.setSigma_MB([0.0, 0.0, 0.0])
    sc_sim.AddModelToTask(dyn_task_name, prescribed_platform)
    sc_object.addStateEffector(prescribed_platform)

    # Shared prescribed motion profiler parameters
    prescribed_ang_accel_max = 0.5 * macros.D2R  # [rad/s^2]
    prescribed_trans_accel_max = 0.0025  # [m/s^2]
    prescribed_theta_ref = 10.0 * macros.D2R  # [rad]
    prescribed_pos_ref = 0.5  # [m]

    # Create prescribed platform rotational motion profiler
    one_dof_rotation_profiler = prescribedRotation1DOF.PrescribedRotation1DOF()
    one_dof_rotation_profiler.ModelTag = "prescribedRotation1DOF"
    one_dof_rotation_profiler.setRotHat_M(prescribed_rot_axis_M)
    one_dof_rotation_profiler.setThetaDDotMax(prescribed_ang_accel_max)
    one_dof_rotation_profiler.setThetaInit(prescribed_theta_init)
    one_dof_rotation_profiler.setCoastOptionBangDuration(0.75)
    one_dof_rotation_profiler.setSmoothingDuration(0.75)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_rotation_profiler)

    # Create prescribed platform rotational motion reference message
    prescribed_rotation_msg_data = messaging.HingedRigidBodyMsgPayload()
    prescribed_rotation_msg_data.theta = prescribed_theta_ref
    prescribed_rotation_msg_data.thetaDot = 0.0
    prescribed_rotation_msg = messaging.HingedRigidBodyMsg().write(prescribed_rotation_msg_data)
    one_dof_rotation_profiler.spinningBodyInMsg.subscribeTo(prescribed_rotation_msg)
    prescribed_platform.prescribedRotationInMsg.subscribeTo(one_dof_rotation_profiler.prescribedRotationOutMsg)

    # Create prescribed platform translational motion profiler
    one_dof_translation_profiler = prescribedLinearTranslation.PrescribedLinearTranslation()
    one_dof_translation_profiler.ModelTag = "prescribedLinearTranslation"
    one_dof_translation_profiler.setTransHat_M(prescribed_trans_axis_M)
    one_dof_translation_profiler.setTransAccelMax(prescribed_trans_accel_max)
    one_dof_translation_profiler.setTransPosInit(depth_prescribed)
    one_dof_translation_profiler.setCoastOptionBangDuration(1.0)
    one_dof_translation_profiler.setSmoothingDuration(1.0)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_translation_profiler)

    # Create prescribed array boom translational motion reference message
    prescribed_translation_msg_data = messaging.LinearTranslationRigidBodyMsgPayload()
    prescribed_translation_msg_data.rho = prescribed_pos_ref
    prescribed_translation_msg_data.rhoDot = 0.0
    prescribed_translation_msg = messaging.LinearTranslationRigidBodyMsg().write(prescribed_translation_msg_data)
    one_dof_translation_profiler.linearTranslationRigidBodyInMsg.subscribeTo(prescribed_translation_msg)
    prescribed_platform.prescribedTranslationInMsg.subscribeTo(one_dof_translation_profiler.prescribedTranslationOutMsg)

    # Shared translating body parameters
    mass_teles = 50.0  # [kg]
    length_teles = 0.2  # [m]
    width_teles = 0.2  # [m]
    depth_teles = 0.5  # [m]
    I_teles_11 = (1 / 12) * mass_teles * (length_teles * length_teles + depth_teles * depth_teles)  # [kg m^2]
    I_teles_22 = (1 / 12) * mass_teles * (length_teles * length_teles + width_teles * width_teles)  # [kg m^2]
    I_teles_33 = (1 / 12) * mass_teles * (width_teles * width_teles + depth_teles * depth_teles)  # [kg m^2]
    I_teles_FcF = [[I_teles_11, 0.0, 0.0], [0.0, I_teles_22, 0.0], [0.0, 0.0, I_teles_33]]  # [kg m^2]
    teles_rho_init = 0.0
    teles_rho_dot_init = 0.0
    r_FcF_F = np.array([0.5 * depth_teles, 0.0, 0.0])
    telescoping_cant_angle = 60 * macros.D2R

    # Compute r_FP_P for each telescoping body
    spacing_distance = 0.25
    l = spacing_distance - 0.5 * width_teles * np.cos(telescoping_cant_angle)
    r_F01P_P = np.array([l * np.cos(30 * macros.D2R),
                         l * np.sin(30 * macros.D2R),
                         -(0.5 * width_teles * np.sin(telescoping_cant_angle))])
    r_F02P_P = np.array([0,
                         -l,
                         -(0.5 * width_teles * np.sin(telescoping_cant_angle))])
    r_F03P_P = np.array([-l * np.cos(30 * macros.D2R),
                         l * np.sin(30 * macros.D2R),
                         -(0.5 * width_teles * np.sin(telescoping_cant_angle))])

    # Compute dcm_FP for telescoping body 1
    theta_F1IntP = 30 * macros.D2R
    dcm_F1IntP = np.array([[np.cos(theta_F1IntP), np.sin(theta_F1IntP), 0.0],
                           [-np.sin(theta_F1IntP), np.cos(theta_F1IntP), 0.0],
                           [0.0, 0.0, 1.0]])
    theta_F1F1Int = telescoping_cant_angle
    dcm_F1F1Int = np.array([[np.cos(theta_F1F1Int), 0.0, -np.sin(theta_F1F1Int)],
                            [0.0, 1.0, 0.0],
                            [np.sin(theta_F1F1Int), 0.0, np.cos(theta_F1F1Int)]])
    dcm_F1P = dcm_F1F1Int @ dcm_F1IntP
    dcm_PF1 = dcm_F1P.T

    # Compute dcm_FP for telescoping body 2
    theta_F2IntP = -90 * macros.D2R
    dcm_F2IntP = np.array([[np.cos(theta_F2IntP), np.sin(theta_F2IntP), 0.0],
                           [-np.sin(theta_F2IntP), np.cos(theta_F2IntP), 0.0],
                           [0.0, 0.0, 1.0]])
    theta_F2F2Int = telescoping_cant_angle
    dcm_F2F2Int = np.array([[np.cos(theta_F2F2Int), 0.0, -np.sin(theta_F2F2Int)],
                            [0.0, 1.0, 0.0],
                            [np.sin(theta_F2F2Int), 0.0, np.cos(theta_F2F2Int)]])
    dcm_F2P = dcm_F2F2Int @ dcm_F2IntP
    dcm_PF2 = dcm_F2P.T

    # Compute dcm_FP for telescoping body 3
    theta_F3IntP = 150 * macros.D2R
    dcm_F3IntP = np.array([[np.cos(theta_F3IntP), np.sin(theta_F3IntP), 0.0],
                           [-np.sin(theta_F3IntP), np.cos(theta_F3IntP), 0.0],
                           [0.0, 0.0, 1.0]])
    theta_F3F3Int = telescoping_cant_angle
    dcm_F3F3Int = np.array([[np.cos(theta_F3F3Int), 0.0, -np.sin(theta_F3F3Int)],
                            [0.0, 1.0, 0.0],
                            [np.sin(theta_F3F3Int), 0.0, np.cos(theta_F3F3Int)]])
    dcm_F3P = dcm_F3F3Int @ dcm_F3IntP
    dcm_PF3 = dcm_F3P.T

    # Compute axis of translation for each telescoping body
    f_hat_F = np.array([1.0, 0.0, 0.0])
    f1_hat_P = dcm_PF1 @ f_hat_F
    f2_hat_P = dcm_PF2 @ f_hat_F
    f3_hat_P = dcm_PF3 @ f_hat_F

    # Create the translating bodies
    num_teles_struts = 3
    teles_strut_list = list()
    for idx in range(num_teles_struts):
        strut_num = idx + 1
        teles_strut_list.append(linearTranslationOneDOFStateEffector.LinearTranslationOneDOFStateEffector())
        teles_strut_list[idx].ModelTag = "translatingBody" + str(strut_num)
        teles_strut_list[idx].setMass(mass_teles)
        teles_strut_list[idx].setK(8)
        teles_strut_list[idx].setC(8)
        teles_strut_list[idx].setRhoInit(teles_rho_init)
        teles_strut_list[idx].setRhoDotInit(teles_rho_dot_init)
        teles_strut_list[idx].setR_FcF_F(r_FcF_F)
        teles_strut_list[idx].setIPntFc_F(I_teles_FcF)
        sc_sim.AddModelToTask(dyn_task_name, teles_strut_list[idx])

        # Connect the trusses to the prescribed platform
        prescribed_platform.addStateEffector(teles_strut_list[idx])

    teles_strut_list[0].setFHat_B(f1_hat_P)
    teles_strut_list[1].setFHat_B(f2_hat_P)
    teles_strut_list[2].setFHat_B(f3_hat_P)

    teles_strut_list[0].setR_F0B_B(r_F01P_P)
    teles_strut_list[1].setR_F0B_B(r_F02P_P)
    teles_strut_list[2].setR_F0B_B(r_F03P_P)

    teles_strut_list[0].setDCM_FB(dcm_F1P)
    teles_strut_list[1].setDCM_FB(dcm_F2P)
    teles_strut_list[2].setDCM_FB(dcm_F3P)

    # Set up data logging
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    prescribed_translation_data_log = prescribed_platform.prescribedTranslationOutMsg.recorder()
    prescribed_rotation_data_log = prescribed_platform.prescribedRotationOutMsg.recorder()
    one_dof_rotation_profiler_data_log = one_dof_rotation_profiler.spinningBodyOutMsg.recorder()
    teles_strut_1_data_log = teles_strut_list[0].translatingBodyOutMsg.recorder()
    teles_strut_2_data_log = teles_strut_list[1].translatingBodyOutMsg.recorder()
    teles_strut_3_data_log = teles_strut_list[2].translatingBodyOutMsg.recorder()
    sc_sim.AddModelToTask(dyn_task_name, sc_state_data_log)
    sc_sim.AddModelToTask(dyn_task_name, prescribed_translation_data_log)
    sc_sim.AddModelToTask(dyn_task_name, prescribed_rotation_data_log)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_rotation_profiler_data_log)
    sc_sim.AddModelToTask(dyn_task_name, teles_strut_1_data_log)
    sc_sim.AddModelToTask(dyn_task_name, teles_strut_2_data_log)
    sc_sim.AddModelToTask(dyn_task_name, teles_strut_3_data_log)

    # Add Vizard
    sc_body_list = [sc_object]
    sc_body_list.append(["prescribedPlatform", prescribed_platform.prescribedMotionConfigLogOutMsg])
    for idx in range(num_teles_struts):
        sc_body_list.append(["translatingBody" + str(idx + 1), teles_strut_list[idx].translatingBodyConfigLogOutMsg])

    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(sc_sim, data_rec_task_name, sc_body_list,
                                                  #saveFile=filename
                                                  )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[sc_object.ModelTag]
                                     , modelPath="CUBE"
                                     , scale=[width_hub, length_hub, depth_hub]
                                     , color=vizSupport.toRGBA255("gray"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["prescribedPlatform"]
                                     , modelPath="CUBE"
                                     , scale=[width_prescribed, length_prescribed, depth_prescribed]
                                     , color=vizSupport.toRGBA255("green"))
        for idx in range(num_teles_struts):
            vizSupport.createCustomModel(viz
                                         , simBodiesToModify=["translatingBody" + str(idx + 1)]
                                         , modelPath="CUBE"
                                         , scale=[depth_teles, length_teles, width_teles]
                                         , color=vizSupport.toRGBA255("purple"))
        viz.settings.orbitLinesOn = -1

    # Run the simulation (chunk 1 is only prescribed motion)
    sc_sim.InitializeSimulation()
    sim_time_1 = 90.0  # [s]
    sc_sim.ConfigureStopTime(macros.sec2nano(sim_time_1))
    sc_sim.ExecuteSimulation()

    # Create telescoping strut 1 reference message
    strut_1_rho_ref = 0.2  # [m]
    linear_translation_1_msg_data = messaging.LinearTranslationRigidBodyMsgPayload()
    linear_translation_1_msg_data.rho = strut_1_rho_ref
    linear_translation_1_msg_data.rhoDot = 0.0
    linear_translation_1_msg = messaging.LinearTranslationRigidBodyMsg().write(linear_translation_1_msg_data, macros.sec2nano(sim_time_1))
    teles_strut_list[0].translatingBodyRefInMsg.subscribeTo(linear_translation_1_msg)

    # Create telescoping strut 2 reference message
    strut_2_rho_ref = 0.1  # [m]
    linear_translation_2_msg_data = messaging.LinearTranslationRigidBodyMsgPayload()
    linear_translation_2_msg_data.rho = strut_2_rho_ref
    linear_translation_2_msg_data.rhoDot = 0.0
    linear_translation_2_msg = messaging.LinearTranslationRigidBodyMsg().write(linear_translation_2_msg_data, macros.sec2nano(sim_time_1))
    teles_strut_list[1].translatingBodyRefInMsg.subscribeTo(linear_translation_2_msg)

    # Create telescoping strut 3 reference message
    strut_3_rho_ref = 0.15  # [m]
    linear_translation_3_msg_data = messaging.LinearTranslationRigidBodyMsgPayload()
    linear_translation_3_msg_data.rho = strut_3_rho_ref
    linear_translation_3_msg_data.rhoDot = 0.0
    linear_translation_3_msg = messaging.LinearTranslationRigidBodyMsg().write(linear_translation_3_msg_data, macros.sec2nano(sim_time_1))
    teles_strut_list[2].translatingBodyRefInMsg.subscribeTo(linear_translation_3_msg)

    # Run the simulation (chunk 2 is the translating body motion only)
    sim_time_2 = 60.0  # [s]
    sc_sim.ConfigureStopTime(macros.sec2nano(sim_time_1 + sim_time_2))
    sc_sim.ExecuteSimulation()

    # Extract the logged variables
    timespan = sc_state_data_log.times() * macros.NANO2SEC  # [s]
    r_BN_N = sc_state_data_log.r_BN_N  # [m]
    sigma_BN = sc_state_data_log.sigma_BN
    omega_BN_B = sc_state_data_log.omega_BN_B * macros.R2D  # [deg/s]
    r_PM_M = prescribed_translation_data_log.r_PM_M  # [m]
    r_prime_PM_M = prescribed_translation_data_log.rPrime_PM_M  # [m/s]
    r_prime_prime_PM_M = prescribed_translation_data_log.rPrimePrime_PM_M  # [m/s^2]
    prescribed_theta = macros.R2D * one_dof_rotation_profiler_data_log.theta  # [deg]
    omega_PM_P = prescribed_rotation_data_log.omega_PM_P * macros.R2D  # [deg/s]
    omega_prime_PM_P = prescribed_rotation_data_log.omegaPrime_PM_P * macros.R2D  # [deg/s^2]
    teles_strut_1_rho = teles_strut_1_data_log.rho  # [m]
    teles_strut_2_rho = teles_strut_2_data_log.rho  # [m]
    teles_strut_3_rho = teles_strut_3_data_log.rho  # [m]
    teles_strut_1_rho_dot = teles_strut_1_data_log.rhoDot  # [m/s]
    teles_strut_2_rho_dot = teles_strut_2_data_log.rhoDot  # [m/s]
    teles_strut_3_rho_dot = teles_strut_3_data_log.rhoDot  # [m/s]

    figure_list = {}
    plt.close("all")

    # Plot prescribed platform displacements
    prescribed_rho = np.dot(r_PM_M, prescribed_trans_axis_M)  # [m]
    fig1, ax1 = plt.subplots()
    ax1.plot(timespan, prescribed_theta, label=r"$\theta_{P}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("(deg)", color="teal", fontsize=16)
    ax2 = ax1.twinx()
    ax2.plot(timespan[1:], 100 * prescribed_rho[1:], label=r'$\rho_{P}$', color="darkviolet")
    ax2.set_ylabel("(cm)", color="darkviolet", fontsize=16)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.legend(handles=handles, labels=labels, loc="center right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_1"
    figure_list[plt_name] = plt.figure(1)

    # Plot prescribed platform velocities
    prescribed_theta_dot = np.dot(omega_PM_P, prescribed_rot_axis_M)  # [deg/s]
    prescribed_rho_dot = np.dot(r_prime_PM_M, prescribed_trans_axis_M)  # [m/s]
    fig2, ax1 = plt.subplots()
    ax1.plot(timespan, prescribed_theta_dot, label=r"$\dot{\theta}_{P}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("(deg/s)", color="teal", fontsize=16)
    ax2 = ax1.twinx()
    plt.plot(timespan, 100 * prescribed_rho_dot, label=r'$\dot{\rho}_{P}$', color="darkviolet")
    ax2.set_ylabel("(cm/s)", color="darkviolet", fontsize=16)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.legend(handles=handles, labels=labels, loc="center right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_2"
    figure_list[plt_name] = plt.figure(2)

    # Plot prescribed platform accelerations
    prescribed_theta_ddot = np.dot(omega_prime_PM_P, prescribed_rot_axis_M)  # [deg/s^2]
    prescribed_rho_ddot = np.dot(r_prime_prime_PM_M, prescribed_trans_axis_M)  # [m/s^2]
    fig3, ax1 = plt.subplots()
    ax1.plot(timespan, prescribed_theta_ddot, label=r"$\ddot{\theta}_{P}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("(deg/$s^2$)", color="teal", fontsize=16)
    ax2 = ax1.twinx()
    ax2.plot(timespan, 100.0 * prescribed_rho_ddot, label=r"$\ddot{\rho}_{P}$", color="darkviolet")
    ax2.set_ylabel("(cm/$s^2$)", color="darkviolet", fontsize=16)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.legend(handles=handles, labels=labels, loc="upper right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_3"
    figure_list[plt_name] = plt.figure(3)

    # Plot telescoping strut displacements
    plt.figure(4)
    plt.clf()
    plt.plot(timespan, 100 * teles_strut_1_rho, label=r'$\rho_{T_1}$', color="teal")
    plt.plot(timespan, 100 * teles_strut_2_rho, label=r'$\rho_{T_2}$', color="darkviolet")
    plt.plot(timespan, 100 * teles_strut_3_rho, label=r'$\rho_{T_3}$', color="blue")
    plt.title(r'Telescoping Strut Displacements', fontsize=16)
    plt.ylabel('(cm)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc="center left", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_4"
    figure_list[plt_name] = plt.figure(4)

    # Plot telescoping strut displacement rates
    plt.figure(5)
    plt.clf()
    plt.plot(timespan, 100 * teles_strut_1_rho_dot, label=r'$\dot{\rho}_{T_1}$', color="teal")
    plt.plot(timespan, 100 * teles_strut_2_rho_dot, label=r'$\dot{\rho}_{T_2}$', color="darkviolet")
    plt.plot(timespan, 100 * teles_strut_3_rho_dot, label=r'$\dot{\rho}_{T_3}$', color="blue")
    plt.title(r'Telescoping Strut Displacement Rates', fontsize=16)
    plt.ylabel('(cm/s)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc="upper right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_5"
    figure_list[plt_name] = plt.figure(5)

    # Plot hub inertial position
    plt.figure(6)
    plt.clf()
    plt.plot(timespan, r_BN_N[:, 0], label=r'$r_1$', color="teal")
    plt.plot(timespan, r_BN_N[:, 1], label=r'$r_2$', color="darkviolet")
    plt.plot(timespan, r_BN_N[:, 2], label=r'$r_3$', color="blue")
    plt.title(r'Hub Inertial Position ${}^\mathcal{N} r_{B/N}$', fontsize=16)
    plt.ylabel('(m)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc="center right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_6"
    figure_list[plt_name] = plt.figure(6)

    # Plot hub inertial attitude
    plt.figure(7)
    plt.clf()
    plt.plot(timespan, sigma_BN[:, 0], label=r'$\sigma_1$', color="teal")
    plt.plot(timespan, sigma_BN[:, 1], label=r'$\sigma_2$', color="darkviolet")
    plt.plot(timespan, sigma_BN[:, 2], label=r'$\sigma_3$', color="blue")
    plt.title(r'Hub Inertial Attitude $\sigma_{\mathcal{B}/\mathcal{N}}$', fontsize=16)
    plt.ylabel('', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc="center right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_7"
    figure_list[plt_name] = plt.figure(7)

    # Plot hub inertial angular velocity
    plt.figure(8)
    plt.clf()
    plt.plot(timespan, omega_BN_B[:, 0], label=r'$\omega_1$', color="teal")
    plt.plot(timespan, omega_BN_B[:, 1], label=r'$\omega_2$', color="darkviolet")
    plt.plot(timespan, omega_BN_B[:, 2], label=r'$\omega_3$', color="blue")
    plt.title(r'Hub Inertial Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$', fontsize=16)
    plt.ylabel('(deg/s)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc="lower right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_8"
    figure_list[plt_name] = plt.figure(8)

    if show_plots:
        plt.show()
    plt.close("all")

    return figure_list


if __name__ == "__main__":
    run(True)
