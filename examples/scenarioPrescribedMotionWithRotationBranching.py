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
spinningBodyOneDOFStateEffector module. The prescribed motion module can be configured to have state effectors
attached to it rather than the spacecraft hub. To do so, the state effectors are set up normally, except their
parameters are set with respect to the prescribed motion effector rather than the hub. The parameters that were
previously required to be provided relative to the hub frame must instead be provided relative to the prescribed
motion frame.

This example configures an ISS-scale spacecraft containing a cylindrical rigid hub,
two large prescribed trusses which rotate about their longitudinal axes, and eight single-axis solar panels
which rotate about their transverse (bending) axes. The spacecraft configuration is shown below.

.. image:: /_images/static/scenarioPrescribedMotionWithRotationBranching.svg
   :align: center

The spacecraft is at rest at the start of the simulation.
To demonstrate the impact of the prescribed truss motion on the solar panel dynamics, all solar panels are
given zero initial deflections. The trusses are profiled to symmetrically rotate 45 degrees from their
initial configurations using a smoothed bang-coast-bang acceleration profile.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioPrescribedMotionWithRotationBranching.py

The prescribed truss motion, solar panel deflections and their rates, and the hub response to the panel deflections
is plotted in this scenario.

Illustration of Simulation Results
----------------------------------

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithRotationBranching_1.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithRotationBranching_2.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithRotationBranching_3.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithRotationBranching_4.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithRotationBranching_5.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithRotationBranching_6.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithRotationBranching_7.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedMotionWithRotationBranching_8.svg
    :align: center

"""

#
#   Prescribed Motion with 1-DOF Rotation Branching Scenario
#   Author:             Leah Kiner
#   Creation Date:      October 1, 2025
#

import inspect
import matplotlib.pyplot as plt
import numpy as np
import os

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import prescribedMotionStateEffector, spacecraft, spinningBodyOneDOFStateEffector
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
    mass_hub = 15000  # [kg]
    length_hub = 8  # [m]
    width_hub = 8  # [m]
    depth_hub = 20  # [m]
    I_hub_11 = (1 / 12) * mass_hub * (length_hub * length_hub + depth_hub * depth_hub)  # [kg m^2]
    I_hub_22 = (1 / 12) * mass_hub * (length_hub * length_hub + width_hub * width_hub)  # [kg m^2]
    I_hub_33 = (1 / 12) * mass_hub * (width_hub * width_hub + depth_hub * depth_hub)  # [kg m^2]

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "scObject"
    sc_object.hub.mHub = mass_hub  # kg
    sc_object.hub.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
    sc_object.hub.IHubPntBc_B = [[I_hub_11, 0.0, 0.0], [0.0, I_hub_22, 0.0], [0.0, 0.0, I_hub_33]]  # [kg m^2]
    sc_object.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_sim.AddModelToTask(dyn_task_name, sc_object)

    # Shared prescribed array boom parameters
    prescribed_pos_init = 0.0
    prescribed_theta_init = 0.0
    prescribed_trans_axis_M = np.array([1.0, 0.0, 0.0])
    prescribed_rot_axis_M = np.array([1.0, 0.0, 0.0])
    prv_P0M = prescribed_theta_init * prescribed_rot_axis_M
    sigma_P0M = rbk.PRV2MRP(prv_P0M)
    mass_prescribed = 6000.0  # [kg]
    length_prescribed = 50.0  # [m]
    width_prescribed = 4.0  # [m]
    depth_prescribed = 4.0  # [m]
    I_prescribed_11 = (1 / 12) * mass_prescribed * (length_prescribed * length_prescribed + depth_prescribed * depth_prescribed)  # [kg m^2]
    I_prescribed_22 = (1 / 12) * mass_prescribed * (length_prescribed * length_prescribed + width_prescribed * width_prescribed)  # [kg m^2]
    I_prescribed_33 = (1 / 12) * mass_prescribed * (width_prescribed * width_prescribed + depth_prescribed * depth_prescribed)  # [kg m^2]
    I_prescribed_Pc_P = [[I_prescribed_11, 0.0, 0.0], [0.0, I_prescribed_22, 0.0], [0.0, 0.0,I_prescribed_33]]  # [kg m^2]

    # Create prescribed array boom 1
    prescribed_truss_1 = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribed_truss_1.ModelTag = "prescribedTruss1"
    prescribed_truss_1.setMass(mass_prescribed)
    prescribed_truss_1.setIPntPc_P(I_prescribed_Pc_P)
    prescribed_truss_1.setR_MB_B([4.0, 0.0, 0.0])
    prescribed_truss_1.setR_PcP_P([25.0, 0.0, 0.0])
    prescribed_truss_1.setR_PM_M([0.0, 0.0, 0.0])
    prescribed_truss_1.setRPrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_truss_1.setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_truss_1.setOmega_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_truss_1.setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_truss_1.setSigma_PM(sigma_P0M)
    prescribed_truss_1.setSigma_MB([0.0, 0.0, 0.0])
    sc_sim.AddModelToTask(dyn_task_name, prescribed_truss_1)
    sc_object.addStateEffector(prescribed_truss_1)

    # Create prescribed array boom 2
    prescribed_truss_2 = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribed_truss_2.ModelTag = "prescribedTruss2"
    prescribed_truss_2.setMass(mass_prescribed)
    prescribed_truss_2.setIPntPc_P(I_prescribed_Pc_P)
    prescribed_truss_2.setR_MB_B([-4.0, 0.0, 0.0])
    prescribed_truss_2.setR_PcP_P([25.0, 0.0, 0.0])
    prescribed_truss_2.setR_PM_M([0.0, 0.0, 0.0])
    prescribed_truss_2.setRPrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_truss_2.setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_truss_2.setOmega_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_truss_2.setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_truss_2.setSigma_PM(sigma_P0M)
    dcm_MB = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
    prescribed_truss_2.setSigma_MB(rbk.C2MRP(dcm_MB))
    sc_sim.AddModelToTask(dyn_task_name, prescribed_truss_2)
    sc_object.addStateEffector(prescribed_truss_2)

    # Shared prescribed motion profiler parameters
    prescribed_ang_accel_max = 0.5 * macros.D2R  # [rad/s^2]
    prescribed_trans_accel_max = 0.005  # [m/s^2]
    prescribed_theta_ref = 45.0 * macros.D2R  # [rad]
    prescribed_pos_ref = 0.0  # [m]

    # Create prescribed array boom 1 rotational motion profiler
    one_dof_rotation_profiler_1 = prescribedRotation1DOF.PrescribedRotation1DOF()
    one_dof_rotation_profiler_1.ModelTag = "prescribedRotation1DOF_1"
    one_dof_rotation_profiler_1.setRotHat_M(prescribed_rot_axis_M)
    one_dof_rotation_profiler_1.setThetaDDotMax(prescribed_ang_accel_max)
    one_dof_rotation_profiler_1.setThetaInit(prescribed_theta_init)
    one_dof_rotation_profiler_1.setCoastOptionBangDuration(1.0)
    one_dof_rotation_profiler_1.setSmoothingDuration(1.0)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_rotation_profiler_1)

    # Create prescribed array boom 2 rotational motion profiler
    one_dof_rotation_profiler_2 = prescribedRotation1DOF.PrescribedRotation1DOF()
    one_dof_rotation_profiler_2.ModelTag = "prescribedRotation1DOF_2"
    one_dof_rotation_profiler_2.setRotHat_M(prescribed_rot_axis_M)
    one_dof_rotation_profiler_2.setThetaDDotMax(prescribed_ang_accel_max)
    one_dof_rotation_profiler_2.setThetaInit(prescribed_theta_init)
    one_dof_rotation_profiler_2.setCoastOptionBangDuration(1.0)
    one_dof_rotation_profiler_2.setSmoothingDuration(1.0)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_rotation_profiler_2)

    # Create prescribed array boom 1 rotational motion reference message
    prescribed_rotation_1_msg_data = messaging.HingedRigidBodyMsgPayload()
    prescribed_rotation_1_msg_data.theta = prescribed_theta_ref
    prescribed_rotation_1_msg_data.thetaDot = 0.0
    prescribed_rotation_1_msg = messaging.HingedRigidBodyMsg().write(prescribed_rotation_1_msg_data)
    one_dof_rotation_profiler_1.spinningBodyInMsg.subscribeTo(prescribed_rotation_1_msg)
    prescribed_truss_1.prescribedRotationInMsg.subscribeTo(one_dof_rotation_profiler_1.prescribedRotationOutMsg)

    # Create prescribed array boom 2 rotational motion reference message
    prescribed_rotation_2_msg_data = messaging.HingedRigidBodyMsgPayload()
    prescribed_rotation_2_msg_data.theta = - prescribed_theta_ref
    prescribed_rotation_2_msg_data.thetaDot = 0.0
    prescribed_rotation_2_msg = messaging.HingedRigidBodyMsg().write(prescribed_rotation_2_msg_data)
    one_dof_rotation_profiler_2.spinningBodyInMsg.subscribeTo(prescribed_rotation_2_msg)
    prescribed_truss_2.prescribedRotationInMsg.subscribeTo(one_dof_rotation_profiler_2.prescribedRotationOutMsg)

    # Create prescribed array boom 1 translational motion profiler
    one_dof_translation_profiler_1 = prescribedLinearTranslation.PrescribedLinearTranslation()
    one_dof_translation_profiler_1.ModelTag = "prescribedLinearTranslation_1"
    one_dof_translation_profiler_1.setTransHat_M(prescribed_trans_axis_M)
    one_dof_translation_profiler_1.setTransAccelMax(prescribed_trans_accel_max)
    one_dof_translation_profiler_1.setTransPosInit(prescribed_pos_init)
    one_dof_translation_profiler_1.setCoastOptionBangDuration(1.0)
    one_dof_translation_profiler_1.setSmoothingDuration(1.0)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_translation_profiler_1)

    # Create prescribed array boom 2 translational motion profiler
    one_dof_translation_profiler_2 = prescribedLinearTranslation.PrescribedLinearTranslation()
    one_dof_translation_profiler_2.ModelTag = "prescribedLinearTranslation_2"
    one_dof_translation_profiler_2.setTransHat_M(prescribed_trans_axis_M)
    one_dof_translation_profiler_2.setTransAccelMax(prescribed_trans_accel_max)
    one_dof_translation_profiler_2.setTransPosInit(prescribed_pos_init)
    one_dof_translation_profiler_2.setCoastOptionBangDuration(1.0)
    one_dof_translation_profiler_2.setSmoothingDuration(1.0)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_translation_profiler_2)

    # Create prescribed array boom translational motion reference message
    prescribed_translation_msg_data = messaging.LinearTranslationRigidBodyMsgPayload()
    prescribed_translation_msg_data.rho = prescribed_pos_ref
    prescribed_translation_msg_data.rhoDot = 0.0
    prescribed_translation_msg = messaging.LinearTranslationRigidBodyMsg().write(prescribed_translation_msg_data)
    one_dof_translation_profiler_1.linearTranslationRigidBodyInMsg.subscribeTo(prescribed_translation_msg)
    one_dof_translation_profiler_2.linearTranslationRigidBodyInMsg.subscribeTo(prescribed_translation_msg)
    prescribed_truss_1.prescribedTranslationInMsg.subscribeTo(one_dof_translation_profiler_1.prescribedTranslationOutMsg)
    prescribed_truss_2.prescribedTranslationInMsg.subscribeTo(one_dof_translation_profiler_2.prescribedTranslationOutMsg)

    # Shared spinning body parameters
    mass_panel = 1000.0  # [kg]
    length_panel = 10.0  # [m]
    width_panel = 0.3  # [m]
    depth_panel = 30.0  # [m]
    I_panel_11 = (1 / 12) * mass_panel * (length_panel * length_panel + depth_panel * depth_panel)  # [kg m^2]
    I_panel_22 = (1 / 12) * mass_panel * (length_panel * length_panel + width_panel * width_panel)  # [kg m^2]
    I_panel_33 = (1 / 12) * mass_panel * (width_prescribed * width_prescribed + depth_panel * depth_panel)  # [kg m^2]
    I_panel_ScS = [[I_panel_11, 0.0, 0.0], [0.0, I_panel_22, 0.0], [0.0, 0.0, I_panel_33]]  # [kg m^2]
    r_ScS_S = [[0.0], [0.0], [15.0]]
    panel_s_hat_S = [[1], [0], [0]]
    k = 700000.0
    c = 50000.0

    r_S1B_B = [[50.0], [0.0], [2.0]]  # [m] r_S1P_P
    r_S2B_B = [[35.0], [0.0], [2.0]]  # [m] r_S2P_P
    r_S3B_B = [[50.0], [0.0], [-2.0]]  # [m] r_S3P_P
    r_S4B_B = [[35.0], [0.0], [-2.0]]  # [m] r_S4P_P
    r_S5B_B = [[35.0], [0.0], [-2.0]]  # [m] r_S5P_P
    r_S6B_B = [[50.0], [0.0], [-2.0]]  # [m] r_S6P_P
    r_S7B_B = [[35.0], [0.0], [2.0]]  # [m] r_S7P_P
    r_S8B_B = [[50.0], [0.0], [2.0]]  # [m] r_S8P_P

    # Create the spinning bodies
    num_panels = 8
    spinning_panel_list = list()
    for idx in range(num_panels):
        panel_num = idx + 1
        spinning_panel_list.append(spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector())
        spinning_panel_list[idx].ModelTag = "spinningBody" + str(panel_num)
        spinning_panel_list[idx].mass = mass_panel
        spinning_panel_list[idx].IPntSc_S = I_panel_ScS
        spinning_panel_list[idx].r_ScS_S = r_ScS_S
        spinning_panel_list[idx].sHat_S = panel_s_hat_S
        spinning_panel_list[idx].k = k
        spinning_panel_list[idx].c = c

        if panel_num == 1 or panel_num == 2 or panel_num == 7 or panel_num == 8:
            spinning_panel_list[idx].dcm_S0B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        else:
            spinning_panel_list[idx].dcm_S0B = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]

        sc_sim.AddModelToTask(dyn_task_name, spinning_panel_list[idx])

    spinning_panel_list[0].r_SB_B = r_S1B_B
    spinning_panel_list[1].r_SB_B = r_S2B_B
    spinning_panel_list[2].r_SB_B = r_S3B_B
    spinning_panel_list[3].r_SB_B = r_S4B_B
    spinning_panel_list[4].r_SB_B = r_S5B_B
    spinning_panel_list[5].r_SB_B = r_S6B_B
    spinning_panel_list[6].r_SB_B = r_S7B_B
    spinning_panel_list[7].r_SB_B = r_S8B_B

    # Connect the solar panels to the correct prescribed truss
    prescribed_truss_1.addStateEffector(spinning_panel_list[0])
    prescribed_truss_1.addStateEffector(spinning_panel_list[1])
    prescribed_truss_1.addStateEffector(spinning_panel_list[2])
    prescribed_truss_1.addStateEffector(spinning_panel_list[3])
    prescribed_truss_2.addStateEffector(spinning_panel_list[4])
    prescribed_truss_2.addStateEffector(spinning_panel_list[5])
    prescribed_truss_2.addStateEffector(spinning_panel_list[6])
    prescribed_truss_2.addStateEffector(spinning_panel_list[7])

    # Set up data logging
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    one_dof_rotation_profiler_1_data_log = one_dof_rotation_profiler_1.spinningBodyOutMsg.recorder()
    one_dof_rotation_profiler_2_data_log = one_dof_rotation_profiler_2.spinningBodyOutMsg.recorder()
    prescribed_rotation_1_data_log = prescribed_truss_1.prescribedRotationOutMsg.recorder()
    prescribed_rotation_2_data_log = prescribed_truss_2.prescribedRotationOutMsg.recorder()
    spinning_panel_1_data_log = spinning_panel_list[0].spinningBodyOutMsg.recorder()
    spinning_panel_3_data_log = spinning_panel_list[2].spinningBodyOutMsg.recorder()
    sc_sim.AddModelToTask(data_rec_task_name, sc_state_data_log)
    sc_sim.AddModelToTask(data_rec_task_name, one_dof_rotation_profiler_1_data_log)
    sc_sim.AddModelToTask(data_rec_task_name, one_dof_rotation_profiler_2_data_log)
    sc_sim.AddModelToTask(data_rec_task_name, prescribed_rotation_1_data_log)
    sc_sim.AddModelToTask(data_rec_task_name, prescribed_rotation_2_data_log)
    sc_sim.AddModelToTask(data_rec_task_name, spinning_panel_1_data_log)
    sc_sim.AddModelToTask(data_rec_task_name, spinning_panel_3_data_log)

    # Add Vizard
    sc_body_list = [sc_object]
    sc_body_list.append(["prescribedTruss1", prescribed_truss_1.prescribedMotionConfigLogOutMsg])
    sc_body_list.append(["prescribedTruss2", prescribed_truss_2.prescribedMotionConfigLogOutMsg])
    for idx in range(num_panels):
        sc_body_list.append(["spinningBody" + str(idx + 1), spinning_panel_list[idx].spinningBodyConfigLogOutMsg])

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
                                     , simBodiesToModify=["prescribedTruss1"]
                                     , modelPath="CUBE"
                                     , scale=[length_prescribed, width_prescribed, depth_prescribed]
                                     , color=vizSupport.toRGBA255("green"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["prescribedTruss2"]
                                     , modelPath="CUBE"
                                     , scale=[length_prescribed, width_prescribed, depth_prescribed]
                                     , color=vizSupport.toRGBA255("green"))
        for idx in range(num_panels):
            vizSupport.createCustomModel(viz
                                         , simBodiesToModify=["spinningBody" + str(idx + 1)]
                                         , modelPath="CUBE"
                                         , scale=[length_panel, width_panel, depth_panel]
                                         , color=vizSupport.toRGBA255("blue"))
        viz.settings.orbitLinesOn = -1

    # Run the simulation
    sc_sim.InitializeSimulation()
    sim_time = 90.0  # [s]
    sc_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    sc_sim.ExecuteSimulation()

    # Extract the logged variables
    timespan = sc_state_data_log.times() * macros.NANO2SEC  # [s]
    r_BN_N = sc_state_data_log.r_BN_N  # [m]
    sigma_BN = sc_state_data_log.sigma_BN
    omega_BN_B = sc_state_data_log.omega_BN_B * macros.R2D  # [deg/s]
    prescribed_theta_1 = one_dof_rotation_profiler_1_data_log.theta * macros.R2D  # [deg]
    prescribed_theta_2 = one_dof_rotation_profiler_2_data_log.theta * macros.R2D  # [deg]
    omega_P1M_P1 = prescribed_rotation_1_data_log.omega_PM_P * macros.R2D  # [deg/s]
    omega_P2M_P2 = prescribed_rotation_2_data_log.omega_PM_P * macros.R2D  # [deg/s]
    omega_prime_P1M_P1 = prescribed_rotation_1_data_log.omegaPrime_PM_P * macros.R2D  # [deg/s^2]
    omega_prime_P2M_P2 = prescribed_rotation_2_data_log.omegaPrime_PM_P * macros.R2D  # [deg/s^2]
    spinning_panel_1_theta = spinning_panel_1_data_log.theta * macros.R2D  # [deg]
    spinning_panel_3_theta = spinning_panel_3_data_log.theta * macros.R2D  # [deg]
    spinning_panel_1_theta_dot = spinning_panel_1_data_log.thetaDot * macros.R2D  # [deg]
    spinning_panel_3_theta_dot = spinning_panel_3_data_log.thetaDot * macros.R2D  # [deg]

    figure_list = {}
    plt.close("all")

    # Plot prescribed truss angles
    plt.figure(1)
    plt.clf()
    plt.plot(timespan, prescribed_theta_1, label=r'$\theta_{P_1}$', color="teal")
    plt.plot(timespan, prescribed_theta_2, label=r'$\theta_{P_2}$', color="darkviolet")
    plt.title(r'Prescribed Truss Angles', fontsize=16)
    plt.ylabel('(deg)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc="center right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_1"
    figure_list[plt_name] = plt.figure(1)

    # Plot prescribed truss angle rates
    prescribed_theta_dot_1 = np.dot(omega_P1M_P1, prescribed_rot_axis_M)  # [deg/s]
    prescribed_theta_dot_2 = np.dot(omega_P2M_P2, prescribed_rot_axis_M)  # [deg/s]

    plt.figure(2)
    plt.clf()
    plt.plot(timespan, prescribed_theta_dot_1, label=r'$\dot{\theta}_{P_1}$', color="teal")
    plt.plot(timespan, prescribed_theta_dot_2, label=r'$\dot{\theta}_{P_2}$', color="darkviolet")
    plt.title(r'Prescribed Truss Angle Rates', fontsize=16)
    plt.ylabel('(deg/s)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc="upper right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_2"
    figure_list[plt_name] = plt.figure(2)

    # Plot prescribed truss accelerations
    prescribed_theta_ddot_1 = np.dot(omega_prime_P1M_P1, prescribed_rot_axis_M)  # [deg/s^2]
    prescribed_theta_ddot_2 = np.dot(omega_prime_P2M_P2, prescribed_rot_axis_M)  # [deg/s^2]

    plt.figure(3)
    plt.clf()
    plt.plot(timespan, prescribed_theta_ddot_1, label=r'$\ddot{\theta}_{P_1}$', color="teal")
    plt.plot(timespan, prescribed_theta_ddot_2, label=r'$\ddot{\theta}_{P_2}$', color="darkviolet")
    plt.title(r'Prescribed Truss Accelerations', fontsize=16)
    plt.ylabel('(deg/s$^2$)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc="upper right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_3"
    figure_list[plt_name] = plt.figure(3)

    # Plot spinning body solar panel angles
    plt.figure(4)
    plt.clf()
    plt.plot(timespan, spinning_panel_1_theta, label=r'$\theta_{S_{1,2,5,6}}$', color="teal")
    plt.plot(timespan, spinning_panel_3_theta, label=r'$\theta_{S_{3,4,7,8}}$', color="darkviolet")
    plt.title(r'Solar Array Angles', fontsize=16)
    plt.ylabel('(deg)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc="upper right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_4"
    figure_list[plt_name] = plt.figure(4)

    # Plot spinning body solar panel angle rates
    plt.figure(5)
    plt.clf()
    plt.plot(timespan, spinning_panel_1_theta_dot, label=r'$\dot{\theta}_{S_{1,2,5,6}}$', color="teal")
    plt.plot(timespan, spinning_panel_3_theta_dot, label=r'$\dot{\theta}_{S_{3,4,7,8}}$', color="darkviolet")
    plt.title(r'Solar Array Angle Rates', fontsize=16)
    plt.ylabel('(deg/s)', fontsize=16)
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
    plt.legend(loc="center left", prop={"size": 16})
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
    plt.legend(loc="center right", prop={"size": 16})
    plt.grid(True)
    plt_name = filename + "_8"
    figure_list[plt_name] = plt.figure(8)

    if show_plots:
        plt.show()
    plt.close("all")

    return figure_list


if __name__ == "__main__":
    run(True)
