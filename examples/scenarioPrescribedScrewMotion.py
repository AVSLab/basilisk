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

This scenario demonstrates how to configure prescribed helical screw motion using the
:ref:`prescribedRotation1DOF` kinematic profiler with :ref:`prescribedMotionStateEffector`.

This example configures a spacecraft containing a cylindrical rigid hub with two attached prescribed components.
The spacecraft is at rest at the start of the simulation. The prescribed components are given identical mass properties,
but each has a different hub-relative prescribed motion profile. The first prescribed component moves against the side
of the rigid hub along its third axis, while the second component moves away from the hub along it's second axis.
Both prescribed components complete two revolutions about their spin axes during the simulation.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioPrescribedScrewMotion.py

The scalar and vector prescribed displacements, velocities, and accelerations are plotted in this scenario, along with
the hub response to the prescribed screw motion.

Illustration of Simulation Results
----------------------------------

.. image:: /_images/Scenarios/scenarioPrescribedScrewMotion_1.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedScrewMotion_2.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedScrewMotion_3.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedScrewMotion_4.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedScrewMotion_5.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedScrewMotion_6.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedScrewMotion_7.svg
    :align: center

.. image:: /_images/Scenarios/scenarioPrescribedScrewMotion_8.svg
    :align: center

"""

#
#   Prescribed Screw Motion Example
#   Author:             Leah Kiner
#   Creation Date:      November 18, 2025
#

import matplotlib.pyplot as plt
import numpy as np
import os

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import prescribedMotionStateEffector, spacecraft
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.architecture import messaging
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
    depth_hub = 2.0  # [m]
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

    # Prescribed body parameters
    mass_prescribed = 20  # [kg]
    length_prescribed = 0.2  # [m]
    width_prescribed = 0.8  # [m]
    depth_prescribed = 0.2  # [m]
    I_prescribed_length = (1 / 12) * mass_prescribed * (width_prescribed * width_prescribed + depth_prescribed * depth_prescribed)  # [kg m^2]
    I_prescribed_width = (1 / 12) * mass_prescribed * (length_prescribed * length_prescribed + depth_prescribed * depth_prescribed)  # [kg m^2]
    I_prescribed_depth = (1 / 12) * mass_prescribed * (length_prescribed * length_prescribed + width_prescribed * width_prescribed)  # [kg m^2]
    I_prescribed_Pc1_P1 = [[I_prescribed_length, 0.0, 0.0], [0.0, I_prescribed_width, 0.0], [0.0, 0.0, I_prescribed_depth]]  # [kg m^2]
    I_prescribed_Pc2_P2 = [[I_prescribed_length, 0.0, 0.0], [0.0, I_prescribed_depth, 0.0], [0.0, 0.0, I_prescribed_width]]  # [kg m^2]
    prescribed_rot_axis_1_M = np.array([0.0, 1.0, 0.0])
    prescribed_rot_axis_2_M = np.array([0.0, 0.0, 1.0])
    init_displacement = 0.5 * width_prescribed  # [m]

    # Create the first prescribed body
    prescribed_body_1 = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribed_body_1.ModelTag = "prescribedBody1"
    prescribed_body_1.setMass(mass_prescribed)
    prescribed_body_1.setIPntPc_P(I_prescribed_Pc1_P1)
    prescribed_body_1.setR_MB_B([0.0, 0.5 * width_hub, 0.0])
    prescribed_body_1.setR_PcP_P([0.0, 0.0, 0.0])
    prescribed_body_1.setR_PM_M([0.0, init_displacement, 0.0])
    prescribed_body_1.setRPrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_body_1.setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_body_1.setOmega_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_body_1.setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_body_1.setSigma_PM([0.0, 0.0, 0.0])
    prescribed_body_1.setSigma_MB([0.0, 0.0, 0.0])
    sc_sim.AddModelToTask(dyn_task_name, prescribed_body_1)
    sc_object.addStateEffector(prescribed_body_1)

    # Create the second prescribed body
    prescribed_body_2 = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribed_body_2.ModelTag = "prescribedBody2"
    prescribed_body_2.setMass(mass_prescribed)
    prescribed_body_2.setIPntPc_P(I_prescribed_Pc2_P2)
    prescribed_body_2.setR_MB_B([0.65 * length_hub, 0.0, -0.5 * depth_hub])
    prescribed_body_2.setR_PcP_P([0.0, 0.0, 0.0])
    prescribed_body_2.setR_PM_M([0.0, 0.0, init_displacement])
    prescribed_body_2.setRPrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_body_2.setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_body_2.setOmega_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_body_2.setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_body_2.setSigma_PM([0.0, 0.0, 0.0])
    prescribed_body_2.setSigma_MB([0.0, 0.0, 0.0])
    sc_sim.AddModelToTask(dyn_task_name, prescribed_body_2)
    sc_object.addStateEffector(prescribed_body_2)

    # Prescribed motion rotational profiler parameters
    prescribed_theta_init = 0.0  # [rad]
    prescribed_ang_accel_max = 1.0 * macros.D2R  # [rad/s^2]
    prescribed_theta_ref = 720.0 * macros.D2R  # [rad]
    screw_constant_1 = 0.1
    screw_constant_2 = 0.12

    # Create the first rotational motion profiler
    one_dof_rotation_profiler_1 = prescribedRotation1DOF.PrescribedRotation1DOF()
    one_dof_rotation_profiler_1.ModelTag = "prescribedRotation1DOF1"
    one_dof_rotation_profiler_1.setRotHat_M(prescribed_rot_axis_1_M)
    one_dof_rotation_profiler_1.setThetaDDotMax(prescribed_ang_accel_max)
    one_dof_rotation_profiler_1.setThetaInit(prescribed_theta_init)
    one_dof_rotation_profiler_1.setCoastOptionBangDuration(0.75)
    one_dof_rotation_profiler_1.setSmoothingDuration(0.75)
    one_dof_rotation_profiler_1.setInitDisplacement(init_displacement)
    one_dof_rotation_profiler_1.setScrewConstant(screw_constant_1)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_rotation_profiler_1)

    # Create the second rotational motion profiler
    one_dof_rotation_profiler_2 = prescribedRotation1DOF.PrescribedRotation1DOF()
    one_dof_rotation_profiler_2.ModelTag = "prescribedRotation1DOF2"
    one_dof_rotation_profiler_2.setRotHat_M(prescribed_rot_axis_2_M)
    one_dof_rotation_profiler_2.setThetaDDotMax(prescribed_ang_accel_max)
    one_dof_rotation_profiler_2.setThetaInit(prescribed_theta_init)
    one_dof_rotation_profiler_2.setCoastOptionBangDuration(0.75)
    one_dof_rotation_profiler_2.setSmoothingDuration(0.75)
    one_dof_rotation_profiler_2.setInitDisplacement(init_displacement)
    one_dof_rotation_profiler_2.setScrewConstant(screw_constant_2)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_rotation_profiler_2)

    # Create the prescribed rotational motion reference message
    prescribed_rotation_msg_data = messaging.HingedRigidBodyMsgPayload()
    prescribed_rotation_msg_data.theta = prescribed_theta_ref
    prescribed_rotation_msg_data.thetaDot = 0.0
    prescribed_rotation_msg = messaging.HingedRigidBodyMsg().write(prescribed_rotation_msg_data)

    # Connect messages
    one_dof_rotation_profiler_1.spinningBodyInMsg.subscribeTo(prescribed_rotation_msg)
    one_dof_rotation_profiler_2.spinningBodyInMsg.subscribeTo(prescribed_rotation_msg)
    prescribed_body_1.prescribedRotationInMsg.subscribeTo(one_dof_rotation_profiler_1.prescribedRotationOutMsg)
    prescribed_body_1.prescribedTranslationInMsg.subscribeTo(one_dof_rotation_profiler_1.prescribedTranslationOutMsg)
    prescribed_body_2.prescribedRotationInMsg.subscribeTo(one_dof_rotation_profiler_2.prescribedRotationOutMsg)
    prescribed_body_2.prescribedTranslationInMsg.subscribeTo(one_dof_rotation_profiler_2.prescribedTranslationOutMsg)

    # Set up data logging
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    prescribed_translation_1_data_log = prescribed_body_1.prescribedTranslationOutMsg.recorder()
    prescribed_translation_2_data_log = prescribed_body_2.prescribedTranslationOutMsg.recorder()
    prescribed_rotation_1_data_log = prescribed_body_1.prescribedRotationOutMsg.recorder()
    prescribed_rotation_2_data_log = prescribed_body_2.prescribedRotationOutMsg.recorder()
    one_dof_rotation_profiler_1_data_log = one_dof_rotation_profiler_1.spinningBodyOutMsg.recorder()
    one_dof_rotation_profiler_2_data_log = one_dof_rotation_profiler_2.spinningBodyOutMsg.recorder()
    sc_sim.AddModelToTask(dyn_task_name, sc_state_data_log)
    sc_sim.AddModelToTask(dyn_task_name, prescribed_translation_1_data_log)
    sc_sim.AddModelToTask(dyn_task_name, prescribed_translation_2_data_log)
    sc_sim.AddModelToTask(dyn_task_name, prescribed_rotation_1_data_log)
    sc_sim.AddModelToTask(dyn_task_name, prescribed_rotation_2_data_log)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_rotation_profiler_1_data_log)
    sc_sim.AddModelToTask(dyn_task_name, one_dof_rotation_profiler_2_data_log)

    # Add Vizard
    sc_body_list = [sc_object]
    sc_body_list.append(["prescribedBody1", prescribed_body_1.prescribedMotionConfigLogOutMsg])
    sc_body_list.append(["prescribedBody2", prescribed_body_2.prescribedMotionConfigLogOutMsg])
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(sc_sim, data_rec_task_name, sc_body_list,
                                                  #saveFile=filename
                                                  )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[sc_object.ModelTag]
                                     , modelPath="CYLINDER"
                                     , scale=[width_hub, length_hub, 0.75 * depth_hub]
                                     , color=vizSupport.toRGBA255("gray"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["prescribedBody1"]
                                     , modelPath="CUBE"
                                     , scale=[length_prescribed, width_prescribed, depth_prescribed]
                                     , color=vizSupport.toRGBA255("green"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["prescribedBody2"]
                                     , modelPath="CUBE"
                                     , scale=[length_prescribed, depth_prescribed, width_prescribed]
                                     , color=vizSupport.toRGBA255("green"))

        viz.settings.orbitLinesOn = -1

    # Run the simulation
    sc_sim.InitializeSimulation()
    sim_time_1 = 600.0  # [s]
    sc_sim.ConfigureStopTime(macros.sec2nano(sim_time_1))
    sc_sim.ExecuteSimulation()

    # Extract the logged variables
    timespan = sc_state_data_log.times() * macros.NANO2SEC  # [s]
    r_BN_N = sc_state_data_log.r_BN_N  # [m]
    sigma_BN = sc_state_data_log.sigma_BN
    omega_BN_B = sc_state_data_log.omega_BN_B * macros.R2D  # [deg/s]
    r_PM_M_1 = prescribed_translation_1_data_log.r_PM_M  # [m]
    r_PM_M_2 = prescribed_translation_2_data_log.r_PM_M  # [m]
    rPrime_PM_M_1 = prescribed_translation_1_data_log.rPrime_PM_M  # [m/s]
    rPrime_PM_M_2 = prescribed_translation_2_data_log.rPrime_PM_M  # [m/s]
    rPrimePrime_PM_M_1 = prescribed_translation_1_data_log.rPrimePrime_PM_M  # [m/s^2]
    rPrimePrime_PM_M_2 = prescribed_translation_2_data_log.rPrimePrime_PM_M  # [m/s^2]
    prescribed_theta_1 = macros.R2D * one_dof_rotation_profiler_1_data_log.theta  # [deg]
    prescribed_theta_2 = macros.R2D * one_dof_rotation_profiler_2_data_log.theta  # [deg]
    omega_PM_P_1 = prescribed_rotation_1_data_log.omega_PM_P * macros.R2D  # [deg/s]
    omega_PM_P_2 = prescribed_rotation_2_data_log.omega_PM_P * macros.R2D  # [deg/s]
    omegaPrime_PM_P_1 = prescribed_rotation_1_data_log.omegaPrime_PM_P * macros.R2D  # [deg/s^2]
    omegaPrime_PM_P_2 = prescribed_rotation_2_data_log.omegaPrime_PM_P * macros.R2D  # [deg/s^2]

    figure_list = {}
    plt.close("all")

    # Plot prescribed angles and displacements
    prescribed_rho_1 = np.dot(r_PM_M_1, prescribed_rot_axis_1_M)  # [m]
    prescribed_rho_2 = np.dot(r_PM_M_2, prescribed_rot_axis_2_M)  # [m]
    prescribed_rho_1_truth = screw_constant_1 * macros.D2R * prescribed_theta_1 + init_displacement
    prescribed_rho_2_truth = screw_constant_2 * macros.D2R *  prescribed_theta_2 + init_displacement
    plt.figure(1)
    plt.clf()
    plt.plot(prescribed_theta_1[1:], prescribed_rho_1[1:], label=r'$P_1$ Sim', color="teal")
    plt.plot(prescribed_theta_1[1:], prescribed_rho_1_truth[1:], '--', label=r'$P_1$ Truth', color="teal")
    plt.plot(prescribed_theta_2[1:], prescribed_rho_2[1:], label=r'$P_2$ Sim', color="darkviolet")
    plt.plot(prescribed_theta_2[1:], prescribed_rho_2_truth[1:], '--', label=r'$P_2$ Truth', color="darkviolet")
    plt.title(r'Prescribed Displacements VS Angles')
    plt.ylabel('(m)')
    plt.xlabel('(deg)')
    plt.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    plt.grid(True)
    plt_name = filename + "_1"
    figure_list[plt_name] = plt.figure(1)

    # Plot difference between simulated displacements and truth values
    prescribed_rho_1_error = np.abs(prescribed_rho_1_truth[1:] - prescribed_rho_1[1:])
    prescribed_rho_2_error = np.abs(prescribed_rho_2_truth[1:] - prescribed_rho_2[1:])
    plt.figure(2)
    plt.clf()
    plt.plot(timespan[1:], prescribed_rho_1_error, label=r'$P_1$ Error', color="teal")
    plt.plot(timespan[1:], prescribed_rho_2_error, label=r'$P_1$ Error', color="darkviolet")
    plt.title(r'Prescribed Displacement Errors')
    plt.ylabel('(m)')
    plt.xlabel('(sec)')
    plt.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    plt.grid(True)
    plt_name = filename + "_2"
    figure_list[plt_name] = plt.figure(2)

    # Plot prescribed displacements
    prescribed_rho_1 = np.dot(r_PM_M_1, prescribed_rot_axis_1_M)  # [m]
    prescribed_rho_2 = np.dot(r_PM_M_2, prescribed_rot_axis_2_M)  # [m]
    fig1, ax1 = plt.subplots()
    ax1.plot(timespan, prescribed_theta_1, label=r"$\theta_{P_1}$", color="teal")
    ax1.plot(timespan, prescribed_theta_2, label=r"$\theta_{P_2}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("(deg)", color="teal")
    ax2 = ax1.twinx()
    ax2.plot(timespan[1:], 100 * prescribed_rho_1[1:], label=r'$\rho_{P_1}$', color="darkviolet")
    ax2.plot(timespan[1:], 100 * prescribed_rho_2[1:], label=r'$\rho_{P_2}$', color="darkviolet")
    ax2.set_ylabel("(cm)", color="darkviolet")
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.title("Prescribed Displacements")
    plt.legend(handles=handles, labels=labels, loc="center left", bbox_to_anchor=(1.25, 0.5))
    plt.grid(True)
    plt_name = filename + "_3"
    figure_list[plt_name] = plt.figure(3)

    # Plot prescribed velocities
    prescribed_theta_dot_1 = np.dot(omega_PM_P_1, prescribed_rot_axis_1_M)  # [deg/s]
    prescribed_theta_dot_2 = np.dot(omega_PM_P_2, prescribed_rot_axis_2_M)  # [deg/s]
    prescribed_rho_dot_1 = np.dot(rPrime_PM_M_1, prescribed_rot_axis_1_M)  # [m/s]
    prescribed_rho_dot_2 = np.dot(rPrime_PM_M_2, prescribed_rot_axis_2_M)  # [m/s]
    fig2, ax1 = plt.subplots()
    ax1.plot(timespan, prescribed_theta_dot_1, label=r"$\dot{\theta}_{P_1}$", color="teal")
    ax1.plot(timespan, prescribed_theta_dot_2, label=r"$\dot{\theta}_{P_2}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("(deg/s)", color="teal")
    ax2 = ax1.twinx()
    plt.plot(timespan, 100 * prescribed_rho_dot_1, label=r'$\dot{\rho}_{P_1}$', color="darkviolet")
    plt.plot(timespan, 100 * prescribed_rho_dot_2, label=r'$\dot{\rho}_{P_2}$', color="darkviolet")
    ax2.set_ylabel("(cm/s)", color="darkviolet")
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.title("Prescribed Velocities")
    plt.legend(handles=handles, labels=labels, loc="center left", bbox_to_anchor=(1.25, 0.5))
    plt.grid(True)
    plt_name = filename + "_4"
    figure_list[plt_name] = plt.figure(4)

    # Plot prescribed accelerations
    prescribed_theta_ddot_1 = np.dot(omegaPrime_PM_P_1, prescribed_rot_axis_1_M)  # [deg/s^2]
    prescribed_theta_ddot_2 = np.dot(omegaPrime_PM_P_2, prescribed_rot_axis_2_M)  # [deg/s^2]
    prescribed_rho_ddot_1 = np.dot(rPrimePrime_PM_M_1, prescribed_rot_axis_1_M)  # [m/s^2]
    prescribed_rho_ddot_2 = np.dot(rPrimePrime_PM_M_2, prescribed_rot_axis_2_M)  # [m/s^2]
    fig3, ax1 = plt.subplots()
    ax1.plot(timespan, prescribed_theta_ddot_1, label=r"$\ddot{\theta}_{P_1}$", color="teal")
    ax1.plot(timespan, prescribed_theta_ddot_2, label=r"$\ddot{\theta}_{P_2}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("(deg/$s^2$)", color="teal")
    ax2 = ax1.twinx()
    ax2.plot(timespan, 100.0 * prescribed_rho_ddot_1, label=r"$\ddot{\rho}_{P_1}$", color="darkviolet")
    ax2.plot(timespan, 100.0 * prescribed_rho_ddot_2, label=r"$\ddot{\rho}_{P_2}$", color="darkviolet")
    ax2.set_ylabel("(cm/$s^2$)", color="darkviolet")
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.title("Prescribed Accelerations")
    plt.legend(handles=handles, labels=labels, loc="center left", bbox_to_anchor=(1.25, 0.5))
    plt.grid(True)
    plt_name = filename + "_5"
    figure_list[plt_name] = plt.figure(5)

    # Plot hub inertial position
    plt.figure(6)
    plt.clf()
    plt.plot(timespan, r_BN_N[:, 0], label=r'$r_1$', color="teal")
    plt.plot(timespan, r_BN_N[:, 1], label=r'$r_2$', color="darkviolet")
    plt.plot(timespan, r_BN_N[:, 2], label=r'$r_3$', color="blue")
    plt.title(r'Hub Inertial Position ${}^\mathcal{N} r_{B/N}$')
    plt.ylabel('(m)')
    plt.xlabel('Time (s)')
    plt.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    plt.grid(True)
    plt_name = filename + "_6"
    figure_list[plt_name] = plt.figure(6)

    # Plot hub inertial attitude
    plt.figure(7)
    plt.clf()
    plt.plot(timespan, sigma_BN[:, 0], label=r'$\sigma_1$', color="teal")
    plt.plot(timespan, sigma_BN[:, 1], label=r'$\sigma_2$', color="darkviolet")
    plt.plot(timespan, sigma_BN[:, 2], label=r'$\sigma_3$', color="blue")
    plt.title(r'Hub Inertial Attitude $\sigma_{\mathcal{B}/\mathcal{N}}$')
    plt.ylabel('')
    plt.xlabel('Time (s)')
    plt.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    plt.grid(True)
    plt_name = filename + "_7"
    figure_list[plt_name] = plt.figure(7)

    # Plot hub inertial angular velocity
    plt.figure(8)
    plt.clf()
    plt.plot(timespan, omega_BN_B[:, 0], label=r'$\omega_1$', color="teal")
    plt.plot(timespan, omega_BN_B[:, 1], label=r'$\omega_2$', color="darkviolet")
    plt.plot(timespan, omega_BN_B[:, 2], label=r'$\omega_3$', color="blue")
    plt.title(r'Hub Inertial Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$')
    plt.ylabel('(deg/s)')
    plt.xlabel('Time (s)')
    plt.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    plt.grid(True)
    plt_name = filename + "_8"
    figure_list[plt_name] = plt.figure(8)

    if show_plots:
        plt.show()
    plt.close("all")

    return figure_list


if __name__ == "__main__":
    run(True)
