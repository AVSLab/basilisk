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

r"""
Overview
--------

This scenario demonstrates how to configure specific rigid body component motion using the
:ref:`generalSingleBodyStateEffector` dynamics simulation module.

This example sets up a spacecraft inspired by the IXPE mission spacecraft. The spacecraft system contains four rigid
bodies: a cubic rigid hub, two 1-DOF solar panels, and a 1-DOF cylindrical payload. Initially at  rest, the spacecraft
first deploys both solar panels. After 5 minutes, the payload is deployed along the spacecraft z-axis following 1-DOF
screw motion. The payload translates 3 meters and completes 3.5 revolutions during its 3-minute deployment.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioGeneralSingleBodies.py

All component scalar displacements, velocities, and accelerations are plotted in this scenario, along with
the hub response to the deployment.

Illustration of Simulation Results
----------------------------------

.. image:: /_images/Scenarios/scenarioGeneralSingleBodies_1.svg
    :align: center

.. image:: /_images/Scenarios/scenarioGeneralSingleBodies_2.svg
    :align: center

.. image:: /_images/Scenarios/scenarioGeneralSingleBodies_3.svg
    :align: center

.. image:: /_images/Scenarios/scenarioGeneralSingleBodies_4.svg
    :align: center

.. image:: /_images/Scenarios/scenarioGeneralSingleBodies_5.svg
    :align: center

.. image:: /_images/Scenarios/scenarioGeneralSingleBodies_6.svg
    :align: center

.. image:: /_images/Scenarios/scenarioGeneralSingleBodies_7.svg
    :align: center

.. image:: /_images/Scenarios/scenarioGeneralSingleBodies_8.svg
    :align: center

"""

#
#   General Single Body State Effector Example
#   Author:             Leah Kiner
#   Creation Date:      June 4, 2026
#

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import generalSingleBodyStateEffector, spacecraft, prescribedRotation1DOF
from Basilisk.architecture import messaging
from Basilisk.utilities import vizSupport
from Basilisk.utilities import RigidBodyKinematics as rbk

filename = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))

matplotlib.rc('xtick', labelsize=14)
matplotlib.rc('ytick', labelsize=14)

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
    mass_hub = 130.0  # [kg]
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

    # Create the panels' rotational degree of freedom
    rotHat_S = np.array([0.0, 0.0, 1.0])
    theta_init_panel = 0.0 * macros.D2R
    theta_dot_init_panel = 0.0 * macros.D2R
    k_panel_1 = 10.0
    c_panel_1 = 1.0
    one_dof_rotation_panel_1 = generalSingleBodyStateEffector.DOF()
    one_dof_rotation_panel_1.setDOFAxis(rotHat_S)
    one_dof_rotation_panel_1.setBetaInit(theta_init_panel)
    one_dof_rotation_panel_1.setBetaDotInit(theta_dot_init_panel)
    one_dof_rotation_panel_1.setSpringConstantK(k_panel_1)
    one_dof_rotation_panel_1.setDampingConstantK(c_panel_1)
    k_panel_2 = 10.0
    c_panel_2 = 1.0
    one_dof_rotation_panel_2 = generalSingleBodyStateEffector.DOF()
    one_dof_rotation_panel_2.setDOFAxis(rotHat_S)
    one_dof_rotation_panel_2.setBetaInit(theta_init_panel)
    one_dof_rotation_panel_2.setBetaDotInit(theta_dot_init_panel)
    one_dof_rotation_panel_2.setSpringConstantK(k_panel_2)
    one_dof_rotation_panel_2.setDampingConstantK(c_panel_2)

    # Create the solar panels
    mass_panel = 10.0  # [kg]
    length_panel = 1.0  # [m]
    width_panel = 1.0  # [m]
    depth_panel = 0.05  # [m]
    I_panel_length = (1 / 12) * mass_panel * (width_panel * width_panel + depth_panel * depth_panel)  # [kg m^2]
    I_panel_width = (1 / 12) * mass_panel * (length_panel * length_panel + depth_panel * depth_panel)  # [kg m^2]
    I_panel_depth = (1 / 12) * mass_panel * (length_panel * length_panel + width_panel * width_panel)  # [kg m^2]
    I_panel_Sc_S = [[I_panel_length, 0.0, 0.0], [0.0, I_panel_width, 0.0], [0.0, 0.0, I_panel_depth]]  # [kg m^2]
    r_GcG_G = [[0.0], [-0.5], [0.0]]
    r_S10B_B = [[0.5], [0.5], [0.0]]
    r_S20B_B = [[-0.5], [0.5], [0.0]]
    dcm_S10B = np.array([[1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0],
                         [0.0, 0.0, 1.0]])
    dcm_S20B = np.array([[-1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0],
                         [0.0, 0.0, -1.0]])

    panel_1 = generalSingleBodyStateEffector.GeneralSingleBodyStateEffector()
    panel_1.ModelTag = "panel1"
    panel_1.setMass(mass_panel)
    panel_1.setIPntGc_G(I_panel_Sc_S)
    panel_1.setR_GcG_G(r_GcG_G)
    panel_1.setR_G0B_B(r_S10B_B)
    panel_1.setDCM_G0B(dcm_S10B)
    panel_1.addRotationalDOF(one_dof_rotation_panel_1)
    sc_object.addStateEffector(panel_1)
    sc_sim.AddModelToTask(dyn_task_name, panel_1)

    panel_2 = generalSingleBodyStateEffector.GeneralSingleBodyStateEffector()
    panel_2.ModelTag = "panel2"
    panel_2.setMass(mass_panel)
    panel_2.setIPntGc_G(I_panel_Sc_S)
    panel_2.setR_GcG_G(r_GcG_G)
    panel_2.setR_G0B_B(r_S20B_B)
    panel_2.setDCM_G0B(dcm_S20B)
    panel_2.addRotationalDOF(one_dof_rotation_panel_2)
    sc_object.addStateEffector(panel_2)
    sc_sim.AddModelToTask(dyn_task_name, panel_2)

    # Create the payload screw degree of freedom
    rotHat_P = np.array([0.0, 0.0, 1.0])
    theta_init_payload = 0.0 * macros.D2R
    theta_dot_init_payload = 0.0 * macros.D2R
    k = 100.0
    c = 0.0
    payload_theta_ref = 1260.0 * macros.D2R  # [rad]
    payload_rho_ref = 3.0  # [m]
    screw_constant = payload_rho_ref / payload_theta_ref
    one_dof_rot_screw = generalSingleBodyStateEffector.DOF()
    one_dof_rot_screw.setDOFAxis(rotHat_P)
    one_dof_rot_screw.setBetaInit(theta_init_payload)
    one_dof_rot_screw.setBetaDotInit(theta_dot_init_payload)
    one_dof_rot_screw.setSpringConstantK(k)
    one_dof_rot_screw.setDampingConstantK(c)
    one_dof_rot_screw.setScrewConstant(screw_constant)

    # Create the payload
    mass_payload = 170.0  # [kg]
    length_payload = 1.0  # [m]
    width_payload = 1.0  # [m]
    depth_payload = 1.0  # [m]
    I_payload_length = (1 / 12) * mass_payload * (width_payload * width_payload + depth_payload * depth_payload)  # [kg m^2]
    I_payload_width = (1 / 12) * mass_payload * (length_payload * length_payload + depth_payload * depth_payload)  # [kg m^2]
    I_payload_depth = (1 / 12) * mass_payload * (length_payload * length_payload + width_payload * width_payload)  # [kg m^2]
    I_payload_Pc_P = [[I_payload_length, 0.0, 0.0], [0.0, I_payload_width, 0.0], [0.0, 0.0, I_payload_depth]]  # [kg m^2]
    r_PcP_P = [[0.0], [0.0], [0.5]]
    r_P0B_B = [[0.0], [0.0], [0.5]]
    dcm_P0B = np.array([[1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0],
                         [0.0, 0.0, 1.0]])

    payload = generalSingleBodyStateEffector.GeneralSingleBodyStateEffector()
    payload.ModelTag = "payload"
    payload.setMass(mass_payload)
    payload.setIPntGc_G(I_payload_Pc_P)
    payload.setR_GcG_G(r_PcP_P)
    payload.setR_G0B_B(r_P0B_B)
    payload.setDCM_G0B(dcm_P0B)
    payload.addRotationalDOF(one_dof_rot_screw)
    sc_object.addStateEffector(payload)
    sc_sim.AddModelToTask(dyn_task_name, payload)

    # Create the reference profile for the solar panels
    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload(
        theta=90.0 * macros.D2R,  # [rad]
        thetaDot=0.0,  # [rad/s]
    )
    hingedRigidBodyMessage1 = messaging.HingedRigidBodyMsg().write(
        hingedRigidBodyMessageData
    )

    panel_profiler = prescribedRotation1DOF.PrescribedRotation1DOF()
    panel_profiler.ModelTag = "panelProfiler"
    panel_profiler.setThetaDDotMax(0.025 * macros.D2R)
    panel_profiler.setThetaInit(theta_init_panel)
    panel_profiler.setSmoothingDuration(10)
    sc_sim.AddModelToTask(dyn_task_name, panel_profiler)
    panel_profiler.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage1)
    panel_1.spinningBodyRefInMsg[0].subscribeTo(
        panel_profiler.spinningBodyOutMsg
    )
    panel_2.spinningBodyRefInMsg[0].subscribeTo(
        panel_profiler.spinningBodyOutMsg
    )

    # Create the reference profile for the payload
    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload(
        theta=theta_init_payload,  # [rad]
        thetaDot=0.0,  # [rad/s]
    )
    hingedRigidBodyMessage2 = messaging.HingedRigidBodyMsg().write(
        hingedRigidBodyMessageData
    )

    payload_profiler = prescribedRotation1DOF.PrescribedRotation1DOF()
    payload_profiler.ModelTag = "panelProfiler"
    payload_profiler.setThetaDDotMax(0.16 * macros.D2R)
    payload_profiler.setThetaInit(theta_init_payload)
    payload_profiler.setSmoothingDuration(10)
    payload_profiler.setScrewConstant(screw_constant)
    sc_sim.AddModelToTask(dyn_task_name, payload_profiler)
    payload_profiler.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage2)
    payload.spinningBodyRefInMsg[0].subscribeTo(
        payload_profiler.spinningBodyOutMsg
    )

    # Set up data logging
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    payload_state_data_log = payload.generalSingleBodyConfigLogOutMsg.recorder()
    panel_1_states_data_log = []
    panel_2_states_data_log = []
    payload_rot_states_data_log = []
    for outMsg in panel_1.spinningBodyOutMsgs:
        panel_1_states_data_log.append(outMsg.recorder())
        sc_sim.AddModelToTask(dyn_task_name, panel_1_states_data_log[-1])
    for outMsg in panel_2.spinningBodyOutMsgs:
        panel_2_states_data_log.append(outMsg.recorder())
        sc_sim.AddModelToTask(dyn_task_name, panel_2_states_data_log[-1])
    for outMsg in payload.spinningBodyOutMsgs:
        payload_rot_states_data_log.append(outMsg.recorder())
        sc_sim.AddModelToTask(dyn_task_name, payload_rot_states_data_log[-1])
    sc_sim.AddModelToTask(dyn_task_name, sc_state_data_log)
    sc_sim.AddModelToTask(dyn_task_name, payload_state_data_log)

    # Add Vizard
    sc_body_list = [sc_object]
    sc_body_list.append(["panel1", panel_1.generalSingleBodyConfigLogOutMsg])
    sc_body_list.append(["panel2", panel_2.generalSingleBodyConfigLogOutMsg])
    sc_body_list.append(["payload", payload.generalSingleBodyConfigLogOutMsg])
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(sc_sim, data_rec_task_name, sc_body_list,
                                                  saveFile=filename
                                                  )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[sc_object.ModelTag]
                                     , modelPath="CUBE"
                                     , scale=[width_hub, length_hub, 0.75 * depth_hub]
                                     , color=vizSupport.toRGBA255("gray"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["panel1"]
                                     , modelPath="CUBE"
                                     , scale=[depth_panel, length_panel, width_panel]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["panel2"]
                                     , modelPath="CUBE"
                                     , scale=[depth_panel, length_panel, width_panel]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["payload"]
                                     , modelPath="CYLINDER"
                                     , scale=[length_payload, width_payload, 0.5 * depth_payload]
                                     , color=vizSupport.toRGBA255("orange"))
        viz.settings.orbitLinesOn = -1

    # Run the first simulation chunk (solar panel deployment)
    sim_time_1 = 135.0  # [s]
    sc_sim.InitializeSimulation()
    sc_sim.ConfigureStopTime(macros.sec2nano(sim_time_1))
    sc_sim.ExecuteSimulation()

    # Create the reference profile for the payload
    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload(
        theta=payload_theta_ref,  # [rad]
        thetaDot=0.0,  # [rad/s]
    )
    hingedRigidBodyMessage2 = messaging.HingedRigidBodyMsg().write(
        hingedRigidBodyMessageData
    )
    payload_profiler.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage2)

    # Run the second simulation chunk (payload deployment)
    sim_time_2 = 195.0  # [s]
    sc_sim.ConfigureStopTime(macros.sec2nano(sim_time_1 + sim_time_2))
    sc_sim.ExecuteSimulation()

    # Extract logged data
    timespan = sc_state_data_log.times() * macros.NANO2SEC
    r_BN_N = sc_state_data_log.r_BN_N  # [m]
    v_BN_N = sc_state_data_log.v_BN_N  # [m/s]
    sigma_BN = sc_state_data_log.sigma_BN
    omega_BN_B = sc_state_data_log.omega_BN_B * macros.R2D  # [deg/s]
    panel_1_theta = []  # [deg]
    panel_2_theta = []  # [deg]
    payload_theta = []  # [deg]
    panel_1_theta_dot = []  # [deg/s]
    panel_2_theta_dot = []  # [deg/s]
    payload_theta_dot = []  # [deg/s]
    for data in panel_1_states_data_log:
        panel_1_theta.append(data.theta * macros.R2D)
        panel_1_theta_dot.append(data.thetaDot * macros.R2D)
    for data in panel_2_states_data_log:
        panel_2_theta.append(data.theta * macros.R2D)
        panel_2_theta_dot.append(data.thetaDot * macros.R2D)
    for data in payload_rot_states_data_log:
        payload_theta.append(data.theta * macros.R2D)
        payload_theta_dot.append(data.thetaDot * macros.R2D)

    # Compute payload displacement
    payload_rho = []  # [m]
    payload_rho_dot = []  # [m]
    r_PcP_P_vec = np.array(r_PcP_P).flatten()
    for i in range(len(payload_state_data_log.sigma_BN)):
        dcm_PN = rbk.MRP2C(payload_state_data_log.sigma_BN[i])

        r_PcN_N = payload_state_data_log.r_BN_N[i]
        r_PB_P = dcm_PN @ (r_PcN_N - r_BN_N[i]) - r_PcP_P_vec
        payload_rho.append(r_PB_P.dot(rotHat_P))

        v_PcN_N = payload_state_data_log.v_BN_N[i]
        v_PB_P = dcm_PN @ (v_PcN_N - v_BN_N[i])
        payload_rho_dot.append(v_PB_P.dot(rotHat_P))

    figure_list = {}
    plt.close("all")

    # Plot the solar panel angles and rates
    plt.figure(1)
    plt.clf()
    plt.plot(timespan, panel_1_theta[0], label=r'$\theta_1$', color="teal")
    plt.plot(timespan, panel_2_theta[0], label=r'$\theta_2$', color="darkviolet")
    # plt.title(r'Solar Panel Angles')
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc="center", prop={"size": 14})
    plt.grid(True)
    plt_name = filename + "_1"
    figure_list[plt_name] = plt.figure(1)

    plt.figure(2)
    plt.clf()
    plt.plot(timespan, panel_1_theta_dot[0], label=r'$\dot{\theta}_1$', color="teal")
    plt.plot(timespan, panel_2_theta_dot[0], label=r'$\dot{\theta}_2$', color="darkviolet")
    # plt.title(r'Solar Panel Angle Rates')
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc="center right", prop={"size": 14})
    plt.grid(True)
    plt_name = filename + "_2"
    figure_list[plt_name] = plt.figure(2)

    # Plot the payload displacements and rates
    fig3, ax1 = plt.subplots()
    ax1.plot(timespan, payload_theta[0], label=r"$\theta$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=14)
    ax1.set_ylabel("(deg)", color="teal", fontsize=14)
    ax2 = ax1.twinx()
    ax2.plot(timespan, payload_rho, label=r'$\rho$', color="darkviolet")
    ax2.set_ylabel("(m)", color="darkviolet", fontsize=14)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    # plt.title("Payload Displacements")
    plt.legend(handles=handles, labels=labels, loc="center left", prop={"size": 14})
    plt.grid(True)
    plt_name = filename + "_3"
    figure_list[plt_name] = plt.figure(3)

    fig4, ax1 = plt.subplots()
    ax1.plot(timespan, payload_theta_dot[0], label=r"$\dot{\theta}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=14)
    ax1.set_ylabel("(deg/s)", color="teal", fontsize=14)
    ax2 = ax1.twinx()
    ax2.plot(timespan, payload_rho_dot, label=r'$\dot{\rho}$', color="darkviolet")
    ax2.set_ylabel("(m/s)", color="darkviolet", fontsize=14)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    # plt.title("Payload Rates")
    plt.legend(handles=handles, labels=labels, loc="center left", prop={"size": 14})
    plt.grid(True)
    plt_name = filename + "_4"
    figure_list[plt_name] = plt.figure(4)

    # Plot hub inertial attitude
    plt.figure(5)
    plt.clf()
    plt.plot(timespan, sigma_BN[:, 0], label=r'$\sigma_1$', color="teal")
    plt.plot(timespan, sigma_BN[:, 1], label=r'$\sigma_2$', color="darkviolet")
    plt.plot(timespan, sigma_BN[:, 2], label=r'$\sigma_3$', color="blue")
    # plt.title(r'Hub Inertial Attitude $\sigma_{\mathcal{B}/\mathcal{N}}$')
    plt.ylabel('', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc="center left", prop={"size": 14})
    plt.grid(True)
    plt_name = filename + "_5"
    figure_list[plt_name] = plt.figure(5)

    # Plot hub inertial angular velocity
    plt.figure(6)
    plt.clf()
    plt.plot(timespan, omega_BN_B[:, 0], label=r'$\omega_1$', color="teal")
    plt.plot(timespan, omega_BN_B[:, 1], label=r'$\omega_2$', color="darkviolet")
    plt.plot(timespan, omega_BN_B[:, 2], label=r'$\omega_3$', color="blue")
    # plt.title(r'Hub Inertial Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$')
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc="center left", prop={"size": 14})
    plt.grid(True)
    plt_name = filename + "_6"
    figure_list[plt_name] = plt.figure(6)

    # Plot hub inertial position
    plt.figure(7)
    plt.clf()
    plt.plot(timespan, r_BN_N[:, 0], label=r'$r_1$', color="teal")
    plt.plot(timespan, r_BN_N[:, 1], label=r'$r_2$', color="darkviolet")
    plt.plot(timespan, r_BN_N[:, 2], label=r'$r_3$', color="blue")
    # plt.title(r'Hub Inertial Position ${}^\mathcal{N} r_{B/N}$')
    plt.ylabel('(m)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc="lower left", prop={"size": 14})
    plt.grid(True)
    plt_name = filename + "_7"
    figure_list[plt_name] = plt.figure(7)

    if show_plots:
        plt.show()
    plt.close("all")

    return figure_list


if __name__ == "__main__":
    run(True)
