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

import inspect
import os

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import prescribedLinearTranslation
from Basilisk.simulation import prescribedMotionStateEffector
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.simulation import spacecraft
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

matplotlib.rc('xtick', labelsize=16)
matplotlib.rc('ytick', labelsize=16)


@pytest.mark.parametrize("theta_init", [0.0, macros.D2R * 10.0, macros.D2R * -5.0])
@pytest.mark.parametrize("theta_ref", [0.0, macros.D2R * 5.0])
@pytest.mark.parametrize("pos_init", [0.0, 0.1, -0.1])
@pytest.mark.parametrize("pos_ref", [0.0, 0.05])
def test_prescribed_motion_state_effector(show_plots,
                                          theta_init,
                                          theta_ref,
                                          pos_init,
                                          pos_ref):
    r"""
    **Verification Test Description**

    The unit test for this module is an integrated test with two prescribed motion kinematic profiler modules.
    This is required because the prescribed motion module does not define or integrate states for the prescribed body.
    Both the :ref:`prescribedRotation1DOF` and :ref:`prescribedLinearTranslation` modules are configured so that
    the prescribed body simultaneously translates and rotates relative to the spacecraft hub. The initial attitude
    and displacement of the prescribed body are varied relative to the hub.

    The test checks that the conservation quantities of the spacecraft orbital energy, orbital angular momentum,
    and rotational angular momentum remain constant over the duration of the simulation.

    **Test Parameters**

    Args:
        theta_init (float): [rad] Initial PRV angle of the P frame with respect to the M frame
        theta_ref (float): [rad] Reference PRV angle of the P frame with respect to the M frame
        pos_init (float): [m] Initial displacement of the P frame with respect to the M frame
        pos_ref (float): [m] Reference displacement of the P frame with respect to the M frame

    **Description of Variables Being Tested**

    The test checks that the conservation quantities of the spacecraft orbital energy, orbital angular momentum,
    and rotational angular momentum remain constant over the duration of the simulation.
    """

    task_name = "unitTask"
    process_name = "testProcess"
    sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.001)
    process = sim.CreateNewProcess(process_name)
    process.addTask(sim.CreateNewTask(task_name, process_rate))

    # Hub mass properties
    mass_hub = 800  # [kg]
    length_hub = 1.0  # [m]
    width_hub = 1.0  # [m]
    depth_hub = 1.0  # [m]
    I_hub_11 = (1 / 12) * mass_hub * (length_hub * length_hub + depth_hub * depth_hub)  # [kg m^2]
    I_hub_22 = (1 / 12) * mass_hub * (length_hub * length_hub + width_hub * width_hub)  # [kg m^2]
    I_hub_33 = (1 / 12) * mass_hub * (width_hub * width_hub + depth_hub * depth_hub)  # [kg m^2]

    # Create the spacecraft
    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "scObject"
    sc_object.hub.mHub = mass_hub
    sc_object.hub.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
    sc_object.hub.IHubPntBc_B = [[I_hub_11, 0.0, 0.0],
                                 [0.0, I_hub_22, 0.0],
                                 [0.0, 0.0, I_hub_33]]  # [kg m^2]
    sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    sc_object.hub.omega_BN_BInit = [[0.01], [-0.01], [0.01]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sim.AddModelToTask(task_name, sc_object)

    # Define the prescribed motion state effector properties
    trans_axis_M = np.array([1.0, 0.0, 0.0])
    rot_axis_M = np.array([1.0, 0.0, 0.0])
    prv_P0M = theta_init * rot_axis_M
    sigma_P0M = rbk.PRV2MRP(prv_P0M)

    mass_prescribed = 100  # [kg]
    length_prescribed = 1.0  # [m]
    width_prescribed = 1.0  # [m]
    depth_prescribed = 1.0  # [m]
    I_prescribed_11 = (1 / 12) * mass_prescribed * (length_prescribed
                                                                      * length_prescribed
                                                                      + depth_prescribed
                                                                      * depth_prescribed)  # [kg m^2]
    I_prescribed_22 = (1 / 12) * mass_prescribed * (length_prescribed
                                                                      * length_prescribed
                                                                      + width_prescribed
                                                                      * width_prescribed)  # [kg m^2]
    I_prescribed_33 = (1 / 12) * mass_prescribed * (width_prescribed
                                                                      * width_prescribed
                                                                      + depth_prescribed
                                                                      * depth_prescribed)  # [kg m^2]
    I_prescribed_Pc_P = [[I_prescribed_11, 0.0, 0.0],
                         [0.0, I_prescribed_22, 0.0],
                         [0.0, 0.0, I_prescribed_33]]  # [kg m^2]

    # Create the prescribed motion state effector
    prescribed_body = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribed_body.ModelTag = "prescribedMotion"
    prescribed_body.setMass(mass_prescribed)
    prescribed_body.setIPntPc_P(I_prescribed_Pc_P)
    prescribed_body.setR_MB_B([0.5, 0.0, 0.0])
    prescribed_body.setR_PcP_P([0.5, 0.0, 0.0])
    prescribed_body.setR_PM_M([0.0, 0.0, 0.0])
    prescribed_body.setRPrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setOmega_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setSigma_PM(sigma_P0M)
    prescribed_body.setSigma_MB([0.0, 0.0, 0.0])
    sc_object.addStateEffector(prescribed_body)
    sim.AddModelToTask(task_name, prescribed_body)

    # Create the prescribed rotational motion profiler
    ang_accel_max = 0.5 * macros.D2R  # [rad/s^2]
    one_dof_rotation_profiler = prescribedRotation1DOF.PrescribedRotation1DOF()
    one_dof_rotation_profiler.ModelTag = "prescribedRotation1DOF"
    one_dof_rotation_profiler.setRotHat_M(rot_axis_M)
    one_dof_rotation_profiler.setThetaDDotMax(ang_accel_max)
    one_dof_rotation_profiler.setThetaInit(theta_init)
    one_dof_rotation_profiler.setCoastOptionBangDuration(1.0)
    one_dof_rotation_profiler.setSmoothingDuration(1.0)
    sim.AddModelToTask(task_name, one_dof_rotation_profiler)

    # Create the prescribed rotational motion reference message
    prescribed_rotation_msg_data = messaging.HingedRigidBodyMsgPayload()
    prescribed_rotation_msg_data.theta = theta_ref
    prescribed_rotation_msg_data.thetaDot = 0.0
    prescribed_rotation_msg = messaging.HingedRigidBodyMsg().write(prescribed_rotation_msg_data)
    one_dof_rotation_profiler.spinningBodyInMsg.subscribeTo(prescribed_rotation_msg)
    prescribed_body.prescribedRotationInMsg.subscribeTo(one_dof_rotation_profiler.prescribedRotationOutMsg)

    # Create the prescribed translational motion profiler
    trans_accel_max = 0.005  # [m/s^2]
    one_dof_translation_profiler = prescribedLinearTranslation.PrescribedLinearTranslation()
    one_dof_translation_profiler.ModelTag = "prescribedLinearTranslation"
    one_dof_translation_profiler.setTransHat_M(trans_axis_M)
    one_dof_translation_profiler.setTransAccelMax(trans_accel_max)
    one_dof_translation_profiler.setTransPosInit(pos_init)
    one_dof_translation_profiler.setCoastOptionBangDuration(1.0)
    one_dof_translation_profiler.setSmoothingDuration(1.0)
    sim.AddModelToTask(task_name, one_dof_translation_profiler)

    # Create the prescribed translational motion reference message
    prescribed_translation_msg_data = messaging.LinearTranslationRigidBodyMsgPayload()
    prescribed_translation_msg_data.rho = pos_ref
    prescribed_translation_msg_data.rhoDot = 0.0
    prescribed_translation_msg = messaging.LinearTranslationRigidBodyMsg().write(prescribed_translation_msg_data)
    one_dof_translation_profiler.linearTranslationRigidBodyInMsg.subscribeTo(prescribed_translation_msg)
    prescribed_body.prescribedTranslationInMsg.subscribeTo(one_dof_translation_profiler.prescribedTranslationOutMsg)

    # Add Earth gravity to the simulation
    earth_gravity = gravityEffector.GravBodyData()
    earth_gravity.planetName = "earth_planet_data"
    earth_gravity.mu = 0.3986004415E+15
    earth_gravity.isCentralBody = True
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([earth_gravity])

    # Set up data logging
    conservation_data_log = sc_object.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    prescribed_rotation_data_log = prescribed_body.prescribedRotationOutMsg.recorder()
    prescribed_translation_data_log = prescribed_body.prescribedTranslationOutMsg.recorder()
    one_dof_rotation_profiler_data_log = one_dof_rotation_profiler.spinningBodyOutMsg.recorder()
    sim.AddModelToTask(task_name, conservation_data_log)
    sim.AddModelToTask(task_name, prescribed_rotation_data_log)
    sim.AddModelToTask(task_name, prescribed_translation_data_log)
    sim.AddModelToTask(task_name, one_dof_rotation_profiler_data_log)

    # Run the simulation
    sim.InitializeSimulation()
    sim_time = 30.0  # [s]
    sim.ConfigureStopTime(macros.sec2nano(sim_time))
    sim.ExecuteSimulation()

    # Extract the logged data
    orb_energy = conservation_data_log.totOrbEnergy
    orb_ang_mom_n = conservation_data_log.totOrbAngMomPntN_N
    rot_ang_mom_n = conservation_data_log.totRotAngMomPntC_N
    rot_energy = conservation_data_log.totRotEnergy
    timespan = prescribed_rotation_data_log.times() * macros.NANO2SEC  # [s]
    prescribed_theta = macros.R2D * one_dof_rotation_profiler_data_log.theta  # [deg]
    omega_pm_p = prescribed_rotation_data_log.omega_PM_P * macros.R2D  # [deg/s]
    omega_prime_pm_p = prescribed_rotation_data_log.omegaPrime_PM_P * macros.R2D  # [deg/s^2]
    r_pm_m = prescribed_translation_data_log.r_PM_M  # [m]
    r_prime_pm_m = prescribed_translation_data_log.rPrime_PM_M  # [m/s]
    r_prime_prime_pm_m = prescribed_translation_data_log.rPrimePrime_PM_M  # [m/s^2]

    # Plot prescribed position and angle
    theta_ref_plotting = np.ones(len(timespan)) * theta_ref * macros.R2D  # [deg]
    prescribed_rho_ref_plotting = np.ones(len(timespan)) * pos_ref  # [m]
    prescribed_rho = np.dot(r_pm_m, trans_axis_M)  # [m]
    start_index = 3

    fig1, ax1 = plt.subplots()
    ax1.plot(timespan[start_index:], prescribed_theta[start_index:], label=r"$\theta$", color="teal")
    ax1.plot(timespan[start_index:], theta_ref_plotting[start_index:], "--", label=r"$\theta_{\text{ref}}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("Angle (deg)", color="teal", fontsize=16)
    ax2 = ax1.twinx()
    ax2.plot(timespan[start_index:], 100.0 * prescribed_rho[start_index:], label=r"$\rho$", color="darkviolet")
    ax2.plot(timespan[start_index:], 100.0 * prescribed_rho_ref_plotting[start_index:], "--", label=r"$\rho_{\text{ref}}$", color="darkviolet")
    ax2.set_ylabel("Displacement (cm)", color="darkviolet", fontsize=16)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.legend(handles=handles, labels=labels, loc="center left", prop={"size": 16})
    plt.grid(True)

    # Plot prescribed velocities
    prescribed_rho_dot = np.dot(r_prime_pm_m, trans_axis_M)  # [m/s]
    prescribed_theta_dot = np.dot(omega_pm_p, rot_axis_M)  # [deg/s]

    fig2, ax1 = plt.subplots()
    ax1.plot(timespan[start_index:], prescribed_theta_dot[start_index:], label=r"$\dot{\theta}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("Angle Rate (deg/s)", color="teal", fontsize=16)
    ax2 = ax1.twinx()
    ax2.plot(timespan[start_index:], 100.0 * prescribed_rho_dot[start_index:], label=r"$\dot{\rho}$", color="darkviolet")
    ax2.set_ylabel("Displacement Rate (cm/s)", color="darkviolet", fontsize=16)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.legend(handles=handles, labels=labels, loc="center", prop={"size": 16})
    plt.grid(True)

    # Plot prescribed accelerations
    prescribed_rho_ddot = np.dot(r_prime_prime_pm_m, trans_axis_M)
    prescribed_theta_ddot = np.dot(omega_prime_pm_p, rot_axis_M)  # [deg/s^2]

    fig3, ax1 = plt.subplots()
    ax1.plot(timespan[start_index:], prescribed_theta_ddot[start_index:], label=r"$\ddot{\theta}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (s)", fontsize=16)
    ax1.set_ylabel("Angular Acceleration (deg/$s^2$)", color="teal", fontsize=16)
    ax2 = ax1.twinx()
    ax2.plot(timespan[start_index:], 100.0 * prescribed_rho_ddot[start_index:], label=r"$\ddot{\rho}$", color="darkviolet")
    ax2.set_ylabel("Linear Acceleration (cm/$s^2$)", color="darkviolet", fontsize=16)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.legend(handles=handles, labels=labels, loc="upper right", prop={"size": 16})
    plt.grid(True)

    # Plot orbital angular momentum relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan[start_index:], (orb_ang_mom_n[start_index:, 0] - orb_ang_mom_n[start_index, 0]) / orb_ang_mom_n[start_index, 0], color="teal", label=r'$\hat{n}_1$')
    plt.plot(timespan[start_index:], (orb_ang_mom_n[start_index:, 1] - orb_ang_mom_n[start_index, 1]) / orb_ang_mom_n[start_index, 1], color="darkviolet", label=r'$\hat{n}_2$')
    plt.plot(timespan[start_index:], (orb_ang_mom_n[start_index:, 2] - orb_ang_mom_n[start_index, 2]) / orb_ang_mom_n[start_index, 2], color="blue", label=r'$\hat{n}_3$')
    plt.title('Orbital Angular Momentum Relative Difference', fontsize=16)
    plt.ylabel('Relative Difference (Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='lower right', prop={'size': 16})
    plt.grid(True)

    # Plot orbital energy relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan[start_index:], (orb_energy[start_index:] - orb_energy[start_index]) / orb_energy[start_index], color="teal")
    plt.title('Orbital Energy Relative Difference', fontsize=16)
    plt.ylabel('Relative Difference (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    # Plot sc angular momentum relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan[start_index:], (rot_ang_mom_n[start_index:, 0] - rot_ang_mom_n[start_index, 0]) / rot_ang_mom_n[start_index, 0], color="teal", label=r'$\hat{n}_1$')
    plt.plot(timespan[start_index:], (rot_ang_mom_n[start_index:, 1] - rot_ang_mom_n[start_index, 1]) / rot_ang_mom_n[start_index, 1], color="darkviolet", label=r'$\hat{n}_2$')
    plt.plot(timespan[start_index:], (rot_ang_mom_n[start_index:, 2] - rot_ang_mom_n[start_index, 2]) / rot_ang_mom_n[start_index, 2], color="blue", label=r'$\hat{n}_3$')
    plt.title('Rotational Angular Momentum Relative Difference', fontsize=16)
    plt.ylabel('Relative Difference (Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='upper right', prop={'size': 16})
    plt.grid(True)

    # Plot sc energy difference
    plt.figure()
    plt.clf()
    plt.plot(timespan[start_index:], (rot_energy[start_index:] - rot_energy[start_index]), color="teal")
    plt.title('Rotational Energy Difference', fontsize=16)
    plt.ylabel('Difference (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

    # Check conservation of orbital angular momentum, energy, and sc rotational angular momentum
    np.testing.assert_allclose(orb_ang_mom_n[start_index], orb_ang_mom_n[-1], rtol=1e-6, verbose=True)
    np.testing.assert_allclose(orb_energy[start_index], orb_energy[-1], rtol=1e-6, verbose=True)
    np.testing.assert_allclose(rot_ang_mom_n[start_index], rot_ang_mom_n[-1], rtol=1e-6, verbose=True)


if __name__ == "__main__":
    test_prescribed_motion_state_effector(
        True,  # show_plots
        0.0 * macros.D2R,  # theta_init [rad]
        10.0 * macros.D2R,  # theta_ref [rad]
        0.0,  # pos_init [m]
        0.1,  # pos_ref [m]
    )
