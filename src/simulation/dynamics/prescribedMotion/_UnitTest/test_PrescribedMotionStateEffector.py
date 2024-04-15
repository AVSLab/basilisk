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
#   Integrated Unit Test Script
#   Module Name:        prescribedMotion
#   Author:             Leah Kiner
#   Creation Date:      Jan 10, 2022
#   Last Updated:       April 15, 2024
#

import inspect
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
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
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

@pytest.mark.parametrize("theta_init", [0.0, macros.D2R * 2.0])
@pytest.mark.parametrize("theta_ref", [0.0, macros.D2R * 5.0])
@pytest.mark.parametrize("accuracy", [1e-7])
def test_PrescribedMotionStateEffector_Rotation(show_plots, theta_init, theta_ref, accuracy):
    r"""
    **Rotational 1-DOF Prescribed Motion Verification Test Description**

    This prescribed motion unit test function is an integrated test with the :ref:`prescribedRotation1DOF` kinematic
    profiler module. The profiler module prescribes a 1-degree-of-freedom (1-DOF) rotation for the prescribed state
    effector relative to the spacecraft hub.

    This unit test function verifies the prescribed motion state effector dynamics by checking to ensure that the
    orbital angular momentum, orbital energy, and spacecraft rotational angular momentum are reasonably conserved.

    **Test Parameters**

    Args:
        theta_init (float): [rad] Initial hub-relative angle of the prescribed motion effector
        theta_ref (float): [rad] Reference hub-relative angle of the prescribed motion effector
        accuracy (float): Absolute accuracy value used in the verification tests

    **Description of Variables Being Tested**

    This unit test checks to ensure that the initial simulation values of orbital energy, orbital angular momentum,
    and the spacecraft rotational angular momentum match the final simulation values to a reasonable extent.
    """

    __tracebackhide__ = True

    unit_task_name = "unitTask"
    unit_process_name = "testProcess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_increment = 0.001  # [s]
    test_process_rate = macros.sec2nano(test_increment)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add the spacecraft module to test file
    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "spacecraftBody"
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    # Define the mass properties of the rigid spacecraft hub
    set_spacecraft_hub(sc_object)

    # Create the prescribed motion state effector
    rot_axis_m = np.array([1.0, 0.0, 0.0])
    prv_fm_init = theta_init * rot_axis_m
    sigma_fm = rbk.PRV2MRP(prv_fm_init)
    prescribed_motion_body = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    set_prescribed_motion_effector(prescribed_motion_body)
    prescribed_motion_body.setSigma_FM(sigma_fm)
    sc_object.addStateEffector(prescribed_motion_body)
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_motion_body)

    # Create an instance of the prescribedRotation1DOF module to be tested
    ang_accel_max = 0.2 * macros.D2R  # [rad/s^2]
    prescribed_rot_1_dof = prescribedRotation1DOF.PrescribedRotation1DOF()
    prescribed_rot_1_dof.setRotHat_M(rot_axis_m)
    prescribed_rot_1_dof.setThetaDDotMax(ang_accel_max)
    prescribed_rot_1_dof.setThetaInit(theta_init)
    prescribed_rot_1_dof.setCoastOptionBangDuration(1.0)
    prescribed_rot_1_dof.setSmoothingDuration(1.0)
    prescribed_rot_1_dof.ModelTag = "prescribedRotation1DOF"
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_rot_1_dof)

    # Create the prescribedRotation1DOF input message
    spinning_body_message_data = messaging.HingedRigidBodyMsgPayload()
    spinning_body_message_data.theta = theta_ref
    spinning_body_message_data.thetaDot = 0.0  # [rad/s]
    spinning_body_message = messaging.HingedRigidBodyMsg().write(spinning_body_message_data)
    prescribed_rot_1_dof.spinningBodyInMsg.subscribeTo(spinning_body_message)
    prescribed_motion_body.prescribedRotationInMsg.subscribeTo(prescribed_rot_1_dof.prescribedRotationOutMsg)

    # Add Earth gravity to the simulation
    earth_grav_body = gravityEffector.GravBodyData()
    earth_grav_body.planetName = "earth_planet_data"
    earth_grav_body.mu = 0.3986004415E+15
    earth_grav_body.isCentralBody = True
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([earth_grav_body])

    # Log data
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    prescribed_rot_state_data_log = prescribed_motion_body.prescribedRotationOutMsg.recorder()
    theta_data_log = prescribed_rot_1_dof.spinningBodyOutMsg.recorder()
    sc_energy_momentum_log = sc_object.logger(["totOrbEnergy",
                                               "totOrbAngMomPntN_N",
                                               "totRotAngMomPntC_N",
                                               "totRotEnergy"])
    unit_test_sim.AddModelToTask(unit_task_name, sc_state_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_rot_state_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, theta_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, sc_energy_momentum_log)

    # Initialize the simulation
    unit_test_sim.InitializeSimulation()

    # Set the simulation time
    sim_time = 30  # [s]
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))

    # Begin the simulation
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    timespan = sc_state_data_log.times() * macros.NANO2SEC  # [s]
    theta = macros.R2D * theta_data_log.theta  # [deg]
    omega_fm_f = prescribed_rot_state_data_log.omega_FM_F * macros.R2D  # [deg]
    omega_prime_fm_f = prescribed_rot_state_data_log.omegaPrime_FM_F * macros.R2D  # [deg]
    orb_energy = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(),
                                               sc_energy_momentum_log.totOrbEnergy)
    orb_ang_mom_n = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(),
                                                  sc_energy_momentum_log.totOrbAngMomPntN_N)
    rot_ang_mom_n = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(),
                                                  sc_energy_momentum_log.totRotAngMomPntC_N)
    rot_energy = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(),
                                               sc_energy_momentum_log.totRotEnergy)

    # Plot results
    if show_plots:
        # Plot theta_FM
        theta_ref_plotting = np.ones(len(timespan)) * theta_ref * macros.R2D  # [deg]
        theta_init_plotting = np.ones(len(timespan)) * theta_init * macros.R2D  # [deg]
        plt.figure()
        plt.clf()
        plt.plot(timespan, theta, label=r'$\theta$')
        plt.plot(timespan, theta_init_plotting, '--', label=r'$\theta_{0}$')
        plt.plot(timespan, theta_ref_plotting, '--', label=r'$\theta_{Ref}$')
        plt.title(r'$\theta$ Profiled Trajectory', fontsize=16)
        plt.ylabel('(deg)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='center right', prop={'size': 16})
        plt.grid(True)

        # Plot omega_fm_f
        plt.figure()
        plt.clf()
        plt.plot(timespan, omega_fm_f[:, 0], label='1')
        plt.plot(timespan, omega_fm_f[:, 1], label='2')
        plt.plot(timespan, omega_fm_f[:, 2], label='3')
        plt.title(r'${}^\mathcal{F} \omega_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=16)
        plt.ylabel('(deg/s)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='upper right', prop={'size': 16})
        plt.grid(True)

        # Plot omega_prime_fm_f
        plt.figure()
        plt.clf()
        plt.plot(timespan, omega_prime_fm_f[:, 0], label='1')
        plt.plot(timespan, omega_prime_fm_f[:, 1], label='2')
        plt.plot(timespan, omega_prime_fm_f[:, 2], label='3')
        plt.title(r'${}^\mathcal{F} \omega Prime_{\mathcal{F}/\mathcal{M}}$ Profiled Angular Acceleration', fontsize=16)
        plt.ylabel(r'(deg/$s^2$)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='upper right', prop={'size': 16})
        plt.grid(True)

        # Plot conservation quantities
        plot_conservation(timespan, orb_ang_mom_n, orb_energy, rot_ang_mom_n, rot_energy)

        plt.show()

    plt.close("all")

    # Check conservation quantities
    check_conservation(orb_ang_mom_n, orb_energy, rot_ang_mom_n, accuracy)

@pytest.mark.parametrize("trans_pos_init", [0.0, 0.1])
@pytest.mark.parametrize("trans_pos_ref", [0.0, 0.1])
@pytest.mark.parametrize("accuracy", [1e-8])
def test_PrescribedMotionStateEffector_Translation(show_plots, trans_pos_init, trans_pos_ref, accuracy):
    r"""
    **Linear Translational Prescribed Motion Verification Test Description**

    This prescribed motion unit test function is an integrated test with the :ref:`prescribedLinearTranslation`
    kinematic profiler module. The profiler module prescribes linear translational motion for the prescribed state
    effector relative to the spacecraft hub.

    This unit test function verifies the prescribed motion state effector dynamics by checking to ensure that the
    orbital angular momentum, orbital energy, and spacecraft rotational angular momentum are reasonably conserved.

    **Test Parameters**

    Args:
        trans_pos_init (float): [m] Initial displacement of the prescribed motion effector relative to the spacecraft hub
        trans_pos_ref (float): [m] Reference displacement of the prescribed motion effector relative to the spacecraft hub
        accuracy (float): Absolute accuracy value used in the verification tests

    **Description of Variables Being Tested**

    This unit test checks to ensure that the initial simulation values of orbital energy, orbital angular momentum,
    and the spacecraft rotational angular momentum match the final simulation values to a reasonable extent.
    """

    __tracebackhide__ = True

    unit_task_name = "unitTask"
    unit_process_name = "test_processess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_increment = 0.001  # [s]
    test_process_rate = macros.sec2nano(test_increment)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add the spacecraft module to test file
    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "spacecraftBody"
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    # Define the mass properties of the rigid spacecraft hub
    set_spacecraft_hub(sc_object)

    # Create the prescribed motion state effector
    trans_axis_m = np.array([1.0, 0.0, 0.0])
    prescribed_motion_body = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    set_prescribed_motion_effector(prescribed_motion_body)
    prescribed_motion_body.setR_FM_M(trans_pos_init * trans_axis_m)
    sc_object.addStateEffector(prescribed_motion_body)
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_motion_body)

    # Create an instance of the prescribedLinearTranslation module to be tested
    trans_accel_max = 0.005  # [m/s^2]
    prescribed_translation = prescribedLinearTranslation.PrescribedLinearTranslation()
    prescribed_translation.setTransHat_M(trans_axis_m)
    prescribed_translation.setTransAccelMax(trans_accel_max)
    prescribed_translation.setTransPosInit(trans_pos_init)
    prescribed_translation.setCoastOptionBangDuration(1.0)
    prescribed_translation.setSmoothingDuration(1.0)
    prescribed_translation.ModelTag = "prescribedLinearTranslation"
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_translation)

    # Create the prescribed_translation input message
    linear_translating_body_message_data = messaging.LinearTranslationRigidBodyMsgPayload()
    linear_translating_body_message_data.rho = trans_pos_ref
    linear_translating_body_message_data.rhoDot = 0.0
    linear_translating_body_message = messaging.LinearTranslationRigidBodyMsg().write(linear_translating_body_message_data)
    prescribed_translation.linearTranslationRigidBodyInMsg.subscribeTo(linear_translating_body_message)
    prescribed_motion_body.prescribedTranslationInMsg.subscribeTo(prescribed_translation.prescribedTranslationOutMsg)

    # Add Earth gravity to the simulation
    earth_grav_body = gravityEffector.GravBodyData()
    earth_grav_body.planetName = "earth_planet_data"
    earth_grav_body.mu = 0.3986004415E+15
    earth_grav_body.isCentralBody = True
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([earth_grav_body])

    # Log data
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    prescribed_trans_state_data_log = prescribed_motion_body.prescribedTranslationOutMsg.recorder()
    sc_energy_momentum_log = sc_object.logger(["totOrbEnergy",
                                               "totOrbAngMomPntN_N",
                                               "totRotAngMomPntC_N",
                                               "totRotEnergy"])
    unit_test_sim.AddModelToTask(unit_task_name, sc_state_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_trans_state_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, sc_energy_momentum_log)

    # Initialize the simulation
    unit_test_sim.InitializeSimulation()
    sim_time = 30
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    timespan = sc_state_data_log.times() * macros.NANO2SEC  # [s]
    r_fm_m = prescribed_trans_state_data_log.r_FM_M
    r_prime_fm_m = prescribed_trans_state_data_log.rPrime_FM_M
    r_prime_prime_fm_m = prescribed_trans_state_data_log.rPrimePrime_FM_M
    orb_energy = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(),
                                               sc_energy_momentum_log.totOrbEnergy)
    orb_ang_mom_n = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(),
                                                  sc_energy_momentum_log.totOrbAngMomPntN_N)
    rot_ang_mom_n = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(),
                                                  sc_energy_momentum_log.totRotAngMomPntC_N)
    rot_energy = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(),
                                               sc_energy_momentum_log.totRotEnergy)

    # Plot results
    if show_plots:
        # Plot r_FM_F
        r_fm_m_ref = trans_pos_ref * trans_axis_m
        r_fm_m_1_ref = np.ones(len(timespan)) * r_fm_m_ref[0]
        r_fm_m_2_ref = np.ones(len(timespan)) * r_fm_m_ref[1]
        r_fm_m_3_ref = np.ones(len(timespan)) * r_fm_m_ref[2]
        plt.figure()
        plt.clf()
        plt.plot(timespan, r_fm_m[:, 0], label=r'$r_{1}$')
        plt.plot(timespan, r_fm_m[:, 1], label=r'$r_{2}$')
        plt.plot(timespan, r_fm_m[:, 2], label=r'$r_{3}$')
        plt.plot(timespan, r_fm_m_1_ref, '--', label=r'$r_{1 Ref}$')
        plt.plot(timespan, r_fm_m_2_ref, '--', label=r'$r_{2 Ref}$')
        plt.plot(timespan, r_fm_m_3_ref, '--', label=r'$r_{3 Ref}$')
        plt.title(r'${}^\mathcal{M} r_{F/M}$ Profiled Trajectory', fontsize=14)
        plt.ylabel('(m)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='center left', prop={'size': 16})
        plt.grid(True)

        # Plot rPrime_FM_F
        plt.figure()
        plt.clf()
        plt.plot(timespan, r_prime_fm_m[:, 0], label='1')
        plt.plot(timespan, r_prime_fm_m[:, 1], label='2')
        plt.plot(timespan, r_prime_fm_m[:, 2], label='3')
        plt.title(r'${}^\mathcal{M} rPrime_{F/M}$ Profiled Trajectory', fontsize=14)
        plt.ylabel('(m/s)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='upper left', prop={'size': 16})
        plt.grid(True)

        # Plot r_prime_prime_fm_m
        plt.figure()
        plt.clf()
        plt.plot(timespan, r_prime_prime_fm_m[:, 0], label='1')
        plt.plot(timespan, r_prime_prime_fm_m[:, 1], label='2')
        plt.plot(timespan, r_prime_prime_fm_m[:, 2], label='3')
        plt.title(r'${}^\mathcal{M} rPrimePrime_{F/M}$ Profiled Acceleration', fontsize=14)
        plt.ylabel(r'(m/s$^2$)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='lower left', prop={'size': 16})
        plt.grid(True)

        # Plot conservation quantities
        plot_conservation(timespan, orb_ang_mom_n, orb_energy, rot_ang_mom_n, rot_energy)

        plt.show()

    plt.close("all")

    # Check conservation quantities
    check_conservation(orb_ang_mom_n, orb_energy, rot_ang_mom_n, accuracy)

def set_spacecraft_hub(sc_object):
    mass_hub = 800  # [kg]
    length_hub = 1.0  # [m]
    width_hub = 1.0  # [m]
    depth_hub = 1.0  # [m]
    i_hub_11 = (1 / 12) * mass_hub * (length_hub * length_hub + depth_hub * depth_hub)  # [kg m^2]
    i_hub_22 = (1 / 12) * mass_hub * (length_hub * length_hub + width_hub * width_hub)  # [kg m^2]
    i_hub_33 = (1 / 12) * mass_hub * (width_hub * width_hub + depth_hub * depth_hub)  # [kg m^2]
    sc_object.hub.mHub = mass_hub  # kg
    sc_object.hub.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
    sc_object.hub.IHubPntBc_B = [[i_hub_11, 0.0, 0.0],
                                 [0.0, i_hub_22, 0.0],
                                 [0.0, 0.0, i_hub_33]]  # [kg m^2] (Hub approximated as a cube)
    sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]

def set_prescribed_motion_effector(prescribed_motion_body):
    mass_prescribed_body = 10  # [kg]
    length_prescribed_body = 0.1  # [m]
    width_prescribed_body = 0.1  # [m]
    depth_prescribed_body = 0.1  # [m]
    i_prescribed_body_11 = (1 / 12) * mass_prescribed_body * (length_prescribed_body
                                                              * length_prescribed_body
                                                              + depth_prescribed_body
                                                              * depth_prescribed_body)  # [kg m^2]
    i_prescribed_body_22 = (1 / 12) * mass_prescribed_body * (length_prescribed_body
                                                              * length_prescribed_body
                                                              + width_prescribed_body
                                                              * width_prescribed_body)  # [kg m^2]
    i_prescribed_body_33 = (1 / 12) * mass_prescribed_body * (width_prescribed_body
                                                              * width_prescribed_body
                                                              + depth_prescribed_body
                                                              * depth_prescribed_body)  # [kg m^2]
    i_prescribed_body_fc_f = [[i_prescribed_body_11, 0.0, 0.0],
                              [0.0, i_prescribed_body_22, 0.0],
                              [0.0, 0.0,
                               i_prescribed_body_33]]  # [kg m^2] (approximated as a cube)
    prescribed_motion_body.setMass(mass_prescribed_body)
    prescribed_motion_body.setIPntFc_F(i_prescribed_body_fc_f)
    prescribed_motion_body.setR_MB_B([0.0, 0.0, 0.0])
    prescribed_motion_body.setR_FcF_F([0.0, 0.0, 0.0])
    prescribed_motion_body.setR_FM_M([0.0, 0.0, 0.0])
    prescribed_motion_body.setRPrime_FM_M([0.0, 0.0, 0.0])
    prescribed_motion_body.setRPrimePrime_FM_M([0.0, 0.0, 0.0])
    prescribed_motion_body.setOmega_FM_F([0.0, 0.0, 0.0])
    prescribed_motion_body.setOmegaPrime_FM_F([0.0, 0.0, 0.0])
    prescribed_motion_body.setSigma_FM([0.0, 0.0, 0.0])
    prescribed_motion_body.setOmega_MB_B([0.0, 0.0, 0.0])
    prescribed_motion_body.setOmegaPrime_MB_B([0.0, 0.0, 0.0])
    prescribed_motion_body.setSigma_MB([0.0, 0.0, 0.0])
    prescribed_motion_body.ModelTag = "prescribedMotionBody"

def plot_conservation(timespan, orb_ang_mom_n, orb_energy, rot_ang_mom_n, rot_energy):
    plt.figure()
    plt.clf()
    plt.plot(timespan, (orb_ang_mom_n[:, 1] - orb_ang_mom_n[0, 1]) / orb_ang_mom_n[0, 1],
             timespan, (orb_ang_mom_n[:, 2] - orb_ang_mom_n[0, 2]) / orb_ang_mom_n[0, 2],
             timespan, (orb_ang_mom_n[:, 3] - orb_ang_mom_n[0, 3]) / orb_ang_mom_n[0, 3])
    plt.title('Orbital Angular Momentum Relative Difference', fontsize=14)
    plt.ylabel('Relative Difference (Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    plt.figure()
    plt.clf()
    plt.plot(timespan, (orb_energy[:, 1] - orb_energy[0, 1]) / orb_energy[0, 1])
    plt.title('Orbital Energy Relative Difference', fontsize=14)
    plt.ylabel('Relative Difference (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    plt.figure()
    plt.clf()
    plt.plot(timespan, (rot_ang_mom_n[:, 1] - rot_ang_mom_n[0, 1]),
             timespan, (rot_ang_mom_n[:, 2] - rot_ang_mom_n[0, 2]),
             timespan, (rot_ang_mom_n[:, 3] - rot_ang_mom_n[0, 3]))
    plt.title('Rotational Angular Momentum Difference', fontsize=14)
    plt.ylabel('Difference (Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    plt.figure()
    plt.clf()
    plt.plot(timespan, (rot_energy[:, 1] - rot_energy[0, 1]))
    plt.title('Total Energy Difference', fontsize=14)
    plt.ylabel('Difference (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

def check_conservation(orb_ang_mom_n, orb_energy, rot_ang_mom_n, accuracy):
    orb_ang_mom_init = [[orb_ang_mom_n[0, 1], orb_ang_mom_n[0, 2], orb_ang_mom_n[0, 3]]]
    orb_ang_mom_final = [orb_ang_mom_n[-1]]
    rot_ang_mom_init = [[rot_ang_mom_n[0, 1], rot_ang_mom_n[0, 2], rot_ang_mom_n[0, 3]]]
    rot_ang_mom_final = [rot_ang_mom_n[-1]]
    orb_energy_init = [[orb_energy[0, 1]]]
    orb_energy_final = [orb_energy[-1]]

    orb_ang_mom_final = np.delete(orb_ang_mom_final, 0, axis=1)  # remove the time column
    rot_ang_mom_final = np.delete(rot_ang_mom_final, 0, axis=1)  # remove the time column
    orb_energy_final = np.delete(orb_energy_final, 0, axis=1)  # remove the time column

    # Orbital angular momentum check
    np.testing.assert_allclose(orb_ang_mom_init,
                               orb_ang_mom_final,
                               atol=accuracy,
                               verbose=True)

    # Rotational angular momentum check
    np.testing.assert_allclose(rot_ang_mom_init,
                               rot_ang_mom_final,
                               atol=accuracy,
                               verbose=True)

    # Orbital energy check
    np.testing.assert_allclose(orb_energy_init,
                               orb_energy_final,
                               atol=accuracy,
                               verbose=True)

if __name__ == "__main__":
    test_PrescribedMotionStateEffector_Translation(
        True,  # show_plots
        0.0,  # trans_pos_init [m]
        0.1,  # trans_pos_ref [m]
        1e-8  # accuracy
    )

    test_PrescribedMotionStateEffector_Rotation(
        True,  # show_plots
        0.0,  # theta_init [rad]
        5.0 * macros.D2R,  # theta_ref [rad]
        1e-7  # accuracy
    )
    