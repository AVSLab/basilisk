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
#   Unit Test Script
#   Module Name:        prescribedMotion integrated unit test with prescribedLinearTranslation
#   Author:             Leah Kiner
#   Creation Date:      March 12, 2024
#

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
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

matplotlib.rc('xtick', labelsize=16)
matplotlib.rc('ytick', labelsize=16)

@pytest.mark.parametrize("orbit_sim", [True, False])
@pytest.mark.parametrize("trans_pos_init", [0.0, 0.5])
@pytest.mark.parametrize("trans_pos_ref", [0.0, -0.5, 1.0])
@pytest.mark.parametrize("accuracy", [1e-8])
def test_PrescribedMotionStateEffector_Translation(show_plots, orbit_sim, trans_pos_init, trans_pos_ref, accuracy):
    r"""
    **Validation Test Description**

    The unit test for this module is an integrated test with two flight software profiler modules. This is required
    because the dynamics module must be connected to a flight software profiler module to define the states of the
    prescribed secondary body that is connected to the rigid spacecraft hub. The integrated test for this module has
    two simple scenarios it is testing. The first scenario prescribes a 1 DOF rotation for the prescribed body
    using the :ref:`prescribedRotation1DOF` flight software module. The second scenario prescribes
    linear translation for the prescribed body using the :ref:`prescribedLinearTranslation` flight software module.

    This unit test ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
    initial and reference PRV angles and maximum angular accelerations. The final prescribed attitude and angular
    velocity magnitude are compared with the reference values. This unit test also ensures that the profiled
    translational maneuver is properly computed for a series of initial and reference positions and maximum
    accelerations. The final prescribed position and velocity magnitudes are compared with the reference values.
    Additionally for each scenario, the conservation quantities of orbital angular momentum, rotational angular
    momentum, and orbital energy are checked to validate the module dynamics.

    **Test Parameters**

    Args:
        orbit_sim (bool): Choose if inertial orbit is simulated
        trans_pos_init (float): [m] Initial scalar position of the F frame with respect to the M frame
        trans_pos_ref (float): [m] Reference scalar position of the F frame with respect to the M frame
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This unit test ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
    initial and reference PRV angles and maximum angular accelerations. The final prescribed angle ``theta_FM_Final``
    and angular velocity magnitude ``thetaDot_Final`` are compared with the reference values ``theta_Ref`` and
    ``thetaDot_Ref``, respectively. This unit test also ensures that the profiled translational maneuver is properly
    computed for a series of initial and reference positions and maximum accelerations. The final prescribed position
    magnitude ``r_fm_m_final`` and velocity magnitude ``r_prime_fm_m_final`` are compared with the reference values
    ``r_fm_m_ref`` and ``r_prime_fm_m_ref``, respectively. Additionally for each scenario, the conservation quantities
    of orbital angular momentum, rotational angular momentum, and orbital energy are checked to validate the module
    dynamics.
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

    # Define the mass properties of the rigid spacecraft hub
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

    # Set the initial inertial hub states
    if orbit_sim:
        sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
        sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    else:
        sc_object.hub.r_CN_NInit = [[1.0], [1.0], [1.0]]
        sc_object.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]

    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]

    # Add the sc_object to the runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    # Define the prescribed motion state effector properties
    trans_axis_m = np.array([1.0, 0.0, 0.0])
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
    print(sc_object.hub.IHubPntBc_B)
    print(i_prescribed_body_fc_f)

    # Create the prescribed motion state effector
    prescribed_motion_body = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribed_motion_body.ModelTag = "prescribedMotionBody"
    prescribed_motion_body.mass = mass_prescribed_body
    prescribed_motion_body.IPntFc_F = i_prescribed_body_fc_f
    prescribed_motion_body.r_MB_B = [0.0, 0.0, 0.0]
    prescribed_motion_body.r_FcF_F = [0.0, 0.0, 0.0]
    prescribed_motion_body.r_fm_m = trans_pos_init * trans_axis_m
    prescribed_motion_body.r_prime_fm_m = [0.0, 0.0, 0.0]
    prescribed_motion_body.r_prime_prime_fm_m = [0.0, 0.0, 0.0]
    prescribed_motion_body.omega_FM_F = [0.0, 0.0, 0.0]
    prescribed_motion_body.omegaPrime_FM_F = [0.0, 0.0, 0.0]
    prescribed_motion_body.sigma_FM = [0.0, 0.0, 0.0]
    prescribed_motion_body.omega_MB_B = [0.0, 0.0, 0.0]
    prescribed_motion_body.omegaPrime_MB_B = [0.0, 0.0, 0.0]
    prescribed_motion_body.sigma_MB = [0.0, 0.0, 0.0]

    # Add prescribed_motion_body to spacecraft
    sc_object.addStateEffector(prescribed_motion_body)

    # Add the test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_motion_body)

    # Create an instance of the prescribedLinearTranslation module to be tested
    prescribed_translation = prescribedLinearTranslation.PrescribedLinearTranslation()
    prescribed_translation.ModelTag = "prescribedLinearTranslation"

    # Add the prescribedLinearTranslation test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_translation)

    # Initialize the prescribedLinearTranslation test module configuration data
    trans_accel_max = 0.005  # [m/s^2]
    prescribed_translation.setTransHat_M(trans_axis_m)
    prescribed_translation.setTransAccelMax(trans_accel_max)
    prescribed_translation.setTransPosInit(trans_pos_init)
    prescribed_translation.setCoastOptionBangDuration(1.0)
    prescribed_translation.setSmoothingDuration(1.0)

    # Create the prescribed_translation input message
    linear_translating_body_message_data = messaging.LinearTranslationRigidBodyMsgPayload()
    linear_translating_body_message_data.rho = trans_pos_ref
    linear_translating_body_message_data.rhoDot = 0.0
    linear_translating_body_message = messaging.LinearTranslationRigidBodyMsg().write(linear_translating_body_message_data)
    prescribed_translation.linearTranslationRigidBodyInMsg.subscribeTo(linear_translating_body_message)

    prescribed_motion_body.prescribedTranslationInMsg.subscribeTo(prescribed_translation.prescribedTranslationOutMsg)

    if orbit_sim:
        # Add Earth gravity to the simulation
        earth_grav_body = gravityEffector.GravBodyData()
        earth_grav_body.planetName = "earth_planet_data"
        earth_grav_body.mu = 0.3986004415E+15
        earth_grav_body.isCentralBody = True
        sc_object.gravField.gravBodies = spacecraft.GravBodyVector([earth_grav_body])

    # Log data
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    prescribed_trans_state_data_log = prescribed_motion_body.prescribedTranslationOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, sc_state_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_trans_state_data_log)
    if orbit_sim:
        # Add energy and momentum variables to log
        sc_energy_momentum_log = sc_object.logger(["totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totRotEnergy"])
        unit_test_sim.AddModelToTask(unit_task_name, sc_energy_momentum_log)

    # Initialize the simulation
    unit_test_sim.InitializeSimulation()
    sim_time = 15
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    timespan = sc_state_data_log.times() * macros.NANO2SEC  # [s]
    r_bn_n = sc_state_data_log.r_BN_N
    sigma_bn = sc_state_data_log.sigma_BN
    omega_bn_b = sc_state_data_log.omega_BN_B * macros.R2D  # [deg]
    r_fm_m = prescribed_trans_state_data_log.r_FM_M
    r_prime_fm_m = prescribed_trans_state_data_log.rPrime_FM_M
    r_prime_prime_fm_m = prescribed_trans_state_data_log.rPrimePrime_FM_M
    r_fm_m_final = r_fm_m[-1, :]
    r_prime_fm_m_final = r_prime_fm_m[-1, :]

    if orbit_sim:
        orb_energy = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(), sc_energy_momentum_log.totOrbEnergy)
        orb_ang_mom_n = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(), sc_energy_momentum_log.totOrbAngMomPntN_N)
        rot_ang_mom_n = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(), sc_energy_momentum_log.totRotAngMomPntC_N)
        rot_energy = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(), sc_energy_momentum_log.totRotEnergy)

        # Setup the conservation quantities
        orb_ang_mom_init = [[orb_ang_mom_n[0, 1], orb_ang_mom_n[0, 2], orb_ang_mom_n[0, 3]]]
        orb_ang_mom_final = [orb_ang_mom_n[-1]]
        rot_ang_mom_init = [[rot_ang_mom_n[0, 1], rot_ang_mom_n[0, 2], rot_ang_mom_n[0, 3]]]
        rot_ang_mom_final = [rot_ang_mom_n[-1]]
        orb_energy_init = [[orb_energy[0, 1]]]
        orb_energy_final = [orb_energy[-1]]

    else:
        r_cn_n = sc_state_data_log.r_CN_N  # [m]

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
    # plt.title(r'${}^\mathcal{M} r_{F/M}$ Profiled Trajectory', fontsize=14)
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
    # plt.title(r'${}^\mathcal{M} rPrime_{F/M}$ Profiled Trajectory', fontsize=14)
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
    # plt.title(r'${}^\mathcal{M} rPrimePrime_{F/M}$ Profiled Acceleration', fontsize=14)
    plt.ylabel(r'(m/s$^2$)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='lower left', prop={'size': 16})
    plt.grid(True)

    # # Plot r_bn_n
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, r_bn_n[:, 0], label=r'$r_{1}$')
    # plt.plot(timespan, r_bn_n[:, 1], label=r'$r_{2}$')
    # plt.plot(timespan, r_bn_n[:, 2], label=r'$r_{3}$')
    # plt.title(r'${}^\mathcal{N} r_{B/N}$ Spacecraft Inertial Trajectory', fontsize=14)
    # plt.ylabel('(m)', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='center left', prop={'size': 16})
    # plt.grid(True)
    #
    # # Plot sigma_bn
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, sigma_bn[:, 0], label=r'$\sigma_{1}$')
    # plt.plot(timespan, sigma_bn[:, 1], label=r'$\sigma_{2}$')
    # plt.plot(timespan, sigma_bn[:, 2], label=r'$\sigma_{3}$')
    # plt.title(r'$\sigma_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial MRP Attitude', fontsize=14)
    # plt.ylabel('', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='lower left', prop={'size': 16})
    # plt.grid(True)
    #
    # # Plot omega_bn_b
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, omega_bn_b[:, 0], label=r'$\omega_{1}$')
    # plt.plot(timespan, omega_bn_b[:, 1], label=r'$\omega_{2}$')
    # plt.plot(timespan, omega_bn_b[:, 2], label=r'$\omega_{3}$')
    # plt.title(r'Spacecraft Hub Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$', fontsize=14)
    # plt.ylabel('(deg/s)', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='lower left', prop={'size': 16})
    # plt.grid(True)

    if orbit_sim:
        # Plot conservation quantities
        plt.figure()
        plt.clf()
        plt.plot(timespan, (orb_ang_mom_n[:, 1] - orb_ang_mom_n[0, 1]) / orb_ang_mom_n[0, 1],
                 timespan, (orb_ang_mom_n[:, 2] - orb_ang_mom_n[0, 2]) / orb_ang_mom_n[0, 2],
                 timespan, (orb_ang_mom_n[:, 3] - orb_ang_mom_n[0, 3]) / orb_ang_mom_n[0, 3])
        # plt.title('Orbital Angular Momentum Relative Difference', fontsize=14)
        plt.ylabel('Relative Difference (Nms)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.grid(True)

        plt.figure()
        plt.clf()
        plt.plot(timespan, (orb_energy[:, 1] - orb_energy[0, 1]) / orb_energy[0, 1])
        # plt.title('Orbital Energy Relative Difference', fontsize=14)
        plt.ylabel('Relative Difference (J)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.grid(True)

        plt.figure()
        plt.clf()
        plt.plot(timespan, (rot_ang_mom_n[:, 1] - rot_ang_mom_n[0, 1]),
                 timespan, (rot_ang_mom_n[:, 2] - rot_ang_mom_n[0, 2]),
                 timespan, (rot_ang_mom_n[:, 3] - rot_ang_mom_n[0, 3]))
        # plt.title('Rotational Angular Momentum Difference', fontsize=14)
        plt.ylabel('Difference (Nms)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.grid(True)

        plt.figure()
        plt.clf()
        plt.plot(timespan, (rot_energy[:, 1] - rot_energy[0, 1]))
        # plt.title('Total Energy Difference', fontsize=14)
        plt.ylabel('Difference (J)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.grid(True)

    else:
        # Plot r_cn_n
        plt.figure()
        plt.clf()
        plt.plot(timespan, (r_cn_n[:, 0] - r_cn_n[0, 0]) / r_cn_n[0, 0], label=r'$r_{1}$')
        plt.plot(timespan, (r_cn_n[:, 1] - r_cn_n[0, 1]) / r_cn_n[0, 1], label=r'$r_{2}$')
        plt.plot(timespan, (r_cn_n[:, 2] - r_cn_n[0, 2]) / r_cn_n[0, 2], label=r'$r_{3}$')
        plt.title(r'${}^\mathcal{N} r_{C/N}$ Spacecraft COM Inertial Trajectory', fontsize=16)
        plt.ylabel('(m)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='center left', prop={'size': 16})
        plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

    # Begin the test analysis
    if orbit_sim:
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

    # Check to ensure the initial velocity converged to the reference velocity
    r_prime_fm_m_ref = np.array([0.0, 0.0, 0.0])
    np.testing.assert_allclose(r_prime_fm_m_ref,
                               r_prime_fm_m_final,
                               atol=accuracy,
                               verbose=True)

    # Check to ensure the initial position converged to the reference position
    r_fm_m_ref = trans_pos_ref * trans_axis_m
    np.testing.assert_allclose(r_fm_m_ref,
                               r_fm_m_final,
                               atol=accuracy,
                               verbose=True)

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_PrescribedMotionStateEffector_Translation(
        True,  # show_plots
        True,  # orbit_sim
        0.0,  # trans_pos_init [m]
        0.1,  # trans_pos_ref [m]
        1e-8  # accuracy
       )
