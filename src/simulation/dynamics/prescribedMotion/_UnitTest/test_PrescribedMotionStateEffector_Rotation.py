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
#   Module Name:        prescribedMotion integrated unit test with prescribedRotation1DOF
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

matplotlib.rc('xtick', labelsize=16)
matplotlib.rc('ytick', labelsize=16)


@pytest.mark.parametrize("theta_init", [0.0, macros.D2R * 10.0])
@pytest.mark.parametrize("theta_ref", [0.0, macros.D2R * 5.0])
@pytest.mark.parametrize("accuracy", [1e-8])
def test_PrescribedMotionStateEffector_Rotation(show_plots, theta_init, theta_ref, accuracy):
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
        theta_init (float): [rad] Initial PRV angle of the F frame with respect to the M frame
        theta_ref (float): [rad] Reference PRV angle of the F frame with respect to the M frame
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This unit test ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
    initial and reference PRV angles and maximum angular accelerations. The final prescribed angle ``theta_FM_Final``
    and angular velocity magnitude ``theta_dot_final`` are compared with the reference values ``theta_ref`` and
    ``thetaDot_Ref``, respectively. This unit test also ensures that the profiled translational maneuver is properly
    computed for a series of initial and reference positions and maximum accelerations. The final prescribed position
    magnitude ``r_FM_M_Final`` and velocity magnitude ``rPrime_FM_M_Final`` are compared with the reference values
    ``r_FM_M_Ref`` and ``rPrime_FM_M_Ref``, respectively. Additionally for each scenario, the conservation quantities
    of orbital angular momentum, rotational angular momentum, and orbital energy are checked to validate the module
    dynamics.
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
    sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    sc_object.hub.omega_bn_bInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.sigma_bnInit = [[0.0], [0.0], [0.0]]

    # Add the sc_object to the runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    # Define the prescribed motion state effector properties
    rot_axis_m = np.array([1.0, 0.0, 0.0])
    prv_fm_init = theta_init * rot_axis_m
    sigma_fm = rbk.PRV2MRP(prv_fm_init)
    mass_prescribed_body = 8  # [kg]
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

    # Create the prescribed motion state effector
    prescribed_motion_body = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    prescribed_motion_body.ModelTag = "prescribedMotionBody"
    prescribed_motion_body.mass = mass_prescribed_body
    prescribed_motion_body.IPntFc_F = i_prescribed_body_fc_f
    prescribed_motion_body.r_MB_B = [0.0, 0.0, 0.0]
    prescribed_motion_body.r_FcF_F = [0.0, 0.0, 0.0]
    prescribed_motion_body.r_FM_M = [0.0, 0.0, 0.0]
    prescribed_motion_body.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    prescribed_motion_body.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
    prescribed_motion_body.omega_fm_f = np.array([0.0, 0.0, 0.0])
    prescribed_motion_body.omega_prime_fm_f = np.array([0.0, 0.0, 0.0])
    prescribed_motion_body.sigma_fm = sigma_fm
    prescribed_motion_body.omega_MB_B = [0.0, 0.0, 0.0]
    prescribed_motion_body.omegaPrime_MB_B = [0.0, 0.0, 0.0]
    prescribed_motion_body.sigma_MB = [0.0, 0.0, 0.0]

    # Add prescribed_motion_body to spacecraft
    sc_object.addStateEffector(prescribed_motion_body)

    # Add the test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_motion_body)

    # Create an instance of the prescribedRotation1DOF module to be tested
    prescribed_rot_1_dof = prescribedRotation1DOF.PrescribedRotation1DOF()
    prescribed_rot_1_dof.ModelTag = "prescribedRotation1DOF"

    # Add the prescribedRotation1DOF test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_rot_1_dof)

    # Initialize the prescribedRotation1DOF test module configuration data
    ang_accel_max = 0.5 * macros.D2R  # [rad/s^2]
    prescribed_rot_1_dof.setRotHat_M(rot_axis_m)
    prescribed_rot_1_dof.setThetaDDotMax(ang_accel_max)
    prescribed_rot_1_dof.setThetaInit(theta_init)
    prescribed_rot_1_dof.setCoastOptionRampDuration(1.0)
    prescribed_rot_1_dof.setSmoothingDuration(1.0)

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

    # Add energy and momentum variables to log
    sc_energy_momentum_log = sc_object.logger(["totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totRotEnergy"])
    unit_test_sim.AddModelToTask(unit_task_name, sc_energy_momentum_log)

    # Add other states to log
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    prescribed_rot_state_data_log = prescribed_motion_body.prescribedRotationOutMsg.recorder()
    theta_data_log = prescribed_rot_1_dof.spinningBodyOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, sc_state_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, prescribed_rot_state_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, theta_data_log)

    # Initialize the simulation
    unit_test_sim.InitializeSimulation()

    # Set the simulation time
    sim_time = 15  # [s]
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))

    # Begin the simulation
    unit_test_sim.ExecuteSimulation()

    # Extract the logged data
    orb_energy = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(), sc_energy_momentum_log.totOrbEnergy)
    orb_ang_mom_n = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(), sc_energy_momentum_log.totOrbAngMomPntN_N)
    rot_ang_mom_n = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(), sc_energy_momentum_log.totRotAngMomPntC_N)
    rot_energy = unitTestSupport.addTimeColumn(sc_energy_momentum_log.times(), sc_energy_momentum_log.totRotEnergy)
    timespan = sc_state_data_log.times() * macros.NANO2SEC  # [s]
    omega_bn_b = sc_state_data_log.omega_BN_B * macros.R2D  # [deg]
    r_bn_n = sc_state_data_log.r_BN_N
    sigma_bn = sc_state_data_log.sigma_BN
    theta = macros.R2D * theta_data_log.theta  # [deg]
    omega_fm_f = prescribed_rot_state_data_log.omega_FM_F * macros.R2D  # [deg]
    omega_prime_fm_f = prescribed_rot_state_data_log.omegaPrime_FM_F * macros.R2D  # [deg]
    theta_dot_final = np.linalg.norm(omega_fm_f[-1, :]) * macros.R2D  # [deg]
    theta_final = theta[-1]  # [deg]

    # Setup the conservation quantities
    orb_ang_mom_init = [[orb_ang_mom_n[0, 1], orb_ang_mom_n[0, 2], orb_ang_mom_n[0, 3]]]
    orb_ang_mom_final = [orb_ang_mom_n[-1]]
    rot_ang_mom_init = [[rot_ang_mom_n[0, 1], rot_ang_mom_n[0, 2], rot_ang_mom_n[0, 3]]]
    rot_ang_mom_final = [rot_ang_mom_n[-1]]
    orb_energy_init = [[orb_energy[0, 1]]]
    orb_energy_final = [orb_energy[-1]]

    # Plot theta_FM
    theta_ref_plotting = np.ones(len(timespan)) * theta_ref * macros.R2D  # [deg]
    theta_init_plotting = np.ones(len(timespan)) * theta_init * macros.R2D  # [deg]
    plt.figure()
    plt.clf()
    plt.plot(timespan, theta, label=r'$\theta$')
    plt.plot(timespan, theta_ref_plotting, '--', label=r'$\theta_{Ref}$')
    plt.plot(timespan, theta_init_plotting, '--', label=r'$\theta_{0}$')
    plt.title(r'$\theta$ Profiled Trajectory', fontsize=16)
    plt.ylabel('(deg)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='center right', prop={'size': 16})
    plt.grid(True)

    # Plot omega_fm_f
    plt.figure()
    plt.clf()
    plt.plot(timespan, omega_fm_f[:, 0], label=r'$\omega_{1}$')
    plt.plot(timespan, omega_fm_f[:, 1], label=r'$\omega_{2}$')
    plt.plot(timespan, omega_fm_f[:, 2], label=r'$\omega_{3}$')
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

    # # Plot r_bn_n
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, r_bn_n[:, 0], label=r'$r_{1}$')
    # plt.plot(timespan, r_bn_n[:, 1], label=r'$r_{2}$')
    # plt.plot(timespan, r_bn_n[:, 2], label=r'$r_{3}$')
    # plt.title(r'${}^\mathcal{N} r_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial Trajectory', fontsize=16)
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
    # plt.title(r'$\sigma_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial MRP Attitude', fontsize=16)
    # plt.ylabel('', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.legend(loc='center left', prop={'size': 16})
    # plt.grid(True)
    #
    # # Plot omega_bn_b
    # plt.figure()
    # plt.clf()
    # plt.plot(timespan, omega_bn_b[:, 0], label=r'$\omega_{1}$')
    # plt.plot(timespan, omega_bn_b[:, 1], label=r'$\omega_{2}$')
    # plt.plot(timespan, omega_bn_b[:, 2], label=r'$\omega_{3}$')
    # plt.title(r'Spacecraft Hub Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$', fontsize=16)
    # plt.xlabel('Time (s)', fontsize=16)
    # plt.ylabel('(deg/s)', fontsize=16)
    # plt.legend(loc='lower right', prop={'size': 16})
    # plt.grid(True)

    # Plot conservation quantities
    plt.figure()
    plt.clf()
    plt.plot(timespan, (orb_ang_mom_n[:, 1] - orb_ang_mom_n[0, 1]) / orb_ang_mom_n[0, 1],
             timespan, (orb_ang_mom_n[:, 2] - orb_ang_mom_n[0, 2]) / orb_ang_mom_n[0, 2],
             timespan, (orb_ang_mom_n[:, 3] - orb_ang_mom_n[0, 3]) / orb_ang_mom_n[0, 3])
    plt.title('Orbital Angular Momentum Relative Difference', fontsize=16)
    plt.ylabel('(Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    plt.figure()
    plt.clf()
    plt.plot(timespan, (orb_energy[:, 1] - orb_energy[0, 1]) / orb_energy[0, 1])
    plt.title('Orbital Energy Relative Difference', fontsize=16)
    plt.ylabel('Energy (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    plt.figure()
    plt.clf()
    plt.plot(timespan, (rot_ang_mom_n[:, 1] - rot_ang_mom_n[0, 1]),
             timespan, (rot_ang_mom_n[:, 2] - rot_ang_mom_n[0, 2]),
             timespan, (rot_ang_mom_n[:, 3] - rot_ang_mom_n[0, 3]))
    plt.title('Rotational Angular Momentum Difference', fontsize=16)
    plt.ylabel('(Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    plt.figure()
    plt.clf()
    plt.plot(timespan, (rot_energy[:, 1] - rot_energy[0, 1]))
    plt.title('Total Energy Difference', fontsize=16)
    plt.ylabel('Energy (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

    # Begin the test analysis
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

    # Check to ensure the initial angle rate converged to the reference angle rate
    np.testing.assert_allclose(0.0,
                               theta_dot_final,
                               atol=accuracy,
                               verbose=True)

    # Check to ensure the initial angle converged to the reference angle
    np.testing.assert_allclose(theta_ref * macros.R2D,
                               theta_final,
                               atol=accuracy,
                               verbose=True)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_PrescribedMotionStateEffector_Rotation(
        True,  # show_plots
        0.0,  # theta_init [rad]
        10.0 * macros.D2R,  # theta_ref [rad]
        1e-8  # accuracy
    )
