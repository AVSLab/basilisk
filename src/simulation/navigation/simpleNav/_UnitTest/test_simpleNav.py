# ISC License
#
# Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, CU Boulder
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


import math
import numpy as np
import matplotlib.pyplot as plt
from Basilisk.architecture import messaging
from Basilisk.simulation import simpleNav
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import RigidBodyKinematics as rbk


def test_long_sim(show_plots):
    """This unit test integrates the spacecraft motion (changing attutide/rate and postition/velocity)
    and ensures that the simple navigation output is consistent with the noise inputs"""
    simple_nav_sim(show_plots, noise=True, number_steps=1000)


def test_no_noise(show_plots):
    """This unit test reads a single spacecraft message and checks the output in the case
     where no noise is added"""
    simple_nav_sim(show_plots, noise=False)


def test_include_noise(show_plots):
    """This unit test reads a single spacecraft message and checks the output in the case
     where noise is added.
     The output is ensured to be within 3-sigma of the input, where sigma is the standard deviation
     input into the simple nav Gauss-Markov process"""
    simple_nav_sim(show_plots, noise=True)


def test_errors_present(show_plots):
    """This unit ensures that the errors are added by checking the number of excursions due to the
    white and brown noise added by the Gauss Markov model"""
    simple_nav_sim(show_plots, noise=True, number_steps=1000, sigma_test=0)


def simple_nav_sim(show_plots, noise=False, number_steps=1, sigma_test=3):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    unit_test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    unit_test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, int(1E8)))

    simple_nav_object = simpleNav.SimpleNav()
    unit_test_sim.AddModelToTask(unit_task_name, simple_nav_object)

    spice_message = messaging.SpicePlanetStateMsgPayload()
    state_message = messaging.SCStatesMsgPayload()
    sun_position = [1e10, 1e6, 1e2]
    vehicle_position = [-3e6, 1e3, -2e1]
    vehicle_velocity = [2.0, 3.0, 1.0]
    vehicle_attitude = [0.3, 0.1, 0.1]
    vehicle_rate = [0.01, 0.02, 0.03]
    sun_heading_B = np.array(sun_position) - np.array(vehicle_position)
    sun_heading_B = np.dot(rbk.MRP2C(vehicle_attitude), sun_heading_B / np.linalg.norm(sun_heading_B))

    state_message.r_BN_N = vehicle_position
    state_message.v_BN_N = vehicle_velocity
    state_message.sigma_BN = vehicle_attitude
    state_message.omega_BN_B = vehicle_rate
    spice_message.PositionVector = sun_position
    spice_message.PlanetName = "sun"

    # Inertial State output Message
    spacecraft_state_message = messaging.SCStatesMsg().write(state_message)
    simple_nav_object.scStateInMsg.subscribeTo(spacecraft_state_message)

    # Sun Planet Data Message
    sun_state_message = messaging.SpicePlanetStateMsg().write(spice_message)
    simple_nav_object.sunStateInMsg.subscribeTo(sun_state_message)

    simple_nav_object.ModelTag = "SimpleNavigation"

    # Set bounds for noise parameters
    position_bounds = 1000.0
    velocity_bounds = 1.0
    attitude_bounds = 5E-3
    rate_bounds = 0.02
    sun_bounds = 5.0 * math.pi / 180.0
    dv_bounds = 0.053

    position_sigma = 5.0
    velocity_sigma = 0.035
    attitude_sigma = 1.0 / 360.0 * math.pi / 180.0
    rate_sigma = 0.05 * math.pi / 180.0
    sun_sigma = math.pi / 180.0
    dv_sigma = 0.1 * math.pi / 180.0

    p_matrix = np.diag([position_sigma] * 3 + [velocity_sigma] * 3 + [attitude_sigma] * 3 +
                       [rate_sigma] * 3 + [sun_sigma] * 3 + [dv_sigma] * 3)
    error_bounds = [position_bounds] * 3 + [velocity_bounds] * 3 + [attitude_bounds] * 3 + \
                   [rate_bounds] * 3 + [sun_bounds] * 3 + [dv_bounds] * 3

    if noise and number_steps == 1:
        # Zero error bounds in order to just test against the standard deviation variation
        position_bounds = 0
        velocity_bounds = 0
        attitude_bounds = 0
        rate_bounds = 0
        sun_bounds = 0
        dv_bounds = 0
    elif not noise:
        # If no noise is present, set the bounds such that the numpy test have low absolute tolerance
        position_sigma = np.finfo(float).eps
        velocity_sigma = np.finfo(float).eps
        attitude_sigma = np.finfo(float).eps
        rate_sigma = np.finfo(float).eps
        sun_sigma = np.finfo(float).eps
        dv_sigma = np.finfo(float).eps
        p_matrix = np.diag([position_sigma] * 3 + [velocity_sigma] * 3 + [attitude_sigma] * 3 +
                           [rate_sigma] * 3 + [sun_sigma] * 3 + [dv_sigma] * 3)
        error_bounds = [position_sigma] * 3 + [velocity_sigma] * 3 + [attitude_sigma] * 3 + \
                       [rate_sigma] * 3 + [sun_sigma] * 3 + [dv_sigma] * 3

    simple_nav_object.walkBounds = error_bounds
    simple_nav_object.PMatrix = p_matrix.tolist()
    simple_nav_object.crossTrans = True
    simple_nav_object.crossAtt = False

    # setup logging
    rotational_data_log = simple_nav_object.attOutMsg.recorder()
    translational_data_log = simple_nav_object.transOutMsg.recorder()
    ephemeris_data_log = simple_nav_object.scEphemOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, rotational_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, translational_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, ephemeris_data_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(int(number_steps * 1E8))
    unit_test_sim.ExecuteSimulation()

    # pull simulation data
    position_output = translational_data_log.r_BN_N
    velocity_output = translational_data_log.v_BN_N
    dv_output = translational_data_log.vehAccumDV

    attitude_output = rotational_data_log.sigma_BN
    rate_output = rotational_data_log.omega_BN_B
    sun_output = rotational_data_log.vehSunPntBdy

    ephemeris_position_output = ephemeris_data_log.r_BdyZero_N
    ephemeris_velocity_output = ephemeris_data_log.v_BdyZero_N
    ephemeris_attitude_output = ephemeris_data_log.sigma_BN
    ephemeris_rate_output = ephemeris_data_log.omega_BN_B

    # If the sigma is non-zero ensure errors are below the expected bounds, if not ensure errors are large enough
    if sigma_test != 0:
        np.testing.assert_allclose(ephemeris_position_output[number_steps, :],
                                   vehicle_position,
                                   atol=position_bounds + sigma_test * position_sigma,
                                   equal_nan=False,
                                   err_msg="Ephem position and true translational navigation message must be equal",
                                   verbose=True)
        np.testing.assert_allclose(ephemeris_velocity_output[number_steps, :],
                                   vehicle_velocity,
                                   rtol=velocity_bounds + sigma_test * velocity_sigma,
                                   equal_nan=False,
                                   err_msg="Ephem velocity and true translational navigation message must be equal",
                                   verbose=True)
        np.testing.assert_allclose(ephemeris_attitude_output[number_steps, :],
                                   vehicle_attitude,
                                   atol=attitude_bounds + sigma_test * attitude_sigma,
                                   equal_nan=False,
                                   err_msg="Ephem attitude and true translational navigation message must be equal",
                                   verbose=True)
        np.testing.assert_allclose(ephemeris_rate_output[number_steps, :],
                                   vehicle_rate,
                                   atol=rate_bounds + sigma_test * rate_sigma,
                                   equal_nan=False,
                                   err_msg="Ephem body rate and true translational navigation message must be equal",
                                   verbose=True)

        np.testing.assert_allclose(position_output[number_steps, :],
                                   vehicle_position,
                                   atol=position_bounds + sigma_test * position_sigma,
                                   equal_nan=False,
                                   err_msg="Output position must be within error bounds",
                                   verbose=True)
        np.testing.assert_allclose(velocity_output[number_steps, :],
                                   vehicle_velocity,
                                   atol=velocity_bounds + sigma_test * velocity_sigma,
                                   equal_nan=False,
                                   err_msg="Output velocity must be within error bounds",
                                   verbose=True)
        np.testing.assert_allclose(attitude_output[number_steps, :],
                                   vehicle_attitude,
                                   atol=attitude_bounds + sigma_test * attitude_sigma,
                                   equal_nan=False,
                                   err_msg="Output attitude must be within error bounds",
                                   verbose=True)
        np.testing.assert_allclose(rate_output[number_steps, :],
                                   vehicle_rate,
                                   atol=rate_bounds + sigma_test * rate_sigma,
                                   equal_nan=False,
                                   err_msg="Output rates must be within error bounds",
                                   verbose=True)
        # simple nav perturbs the sun heading by creating an MRP with the random values of Gauss Markov
        # and rotating the heading about that rotation. Therefore, check that the dot product of the output
        # and test is within 4*atan(|sigma|) = Phi
        dot_product = np.dot(sun_output[number_steps, :] / np.linalg.norm(sun_output[number_steps, :]), sun_heading_B)
        clipped_product = np.clip(dot_product, -1, 1)
        np.testing.assert_array_less(np.arccos(clipped_product),
                                     4 * np.arctan(sun_bounds + sigma_test * sun_sigma),
                                     err_msg="Output sun heading must be within error bounds",
                                     verbose=True)
        np.testing.assert_allclose(dv_output[number_steps, :],
                                   np.zeros(3),
                                   atol=dv_bounds + sigma_test * dv_sigma,
                                   equal_nan=False,
                                   err_msg="Output dv must be within error bounds",
                                   verbose=True)
    else:
        bound_fraction = 0.1
        excursion_fraction = 0.25
        ephem_position_excursion = np.where(np.linalg.norm(ephemeris_position_output -
                                                           vehicle_position,
                                                           axis=1) > position_bounds * bound_fraction)[0]
        np.testing.assert_array_less(len(ephemeris_position_output) * excursion_fraction,
                                     len(ephem_position_excursion),
                                     err_msg="Ephemeris position errors were not added as expected",
                                     verbose=True)
        ephem_velocity_excursion = np.where(np.linalg.norm(ephemeris_velocity_output -
                                                           vehicle_velocity,
                                                           axis=1) > velocity_bounds * bound_fraction)[0]
        np.testing.assert_array_less(len(ephemeris_velocity_output) * excursion_fraction,
                                     len(ephem_velocity_excursion),
                                     err_msg="Ephemeris velocity errors were not added as expected",
                                     verbose=True)
        ephem_attitude_excursion = np.where(np.linalg.norm(ephemeris_attitude_output -
                                                           vehicle_attitude,
                                                           axis=1) > attitude_bounds * bound_fraction)[0]
        np.testing.assert_array_less(len(ephemeris_attitude_output) * excursion_fraction,
                                     len(ephem_attitude_excursion),
                                     err_msg="Ephemeris attitude errors were not added as expected",
                                     verbose=True)
        ephem_rate_excursion = np.where(np.linalg.norm(ephemeris_rate_output -
                                                       vehicle_rate, axis=1) > rate_bounds * bound_fraction)[0]
        np.testing.assert_array_less(len(ephemeris_rate_output) * excursion_fraction,
                                     len(ephem_rate_excursion),
                                     err_msg="Ephemeris rate errors were not added as expected",
                                     verbose=True)
        position_excursion = np.where(np.linalg.norm(position_output -
                                                     vehicle_position, axis=1) > position_bounds * bound_fraction)[0]
        np.testing.assert_array_less(len(position_output) * excursion_fraction,
                                     len(position_excursion),
                                     err_msg="Position errors were not added as expected",
                                     verbose=True)
        velocity_excursion = np.where(np.linalg.norm(velocity_output -
                                                     vehicle_velocity, axis=1) > velocity_bounds * bound_fraction)[0]
        np.testing.assert_array_less(len(velocity_output) * excursion_fraction,
                                     len(velocity_excursion),
                                     err_msg="Velocity errors were not added as expected",
                                     verbose=True)
        attitude_excursion = np.where(np.linalg.norm(attitude_output -
                                                     vehicle_attitude, axis=1) > attitude_bounds * bound_fraction)[0]
        np.testing.assert_array_less(len(attitude_output) * excursion_fraction,
                                     len(attitude_excursion),
                                     err_msg="Attitude errors were not added as expected",
                                     verbose=True)
        rate_excursion = np.where(np.linalg.norm(rate_output -
                                                 vehicle_rate, axis=1) > rate_bounds * bound_fraction)[0]
        np.testing.assert_array_less(len(rate_output) * excursion_fraction,
                                     len(rate_excursion),
                                     err_msg="Attitude errors were not added as expected",
                                     verbose=True)
        dv_excursion = np.where(np.linalg.norm(dv_output, axis=1) > 0)[0]
        np.testing.assert_array_less(len(dv_output) * excursion_fraction,
                                     len(dv_excursion),
                                     err_msg="DV errors were not added as expected",
                                     verbose=True)

    if show_plots and number_steps > 1:
        plt.clf()
        plt.figure(1, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(3, 1, 1)
        plt.plot(translational_data_log.times() * 1.0E-9, position_output[:, 0])
        plt.ylabel('X-Position (m)')
        plt.subplot(3, 1, 2)
        plt.plot(translational_data_log.times() * 1.0E-9, position_output[:, 1])
        plt.ylabel('Y-Position (m)')
        plt.subplot(3, 1, 3)
        plt.plot(translational_data_log.times() * 1.0E-9, position_output[:, 2])
        plt.xlabel('Time (s)')
        plt.ylabel('Z-Position (m)')

        plt.figure(2, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.subplot(3, 1, 1)
        plt.plot(rotational_data_log.times() * 1.0E-9, attitude_output[:, 0])
        plt.ylabel('X-rotation (-)')
        plt.subplot(3, 1, 2)
        plt.plot(rotational_data_log.times() * 1.0E-9, attitude_output[:, 1])
        plt.ylabel('Y-rotation (-)')
        plt.subplot(3, 1, 3)
        plt.plot(rotational_data_log.times() * 1.0E-9, attitude_output[:, 2])
        plt.ylabel('Z-rotation (-)')
        plt.xlabel('Time (s)')
        plt.ylabel('Attitude MRP')
        plt.show()
        plt.close('all')
    return


if __name__ == "__main__":
    simple_nav_sim(True, True, number_steps=1000)
