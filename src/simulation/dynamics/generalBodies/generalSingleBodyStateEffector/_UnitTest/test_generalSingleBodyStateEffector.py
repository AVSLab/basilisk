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
#   Unit Test Script
#   Module Name:        generalSingleBodyStateEffector
#   Author:             Leah Kiner
#   Creation Date:      February 19, 2026
#

import inspect
import os
import matplotlib.pyplot as plt
import numpy
import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import spacecraft, generalSingleBodyStateEffector, gravityEffector
from Basilisk.architecture import messaging

@pytest.mark.parametrize("dof_1_rot, dof_1_trans, dof_2_rot, dof_2_trans, dof_3_rot, dof_3_trans", [
    (1, 0, 0, 0, 0, 0)
    , (0, 1, 0, 0, 0, 0)
    , (1, 0, 1, 0, 0, 0)
    , (0, 1, 0, 1, 0, 0)
    , (1, 0, 0, 1, 0, 0)
    , (0, 1, 1, 0, 0, 0)
    , (1, 0, 1, 0, 1, 0)
    , (0, 1, 0, 1, 0, 1)
    , (1, 0, 1, 0, 0, 1)
    , (1, 0, 0, 1, 1, 0)
    , (0, 1, 1, 0, 1, 0)
    , (0, 1, 0, 1, 1, 0)
    , (0, 1, 1, 0, 0, 1)
    , (1, 0, 0, 1, 0, 1)
    , (1, 1, 1, 1, 0, 0)
    , (1, 1, 1, 1, 1, 0)
    , (1, 1, 1, 1, 1, 1)
])

def test_general_single_body_state_effector(show_plots,
                                            dof_1_rot,
                                            dof_1_trans,
                                            dof_2_rot,
                                            dof_2_trans,
                                            dof_3_rot,
                                            dof_3_trans):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft with a general rigid body attached to a rigid hub.
    Earth's gravity is included in the simulation. The test configures one through six degrees of freedom for the
    general body. Rotation and translation combinations cover one through three degrees of freedom, with mixed cases
    exercising four, five, and six degrees of freedom. Conservation of energy and angular momentum verifies the dynamics.

    **Description of Variables Being Tested**

    The tests with conservative forces (only Earth's gravity) check that both the orbital and rotational quantities of
    energy and angular momentum are maintained within a strict tolerance. For the tests with nonconservative forces,
    the spacecraft rotational energy is not checked.
    """
    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.0001  # [s]
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Create the spacecraft module
    sc_object = create_spacecraft_hub()
    test_sim.AddModelToTask(task_name, sc_object)

    # Add Earth gravity to the simulation
    earth = gravityEffector.GravBodyData()
    earth.planetName = "earth_planet_data"
    earth.mu = 0.3986004415E+15
    earth.isCentralBody = True
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([earth])

    # Create the general effector
    general_body = create_general_effector()
    dof_1_hat_G = np.array([1.0, 0.0, 0.0])
    if dof_1_rot:
        one_dof_rotation = create_rotational_dof(dof_1_hat_G)
        general_body.addRotDOF(one_dof_rotation)
    if dof_1_trans:
        one_dof_translation = create_translational_dof(dof_1_hat_G)
        general_body.addTransDOF(one_dof_translation)
    dof_2_hat_G = np.array([0.0, 1.0, 0.0])
    if dof_2_rot:
        one_dof_rotation = create_rotational_dof(dof_2_hat_G)
        general_body.addRotDOF(one_dof_rotation)
    if dof_2_trans:
        one_dof_translation = create_translational_dof(dof_2_hat_G)
        general_body.addTransDOF(one_dof_translation)
    dof_3_hat_G = np.array([0.0, 0.0, 1.0])
    if dof_3_rot:
        one_dof_rotation = create_rotational_dof(dof_3_hat_G)
        general_body.addRotDOF(one_dof_rotation)
    if dof_3_trans:
        one_dof_translation = create_translational_dof(dof_3_hat_G)
        general_body.addTransDOF(one_dof_translation)
    sc_object.addStateEffector(general_body)
    test_sim.AddModelToTask(task_name, general_body)

    # Set up data logging
    energy_momentum_data_log = sc_object.logger(["totRotEnergy", "totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N"])
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    general_body_rot_states_data_log = []
    general_body_trans_states_data_log = []
    for rotOutMsg in general_body.spinningBodyOutMsgs:
        general_body_rot_states_data_log.append(rotOutMsg.recorder())
        test_sim.AddModelToTask(task_name, general_body_rot_states_data_log[-1])
    for transOutMsg in general_body.translatingBodyOutMsgs:
        general_body_trans_states_data_log.append(transOutMsg.recorder())
        test_sim.AddModelToTask(task_name, general_body_trans_states_data_log[-1])

    test_sim.AddModelToTask(task_name, energy_momentum_data_log)
    test_sim.AddModelToTask(task_name, sc_state_data_log)

    # Run the simulation
    test_sim.InitializeSimulation()
    sim_time_sec = 3.0  # [s]
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time_sec))
    test_sim.ExecuteSimulation()

    # Extract logged data
    timespan = sc_state_data_log.times() * macros.NANO2SEC  # [s]
    orb_energy = energy_momentum_data_log.totOrbEnergy  # [J]
    orb_ang_momentum_N = energy_momentum_data_log.totOrbAngMomPntN_N  # [N m s]
    rot_ang_momentum_N = energy_momentum_data_log.totRotAngMomPntC_N  # [N m s]
    rot_energy = energy_momentum_data_log.totRotEnergy  # [J]
    theta = []  # [deg]
    theta_dot = []  # [deg/s]
    rho = []  # [m]
    rho_dot = []  # [m/s]
    for rotData in general_body_rot_states_data_log:
        theta.append(rotData.theta * macros.R2D)
        theta_dot.append(rotData.thetaDot * macros.R2D)
    for transData in general_body_trans_states_data_log:
        rho.append(transData.rho)
        rho_dot.append(transData.rhoDot)

    # Plot results
    if dof_1_rot + dof_2_rot + dof_3_rot != 0:
        plot_rotational_states(timespan, theta, theta_dot)
    if dof_1_trans + dof_2_trans + dof_3_trans != 0:
        plot_translational_states(timespan, rho, rho_dot)
    plot_conservation(timespan,
                      orb_ang_momentum_N,
                      orb_energy,
                      rot_ang_momentum_N,
                      rot_energy)

    if show_plots:
        plt.show()
    plt.close("all")

    # Unit test check
    unit_test_verification_check(orb_ang_momentum_N, orb_energy, rot_ang_momentum_N, rot_energy)


def create_spacecraft_hub():
    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "scObject"
    sc_object.hub.mHub = 750.0  # [kg]
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]  # [m]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg m^2]
    sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]  # [m]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]  # [m/s]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]  # [rad/s]

    return sc_object

def create_general_effector():
    r_G0B_B = np.array([-0.1, 0.1, 0.1])  # [m]
    dcm_G0B = np.array([[0.0, -1.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [1.0, 0.0, 0.0]])
    general_body = generalSingleBodyStateEffector.GeneralSingleBodyStateEffector()
    general_body.ModelTag = "generalBody"
    general_body.setMass(20.0)  # [kg]
    general_body.setIPntGc_G([[50.0, 0.0, 0.0],
                              [0.0, 80.0, 0.0],
                              [0.0, 0.0, 60.0]])  # [kg m^2]
    general_body.setR_GcG_G(np.array([0.1, -0.1, 0.1]))  # [m]
    general_body.setR_G0B_B(r_G0B_B)  # [m]
    general_body.setDCM_G0B(dcm_G0B)

    return general_body

def create_rotational_dof(rot_hat_G):
    theta_init = 5.0 * macros.D2R  # [rad]
    spring_constant_k = 100.0  # [N m/rad]
    one_dof_rotation = generalSingleBodyStateEffector.DOF()
    one_dof_rotation.setDOFAxis(rot_hat_G)
    one_dof_rotation.setBetaInit(theta_init)  # [rad]
    one_dof_rotation.setSpringConstantK(spring_constant_k)  # [N m/rad]

    return one_dof_rotation

def create_translational_dof(trans_hat_G):
    rho_init = 0.1  # [m]
    spring_constant_k = 100.0  # [N m/rad]
    one_dof_translation = generalSingleBodyStateEffector.DOF()
    one_dof_translation.setDOFAxis(trans_hat_G)
    one_dof_translation.setBetaInit(rho_init)  # [m]
    one_dof_translation.setSpringConstantK(spring_constant_k)  # [N m/rad]

    return one_dof_translation

def plot_rotational_states(timespan, theta, theta_dot):
    plt.figure()
    plt.clf()
    for idx, angle in enumerate(theta):
        plt.plot(timespan, angle, label=r'$\theta_' + str(idx + 1) + '$')
    plt.title(r'General Body Angle')
    plt.ylabel('Angle (deg)')
    plt.xlabel('Time (sec)')
    plt.legend(loc='center right')
    plt.grid(True)

    plt.figure()
    plt.clf()
    for idx, angle_rate in enumerate(theta_dot):
        plt.plot(timespan, angle_rate, label=r'$\dot{\theta}_' + str(idx + 1) + '$')
    plt.title(r'General Body Angle Rate')
    plt.ylabel('Angle Rate (deg/s)')
    plt.xlabel('Time (sec)')
    plt.legend(loc='center right')
    plt.grid(True)

def plot_translational_states(timespan, rho, rho_dot):
    plt.figure()
    plt.clf()
    for idx, disp in enumerate(rho):
        plt.plot(timespan, disp, label=r'$\rho_' + str(idx + 1) + '$')
    plt.title(r'General Body Displacement')
    plt.ylabel('Displacement (m)')
    plt.xlabel('Time (sec)')
    plt.legend(loc='center right')
    plt.grid(True)

    plt.figure()
    plt.clf()
    for idx, rate in enumerate(rho_dot):
        plt.plot(timespan, rate, label=r'$\dot{\rho}_' + str(idx + 1) + '$')
    plt.title(r'General Body Displacement Rate')
    plt.ylabel('Velocity (m/s)')
    plt.xlabel('Time (sec)')
    plt.legend(loc='center right')
    plt.grid(True)

def plot_conservation(timespan, orb_ang_momentum_N, orb_energy, rot_ang_momentum_N, rot_energy):
    # Plot orbital angular momentum relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (orb_ang_momentum_N[:, 0] - orb_ang_momentum_N[0, 0]) / orb_ang_momentum_N[0, 0], color="teal", label=r'$\hat{n}_1$')
    plt.plot(timespan, (orb_ang_momentum_N[:, 1] - orb_ang_momentum_N[0, 1]) / orb_ang_momentum_N[0, 1], color="darkviolet", label=r'$\hat{n}_2$')
    plt.plot(timespan, (orb_ang_momentum_N[:, 2] - orb_ang_momentum_N[0, 2]) / orb_ang_momentum_N[0, 2], color="blue", label=r'$\hat{n}_3$')
    plt.title('Orbital Angular Momentum')
    plt.ylabel('Relative Difference (Nms)')
    plt.xlabel('Time (s)')
    plt.legend(loc='center right')
    plt.grid(True)

    # Plot orbital energy relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (orb_energy - orb_energy[0]) / orb_energy[0], color="teal")
    plt.title('Orbital Energy')
    plt.ylabel('Relative Difference (J)')
    plt.xlabel('Time (s)')
    plt.grid(True)

    # Plot sc angular momentum relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (rot_ang_momentum_N[:, 0] - rot_ang_momentum_N[0, 0]) / rot_ang_momentum_N[0, 0], color="teal", label=r'$\hat{n}_1$')
    plt.plot(timespan, (rot_ang_momentum_N[:, 1] - rot_ang_momentum_N[0, 1]) / rot_ang_momentum_N[0, 1], color="darkviolet", label=r'$\hat{n}_2$')
    plt.plot(timespan, (rot_ang_momentum_N[:, 2] - rot_ang_momentum_N[0, 2]) / rot_ang_momentum_N[0, 2], color="blue", label=r'$\hat{n}_3$')
    plt.title('Rotational Angular Momentum')
    plt.ylabel('Relative Difference (Nms)')
    plt.xlabel('Time (s)')
    plt.legend(loc='center right')
    plt.grid(True)

    # Plot sc energy relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (rot_energy - rot_energy[0]) / rot_energy[0], color="teal")
    plt.title('Rotational Energy')
    plt.ylabel('Relative Difference (J)')
    plt.xlabel('Time (s)')
    plt.grid(True)

def unit_test_verification_check(orb_ang_momentum_N, orb_energy, rot_ang_momentum_N, rot_energy):
    accuracy = 1e-13
    np.testing.assert_allclose(orb_ang_momentum_N[0], orb_ang_momentum_N[-1], rtol=accuracy, verbose=True)
    np.testing.assert_allclose(orb_energy[0], orb_energy[-1], rtol=accuracy, verbose=True)
    np.testing.assert_allclose(rot_ang_momentum_N[0], rot_ang_momentum_N[-1], rtol=accuracy, verbose=True)
    np.testing.assert_allclose(rot_energy[0], rot_energy[-1], rtol=1e-13, verbose=True)

if __name__ == "__main__":
    test_general_single_body_state_effector(True, 1, 0, 0, 0, 0, 0)
    test_general_single_body_state_effector(True, 0, 1, 0, 0, 0, 0)
    test_general_single_body_state_effector(True, 1, 0, 1, 0, 0, 0)
    test_general_single_body_state_effector(True, 0, 1, 0, 1, 0, 0)
    test_general_single_body_state_effector(True, 1, 0, 0, 1, 0, 0)
    test_general_single_body_state_effector(True, 0, 1, 1, 0, 0, 0)
    test_general_single_body_state_effector(True, 1, 0, 1, 0, 1, 0)
    test_general_single_body_state_effector(True, 0, 1, 0, 1, 0, 1)
    test_general_single_body_state_effector(True, 1, 0, 1, 0, 0, 1)
    test_general_single_body_state_effector(True, 1, 0, 0, 1, 1, 0)
    test_general_single_body_state_effector(True, 0, 1, 1, 0, 1, 0)
    test_general_single_body_state_effector(True, 0, 1, 0, 1, 1, 0)
    test_general_single_body_state_effector(True, 0, 1, 1, 0, 0, 1)
    test_general_single_body_state_effector(True, 1, 0, 0, 1, 0, 1)
    test_general_single_body_state_effector(True, 1, 1, 1, 1, 0, 0)
    test_general_single_body_state_effector(True, 1, 1, 1, 1, 1, 0)
    test_general_single_body_state_effector(True, 1, 1, 1, 1, 1, 1)
