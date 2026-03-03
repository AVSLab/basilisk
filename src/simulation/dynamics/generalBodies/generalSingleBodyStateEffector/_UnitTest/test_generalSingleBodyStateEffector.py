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

test_time_step_sec = 0.0001
# @pytest.mark.parametrize("screw_constant", [0.5, 1.0, 1.5])
def test_general_one_dof_rotation(show_plots):
    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Create the spacecraft module
    sc_object = create_spacecraft_hub()
    test_sim.AddModelToTask(task_name, sc_object)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Create the general effector
    general_body = generalSingleBodyStateEffector.GeneralSingleBodyStateEffector()
    general_body.ModelTag = "generalBody"
    general_body.setMass(np.random.uniform(5.0, 50.0))
    general_body.setIPntGc_G([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                              [0.0, np.random.uniform(5.0, 100.0), 0.0],
                              [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    general_body.setR_GcG_G([[np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)],
                             [np.random.uniform(-1.0, 1.0)]])

    rotHat_G = np.array([[np.sqrt(1/2)],
                         [np.sqrt(1/2)],
                         [0]])
    r_G0B_B = [[np.random.uniform(-1.0, 1.0)],
               [np.random.uniform(-1.0, 1.0)],
               [np.random.uniform(-1.0, 1.0)]]
    dcm_G0B = np.array([[1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, -1.0]])
    thetaInit = np.random.uniform(-10.0, 10.0) * macros.D2R
    thetaDotInit = 0.0
    spring_constant_k = np.random.uniform(50.0, 100.0)
    damper_constant_c = 0.0

    one_dof_rotation = generalSingleBodyStateEffector.DOF()
    one_dof_rotation.setDOFAxis(rotHat_G)
    one_dof_rotation.setR_G0P_P(r_G0B_B)
    one_dof_rotation.setDCM_G0P(dcm_G0B)
    one_dof_rotation.setBetaInit(thetaInit)
    one_dof_rotation.setBetaDotInit(thetaDotInit)
    one_dof_rotation.setSpringConstantK(spring_constant_k)
    one_dof_rotation.setDampingConstantK(damper_constant_c)
    general_body.addRotationalDOF(one_dof_rotation)
    sc_object.addStateEffector(general_body)
    test_sim.AddModelToTask(task_name, general_body)

    # Set up data logging
    energy_momentum_data_log = sc_object.logger(["totRotEnergy", "totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N"])
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    general_body_theta_states_data_log = []
    for outMsg in general_body.spinningBodyOutMsgs:
        general_body_theta_states_data_log.append(outMsg.recorder())
        test_sim.AddModelToTask(task_name, general_body_theta_states_data_log[-1])
    test_sim.AddModelToTask(task_name, energy_momentum_data_log)
    test_sim.AddModelToTask(task_name, sc_state_data_log)

    # Rum the simulation
    sim_time = 1.0
    test_sim.InitializeSimulation()
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    test_sim.ExecuteSimulation()

    # Extract logged data
    timespan = sc_state_data_log.times() * macros.NANO2SEC
    orb_energy = energy_momentum_data_log.totOrbEnergy
    orb_ang_momentum_N = energy_momentum_data_log.totOrbAngMomPntN_N
    rot_ang_momentum_N = energy_momentum_data_log.totRotAngMomPntC_N
    rot_energy = energy_momentum_data_log.totRotEnergy
    theta = []
    theta_dot = []
    for data in general_body_theta_states_data_log:
        theta.append(data.theta * macros.R2D)
        theta_dot.append(data.thetaDot * macros.R2D)

    # Plot results
    plot_conservation(timespan,
                      orb_ang_momentum_N,
                      orb_energy,
                      rot_ang_momentum_N,
                      rot_energy)

    # Plot general body theta
    plt.figure(5)
    plt.clf()
    for idx, angle in enumerate(theta):
        plt.plot(timespan, angle, label=r'$\theta_' + str(idx + 1) + '$')
    plt.title(r'General Body Angle', fontsize=14)
    plt.ylabel('Angle (deg)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    # Plot general body thetaDot
    plt.figure(6)
    plt.clf()
    for idx, angle_rate in enumerate(theta_dot):
        plt.plot(timespan, angle_rate, label=r'$\dot{\theta}_' + str(idx + 1) + '$')
    plt.title(r'General Body Angle Rate', fontsize=14)
    plt.ylabel('Angle Rate (deg/s)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

    # Unit test check
    unit_test_verification_check(orb_ang_momentum_N, orb_energy, rot_ang_momentum_N, rot_energy)

# @pytest.mark.parametrize("screw_constant", [0.5, 1.0, 1.5])
def test_general_one_dof_translation(show_plots):
    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Create the spacecraft module
    sc_object = create_spacecraft_hub()
    test_sim.AddModelToTask(task_name, sc_object)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Create the general effector
    general_body = generalSingleBodyStateEffector.GeneralSingleBodyStateEffector()
    general_body.ModelTag = "generalBody"
    general_body.setMass(20)
    general_body.setIPntGc_G([[50.0, 0.0, 0.0],
                              [0.0, 80.0, 0.0],
                              [0.0, 0.0, 60.0]])
    general_body.setR_GcG_G([[0.1],
                             [-0.1],
                             [0.1]])

    transHat_G = np.array([[1.0],
                           [0.0],
                           [0.0]])
    r_G0B_B = [[-0.1],
               [0.1],
               [0.1]]
    dcm_G0B = np.array([[1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, -1.0]])
    rhoInit = 1.0
    rhoDotInit = 0.05
    spring_constant_k = 100
    damper_constant_c = 0.0

    one_dof_translation = generalSingleBodyStateEffector.DOF()
    one_dof_translation.setDOFAxis(transHat_G)
    one_dof_translation.setR_G0P_P(r_G0B_B)
    one_dof_translation.setDCM_G0P(dcm_G0B)
    one_dof_translation.setBetaInit(rhoInit)
    one_dof_translation.setBetaDotInit(rhoDotInit)
    one_dof_translation.setSpringConstantK(spring_constant_k)
    one_dof_translation.setDampingConstantK(damper_constant_c)
    general_body.addTranslationalDOF(one_dof_translation)
    sc_object.addStateEffector(general_body)
    test_sim.AddModelToTask(task_name, general_body)

    # Set up data logging
    energy_momentum_data_log = sc_object.logger(["totRotEnergy", "totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N"])
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    general_body_trans_states_data_log = []
    for outMsg in general_body.translatingBodyOutMsgs:
        general_body_trans_states_data_log.append(outMsg.recorder())
        test_sim.AddModelToTask(task_name, general_body_trans_states_data_log[-1])
    test_sim.AddModelToTask(task_name, energy_momentum_data_log)
    test_sim.AddModelToTask(task_name, sc_state_data_log)

    # Rum the simulation
    sim_time = 10.0
    test_sim.InitializeSimulation()
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    test_sim.ExecuteSimulation()

    # Extract logged data
    timespan = sc_state_data_log.times() * macros.NANO2SEC
    orb_energy = energy_momentum_data_log.totOrbEnergy
    orb_ang_momentum_N = energy_momentum_data_log.totOrbAngMomPntN_N
    rot_ang_momentum_N = energy_momentum_data_log.totRotAngMomPntC_N
    rot_energy = energy_momentum_data_log.totRotEnergy
    rho = []
    rho_dot = []
    for data in general_body_trans_states_data_log:
        rho.append(data.rho)
        rho_dot.append(data.rhoDot)

    # Plot results
    plot_conservation(timespan,
                      orb_ang_momentum_N,
                      orb_energy,
                      rot_ang_momentum_N,
                      rot_energy)

    # Plot general body displacement
    plt.figure(5)
    plt.clf()
    for idx, disp in enumerate(rho):
        plt.plot(timespan, disp, label=r'$\rho' + str(idx + 1) + '$')
    plt.title(r'General Body Displacement', fontsize=14)
    plt.ylabel('Displacement (m)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    # Plot general body rhoDof
    plt.figure(6)
    plt.clf()
    for idx, rate in enumerate(rho_dot):
        plt.plot(timespan, rate, label=r'$\dot{\rho}_' + str(idx + 1) + '$')
    plt.title(r'General Body Displacement Rate', fontsize=14)
    plt.ylabel('Velocity (m/s)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

    # Unit test check
    unit_test_verification_check(orb_ang_momentum_N, orb_energy, rot_ang_momentum_N, rot_energy)

def create_spacecraft_hub():

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "scObject"
    sc_object.hub.mHub = 750.0  # kg
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]  # [m]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg m^2]
    sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]  # [m]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]  # [m/s]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]  # [rad/s]

    return sc_object

def plot_conservation(timespan, orb_ang_momentum_N, orb_energy, rot_ang_momentum_N, rot_energy):
    # Plot orbital angular momentum relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (orb_ang_momentum_N[:, 0] - orb_ang_momentum_N[0, 0]) / orb_ang_momentum_N[0, 0], color="teal", label=r'$\hat{n}_1$')
    plt.plot(timespan, (orb_ang_momentum_N[:, 1] - orb_ang_momentum_N[0, 1]) / orb_ang_momentum_N[0, 1], color="darkviolet", label=r'$\hat{n}_2$')
    plt.plot(timespan, (orb_ang_momentum_N[:, 2] - orb_ang_momentum_N[0, 2]) / orb_ang_momentum_N[0, 2], color="blue", label=r'$\hat{n}_3$')
    plt.title('Orbital Angular Momentum Relative Difference', fontsize=16)
    plt.ylabel('Relative Difference (Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='lower right', prop={'size': 16})
    plt.grid(True)

    # Plot orbital energy relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (orb_energy - orb_energy[0]) / orb_energy[0], color="teal")
    plt.title('Orbital Energy Relative Difference', fontsize=16)
    plt.ylabel('Relative Difference (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    # Plot sc angular momentum relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (rot_ang_momentum_N[:, 0] - rot_ang_momentum_N[0, 0]) / rot_ang_momentum_N[0, 0], color="teal", label=r'$\hat{n}_1$')
    plt.plot(timespan, (rot_ang_momentum_N[:, 1] - rot_ang_momentum_N[0, 1]) / rot_ang_momentum_N[0, 1], color="darkviolet", label=r'$\hat{n}_2$')
    plt.plot(timespan, (rot_ang_momentum_N[:, 2] - rot_ang_momentum_N[0, 2]) / rot_ang_momentum_N[0, 2], color="blue", label=r'$\hat{n}_3$')
    plt.title('Rotational Angular Momentum Relative Difference', fontsize=16)
    plt.ylabel('Relative Difference (Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='upper right', prop={'size': 16})
    plt.grid(True)

    # Plot sc energy relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (rot_energy - rot_energy[0]) / rot_energy[0], color="teal")
    plt.title('Rotational Energy Difference', fontsize=16)
    plt.ylabel('Difference (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

def unit_test_verification_check(orb_ang_momentum_N, orb_energy, rot_ang_momentum_N, rot_energy):
    accuracy = 1e-13
    np.testing.assert_allclose(orb_ang_momentum_N[0], orb_ang_momentum_N[-1], rtol=accuracy, verbose=True)
    np.testing.assert_allclose(orb_energy[0], orb_energy[-1], rtol=accuracy, verbose=True)
    np.testing.assert_allclose(rot_ang_momentum_N[0], rot_ang_momentum_N[-1], rtol=accuracy, verbose=True)
    np.testing.assert_allclose(rot_energy[0], rot_energy[-1], rtol=accuracy, verbose=True)


if __name__ == "__main__":
    # test_general_one_dof_rotation(True)
    test_general_one_dof_translation(True)

    # test_general_two_dof_rotation(True)
    # test_general_two_dof_translation(True)
