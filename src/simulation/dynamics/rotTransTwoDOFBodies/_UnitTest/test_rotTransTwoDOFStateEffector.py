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
#   Module Name:        rotTransTwoDOFStateEffector
#   Author:             Leah Kiner
#   Creation Date:      September 4, 2026
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

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros, vizSupport
from Basilisk.simulation import spacecraft, rotTransTwoDOFStateEffector, gravityEffector
from Basilisk.architecture import messaging

test_time_step_sec = 0.0001
sim_time_sec = 30.0
def test_rot_trans_two_dof_state_effector(show_plots):
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
    earth = gravityEffector.GravBodyData()
    earth.planetName = "earth_planet_data"
    earth.mu = 0.3986004415E+15  # meters!
    earth.isCentralBody = True
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([earth])

    # Create the rotating body
    mass_rotating = 100.0  # [kg]
    IPntSc_S = [[50.0, 0.0, 0.0],
                [0.0, 80.0, 0.0],
                [0.0, 0.0, 60.0]]  # [kg m^2]
    r_ScS_S = [0.1, 0.0, -0.1]  # [m]
    r_SB_B = [-1.0, 0.0, 0.0]  # [m]
    dcm_S0B = np.array([[-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, 1.0]])
    sHat_S = [1.0, 0.0, 0.0]
    theta_init = 10.0 * macros.D2R  # [rad]
    k_1 = 100.0

    # Create the translating body
    mass_translating = 50.0  # [kg]
    IPntTc_T = [[25.0, 0.0, 0.0],
                [0.0, 40.0, 0.0],
                [0.0, 0.0, 30.0]]  # [kg m^2]
    r_TcT_T = [-0.1, -0.1, 0.0]  # [m]
    dcm_TS = np.array([[0.0, 0.0, -1.0],
                    [0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0]])
    r_T0S_S = [0.0, 1.0, 0.0]  # [m]
    tHat_T = [0.0, 1.0, 0.0]
    rho_init = 0.5  # [m]
    k_2 = 50.0

    rot_trans_effector = rotTransTwoDOFStateEffector.RotTransTwoDOFStateEffector()
    # Set rotating body properties
    rot_trans_effector.setMass1(mass_rotating)
    rot_trans_effector.setIPntSc_S(IPntSc_S)
    rot_trans_effector.setR_ScS_S(r_ScS_S)
    rot_trans_effector.setR_SB_B(r_SB_B)
    rot_trans_effector.setDCM_S0B(dcm_S0B)
    rot_trans_effector.setSHat_S(sHat_S)
    rot_trans_effector.setThetaInit(theta_init)
    rot_trans_effector.setK1(k_1)
    # Set translating body properties
    rot_trans_effector.setMass2(mass_translating)
    rot_trans_effector.setITPntTc_T(IPntTc_T)
    rot_trans_effector.setR_TcT_T(r_TcT_T)
    rot_trans_effector.setR_T0S_S(r_T0S_S)
    rot_trans_effector.setDCM_TS(dcm_TS)
    rot_trans_effector.setTHat_T(tHat_T)
    rot_trans_effector.setRhoInit(rho_init)
    rot_trans_effector.setK2(k_2)

    sc_object.addStateEffector(rot_trans_effector)
    test_sim.AddModelToTask(task_name, rot_trans_effector)

    # Set up data logging
    energy_momentum_data_log = sc_object.logger(["totRotEnergy", "totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N"])
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    rot_states_data_log = rot_trans_effector.spinningBodyOutMsg.recorder()
    trans_states_data_log = rot_trans_effector.translatingBodyOutMsg.recorder()
    test_sim.AddModelToTask(task_name, energy_momentum_data_log)
    test_sim.AddModelToTask(task_name, sc_state_data_log)
    test_sim.AddModelToTask(task_name, rot_states_data_log)
    test_sim.AddModelToTask(task_name, trans_states_data_log)

    # Add Vizard
    sc_body_list = [sc_object]
    sc_body_list.append(["rotatingBody", rot_trans_effector.bodyConfigLogOutMsgs[0]])
    sc_body_list.append(["translatingBody", rot_trans_effector.bodyConfigLogOutMsgs[1]])
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(test_sim, task_name, sc_body_list,
                                                  saveFile=filename
                                                  )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[sc_object.ModelTag]
                                     , modelPath="CUBE"
                                     , scale=[1.0, 1.0, 1.0]
                                     , color=vizSupport.toRGBA255("gray"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["rotatingBody"]
                                     , modelPath="CUBE"
                                     , scale=[1.0, 1.0, 1.0]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["translatingBody"]
                                     , modelPath="CUBE"
                                     , scale=[1.0, 1.0, 1.0]
                                     , color=vizSupport.toRGBA255("purple"))
        viz.settings.orbitLinesOn = -1

    # Rum the simulation
    test_sim.InitializeSimulation()
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time_sec))
    test_sim.ExecuteSimulation()

    # Extract logged data
    timespan = sc_state_data_log.times() * macros.NANO2SEC
    orb_energy = energy_momentum_data_log.totOrbEnergy
    orb_ang_momentum_N = energy_momentum_data_log.totOrbAngMomPntN_N
    rot_ang_momentum_N = energy_momentum_data_log.totRotAngMomPntC_N
    rot_energy = energy_momentum_data_log.totRotEnergy
    theta = rot_states_data_log.theta * macros.R2D
    theta_dot = rot_states_data_log.thetaDot * macros.R2D
    rho = trans_states_data_log.rho
    rho_dot = trans_states_data_log.rhoDot

    # Plot results
    plot_rotational_states(timespan, theta, theta_dot)
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
    sc_object.hub.mHub = 750.0  # kg
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]  # [m]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg m^2]
    sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]  # [m]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]  # [m/s]
    sc_object.hub.sigma_BNInit = [[0.1], [-0.1], [0.1]]
    sc_object.hub.omega_BN_BInit = [[0.01], [-0.01], [0.01]]  # [rad/s]

    return sc_object

def plot_rotational_states(timespan, theta, theta_dot):
    plt.figure(5)
    plt.clf()
    plt.plot(timespan, theta)
    plt.title(r'Rotating Body Angle', fontsize=14)
    plt.ylabel('Angle (deg)', fontsize=14)
    plt.xlabel('Time (sec)', fontsize=14)
    plt.grid(True)

    plt.figure(6)
    plt.clf()
    plt.plot(timespan, theta_dot)
    plt.title(r'Rotating Body Angle Rate', fontsize=14)
    plt.ylabel('Angle Rate (deg/s)', fontsize=14)
    plt.xlabel('Time (sec)', fontsize=14)
    plt.grid(True)

def plot_translational_states(timespan, rho, rho_dot):
    plt.figure(7)
    plt.clf()
    plt.plot(timespan, rho)
    plt.title(r'Translating Body Displacement', fontsize=14)
    plt.ylabel('Displacement (m)', fontsize=14)
    plt.xlabel('Time (sec)', fontsize=14)
    plt.grid(True)

    plt.figure(8)
    plt.clf()
    plt.plot(timespan, rho_dot)
    plt.title(r'Translating Body Displacement Rate', fontsize=14)
    plt.ylabel('Velocity (m/s)', fontsize=14)
    plt.xlabel('Time (sec)', fontsize=14)
    plt.grid(True)

def plot_conservation(timespan, orb_ang_momentum_N, orb_energy, rot_ang_momentum_N, rot_energy):
    # Plot orbital angular momentum relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (orb_ang_momentum_N[:, 0] - orb_ang_momentum_N[0, 0]) / orb_ang_momentum_N[0, 0], color="teal", label=r'$\hat{n}_1$')
    plt.plot(timespan, (orb_ang_momentum_N[:, 1] - orb_ang_momentum_N[0, 1]) / orb_ang_momentum_N[0, 1], color="darkviolet", label=r'$\hat{n}_2$')
    plt.plot(timespan, (orb_ang_momentum_N[:, 2] - orb_ang_momentum_N[0, 2]) / orb_ang_momentum_N[0, 2], color="blue", label=r'$\hat{n}_3$')
    plt.title('Orbital Angular Momentum', fontsize=16)
    plt.ylabel('Relative Difference (Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='lower right', prop={'size': 16})
    plt.grid(True)

    # Plot orbital energy relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (orb_energy - orb_energy[0]) / orb_energy[0], color="teal")
    plt.title('Orbital Energy', fontsize=16)
    plt.ylabel('Relative Difference (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

    # Plot sc angular momentum relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (rot_ang_momentum_N[:, 0] - rot_ang_momentum_N[0, 0]) / rot_ang_momentum_N[0, 0], color="teal", label=r'$\hat{n}_1$')
    plt.plot(timespan, (rot_ang_momentum_N[:, 1] - rot_ang_momentum_N[0, 1]) / rot_ang_momentum_N[0, 1], color="darkviolet", label=r'$\hat{n}_2$')
    plt.plot(timespan, (rot_ang_momentum_N[:, 2] - rot_ang_momentum_N[0, 2]) / rot_ang_momentum_N[0, 2], color="blue", label=r'$\hat{n}_3$')
    plt.title('Rotational Angular Momentum', fontsize=16)
    plt.ylabel('Relative Difference (Nms)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='upper right', prop={'size': 16})
    plt.grid(True)

    # Plot sc energy relative difference
    plt.figure()
    plt.clf()
    plt.plot(timespan, (rot_energy - rot_energy[0]) / rot_energy[0], color="teal")
    plt.title('Rotational Energy', fontsize=16)
    plt.ylabel('Relative Difference (J)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.grid(True)

def unit_test_verification_check(orb_ang_momentum_N,
                                 orb_energy,
                                 rot_ang_momentum_N,
                                 rot_energy):
    accuracy = 1e-13
    np.testing.assert_allclose(orb_ang_momentum_N[0], orb_ang_momentum_N[-1], rtol=accuracy, verbose=True)
    np.testing.assert_allclose(orb_energy[0], orb_energy[-1], rtol=accuracy, verbose=True)
    np.testing.assert_allclose(rot_ang_momentum_N[0], rot_ang_momentum_N[-1], rtol=accuracy, verbose=True)
    np.testing.assert_allclose(rot_energy[0], rot_energy[-1], rtol=1e-13, verbose=True)

if __name__ == "__main__":
    test_rot_trans_two_dof_state_effector(True)
