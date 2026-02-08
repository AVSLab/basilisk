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
#

r"""
Overview
--------

This scenario further demonstrates the capabilities of :ref:`constraintDynamicEffector` in simulating a dynamic coupling
between two spacecraft. The constraint effector allows the simulation of two spacecraft rigid hubs attached
through an arm of variable rigidity and is agnostic to the parameters of either vehicle. The scenario compares the
motion of two spacecraft with solar arrays or fuel slosh against a truth model to understand whether the constraint
effector accurately captures the coupled dynamics between the two spacecraft.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioConstrainedDynamicsComponentAnalysis.py

The scenario outputs three plots. The first one describes the translational constraint violations, and the second the
rotational constraint violations. These constraint violations ensure the constraints are being enforced to a sufficiently
high fidelity for the chosen gains (1E4). The third plot shows the angle (panels) or displacement (slosh) error between
the two-spacecraft model with the constraint effector and the one-spacecraft truth model.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, component_list = ['panels'], sc_model = 'MEV2'

Here, each spacecraft have a solar array attached to them. At the beginning of the simulation, the servicer's panel has
an initial angular velocity to induce an excitation that will affect the entire system.

The time history for both the direction and attitude constraint violations are shown below, which demonstrate that the
constraint effector is accurately enforcing the rigid direction constraint.

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsComponentAnalysisDirectionConstraint_panels.svg
   :align: center

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsComponentAnalysisAttitudeConstraint_panels.svg
   :align: center

The figure below shows the error between the angle of each panel. The error oscillates at the panels natural
frequency, and dampens out with time as expected. Note that the magnitude of the error is much smaller than the initial
excitation introduced in the servicer's panel.

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsComponentAnalysisPanelAngleErrors.svg
   :align: center

::

    show_plots = True, component_list = ['slosh'], sc_model = 'MEV2'

Here, the servicer has a fuel tank with fuel slosh particles along each of the body axis. At the beginning of the
simulation, the servicer's slosh has an initial linear velocity to induce an excitation that will affect the entire
system.

The time history for both the direction and attitude constraint violations are shown below, which demonstrate that the
constraint effector is accurately enforcing the rigid direction constraint.

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsComponentAnalysisDirectionConstraint_slosh.svg
   :align: center

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsComponentAnalysisAttitudeConstraint_slosh.svg
   :align: center

The figure below shows the error between the slosh mass displacement of both models. The error oscillates at
the slosh particle's natural frequency, and dampens out with time as expected. Note that the magnitude of the error is
much smaller than the initial excitation introduced in the servicer's slosh particles.

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsComponentAnalysisSloshMassDisplacementErrors.svg
   :align: center

"""

#
#   Basilisk Scenario Script
#
#   Purpose:            Illustrates coupled spacecraft with additional components attached.
#   Author:             João Vaz Carneiro
#   Creation Date:      Oct 2, 2025
#

# Basilisk imports
from Basilisk.utilities import (SimulationBaseClass, macros, RigidBodyKinematics, vizSupport,
                                pythonVariableLogger, unitTestSupport)
from Basilisk.simulation import (spacecraft, constraintDynamicEffector, svIntegrators,
                                 linearSpringMassDamper, hingedRigidBodyStateEffector, fuelTank)
# plotting imports
import numpy as np
import matplotlib.pyplot as plt

from Basilisk import __path__
import os
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


class SimBaseClass(SimulationBaseClass.SimBaseClass):
    def __init__(self, dynRate=0.001):
        self.dynRateSec = dynRate
        self.dynRateNanos = macros.sec2nano(dynRate)
        SimulationBaseClass.SimBaseClass.__init__(self)

        # Create simulation variable names
        self.simTaskName = "simTask"
        simProcessName = "simProcess"

        # Create the dynamics task and set the integration update time
        self.dynProcess = self.CreateNewProcess(simProcessName)
        self.dynProcess.addTask(self.CreateNewTask(self.simTaskName, self.dynRateNanos))


class solar_panel_geometry:
    mass = 100.0
    IPntS_S = np.array([[150.0, 0.0, 0.0], [0.0, 300.0, 0.0], [0.0, 0.0, 150.0]])
    d = 1.0  # note this is the distance from hinge point to hinged rigid body COM, not the damping coefficient
    r_H1B_B = np.array([0.0, 1.0, 1.0])
    r_H2B_B = np.array([0.0, -1.0, -1.0])
    dcm_H1B = np.array([[0.0, -1.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0]])
    dcm_H2B = np.array([[0.0, 1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0]])

    xi = 0.2
    freq = 2 * 2 * np.pi
    k = IPntS_S[1, 1] * freq ** 2
    c = IPntS_S[1, 1] * 2 * xi * freq
    thetaInit = 0.0 * macros.D2R
    thetaDotInit = 0.0 * macros.D2R


class slosh_fuel_tank_geometry:
    massInit = 50.0
    r_P1B_B = np.array([0.01, 0.0, 0.01])
    p1Hat_B = np.array([1, 0, 0])
    r_P2B_B = np.array([-0.01, 0.01, 0.0])
    p2Hat_B = np.array([0, 1, 0])
    r_P3B_B = np.array([0.01, 0.0, 0.01])
    p3Hat_B = np.array([0, 0, 1])

    propMassInit = 350
    r_TcT_TInit = np.zeros(3)
    r_TB_B = np.array([0, 0, 0.1])
    radiusTankInit = 0.5

    xi = 0.2
    freq = 2 * 2 * np.pi
    stiffness_parameter = massInit * freq ** 2  # kg/s^2 (N/m)
    damping_parameter = massInit * 2 * xi * freq  # kg/s
    rhoInit = 0.0
    rhoDotInit = 0.0


def plot_direction_violations(timeData, psi_B1):
    plt.figure(1)
    for i in range(3):
        plt.semilogy(timeData, np.abs(psi_B1[:, i]), alpha=0.8,
                     color=unitTestSupport.getLineColor(i, 4))
    plt.semilogy(timeData, np.linalg.norm(psi_B1, axis=1),
                 color=unitTestSupport.getLineColor(3, 4))
    plt.legend([r'$\psi_1$', r'$\psi_2$', r'$\psi_3$', r'$|\mathbf{\psi}|$'], loc='best')
    plt.xlabel('Time [sec]')
    plt.ylabel('Direction Constraint \nViolation ' r'$\psi$ [m]')
    plt.axis("tight")


def plot_attitude_violations(timeData, sigma_B2B1):
    plt.figure(2)
    for i in range(3):
        plt.semilogy(timeData,
                     np.abs(4 * np.arctan(sigma_B2B1[:, i]) * macros.R2D), alpha=0.8,
                     color=unitTestSupport.getLineColor(i, 4))
    plt.semilogy(timeData,
                 np.linalg.norm(4 * np.arctan(sigma_B2B1) * macros.R2D, axis=1),
                 color=unitTestSupport.getLineColor(3, 4))
    plt.legend([r'$\phi_1$', r'$\phi_2$', r'$\phi_3$', r'$|\mathbf{\phi}|$'], loc='best')
    plt.xlabel('Time [sec]')
    plt.ylabel('Attitude Constraint \nViolation ' r'$\phi$ [deg]')
    plt.axis("tight")


def plot_panel_angle_error(timeData, sp1ThetaLog, sp2ThetaLog, sp3ThetaLog, sp4ThetaLog):
    plt.figure(3)
    plt.plot(timeData, (sp1ThetaLog - sp3ThetaLog) * macros.R2D,
             color=unitTestSupport.getLineColor(0, 2), label=r'$\Delta\theta_1$')
    plt.plot(timeData, (sp2ThetaLog - sp4ThetaLog) * macros.R2D,
             color=unitTestSupport.getLineColor(1, 2), label=r'$\Delta\theta_2$')
    plt.legend(loc='best')
    plt.xlabel('Time [sec]')
    plt.ylabel('Panel Angle Error [deg]')


def plot_slosh_displacement_error(timeData, rhoLog, rhoTLog):
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, rhoLog[idx] - rhoTLog[idx],
                 color=unitTestSupport.getLineColor(idx, 3), label=r'$\Delta\rho_' + str(idx + 1) + '$')
    plt.legend(loc='best')
    plt.xlabel('Time [sec]')
    plt.ylabel('Mass Displacement Error [m]')


def run(show_plots, component_list, sc_model):
    """
    Args:
        show_plots (bool): Determines if the script should display plots
        component_list (str): Choose which components to add between ``panels``, ``slosh`` or ``None``
        sc_model (str): Choose which spacecraft models to use between ``bskSat``, ``MEV1``, or ``MEV2``
    """
    scSim = SimBaseClass()

    set_up_spacecraft(scSim, sc_model)
    set_up_initial_conditions(scSim, component_list)
    set_up_constraint_effector(scSim)
    set_up_additional_effectors(scSim, component_list)

    log_data(scSim, component_list)
    set_up_vizard(scSim)

    run_simulation(scSim, component_list)

    process_data(scSim)
    return plotting(scSim, component_list, show_plots)


def set_up_spacecraft(scSim, sc_model):
    # Create spacecraft
    scSim.scObject1 = spacecraft.Spacecraft()  # chaser spacecraft
    scSim.scObject1.ModelTag = "chaser"
    scSim.scObject2 = spacecraft.Spacecraft()  # target spacecraft
    scSim.scObject2.ModelTag = "target"

    # Add test modules to runtime call list
    scSim.AddModelToTask(scSim.simTaskName, scSim.scObject1)
    scSim.AddModelToTask(scSim.simTaskName, scSim.scObject2)

    # Set the servicer spacecraft integrator to RKF45
    integratorObject = svIntegrators.svIntegratorRKF45(scSim.scObject1)
    scSim.scObject1.setIntegrator(integratorObject)
    integratorObject.this.disown()

    # Sync dynamics integration of target spacecraft to dynamics integration of servicer spacecraft
    scSim.scObject1.syncDynamicsIntegration(scSim.scObject2)

    # Define mass properties of the rigid hub of both spacecraft
    if sc_model == 'bskSat':  # uses two identical "bskSats" which can be found in other Basilisk example scripts
        scSim.scObject1.hub.mHub = 750.0  # [kg] chaser spacecraft mass
        scSim.scObject1.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B1 relative to chaser COM
        scSim.scObject1.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.scObject2.hub.mHub = 750.0  # [kg] target spacecraft mass
        scSim.scObject2.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B2 relative to target COM
        scSim.scObject2.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.r_P1B1_mag = 0.1  # [m] position vector magnitude from chaser COM to connection point
        scSim.r_P2B2_mag = 0.1  # [m] position vector magnitude from target COM to connection point
        scSim.r_P2P1_mag = 0.1  # [m] arm length |r_P2P1|

    elif sc_model == 'MEV1':  # chaser is MEV-1 and target is Intelsat-901
        scSim.scObject1.hub.mHub = 2330.0  # [kg] chaser spacecraft mass
        scSim.scObject1.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B1 relative to chaser COM
        scSim.scObject1.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.scObject2.hub.mHub = 4723.0  # [kg] target spacecraft mass
        scSim.scObject2.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B2 relative to target COM
        scSim.scObject2.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.r_P1B1_mag = 1.0  # [m] position vector magnitude from chaser COM to connection point
        scSim.r_P2B2_mag = 0.5  # [m] position vector magnitude from target COM to connection point
        scSim.r_P2P1_mag = 0.5  # [m] arm length |r_P2P1|

    elif sc_model == 'MEV2':  # chaser is MEV-2 and target is Intelsat 10-02
        scSim.scObject1.hub.mHub = 2875.0  # [kg] chaser spacecraft mass
        scSim.scObject1.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B1 relative to chaser COM
        scSim.scObject1.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.scObject2.hub.mHub = 3298.0  # [kg] target spacecraft mass
        scSim.scObject2.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B2 relative to target COM
        scSim.scObject2.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.r_P1B1_mag = 1.0  # [m] position vector magnitude from chaser COM to connection point
        scSim.r_P2B2_mag = 0.5  # [m] position vector magnitude from target COM to connection point
        scSim.r_P2P1_mag = 0.5  # [m] arm length |r_P2P1|

    else:
        raise Exception("sc_model must be 'bskSat', 'MEV1', or 'MEV2'")

    # Create single spacecraft truth model for comparison
    scSim.scObjectT = spacecraft.Spacecraft()
    scSim.scObjectT.ModelTag = "spacecraftBodyT"

    # Set truth spacecraft integrator to be the same as the constraint effector model
    integratorObjectT = svIntegrators.svIntegratorRKF45(scSim.scObjectT)
    scSim.scObjectT.setIntegrator(integratorObjectT)
    integratorObjectT.this.disown()

    scSim.AddModelToTask(scSim.simTaskName, scSim.scObjectT)


def set_up_initial_conditions(scSim, component_list):
    # Note: here we assume that the N, B1 and B2 frames are identical, therefore no frame is defined
    r_CN = np.array([1, 0, 0])  # [m]
    rDot_CN = np.array([0., 0., 0.])  # [m/s]
    rHat = np.array([1, 1, 1]) / np.linalg.norm(np.array([1, 1, 1]))
    scSim.r_P1B1_B1 = scSim.r_P1B1_mag * rHat
    scSim.r_P2B2_B2 = - scSim.r_P2B2_mag * rHat
    scSim.r_P2P1_B1Init = scSim.r_P2P1_mag * rHat
    r_B2B1 = scSim.r_P1B1_B1 + scSim.r_P2P1_B1Init - scSim.r_P2B2_B2

    # Center of mass calculation
    r_C1B1 = np.zeros(3)
    r_C2B2 = np.zeros(3)
    total_mass_1 = scSim.scObject1.hub.mHub
    total_mass_2 = scSim.scObject2.hub.mHub

    # Need to adjust center of mass if components are present
    if "panels" in component_list:
        r_C1B1 = (total_mass_1 * r_C1B1 + solar_panel_geometry.mass * (solar_panel_geometry.r_H1B_B - solar_panel_geometry.dcm_H1B[0, :] * solar_panel_geometry.d)) / (total_mass_1 + solar_panel_geometry.mass)
        r_C2B2 = (total_mass_1 * r_C2B2 + solar_panel_geometry.mass * (solar_panel_geometry.r_H2B_B - solar_panel_geometry.dcm_H2B[0, :] * solar_panel_geometry.d)) / (total_mass_2 + solar_panel_geometry.mass)

        total_mass_1 += solar_panel_geometry.mass
        total_mass_2 += solar_panel_geometry.mass
    if "slosh" in component_list:
        r_C1B1 = ((total_mass_1 * r_C1B1 +
                  slosh_fuel_tank_geometry.massInit * (slosh_fuel_tank_geometry.r_P1B_B + slosh_fuel_tank_geometry.r_P2B_B + slosh_fuel_tank_geometry.r_P3B_B)
                  + slosh_fuel_tank_geometry.propMassInit * (slosh_fuel_tank_geometry.r_TcT_TInit + slosh_fuel_tank_geometry.r_TB_B)) /
                  (total_mass_1 + 3 * slosh_fuel_tank_geometry.massInit + slosh_fuel_tank_geometry.propMassInit))

        total_mass_1 += 3 * slosh_fuel_tank_geometry.massInit + slosh_fuel_tank_geometry.propMassInit
    scSim.r_B1C = - (total_mass_1 * r_C1B1 + total_mass_2 * (r_C2B2 + r_B2B1)) / (total_mass_1 + total_mass_2)
    scSim.r_B2C = scSim.r_B1C + r_B2B1

    # Compute rotational states
    sigma_B1N = [[0.0], [0.0], [0.0]]
    sigma_B2N = [[0.0], [0.0], [0.0]]
    omega_B1N_B1 = np.array([0.0, 0.0, 0.0])
    omega_B2N_B2 = np.array([0.0, 0.0, 0.0])

    # Set the initial values for all spacecraft states
    scSim.scObject1.hub.r_CN_NInit = r_C1B1 + scSim.r_B1C + r_CN
    scSim.scObject1.hub.v_CN_NInit = rDot_CN
    scSim.scObject1.hub.sigma_BNInit = sigma_B1N
    scSim.scObject1.hub.omega_BN_BInit = omega_B1N_B1
    scSim.scObject2.hub.r_CN_NInit = r_C2B2 + scSim.r_B2C + r_CN
    scSim.scObject2.hub.v_CN_NInit = rDot_CN
    scSim.scObject2.hub.sigma_BNInit = sigma_B2N
    scSim.scObject2.hub.omega_BN_BInit = omega_B2N_B2

    # Set truth mass properties
    scSim.r_BcC = (scSim.scObject1.hub.mHub * scSim.r_B1C + scSim.scObject2.hub.mHub * scSim.r_B2C) / (scSim.scObject1.hub.mHub + scSim.scObject2.hub.mHub)
    rTilde_B1Bc = np.array(RigidBodyKinematics.v3Tilde(scSim.r_B1C - scSim.r_BcC))
    rTilde_B2Bc = np.array(RigidBodyKinematics.v3Tilde(scSim.r_B2C - scSim.r_BcC))
    IPntBc = (scSim.scObject1.hub.IHubPntBc_B - scSim.scObject1.hub.mHub * rTilde_B1Bc @ rTilde_B1Bc + scSim.scObject2.hub.IHubPntBc_B - scSim.scObject2.hub.mHub * rTilde_B2Bc @ rTilde_B2Bc)
    scSim.scObjectT.hub.IHubPntBc_B = IPntBc
    scSim.scObjectT.hub.mHub = scSim.scObject1.hub.mHub + scSim.scObject2.hub.mHub

    # Set initial state values
    scSim.scObjectT.hub.r_CN_NInit = r_CN
    scSim.scObjectT.hub.v_CN_NInit = rDot_CN
    scSim.scObjectT.hub.sigma_BNInit = sigma_B1N
    scSim.scObjectT.hub.omega_BN_BInit = np.array([0.0, 0.0, 0.0])


def set_up_constraint_effector(scSim):
    # Set up the constraint effector
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    constraintEffector.setR_P1B1_B1(scSim.r_P1B1_B1)
    constraintEffector.setR_P2B2_B2(scSim.r_P2B2_B2)
    constraintEffector.setR_P2P1_B1Init(scSim.r_P2P1_B1Init)
    constraintEffector.setAlpha(1E4)
    constraintEffector.setBeta(constraintEffector.getAlpha())
    constraintEffector.ModelTag = "constraintEffector"

    # Add constraints to both spacecraft
    scSim.scObject1.addDynamicEffector(constraintEffector)
    scSim.scObject2.addDynamicEffector(constraintEffector)
    scSim.AddModelToTask(scSim.simTaskName, constraintEffector)


def set_up_additional_effectors(scSim, component_list):
    if "panels" in component_list:
        scSim.solarPanel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
        scSim.solarPanel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

        # define spacecraft 1's solar panel properties
        scSim.solarPanel1.mass = solar_panel_geometry.mass
        scSim.solarPanel1.IPntS_S = solar_panel_geometry.IPntS_S
        scSim.solarPanel1.d = solar_panel_geometry.d
        scSim.solarPanel1.k = solar_panel_geometry.k
        scSim.solarPanel1.c = solar_panel_geometry.c
        scSim.solarPanel1.r_HB_B = solar_panel_geometry.r_H1B_B
        scSim.solarPanel1.dcm_HB = solar_panel_geometry.dcm_H1B
        scSim.solarPanel1.thetaInit = solar_panel_geometry.thetaInit
        scSim.solarPanel1.thetaDotInit = solar_panel_geometry.thetaDotInit

        # define spacecraft 2's solar panel properties
        scSim.solarPanel2.mass = solar_panel_geometry.mass
        scSim.solarPanel2.IPntS_S = solar_panel_geometry.IPntS_S
        scSim.solarPanel2.d = solar_panel_geometry.d
        scSim.solarPanel2.k = solar_panel_geometry.k
        scSim.solarPanel2.c = solar_panel_geometry.c
        scSim.solarPanel2.r_HB_B = solar_panel_geometry.r_H2B_B
        scSim.solarPanel2.dcm_HB = solar_panel_geometry.dcm_H2B
        scSim.solarPanel2.thetaInit = solar_panel_geometry.thetaInit
        scSim.solarPanel2.thetaDotInit = solar_panel_geometry.thetaDotInit

        # add solar panels to spacecraft
        scSim.scObject1.addStateEffector(scSim.solarPanel1)
        scSim.scObject2.addStateEffector(scSim.solarPanel2)

        # define truth model
        scSim.solarPanel3 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
        scSim.solarPanel4 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

        # define the truth spacecraft's first solar panel properties
        scSim.solarPanel3.mass = solar_panel_geometry.mass
        scSim.solarPanel3.IPntS_S = solar_panel_geometry.IPntS_S
        scSim.solarPanel3.d = solar_panel_geometry.d
        scSim.solarPanel3.k = solar_panel_geometry.k
        scSim.solarPanel3.c = solar_panel_geometry.c
        scSim.solarPanel3.r_HB_B = solar_panel_geometry.r_H1B_B + scSim.r_B1C - scSim.r_BcC
        scSim.solarPanel3.dcm_HB = solar_panel_geometry.dcm_H1B
        scSim.solarPanel3.thetaInit = solar_panel_geometry.thetaInit
        scSim.solarPanel3.thetaDotInit = solar_panel_geometry.thetaDotInit

        # define the truth spacecraft's second solar panel properties
        scSim.solarPanel4.mass = solar_panel_geometry.mass
        scSim.solarPanel4.IPntS_S = solar_panel_geometry.IPntS_S
        scSim.solarPanel4.d = solar_panel_geometry.d
        scSim.solarPanel4.k = solar_panel_geometry.k
        scSim.solarPanel4.c = solar_panel_geometry.c
        scSim.solarPanel4.r_HB_B = solar_panel_geometry.r_H2B_B + scSim.r_B2C - scSim.r_BcC
        scSim.solarPanel4.dcm_HB = solar_panel_geometry.dcm_H2B
        scSim.solarPanel4.thetaInit = solar_panel_geometry.thetaInit
        scSim.solarPanel4.thetaDotInit = solar_panel_geometry.thetaDotInit

        # add solar panels to spacecraft
        scSim.scObjectT.addStateEffector(scSim.solarPanel3)
        scSim.scObjectT.addStateEffector(scSim.solarPanel4)

    if "slosh" in component_list:
        # define particle 1
        scSim.particle1 = linearSpringMassDamper.LinearSpringMassDamper()
        scSim.particle1.k = slosh_fuel_tank_geometry.stiffness_parameter
        scSim.particle1.c = slosh_fuel_tank_geometry.damping_parameter
        scSim.particle1.r_PB_B = slosh_fuel_tank_geometry.r_P1B_B
        scSim.particle1.pHat_B = slosh_fuel_tank_geometry.p1Hat_B
        scSim.particle1.rhoInit = slosh_fuel_tank_geometry.rhoInit
        scSim.particle1.rhoDotInit = slosh_fuel_tank_geometry.rhoDotInit
        scSim.particle1.massInit = slosh_fuel_tank_geometry.massInit

        # define particle 2
        scSim.particle2 = linearSpringMassDamper.LinearSpringMassDamper()
        scSim.particle2.k = slosh_fuel_tank_geometry.stiffness_parameter
        scSim.particle2.c = slosh_fuel_tank_geometry.damping_parameter
        scSim.particle2.r_PB_B = slosh_fuel_tank_geometry.r_P2B_B
        scSim.particle2.pHat_B = slosh_fuel_tank_geometry.p2Hat_B
        scSim.particle2.rhoInit = slosh_fuel_tank_geometry.rhoInit
        scSim.particle2.rhoDotInit = slosh_fuel_tank_geometry.rhoDotInit
        scSim.particle2.massInit = slosh_fuel_tank_geometry.massInit

        # define particle 3
        scSim.particle3 = linearSpringMassDamper.LinearSpringMassDamper()
        scSim.particle3.k = slosh_fuel_tank_geometry.stiffness_parameter
        scSim.particle3.c = slosh_fuel_tank_geometry.damping_parameter
        scSim.particle3.r_PB_B = slosh_fuel_tank_geometry.r_P3B_B
        scSim.particle3.pHat_B = slosh_fuel_tank_geometry.p3Hat_B
        scSim.particle3.rhoInit = slosh_fuel_tank_geometry.rhoInit
        scSim.particle3.rhoDotInit = slosh_fuel_tank_geometry.rhoDotInit
        scSim.particle3.massInit = slosh_fuel_tank_geometry.massInit

        # define the fuel tank
        scSim.tank1 = fuelTank.FuelTank()
        tankModel = fuelTank.FuelTankModelConstantVolume()
        tankModel.propMassInit = slosh_fuel_tank_geometry.propMassInit
        tankModel.r_TcT_TInit = slosh_fuel_tank_geometry.r_TcT_TInit
        tankModel.radiusTankInit = slosh_fuel_tank_geometry.radiusTankInit
        tankModel.this.disown()
        scSim.tank1.setTankModel(tankModel)
        scSim.tank1.r_TB_B = slosh_fuel_tank_geometry.r_TB_B
        scSim.tank1.nameOfMassState = "fuelTankMass1"
        scSim.tank1.pushFuelSloshParticle(scSim.particle1)
        scSim.tank1.pushFuelSloshParticle(scSim.particle2)
        scSim.tank1.pushFuelSloshParticle(scSim.particle3)
        scSim.tank1.updateOnly = True

        # add fuel slosh to the servicer spacecraft
        scSim.scObject1.addStateEffector(scSim.tank1)
        scSim.scObject1.addStateEffector(scSim.particle1)
        scSim.scObject1.addStateEffector(scSim.particle2)
        scSim.scObject1.addStateEffector(scSim.particle3)

        # define particle 4
        scSim.particle4 = linearSpringMassDamper.LinearSpringMassDamper()
        scSim.particle4.k = slosh_fuel_tank_geometry.stiffness_parameter
        scSim.particle4.c = slosh_fuel_tank_geometry.damping_parameter
        scSim.particle4.r_PB_B = slosh_fuel_tank_geometry.r_P1B_B + scSim.r_B1C - scSim.r_BcC
        scSim.particle4.pHat_B = slosh_fuel_tank_geometry.p1Hat_B
        scSim.particle4.rhoInit = slosh_fuel_tank_geometry.rhoInit
        scSim.particle4.rhoDotInit = slosh_fuel_tank_geometry.rhoDotInit
        scSim.particle4.massInit = slosh_fuel_tank_geometry.massInit

        # define particle 5
        scSim.particle5 = linearSpringMassDamper.LinearSpringMassDamper()
        scSim.particle5.k = slosh_fuel_tank_geometry.stiffness_parameter
        scSim.particle5.c = slosh_fuel_tank_geometry.damping_parameter
        scSim.particle5.r_PB_B = slosh_fuel_tank_geometry.r_P2B_B + scSim.r_B1C - scSim.r_BcC
        scSim.particle5.pHat_B = slosh_fuel_tank_geometry.p2Hat_B
        scSim.particle5.rhoInit = slosh_fuel_tank_geometry.rhoInit
        scSim.particle5.rhoDotInit =slosh_fuel_tank_geometry.rhoDotInit
        scSim.particle5.massInit = slosh_fuel_tank_geometry.massInit

        # define particle 6
        scSim.particle6 = linearSpringMassDamper.LinearSpringMassDamper()
        scSim.particle6.k = slosh_fuel_tank_geometry.stiffness_parameter
        scSim.particle6.c = slosh_fuel_tank_geometry.damping_parameter
        scSim.particle6.r_PB_B = slosh_fuel_tank_geometry.r_P3B_B + scSim.r_B1C - scSim.r_BcC
        scSim.particle6.pHat_B = slosh_fuel_tank_geometry.p3Hat_B
        scSim.particle6.rhoInit = slosh_fuel_tank_geometry.rhoInit
        scSim.particle6.rhoDotInit = slosh_fuel_tank_geometry.rhoDotInit
        scSim.particle6.massInit = slosh_fuel_tank_geometry.massInit

        # define the fuel tank
        scSim.tank2 = fuelTank.FuelTank()
        tankModelT = fuelTank.FuelTankModelConstantVolume()
        tankModelT.propMassInit = slosh_fuel_tank_geometry.propMassInit
        tankModelT.r_TcT_TInit = slosh_fuel_tank_geometry.r_TcT_TInit
        tankModelT.radiusTankInit = slosh_fuel_tank_geometry.radiusTankInit
        tankModelT.this.disown()
        scSim.tank2.setTankModel(tankModelT)
        scSim.tank2.r_TB_B = slosh_fuel_tank_geometry.r_TB_B + scSim.r_B1C - scSim.r_BcC
        scSim.tank2.nameOfMassState = "fuelTankMass2"
        scSim.tank2.pushFuelSloshParticle(scSim.particle4)
        scSim.tank2.pushFuelSloshParticle(scSim.particle5)
        scSim.tank2.pushFuelSloshParticle(scSim.particle6)
        scSim.tank2.updateOnly = True

        # add fuel slosh to the servicer spacecraft
        scSim.scObjectT.addStateEffector(scSim.tank2)
        scSim.scObjectT.addStateEffector(scSim.particle4)
        scSim.scObjectT.addStateEffector(scSim.particle5)
        scSim.scObjectT.addStateEffector(scSim.particle6)


def log_data(scSim, component_list):
    # Log the spacecraft state message
    scSim.datLog1 = scSim.scObject1.scStateOutMsg.recorder()
    scSim.datLog2 = scSim.scObject2.scStateOutMsg.recorder()
    scSim.AddModelToTask(scSim.simTaskName, scSim.datLog1)
    scSim.AddModelToTask(scSim.simTaskName, scSim.datLog2)

    if "panels" in component_list:
        scSim.sp1Log = scSim.solarPanel1.hingedRigidBodyOutMsg.recorder()
        scSim.sp2Log = scSim.solarPanel2.hingedRigidBodyOutMsg.recorder()
        scSim.AddModelToTask(scSim.simTaskName, scSim.sp1Log)
        scSim.AddModelToTask(scSim.simTaskName, scSim.sp2Log)

        scSim.sp3Log = scSim.solarPanel3.hingedRigidBodyOutMsg.recorder()
        scSim.sp4Log = scSim.solarPanel4.hingedRigidBodyOutMsg.recorder()
        scSim.AddModelToTask(scSim.simTaskName, scSim.sp3Log)
        scSim.AddModelToTask(scSim.simTaskName, scSim.sp4Log)

    if "slosh" in component_list:
        scSim.rhoLog = pythonVariableLogger.PythonVariableLogger({
            "rho1": lambda _: scSim.scObject1.dynManager.getStateObject('linearSpringMassDamperRho1').getState(),
            "rho2": lambda _: scSim.scObject1.dynManager.getStateObject('linearSpringMassDamperRho2').getState(),
            "rho3": lambda _: scSim.scObject1.dynManager.getStateObject('linearSpringMassDamperRho3').getState(),
        })
        scSim.AddModelToTask(scSim.simTaskName, scSim.rhoLog)

        scSim.rhoTLog = pythonVariableLogger.PythonVariableLogger({
            "rho4": lambda _: scSim.scObjectT.dynManager.getStateObject('linearSpringMassDamperRho4').getState(),
            "rho5": lambda _: scSim.scObjectT.dynManager.getStateObject('linearSpringMassDamperRho5').getState(),
            "rho6": lambda _: scSim.scObjectT.dynManager.getStateObject('linearSpringMassDamperRho6').getState(),
        })
        scSim.AddModelToTask(scSim.simTaskName, scSim.rhoTLog)

    scSim.datLogT = scSim.scObjectT.scStateOutMsg.recorder()
    scSim.AddModelToTask(scSim.simTaskName, scSim.datLogT)


def set_up_vizard(scSim):
    return vizSupport.enableUnityVisualization(scSim, scSim.simTaskName,
                                               [scSim.scObject1, scSim.scObject2, scSim.scObjectT],
                                               # saveFile=__file__
                                               )


def run_simulation(scSim, component_list):
    # Initialize the simulation
    scSim.SetProgressBar(True)
    scSim.InitializeSimulation()

    # Add mid-simulation perturbations if components are present to be perturbed
    if "panels" in component_list:
        scSim.scObject1.dynManager.getStateObject(scSim.solarPanel1.nameOfThetaDotState).setState([[7 * macros.D2R]])
        scSim.scObjectT.dynManager.getStateObject(scSim.solarPanel3.nameOfThetaDotState).setState([[7 * macros.D2R]])

    if "slosh" in component_list:
        scSim.scObject1.dynManager.getStateObject(scSim.particle1.nameOfRhoDotState).setState([[0.02]])
        scSim.scObject1.dynManager.getStateObject(scSim.particle2.nameOfRhoDotState).setState([[-0.025]])
        scSim.scObject1.dynManager.getStateObject(scSim.particle3.nameOfRhoDotState).setState([[-0.015]])

        scSim.scObjectT.dynManager.getStateObject(scSim.particle4.nameOfRhoDotState).setState([[0.02]])
        scSim.scObjectT.dynManager.getStateObject(scSim.particle5.nameOfRhoDotState).setState([[-0.025]])
        scSim.scObjectT.dynManager.getStateObject(scSim.particle6.nameOfRhoDotState).setState([[-0.015]])

    # Run the simulation
    scSim.ConfigureStopTime(macros.sec2nano(10))
    scSim.ExecuteSimulation()


def process_data(scSim):
    # Grab the time vector
    scSim.time_data = scSim.datLog1.times() * macros.NANO2SEC
    num_steps = len(scSim.time_data)

    # Collect the logged spacecraft states
    r_B1N_N_hist = scSim.datLog1.r_BN_N
    scSim.sigma_B1N_hist = scSim.datLog1.sigma_BN
    r_B2N_N_hist = scSim.datLog2.r_BN_N
    scSim.sigma_B2N_hist = scSim.datLog2.sigma_BN

    # Compute constraint violations
    r_B1N_B1 = np.empty(r_B1N_N_hist.shape)
    r_B2N_B1 = np.empty(r_B2N_N_hist.shape)
    r_P2B2_B1 = np.empty(r_B1N_N_hist.shape)
    scSim.sigma_B2B1 = np.empty(scSim.sigma_B1N_hist.shape)
    for i in range(num_steps):
        dcm_B1N = RigidBodyKinematics.MRP2C(scSim.sigma_B1N_hist[i, :])
        r_B1N_B1[i, :] = dcm_B1N @ r_B1N_N_hist[i, :]
        r_B2N_B1[i, :] = dcm_B1N @ r_B2N_N_hist[i, :]
        dcm_NB2 = RigidBodyKinematics.MRP2C(scSim.sigma_B2N_hist[i, :]).transpose()
        r_P2B2_B1[i, :] = dcm_B1N @ dcm_NB2 @ scSim.r_P2B2_B2
        scSim.sigma_B2B1[i, :] = RigidBodyKinematics.subMRP(scSim.sigma_B2N_hist[i, :], scSim.sigma_B1N_hist[i, :])
    scSim.psi_B1 = r_B1N_B1 + scSim.r_P1B1_B1 + scSim.r_P2P1_B1Init - (r_B2N_B1 + r_P2B2_B1)


def plotting(scSim, component_list, show_plots):
    larger_size = 20
    smaller_size = 18
    fontdict = {'family': 'serif',
                'weight': 'normal',
                'size': larger_size}
    plt.rc('font', **fontdict)
    plt.rc('axes', labelsize=larger_size)
    plt.rc('xtick', labelsize=smaller_size)
    plt.rc('ytick', labelsize=smaller_size)
    plt.rc('legend', fontsize=smaller_size)
    plt.rcParams["figure.figsize"] = (7, 4)
    plt.rcParams['axes.autolimit_mode'] = 'data'  # no “round number” padding
    plt.rcParams['axes.xmargin'] = 0
    plt.close("all")

    plot_direction_violations(scSim.time_data, scSim.psi_B1)
    figureList = {}
    pltName = fileName + "DirectionConstraint_" + component_list[0]
    figureList[pltName] = plt.figure(1)

    plot_attitude_violations(scSim.time_data, scSim.sigma_B2B1)
    pltName = fileName + "AttitudeConstraint_" + component_list[0]
    figureList[pltName] = plt.figure(2)

    if "panels" in component_list:
        spTimes = scSim.sp1Log.times() * macros.NANO2SEC
        sp1ThetaLog = scSim.sp1Log.theta
        sp2ThetaLog = scSim.sp2Log.theta
        sp3ThetaLog = scSim.sp3Log.theta
        sp4ThetaLog = scSim.sp4Log.theta
        plot_panel_angle_error(spTimes, sp1ThetaLog, sp2ThetaLog, sp3ThetaLog, sp4ThetaLog)
        pltName = fileName + "PanelAngleErrors"
        figureList[pltName] = plt.figure(3)

    if "slosh" in component_list:
        rhoTimes = scSim.datLog1.times() * macros.NANO2SEC
        rho1Log = scSim.rhoLog.rho1
        rho2Log = scSim.rhoLog.rho2
        rho3Log = scSim.rhoLog.rho3
        rho4Log = scSim.rhoTLog.rho4
        rho5Log = scSim.rhoTLog.rho5
        rho6Log = scSim.rhoTLog.rho6
        rhoOut = [rho1Log, rho2Log, rho3Log]
        rhoOutT = [rho4Log, rho5Log, rho6Log]
        plot_slosh_displacement_error(rhoTimes, rhoOut, rhoOutT)
        pltName = fileName + "SloshMassDisplacementErrors"
        figureList[pltName] = plt.figure(3)

    if show_plots:
        plt.show()

    plt.close("all")

    return figureList


if __name__ == "__main__":
    run(
        True,  # show_plots
        ["panels"],  # list of components ["panels", "slosh", None]
        sc_model='MEV2',  # spacecraft model ['bskSat', 'testing', 'MEV1', 'MEV2']
    )
