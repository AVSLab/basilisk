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

r"""
Overview
--------

This scenario demonstrates the capabilities of :ref:`constraintDynamicEffector` in simulating a dynamic coupling
between two spacecraft. The constraint effector allows the simulation of two spacecraft rigid hubs attached
through an arm of variable rigidity and is agnostic to the parameters of either vehicle. The scenario can be run
with gravity to place the set of spacecraft in low Earth orbit or without gravity to simulate a deep space scenario.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioConstrainedDynamics.py

The scenario outputs two plots: one for the translational constraint violations, and another for the rotational constraint
violations. These constraint violations can be used to evaluate whether the simulation is replicating coupled motion with
the desired fidelity and tune gains if the bounds should be changed. Increasing alpha and beta will reduce the magnitude
of the constraint violations while increasing run time.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, env = 'Gravity'

Here, two identical spacecraft are connected along their body frame x-axes with a slight tumble rate while in low Earth orbit.
The time history for both the holonomic direction and attitude constraint violations are shown below. Note the y-axis log scale
showing the order of magnitude of the violations. The initial transient rises from perfect initial setup conditions before
settling to an equilibrium. This equilibrium is impacted by how dynamic the scenario is (rotation rate, gravity, thrusting),
the mass and inertia of the respective vehicles, and how stiff the gains are driven by the alpha and beta tuning parameters.

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsDirectionConstraint.svg
   :align: center

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsAttitudeConstraint.svg
   :align: center

"""

#
#   Basilisk Scenario Script
#
#   Purpose:            Illustrates two spacecraft connected via a holonomic constraint
#   Author:             Andrew Morell
#   Creation Date:      May 14, 2024
#

# system imports
import os

# Basilisk imports
from Basilisk.architecture import messaging
from Basilisk.utilities import (SimulationBaseClass, orbitalMotion, macros, RigidBodyKinematics)
from Basilisk.simulation import (spacecraft, constraintDynamicEffector, gravityEffector, svIntegrators)
import matplotlib.pyplot as plt

# Utility imports
import numpy as np

from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

def run(show_plots, env):
    """
    The scenario can be run with the following setup parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        env (str): Choose whether in Earth orbit ``Gravity`` or deep space ``NoGravity``

    """

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Create simulation variable names
    simTaskName = "simTask"  # arbitrary name (don't change)
    simProcessName = "simProcess"  # arbitrary name (don't change)

    # Create the simulation process and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.1)  # update process rate update time
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Create both spacecraft
    scObject1 = spacecraft.Spacecraft()
    scObject1.ModelTag = "spacecraftBody1"
    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "spacecraftBody2"

    # Set the integrator to RKF45
    integratorObject1 = svIntegrators.svIntegratorRKF45(scObject1)
    scObject1.setIntegrator(integratorObject1)
    # Sync dynamics integration across both spacecraft
    scObject1.syncDynamicsIntegration(scObject2)

    # Define mass properties of the rigid hub of both spacecraft
    scObject1.hub.mHub = 750.0
    scObject1.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject1.hub.IHubPntBc_B = [[600.0, 0.0, 0.0], [0.0, 600.0, 0.0], [0.0, 0.0, 600.0]]
    scObject2.hub.mHub = 750.0
    scObject2.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject2.hub.IHubPntBc_B = [[600.0, 0.0, 0.0], [0.0, 600.0, 0.0], [0.0, 0.0, 600.0]]

    # Add Earth gravity to the simulation if requested
    if env == 'Gravity':
        earthGravBody = gravityEffector.GravBodyData()
        earthGravBody.planetName = "earth_planet_data"
        earthGravBody.mu = 0.3986004415E+15  # [meters^3/s^2]
        earthGravBody.isCentralBody = True
        scObject1.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])
        scObject2.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])
        # set orbital altitude to LEO
        oe = orbitalMotion.ClassicElements()
        oe.a = earthGravBody.radEquator + 7500e3  # meters
        oe.e = 0.01
        oe.i = 30.0 * macros.D2R
        oe.Omega = 60.0 * macros.D2R
        oe.omega = 15.0 * macros.D2R
        oe.f = 90.0 * macros.D2R
        r_B2N_N_0, rDot_B2N_N = orbitalMotion.elem2rv(earthGravBody.mu, oe)
    else: # If no gravity requested, place in free-floating space
        r_B2N_N_0 = np.array([1,1,1])
        rDot_B2N_N = np.array([1,1,1])

    # With initial attitudes at zero (B1, B2, and N frames all initially aligned)
    dir = r_B2N_N_0/np.linalg.norm(r_B2N_N_0)
    l = 0.1
    COMoffset = 0.1 # distance from COM to where the arm connects to the spacecraft hub, same for both spacecraft [meters]
    r_P1B1_B1 = np.dot(dir,COMoffset)
    r_P2B2_B2 = np.dot(-dir,COMoffset)
    r_P2P1_B1Init = np.dot(dir,l)
    r_B1N_N_0 = r_B2N_N_0 + r_P2B2_B2 - r_P2P1_B1Init - r_P1B1_B1
    rDot_B1N_N = rDot_B2N_N

    # Compute rotational states
    # let C be the frame at the combined COM of the two vehicles
    r_CN_N = (r_B1N_N_0 * scObject1.hub.mHub + r_B2N_N_0 * scObject2.hub.mHub) / (scObject1.hub.mHub + scObject2.hub.mHub)
    r_B1C_N = r_B1N_N_0 - r_CN_N
    r_B2C_N = r_B2N_N_0 - r_CN_N
    # compute relative velocity due to spin and COM offset
    target_spin = [0.01,0.01,0.01]
    omega_CN_N = np.array(target_spin)
    omega_B1N_B1_0 = omega_CN_N
    omega_B2N_B2_0 = omega_CN_N
    dv_B1C_N = np.cross(omega_CN_N,r_B1C_N)
    dv_B2C_N = np.cross(omega_CN_N,r_B2C_N)
    rDot_B1N_N_0 = rDot_B1N_N + dv_B1C_N
    rDot_B2N_N_0 = rDot_B2N_N + dv_B2C_N

    # Set the initial values for all spacecraft states
    scObject1.hub.r_CN_NInit = r_B1N_N_0
    scObject1.hub.v_CN_NInit = rDot_B1N_N_0
    scObject1.hub.omega_BN_BInit = omega_B1N_B1_0
    scObject2.hub.r_CN_NInit = r_B2N_N_0
    scObject2.hub.v_CN_NInit = rDot_B2N_N_0
    scObject2.hub.omega_BN_BInit = omega_B2N_B2_0

    # Create the constraint effector module
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    # Set up the constraint effector physical parameters
    constraintEffector.setR_P1B1_B1(r_P1B1_B1)
    constraintEffector.setR_P2B2_B2(r_P2B2_B2)
    constraintEffector.setR_P2P1_B1Init(r_P2P1_B1Init)
    constraintEffector.setAlpha(1E2)
    constraintEffector.setBeta(1e3)
    constraintEffector.ModelTag = "constraintEffector"

    # Add the constraint to both spacecraft
    scObject1.addDynamicEffector(constraintEffector)
    scObject2.addDynamicEffector(constraintEffector)

    # Add the modules to runtime call list
    scSim.AddModelToTask(simTaskName, scObject1)
    scSim.AddModelToTask(simTaskName, scObject2)
    scSim.AddModelToTask(simTaskName, constraintEffector)

    # Record the spacecraft states
    datLog1 = scObject1.scStateOutMsg.recorder()
    datLog2 = scObject2.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, datLog1)
    scSim.AddModelToTask(simTaskName, datLog2)

    # Initialize the simulation
    scSim.InitializeSimulation()
    scSim.ShowExecutionOrder()

    # Setup and run the simulation
    stopTime = macros.min2nano(1)
    scSim.ConfigureStopTime(stopTime)
    scSim.ExecuteSimulation()

    # Retrieve the recorded spacecraft states
    constraintTimeData = datLog1.times() * macros.NANO2SEC
    r_B1N_N_hist = datLog1.r_BN_N
    sigma_B1N_hist = datLog1.sigma_BN
    r_B2N_N_hist = datLog2.r_BN_N
    sigma_B2N_hist = datLog2.sigma_BN

    # Compute constraint violations
    r_B1N_B1 = np.empty(r_B1N_N_hist.shape)
    r_B2N_B1 = np.empty(r_B2N_N_hist.shape)
    r_P2B2_B1 = np.empty(r_B1N_N_hist.shape)
    sigma_B2B1 = np.empty(sigma_B1N_hist.shape)
    for i in range(r_B1N_N_hist.shape[0]):
        dcm_B1N = RigidBodyKinematics.MRP2C(sigma_B1N_hist[i,:])
        r_B1N_B1[i,:] = dcm_B1N@r_B1N_N_hist[i,:]
        r_B2N_B1[i,:] = dcm_B1N@r_B2N_N_hist[i,:]
        dcm_NB2 = np.transpose(RigidBodyKinematics.MRP2C(sigma_B2N_hist[i,:]))
        r_P2B2_B1[i,:] = dcm_B1N@dcm_NB2@r_P2B2_B2
        sigma_B2B1[i,:] = RigidBodyKinematics.subMRP(sigma_B2N_hist[i,:],sigma_B1N_hist[i,:])
    psi_B1 = r_B1N_B1 + r_P1B1_B1 + r_P2P1_B1Init - (r_B2N_B1 + r_P2B2_B1)

    #
    # Plotting Results
    #
    figureList = {}

    plt.close("all")
    plt.figure(1)
    plt.clf()
    for i in range(3):
        plt.semilogy(constraintTimeData, np.abs(psi_B1[:, i]))
    plt.semilogy(constraintTimeData, np.linalg.norm(psi_B1,axis=1))
    plt.legend([r'$\psi_1$',r'$\psi_2$',r'$\psi_3$',r'$\psi$ magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'relative connection position: $\psi$ (meters)')
    plt.title('Direction Constraint Violation Components')
    pltName = fileName + "directionConstraint"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    plt.clf()
    for i in range(3):
        plt.semilogy(constraintTimeData, np.abs(4*np.arctan(sigma_B2B1[:, i]) * macros.R2D))
    plt.semilogy(constraintTimeData, np.linalg.norm(4*np.arctan(sigma_B2B1) * macros.R2D,axis=1))
    plt.legend([r'$\phi_1$',r'$\phi_2$',r'$\phi_3$',r'$\phi$ magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'relative attitude angle: $\phi$ (deg)')
    plt.title('Attitude Constraint Violation Components')
    pltName = fileName + "attitudeConstraint"
    figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()
    plt.close("all") # close the plots being saved off to avoid over-writing old and new figures

    return figureList

if __name__ == "__main__":
    run(
        True,       # show_plots: True or False
        'Gravity'   # env: either 'Gravity' or 'NoGravity'
    )
