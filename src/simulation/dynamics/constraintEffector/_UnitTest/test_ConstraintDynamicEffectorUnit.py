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
#   Module Name:        constraintEffector
#   Author:             João Vaz Carneiro and Andrew Morell
#   Creation Date:      May 14, 2024
#

import inspect
import os
import pytest
import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, orbitalMotion, macros, RigidBodyKinematics
from Basilisk.simulation import spacecraft, constraintDynamicEffector, gravityEffector, svIntegrators

# uncomment this line if this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail()

@pytest.mark.parametrize("function", ["constraintEffectorOrbitalConservation", "constraintEffectorRotationalConservation"])
def test_constraintEffector(show_plots, function):
    r"""Module Unit Test
    **Validation Test Description**

    This unit test sets up two spacecraft connected by a holonomic constraint effector acting as a physical connection
    between them. The two spacecraft are set up with an identical mass and symmetrical inertias. One scenario includes
    gravity to check orbital conservation while the other scenario excludes gravity to check more sensitive constraint
    violations.

    **Description of Variables Being Tested**

    In this file we are checking the principles of conservation of energy and angular momentum. Both the orbital and
    rotational energy and angular momentum must be maintained. While Baumgarte stabilization is an approximate method,
    there is still an expected threshold below which these quantities are conserved:

    - ``combinedOrbAngMom``
    - ``combinedOrbEnergy``
    - ``combinedRotAngMom``
    - ``combinedRotEnergy``

    We are also checking the constraint violation magnitudes throughout the simulation. Due to the nature of the
    Baumgarte stabilization method, there must first be a constraint violation before a force is applied to correct
    the constraint violation and keep the spacecraft aligned. For slow moving spacecraft systems such as this, the
    constraint violations are expected to remain below a threshold dependent on how dynamic a scenario is. The
    threshold for the scenario with gravity therefore expects a larger threshold than the scenario without gravity.
    Accuracy checks based on prior testings are included here to ensure that future changes don't worsen performance,
    not to verify any theoretical thresholds. The direction and attitude constraint violations checked are:

    - ``psi_B1``
    - ``sigma_B2B1``

    """

    testFunction = globals().get(function)

    if testFunction is None:
        raise ValueError(f"Function '{function}' not found in global scope")

    result = testFunction(show_plots)


def constraintEffectorOrbitalConservation(show_plots):

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

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

    # Add Earth gravity to the simulation
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
    # Set up the constraint effector
    constraintEffector.ModelTag = "constraintEffector"
    constraintEffector.setR_P1B1_B1(r_P1B1_B1)
    constraintEffector.setR_P2B2_B2(r_P2B2_B2)
    constraintEffector.setR_P2P1_B1Init(r_P2P1_B1Init)
    constraintEffector.setAlpha(1E3)
    constraintEffector.setBeta(1e3)
    # Add constraints to both spacecraft
    scObject1.addDynamicEffector(constraintEffector)
    scObject2.addDynamicEffector(constraintEffector)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject1)
    unitTestSim.AddModelToTask(unitTaskName, scObject2)
    unitTestSim.AddModelToTask(unitTaskName, constraintEffector)

    # Record the spacecraft state message
    datLog1 = scObject1.scStateOutMsg.recorder()
    datLog2 = scObject2.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog1)
    unitTestSim.AddModelToTask(unitTaskName, datLog2)

    # Log energy and momentum variables
    conservationData1 = scObject1.logger(["totOrbAngMomPntN_N", "totOrbEnergy"])
    conservationData2 = scObject2.logger(["totOrbAngMomPntN_N", "totOrbEnergy"])
    unitTestSim.AddModelToTask(unitTaskName,conservationData1)
    unitTestSim.AddModelToTask(unitTaskName,conservationData2)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Setup and run the simulation
    stopTime = 1
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    # collect the recorded spacecraft states
    constraintTimeData = datLog1.times() * macros.NANO2SEC
    r_B1N_N_hist = datLog1.r_BN_N
    sigma_B1N_hist = datLog1.sigma_BN
    r_B2N_N_hist = datLog2.r_BN_N
    sigma_B2N_hist = datLog2.sigma_BN

    # collect the logged conservation variables
    conservationTimeData = conservationData1.times() * macros.NANO2SEC
    orbEnergy1 = conservationData1.totOrbEnergy
    orbAngMom1_N = conservationData1.totOrbAngMomPntN_N
    orbEnergy2 = conservationData2.totOrbEnergy
    orbAngMom2_N = conservationData2.totOrbAngMomPntN_N

    # Compute constraint violations
    r_B1N_B1 = np.empty(r_B1N_N_hist.shape)
    r_B2N_B1 = np.empty(r_B2N_N_hist.shape)
    r_P2B2_B1 = np.empty(r_B1N_N_hist.shape)
    sigma_B2B1 = np.empty(sigma_B1N_hist.shape)
    for i in range(orbAngMom1_N.shape[0]):
        dcm_B1N = RigidBodyKinematics.MRP2C(sigma_B1N_hist[i,:])
        r_B1N_B1[i,:] = dcm_B1N@r_B1N_N_hist[i,:]
        r_B2N_B1[i,:] = dcm_B1N@r_B2N_N_hist[i,:]
        dcm_NB2 = np.transpose(RigidBodyKinematics.MRP2C(sigma_B2N_hist[i,:]))
        r_P2B2_B1[i,:] = dcm_B1N@dcm_NB2@r_P2B2_B2
        sigma_B2B1[i,:] = RigidBodyKinematics.subMRP(sigma_B2N_hist[i,:],sigma_B1N_hist[i,:])
    psi_B1 = r_B1N_B1 + r_P1B1_B1 + r_P2P1_B1Init - (r_B2N_B1 + r_P2B2_B1)

    # Compute conservation quantities
    combinedOrbAngMom = orbAngMom1_N + orbAngMom2_N
    combinedOrbEnergy = orbEnergy1 + orbEnergy2

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    for i in range(3):
        plt.semilogy(constraintTimeData, np.abs(psi_B1[:, i]))
    plt.semilogy(constraintTimeData, np.linalg.norm(psi_B1,axis=1))
    plt.legend([r'$\psi_1$',r'$\psi_2$',r'$\psi_3$',r'$\psi$ magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'variation from fixed position: $\psi$ (meters)')
    plt.title('Direction Constraint Violation Components')

    plt.figure()
    plt.clf()
    for i in range(3):
        plt.semilogy(constraintTimeData, np.abs(4*np.arctan(sigma_B2B1[:, i]) * macros.R2D))
    plt.semilogy(constraintTimeData, np.linalg.norm(4*np.arctan(sigma_B2B1) * macros.R2D,axis=1))
    plt.legend([r'$\phi_1$',r'$\phi_2$',r'$\phi_3$',r'$\phi$ magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'relative attitude angle: $\phi$ (deg)')
    plt.title('Attitude Constraint Violation Components')

    plt.figure()
    plt.clf()
    plt.plot(conservationTimeData, (combinedOrbAngMom[:,0] - combinedOrbAngMom[0,0])/combinedOrbAngMom[0,0],
            conservationTimeData, (combinedOrbAngMom[:,1] - combinedOrbAngMom[0,1])/combinedOrbAngMom[0,1],
            conservationTimeData, (combinedOrbAngMom[:,2] - combinedOrbAngMom[0,2])/combinedOrbAngMom[0,2])
    plt.xlabel('time (seconds)')
    plt.ylabel('Relative Difference')
    plt.title('Combined Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(conservationTimeData, (combinedOrbEnergy - combinedOrbEnergy[0])/combinedOrbEnergy[0])
    plt.xlabel('time (seconds)')
    plt.ylabel('Relative Difference')
    plt.title('Combined Orbital Energy')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    dirCnstAccuracy = 1E-7
    attCnstAccuracy = 1E-4
    accuracy = 1E-12
    np.testing.assert_allclose(psi_B1, 0, atol=dirCnstAccuracy,
                               err_msg='direction constraint component magnitude exceeded in orbital conservation test')
    np.testing.assert_allclose(sigma_B2B1, 0, atol=attCnstAccuracy,
                               err_msg='attitude constraint component magnitude exceeded in orbital conservation test')
    for i in range(3):
        np.testing.assert_allclose(orbAngMom1_N[:,i]+orbAngMom2_N[:,i], orbAngMom1_N[0,i]+orbAngMom2_N[0,i], atol=accuracy,
                                   err_msg='orbital angular momentum difference component magnitude exceeded')
    np.testing.assert_allclose(orbEnergy1+orbEnergy2, orbEnergy1[0]+orbEnergy2[0], atol=accuracy,
                               err_msg='orbital energy difference magnitude exceeded')

def constraintEffectorRotationalConservation(show_plots):

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

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

    # With initial attitudes at zero (B1, B2, and N frames all initially aligned)
    r_B2N_N_0 = np.array([1,1,1])
    rDot_B2N_N = np.array([1,1,1])
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
    # Set up the constraint effector
    constraintEffector.ModelTag = "constraintEffector"
    constraintEffector.setR_P1B1_B1(r_P1B1_B1)
    constraintEffector.setR_P2B2_B2(r_P2B2_B2)
    constraintEffector.setR_P2P1_B1Init(r_P2P1_B1Init)
    constraintEffector.setAlpha(1E3)
    constraintEffector.setBeta(1e3)
    # Add constraints to both spacecraft
    scObject1.addDynamicEffector(constraintEffector)
    scObject2.addDynamicEffector(constraintEffector)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject1)
    unitTestSim.AddModelToTask(unitTaskName, scObject2)
    unitTestSim.AddModelToTask(unitTaskName, constraintEffector)

    # Record the spacecraft state message
    datLog1 = scObject1.scStateOutMsg.recorder()
    datLog2 = scObject2.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog1)
    unitTestSim.AddModelToTask(unitTaskName, datLog2)

    # Log energy and momentum variables
    conservationData1 = scObject1.logger(["totRotAngMomPntC_N", "totRotEnergy"])
    conservationData2 = scObject2.logger(["totRotAngMomPntC_N", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName,conservationData1)
    unitTestSim.AddModelToTask(unitTaskName,conservationData2)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Setup and run the simulation
    stopTime = 1
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    # collect the recorded spacecraft states
    constraintTimeData = datLog1.times() * macros.NANO2SEC
    r_B1N_N_hist = datLog1.r_BN_N
    rdot_B1N_N_hist = datLog1.v_BN_N
    sigma_B1N_hist = datLog1.sigma_BN
    omega_B1N_B1_hist = datLog1.omega_BN_B
    r_B2N_N_hist = datLog2.r_BN_N
    rdot_B2N_N_hist = datLog2.v_BN_N
    sigma_B2N_hist = datLog2.sigma_BN
    omega_B2N_B2_hist = datLog2.omega_BN_B

    # collect the logged conservation variables
    conservationTimeData = conservationData1.times() * macros.NANO2SEC
    rotAngMom1PntC1_N = conservationData1.totRotAngMomPntC_N
    rotEnergy1 = conservationData1.totRotEnergy
    rotAngMom2PntC2_N = conservationData2.totRotAngMomPntC_N
    rotEnergy2 = conservationData2.totRotEnergy

    # Compute constraint violations
    r_B1N_B1 = np.empty(r_B1N_N_hist.shape)
    r_B2N_B1 = np.empty(r_B2N_N_hist.shape)
    r_P2B2_B1 = np.empty(r_B1N_N_hist.shape)
    for i in range(rotAngMom1PntC1_N.shape[0]):
        dcm_B1N = RigidBodyKinematics.MRP2C(sigma_B1N_hist[i,:])
        r_B1N_B1[i,:] = dcm_B1N@r_B1N_N_hist[i,:]
        r_B2N_B1[i,:] = dcm_B1N@r_B2N_N_hist[i,:]
        dcm_NB2 = np.transpose(RigidBodyKinematics.MRP2C(sigma_B2N_hist[i,:]))
        r_P2B2_B1[i,:] = dcm_B1N@dcm_NB2@r_P2B2_B2
    psi_B1 = r_B1N_B1 + r_P1B1_B1 + r_P2P1_B1Init - (r_B2N_B1 + r_P2B2_B1)
    sigma_B2B1 = sigma_B2N_hist-sigma_B1N_hist

    # Compute conservation quantities
    r_CN_N_hist = (r_B1N_N_hist * scObject1.hub.mHub + r_B2N_N_hist * scObject2.hub.mHub)/(scObject1.hub.mHub+scObject2.hub.mHub)
    r_B1C_N_hist = r_B1N_N_hist - r_CN_N_hist
    r_B2C_N_hist = r_B2N_N_hist - r_CN_N_hist
    rdot_CN_N_hist = (rdot_B1N_N_hist * scObject1.hub.mHub + rdot_B2N_N_hist * scObject2.hub.mHub)/(scObject1.hub.mHub+scObject2.hub.mHub)
    rdot_B1C_N_hist = rdot_B1N_N_hist - rdot_CN_N_hist
    rdot_B2C_N_hist = rdot_B2N_N_hist - rdot_CN_N_hist
    rotAngMom1PntCT_N = np.empty(rotAngMom1PntC1_N.shape)
    rotAngMom2PntCT_N = np.empty(rotAngMom2PntC2_N.shape)
    for i1 in range(rotAngMom1PntC1_N.shape[0]):
        rotAngMom1PntCT_N[i1,:] = np.cross(r_CN_N_hist[i1,:],scObject1.hub.mHub*rdot_B1C_N_hist[i1,:]) + RigidBodyKinematics.MRP2C(sigma_B1N_hist[i1,:]).transpose()@(scObject1.hub.IHubPntBc_B@omega_B1N_B1_hist[i1,:]) + scObject1.hub.mHub*np.cross(r_B1C_N_hist[i1,:],rdot_B1C_N_hist[i1,:])
        rotAngMom2PntCT_N[i1,:] = np.cross(r_CN_N_hist[i1,:],scObject2.hub.mHub*rdot_B2C_N_hist[i1,:]) + RigidBodyKinematics.MRP2C(sigma_B2N_hist[i1,:]).transpose()@(scObject2.hub.IHubPntBc_B@omega_B2N_B2_hist[i1,:]) + scObject2.hub.mHub*np.cross(r_B2C_N_hist[i1,:],rdot_B2C_N_hist[i1,:])
    combinedRotAngMom = rotAngMom1PntCT_N + rotAngMom2PntCT_N
    combinedRotEnergy = rotEnergy1 + rotEnergy2

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    for i in range(3):
        plt.semilogy(constraintTimeData, np.abs(psi_B1[:, i]))
    plt.semilogy(constraintTimeData, np.linalg.norm(psi_B1,axis=1))
    plt.legend([r'$\psi_1$',r'$\psi_2$',r'$\psi_3$',r'$\psi$ magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'variation from fixed position: $\psi$ (meters)')
    plt.title('Direction Constraint Violation Components')

    plt.figure()
    plt.clf()
    for i in range(3):
        plt.semilogy(constraintTimeData, np.abs(4*np.arctan(sigma_B2B1[:, i]) * macros.R2D))
    plt.semilogy(constraintTimeData, np.linalg.norm(4*np.arctan(sigma_B2B1) * macros.R2D,axis=1))
    plt.legend([r'$\phi_1$',r'$\phi_2$',r'$\phi_3$',r'$\phi$ magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'relative attitude angle: $\phi$ (deg)')
    plt.title('Attitude Constraint Violation Components')

    plt.figure()
    plt.clf()
    plt.plot(conservationTimeData, (combinedRotAngMom[:,0] - combinedRotAngMom[0,0])/combinedRotAngMom[0,0],
             conservationTimeData, (combinedRotAngMom[:,1] - combinedRotAngMom[0,1])/combinedRotAngMom[0,1],
             conservationTimeData, (combinedRotAngMom[:,2] - combinedRotAngMom[0,2])/combinedRotAngMom[0,2])
    plt.xlabel('time (seconds)')
    plt.ylabel('Relative Difference')
    plt.title('Combined Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(conservationTimeData, (combinedRotEnergy - combinedRotEnergy[0])/combinedRotEnergy[0])
    plt.xlabel('time (seconds)')
    plt.ylabel('Relative Difference')
    plt.title('Combined Rotational Energy')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1E-12
    np.testing.assert_allclose(psi_B1, 0, atol=accuracy,
                               err_msg='direction constraint component magnitude exceeded in rotational conservation test')
    np.testing.assert_allclose(sigma_B2B1, 0, atol=accuracy,
                               err_msg='attitude constraint component magnitude exceeded in rotational conservation test')
    for i in range(3):
        np.testing.assert_allclose(rotAngMom1PntCT_N[:,i]+rotAngMom2PntCT_N[:,i], rotAngMom1PntCT_N[0,i]+rotAngMom2PntCT_N[0,i], atol=accuracy,
                                   err_msg='rotational angular momentum difference component magnitude exceeded')
    np.testing.assert_allclose(rotEnergy1+rotEnergy2, rotEnergy1[0]+rotEnergy2[0], atol=accuracy,
                               err_msg='rotational energy difference magnitude exceeded')

if __name__ == "__main__":
    # constraintEffectorOrbitalConservation(True)
    constraintEffectorRotationalConservation(True)
