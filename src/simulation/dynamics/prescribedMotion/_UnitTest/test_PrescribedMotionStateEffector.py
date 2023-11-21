# ISC License
#
# Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        prescribedMotion integrated unit test with prescribedRot1DOF and prescribedTranslation
#   Author:             Leah Kiner
#   Creation Date:      Jan 10, 2022
#

import pytest
import inspect
import os
import numpy as np
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
import matplotlib
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import prescribedRot1DOF, prescribedTrans
from Basilisk.simulation import spacecraft, prescribedMotionStateEffector, gravityEffector
from Basilisk.utilities import macros, RigidBodyKinematics as rbk
from Basilisk.architecture import messaging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

matplotlib.rc('xtick', labelsize=16)
matplotlib.rc('ytick', labelsize=16)

# Vary the simulation parameters for pytest
@pytest.mark.parametrize("rotTest", [True, False])
@pytest.mark.parametrize("thetaInit", [0, np.pi/18])
@pytest.mark.parametrize("theta_Ref", [np.pi/36])
@pytest.mark.parametrize("posInit", [0, 0.2])
@pytest.mark.parametrize("posRef", [0.1])
@pytest.mark.parametrize("accuracy", [1e-8])
def test_PrescribedMotionTestFunction(show_plots, rotTest, thetaInit, theta_Ref, posInit, posRef, accuracy):
    r"""
    **Validation Test Description**

    The unit test for this module is an integrated test with two flight software profiler modules. This is required
    because the dynamics module must be connected to a flight software profiler module to define the states of the
    prescribed secondary body that is connected to the rigid spacecraft hub. The integrated test for this module has
    two simple scenarios it is testing. The first scenario prescribes a 1 DOF rotational attitude maneuver for the
    prescribed body using the :ref:`prescribedRot1DOF` flight software module. The second scenario prescribes a
    translational maneuver for the prescribed body using the :ref:`prescribedTrans` flight software module.

    This unit test ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
    initial and reference PRV angles and maximum angular accelerations. The final prescribed attitude and angular
    velocity magnitude are compared with the reference values. This unit test also ensures that the profiled
    translational maneuver is properly computed for a series of initial and reference positions and maximum
    accelerations. The final prescribed position and velocity magnitudes are compared with the reference values.
    Additionally for each scenario, the conservation quantities of orbital angular momentum, rotational angular
    momentum, and orbital energy are checked to validate the module dynamics.

    **Test Parameters**

    Args:
        rotTest (bool): (True) Runs the rotational motion test. (False) Runs the translational motion test.
        thetaInit (float): [rad] Initial PRV angle of the F frame with respect to the M frame
        theta_Ref (float): [rad] Reference PRV angle of the F frame with respect to the M frame
        thetaDDotMax (float): [rad/s^2] Maximum angular acceleration for the attitude maneuver
        scalarPosInit (float): [m] Initial scalar position of the F frame with respect to the M frame
        scalarPosRef (float): [m] Reference scalar position of the F frame with respect to the M frame
        scalarAccelMax (float): [m/s^2] Maximum acceleration for the translational maneuver
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This unit test ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
    initial and reference PRV angles and maximum angular accelerations. The final prescribed angle ``theta_FM_Final``
    and angular velocity magnitude ``thetaDot_Final`` are compared with the reference values ``theta_Ref`` and
    ``thetaDot_Ref``, respectively. This unit test also ensures that the profiled translational maneuver is properly
    computed for a series of initial and reference positions and maximum accelerations. The final prescribed position
    magnitude ``r_FM_M_Final`` and velocity magnitude ``rPrime_FM_M_Final`` are compared with the reference values
    ``r_FM_M_Ref`` and ``rPrime_FM_M_Ref``, respectively. Additionally for each scenario, the conservation quantities
    of orbital angular momentum, rotational angular momentum, and orbital energy are checked to validate the module
    dynamics.
    """

    [testResults, testMessage] = PrescribedMotionTestFunction(show_plots, rotTest, thetaInit, theta_Ref, posInit, posRef, accuracy)

    assert testResults < 1, testMessage


def PrescribedMotionTestFunction(show_plots, rotTest, thetaInit, theta_Ref, posInit, posRef, accuracy):
    """Call this routine directly to run the unit test."""
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testIncrement = 0.1  # [s]
    testProcessRate = macros.sec2nano(testIncrement)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add the spacecraft module to test file
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Define the mass properties of the rigid spacecraft hub
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial inertial hub states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.omega_BN_BInit = np.array([0.0, 0.0, 0.0])
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]

    # Add the scObject to the runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add the prescribedMotion dynamics module to test file
    platform = prescribedMotionStateEffector.PrescribedMotionStateEffector()

    # Define the state effector properties
    transAxis_M = np.array([1.0, 0.0, 0.0])
    rotAxis_M = np.array([1.0, 0.0, 0.0])
    r_FM_M = posInit * transAxis_M
    prvInit_FM = thetaInit * rotAxis_M
    sigma_FM = rbk.PRV2MRP(prvInit_FM)

    platform.mass = 100.0
    platform.IPntFc_F = [[50.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    platform.r_MB_B = [0.0, 0.0, 0.0]
    platform.r_FcF_F = [0.0, 0.0, 0.0]
    platform.r_FM_M = r_FM_M
    platform.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    platform.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
    platform.omega_FM_F = np.array([0.0, 0.0, 0.0])
    platform.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
    platform.sigma_FM = sigma_FM
    platform.omega_MB_B = [0.0, 0.0, 0.0]
    platform.omegaPrime_MB_B = [0.0, 0.0, 0.0]
    platform.sigma_MB = [0.0, 0.0, 0.0]
    platform.ModelTag = "Platform"

    # Add platform to spacecraft
    scObject.addStateEffector(platform)

    # Add the test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, platform)

    if rotTest:
    
        # ** ** ** ** ** ROTATIONAL 1 DOF INTEGRATED TEST: ** ** ** ** **

        # Create an instance of the prescribedRot1DOF module to be tested
        PrescribedRot1DOF = prescribedRot1DOF.prescribedRot1DOF()
        PrescribedRot1DOF.ModelTag = "prescribedRot1DOF"

        # Add the prescribedRot1DOF test module to runtime call list
        unitTestSim.AddModelToTask(unitTaskName, PrescribedRot1DOF)

        # Initialize the prescribedRot1DOF test module configuration data
        accelMax = 0.01  # [rad/s^2]
        #accelMax = np.pi / 180  # [rad/s^2]
        PrescribedRot1DOF.r_FM_M = r_FM_M
        PrescribedRot1DOF.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
        PrescribedRot1DOF.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
        PrescribedRot1DOF.rotAxis_M = rotAxis_M
        PrescribedRot1DOF.thetaDDotMax = accelMax
        PrescribedRot1DOF.omega_FM_F = np.array([0.0, 0.0, 0.0])
        PrescribedRot1DOF.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
        PrescribedRot1DOF.sigma_FM = sigma_FM

        # Create the prescribedRot1DOF input message
        thetaDot_Ref = 0.0  # [rad/s]
        SpinningBodyMessageData = messaging.HingedRigidBodyMsgPayload()
        SpinningBodyMessageData.theta = theta_Ref
        SpinningBodyMessageData.thetaDot = thetaDot_Ref
        SpinningBodyMessage = messaging.HingedRigidBodyMsg().write(SpinningBodyMessageData)
        PrescribedRot1DOF.spinningBodyInMsg.subscribeTo(SpinningBodyMessage)
        
        # Connect the PrescribedRot1DOF module's prescribedMotion output message to the prescribedMotion module's prescribedMotion input message
        platform.prescribedMotionInMsg.subscribeTo(PrescribedRot1DOF.prescribedMotionOutMsg)

        # Add Earth gravity to the simulation
        earthGravBody = gravityEffector.GravBodyData()
        earthGravBody.planetName = "earth_planet_data"
        earthGravBody.mu = 0.3986004415E+15
        earthGravBody.isCentralBody = True
        scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

        # Add energy and momentum variables to log
        scObjectLog = scObject.logger(["totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totRotEnergy"])
        unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

        # Add other states to log
        scStateData = scObject.scStateOutMsg.recorder()
        prescribedStateData = platform.prescribedMotionOutMsg.recorder()
        dataLog = PrescribedRot1DOF.prescribedMotionOutMsg.recorder()
        unitTestSim.AddModelToTask(unitTaskName, scStateData)
        unitTestSim.AddModelToTask(unitTaskName, prescribedStateData)
        unitTestSim.AddModelToTask(unitTaskName, dataLog)

        # Initialize the simulation
        unitTestSim.InitializeSimulation()

        # Set the simulation time
        simTime = np.sqrt(((0.5 * np.abs(theta_Ref - thetaInit)) * 8) / accelMax) + 3 * testIncrement
        unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))

        # Begin the simulation
        unitTestSim.ExecuteSimulation()

        # Extract the logged data
        orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)
        orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
        rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
        rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)
        omega_BN_B = scStateData.omega_BN_B
        r_BN_N = scStateData.r_BN_N
        sigma_BN = scStateData.sigma_BN
        omega_FM_F = dataLog.omega_FM_F
        omegaPrime_FM_F = dataLog.omegaPrime_FM_F
        sigma_FM = dataLog.sigma_FM
        timespan = dataLog.times()
        thetaDot_Final = np.linalg.norm(omega_FM_F[-1, :])
        sigma_FM_Final = sigma_FM[-1, :]
        theta_FM_Final = 4 * np.arctan(np.linalg.norm(sigma_FM_Final))

        # Setup the conservation quantities
        initialOrbAngMom_N = [[orbAngMom_N[0, 1], orbAngMom_N[0, 2], orbAngMom_N[0, 3]]]
        finalOrbAngMom = [orbAngMom_N[-1]]
        initialRotAngMom_N = [[rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]]
        finalRotAngMom = [rotAngMom_N[-1]]
        initialOrbEnergy = [[orbEnergy[0, 1]]]
        finalOrbEnergy = [orbEnergy[-1]]

        # Convert the logged sigma_FM MRPs to a scalar theta_FM array
        n = len(timespan)
        theta_FM = []
        for i in range(n):
            theta_FM.append((180 / np.pi) * (4 * np.arctan(np.linalg.norm(sigma_FM[i, :]))))

        plt.close("all")

        # Plot theta_FM
        theta_Ref_plotting = np.ones(len(timespan)) * theta_Ref
        thetaInit_plotting = np.ones(len(timespan)) * thetaInit
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, theta_FM, label=r'$\Phi$')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * theta_Ref_plotting, '--', label=r'$\Phi_{Ref}$')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * thetaInit_plotting, '--', label=r'$\Phi_{0}$')
        plt.title(r'$\Phi_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
        plt.ylabel('(deg)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='center right', prop={'size': 16})

        # Plot omega_FM_F
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omega_FM_F[:, 0], label=r'$\omega_{1}$')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omega_FM_F[:, 1], label=r'$\omega_{2}$')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omega_FM_F[:, 2], label=r'$\omega_{3}$')
        plt.title(r'${}^\mathcal{F} \omega_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
        plt.ylabel('(deg/s)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='upper right', prop={'size': 16})

        # Plotting omegaPrime_FM_F
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omegaPrime_FM_F[:, 0], label='1')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omegaPrime_FM_F[:, 1], label='2')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omegaPrime_FM_F[:, 2], label='3')
        plt.title(r'${}^\mathcal{F} \omega Prime_{\mathcal{F}/\mathcal{M}}$ Profiled Angular Acceleration', fontsize=14)
        plt.ylabel(r'(deg/$s^2$)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='upper right', prop={'size': 16})

        # Plot r_BN_N
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, r_BN_N[:, 0], label=r'$r_{1}$')
        plt.plot(timespan * macros.NANO2SEC, r_BN_N[:, 1], label=r'$r_{2}$')
        plt.plot(timespan * macros.NANO2SEC, r_BN_N[:, 2], label=r'$r_{3}$')
        plt.title(r'${}^\mathcal{N} r_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial Trajectory', fontsize=14)
        plt.ylabel('(m)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='center left', prop={'size': 16})

        # Plot sigma_BN
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, sigma_BN[:, 0], label=r'$\sigma_{1}$')
        plt.plot(timespan * macros.NANO2SEC, sigma_BN[:, 1], label=r'$\sigma_{2}$')
        plt.plot(timespan * macros.NANO2SEC, sigma_BN[:, 2], label=r'$\sigma_{3}$')
        plt.title(r'$\sigma_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial MRP Attitude', fontsize=14)
        plt.ylabel('', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='center left', prop={'size': 16})

        # Plot omega_BN_B
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omega_BN_B[:, 0], label=r'$\omega_{1}$')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omega_BN_B[:, 1], label=r'$\omega_{2}$')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omega_BN_B[:, 2], label=r'$\omega_{3}$')
        plt.title(r'Spacecraft Hub Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$', fontsize=14)
        plt.xlabel('Time (s)', fontsize=16)
        plt.ylabel('(deg/s)', fontsize=16)
        plt.legend(loc='lower right', prop={'size': 16})

        # Plotting: Conservation quantities
        plt.figure()
        plt.clf()
        plt.plot(orbAngMom_N[:, 0] * macros.NANO2SEC, (orbAngMom_N[:, 1] - orbAngMom_N[0, 1]) / orbAngMom_N[0, 1],
                 orbAngMom_N[:, 0] * macros.NANO2SEC, (orbAngMom_N[:, 2] - orbAngMom_N[0, 2]) / orbAngMom_N[0, 2],
                 orbAngMom_N[:, 0] * macros.NANO2SEC, (orbAngMom_N[:, 3] - orbAngMom_N[0, 3]) / orbAngMom_N[0, 3])
        plt.title('Orbital Angular Momentum Relative Difference', fontsize=14)
        plt.ylabel('(Nms)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)

        plt.figure()
        plt.clf()
        plt.plot(orbEnergy[:, 0] * macros.NANO2SEC, (orbEnergy[:, 1] - orbEnergy[0, 1]) / orbEnergy[0, 1])
        plt.title('Orbital Energy Relative Difference', fontsize=14)
        plt.ylabel('Energy (J)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)

        plt.figure()
        plt.clf()
        plt.plot(rotAngMom_N[:, 0] * macros.NANO2SEC, (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]),
                 rotAngMom_N[:, 0] * macros.NANO2SEC, (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]),
                 rotAngMom_N[:, 0] * macros.NANO2SEC, (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]))
        plt.title('Rotational Angular Momentum Difference', fontsize=14)
        plt.ylabel('(Nms)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)

        plt.figure()
        plt.clf()
        plt.plot(rotEnergy[:, 0] * macros.NANO2SEC, (rotEnergy[:, 1] - rotEnergy[0, 1]))
        plt.title('Total Energy Difference', fontsize=14)
        plt.ylabel('Energy (J)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)

        if show_plots:
            plt.show()
        plt.close("all")

        # Begin the test analysis
        finalOrbAngMom = np.delete(finalOrbAngMom, 0, axis=1)  # remove the time column
        finalRotAngMom = np.delete(finalRotAngMom, 0, axis=1)  # remove the time column
        finalOrbEnergy = np.delete(finalOrbEnergy, 0, axis=1)  # remove the time column

        for i in range(0, len(initialOrbAngMom_N)):
            if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i], initialOrbAngMom_N[i], 3, accuracy):
                testFailCount += 1
                testMessages.append(
                    "FAILED: Prescribed Motion integrated test failed orbital angular momentum unit test")

        for i in range(0, len(initialRotAngMom_N)):
            if not unitTestSupport.isArrayEqual(finalRotAngMom[i], initialRotAngMom_N[i], 3, accuracy):
                testFailCount += 1
                testMessages.append(
                    "FAILED: Prescribed Motion integrated test failed rotational angular momentum unit test")

        for i in range(0, len(initialOrbEnergy)):
            if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i], initialOrbEnergy[i], 1, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Prescribed Motion integrated test failed orbital energy unit test")

        # Check to ensure the initial angle rate converged to the reference angle rate
        if not unitTestSupport.isDoubleEqual(thetaDot_Final, thetaDot_Ref, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + PrescribedRot1DOF.ModelTag + "thetaDot_Final and thetaDot_Ref do not match")

        # Check to ensure the initial angle converged to the reference angle
        if not unitTestSupport.isDoubleEqual(theta_FM_Final, theta_Ref, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + PrescribedRot1DOF.ModelTag + "theta_FM_Final and theta_Ref do not match")
            # testMessages.append("theta_FM_Final: " + str(theta_FM_Final) + " theta_Ref: " + str(theta_Ref))

        if testFailCount == 0:
            print("PASSED: " + "prescribedMotion and prescribedRot1DOF integrated test")

    else:

        # ** ** ** ** ** TRANSLATIONAL INTEGRATED TEST ** ** ** ** **

        # Create an instance of the prescribedTrans module to be tested
        PrescribedTrans = prescribedTrans.prescribedTrans()
        PrescribedTrans.ModelTag = "prescribedTrans"

        # Add the prescribedTrans test module to runtime call list
        unitTestSim.AddModelToTask(unitTaskName, PrescribedTrans)

        # Initialize the prescribedTrans test module configuration data
        accelMax = 0.005  # [m/s^2]
        PrescribedTrans.r_FM_M = r_FM_M
        PrescribedTrans.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
        PrescribedTrans.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
        PrescribedTrans.transAxis_M = transAxis_M
        PrescribedTrans.scalarAccelMax = accelMax
        PrescribedTrans.omega_FM_F = np.array([0.0, 0.0, 0.0])
        PrescribedTrans.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
        PrescribedTrans.sigma_FM = sigma_FM

        # Create the prescribedTrans input message
        velRef = 0.0  # [m/s]
        PrescribedTransMessageData = messaging.PrescribedTransMsgPayload()
        PrescribedTransMessageData.scalarPos = posRef
        PrescribedTransMessageData.scalarVel = velRef
        PrescribedTransMessage = messaging.PrescribedTransMsg().write(PrescribedTransMessageData)
        PrescribedTrans.prescribedTransInMsg.subscribeTo(PrescribedTransMessage)

        # Connect the PrescribedTrans module's prescribedMotion output message to the prescribedMotion module's prescribedMotion input message
        platform.prescribedMotionInMsg.subscribeTo(PrescribedTrans.prescribedMotionOutMsg)

        # Add Earth gravity to the simulation
        earthGravBody = gravityEffector.GravBodyData()
        earthGravBody.planetName = "earth_planet_data"
        earthGravBody.mu = 0.3986004415E+15
        earthGravBody.isCentralBody = True
        scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

        # Add energy and momentum variables to log
        scObjectLog = scObject.logger(["totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totRotEnergy"])
        unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

        # Add other states to log
        scStateData = scObject.scStateOutMsg.recorder()
        prescribedStateData = platform.prescribedMotionOutMsg.recorder()
        dataLog = PrescribedTrans.prescribedMotionOutMsg.recorder()
        unitTestSim.AddModelToTask(unitTaskName, scStateData)
        unitTestSim.AddModelToTask(unitTaskName, prescribedStateData)
        unitTestSim.AddModelToTask(unitTaskName, dataLog)

        # Initialize the simulation
        unitTestSim.InitializeSimulation()

        # Set the simulation time
        simTime = np.sqrt(((0.5 * np.abs(posRef - posInit)) * 8) / accelMax) + 3 * testIncrement
        unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))

        # Begin the simulation
        unitTestSim.ExecuteSimulation()

        # Extract the logged data
        orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)
        orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
        rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
        rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)
        r_BN_N = scStateData.r_BN_N
        sigma_BN = scStateData.sigma_BN
        omega_BN_B = scStateData.omega_BN_B
        r_FM_M = dataLog.r_FM_M
        rPrime_FM_M = dataLog.rPrime_FM_M
        rPrimePrime_FM_M = dataLog.rPrimePrime_FM_M
        timespan = dataLog.times()
        r_FM_M_Final = r_FM_M[-1, :]
        rPrime_FM_M_Final = rPrime_FM_M[-1, :]

        # Setup the conservation quantities
        initialOrbAngMom_N = [[orbAngMom_N[0, 1], orbAngMom_N[0, 2], orbAngMom_N[0, 3]]]
        finalOrbAngMom = [orbAngMom_N[-1]]
        initialRotAngMom_N = [[rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]]
        finalRotAngMom = [rotAngMom_N[-1]]
        initialOrbEnergy = [[orbEnergy[0, 1]]]
        finalOrbEnergy = [orbEnergy[-1]]
        initialRotEnergy = [[rotEnergy[0, 1]]]
        finalRotEnergy = [rotEnergy[-1]]

        # Plot r_FM_F
        r_FM_M_Ref = posRef * transAxis_M
        r_FM_M_1_Ref = np.ones(len(timespan)) * r_FM_M_Ref[0]
        r_FM_M_2_Ref = np.ones(len(timespan)) * r_FM_M_Ref[1]
        r_FM_M_3_Ref = np.ones(len(timespan)) * r_FM_M_Ref[2]

        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, r_FM_M[:, 0], label=r'$r_{1}$')
        plt.plot(timespan * macros.NANO2SEC, r_FM_M[:, 1], label=r'$r_{2}$')
        plt.plot(timespan * macros.NANO2SEC, r_FM_M[:, 2], label=r'$r_{3}$')
        plt.plot(timespan * macros.NANO2SEC, r_FM_M_1_Ref, '--', label=r'$r_{1 Ref}$')
        plt.plot(timespan * macros.NANO2SEC, r_FM_M_2_Ref, '--', label=r'$r_{2 Ref}$')
        plt.plot(timespan * macros.NANO2SEC, r_FM_M_3_Ref, '--', label=r'$r_{3 Ref}$')
        plt.title(r'${}^\mathcal{M} r_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
        plt.ylabel('(m)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='center left', prop={'size': 16})

        # Plot rPrime_FM_F
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, rPrime_FM_M[:, 0], label='1')
        plt.plot(timespan * macros.NANO2SEC, rPrime_FM_M[:, 1], label='2')
        plt.plot(timespan * macros.NANO2SEC, rPrime_FM_M[:, 2], label='3')
        plt.title(r'${}^\mathcal{M} rPrime_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
        plt.ylabel('(m/s)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='upper left', prop={'size': 16})

        # Plotting rPrimePrime_FM_M
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * rPrimePrime_FM_M[:, 0], label='1')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * rPrimePrime_FM_M[:, 1], label='2')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * rPrimePrime_FM_M[:, 2], label='3')
        plt.title(r'${}^\mathcal{M} rPrimePrime_{\mathcal{F}/\mathcal{M}}$ Profiled Acceleration', fontsize=14)
        plt.ylabel(r'(m/s$^2$)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='lower left', prop={'size': 16})

        # Plot r_BN_N
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, r_BN_N[:, 0], label=r'$r_{1}$')
        plt.plot(timespan * macros.NANO2SEC, r_BN_N[:, 1], label=r'$r_{2}$')
        plt.plot(timespan * macros.NANO2SEC, r_BN_N[:, 2], label=r'$r_{3}$')
        plt.title(r'${}^\mathcal{N} r_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial Trajectory', fontsize=14)
        plt.ylabel('(m)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='center left', prop={'size': 16})

        # Plot sigma_BN
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, sigma_BN[:, 0], label=r'$\sigma_{1}$')
        plt.plot(timespan * macros.NANO2SEC, sigma_BN[:, 1], label=r'$\sigma_{2}$')
        plt.plot(timespan * macros.NANO2SEC, sigma_BN[:, 2], label=r'$\sigma_{3}$')
        plt.title(r'$\sigma_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial MRP Attitude', fontsize=14)
        plt.ylabel('', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='lower left', prop={'size': 16})

        # Plot omega_BN_B
        plt.figure()
        plt.clf()
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omega_BN_B[:, 0], label=r'$\omega_{1}$')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omega_BN_B[:, 1], label=r'$\omega_{2}$')
        plt.plot(timespan * macros.NANO2SEC, (180 / np.pi) * omega_BN_B[:, 2], label=r'$\omega_{3}$')
        plt.title(r'Spacecraft Hub Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$', fontsize=14)
        plt.ylabel('(deg/s)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)
        plt.legend(loc='lower left', prop={'size': 16})

        # Plotting: Conservation quantities
        plt.figure()
        plt.clf()
        plt.plot(orbAngMom_N[:, 0] * macros.NANO2SEC, (orbAngMom_N[:, 1] - orbAngMom_N[0, 1]) / orbAngMom_N[0, 1],
                 orbAngMom_N[:, 0] * macros.NANO2SEC, (orbAngMom_N[:, 2] - orbAngMom_N[0, 2]) / orbAngMom_N[0, 2],
                 orbAngMom_N[:, 0] * macros.NANO2SEC, (orbAngMom_N[:, 3] - orbAngMom_N[0, 3]) / orbAngMom_N[0, 3])
        plt.title('Orbital Angular Momentum Relative Difference', fontsize=14)
        plt.ylabel('(Nms)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)

        plt.figure()
        plt.clf()
        plt.plot(orbEnergy[:, 0] * macros.NANO2SEC, (orbEnergy[:, 1] - orbEnergy[0, 1]) / orbEnergy[0, 1])
        plt.title('Orbital Energy Relative Difference', fontsize=14)
        plt.ylabel('Energy (J)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)

        plt.figure()
        plt.clf()
        plt.plot(rotAngMom_N[:, 0] * macros.NANO2SEC, (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]),
                 rotAngMom_N[:, 0] * macros.NANO2SEC, (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]),
                 rotAngMom_N[:, 0] * macros.NANO2SEC, (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]))
        plt.title('Rotational Angular Momentum Difference', fontsize=14)
        plt.ylabel('(Nms)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)

        plt.figure()
        plt.clf()
        plt.plot(rotEnergy[:, 0] * macros.NANO2SEC, (rotEnergy[:, 1] - rotEnergy[0, 1]))
        plt.title('Total Energy Difference', fontsize=14)
        plt.ylabel('Energy (J)', fontsize=16)
        plt.xlabel('Time (s)', fontsize=16)

        if show_plots:
            plt.show()
        plt.close("all")

        # Begin the test analysis
        finalOrbAngMom = np.delete(finalOrbAngMom, 0, axis=1)  # remove the time column
        finalRotAngMom = np.delete(finalRotAngMom, 0, axis=1)  # remove the time column
        finalOrbEnergy = np.delete(finalOrbEnergy, 0, axis=1)  # remove the time column

        for i in range(0, len(initialOrbAngMom_N)):
            if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i], initialOrbAngMom_N[i], 3, accuracy):
                testFailCount += 1
                testMessages.append(
                    "FAILED: Prescribed Motion integrated test failed orbital angular momentum unit test")

        for i in range(0, len(initialRotAngMom_N)):
            if not unitTestSupport.isArrayEqual(finalRotAngMom[i], initialRotAngMom_N[i], 3, accuracy):
                testFailCount += 1
                testMessages.append(
                    "FAILED: Prescribed Motion integrated test failed rotational angular momentum unit test")

        for i in range(0, len(initialOrbEnergy)):
            if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i], initialOrbEnergy[i], 1, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Prescribed Motion integrated test failed orbital energy unit test")

        # Check to ensure the initial velocity converged to the reference velocity
        rPrime_FM_M_Ref = np.array([0.0, 0.0, 0.0])
        if not unitTestSupport.isArrayEqual(rPrime_FM_M_Final, rPrime_FM_M_Ref, 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + PrescribedTrans.ModelTag + "rPrime_FM_M_Final and rPrime_FM_M_Ref do not match")
            testMessages.append("rPrime_FM_M_Final: " + str(rPrime_FM_M_Final) + " rPrime_FM_M_Ref: " + str(rPrime_FM_M_Ref))

        # Check to ensure the initial position converged to the reference position
        r_FM_M_Ref = np.array([posRef, 0.0, 0.0])
        if not unitTestSupport.isArrayEqual(r_FM_M_Final, r_FM_M_Ref, 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + PrescribedTrans.ModelTag + "r_FM_M_Final and r_FM_M_Ref do not match")
            testMessages.append("r_FM_M_Final: " + str(r_FM_M_Final) + " r_FM_M_Ref: " + str(r_FM_M_Ref))

        if testFailCount == 0:
            print("PASSED: " + "prescribedMotion and prescribedTrans integrated test")

    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    PrescribedMotionTestFunction(
        True,        # show_plots
        False,       # rotTest, (True: prescribedRot1DOF integrated test,
                     #           False: prescribedTrans integrated test)
         0.0,        # thetaInit [rad]
         np.pi / 12,    # theta_Ref [rad]
         0.0,        # posInit [m]
         0.5,        # posRef [m]
         1e-8        # accuracy
       )
