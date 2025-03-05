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
#   Module Name:        linearTranslationNDOF
#   Author:             Peter Johnson
#   Creation Date:      March 7, 2024
#

import inspect
import os

import numpy as np
import pytest
import numpy
import matplotlib.pyplot as plt

# plt.rcParams['text.usetex'] = True

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import spacecraft, linearTranslationNDOFStateEffector, gravityEffector
from Basilisk.architecture import messaging


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("function", ["translatingBodyNoInput"
    , "translatingBodyLockAxis"
    , "translatingBodyCommandedForce"])
def test_translatingBody(show_plots, function):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft with four single-axis translating rigid bodies attached to a rigid hub. Each
    translating body's center of mass is off-center from the translating axis and the position of the axis is arbitrary.
    The scenario includes gravity acting on both the spacecraft and the effector.

    **Description of Variables Being Tested**

    In this file we are checking the principles of conservation of energy and angular momentum. Both the orbital and
    rotational energy and angular momentum must be maintained when conservative forces like gravity are present.
    Therefore, the values of the variables

    - ``finalOrbAngMom``
    - ``finalOrbEnergy``
    - ``finalRotAngMom``
    - ``finalRotEnergy``

    against their initial values.
    """
    testFunction = globals().get(function)

    if testFunction is None:
        raise ValueError(f"Function '{function}' not found in global scope")

    testFunction(show_plots)

def translatingBodyNoInput(show_plots):
    r"""
    This test does not use any input messages or lock flags, so the links are free to move.
    """
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create four translating rigid bodies
    translatingBodyEffector = linearTranslationNDOFStateEffector.linearTranslationNDOFStateEffector()
    translatingBodyEffector.ModelTag = "translatingBodyEffector"

    # define properties
    translatingBody1 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody1.setMass(np.random.uniform(5.0, 50.0))
    translatingBody1.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                 [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                 [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody1.setDCM_FP([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    translatingBody1.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                [np.random.uniform(-1.0, 1.0)],
                                [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                [np.random.uniform(-1.0, 1.0)],
                                [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody1.setRhoInit(np.random.uniform(-5.0, 10.0))
    translatingBody1.setRhoDotInit(0.05)
    translatingBody1.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody1)

    translatingBody2 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody2.setMass(np.random.uniform(5.0, 50.0))
    translatingBody2.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody2.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody2.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody2.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody2.setRhoDotInit(0.05)
    translatingBody2.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody2)

    translatingBody3 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody3.setMass(np.random.uniform(5.0, 50.0))
    translatingBody3.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody3.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody3.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody3.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody3.setRhoDotInit(0.05)
    translatingBody3.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody3)

    translatingBody4 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody4.setMass(np.random.uniform(5.0, 50.0))
    translatingBody4.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody4.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody4.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setFHat_P([[0.0], [0.0], [1.0]])
    translatingBody4.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody4.setRhoDotInit(0.05)
    translatingBody4.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody4)

    # Add body to spacecraft
    scObject.addStateEffector(translatingBodyEffector)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBodyEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Add states to log
    rho1Data = translatingBodyEffector.translatingBodyOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho1Data)
    rho2Data = translatingBodyEffector.translatingBodyOutMsgs[1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho2Data)
    rho3Data = translatingBodyEffector.translatingBodyOutMsgs[2].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho3Data)
    rho4Data = translatingBodyEffector.translatingBodyOutMsgs[3].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho4Data)

    # Setup and run the simulation
    stopTime = 5000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    orbEnergy = scObjectLog.totOrbEnergy
    rho1 = rho1Data.rho
    rho1Dot = rho1Data.rhoDot
    rho2 = rho2Data.rho
    rho2Dot = rho2Data.rhoDot
    rho3 = rho3Data.rho
    rho3Dot = rho3Data.rhoDot
    rho4 = rho4Data.rho
    rho4Dot = rho4Data.rhoDot

    # Set up the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    initialRotEnergy = rotEnergy[0]
    finalRotEnergy = rotEnergy[-1]

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1, label=r'$\rho_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2, label=r'$\rho_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3, label=r'$\rho_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4, label=r'$\rho_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1Dot, label=r'$\dot{\rho}_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2Dot, label=r'$\dot{\rho}_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3Dot, label=r'$\dot{\rho}_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4Dot, label=r'$\dot{\rho}_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement Rate')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy, err_msg="Orbital energy is not constant.")
    np.testing.assert_allclose(finalRotEnergy, initialRotEnergy, rtol=accuracy,
                               err_msg="Rotational energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy,
                                   err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy,
                                   err_msg="Rotational angular momentum is not constant.")


def translatingBodyLockAxis(show_plots):
    r"""
    This test locks the axis, so the displacement is kept constant throughout the simulation.
    """
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create four translating rigid bodies
    translatingBodyEffector = linearTranslationNDOFStateEffector.linearTranslationNDOFStateEffector()
    translatingBodyEffector.ModelTag = "translatingBodyEffector"

    # define properties
    translatingBody1 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody1.setMass(np.random.uniform(5.0, 50.0))
    translatingBody1.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody1.setDCM_FP([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    translatingBody1.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody1.setRhoInit(np.random.uniform(-5.0, 10.0))
    translatingBody1.setRhoDotInit(0.05)
    translatingBody1.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody1)

    translatingBody2 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody2.setMass(np.random.uniform(5.0, 50.0))
    translatingBody2.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody2.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody2.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody2.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody2.setRhoDotInit(0.05)
    translatingBody2.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody2)

    translatingBody3 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody3.setMass(np.random.uniform(5.0, 50.0))
    translatingBody3.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody3.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody3.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody3.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody3.setRhoDotInit(0.05)
    translatingBody3.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody3)

    translatingBody4 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody4.setMass(np.random.uniform(5.0, 50.0))
    translatingBody4.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody4.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody4.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setFHat_P([[0.0], [0.0], [1.0]])
    translatingBody4.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody4.setRhoDotInit(0.05)
    translatingBody4.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody4)

    # Add body to spacecraft
    scObject.addStateEffector(translatingBodyEffector)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBodyEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # create lock message
    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.effectorLockFlag = [1, 0, 0, 1]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    translatingBodyEffector.motorLockInMsg.subscribeTo(lockMsg)

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Add states to log
    rho1Data = translatingBodyEffector.translatingBodyOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho1Data)
    rho2Data = translatingBodyEffector.translatingBodyOutMsgs[1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho2Data)
    rho3Data = translatingBodyEffector.translatingBodyOutMsgs[2].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho3Data)
    rho4Data = translatingBodyEffector.translatingBodyOutMsgs[3].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho4Data)

    # Setup and run the simulation
    stopTime = 5000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    orbEnergy = scObjectLog.totOrbEnergy
    rho1 = rho1Data.rho
    rho1Dot = rho1Data.rhoDot
    rho2 = rho2Data.rho
    rho2Dot = rho2Data.rhoDot
    rho3 = rho3Data.rho
    rho3Dot = rho3Data.rhoDot
    rho4 = rho4Data.rho
    rho4Dot = rho4Data.rhoDot

    # Set up the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    initialRotEnergy = rotEnergy[0]
    finalRotEnergy = rotEnergy[-1]

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1, label=r'$\rho_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2, label=r'$\rho_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3, label=r'$\rho_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4, label=r'$\rho_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1Dot, label=r'$\dot{\rho}_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2Dot, label=r'$\dot{\rho}_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3Dot, label=r'$\dot{\rho}_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4Dot, label=r'$\dot{\rho}_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement Rate')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy,
                               err_msg="Orbital energy is not constant.")
    np.testing.assert_allclose(finalRotEnergy, initialRotEnergy, rtol=accuracy,
                               err_msg="Rotational energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy,
                                   err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy,
                                   err_msg="Rotational angular momentum is not constant.")


def translatingBodyCommandedForce(show_plots):
    r"""
    This test includes a commanded force to the link, so energy is not conserved.
    """
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create four translating rigid bodies
    translatingBodyEffector = linearTranslationNDOFStateEffector.linearTranslationNDOFStateEffector()
    translatingBodyEffector.ModelTag = "translatingBodyEffector"

    # define properties
    translatingBody1 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody1.setMass(np.random.uniform(5.0, 50.0))
    translatingBody1.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody1.setDCM_FP([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    translatingBody1.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody1.setRhoInit(np.random.uniform(-5.0, 10.0))
    translatingBody1.setRhoDotInit(0.05)
    translatingBody1.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody1)

    translatingBody2 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody2.setMass(np.random.uniform(5.0, 50.0))
    translatingBody2.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody2.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody2.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody2.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody2.setRhoDotInit(0.05)
    translatingBody2.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody2)

    translatingBody3 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody3.setMass(np.random.uniform(5.0, 50.0))
    translatingBody3.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody3.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody3.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody3.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody3.setRhoDotInit(0.05)
    translatingBody3.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody3)

    translatingBody4 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody4.setMass(np.random.uniform(5.0, 50.0))
    translatingBody4.setIPntFc_F([[np.random.uniform(5.0, 100.0), 0.0, 0.0],
                                  [0.0, np.random.uniform(5.0, 100.0), 0.0],
                                  [0.0, 0.0, np.random.uniform(5.0, 100.0)]])
    translatingBody4.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody4.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setFHat_P([[0.0], [0.0], [1.0]])
    translatingBody4.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody4.setRhoDotInit(0.05)
    translatingBody4.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody4)

    # Add body to spacecraft
    scObject.addStateEffector(translatingBodyEffector)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBodyEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Create the force message
    cmdArray = messaging.ArrayMotorForceMsgPayload()
    cmdArray.motorForce = [0.1, -0.2, 0.3, -0.15]  # [Nm]
    cmdMsg = messaging.ArrayMotorForceMsg().write(cmdArray)
    translatingBodyEffector.motorForceInMsg.subscribeTo(cmdMsg)

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Add states to log
    rho1Data = translatingBodyEffector.translatingBodyOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho1Data)
    rho2Data = translatingBodyEffector.translatingBodyOutMsgs[1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho2Data)
    rho3Data = translatingBodyEffector.translatingBodyOutMsgs[2].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho3Data)
    rho4Data = translatingBodyEffector.translatingBodyOutMsgs[3].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho4Data)

    # Setup and run the simulation
    stopTime = 5000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    orbEnergy = scObjectLog.totOrbEnergy
    rho1 = rho1Data.rho
    rho1Dot = rho1Data.rhoDot
    rho2 = rho2Data.rho
    rho2Dot = rho2Data.rhoDot
    rho3 = rho3Data.rho
    rho3Dot = rho3Data.rhoDot
    rho4 = rho4Data.rho
    rho4Dot = rho4Data.rhoDot

    # Set up the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    initialRotEnergy = rotEnergy[0]

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1, label=r'$\rho_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2, label=r'$\rho_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3, label=r'$\rho_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4, label=r'$\rho_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1Dot, label=r'$\dot{\rho}_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2Dot, label=r'$\dot{\rho}_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3Dot, label=r'$\dot{\rho}_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4Dot, label=r'$\dot{\rho}_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement Rate')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy,
                               err_msg="Orbital energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy,
                                   err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy,
                                   err_msg="Rotational angular momentum is not constant.")


if __name__ == "__main__":
    translatingBodyNoInput(True)
    # translatingBodyLockAxis(True)
    # translatingBodyCommandedForce(True)
