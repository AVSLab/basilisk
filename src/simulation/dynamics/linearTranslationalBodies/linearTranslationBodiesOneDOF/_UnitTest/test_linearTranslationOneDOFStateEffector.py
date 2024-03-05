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
#   Module Name:        translatingBodies
#   Author:             Peter Johnson
#   Creation Date:      March 6, 2024
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
from Basilisk.simulation import spacecraft, linearTranslationOneDOFStateEffector, gravityEffector
from Basilisk.architecture import messaging


# uncomment this line if this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail()
# provide a unique test method name, starting with test_


# tests are paramterized by four functions
@pytest.mark.parametrize("function", ["translatingBodyNoInput"
    , "translatingBodyLockFlag"
    , "translatingBodyCommandedForce"
    , "translatingBodyRhoReference"
                                      ])
def test_translatingBody(show_plots, function):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft with a single-axis translating rigid body attached to a rigid hub. The position
    of the boom axis is arbitrary. The scenario includes gravity acting on both the spacecraft and the effector.

    **Description of Variables Being Tested**

    In this file we are checking the principles of conservation of energy and angular momentum. Both the orbital and
    rotational energy and angular momentum must be maintained when conservative forces like gravity are present.
    Therefore, the values of the variables

    - ``finalOrbAngMom``
    - ``finalOrbEnergy``
    - ``finalRotAngMom``
    - ``finalRotEnergy``

    should be constant when tested against their initial values.
    """
    if function == "translatingBodyCommandedForce":
        eval(function + '(show_plots, 1.0)')
    elif function == "translatingBodyRhoReference":
        eval(function + '(show_plots, 0.5)')
    else:
        eval(function + '(show_plots)')


# rho ref and cmd force are zero, no lock flag
def translatingBodyNoInput(show_plots):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the spacecraft module
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Create a linear translating effector
    translatingBody = linearTranslationOneDOFStateEffector.linearTranslationOneDOFStateEffector()

    # Define properties of translating body
    mass = 20.0
    rhoInit = 1.0
    rhoDotInit = 0.05
    fHat_B = [[3.0 / 5.0], [4.0 / 5.0], [0.0]]
    r_FcF_F = [[-1.0], [1.0], [0.0]]
    r_F0B_B = [[-5.0], [4.0], [3.0]]
    IPntFc_F = [[50.0, 0.0, 0.0],
                [0.0, 80.0, 0.0],
                [0.0, 0.0, 60.0]]
    dcm_FB = [[0.0, -1.0, 0.0],
              [0.0, 0.0, -1.0],
              [1.0, 0.0, 0.0]]
    k = 100.0
    c = 0

    # set parameters above
    translatingBody.setMass(mass)
    translatingBody.setK(k)
    translatingBody.setC(c)
    translatingBody.setRhoInit(rhoInit)
    translatingBody.setRhoDotInit(rhoDotInit)
    translatingBody.setFHat_B(fHat_B)
    translatingBody.setR_FcF_F(r_FcF_F)
    translatingBody.setR_F0B_B(r_F0B_B)
    translatingBody.setIPntFc_F(IPntFc_F)
    translatingBody.setDCM_FB(dcm_FB)

    translatingBody.ModelTag = "translatingBody"

    # Add translating body to spacecraft
    scObject.addStateEffector(translatingBody)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBody)
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

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add states to log
    rhoData = translatingBody.translatingBodyOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, rhoData)

    # Setup and run the simulation
    stopTime = 25000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    orbEnergy = scObjectLog.totOrbEnergy
    rho = rhoData.rho
    rhoDot = rhoData.rhoDot

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
    plt.plot(timeSec, rho)
    plt.xlabel('time (s)')
    plt.ylabel('rho')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rhoDot)
    plt.xlabel('time (s)')
    plt.ylabel('rhoDot')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy, err_msg="Orbital energy is not constant.")
    np.testing.assert_allclose(finalRotEnergy, initialRotEnergy, rtol=accuracy, err_msg="Rotational energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy, err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy, err_msg="Rotational angular momentum is not constant.")


# rho ref and cmd force are zero, lock flag is enabled
def translatingBodyLockFlag(show_plots):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the spacecraft module
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Create a linear translating effector
    translatingBody = linearTranslationOneDOFStateEffector.linearTranslationOneDOFStateEffector()

    # Define properties of translating body
    mass = 20.0
    rhoInit = 1.0
    rhoDotInit = 0
    fHat_B = [[3.0 / 5.0], [4.0 / 5.0], [0.0]]
    r_FcF_F = [[-1.0], [1.0], [0.0]]
    r_F0B_B = [[-5.0], [4.0], [3.0]]
    IPntFc_F = [[50.0, 0.0, 0.0],
                [0.0, 80.0, 0.0],
                [0.0, 0.0, 60.0]]
    dcm_FB = [[0.0, -1.0, 0.0],
              [0.0, 0.0, -1.0],
              [1.0, 0.0, 0.0]]
    k = 100.0
    c = 0

    # set parameters above
    translatingBody.setMass(mass)
    translatingBody.setK(k)
    translatingBody.setC(c)
    translatingBody.setRhoInit(rhoInit)
    translatingBody.setRhoDotInit(rhoDotInit)
    translatingBody.setFHat_B(fHat_B)
    translatingBody.setR_FcF_F(r_FcF_F)
    translatingBody.setR_F0B_B(r_F0B_B)
    translatingBody.setIPntFc_F(IPntFc_F)
    translatingBody.setDCM_FB(dcm_FB)

    translatingBody.ModelTag = "translatingBody"

    # Add translating body to spacecraft
    scObject.addStateEffector(translatingBody)

    # create lock message
    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.effectorLockFlag = [1]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    translatingBody.motorLockInMsg.subscribeTo(lockMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBody)
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

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add states to log
    rhoData = translatingBody.translatingBodyOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, rhoData)

    # Setup and run the simulation
    stopTime = 25000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    orbEnergy = scObjectLog.totOrbEnergy
    rho = rhoData.rho
    rhoDot = rhoData.rhoDot

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
    plt.plot(timeSec, rho)
    plt.xlabel('time (s)')
    plt.ylabel('rho')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rhoDot)
    plt.xlabel('time (s)')
    plt.ylabel('rhoDot')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy, err_msg="Orbital energy is not constant.")
    np.testing.assert_allclose(finalRotEnergy, initialRotEnergy, rtol=accuracy, err_msg="Rotational energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy, err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy, err_msg="Rotational angular momentum is not constant.")


# cmd force is nonzero, rho ref is zero, no lock flag
def translatingBodyCommandedForce(show_plots, cmdForce):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the spacecraft module
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Create a linear translating effector
    translatingBody = linearTranslationOneDOFStateEffector.linearTranslationOneDOFStateEffector()

    # Define properties of translating body
    mass = 20.0
    rhoInit = 1.0
    rhoDotInit = 0.05
    fHat_B = [[3.0 / 5.0], [4.0 / 5.0], [0.0]]
    r_FcF_F = [[-1.0], [1.0], [0.0]]
    r_F0B_B = [[-5.0], [4.0], [3.0]]
    IPntFc_F = [[50.0, 0.0, 0.0],
                [0.0, 80.0, 0.0],
                [0.0, 0.0, 60.0]]
    dcm_FB = [[0.0, -1.0, 0.0],
              [0.0, 0.0, -1.0],
              [1.0, 0.0, 0.0]]
    k = 100.0
    c = 0

    # set parameters above
    translatingBody.setMass(mass)
    translatingBody.setK(k)
    translatingBody.setC(c)
    translatingBody.setRhoInit(rhoInit)
    translatingBody.setRhoDotInit(rhoDotInit)
    translatingBody.setFHat_B(fHat_B)
    translatingBody.setR_FcF_F(r_FcF_F)
    translatingBody.setR_F0B_B(r_F0B_B)
    translatingBody.setIPntFc_F(IPntFc_F)
    translatingBody.setDCM_FB(dcm_FB)

    translatingBody.ModelTag = "translatingBody"

    # Add translating body to spacecraft
    scObject.addStateEffector(translatingBody)

    # Create the force cmd force message
    cmdArray = messaging.ArrayMotorForceMsgPayload()
    cmdArray.motorForce = [cmdForce]  # [Nm]
    cmdMsg = messaging.ArrayMotorForceMsg().write(cmdArray)
    translatingBody.motorForceInMsg.subscribeTo(cmdMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBody)
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

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add states to log
    rhoData = translatingBody.translatingBodyOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, rhoData)

    # Setup and run the simulation
    stopTime = 25000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    orbEnergy = scObjectLog.totOrbEnergy
    rho = rhoData.rho
    rhoDot = rhoData.rhoDot

    # Set up the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]

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
    plt.plot(timeSec, rho)
    plt.xlabel('time (s)')
    plt.ylabel('rho')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rhoDot)
    plt.xlabel('time (s)')
    plt.ylabel('rhoDot')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy, err_msg="Orbital energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy, err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy, err_msg="Rotational angular momentum is not constant.")


# rho ref is nonzero, cmd force is zero and lock flag is false
def translatingBodyRhoReference(show_plots, rhoRef):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the spacecraft module
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Create a linear translating effector
    translatingBody = linearTranslationOneDOFStateEffector.linearTranslationOneDOFStateEffector()

    # Define properties of translating body
    mass = 20.0
    rhoInit = 1.0
    rhoDotInit = 0.05
    fHat_B = [[3.0 / 5.0], [4.0 / 5.0], [0.0]]
    r_FcF_F = [[-1.0], [1.0], [0.0]]
    r_F0B_B = [[-5.0], [4.0], [3.0]]
    IPntFc_F = [[50.0, 0.0, 0.0],
                [0.0, 80.0, 0.0],
                [0.0, 0.0, 60.0]]
    dcm_FB = [[0.0, -1.0, 0.0],
              [0.0, 0.0, -1.0],
              [1.0, 0.0, 0.0]]
    k = 100.0
    c = 30

    # set parameters above
    translatingBody.setMass(mass)
    translatingBody.setK(k)
    translatingBody.setC(c)
    translatingBody.setRhoInit(rhoInit)
    translatingBody.setRhoDotInit(rhoDotInit)
    translatingBody.setFHat_B(fHat_B)
    translatingBody.setR_FcF_F(r_FcF_F)
    translatingBody.setR_F0B_B(r_F0B_B)
    translatingBody.setIPntFc_F(IPntFc_F)
    translatingBody.setDCM_FB(dcm_FB)

    translatingBody.ModelTag = "translatingBody"

    # Add translating body to spacecraft
    scObject.addStateEffector(translatingBody)

    # Create the reference message
    translationRef = messaging.LinearTranslationRigidBodyMsgPayload()
    translationRef.rho = rhoRef
    translationRef.rhoDot = 0.0
    translationRefMsg = messaging.LinearTranslationRigidBodyMsg().write(translationRef)
    translatingBody.translatingBodyRefInMsg.subscribeTo(translationRefMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBody)
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

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add states to log
    rhoData = translatingBody.translatingBodyOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, rhoData)

    # Setup and run the simulation
    stopTime = 25000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    orbEnergy = scObjectLog.totOrbEnergy
    rho = rhoData.rho
    rhoDot = rhoData.rhoDot

    # Set up the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    finalAngle = rho[-1]

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
    plt.plot(timeSec, rho)
    plt.xlabel('time (s)')
    plt.ylabel('rho')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, rhoDot)
    plt.xlabel('time (s)')
    plt.ylabel('rhoDot')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12

    np.testing.assert_allclose(finalAngle, rhoRef, atol=0.01, err_msg="Angle doesn't settle to reference angle.")
    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy, err_msg="Orbital energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy, err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy, err_msg="Rotational angular momentum is not constant.")


if __name__ == "__main__":
    translatingBodyNoInput(True)
    # translatingBodyLockFlag(True)
    # translatingBodyCommandedForce(True, 1)
    # translatingBodyRhoReference(True, 0.5)
