
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib as mpl
import matplotlib.pyplot as plt
from Basilisk.simulation import spacecraft
from Basilisk.utilities import macros
from Basilisk.simulation import gravityEffector
from Basilisk.utilities import simIncludeRW
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.architecture import messaging

mpl.rc("figure", figsize=(5.75, 4))

@pytest.mark.parametrize("useFlag, testCase", [
    (False,'BalancedWheels'),
    (False,'JitterSimple'),
    (False,'JitterFullyCoupled'),
    (False, 'BOE'),
    (False, 'FrictionSpinDown'),
    (False, 'FrictionSpinUp')
])

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_reactionWheelIntegratedTest(show_plots,useFlag,testCase):
    """Module Unit Test"""
    [testResults, testMessage] = reactionWheelIntegratedTest(show_plots,useFlag,testCase)
    assert testResults < 1, testMessage

def reactionWheelIntegratedTest(show_plots,useFlag,testCase):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    stepSize = 0.0001
    if testCase == 'BOE':
        stepSize = 0.1
    if testCase == 'FrictionSpinDown' or testCase == 'FrictionSpinUp':
        stepSize = 0.01
    if testCase == 'JitterFullyCoupled':
        stepSize = 0.00001
    testProcessRate = macros.sec2nano(stepSize)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # add RW devices
    # The clearRWSetup() is critical if the script is to run multiple times
    rwFactory = simIncludeRW.rwFactory()
    varMaxMomentum = 100.            # Nms

    if testCase == 'BalancedWheels' or testCase == 'BOE' or testCase == 'FrictionSpinDown' or testCase == 'FrictionSpinUp':
        varRWModel = reactionWheelStateEffector.BalancedWheels
    elif testCase == 'JitterSimple':
        varRWModel = reactionWheelStateEffector.JitterSimple
    elif testCase == 'JitterFullyCoupled':
        varRWModel = reactionWheelStateEffector.JitterFullyCoupled

    if testCase == 'BalancedWheels' or testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        rwFactory.create(
                'Honeywell_HR16'
                ,[1,0,0]                # gsHat_B
                ,Omega = 500.           # RPM
                ,rWB_B = [0.1,0.,0.]    # m
                ,maxMomentum = varMaxMomentum
                ,RWModel= varRWModel
                )
        rwFactory.create(
                'Honeywell_HR16',
                [0,1,0]                 # gsHat_B
                ,Omega = 200.          # RPM
                ,rWB_B = [0.,0.1,0.]     # m
                ,maxMomentum = varMaxMomentum
                ,RWModel= varRWModel
                )
        rwFactory.create(
                'Honeywell_HR16'
                ,[0,0,1]                 # gsHat_B
                ,Omega = -150.           # RPM
                ,rWB_B = [0.,0.,0.1]     # m
                ,maxMomentum = varMaxMomentum
                ,RWModel= varRWModel
                )
    if testCase == 'BOE' or testCase == 'FrictionSpinDown' or testCase == 'FrictionSpinUp':
        initialWheelSpeed = 100.
        rwCopy1 = rwFactory.create(
                'Honeywell_HR16'
                ,[0,0,1]                # gsHat_B
                ,Omega = initialWheelSpeed           # RPM
                ,rWB_B = [0.0,0.,0.]    # m
                ,maxMomentum = varMaxMomentum
                ,RWModel= varRWModel
                )
        if testCase == 'FrictionSpinDown' or testCase == 'FrictionSpinUp':
            rwCopy1.fCoulomb = 0.03
            rwCopy1.fStatic = 0.06
            rwCopy1.betaStatic = 0.15
            rwCopy1.cViscous = 0.001
            rwCopy1.omegaLimitCycle = 0.001
            rwCopy1.Omega = 15.
            rwCopy1.gsHat_B = [[np.sqrt(3)/3], [np.sqrt(3)/3], [np.sqrt(3)/3]]
            rwCopy1.rWB_B = [[0.5],[-0.5],[0.5]]
            rwCopy2 = rwFactory.create(
                'Honeywell_HR16'
                ,[np.sqrt(3)/3,np.sqrt(3)/3,np.sqrt(3)/3]               # gsHat_B
                ,Omega = -initialWheelSpeed           # RPM
                ,rWB_B = [-0.5,0.5,-0.5]    # m
                ,maxMomentum = varMaxMomentum
                ,RWModel= varRWModel
                )
            rwCopy2.fCoulomb = 0.03
            rwCopy2.fStatic = 0.06
            rwCopy2.betaStatic = 0.15
            rwCopy2.cViscous = 0.001
            rwCopy2.omegaLimitCycle = 0.001
            rwCopy2.Omega = -15.
        if testCase == 'FrictionSpinUp':
            rwCopy1.Omega = 0.0
            rwCopy2.Omega = 0.0

    # increase HR16 imbalance for test
    for key, rw in rwFactory.rwList.items():
        rw.U_d *= 1e4
        rw.U_s *= 1e4

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwFactory.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)

    # set RW torque command
    cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    if testCase == 'BalancedWheels' or testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        cmdArray.motorTorque = [0.20, 0.10, -0.50] # [Nm]
    if testCase == 'BOE' or testCase == 'FrictionSpinDown':
        cmdArray.motorTorque = [0.0] # [Nm]
    if testCase == 'FrictionSpinUp':
        cmdArray.motorTorque = [0.1, -0.1]
    cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(cmdMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, rwStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    if testCase == 'BalancedWheels' or testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        unitTestSim.earthGravBody = gravityEffector.GravBodyData()
        unitTestSim.earthGravBody.planetName = "earth_planet_data"
        unitTestSim.earthGravBody.mu = 0.3986004415E+15  # meters!
        unitTestSim.earthGravBody.isCentralBody = True

        scObject.gravField.gravBodies = spacecraft.GravBodyVector([unitTestSim.earthGravBody])

    # log data
    scDataLog = scObject.scStateOutMsg.recorder()
    speedDataLog = rwStateEffector.rwSpeedOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scDataLog)
    unitTestSim.AddModelToTask(unitTaskName, speedDataLog)


    # Define initial conditions of the sim
    if testCase == 'BalancedWheels' or testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        scObject.hub.r_BcB_B = [[-0.0002], [0.0001], [0.1]]
        scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
        scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
        scObject.hub.omega_BN_BInit = [[0.08], [0.01], [0.0]]
        scObject.hub.mHub = 750.0
        scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
        scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    if testCase == 'BOE' or testCase == 'FrictionSpinDown' or testCase == 'FrictionSpinUp':
        wheelSpeedMax = 6000.0*macros.RPM
        wheelJs = varMaxMomentum/wheelSpeedMax
        scObject.hub.mHub = 5.0
        I1Hub = 2.0
        scObject.hub.IHubPntBc_B = [[2., 0.0, 0.0], [0.0, 2., 0.0], [0.0, 0.0, I1Hub + wheelJs]]
        scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
        scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.35]]
        if testCase == 'FrictionSpinDown' or testCase == 'FrictionSpinUp':
            scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
            rw0DataLog = rwStateEffector.rwOutMsgs[0].recorder()
            rw1DataLog = rwStateEffector.rwOutMsgs[1].recorder()
            unitTestSim.AddModelToTask(unitTaskName, rw0DataLog)
            unitTestSim.AddModelToTask(unitTaskName, rw1DataLog)
        scObject.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
        scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]

    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    unitTestSim.InitializeSimulation()

    stopTime = 1.0
    if testCase == 'BOE':
        stopTime = 10.0
    if testCase == 'FrictionSpinDown' or testCase == 'FrictionSpinUp':
        stopTime = 100.0
    if testCase == 'JitterFullyCoupled':
        stopTime = 0.1
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime/2))
    unitTestSim.ExecuteSimulation()

    if testCase == 'BalancedWheels' or testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        cmdArray.motorTorque = [0.0, 0.0, 0.0] # [Nm]
    if testCase == 'BOE':
        motorTorque = 0.2
        cmdArray.motorTorque = [motorTorque]
    cmdMsg.write(cmdArray)

    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)
    orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)

    posData = scDataLog.r_BN_N
    sigmaData = scDataLog.sigma_BN
    omegaData = scDataLog.omega_BN_B
    if testCase == 'BOE' or testCase == 'FrictionSpinDown' or testCase == 'FrictionSpinUp':
        wheelSpeeds = speedDataLog.wheelSpeeds[:, 0]
        if testCase == 'BOE':
            thetaOut = 4.0*np.arctan(sigmaData[:, 2])
            # Find BOE calculations
            timeBOE = np.array([2.0, 4.0, 6.0, 8.0, 10.0])
            timeTorqueOn = 5.0
            omegaBOE = np.zeros(5)
            thetaBOE = np.zeros(5)
            wheelSpeedBOE = np.zeros(5)
            for i in range(5):
                if timeBOE[i] > timeTorqueOn:
                    omegaBOE[i] = scObject.hub.omega_BN_BInit[2][0] - motorTorque/I1Hub*(timeBOE[i]-timeTorqueOn)
                    thetaBOE[i] = scObject.hub.omega_BN_BInit[2][0]*(timeBOE[i]-timeTorqueOn) - 0.5*motorTorque/I1Hub*(timeBOE[i]-timeTorqueOn)**2 + scObject.hub.omega_BN_BInit[2][0]*(timeTorqueOn)
                    wheelSpeedBOE[i] = initialWheelSpeed*macros.RPM + (I1Hub + wheelJs)*motorTorque/(I1Hub*wheelJs)*(timeBOE[i]-timeTorqueOn)
                else:
                    omegaBOE[i] = scObject.hub.omega_BN_BInit[2][0]
                    wheelSpeedBOE[i] = initialWheelSpeed*macros.RPM
                    thetaBOE[i] = scObject.hub.omega_BN_BInit[2][0]*(timeBOE[i])
        if testCase == 'FrictionSpinDown' or testCase == 'FrictionSpinUp':
            wheelSpeedBeforeInteg1 = rw0DataLog.Omega
            wheelSpeedBeforeInteg2 = rw1DataLog.Omega
            frictionTorque1 = rw0DataLog.frictionTorque
            frictionTorque2 = rw1DataLog.frictionTorque

    dataPos = posData[-1]
    dataSigma = sigmaData[-1]

    if testCase == 'BalancedWheels':
        truePos = [
                    [-4.02553766e+06,   7.48712857e+06,   5.24933964e+06]
                    ]

        trueSigma = [
                    [1.99853994e-02,   2.45647716e-03,   8.45356279e-06]
                    ]

    elif testCase == 'JitterSimple':
        truePos = [
                    [-4.02553766e+06,   7.48712857e+06,   5.24933964e+06]
                    ]

        trueSigma = [
                    [1.98964221e-02,   2.24474932e-03,  -5.66618270e-05]
                    ]

    elif testCase == 'JitterFullyCoupled':
        truePos = [
                    [-4.02085866e+06,   7.49022306e+06,   5.24840326e+06]
                    ]

        trueSigma = [
                    [1.98708924e-03,   2.26086385e-04,  -1.60335529e-05]
                    ]

    initialOrbAngMom_N = [
                [orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]
                ]

    finalOrbAngMom = [
                [orbAngMom_N[-1,1], orbAngMom_N[-1,2], orbAngMom_N[-1,3]]
                 ]

    initialRotAngMom_N = [
                [rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]
                ]

    finalRotAngMom = [
                [rotAngMom_N[-1,1], rotAngMom_N[-1,2], rotAngMom_N[-1,3]]
                 ]

    initialOrbEnergy = [
                [orbEnergy[0,1]]
                ]

    finalOrbEnergy = [
                [orbEnergy[-1,1]]
                 ]

    initialRotEnergy = [
                [rotEnergy[int(len(rotEnergy)/2)+1,1]]
                ]

    finalRotEnergy = [
                [rotEnergy[-1,1]]
                 ]

    plt.close("all")
    if testCase == 'BalancedWheels' or testCase == 'JitterFullyCoupled':
        plt.figure()
        plt.clf()
        plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
        plt.xlabel("Time (s)")
        plt.ylabel("Relative Difference")
        unitTestSupport.writeFigureLaTeX("ChangeInOrbitalAngularMomentum" + testCase, "Change in Orbital Angular Momentum " + testCase, plt, r"width=0.8\textwidth", path)
        plt.figure()
        plt.clf()
        plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
        plt.xlabel("Time (s)")
        plt.ylabel("Relative Difference")
        unitTestSupport.writeFigureLaTeX("ChangeInOrbitalEnergy" + testCase, "Change in Orbital Energy " + testCase, plt, r"width=0.8\textwidth", path)
        plt.figure()
        plt.clf()
        plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
        plt.xlabel("Time (s)")
        plt.ylabel("Relative Difference")
        unitTestSupport.writeFigureLaTeX("ChangeInRotationalAngularMomentum" + testCase, "Change in Rotational Angular Momentum " + testCase, plt, r"width=0.8\textwidth", path)
        plt.figure()
        plt.clf()
        plt.plot(rotEnergy[int(len(rotEnergy)/2)+1:,0]*1e-9, (rotEnergy[int(len(rotEnergy)/2)+1:,1] - rotEnergy[int(len(rotEnergy)/2)+1,1])/rotEnergy[int(len(rotEnergy)/2)+1,1])
        plt.xlabel("Time (s)")
        plt.ylabel("Relative Difference")
        unitTestSupport.writeFigureLaTeX("ChangeInRotationalEnergy" + testCase, "Change in Rotational Energy " + testCase, plt, r"width=0.8\textwidth", path)
        if show_plots:
            plt.show()
            plt.close('all')

    if testCase == 'BOE':
        plt.figure()
        plt.clf()
        plt.plot(scDataLog.times()*1e-9, thetaOut, label = 'Basilisk')
        plt.plot(timeBOE, thetaBOE, 'ro', label='BOE')
        plt.legend(loc='upper left', numpoints=1)
        plt.xlabel("Time (s)")
        plt.ylabel("Theta (rad)")
        unitTestSupport.writeFigureLaTeX("ReactionWheelBOETheta", "Reaction Wheel BOE Theta", plt, r"width=0.8\textwidth", path)

        plt.figure()
        plt.clf()
        plt.plot(scDataLog.times()*1e-9, omegaData[:,2], label = 'Basilisk')
        plt.plot(timeBOE, omegaBOE, 'ro', label='BOE')
        plt.legend(loc='upper right', numpoints=1)
        plt.xlabel("Time (s)")
        plt.ylabel("Body Rate (rad/s)")
        unitTestSupport.writeFigureLaTeX("ReactionWheelBOEBodyRate", "Reaction Wheel BOE Body Rate", plt, r"width=0.8\textwidth", path)

        plt.figure()
        plt.clf()
        plt.plot(scDataLog.times()*1e-9, wheelSpeeds, label = 'Basilisk')
        plt.plot(timeBOE, wheelSpeedBOE, 'ro', label='BOE')
        plt.legend(loc ='upper left', numpoints=1)
        plt.xlabel("Time (s)")
        plt.ylabel("Wheel Speed (rad/s)")
        unitTestSupport.writeFigureLaTeX("ReactionWheelBOERWRate", "Reaction Wheel BOE RW Rate", plt, r"width=0.8\textwidth", path)
        if show_plots:
            plt.show()
            plt.close('all')

    if testCase == 'FrictionSpinDown' or testCase == 'FrictionSpinUp':
        plt.figure()
        plt.clf()
        plt.plot(scDataLog.times()*1e-9, omegaData[:,2], label='Basilisk')
        plt.xlabel("Time (s)")
        plt.ylabel("Body Rate (rad/s)")
        unitTestSupport.writeFigureLaTeX("ReactionWheel" + testCase + "TestBodyRates", "Reaction Wheel " + testCase + " Test Body Rates", plt, r"width=0.8\textwidth", path)

        plt.figure()
        plt.clf()
        plt.plot(speedDataLog.times()*1e-9, wheelSpeeds, label = 'RW 1 Wheel Speed')
        plt.plot(speedDataLog.times()*1e-9, wheelSpeeds, label = 'RW 2 Wheel Speed')
        plt.legend(loc='upper right')
        plt.xlabel("Time (s)")
        plt.ylabel("Wheel Speed (rad/s)")
        unitTestSupport.writeFigureLaTeX("ReactionWheel" + testCase + "TestWheelSpeed", "Reaction Wheel " + testCase + " Test Wheel Speed", plt, r"width=0.8\textwidth", path)

        print(wheelSpeedBeforeInteg1)
        print(frictionTorque1)
        plt.figure()
        plt.clf()
        plt.plot(wheelSpeedBeforeInteg1, frictionTorque1, label = 'RW 1 Friction Torque')
        plt.plot(wheelSpeedBeforeInteg2, frictionTorque2, label = 'RW 2 Friction Torque')
        plt.legend(loc='upper right')
        plt.xlabel("Wheel Speed (rad/s)")
        plt.ylabel("Friction Torque (N-m)")
        axes = plt.gca()
        plt.xlim([-15, 15])
        unitTestSupport.writeFigureLaTeX("ReactionWheel" + testCase + "TestFrictionTorque", "Reaction Wheel " + testCase + " Test Friction Torque", plt, r"width=0.8\textwidth", path)
        if show_plots:
            plt.show()
            plt.close('all')

    accuracy = 1e-7
    if testCase == 'BalancedWheels' or testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        for i in range(0,len(truePos)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataPos,truePos[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed pos unit test")

        for i in range(0,len(trueSigma)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataSigma,trueSigma[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed attitude unit test")

    accuracy = 1e-10
    if testCase == 'BalancedWheels' or testCase == 'JitterFullyCoupled':
        for i in range(0,len(initialOrbAngMom_N)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed orbital angular momentum unit test")

        for i in range(0,len(initialRotAngMom_N)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed rotational angular momentum unit test")

        for i in range(0,len(initialOrbEnergy)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i],initialOrbEnergy[i],1,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed orbital energy unit test")

        for i in range(0,len(initialRotEnergy)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i],initialRotEnergy[i],1,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed rotational energy unit test")

    accuracy = 1e-8
    if testCase == 'BOE':
        for i in range(5):
            if abs((omegaBOE[i] - omegaData[int(timeBOE[i]/stepSize),2])/omegaBOE[i]) > accuracy:
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed BOE body rates unit test")
            if abs((thetaBOE[i] - thetaOut[int(timeBOE[i]/stepSize)])/thetaBOE[i]) > accuracy:
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed BOE theta unit test")
            if abs((wheelSpeedBOE[i] - wheelSpeeds[int(timeBOE[i]/stepSize)])/wheelSpeedBOE[i]) > accuracy:
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed BOE wheel speed unit test")

    if testFailCount == 0:
        print("PASSED: " + " Reaction Wheel Integrated Sim " + testCase)

        # print out success message if no errors were found
    if testCase == 'JitterSimple' and testFailCount == 0:
        print("PASSED ")
        colorText = 'ForestGreen'
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
        # Write some snippets for AutoTex
        snippetName = testCase + 'PassFail'
        unitTestSupport.writeTeXSnippet(snippetName, passedText, path)
    elif testCase == 'JitterSimple' and testFailCount > 0:
        colorText = 'Red'
        passedText = r'\textcolor{' + colorText + '}{' + "FAILED" + '}'
        # Write some snippets for AutoTex
        snippetName = testCase + 'PassFail'
        unitTestSupport.writeTeXSnippet(snippetName, passedText, path)

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    # reactionWheelIntegratedTest(True,True,'BalancedWheels')
    reactionWheelIntegratedTest(True,True,'BOE')
