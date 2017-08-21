''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
import sys, os, inspect
import numpy as np
import pytest
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import matplotlib as mpl
import matplotlib.pyplot as plt
import spacecraftPlus
import macros
import gravityEffector
import simIncludeRW
import reactionWheelStateEffector

mpl.rc("figure", figsize=(5.75,4))

@pytest.mark.parametrize("useFlag, testCase", [
    (False,'BalancedWheels'),
    (False,'JitterSimple'),
    (False,'JitterFullyCoupled'),
    (False, 'BOE')
])

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_reactionWheelIntegratedTest(show_plots,useFlag,testCase):
    [testResults, testMessage] = reactionWheelIntegratedTest(show_plots,useFlag,testCase)
    assert testResults < 1, testMessage

def reactionWheelIntegratedTest(show_plots,useFlag,testCase):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    rwCommandName = "reactionwheel_cmds"

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.0001)  # update process rate update time
    if testCase == 'BOE':
        testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # add RW devices
    # The clearRWSetup() is critical if the script is to run multiple times
    rwFactory = simIncludeRW.rwFactory()
    varMaxMomentum = 100.            # Nms

    if testCase == 'BalancedWheels':
        varRWModel = 0
    elif testCase == 'JitterSimple':
        varRWModel = 1
    elif testCase == 'JitterFullyCoupled':
        varRWModel = 2
    elif testCase == 'BOE':
        varRWModel = 0

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
    if testCase == 'BOE':
        rwFactory.create(
                'Honeywell_HR16'
                ,[0,0,1]                # gsHat_B
                ,Omega = 100.           # RPM
                ,rWB_B = [0.0,0.,0.]    # m
                ,maxMomentum = varMaxMomentum
                ,RWModel= varRWModel
                )

    # increase HR16 imbalance for test
    for key, rw in rwFactory.rwList.iteritems():
        rw.U_d *= 1e4
        rw.U_s *= 1e4

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwFactory.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)

    # set RW torque command
    cmdArray = reactionWheelStateEffector.RWArrayTorqueIntMsg()
    if testCase == 'BalancedWheels' or testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        cmdArray.motorTorque = [0.20, 0.10, -0.50] # [Nm]
    if testCase == 'BOE':
        cmdArray.motorTorque = [0.0] # [Nm]
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               rwCommandName,
                               cmdArray)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, rwStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    if testCase == 'BalancedWheels' or testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        unitTestSim.earthGravBody = gravityEffector.GravBodyData()
        unitTestSim.earthGravBody.bodyInMsgName = "earth_planet_data"
        unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
        unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
        unitTestSim.earthGravBody.isCentralBody = True
        unitTestSim.earthGravBody.useSphericalHarmParams = False

        scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])

    # log data
    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)
    unitTestSim.TotalSim.logThisMessage(rwStateEffector.OutputDataString, testProcessRate)

    # Define initial conditions of the sim
    if testCase == 'BalancedWheels' or testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        scObject.hub.r_BcB_B = [[-0.0002], [0.0001], [0.1]]
        scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
        scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
        scObject.hub.omega_BN_BInit = [[0.08], [0.01], [0.0]]
        scObject.hub.mHub = 750.0
        scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
        scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    if testCase == 'BOE':
        scObject.hub.mHub = 5.0
        scObject.hub.IHubPntBc_B = [[10., 0.0, 0.0], [0.0, 10., 0.0], [0.0, 0.0, 10.]]
        scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
        scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.3]]
        scObject.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
        scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]

    unitTestSim.InitializeSimulation()

    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", testProcessRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", testProcessRate, 0, 0, 'double')

    stopTime = 1.0
    if testCase == 'BOE':
        stopTime = 10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime/2))
    unitTestSim.ExecuteSimulation()

    if testCase == 'BalancedWheels' or testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        cmdArray.motorTorque = [0.0, 0.0, 0.0] # [Nm]
    if testCase == 'BOE':
        cmdArray.motorTorque = [10.0]
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               rwCommandName,
                               cmdArray)

    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    orbAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")
    orbEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")

    posData = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    sigmaData = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.sigma_BN',range(3))
    omegaData = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.omega_BN_B',range(3))

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
                    [-4.02553767e+06,   7.48712857e+06,   5.24933964e+06]
                    ]

        trueSigma = [
                    [1.98982396e-02,   2.24454835e-03,  -5.54311143e-05]
                    ]

    initialOrbAngMom_N = [
                [orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]
                ]

    finalOrbAngMom = [
                [orbAngMom_N[-1,0], orbAngMom_N[-1,1], orbAngMom_N[-1,2], orbAngMom_N[-1,3]]
                 ]

    initialRotAngMom_N = [
                [rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]
                ]

    finalRotAngMom = [
                [rotAngMom_N[-1,0], rotAngMom_N[-1,1], rotAngMom_N[-1,2], rotAngMom_N[-1,3]]
                 ]

    initialOrbEnergy = [
                [orbEnergy[0,1]]
                ]

    finalOrbEnergy = [
                [orbEnergy[-1,0], orbEnergy[-1,1]]
                 ]

    initialRotEnergy = [
                [rotEnergy[int(len(rotEnergy)/2),1]]
                ]

    finalRotEnergy = [
                [rotEnergy[-1,0], rotEnergy[-1,1]]
                 ]


    if testCase == 'BalancedWheels' or testCase == 'JitterFullyCoupled':
        plt.figure()
        plt.clf()
        plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
        plt.xlabel("Time (s)")
        plt.ylabel("Relative Difference")
        unitTestSupport.writeFigureLaTeX("ChangeInOrbitalAngularMomentum" + testCase, "Change in Orbital Angular Momentum " + testCase, plt, "width=0.8\\textwidth", path)
        plt.figure()
        plt.clf()
        plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
        plt.xlabel("Time (s)")
        plt.ylabel("Relative Difference")
        unitTestSupport.writeFigureLaTeX("ChangeInOrbitalEnergy" + testCase, "Change in Orbital Energy " + testCase, plt, "width=0.8\\textwidth", path)
        plt.figure()
        plt.clf()
        plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
        plt.xlabel("Time (s)")
        plt.ylabel("Relative Difference")
        unitTestSupport.writeFigureLaTeX("ChangeInRotationalAngularMomentum" + testCase, "Change in Rotational Angular Momentum " + testCase, plt, "width=0.8\\textwidth", path)
        plt.figure()
        plt.clf()
        plt.plot(rotEnergy[int(len(rotEnergy)/2):,0]*1e-9, (rotEnergy[int(len(rotEnergy)/2):,1] - rotEnergy[int(len(rotEnergy)/2),1])/rotEnergy[int(len(rotEnergy)/2),1])
        plt.xlabel("Time (s)")
        plt.ylabel("Relative Difference")
        unitTestSupport.writeFigureLaTeX("ChangeInRotationalEnergy" + testCase, "Change in Rotational Energy " + testCase, plt, "width=0.8\\textwidth", path)
        plt.show(show_plots)

    if testCase == 'BOE':
        plt.figure()
        plt.clf()
        plt.plot(omegaData[:,0]*1e-9, omegaData[:,3])
        plt.xlabel("Time (s)")
        plt.ylabel("Relative Difference")
        unitTestSupport.writeFigureLaTeX("ReactionWheelBOE", "Reaction Wheel BOE", plt, "width=0.8\\textwidth", path)
        plt.show(show_plots)

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

    if testFailCount == 0:
        print "PASSED: " + " Reaction Wheel Integrated Sim " + testCase

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    reactionWheelIntegratedTest(True,False,'BOE')