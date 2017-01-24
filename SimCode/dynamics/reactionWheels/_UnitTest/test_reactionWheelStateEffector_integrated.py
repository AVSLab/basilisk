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
import numpy
import pytest
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
import spacecraftPlus
import sim_model
import macros
import gravityEffector
import spice_interface
import simIncludeRW
import reactionWheelStateEffector
import vehicleConfigData


@pytest.mark.parametrize("testCase", [
    ('BalancedWheels'),
    ('JitterFullyCoupled')
])

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def reactionWheelIntegratedTest(show_plots,testCase):
    [testResults, testMessage] = test_reactionWheelIntegratedTest(show_plots,testCase)
    assert testResults < 1, testMessage

def test_reactionWheelIntegratedTest(show_plots,testCase):
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
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # add RW devices
    # The clearRWSetup() is critical if the script is to run multiple times
    simIncludeRW.clearSetup()
    simIncludeRW.options.maxMomentum = 100
    if testCase == 'JitterFullyCoupled':
        simIncludeRW.options.U_d = 0
        simIncludeRW.options.U_s = 0
        simIncludeRW.options.RWModel = 2
    simIncludeRW.create(
            'Honeywell_HR16',
            [1,0,0],                # gsHat_S
            0.0                     # RPM
            )
    simIncludeRW.create(
            'Honeywell_HR16',
            [0,1,0],                # gsHat_S
            0.0                     # RPM
            )
    simIncludeRW.create(
            'Honeywell_HR16',
            [0,0,1],                # gsHat_S
            0.0,                    # RPM
            [0.5,0.5,0.5]           # r_S (optional)
            )


    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    simIncludeRW.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)

    # set RW torque command
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName, rwCommandName, 8*vehicleConfigData.MAX_EFF_CNT, 2)
    cmdArray = sim_model.new_doubleArray(vehicleConfigData.MAX_EFF_CNT)
    sim_model.doubleArray_setitem(cmdArray, 0, 0*0.020) # RW-1 [Nm]
    sim_model.doubleArray_setitem(cmdArray, 1, 0*0.010) # RW-2 [Nm]
    sim_model.doubleArray_setitem(cmdArray, 2,0*-0.050) # RW-3 [Nm]
    unitTestSim.TotalSim.WriteMessageData(rwCommandName, 8*vehicleConfigData.MAX_EFF_CNT, 1, cmdArray )
    
    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, rwStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    
    # unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    # unitTestSim.earthGravBody.bodyInMsgName = "earth_planet_data"
    # unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
    # unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
    # unitTestSim.earthGravBody.isCentralBody = True
    # unitTestSim.earthGravBody.useSphericalHarmParams = False

    # earthEphemData = spice_interface.SpicePlanetState()
    # earthEphemData.J2000Current = 0.0
    # earthEphemData.PositionVector = [0.0, 0.0, 0.0]
    # earthEphemData.VelocityVector = [0.0, 0.0, 0.0]
    # earthEphemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    # earthEphemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    # earthEphemData.PlanetName = "earth"

    # scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])

    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)

    # msgSize = earthEphemData.getStructSize()
    # unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
    #     unitTestSim.earthGravBody.bodyInMsgName, msgSize, 2)
    # unitTestSim.TotalSim.WriteMessageData(unitTestSim.earthGravBody.bodyInMsgName, msgSize, 0, earthEphemData)

    unitTestSim.InitializeSimulation()

    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", testProcessRate, 0, 2, 'double')

    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")

    posRef.setState([[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]])
    velRef.setState([[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]])
    sigmaRef.setState([[0.1], [0.2], [-0.3]])
    omegaRef.setState([[0.001], [-0.01], [0.03]])

    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    stopTime = 2.5
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    orbAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")

    dataPos = posRef.getState()
    dataSigma = sigmaRef.getState()
    dataPos = [[stopTime, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    dataSigma = [[stopTime, dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]]


    if testCase == 'BalancedWheels':
        truePos = [
                    [-4033333.1061706794, 7481965.68525689, 5250896.597134046]
                    ]

        trueSigma = [
                    [0.10278985463139288, 0.18785123312448332, -0.2813107133882423]
                    ]
    elif testCase == 'JitterFullyCoupled':
        truePos = [
                    [1,1,1]
                    ]

        trueSigma = [
                    [1,1,1]
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

    plt.figure(1)
    plt.plot(orbAngMom_N[:,0]*1e-9, orbAngMom_N[:,1] - orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, orbAngMom_N[:,2] - orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, orbAngMom_N[:,3] - orbAngMom_N[0,3])
    plt.title("Change in Orbital Angular Momentum")
    plt.figure(2)
    plt.plot(rotAngMom_N[:,0]*1e-9, rotAngMom_N[:,1] - rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, rotAngMom_N[:,2] - rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, rotAngMom_N[:,3] - rotAngMom_N[0,3])
    plt.title("Change in Rotational Angular Momentum")
    if show_plots == True:
        plt.show()

    accuracy = 1e-8
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Reaction Wheel Integrated Test failed pos unit test")

    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Reaction Wheel Integrated Test failed attitude unit test")

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

    if testFailCount == 0:
        print "PASSED: " + " Reaction Wheel Integrated Sim Test"

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    test_reactionWheelIntegratedTest(True,'JitterFullyCoupled')
    # test_reactionWheelIntegratedTest(True,'BalancedWheels')