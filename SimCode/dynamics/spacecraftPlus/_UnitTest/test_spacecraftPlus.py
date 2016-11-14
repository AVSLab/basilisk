'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
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
import macros
import spacecraftPlus
import sim_model
import macros
import ctypes
import gravityEffector
import spice_interface
import ExtForceTorque

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def spacecraftPlusAllTest(show_plots):
    [testResults, testMessage] = test_gravityIntegratedSim(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_extForceBodyAndTorque(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_extForceInertialAndTorque(show_plots)
    assert testResults < 1, testMessage

def test_gravityIntegratedSim(show_plots):
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
    
    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()
    
    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))
    
    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    
    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.bodyMsgName = "earth_planet_data"
    unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
    unitTestSim.earthGravBody.isCentralBody = True
    unitTestSim.earthGravBody.useSphericalHarmParams = False

    earthEphemData = spice_interface.SpicePlanetState()
    earthEphemData.J2000Current = 0.0
    earthEphemData.PositionVector = [0.0, 0.0, 0.0]
    earthEphemData.VelocityVector = [0.0, 0.0, 0.0]
    earthEphemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    earthEphemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    earthEphemData.PlanetName = "earth"
    
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])

    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)     

    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
        unitTestSim.earthGravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 2)
    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.WriteMessageData(unitTestSim.earthGravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 0, earthEphemData)

    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")

    posRef.setState([[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]])
    omegaRef.setState([[0.001], [-0.002], [0.003]])
    sigmaRef.setState([[0.0], [0.0], [0.0]])
    velRef.setState([[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]])
    
    scObject.hub.mHub = 100
    scObject.hub.rBcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    stopTime = 60.0*10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    dataPos = posRef.getState()
    dataPos = [[stopTime, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]

    truePos = [
                [-6.78159911e+06,   4.94686541e+06,   5.48674159e+06]
                ]

    moduleOutput = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_N',
                                                  range(3))
    print moduleOutput[-1, :]

    accuracy = 1e-8
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Dynamics Mode failed pos unit test at t=" + str(dataPos[i,0]*macros.NANO2SEC) + "sec\n")

    if testFailCount == 0:
        print "PASSED: " + " Gravity Integrated Sim Test"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def test_extForceBodyAndTorque(show_plots):
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

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.bodyMsgName = "earth_planet_data"
    unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
    unitTestSim.earthGravBody.isCentralBody = True
    unitTestSim.earthGravBody.useSphericalHarmParams = False

    earthEphemData = spice_interface.SpicePlanetState()
    earthEphemData.J2000Current = 0.0
    earthEphemData.PositionVector = [0.0, 0.0, 0.0]
    earthEphemData.VelocityVector = [0.0, 0.0, 0.0]
    earthEphemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    earthEphemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    earthEphemData.PlanetName = "earth"

    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])

    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
        unitTestSim.earthGravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 2)
    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.WriteMessageData(unitTestSim.earthGravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 0, earthEphemData)

    extFTObject = ExtForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [-1, 1, -1]
    extFTObject.extForce_B = [1,2,3]
    VehDynObject.addBodyEffector(extFTObject)
    scSim.AddModelToTask(unitTaskName, extFTObject)

    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")

    posRef.setState([[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]])
    omegaRef.setState([[0.001], [-0.002], [0.003]])
    sigmaRef.setState([[0.0], [0.0], [0.0]])
    velRef.setState([[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]])

    scObject.hub.mHub = 100
    scObject.hub.rBcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    stopTime = 60.0*10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    dataPos = posRef.getState()
    dataSigma = sigmaRef.getState()
    dataPos = [[stopTime, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    dataSigma = [[stopTime, dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]]

    truePos = [
                [-6.78136423e+06, 4.94628599e+06, 5.48655395e+06]
                ]

    trueSigma = [
                [4.91025978e-01, -4.21586707e-01,  3.61459503e-01]
                ]

    accuracy = 1e-8
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: External Body Force and Torque failed pos unit test at t=" + str(dataPos[i,0]*macros.NANO2SEC) + "sec\n")

    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: External Body Force and Torque failed attitude unit test at t=" + str(dataSigma[i,0]*macros.NANO2SEC) + "sec\n")

    if testFailCount == 0:
        print "PASSED: " + " External Body Force and Torque Inegrated Sim Test"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def test_extForceInertialAndTorque1(show_plots):
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

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.bodyMsgName = "earth_planet_data"
    unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
    unitTestSim.earthGravBody.isCentralBody = True
    unitTestSim.earthGravBody.useSphericalHarmParams = False

    earthEphemData = spice_interface.SpicePlanetState()
    earthEphemData.J2000Current = 0.0
    earthEphemData.PositionVector = [0.0, 0.0, 0.0]
    earthEphemData.VelocityVector = [0.0, 0.0, 0.0]
    earthEphemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    earthEphemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    earthEphemData.PlanetName = "earth"

    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])

    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
        unitTestSim.earthGravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 2)
    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.WriteMessageData(unitTestSim.earthGravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 0, earthEphemData)

    extFTObject = ExtForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [-1, 1, -1]
    extFTObject.extForce_N = [-1, -0.5, 0.5]
    VehDynObject.addBodyEffector(extFTObject)
    scSim.AddModelToTask(unitTaskName, extFTObject)

    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")

    posRef.setState([[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]])
    omegaRef.setState([[0.001], [-0.002], [0.003]])
    sigmaRef.setState([[0.0], [0.0], [0.0]])
    velRef.setState([[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]])

    scObject.hub.mHub = 100
    scObject.hub.rBcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    stopTime = 60.0*10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    dataPos = posRef.getState()
    dataSigma = sigmaRef.getState()
    dataPos = [[stopTime, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    dataSigma = [[stopTime, dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]]

    truePos = [
                [-6.78183900e+06, 4.94674963e+06, 5.48686274e+06]
                ]

    trueSigma = [
                [4.91025978e-01, -4.21586707e-01,  3.61459503e-01]
                ]

    accuracy = 1e-8
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: External Inertial Force and Torque failed pos unit test at t=" + str(dataPos[i,0]*macros.NANO2SEC) + "sec\n")

    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: External Inertial Force and Torque failed attitude unit test at t=" + str(dataSigma[i,0]*macros.NANO2SEC) + "sec\n")

    if testFailCount == 0:
        print "PASSED: " + " External Inertial Force and Torque Inegrated Sim Test"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    spacecraftPlusAllTest(False)
