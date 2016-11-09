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
import hingedRigidBodyStateEffector
import sim_model
import macros
import ctypes

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def spacecraftPlusAllTest(show_plots):
    [testResults, testMessage] = test_hubPropagate(show_plots)
    assert testResults < 1, testMessage

def test_hubPropagate(show_plots):
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

    unitTestSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unitTestSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.mass = 100.0
    unitTestSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel1.d = 1.5
    unitTestSim.panel1.k = 100.0
    unitTestSim.panel1.c = 0.0
    unitTestSim.panel1.rHB_B = [[0.5], [0.0], [1.0]]
    unitTestSim.panel1.dcmHB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel1.nameOfThetaState = "hingedRigidBodyTheta1"
    unitTestSim.panel1.nameOfThetaDotState = "hingedRigidBodyThetaDot1"

    # Define Variables for panel 2
    unitTestSim.panel2.mass = 100.0
    unitTestSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel2.d = 1.5
    unitTestSim.panel2.k = 100.0
    unitTestSim.panel2.c = 0.0
    unitTestSim.panel2.rHB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.panel2.dcmHB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel2.nameOfThetaState = "hingedRigidBodyTheta2"
    unitTestSim.panel2.nameOfThetaDotState = "hingedRigidBodyThetaDot2"

    # Add panels to spaceCraft
    # this next line is not working
    scObject.addStateEffector(unitTestSim.panel1)
    scObject.addStateEffector(unitTestSim.panel2)
    
    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    
    unitTestSim.InitializeSimulation()

    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")
    theta1Ref = scObject.dynManager.getStateObject("hingedRigidBodyTheta1")
    thetaDot1Ref = scObject.dynManager.getStateObject("hingedRigidBodyThetaDot1")
    theta2Ref = scObject.dynManager.getStateObject("hingedRigidBodyTheta2")
    thetaDot2Ref = scObject.dynManager.getStateObject("hingedRigidBodyThetaDot2")

    posRef.setState([[0.0], [0.0], [0.0]])
    velRef.setState([[0.0], [0.0], [0.0]])
    sigmaRef.setState([[0.0], [0.0], [0.0]])
    omegaRef.setState([[0.1], [-0.1], [0.1]])
    theta1Ref.setState([[5*numpy.pi/180.0]])
    thetaDot1Ref.setState([[0.0]])
    theta2Ref.setState([[0.0]])
    thetaDot2Ref.setState([[0.0]])

    scObject.hub.mHub = 750
    scObject.hub.rBcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    stopTime = 60.0*10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    dataPos = posRef.getState()
    dataPos = [[stopTime, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    dataSigma =  sigmaRef.getState()
    dataSigma = [[stopTime, dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]]

    truePos = [
                [-13.3842058040891, -13.2968874812058, 0.155873769585104]
                ]
    trueSigma = [
                  [0.228198675962671, -0.329880460528557,  0.266599868549938]
                  ]

    accuracy = 1e-8
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Dynamics Mode failed pos unit test at t=" + str(dataPos[i,0]*macros.NANO2SEC) + "sec\n")

    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Dynamics Mode failed attitude unit test at t=" + str(dataSigma[i,0]*macros.NANO2SEC) + "sec\n")

    if testFailCount == 0:
        print "PASSED: " + " Hub Propagate"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    spacecraftPlusAllTest(False)
