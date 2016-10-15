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
    
    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    
    unitTestSim.InitializeSimulation()
    
    scObject.dynManager.setPropertyValue("m_SC", [[100.0]])
    scObject.dynManager.setPropertyValue("centerOfMassSC", [[1.0], [0.0], [0.0]])
    scObject.dynManager.setPropertyValue("inertiaSC", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    scObject.dynManager.setPropertyValue("inertiaPrimeSC", [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    scObject.dynManager.setPropertyValue("centerOfMassPrimeSC", [[0.0], [0.0], [0.0]])


    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")
    
    posRef.setState([[1.0], [0.0], [0.0]])
    omegaRef.setState([[0.001], [0.0], [0.0]])
    sigmaRef.setState([[0.0], [0.0], [0.0]])
    velRef.setState([[0.01], [0.0], [0.0]])

    scObject.hub.mHub = [[1.0]]
    scObject.hub.rBcB_B = [[1.0], [0.0], [0.0]]
    scObject.hub.IHubPntB_B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    
    unitTestSim.ConfigureStopTime(macros.sec2nano(100.0))
    unitTestSim.ExecuteSimulation()

    print velRef.getState()
    print posRef.getState()
    print omegaRef.getState()
    print sigmaRef.getState()

    if testFailCount == 0:
        print "PASSED: " + " Hub Propagate"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    spacecraftPlusAllTest(False)
