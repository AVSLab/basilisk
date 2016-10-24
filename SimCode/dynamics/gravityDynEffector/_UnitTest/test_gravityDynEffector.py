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
import gravityDynEffector
import spice_interface
import sim_model
import ctypes
import pyswice

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def gravityEffectorAllTest(show_plots):
    [testResults, testMessage] = test_sphericalHarmonics(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_singleGravityBody(show_plots)
    assert testResults < 1, testMessage

def test_sphericalHarmonics(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    spherHarm = gravityDynEffector.SphericalHarmonics()
    
    testHarm = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    
    spherHarm.cBar = gravityDynEffector.MultiArray(testHarm)
    
    vecCheckSuccess = True
    for i in range(len(spherHarm.cBar)):
        for j in range(len(spherHarm.cBar[i])):
            if spherHarm.cBar[i][j] != testHarm[i][j]:
                vecCheckSuccess = False

    
    if(vecCheckSuccess != True):
        testFailCount += 1
        testMessages.append("2D vector not input appropriately to spherical harmonics")

    gravityDynEffector.loadGravFromFile(path + '/GGM03S.txt', spherHarm, 20)
    spherHarm.initializeParameters()
    gravOut = spherHarm.computeField([[0.0], [0.0], [(6378.1363)*1.0E3]], 20, True)
    gravMag = numpy.linalg.norm(numpy.array(gravOut))
    
    if gravMag > 9.9 or gravMag < 9.7:
        testFailCount += 1
        testMessages.append("Gravity magnitude not within allowable tolerance")

    gravOutMax = spherHarm.computeField([[0.0], [0.0], [(6378.1363)*1.0E3]], 100, True)

    if gravOutMax != gravOut:
        testFailCount += 1
        testMessages.append("Gravity ceiling not enforced correctly")

    if testFailCount == 0:
        print "PASSED: " + " Spherical Harmonics"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def test_singleGravityBody(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True
    
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    DateSpice = "2015 February 10, 00:00:00.0 TDB"

    # Create a sim module as an empty container
    TotalSim = SimulationBaseClass.SimBaseClass()
    TotalSim.TotalSim.terminateSimulation()

    DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, macros.sec2nano(0.1)))

    # Initialize the modules that we are using.
    SpiceObject = spice_interface.SpiceInterface()

    SpiceObject.ModelTag = "SpiceInterfaceData"
    SpiceObject.SPICEDataPath = splitPath[0] + '/External/EphemerisData/'
    SpiceObject.OutputBufferCount = 10000
    SpiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun"])
    SpiceObject.UTCCalInit = DateSpice
    TotalSim.AddModelToTask(unitTaskName, SpiceObject)
    TotalSim.InitializeSimulation()

    earthGravBody = gravityDynEffector.GravBodyData()
    earthGravBody.bodyMsgName = "earth_planet_data"
    earthGravBody.outputMsgName = "earth_display_frame_data"
    earthGravBody.isCentralBody = False
    earthGravBody.useSphericalHarmParams = True
    gravityDynEffector.loadGravFromFile(path + '/GGM03S.txt', earthGravBody.spherHarm, 100)
    
    earthGravBody.initBody(0)

    if testFailCount == 0:
        print "PASSED: " + " Single body"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    gravityEffectorAllTest(False)
    
