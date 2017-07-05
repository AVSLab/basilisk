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
import pytest
import sys, os, inspect

#
# Spice Unit Test
#
# Purpose:  Test the proper function of the Spice Ephemeris module.
#           Proper function is tested by comparing Spice Ephermis to
#           JPL Horizons Database for different planets and times of year
# Author:   Thibaud Teil
# Creation Date:  Dec. 20, 2016
#

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import datetime
import unitTestSupport
import SimulationBaseClass
import spice_interface
import numpy as np
import ephemeris_converter
import macros



# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
# @pytest.mark.parametrize("", [])


# provide a unique test method name, starting with test_
def test_ephemConvert(show_plots):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitephemeris_converter(show_plots)
    assert testResults < 1, testMessage

# Run unit test
def unitephemeris_converter(show_plots):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    TotalSim = SimulationBaseClass.SimBaseClass()
    TotalSim.TotalSim.terminateSimulation()

    simulationTime = macros.min2nano(10.)
    numDataPoints = 1000 #Test fails if number of data points too small. This is because the sampling time is too large
    samplingTime = simulationTime / (numDataPoints-1)
    DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, samplingTime))

    # List of planets tested
    planets = ['sun', 'earth', 'mars barycenter']

    # Initialize the ephermis module
    EphemObject = ephemeris_converter.EphemerisConverter()
    EphemObject.ModelTag = 'EphemData'
    messageMap = {}
    for planet in planets:
        messageMap[planet + '_planet_data'] = planet + '_ephemeris_data'
    EphemObject.messageNameMap = ephemeris_converter.map_string_string(messageMap)
    TotalSim.AddModelToTask(unitTaskName, EphemObject)

    # Initialize the spice module
    SpiceObject = spice_interface.SpiceInterface()
    SpiceObject.ModelTag = "SpiceInterfaceData"
    SpiceObject.SPICEDataPath = splitPath[0] + '/External/EphemerisData/'
    SpiceObject.OutputBufferCount = 10000
    SpiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun"])
    SpiceObject.UTCCalInit = "2015 February 10, 00:00:00.0 TDB"
    TotalSim.AddModelToTask(unitTaskName, SpiceObject)

    # Configure simulation
    TotalSim.ConfigureStopTime(int(simulationTime))
    TotalSim.AddVariableForLogging('EphemData.messagesLinked')
    TotalSim.AddVariableForLogging('EphemData.numOutputBuffers')
    for planet in planets:
        TotalSim.TotalSim.logThisMessage(planet + '_planet_data', 2*samplingTime)
        TotalSim.TotalSim.logThisMessage(planet + '_ephemeris_data', 2*samplingTime)

    # Execute simulation
    TotalSim.InitializeSimulation()
    TotalSim.ExecuteSimulation()

    # Get the link confirmation
    LinkMessagesCheck = TotalSim.GetLogVariableData('EphemData.messagesLinked')
    for i in range(len(LinkMessagesCheck[:,0])):
        if LinkMessagesCheck[i,1] - 1.0 > 1E-10:
            testFailCount += 1
            testMessages.append("FAILED: Messages not linked succesfully")


    # Get the position, velocities and time for the message before and after the copy
    for planet in planets:
        for j in range(2*int(simulationTime/simulationTime+1)):
            if (np.linalg.norm(np.array(TotalSim.pullMessageLogData(planet + '_planet_data' + '.PositionVector', range(3)))[j,:] - np.array(TotalSim.pullMessageLogData(planet + '_ephemeris_data' + '.r_BdyZero_N', range(3)))[j,:]) >1E5 ):
                testFailCount += 1
                testMessages.append("FAILED: PositionVector not copied")
            if (np.linalg.norm(np.array(TotalSim.pullMessageLogData(planet + '_planet_data' + '.VelocityVector', range(3)))[j,:] - np.array(TotalSim.pullMessageLogData(planet + '_ephemeris_data' + '.v_BdyZero_N', range(3)))[j,:]) >1E5 ):
                testFailCount += 1
                testMessages.append("FAILED: VelocityVector not copied")


    # print out success message if no error were found
    if testFailCount == 0:
        print   " \n PASSED "

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_ephemConvert(False  # show_plots
                   )
