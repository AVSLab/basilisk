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

    DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, macros.sec2nano(0.1)))

    # Initialize the spice modules that we are using.
    EphemObject = ephemeris_converter.EphemerisConverter()
    EphemObject.ModelTag = 'EphemData'
    TotalSim.AddModelToTask(unitTaskName, EphemObject)

    # Configure simulation
    TotalSim.ConfigureStopTime(int(60.0 * 1E9))
    TotalSim.AddVariableForLogging('EphemData.messagesLinked')
    TotalSim.AddVariableForLogging('EphemData.messageNameMap')

    # Execute simulation
    TotalSim.InitializeSimulation()
    TotalSim.ExecuteSimulation()

    # Get the logged variables (GPS seconds, Julian Date)
    LinkMessagesCheck = TotalSim.GetLogVariableData('EphemData.messagesLinked')
    # print np.array(LinkMessagesCheck)
    # NameMapCheck = TotalSim.GetLogVariableData('EphemData.messageNameMap')
    # print NameMapCheck


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
