''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#
#   Unit Test Script
#   Module Name:        planetEphemeris
#   Author:             Hanspeter Schaub
#   Creation Date:      April 24, 2019
#

import pytest
import os, inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)







# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.simulation import planetEphemeris
from Basilisk.utilities import macros

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("param1", [
     (1)
])

# update "module" in this function name to reflect the module name
def test_module(show_plots, param1):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = fswModuleTestFunction(show_plots, param1)
    assert testResults < 1, testMessage


def fswModuleTestFunction(show_plots, param1):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)


    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # terminateSimulation() is needed if multiple unit test scripts are run
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = planetEphemeris.PlanetEphemeris()
    moduleConfig.ModelTag = 'planetEphemeris'

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleConfig)

    # Initialize the test module configuration data
    planetNames = ["earth", "venus"]
    moduleConfig.planetNames = planetEphemeris.StringVector(planetNames)


    # setup planet ephemeris states
    oeEarth = planetEphemeris.classicElements()
    oeEarth.a = planetEphemeris.SMA_EARTH
    oeEarth.i = 0.0*macros.D2R
    oeEarth.Omega = 0.0*macros.D2R
    oeEarth.omega = 0.0*macros.D2R
    oeEarth.f = 90.0*macros.D2R

    oeVenus = planetEphemeris.classicElements()
    oeVenus.a = planetEphemeris.SMA_VENUS
    oeVenus.i = 0.0*macros.D2R
    oeVenus.Omega = 0.0*macros.D2R
    oeVenus.omega = 0.0*macros.D2R
    oeVenus.f = 180.0*macros.D2R

    moduleConfig.planetElements = planetEphemeris.classicElementVector([oeEarth, oeVenus])

    # setup planet local sideral time at epoch
    moduleConfig.rightAscension = planetEphemeris.DoubleVector([0.*macros.D2R, 272.76*macros.D2R])

    # setup planet local sideral time at epoch
    moduleConfig.declination = planetEphemeris.DoubleVector([90.*macros.D2R, 67.16*macros.D2R])

    # setup planet local sideral time at epoch
    moduleConfig.lst0 = planetEphemeris.DoubleVector([10.*macros.D2R, 30.*macros.D2R])

    # setup planet rotation rate about polar axis
    moduleConfig.rotRate = planetEphemeris.DoubleVector([planetEphemeris.OMEGA_EARTH, planetEphemeris.OMEGA_VENUS])


    # Setup logging on the test module output message so that we get all the writes to it
    for planet in planetNames:
        name = planet + "_planet_data"
        unitTestSim.TotalSim.logThisMessage(planet + "_planet_data", testProcessRate)


    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()


    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()



    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    for planet in planetNames:
        J2000Current = unitTestSim.pullMessageLogData(planet + '_planet_data.J2000Current')
        PositionVector = unitTestSim.pullMessageLogData(planet + '_planet_data.PositionVector', range(3))
        VelocityVector = unitTestSim.pullMessageLogData(planet + '_planet_data.VelocityVector', range(3))
        J20002Pfix = unitTestSim.pullMessageLogData(planet + '_planet_data.J20002Pfix', range(9))
        J20002Pfix_dot = unitTestSim.pullMessageLogData(planet + '_planet_data.J20002Pfix_dot', range(9))
        computeOrient = unitTestSim.pullMessageLogData(planet + '_planet_data.computeOrient')
        # PlanetName = unitTestSim.pullMessageLogData(planet + '_planet_data.PlanetName')

        # print PositionVector


        # FinalPlanetMessage = planetEphemeris.SpicePlanetStateSimMsg()
        # check = unitTestSim.TotalSim.GetWriteData(planet + '_planet_data', FinalPlanetMessage.getStructSize(), FinalPlanetMessage, 0)
        # print FinalPlanetMessage.PlanetName
        # print check
        #
        #
        # check = unitTestSim.TotalSim.GetWriteData('venus_planet_data', FinalPlanetMessage.getStructSize(), FinalPlanetMessage, 0)
        # # print FinalPlanetMessage.PositionVector
        # print check

    #
    # # set the filtered output truth states
    # trueVector=[];
    # if param1==1:
    #     if param2==1:
    #         trueVector = [
    #                    [2.0, 1.0, 0.7],
    #                    [3.0, 1.0, 0.7],
    #                    [4.0, 1.0, 0.7],
    #                    [2.0, 1.0, 0.7],
    #                    [3.0, 1.0, 0.7]
    #                    ]
    #     else:
    #         if param2==3:
    #             trueVector = [
    #                    [2.0, 3.0, 0.7],
    #                    [3.0, 3.0, 0.7],
    #                    [4.0, 3.0, 0.7],
    #                    [2.0, 3.0, 0.7],
    #                    [3.0, 3.0, 0.7]
    #                    ]
    #         else:
    #             testFailCount+=1
    #             testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed with unsupported input parameters")
    # else:
    #     if param1==2:
    #         trueVector = [
    #                    [3.0, 2.0, 0.7],
    #                    [4.0, 2.0, 0.7],
    #                    [5.0, 2.0, 0.7],
    #                    [3.0, 2.0, 0.7],
    #                    [4.0, 2.0, 0.7]
    #                    ]
    #     else:
    #         testFailCount+=1
    #         testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed with unsupported input parameters")
    #
    # # compare the module results to the truth values
    # accuracy = 1e-12
    # unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)
    #
    # dummyTrue = [1.0, 2.0, 3.0, 1.0, 2.0]
    #
    # testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
    #                                                            accuracy, "Output Vector",
    #                                                            testFailCount, testMessages)
    #
    # testFailCount, testMessages = unitTestSupport.compareDoubleArray(dummyTrue, variableState,
    #                                                            accuracy, "dummy parameter",
    #                                                            testFailCount, testMessages)






    # #   print out success message if no error were found
    # snippentName = "passFail" + str(param1)
    # if testFailCount == 0:
    #     colorText = 'ForestGreen'
    #     print "PASSED: " + moduleWrap.ModelTag
    #     passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    # else:
    #     colorText = 'Red'
    #     print "Failed: " + moduleWrap.ModelTag
    #     passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    # unitTestSupport.writeTeXSnippet(snippentName, passedText, path)


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(              # update "module" in function name
                 False,
                 1            # param1 value
               )
