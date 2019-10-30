#
#   Unit Test Script
#   Module Name:        ephemNavConverter
#   Creation Date:      October 16, 2018
#

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.fswAlgorithms.ephem_nav_converter import ephem_nav_converter
from Basilisk.utilities import astroFunctions

import os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

def test_ephem_nav_converter():
    """ Test ephemNavConverter. """
    [testResults, testMessage] = ephemNavConverterTestFunction()
    assert testResults < 1, testMessage

def ephemNavConverterTestFunction():
    """ Test the ephemNavConverter module. Setup a simulation """

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # This is needed if multiple unit test scripts are run
    # This create a fresh and consistent simulation environment for each test run

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))  # Add a new task to the process

    # Construct the ephemNavConverter module
    # Set the names for the input messages
    ephemNavConfig = ephem_nav_converter.EphemNavConverterData()  # Create a config struct
    ephemNavConfig.ephInMsgName = "input_eph_name"
    ephemNavConfig.stateOutMsgName = "output_state_name"
    # ephemNavConfig.outputState = simFswInterfaceMessages.NavTransIntMsg()

    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    ephemNavWrap = unitTestSim.setModelDataWrap(ephemNavConfig)
    ephemNavWrap.ModelTag = "ephemNavConverter"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, ephemNavWrap, ephemNavConfig)

    # Create the input message.
    inputEphem = ephem_nav_converter.EphemerisIntMsg()

    # Get the Earth's position and velocity
    position, velocity = astroFunctions.Earth_RV(astroFunctions.JulianDate([2018, 10, 16]))
    inputEphem.r_BdyZero_N = position
    inputEphem.v_BdyZero_N = velocity
    inputEphem.timeTag = 1.0  # sec
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, ephemNavConfig.ephInMsgName, inputEphem)

    unitTestSim.TotalSim.logThisMessage(ephemNavConfig.stateOutMsgName)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # The result isn't going to change with more time. The module will continue to produce the same result
    unitTestSim.ConfigureStopTime(testProcessRate)  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    posAcc = 1e1
    velAcc = 1e-4
    unitTestSupport.writeTeXSnippet("toleranceValuePos", str(posAcc), path)
    unitTestSupport.writeTeXSnippet("toleranceValueVel", str(velAcc), path)

    outputR = unitTestSim.pullMessageLogData(ephemNavConfig.stateOutMsgName + '.r_BN_N',  list(range(3)))
    outputV = unitTestSim.pullMessageLogData(ephemNavConfig.stateOutMsgName + '.v_BN_N',  list(range(3)))
    outputTime = unitTestSim.pullMessageLogData(ephemNavConfig.stateOutMsgName + '.timeTag')

    trueR = [position, position]
    trueV = [velocity, velocity]
    trueTime = [inputEphem.timeTag, inputEphem.timeTag]

    # At each timestep, make sure the vehicleConfig values haven't changed from the initial values
    testFailCount, testMessages = unitTestSupport.compareArrayND(trueR, outputR,
                                                                 posAcc,
                                                                 "ephemNavConverter output Position",
                                                                 2, testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArrayND(trueV, outputV,
                                                                 velAcc,
                                                                 "ephemNavConverter output Velocity",
                                                                 2, testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(trueTime, outputTime,
                                                                 velAcc,
                                                                 "ephemNavConverter output Time",
                                                                 testFailCount, testMessages)

    #   print out success message if no error were found
    snippentName = "passFail"
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + ephemNavWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + ephemNavWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)


    return [testFailCount, ''.join(testMessages)]


if __name__ == '__main__':
    test_ephem_nav_converter()
