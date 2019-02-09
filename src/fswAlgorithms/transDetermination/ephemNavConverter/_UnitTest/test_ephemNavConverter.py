#
#   Unit Test Script
#   Module Name:        ephemNavConverter
#   Creation Date:      October 16, 2018
#

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.fswAlgorithms import ephem_nav_converter

from Basilisk.simulation import simFswInterfaceMessages
from Basilisk.utilities import astroFunctions


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
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))  # Add a new task to the process

    # Construct the ephemNavConverter module
    # Set the names for the input messages
    moduleConfig = ephem_nav_converter.EphemNavConverterData()  # Create a config struct
    moduleConfig.ephInMsgName = "input_eph_name"
    moduleConfig.stateOutMsgName = "output_state_name"
    moduleConfig.outputState = simFswInterfaceMessages.NavTransIntMsg()

    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "ephemNavConverter"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Create the input message.
    inputEphem = ephem_nav_converter.EphemerisIntMsg() # The clock correlation message ?
    # Get the Earth's position and velocity
    position, velocity = astroFunctions.Earth_RV(astroFunctions.JulianDate([2018, 10, 16]))
    inputEphem.r_BdyZero_N = position
    inputEphem.v_BdyZero_N = velocity
    inputEphem.timeTag = 1.0 # sec
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.ephInMsgName, inputEphem)

    unitTestSim.AddVariableForLogging('ephemNavConverter.outputState.r_BN_N', testProcessRate, 0, 2)
    unitTestSim.AddVariableForLogging('ephemNavConverter.outputState.v_BN_N', testProcessRate, 0, 2)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # The result isn't going to change with more time. The module will continue to produce the same result
    unitTestSim.ConfigureStopTime(testProcessRate)  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    posAcc = 1e1
    velAcc = 1e-4

    outputR = unitTestSim.GetLogVariableData('ephemNavConverter.outputState.r_BN_N')
    outputV = unitTestSim.GetLogVariableData('ephemNavConverter.outputState.v_BN_N')
    # print(outputR)
    # print(outputV)

    trueR = [[1.37456815e+08,   5.78427691e+07,  -2.99637783e+03],
            [ 1.37456815e+08,   5.78427691e+07,  -2.99637783e+03]]
    trueV = [[-1.20387801e+01,   2.73449588e+01,  -1.11931087e-03],
            [ -1.20387801e+01,   2.73449588e+01,  -1.11931087e-03]]

    # At each timestep, make sure the vehicleConfig values haven't changed from the initial values
    testFailCount, testMessages = unitTestSupport.compareArrayND(trueR, outputR,
                                                                 posAcc,
                                                                 "ephemNavConverter output Position",
                                                                 2, testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArrayND(trueV, outputV,
                                                                 velAcc,
                                                                 "ephemNavConverter output Velocity",
                                                                 2, testFailCount, testMessages)

    if testFailCount == 0:
        print("Passed")

    return [testFailCount, ''.join(testMessages)]

if __name__ == '__main__':
    test_ephem_nav_converter()
