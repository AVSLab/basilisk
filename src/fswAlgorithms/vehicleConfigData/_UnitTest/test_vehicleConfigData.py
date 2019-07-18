#
#   Unit Test Script
#   Module Name:        vehicleConfigData
#   Creation Date:      October 5, 2018
#

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import vehicleConfigData

def test_vehicleConfigData():
    [testResults, testMessage] = vehicleConfigDataTestFunction()

def vehicleConfigDataTestFunction():
    """ Test the vehicleConfigData module """

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
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate)) # Add a new task to the process

    # Construct the cssComm module
    moduleConfig = vehicleConfigData.VehConfigInputData() # Create a config struct
    # Populate the config
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    moduleConfig.ISCPntB_B = I
    initialCoM = [1, 1, 1]
    moduleConfig.CoM_B = initialCoM
    moduleConfig.outputPropsName = "outputProps"

    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig) # This calls the algContain to setup the selfInit, crossInit, and update
    moduleWrap.ModelTag = "vehicleConfigData"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Log the output message
    # unitTestSim.AddVariableForLogging('vehicleConfigData.ISCPntB_B', testProcessRate, 0, 8)
    # unitTestSim.AddVariableForLogging('vehicleConfigData.CoM_B', testProcessRate, 0, 2)
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputPropsName, testProcessRate)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    unitTestSim.ConfigureStopTime(testProcessRate)
    unitTestSim.ExecuteSimulation()

    # Get the output from this simulation
    # Ilog = unitTestSim.GetLogVariableData('vehicleConfigData.ISCPntB_B')
    # CoMLog = unitTestSim.GetLogVariableData('vehicleConfigData.CoM_B')

    Ilog = unitTestSim.pullMessageLogData(moduleConfig.outputPropsName+'.ISCPntB_B', list(range(9)))
    CoMLog = unitTestSim.pullMessageLogData(moduleConfig.outputPropsName+'.CoM_B', list(range(3)))

    accuracy = 1e-6

    # At each timestep, make sure the vehicleConfig values haven't changed from the initial values
    testFailCount, testMessages = unitTestSupport.compareArrayND([initialCoM for _ in range(len(CoMLog))], CoMLog, accuracy,
                                                                 "VehicleConfigData CoM",
                                                                 3, testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArrayND([I for _ in range(len(Ilog))], Ilog, accuracy,
                                                                 "VehicleConfigData I",
                                                                 3, testFailCount, testMessages)

    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag)

    return [testFailCount, ''.join(testMessages)]

if __name__ == '__main__':
    test_vehicleConfigData()