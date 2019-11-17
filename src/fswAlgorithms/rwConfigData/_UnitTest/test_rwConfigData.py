#
#   Unit Test Script
#   Module Name:        rwConfigData
#   Creation Date:      October 5, 2018
#

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms.rwConfigData import rwConfigData
from Basilisk.fswAlgorithms.fswMessages import fswMessages

import numpy as np

def test_rwConfigData():
    """Module Unit Test"""
    [testResults, testMessage] = rwConfigDataTestFunction()
    assert testResults < 1, testMessage

def rwConfigDataTestFunction():
    """ Test the rwConfigData module """

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
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate)) # Add a new task to the process

    # Construct the cssComm module
    moduleConfig = rwConfigData.rwConfigData_Config() # Create a config struct
    # Populate the config
    moduleConfig.rwParamsOutMsgName = "rwConfigParamsOutMsg"
    moduleConfig.vehConfigInMsgName = "vehConfigInMsg"
    moduleConfig.rwConstellationInMsgName = "rwConstellationInMsg"

    # Create the messages
    vehConfigFswMsg = fswMessages.VehicleConfigFswMsg()

    rwConstellationFswMsg = fswMessages.RWConstellationFswMsg()
    numRW = 3
    rwConstellationFswMsg.numRW = 3
    gsHat_initial = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    js_initial = np.array([0.08, 0.09, 0.07])
    uMax_initial = np.array([0.2, 0.1, 0.3])

    # Iterate over all of the reaction wheels, create a rwConfigElementFswMsg, and add them to the rwConstellationFswMsg
    rwConfigElementList = list()
    for rw in range(numRW):
        rwConfigElementMsg = fswMessages.RWConfigElementFswMsg()
        rwConfigElementMsg.gsHat_B = gsHat_initial[rw]  # Spin axis unit vector of the wheel in structure # [1, 0, 0]
        rwConfigElementMsg.Js = js_initial[rw]  # Spin axis inertia of wheel [kgm2]
        rwConfigElementMsg.uMax = uMax_initial[rw]  # maximum RW motor torque [Nm]

        # Add this to the list
        rwConfigElementList.append(rwConfigElementMsg)

    # Set the array of the reaction wheels in RWConstellationFswMsg to the list created above
    rwConstellationFswMsg.reactionWheels = rwConfigElementList

    # Set these messages
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.vehConfigInMsgName, vehConfigFswMsg)
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.rwConstellationInMsgName,
                               rwConstellationFswMsg)

    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig) # This calls the algContain to setup the selfInit, crossInit, and update
    moduleWrap.ModelTag = "rwConfigData"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Log the output message
    unitTestSim.TotalSim.logThisMessage(moduleConfig.rwParamsOutMsgName, testProcessRate)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    unitTestSim.ConfigureStopTime(testProcessRate)
    unitTestSim.ExecuteSimulation()

    # Get the output from this simulation
    JsListLog = unitTestSim.pullMessageLogData(moduleConfig.rwParamsOutMsgName+'.JsList', list(range(numRW)))
    uMaxLog = unitTestSim.pullMessageLogData(moduleConfig.rwParamsOutMsgName+'.uMax', list(range(numRW)))
    GsMatrix_B_Log = unitTestSim.pullMessageLogData(moduleConfig.rwParamsOutMsgName+'.GsMatrix_B', list(range(3*numRW)))

    accuracy = 1e-6

    # At each timestep, make sure the vehicleConfig values haven't changed from the initial values
    testFailCount, testMessages = unitTestSupport.compareArrayND([js_initial], JsListLog, accuracy,
                                                                 "rwConfigData JsList",
                                                                 3, testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArrayND([uMax_initial], uMaxLog, accuracy,
                                                                 "rwConfigData uMax",
                                                                 3, testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArrayND([gsHat_initial.flatten()], GsMatrix_B_Log, accuracy,
                                                                 "rwConfigData GsMatrix_B",
                                                                 3*numRW, testFailCount, testMessages)

    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag)

    return [testFailCount, ''.join(testMessages)]

if __name__ == '__main__':
    test_rwConfigData()
