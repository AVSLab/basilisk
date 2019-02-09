#
#   Unit Test Script
#   Module Name:        rwNullSpace
#   Creation Date:      October 5, 2018
#

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.fswAlgorithms import rwNullSpace, fswMessages
from Basilisk.simulation import simFswInterfaceMessages
import itertools


def test_rwNullSpace():
    """ Test rwNullSpace. """
    [testResults, testMessage] = rwNullSpaceTestFunction()
    assert testResults < 1, testMessage

def rwNullSpaceTestFunction():
    """ Test the rwNullSpace module. Setup a simulation, """

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

    # Construct the rwNullSpace module
    # Set the names for the input messages
    moduleConfig = rwNullSpace.rwNullSpaceConfig()  # Create a config struct
    moduleConfig.inputRWSpeeds = "input_rw_speeds"
    moduleConfig.inputRWConfigData = "input_rw_constellation"
    moduleConfig.inputRWCommands = "input_rw_commands"
    moduleConfig.outputControlName = "output_rw_cmd"

    # Set the necessary data in the module. NOTE: This information is more or less random
    moduleConfig.OmegaGain = .5 # The gain factor applied to the RW speeds

    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "rwNullSpace"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    numRW = 3

    inputRWConstellationMsg = fswMessages.RWConstellationFswMsg()
    inputRWConstellationMsg.numRW = numRW

    # Initialize the msg that gives the speed of the reaction wheels
    inputSpeedMsg = rwNullSpace.RWSpeedIntMsg()

    gsHat = [[1, 0, 0], [0,1,0], [0, 0, 1]]

    # Iterate over all of the reaction wheels, create a rwConfigElementFswMsg, and add them to the rwConstellationFswMsg
    rwConfigElementList = list()
    for rw in range(numRW):
        rwConfigElementMsg = fswMessages.RWConfigElementFswMsg()
        rwConfigElementMsg.gsHat_B = gsHat[rw] # Spin axis unit vector of the wheel in structure # [1, 0, 0]
        rwConfigElementMsg.Js = 0.08 # Spin axis inertia of wheel [kgm2]
        rwConfigElementMsg.uMax = 0.2 # maximum RW motor torque [Nm]

        # Add this to the list
        rwConfigElementList.append(rwConfigElementMsg)

    inputSpeedMsg.wheelSpeeds = [10, 20, 30] # The current angular velocities of the RW wheel

    # Set the array of the reaction wheels in RWConstellationFswMsg to the list created above
    inputRWConstellationMsg.reactionWheels = rwConfigElementList

    inputRWCmdMsg = simFswInterfaceMessages.RWArrayTorqueIntMsg()
    inputRWCmdMsg.motorTorque = [0.1, 0.2, 0.15] # [Nm] RW motor torque array

    # Set these messages
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.inputRWSpeeds, inputSpeedMsg)
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.inputRWConfigData, inputRWConstellationMsg)
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.inputRWCommands, inputRWCmdMsg)

    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputControlName, testProcessRate)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    outputCrtlData = unitTestSim.pullMessageLogData(moduleConfig.outputControlName+'.motorTorque', range(3))

    trueVector = [[ 1.00000000e-01,   2.00000000e-01,  1.50000000e-01],
                 [  1.00000000e-01,   2.00000000e-01,  1.50000000e-01],
                 [  1.00000000e-01,   2.00000000e-01,  1.50000000e-01],
                 [  1.00000000e-01,   2.00000000e-01,  1.50000000e-01],
                 [  1.00000000e-01,   2.00000000e-01,  1.50000000e-01]]


    accuracy = 1e-6

    # At each timestep, make sure the vehicleConfig values haven't changed from the initial values
    testFailCount, testMessages = unitTestSupport.compareArrayND(trueVector, outputCrtlData,
                                                                 accuracy,
                                                                 "ThrustRWDesat output",
                                                                 2, testFailCount, testMessages)

    if testFailCount == 0:
        print("Passed")

    return [testFailCount, ''.join(testMessages)]

if __name__ == '__main__':
    test_rwNullSpace()
