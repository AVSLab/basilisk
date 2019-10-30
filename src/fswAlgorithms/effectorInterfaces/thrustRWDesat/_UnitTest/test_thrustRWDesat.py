#
#   Unit Test Script
#   Module Name:        thrustRWDesat
#   Creation Date:      October 5, 2018
#

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros, fswSetupThrusters
from Basilisk.fswAlgorithms.thrustRWDesat import thrustRWDesat
from Basilisk.fswAlgorithms.fswMessages import fswMessages
from Basilisk.simulation.simFswInterfaceMessages import simFswInterfaceMessages


def test_thrustRWDesat():
    """ Test thrustRWDesat. """
    [testResults, testMessage] = thrustRWDesatTestFunction()
    assert testResults < 1, testMessage

def thrustRWDesatTestFunction():
    """ Test the thrustRWDesat module. Setup a simulation, """

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

    # Construct the thrustRWDesat module
    # Set the names for the input messages
    moduleConfig = thrustRWDesat.thrustRWDesatConfig()  # Create a config struct
    moduleConfig.inputSpeedName = "input_rw_speed"
    moduleConfig.inputRWConfigData = "input_rw_constellation"
    moduleConfig.inputThrConfigName = "input_thr_array_config"
    moduleConfig.inputMassPropsName = "input_vehicle_config_mass"
    moduleConfig.outputThrName = "output_thr_array_cmd"

    # Set the necessary data in the module. NOTE: This information is more or less random
    moduleConfig.thrFiringPeriod = .5 # The amount of time to rest between thruster firings [s]
    moduleConfig.DMThresh = 20 # The point at which to stop decrementing momentum [r/s]
    moduleConfig.currDMDir = [1, 0, 0] # The current direction of momentum reduction
    moduleConfig.maxFiring = 5 # Maximum time to fire a jet for [s]

    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "thrustRWDesat"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    numRW = 3

    inputRWConstellationMsg = fswMessages.RWConstellationFswMsg()
    inputRWConstellationMsg.numRW = numRW

    # Initialize the msg that gives the speed of the reaction wheels
    inputSpeedMsg = simFswInterfaceMessages.RWSpeedIntMsg()

    gsHat = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    # Iterate over all of the reaction wheels, create a rwConfigElementFswMsg, and add them to the rwConstellationFswMsg
    rwConfigElementList = list()
    for rw in range(numRW):
        rwConfigElementMsg = fswMessages.RWConfigElementFswMsg()
        rwConfigElementMsg.gsHat_B = gsHat[rw]  # Spin axis unit vector of the wheel in structure # [1, 0, 0]
        rwConfigElementMsg.Js = 0.08  # Spin axis inertia of wheel [kgm2]
        rwConfigElementMsg.uMax = 0.2  # maximum RW motor torque [Nm]

        # Add this to the list
        rwConfigElementList.append(rwConfigElementMsg)

    inputSpeedMsg.wheelSpeeds = [20, 10, 30] # The current angular velocities of the RW wheel

    # Set the array of the reaction wheels in RWConstellationFswMsg to the list created above
    inputRWConstellationMsg.reactionWheels = rwConfigElementList

    # Initialize the msg that gives the mass properties. This just needs the center of mass value
    inputVehicleMsg = fswMessages.VehicleConfigFswMsg()
    inputVehicleMsg.CoM_B = [0, 0, 0] # This is random.

    # setup thruster cluster message
    fswSetupThrusters.clearSetup()
    rcsLocationData = [
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070]
        ]
    rcsDirectionData = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0]
        ]

    for i in range(len(rcsLocationData)):
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], 2.0)
    fswSetupThrusters.writeConfigMessage(moduleConfig.inputThrConfigName,
                                         unitTestSim.TotalSim,
                                         unitProcessName)
    numThrusters = fswSetupThrusters.getNumOfDevices()

    # Set these messages
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.inputSpeedName, inputSpeedMsg)
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.inputRWConfigData, inputRWConstellationMsg)
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.inputMassPropsName, inputVehicleMsg)
    # unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.inputThrConfigName, inputThrArray)

    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputThrName, testProcessRate)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 9*process rate so 10 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # This doesn't work if only 1 number is passed in as the second argument, but we don't need the second
    outputThrData = unitTestSim.pullMessageLogData(moduleConfig.outputThrName+'.OnTimeRequest', list(range(numThrusters)))

    print(outputThrData)

    # This is just what is outputted...
    trueVector = [[0.00000000e+00,   1.97181559e+00,   0.00000000e+00, 0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
                 [ 0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 0.00000000e+00,   1.28062495e+00,   0.00000000e+00],
                 [ 0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 0.00000000e+00,   1.28062495e+00,   0.00000000e+00],
                 [ 0.00000000e+00,   1.97181559e+00,   0.00000000e+00, 0.00000000e+00,   0.00000000e+00,   0.00000000e+00]]

    accuracy = 1e-6

    # At each timestep, make sure the vehicleConfig values haven't changed from the initial values
    testFailCount, testMessages = unitTestSupport.compareArrayND(trueVector, outputThrData,
                                                                 accuracy,
                                                                 "ThrustRWDesat output",
                                                                 2, testFailCount, testMessages)

    if testFailCount == 0:
        print("Passed")

    return [testFailCount, ''.join(testMessages)]

if __name__ == '__main__':
    test_thrustRWDesat()
