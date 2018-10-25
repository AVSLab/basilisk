#
#   Unit Test Script
#   Module Name:        cssComm
#   Creation Date:      October 4, 2018
#

import pytest
import matplotlib.pyplot as plt
import numpy as np
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import cssComm
from Basilisk.simulation import simFswInterfaceMessages, simMessages
from Basilisk.simulation import coarse_sun_sensor, spacecraftPlus
from Basilisk.utilities import orbitalMotion as om

# Note: for pytest, show_plots must be the first argument
@pytest.mark.parametrize("simCase", [0,1])
def test_cssComm(show_plots, simCase):
    if simCase == 0:
        [testResults, testMessage] = cssCommTestFunction()
    elif simCase == 1:
        [testResults, testMessage] = cssCommIntegratedTestFunction(show_plots)
    assert testResults < 1, testMessage

def cssCommTestFunction():
    """ Test the cssComm module """
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
    moduleConfig = cssComm.CSSConfigData() # Create a config struct
    # Populate the config
    moduleConfig.NumSensors = 8
    moduleConfig.MaxSensorValue = 500e-6
    moduleConfig.OutputDataName = "css_data_aggregate"
    ChebyList =  [-1.734963346951471e+06, 3.294117146099591e+06,
                     -2.816333294617512e+06, 2.163709942144332e+06,
                     -1.488025993860025e+06, 9.107359382775769e+05,
                     -4.919712500291216e+05, 2.318436583511218e+05,
                     -9.376105045529010e+04, 3.177536873430168e+04,
                     -8.704033370738143e+03, 1.816188108176300e+03,
                     -2.581556805090373e+02, 1.888418924282780e+01]
    moduleConfig.ChebyCount = len(ChebyList)
    moduleConfig.KellyCheby = ChebyList
    moduleConfig.SensorListName = "css_sensors_data_pass"

    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig) # This calls the algContain to setup the selfInit, crossInit, and update
    moduleWrap.ModelTag = "cssComm"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # The cssComm module reads in from the sensor list, so create that message here
    cssArrayMsg = simFswInterfaceMessages.CSSArraySensorIntMsg()
    # NOTE: This is nonsense. These are random numbers selected because they are below the MaxSensorValue
    # TODO: Make this a realistic input
    cssArrayMsg.CosValue = [100e-6, 200e-6, 100e-6, 300e-6, 400e-6, 300e-6, 200e-6, 100e-6]
    # Write this message
    msgSize = cssArrayMsg.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.SensorListName,
                                          msgSize,
                                          2)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.SensorListName,
                                          msgSize,
                                          0,
                                          cssArrayMsg)

    # Log the output message
    unitTestSim.TotalSim.logThisMessage(moduleConfig.OutputDataName, testProcessRate)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    unitTestSim.ConfigureStopTime(testProcessRate)
    unitTestSim.ExecuteSimulation()

    # Get the output from this simulation
    outputData = unitTestSim.pullMessageLogData(moduleConfig.OutputDataName+".CosValue", range(moduleConfig.NumSensors))

    # Create the true array
    trueCss = [
        [0.317205386467, 0.45791653042, 0.317205386467, 0.615444781018, 0.802325127473, 0.615444781018, 0.45791653042, 0.317205386467],
        [0.317205386467, 0.45791653042, 0.317205386467, 0.615444781018, 0.802325127473, 0.615444781018, 0.45791653042, 0.317205386467]
    ]

    accuracy = 1e-6

    testFailCount, testMessages = unitTestSupport.compareArrayND(trueCss, outputData, accuracy, "ccsComm",
                                                                 moduleConfig.NumSensors, testFailCount, testMessages)

    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag

    return [testFailCount, ''.join(testMessages)]

def cssCommIntegratedTestFunction(showPlots):
    """ This uses the coars_sun_sensor module to generate input for the cssComm module"""

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # This is needed if multiple unit test scripts are run
    # This create a fresh and consistent simulation environment for each test run
    unitTestSim.TotalSim.terminateSimulation()

    simulationTime = macros.sec2nano(120.)

    # Create test thread
    testProcessRate = macros.sec2nano(1.)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))  # Add a new task to the process

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    #
    # set initial spacecraft states
    #
    scObject.hub.r_CN_NInit = [[-om.AU * 1000.0], [0.0], [0.0]]  # m   - r_CN_N
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [1. * macros.D2R]]  # rad/s - omega_BN_B

    # add spacecraftPlus object to the simulation process
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Construct the cssComm module
    moduleConfig = cssComm.CSSConfigData()  # Create a config struct
    # Populate the config
    moduleConfig.NumSensors = 2 # Only two CSS are going to be created for this simulation
    moduleConfig.MaxSensorValue = 500e-6
    moduleConfig.OutputDataName = "css_data_aggregate"
    ChebyList = [-1.734963346951471e+06, 3.294117146099591e+06,
                 -2.816333294617512e+06, 2.163709942144332e+06,
                 -1.488025993860025e+06, 9.107359382775769e+05,
                 -4.919712500291216e+05, 2.318436583511218e+05,
                 -9.376105045529010e+04, 3.177536873430168e+04,
                 -8.704033370738143e+03, 1.816188108176300e+03,
                 -2.581556805090373e+02, 1.888418924282780e+01]
    moduleConfig.ChebyCount = len(ChebyList)
    moduleConfig.KellyCheby = ChebyList
    moduleConfig.SensorListName = "css_sensors_data_pass"

    moduleWrap = unitTestSim.setModelDataWrap(
        moduleConfig)  # This calls the algContain to setup the selfInit, crossInit, and update
    moduleWrap.ModelTag = "cssComm"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Create the CSS sensors. These don't have all of the options seen in scenarioCSS.py enabled
    CSS1 = coarse_sun_sensor.CoarseSunSensor()
    CSS1.ModelTag = "CSS1_sensor"
    CSS1.fov = 80. * macros.D2R
    CSS1.scaleFactor = 2.0
    CSS1.cssDataOutMsgName = "CSS1_output"
    CSS1.sunInMsgName = "sun_message"
    CSS1.nHat_B = np.array([1.0, 0.0, 0.0])

    CSS2 = coarse_sun_sensor.CoarseSunSensor(CSS1)      # make copy of first CSS unit
    CSS2.ModelTag = "CSS2_sensor"
    CSS2.cssDataOutMsgName = "CSS2_output"
    CSS2.nHat_B = np.array([0.0, -1.0, 0.0])

    # Create the cssArray
    cssList = [CSS1, CSS2]
    cssArray = coarse_sun_sensor.CSSConstellation()
    cssArray.ModelTag = "css_array"
    cssArray.sensorList = coarse_sun_sensor.CSSVector(cssList)
    cssArray.outputConstellationMessage = "css_sensors_data_pass"
    unitTestSim.AddModelToTask(unitTaskName, cssArray)

    #
    # create simulation messages
    #
    sunPositionMsg = simMessages.SpicePlanetStateSimMsg()
    sunPositionMsg.PositionVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               CSS1.sunInMsgName,
                               sunPositionMsg)

    # Log the output message
    unitTestSim.TotalSim.logThisMessage(moduleConfig.OutputDataName, testProcessRate)
    unitTestSim.TotalSim.logThisMessage(cssArray.outputConstellationMessage, testProcessRate)

    # Initialize the simulation
    unitTestSim.InitializeSimulationAndDiscover()

    unitTestSim.ConfigureStopTime(simulationTime)
    unitTestSim.ExecuteSimulation()

    # Get the output from this simulation
    outputData = unitTestSim.pullMessageLogData(moduleConfig.OutputDataName + ".CosValue", range(moduleConfig.NumSensors))
    constellationData = unitTestSim.pullMessageLogData(cssArray.outputConstellationMessage+".CosValue", range(len(cssList)))

    # Plot the results
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(1, len(cssList) + 1):
        plt.plot(outputData[:, 0] * macros.NANO2SEC, outputData[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='CSS$_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('CSS Signals ')

    plt.figure(2)
    for idx in range(1, len(cssList) + 1):
        plt.plot(constellationData[:, 0] * macros.NANO2SEC, constellationData[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='CSS$_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('CSS Signals ')

    if showPlots:
        plt.show()

    numTruthPoints = 5
    skipValue = int(simulationTime * macros.NANO2SEC / numTruthPoints)
    accuracy = 1e-6

    cssCommTestData = outputData[::skipValue]
    # print(cssCommTestData)
    trueCSS = [
        [0.00000000e+00, 3.10700765e-02]
        , [0.00000000e+00, 0.00000000e+00]
        , [0.00000000e+00, 0.00000000e+00]
        , [0.00000000e+00, 0.00000000e+00]
        , [3.10700765e-02, 0.00000000e+00]
        , [3.10700765e-02, 0.00000000e+00]
    ]

    testFailCount, testMessages = unitTestSupport.compareArrayND(trueCSS, cssCommTestData, accuracy, "CSSarray", 2,
        testFailCount, testMessages)

    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag

    return [testFailCount, ''.join(testMessages)]

if __name__ == '__main__':
    test_cssComm(simCase=1, show_plots=True)