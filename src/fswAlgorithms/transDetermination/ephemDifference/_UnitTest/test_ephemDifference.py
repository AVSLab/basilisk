#
#   Unit Test Script
#   Module Name:        ephemDifference
#   Creation Date:      October 16, 2018
#


import os, inspect, pytest
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.fswAlgorithms import ephem_difference
from Basilisk.simulation import simFswInterfaceMessages
from Basilisk.utilities import astroFunctions


@pytest.mark.parametrize("ephBdyCount", [3, 0])

def test_ephem_difference(ephBdyCount):
    """ Test ephemDifference. """
    [testResults, testMessage] = ephemDifferenceTestFunction(ephBdyCount)
    assert testResults < 1, testMessage

def ephemDifferenceTestFunction(ephBdyCount):
    """ Test the ephemDifference module. Setup a simulation, """

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
    ephemDiffConfig = ephem_difference.EphemDifferenceData()  # Create a config struct
    ephemDiffConfig.ephBaseInMsgName = "input_eph_base_name"


    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    ephemDiffWrap = unitTestSim.setModelDataWrap(ephemDiffConfig)
    ephemDiffWrap.ModelTag = "ephemDifference"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, ephemDiffWrap, ephemDiffConfig)

    # Create the input message.
    inputEphemBase = simFswInterfaceMessages.EphemerisIntMsg() # The clock correlation message ?
    # Get the Earth's position and velocity
    position, velocity = astroFunctions.Earth_RV(astroFunctions.JulianDate([2018, 10, 16]))
    inputEphemBase.r_BdyZero_N = position
    inputEphemBase.v_BdyZero_N = velocity
    inputEphemBase.timeTag = 1234.0
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, ephemDiffConfig.ephBaseInMsgName, inputEphemBase)

    functions = [astroFunctions.Mars_RV, astroFunctions.Jupiter_RV, astroFunctions.Saturn_RV]

    changeBodyList = list()

    if ephBdyCount is 3:
        for i in range(ephBdyCount):
            # Create the change body message
            changeBodyMsg = ephem_difference.EphemChangeConfig()
            changeBodyMsg.ephInMsgName = 'input_change_body_' + str(i)
            changeBodyMsg.ephOutMsgName = 'output_change_body_' + str(i)

            changeBodyList.append(changeBodyMsg)

            # Create the input message to the change body config
            inputMsg = simFswInterfaceMessages.EphemerisIntMsg()
            position, velocity = functions[i](astroFunctions.JulianDate([2018, 10, 16]))
            inputMsg.r_BdyZero_N = position
            inputMsg.v_BdyZero_N = velocity
            inputMsg.timeTag = 321.0

            # Set this message
            unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, changeBodyMsg.ephInMsgName, inputMsg)

            # Log the output message
            unitTestSim.TotalSim.logThisMessage(changeBodyMsg.ephOutMsgName, testProcessRate)


        # add more ephemeris change objects, but have miss-configured I/O messages, or empty message
        # to trigger the end of the message counting
        changeBodyList.append(ephem_difference.EphemChangeConfig())
        changeBodyList.append(ephem_difference.EphemChangeConfig())
        changeBodyList[ephBdyCount+0].ephOutMsgName = "out1_name"  # should not count as the input name is missing
        changeBodyList[ephBdyCount+1].ephOutMsgName = "out2_name"  # output message name should not be considered

    ephemDiffConfig.changeBodies = changeBodyList

    # unitTestSim.TotalSim.logThisMessage(moduleConfig.outputNavName, testProcessRate)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # The result isn't going to change with more time. The module will continue to produce the same result
    unitTestSim.ConfigureStopTime(0)  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    if ephBdyCount is 3:
        trueRVector = [[69313607.6209608,  -75620898.04028425,   -5443274.17030424],
                       [-5.33462105e+08,  -7.56888610e+08,   1.17556184e+07],
                       [9.94135029e+07,  -1.54721593e+09,   1.65081472e+07]]

        trueVVector = [[15.04232523,  -1.13359121,   0.47668898],
                       [23.2531093,  -33.17628299,  -0.22550391],
                       [21.02793499, -25.86425597,  -0.38273815]]


        posAcc = 1e1
        velAcc = 1e-4
        unitTestSupport.writeTeXSnippet("toleranceValuePos", str(posAcc), path)
        unitTestSupport.writeTeXSnippet("toleranceValueVel", str(velAcc), path)

        for i in range(ephBdyCount):

            outputData_R = unitTestSim.pullMessageLogData('output_change_body_' + str(i) + '.r_BdyZero_N', list(range(3)))
            outputData_V = unitTestSim.pullMessageLogData('output_change_body_' + str(i) + '.v_BdyZero_N', list(range(3)))
            timeTag = unitTestSim.pullMessageLogData('output_change_body_' + str(i) + '.timeTag')

            # At each timestep, make sure the vehicleConfig values haven't changed from the initial values
            testFailCount, testMessages = unitTestSupport.compareArrayND([trueRVector[i]], outputData_R,
                                                                         posAcc,
                                                                         "ephemDifference position output body " + str(i),
                                                                         2, testFailCount, testMessages)
            testFailCount, testMessages = unitTestSupport.compareArrayND([trueVVector[i]], outputData_V,
                                                                         velAcc,
                                                                         "ephemDifference velocity output body " + str(i),
                                                                         2, testFailCount, testMessages)
            if timeTag[0, 1] != 321.0:
                testFailCount += 1
                testMessages.append("ephemDifference timeTag output body " + str(i))

    if ephemDiffConfig.ephBdyCount is not ephBdyCount:
        testFailCount += 1
        testMessages.append("input/output message count is wrong.")

    snippentName = "passFail" + str(ephBdyCount)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + ephemDiffWrap.ModelTag)
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + ephemDiffWrap.ModelTag)
        passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    return [testFailCount, ''.join(testMessages)]


if __name__ == '__main__':
    test_ephem_difference(3)
