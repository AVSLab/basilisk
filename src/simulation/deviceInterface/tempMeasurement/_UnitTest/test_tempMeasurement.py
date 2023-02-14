# 
#  ISC License
# 
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder
# 
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
# 
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
# 
# 

#
# Thermal Sensor Faults Test
#
# Purpose:  Test the proper function of the faulty behaviors of the tempMeasurement module.
#           Results are compared to simple truth values.
#           To test spiking, percent of spike values compared with expected.
# Creation Date:  Feb. 9, 2023
#

import os

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import sensorThermal, tempMeasurement
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros, astroFunctions
from Basilisk.utilities import unitTestSupport

path = os.path.dirname(os.path.abspath(__file__))

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
# Need RNGSeed 464374481
@pytest.mark.parametrize(
    "tempFault",
    [
        "TEMP_FAULT_NOMINAL",    
        "TEMP_FAULT_STUCK_CURRENT",                           
        "TEMP_FAULT_STUCK_VALUE", 
        "TEMP_FAULT_SPIKING",     
        "BIASED"          
    ])
# provide a unique test method name, starting with test_
def test_sensorThermalFault(tempFault):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(tempFault)
    assert testResults < 1, testMessage
    __tracebackhide__ = True


def run(tempFault):

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Create simulation variable names
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.sec2nano(100.)

    #
    #  create the simulation process
    #

    testProcessRate = macros.sec2nano(1.0)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    #
    #   set up the simulation tasks/objects
    #

    #  set device status message
    sensorStatusMsgPayload = messaging.DeviceStatusMsgPayload()
    sensorStatusMsgPayload.deviceStatus = 1
    sensorStatusMsg = messaging.DeviceStatusMsg().write(sensorStatusMsgPayload)

    #  set the spacecraft message
    scStateMsgPayload = messaging.SCStatesMsgPayload()
    scStateMsgPayload.r_BN_N = [6378*1000., 0., 0.]
    scStateMsgPayload.sigma_BN = [0., 0., 0.]
    scStateMsg = messaging.SCStatesMsg().write(scStateMsgPayload)

    #  set the sun message
    sunMsgPayload = messaging.SpicePlanetStateMsgPayload()
    sunMsgPayload.PositionVector = [astroFunctions.AU*1000., 0., 0.]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunMsgPayload)


    #
    #   set up the truth value temperature modeling
    #
    sensorThermalModel = sensorThermal.SensorThermal()
    sensorThermalModel.ModelTag = 'sensorThermalModel'
    sensorThermalModel.nHat_B = [0, 0, 1]
    sensorThermalModel.sensorArea = 1.0  # m^2
    sensorThermalModel.sensorAbsorptivity = 0.25
    sensorThermalModel.sensorEmissivity = 0.34
    sensorThermalModel.sensorMass = 2.0  # kg
    sensorThermalModel.sensorSpecificHeat = 890
    sensorThermalModel.sensorPowerDraw = 30.0  # W
    sensorThermalModel.T_0 = 0  # [ºC]
    sensorThermalModel.sunInMsg.subscribeTo(sunMsg)
    sensorThermalModel.stateInMsg.subscribeTo(scStateMsg)
    sensorThermalModel.sensorStatusInMsg.subscribeTo(sensorStatusMsg)
    unitTestSim.AddModelToTask(unitTaskName, sensorThermalModel)

    #
    # set up the tempMeasurement module
    #
    tempMeasurementModel = tempMeasurement.TempMeasurement()
    tempMeasurementModel.tempInMsg.subscribeTo(sensorThermalModel.temperatureOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, tempMeasurementModel)

    #
    # log data
    #

    # log the RW temperature
    tempLog = tempMeasurementModel.tempOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, tempLog)

    #
    #   initialize Simulation
    #
    # Truth Values
    if tempFault == "TEMP_FAULT_NOMINAL":
        truthValue = -4.251289338192501
        unitTestSim.InitializeSimulation()
        unitTestSim.ConfigureStopTime(simulationTime)
        unitTestSim.ExecuteSimulation()

    elif tempFault == "TEMP_FAULT_STUCK_CURRENT":
        truthValue = -2.1722164230619447
        unitTestSim.InitializeSimulation()
        unitTestSim.ConfigureStopTime(round(simulationTime/2.0))
        unitTestSim.ExecuteSimulation()
        tempMeasurementModel.faultState = tempMeasurement.TEMP_FAULT_STUCK_CURRENT
        unitTestSim.ConfigureStopTime(simulationTime)
        unitTestSim.ExecuteSimulation()

    elif tempFault == "TEMP_FAULT_STUCK_VALUE":
        truthValue = 10.0
        tempMeasurementModel.stuckValue = 10.0
        unitTestSim.InitializeSimulation()
        unitTestSim.ConfigureStopTime(round(simulationTime/2.0))
        unitTestSim.ExecuteSimulation()
        tempMeasurementModel.faultState = tempMeasurement.TEMP_FAULT_STUCK_VALUE
        unitTestSim.ConfigureStopTime(simulationTime)
        unitTestSim.ExecuteSimulation()
    elif tempFault == "TEMP_FAULT_SPIKING":
        tempMeasurementModel.faultState = tempMeasurement.TEMP_FAULT_SPIKING
        truthValue = 0.1
        unitTestSim.InitializeSimulation()
        unitTestSim.ConfigureStopTime(simulationTime)
        unitTestSim.ExecuteSimulation()
        nominals = np.array([ 0.,         -0.04439869, -0.08875756, -0.13307667, 
            -0.17735608, -0.22159584, -0.265796,   -0.30995662, -0.35407775, 
            -0.39815946, -0.44220178, -0.48620479, -0.53016852, -0.57409304, 
            -0.6179784,  -0.66182465, -0.70563184, -0.74940004, -0.79312929, 
            -0.83681965, -0.88047117, -0.9240839,  -0.9676579,  -1.01119321, 
            -1.0546899,  -1.09814802, -1.14156761, -1.18494873, -1.22829144, 
            -1.27159578, -1.31486181, -1.35808958, -1.40127914, -1.44443055, 
            -1.48754385, -1.5306191, -1.57365634, -1.61665564, -1.65961704, 
            -1.7025406,  -1.74542636, -1.78827437, -1.83108469, -1.87385737, 
            -1.91659246, -1.95929001, -2.00195007, -2.04457269, -2.08715792, 
            -2.12970582, -2.17221642, -2.21468979, -2.25712597, -2.29952502, 
            -2.34188697, -2.38421189, -2.42649982, -2.46875082, -2.51096492, 
            -2.55314219, -2.59528266, -2.6373864,  -2.67945344, -2.72148384, 
            -2.76347765, -2.80543491, -2.84735568, -2.88924,    -2.93108792, 
            -2.9728995 , -3.01467477, -3.05641378, -3.0981166,  -3.13978325, 
            -3.1814138,  -3.22300829, -3.26456677, -3.30608928, -3.34757587, 
            -3.38902659, -3.43044149, -3.47182062, -3.51316402, -3.55447174, 
            -3.59574383, -3.63698033, -3.67818129, -3.71934677, -3.76047679, 
            -3.80157142, -3.8426307,  -3.88365467, -3.92464338, -3.96559689, 
            -4.00651522, -4.04739844, -4.08824658, -4.1290597,  -4.16983783, 
            -4.21058103, -4.25128934])

        testValue = 0
        sensorTemp = np.array(tempLog.temperature)
        for ii in range(len(nominals)):
            if abs(nominals[ii]) > 0.0001:
                if abs(nominals[ii] - sensorTemp[ii]) > 1e-6:
                    testValue = testValue + 1
            else:
                if abs(sensorTemp[ii]) > 1e-6:
                    testValue = testValue + 1
        testValue = testValue/len(nominals)
    elif tempFault == "BIASED":
        biasVal = 1.0
        tempMeasurementModel.senBias = biasVal
        truthValue = -3.251289338192501
        unitTestSim.InitializeSimulation()
        unitTestSim.ConfigureStopTime(simulationTime)
        unitTestSim.ExecuteSimulation()
    else:
        NotImplementedError("Fault type specified does not exist.")


    sensorTemp = np.array(tempLog.temperature)
    print(sensorTemp)



    #
    # compare the module results to the true values
    #

    if tempFault == "TEMP_FAULT_SPIKING":
        if not unitTestSupport.isDoubleEqualRelative(testValue, truthValue, 5E-1): # only 101 values so need this to be relatively relaxed (within 50%)
            testFailCount+= 1
    elif not unitTestSupport.isDoubleEqualRelative(sensorTemp[-1], truthValue, 1E-12):
        testFailCount += 1

    return [testFailCount, ''.join(testMessages)]
    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found


if __name__ == "__main__":
    run("TEMP_FAULT_NOMINAL")