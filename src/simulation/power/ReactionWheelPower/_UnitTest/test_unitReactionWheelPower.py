#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Unit Test Script
#   Module Name:        PowerRW
#   Author:             Hanspeter Schaub
#   Creation Date:      January 22, 2020
#

import inspect
import os

import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import ReactionWheelPower
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.architecture import bskLogging


@pytest.mark.parametrize("accuracy", [1e-12])
@pytest.mark.parametrize("setRwMsg", [True, False])
@pytest.mark.parametrize("setEta_e2m", [True, False])
@pytest.mark.parametrize("setEta_m2c", [0, 1, 2])
@pytest.mark.parametrize("OmegaValue", [100.0, -100.0])
@pytest.mark.parametrize("setDeviceStatusMsg", [0, 1, 2])


# update "module" in this function name to reflect the module name
def test_module(show_plots, setRwMsg, setDeviceStatusMsg, setEta_e2m, OmegaValue, setEta_m2c, accuracy):
    """
    **Validation Test Description**

    This unit test checks the output of the RW power module.  Only the power module is created, and all
    required input messages are created from python.  The tests consider both default behavior and
    manually setting behavior.

    **Test Parameters**

    The test parameters are described below.
    All possible permutations of these cases are tests.

    :param show_plots: flag if plots should be shown.  Not used in this script.
    :param setRwMsg: [bool] flag if the RW state message should be set.  If not then a warning is created and the output
                     message should be 0
    :param setDeviceStatusMsg: [int] flag to check if a device is on or off.  If this msg is not set the device
                               should default to being on. The options include:

                               - 0,  use default behavior,
                               - 1,  set msg and turn device off,
                               - 2,  set msg and turn device on

    :param setEta_e2m: [bool] to specify a conversion efficiency from electrical to mechanical power
    :param OmegaValue: [RPM] specifies the RW wheel speed to either positive or negative values
    :param setEta_m2c: [int] flag to set the mechanical to electrical conversion efficiency when breaking the RW.
                             The cases include:

                             - 0, default case of -1 turning of energy recovery,
                             - 1, set the efficiency to zero,
                             - 2, set the efficiency to 0.5

    :param accuracy: [float] accuracy used when compute actual to truth power requirement
    :return: void

    **Description of Variables Being Tested**

    In each case the module output power value is check against a python evaluated truth value.

    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = powerRW(show_plots, setRwMsg, setDeviceStatusMsg, setEta_e2m, OmegaValue, setEta_m2c, accuracy)
    assert testResults < 1, testMessage\


def powerRW(show_plots, setRwMsg, setDeviceStatusMsg, setEta_e2m, OmegaValue, setEta_m2c, accuracy):
    if not setRwMsg:
        bskLogging.setDefaultLogLevel(bskLogging.BSK_ERROR)

    """Module Unit Test"""
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # create the rw power test module
    testModule = ReactionWheelPower.ReactionWheelPower()
    testModule.ModelTag = "bskSat"
    testModule.basePowerNeed = 10.   # baseline power draw, Watts
    rwMsg = messaging.RWConfigLogMsg()
    testModule.rwStateInMsg.subscribeTo(rwMsg)

    if setEta_e2m:
        testModule.elecToMechEfficiency = 0.9
        eta_e2m = testModule.elecToMechEfficiency
    else:
        eta_e2m = 1.0
    if setEta_m2c:
        testModule.mechToElecEfficiency = (setEta_m2c - 1.0)/2.0
        eta_m2e = testModule.mechToElecEfficiency
    else:
        eta_m2e = -1

    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # set the RW status input message
    OmegaValue = OmegaValue * macros.RPM        # convert to rad/sec
    if setRwMsg:
        rwStatusMsg = messaging.RWConfigLogMsgPayload()
        rwStatusMsg.Omega = OmegaValue          # rad/sec
        rwStatusMsg.u_current = 0.010           # Nm
        rwMsg.write(rwStatusMsg)

    # set device status message
    if setDeviceStatusMsg > 0:
        deviceStatusMsg = messaging.DeviceStatusMsgPayload()
        deviceStatusMsg.deviceStatus = setDeviceStatusMsg - 1
        statusMsg = messaging.DeviceStatusMsg().write(deviceStatusMsg)
        testModule.nodeStatusInMsg.subscribeTo(statusMsg)

    dataLog = testModule.nodePowerOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # pull logged data
    drawData = dataLog.netPower
    print(drawData)

    # compare the module results to the truth values
    if setRwMsg and setDeviceStatusMsg != 1:
        wheelPower = OmegaValue * rwStatusMsg.u_current
        truePower = testModule.basePowerNeed
        if wheelPower > 0.0 or eta_m2e < 0.0:
            truePower += abs(wheelPower)/eta_e2m
        else:
            print(eta_m2e)
            truePower += eta_m2e * wheelPower
        truePower *= -1.0
    else:
        truePower = 0.0

    print([truePower]*3)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        [truePower]*3, drawData, accuracy, "powerRW",
        testFailCount, testMessages)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + testModule.ModelTag)
    else:
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]



#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    powerRW(
        False,      # showplots, not used in this test
        False,      # setRwMsg
        0,          # setDeviceStatusMsg (0 - don't set msg, 1 - set msg to OFF, 2 - set msg to ON
        True,      # setEta_e2m
        100.0,      # OmegaValue
        0,       # m2cCase (0 - use default (off), 1 - use 0.0, 2 - use 0.5
        1e-12       # accuracy
    )
