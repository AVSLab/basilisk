#
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#
import inspect
import itertools
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.fswAlgorithms import dvExecuteGuidance            # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging


# parameters
dvMagnitude = [4.3, 5.0, 10.0]
minTime = [0.0, 4.0]
maxTime = [0.0, 3.0]

paramArray = [dvMagnitude, minTime, maxTime]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))

@pytest.mark.parametrize("p1_dv, p2_tmin, p3_tmax", paramList)

def test_dvExecuteGuidance(show_plots, p1_dv, p2_tmin, p3_tmax):
    r"""
    **Validation Test Description**

    This test checks if the dv burn module works correctly for different Delta-V magnitudes, minimum times and
    maximum times.

    **Test Parameters**

    Args:
        :param show_plots: flag if plots should be shown
        :param p1_dv: Delta-V magnitude
        :param p2_tmin: minimum time
        :param p3_tmax: maximum time

    **Description of Variables Being Tested**

    The content of the THRArrayOnTimeCmdMsg and DvExecutionDataMsg output messages is compared with the true values.
    """
    dvExecuteGuidanceTestFunction(show_plots, p1_dv, p2_tmin, p3_tmax)

def dvExecuteGuidanceTestFunction(show_plots, p1_dv, p2_tmin, p3_tmax):
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    updateRate = 0.5
    testProcessRate = macros.sec2nano(updateRate)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = dvExecuteGuidance.dvExecuteGuidance()
    module.ModelTag = "dvExecuteGuidance"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    module.defaultControlPeriod = updateRate
    module.minTime = p2_tmin
    module.maxTime = p3_tmax

    # thruster information
    numThrusters = 6
    acceleration_N = np.array([0.0, 0.0, 2.0])  # acceleration of spacecraft due to thrusters

    # Configure input messages
    navTransMsgData = messaging.NavTransMsgPayload()
    navTransMsgData.vehAccumDV = np.array([0.0, 0.0, 0.0])
    navTransMsg = messaging.NavTransMsg().write(navTransMsgData)

    dvBurnCmdMsgData = messaging.DvBurnCmdMsgPayload()
    dvBurnCmdMsgData.dvInrtlCmd = np.array([0.0, 0.0, p1_dv])
    dvBurnCmdMsg = messaging.DvBurnCmdMsg().write(dvBurnCmdMsgData)

    # Create thruster on time message and add the module as author. This allows us to write an initial message that does
    # not come from the module
    onTimeCmdMsg = messaging.THRArrayOnTimeCmdMsg_C()
    onTimeCmdMsgData = messaging.THRArrayOnTimeCmdMsgPayload()
    # set on time to some non-zero values to simulate that DV burn is executed. Needs to be stopped and zeroed by module
    defaultOnTime = np.ones(numThrusters)
    onTimeCmdMsgData.OnTimeRequest = defaultOnTime
    onTimeCmdMsg.write(onTimeCmdMsgData)
    messaging.THRArrayOnTimeCmdMsg_C_addAuthor(module.thrCmdOutMsg, onTimeCmdMsg)

    # connect messages
    module.navDataInMsg.subscribeTo(navTransMsg)
    module.burnDataInMsg.subscribeTo(dvBurnCmdMsg)

    # Setup logging on the test module output messages so that we get all the writes to it
    onTimeDataLog = onTimeCmdMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, onTimeDataLog)
    burnExecDataLog = module.burnExecOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, burnExecDataLog)

    unitTestSim.InitializeSimulation()

    # compute true values
    numTimeSteps = 10
    onTimeTrue = np.zeros([numTimeSteps, numThrusters])
    burnExecutingTrue = np.zeros([numTimeSteps])
    burnCompleteTrue = np.zeros([numTimeSteps])
    for i in range(0, numTimeSteps):
        navTransMsgData.vehAccumDV = acceleration_N * updateRate * i
        navTransMsg.write(navTransMsgData, unitTestSim.TotalSim.CurrentNanos)

        unitTestSim.ConfigureStopTime(i * testProcessRate)
        unitTestSim.ExecuteSimulation()

        if (np.linalg.norm(navTransMsgData.vehAccumDV) >= np.linalg.norm(dvBurnCmdMsgData.dvInrtlCmd)) and \
                (updateRate * (i+1) > module.minTime) or \
                (module.maxTime != 0.0 and updateRate * (i+1) > module.maxTime):
            onTimeTrue[i] = np.zeros(numThrusters)
            burnExecutingTrue[i] = 0
            burnCompleteTrue[i] = 1
        else:
            onTimeTrue[i] = np.ones(numThrusters)
            burnExecutingTrue[i] = 1
            burnCompleteTrue[i] = 0

    # pull module output
    onTime = onTimeDataLog.OnTimeRequest[:, :numThrusters]
    burnExecuting = burnExecDataLog.burnExecuting
    burnComplete = burnExecDataLog.burnComplete

    # compare the module results to the truth values
    paramsString = ' for DV={}, min time={}, max time={}'.format(
        str(p1_dv),
        str(p2_tmin),
        str(p3_tmax))

    np.testing.assert_equal(onTime,
                            onTimeTrue,
                            err_msg=('Variable: onTime' + paramsString),
                            verbose=True)

    np.testing.assert_equal(burnExecuting,
                            burnExecutingTrue,
                            err_msg=('Variable: burnExecuting' + paramsString),
                            verbose=True)

    np.testing.assert_equal(burnComplete,
                            burnCompleteTrue,
                            err_msg=('Variable: burnComplete' + paramsString),
                            verbose=True)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_dvExecuteGuidance(False, dvMagnitude[0], minTime[0], maxTime[0])
