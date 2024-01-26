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

from Basilisk.utilities import SimulationBaseClass
from Basilisk.fswAlgorithms import dvExecuteGuidance
from Basilisk.utilities import macros
from Basilisk.architecture import messaging


# parameters
dvMagnitude = [4.3, 5.0, 10.0]
minTime = [0.0, 4.0]
maxTime = [0.0, 3.0]
startTime = [0.0, 1.0]

paramArray = [dvMagnitude, minTime, maxTime, startTime]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))

@pytest.mark.parametrize("p1_dv, p2_tmin, p3_tmax, p4_tstart", paramList)
def test_dvExecuteGuidance(show_plots, p1_dv, p2_tmin, p3_tmax, p4_tstart):
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
        :param p4_tstart: burn start time

    **Description of Variables Being Tested**

    The content of the THRArrayOnTimeCmdMsg and DvExecutionDataMsg output messages is compared with the true values.
    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()

    updateRate = 0.5
    testProcessRate = macros.sec2nano(updateRate)
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
    dvBurnCmdMsgData.burnStartTime = macros.sec2nano(p4_tstart)
    dvBurnCmdMsg = messaging.DvBurnCmdMsg().write(dvBurnCmdMsgData)

    # Create thruster on time message and add the module as author. This allows us to write an initial message that does
    # not come from the module
    onTimeCmdMsg = messaging.THRArrayOnTimeCmdMsg_C()
    onTimeCmdMsgData = messaging.THRArrayOnTimeCmdMsgPayload()
    # set on time to some non-zero values to simulate that DV burn is executed. Needs to be stopped/zeroed by module
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
        if updateRate * i > p4_tstart:
            navTransMsgData.vehAccumDV = acceleration_N * (updateRate * i - p4_tstart)
        navTransMsg.write(navTransMsgData, unitTestSim.TotalSim.CurrentNanos)

        # thrusters nominally on, module needs to overwrite and zero if necessary
        onTimeCmdMsg.write(onTimeCmdMsgData, unitTestSim.TotalSim.CurrentNanos)

        unitTestSim.ConfigureStopTime(i * testProcessRate)
        unitTestSim.ExecuteSimulation()

        if (updateRate * (i+1) <= p4_tstart):
            onTimeTrue[i] = np.zeros(numThrusters)
            burnExecutingTrue[i] = 0
            burnCompleteTrue[i] = 0
        elif (np.linalg.norm(navTransMsgData.vehAccumDV) >= np.linalg.norm(dvBurnCmdMsgData.dvInrtlCmd)) and \
                (updateRate * (i+1) - p4_tstart > module.minTime) or \
                (module.maxTime != 0.0 and updateRate * (i+1) - p4_tstart > module.maxTime):
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

    print(onTime)
    print(onTimeTrue)
    print(burnExecuting)
    print(burnExecutingTrue)

    # compare the module results to the truth values
    paramsString = ' for DV={}, min time={}, max time={}, start time={}'.format(
        str(p1_dv),
        str(p2_tmin),
        str(p3_tmax),
        str(p4_tstart))

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
    test_dvExecuteGuidance(False, dvMagnitude[0], minTime[0], maxTime[0], startTime[1])
