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



import sys
import os
import numpy as np
import pytest

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport

from Basilisk.utilities import macros
from Basilisk.simulation import coarse_sun_sensor
from Basilisk.simulation import simMessages


@pytest.mark.parametrize("accuracy", [1e-12])


def test_CSSConfig(show_plots, accuracy):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, accuracy)

    assert testResults < 1, testMessage


def run(show_plots, accuracy):
    """
        At the end of the python script you can specify the following example parameters.

        Args:
            show_plots (bool): Determines if the script should display plots

        """

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))


    # create the CSS modules
    CSS1 = coarse_sun_sensor.CoarseSunSensor()
    CSS1.ModelTag = "CSS1"
    CSS1.fov = 80. * macros.D2R
    CSS1.maxOutput = 10.
    CSS1.cssDataOutMsgName = "CSS1_output"
    CSS1.sunInMsgName = "sun_message"
    # CSS1.cssConfigLogMsgName is not set to check default behavior
    CSS1.nHat_B = [1.0, 0.0, 0.0]
    CSS1.cssConfigLogMsgName = "CSS1_config_log"

    CSS2 = coarse_sun_sensor.CoarseSunSensor()
    CSS2.ModelTag = "CSS2"
    CSS2.r_B = [1., 2., 3.]
    CSS2.fov = 70. * macros.D2R
    CSS2.minOutput = 1.0
    CSS2.maxOutput = 20.
    CSS2.cssDataOutMsgName = "CSS2_output"
    CSS2.sunInMsgName = "sun_message"
    CSS2.cssConfigLogMsgName = "CSS2_custom"
    CSS2.nHat_B = [0.0, -1.0, 0.0]
    CSS2.CSSGroupID = 1

    scSim.AddModelToTask(simTaskName, CSS1)
    scSim.AddModelToTask(simTaskName, CSS2)

    scSim.TotalSim.logThisMessage(CSS1.cssConfigLogMsgName, simulationTimeStep)
    scSim.TotalSim.logThisMessage(CSS2.cssConfigLogMsgName, simulationTimeStep)


    # create sun position input message
    sunPositionMsg = simMessages.SpicePlanetStateSimMsg()
    sunPositionMsg.PositionVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               CSS1.sunInMsgName,
                               sunPositionMsg)

    # create spacecraft state message
    scStateMsg = simMessages.SCPlusStatesSimMsg()
    scStateMsg.r_BN_N = [-10.0, 0.0, 0.0]
    scStateMsg.sigma_BN = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               CSS1.stateInMsgName,
                               scStateMsg)

    scSim.InitializeSimulationAndDiscover()
    scSim.TotalSim.SingleStepProcesses()

    # pull logged data
    dataCSS1pos = scSim.pullMessageLogData(CSS1.cssConfigLogMsgName + ".r_B", list(range(3)))
    dataCSS1nHat = scSim.pullMessageLogData(CSS1.cssConfigLogMsgName + ".nHat_B", list(range(3)))
    dataCSS1fov = scSim.pullMessageLogData(CSS1.cssConfigLogMsgName + ".fov", list(range(1)))
    dataCSS1signal = scSim.pullMessageLogData(CSS1.cssConfigLogMsgName + ".signal", list(range(1)))
    dataCSS1maxSignal = scSim.pullMessageLogData(CSS1.cssConfigLogMsgName + ".maxSignal", list(range(1)))
    dataCSS1minSignal = scSim.pullMessageLogData(CSS1.cssConfigLogMsgName + ".minSignal", list(range(1)))
    dataCSS1CSSGroupID = scSim.pullMessageLogData(CSS1.cssConfigLogMsgName + ".CSSGroupID", list(range(1)))

    dataCSS2pos = scSim.pullMessageLogData(CSS2.cssConfigLogMsgName + ".r_B", list(range(3)))
    dataCSS2nHat = scSim.pullMessageLogData(CSS2.cssConfigLogMsgName + ".nHat_B", list(range(3)))
    dataCSS2fov = scSim.pullMessageLogData(CSS2.cssConfigLogMsgName + ".fov", list(range(1)))
    dataCSS2signal = scSim.pullMessageLogData(CSS2.cssConfigLogMsgName + ".signal", list(range(1)))
    dataCSS2minSignal = scSim.pullMessageLogData(CSS2.cssConfigLogMsgName + ".minSignal", list(range(1)))
    dataCSS2maxSignal = scSim.pullMessageLogData(CSS2.cssConfigLogMsgName + ".maxSignal", list(range(1)))
    dataCSS2CSSGroupID = scSim.pullMessageLogData(CSS2.cssConfigLogMsgName + ".CSSGroupID", list(range(1)))

    # check CSS 1 output
    testFailCount, testMessages = unitTestSupport.compareArray([[0., 0., 0.]], dataCSS1pos,
                                                               accuracy, "CSS1 pos",
                                                               testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArray([CSS1.nHat_B], dataCSS1nHat,
                                                               accuracy, "CSS1 nHat_B",
                                                               testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray([CSS1.fov], dataCSS1fov,
                                                               accuracy, "CSS1 fov",
                                                               testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray([CSS1.maxOutput], dataCSS1signal,
                                                                     accuracy, "CSS1 maxSignal",
                                                                     testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray([0.0], dataCSS1minSignal,
                                                                     accuracy, "CSS1 minSignal",
                                                                     testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray([CSS1.maxOutput], dataCSS1maxSignal,
                                                                     accuracy, "CSS1 maxSignal",
                                                                     testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray([0], dataCSS1CSSGroupID,
                                                                     accuracy, "CSS1 CSSGroupID",
                                                                     testFailCount, testMessages)

    # check CSS 2 output
    testFailCount, testMessages = unitTestSupport.compareArray([CSS2.r_B], dataCSS2pos,
                                                               accuracy, "CSS2 pos",
                                                               testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArray([CSS2.nHat_B], dataCSS2nHat,
                                                               accuracy, "CSS2 nHat_B",
                                                               testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray([CSS2.fov], dataCSS2fov,
                                                                     accuracy, "CSS2 fov",
                                                                     testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray([CSS2.minOutput], dataCSS2signal,
                                                                     accuracy, "CSS2 signal",
                                                                     testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray([CSS2.maxOutput], dataCSS2maxSignal,
                                                                     accuracy, "CSS2 maxSignal",
                                                                     testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray([CSS2.minOutput], dataCSS2minSignal,
                                                                     accuracy, "CSS2 minSignal",
                                                                     testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray([CSS2.CSSGroupID], dataCSS2CSSGroupID,
                                                                     accuracy, "CSS2 CSSGroupID",
                                                                     testFailCount, testMessages)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: CSS config test")
    else:
        print("FAILED: CSS config test")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
if __name__ == "__main__":
    run(
        False,       # show_plots
        1e-12           # accuracy
    )

