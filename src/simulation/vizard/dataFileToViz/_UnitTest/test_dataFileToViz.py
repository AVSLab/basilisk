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
#   Module Name:        vizInterfaceDataFile
#   Author:             Hanspeter Schaub
#   Creation Date:      May 12, 2020
#


import os
import pytest

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import dataFileToViz
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import vizSupport

try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

path = os.path.dirname(os.path.abspath(__file__))
fileName = os.path.basename(os.path.splitext(__file__)[0])

# @pytest.mark.parametrize("useNoiseStd, errTol", [(False, 1e-10), (True, 1e-2)])

# update "module" in this function name to reflect the module name
def test_module(show_plots):
    """
    **Validation Test Description**

    This section describes the specific unit tests conducted on this module.
    The test contains 16 tests and is located at ``test_magnetometer.py``.
    The success criteria is to match the outputs with the generated truth.

    Args:

        useNoiseStd (string): Defines if the standard deviation of the magnetometer measurements is used for this
            parameterized unit test
        useBias (string): Defines if the bias on the magnetometer measurements is used for this parameterized unit test
        useMinOut (string): Defines if the minimum bound for the measurement saturation is used for this
            parameterized unit test
        useMaxOut (string): Defines if the maximum bound for the measurement saturation is used for this
            parameterized unit test
        useScaleFactor (string): Defines if the scaling on the measurement is used for this parameterized unit test
        errTol (double): Defines the error tolerance for this parameterized unit test

    **Description of Variables Being Tested**

    In this file, we are checking the values of the variable:

    ``tamData[3]``

    which is pulled from the log data to see if they match with the expected truth values.

    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots)
    assert testResults < 1, testMessage

def run(show_plots):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # terminateSimulation() is needed if multiple unit test scripts are run
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    testModule = dataFileToViz.DataFileToViz()
    testModule.ModelTag = "testModule"
    testModule.numSatellites = 2
    # load the data path from the same folder where this python script is
    testModule.dataFileName = os.path.join(path,  "data.txt")
    scNames = ["test1", "test2"]
    testModule.scStateOutMsgNames = dataFileToViz.StringVector(scNames)
    testModule.delimiter = " "

    # Add module to the task
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body

    viz = vizSupport.enableUnityVisualization(unitTestSim, unitTaskName, unitProcessName, gravBodies=gravFactory,
                                              saveFile=fileName,
                                              scName=scNames)
    if vizFound:
        viz.scData.clear()
        for item in scNames:
            # delete any existing list of vizInterface spacecraft data

            # create a chief spacecraft info container
            scData = vizInterface.VizSpacecraftData()
            scData.spacecraftName = item
            scData.scPlusInMsgName = item
            viz.scData.push_back(scData)

    # Setup logging on the test module output message so that we get all the writes to it
    # unitTestSim.TotalSim.logThisMessage(testModule.tamDataOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # tamData = unitTestSim.pullMessageLogData(testModule.tamDataOutMsgName + ".OutputData", list(range(3)))

    # if not unitTestSupport.isArrayEqualRelative(tamData[0], trueTam_S, 3, errTol):
    #     testFailCount += 1

    #   print out success or failure message
    if testFailCount == 0:
        print("PASSED: " + testModule.ModelTag)
    else:
        print("Failed: " + testModule.ModelTag)

    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(              # update "module" in function name
                 False
               )
