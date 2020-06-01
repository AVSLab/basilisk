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
import numpy as np

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import dataFileToViz
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import vizSupport
from Basilisk.simulation import bskLogging
from Basilisk.utilities import RigidBodyKinematics as rbk

try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

path = os.path.dirname(os.path.abspath(__file__))
fileName = os.path.basename(os.path.splitext(__file__)[0])

@pytest.mark.parametrize("convertPosUnits", [-1, 1000.])
@pytest.mark.parametrize("attType", [-1, 0, 1, 2])

# update "module" in this function name to reflect the module name
def test_module(show_plots, convertPosUnits, attType):
    """
    **Validation Test Description**

    This section describes the specific unit tests conducted on this module.
    The test reads in simulation from `data.txt`, run the module, and compares the Basilisk
    spacecraft state messages with known values.

    Args:

        convertPosUnits (double): If positive, then this conversion factor is set.  If negative, then the
            default value of 1000. is checked.
        attType (int): -1 (use default), 0 (MRP), 1 (quaternion), 2 (3-2-1 Euler Angles)

    **Description of Variables Being Tested**

    In this file, we are checking the values of the spacecraft state output message for both spacecraft:

    - ``r_BN_N[3]``
    - ``sigma_BN[3]``

    which is pulled from the log data to see if they match with the expected truth values.

    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, convertPosUnits, attType, False)
    assert testResults < 1, testMessage

def run(show_plots, convertPosUnits, attType, verbose):

    if not verbose:
        bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)
    simulationTime = macros.sec2nano(1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    testModule = dataFileToViz.DataFileToViz()
    testModule.ModelTag = "testModule"
    testModule.numSatellites = 2
    # load the data path from the same folder where this python script is
    path = os.path.dirname(os.path.abspath(__file__))
    testModule.dataFileName = os.path.join(path, "data.txt")
    scNames = ["test1", "test2"]
    testModule.scStateOutMsgNames = dataFileToViz.StringVector(scNames)
    testModule.delimiter = ","
    if convertPosUnits > 0:
        testModule.convertPosToMeters = convertPosUnits
    else:
        convertPosUnits = 1000.
    if attType >= 0:
        testModule.attitudeType = attType
    if attType == 1:
        testModule.dataFileName = os.path.join(path, "data4.txt")

    # Add module to the task
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body

    viz = vizSupport.enableUnityVisualization(unitTestSim, unitTaskName, unitProcessName, gravBodies=gravFactory,
                                              # saveFile=fileName,
                                              scName=scNames)
    if vizFound:
        # delete any existing list of vizInterface spacecraft data
        viz.scData.clear()
        for item in scNames:
            # create a chief spacecraft info container
            scData = vizInterface.VizSpacecraftData()
            scData.spacecraftName = item
            scData.scPlusInMsgName = item
            viz.scData.push_back(scData)

    # Setup logging on the test module output message so that we get all the writes to it
    for msgName in scNames:
        unitTestSim.TotalSim.logThisMessage(msgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulationAndDiscover()

    unitTestSim.ConfigureStopTime(simulationTime) 

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    pos1 = unitTestSim.pullMessageLogData(scNames[0] + ".r_BN_N", list(range(3)))
    pos2 = unitTestSim.pullMessageLogData(scNames[1] + ".r_BN_N", list(range(3)))
    att1 = unitTestSim.pullMessageLogData(scNames[0] + ".sigma_BN", list(range(3)))
    att2 = unitTestSim.pullMessageLogData(scNames[1] + ".sigma_BN", list(range(3)))

    # set input data
    if attType == 1:
        line = [0.1, 6761.48, 1569.01, 905.867, -1.95306, 6.3124, 3.64446, 0.182574, 0.365148, 0.547723, 0.730297, 0, 0, 0,
                6761.48, 1569.02, 905.874, -1.95308, 6.31239, 3.64446, -0.182574, 0.365148, 0.547723, 0.730297, 0, 0, 0]
        pos1In = np.array(line[1:1 + 3])
        att1In = rbk.EP2MRP(np.array(line[7:7 + 4]))
        pos2In = np.array(line[14:14 + 3])
        att2In = rbk.EP2MRP(np.array(line[20:20 + 4]))
    else:
        line = [0.1, 6761.48, 1569.01, 905.867, -1.95306, 6.3124, 3.64446, 0.1, 0.2, 0.3, 0, 0, 0,
                     6761.48, 1569.02, 905.874, -1.95308, 6.31239, 3.64446, -0.1, 0.1, 0.3, 0, 0, 0]
        pos1In = np.array(line[1:1+3])
        att1In = np.array(line[7:7+3])
        pos2In = np.array(line[13:13+3])
        att2In = np.array(line[19:19+3])

        if attType == 2:
            att1In = rbk.euler3212MRP(att1In)
            att2In = rbk.euler3212MRP(att2In)

    if not unitTestSupport.isVectorEqual(pos1[0][1:4], pos1In*convertPosUnits, 0.1):
        testFailCount += 1
        testMessages.append("FAILED: " + testModule.ModelTag + " Module failed pos1 check.")
    if not unitTestSupport.isVectorEqual(pos2[0][1:4], pos2In*convertPosUnits, 0.1):
        testFailCount += 1
        testMessages.append("FAILED: " + testModule.ModelTag + " Module failed pos2 check.")
    if not unitTestSupport.isVectorEqual(att1[0][1:4], att1In, 0.1):
        testFailCount += 1
        testMessages.append("FAILED: " + testModule.ModelTag + " Module failed att1 check.")
    if not unitTestSupport.isVectorEqual(att2[0][1:4], att2In, 0.1):
        testFailCount += 1
        testMessages.append("FAILED: " + testModule.ModelTag + " Module failed att2 check.")

    # print out success or failure message
    if testFailCount == 0:
        print("PASSED: " + testModule.ModelTag)
    else:
        print("Failed: " + testModule.ModelTag)
        print(testMessages)

    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
         False,     # showplots
         -1,        # convertPosUnits
         1,        # attType (-1 -> default, 0 -> MRP, 1 -> quaternions, 2 -> 3-2-1 Euler Angles)
         True       # verbosity
       )
