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


@pytest.mark.parametrize("convertPosUnits", [-1, 1000])
@pytest.mark.parametrize("attType", [-1, 0, 1, 2])
@pytest.mark.parametrize("checkThruster", [False, True])
@pytest.mark.parametrize("checkRW", [False, True])


def test_module(show_plots, convertPosUnits, attType, checkThruster, checkRW):
    """
    **Validation Test Description**

    This section describes the specific unit tests conducted on this module.
    The test reads in simulation from ``data.txt``, run the module, and compares the Basilisk
    spacecraft state messages with known values.

    Args:

        convertPosUnits (double): If positive, then this conversion factor is set.  If negative, then the
            default value of 1000. is checked.
        attType (int): -1 (use default), 0 (MRP), 1 (quaternion), 2 (3-2-1 Euler Angles)
        checkThruster (bool): flag to check for simulation data with thrusters
        checkRW (bool): flag to check for simulation data with RW information

    **Description of Variables Being Tested**

    In this file, we are checking the values of the spacecraft state output message for both spacecraft:

    - ``r_BN_N[3]``
    - ``sigma_BN[3]``
    - ``thrustForce``

    which is pulled from the log data to see if they match with the expected truth values.

    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, convertPosUnits, attType, checkThruster, checkRW, False)
    assert testResults < 1, testMessage

def run(show_plots, convertPosUnits, attType, checkThruster, checkRW, verbose):

    if not verbose:
        bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    dtSeconds = 0.1
    simTimeSeconds = 2.0
    testProcessRate = macros.sec2nano(dtSeconds)
    simulationTime = macros.sec2nano(simTimeSeconds)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # create the simulation data file
    rB1N = [6761.48, 1569.01, 905.867]
    vB1N = [-1.95306, 6.3124, 3.64446]
    betaB1N = [0.182574, 0.365148, 0.547723, 0.730297]
    sigmaB1N = [0.1, 0.2, 0.3]
    omega = [0., 0., 0.]
    rB2N = [6761.48, 1569.02, 905.874]
    vB2N = [-1.95308, 6.31239, 3.64446]
    betaB2N = [-0.182574, 0.365148, 0.547723, 0.730297]
    sigmaB2N = [-0.1, 0.1, 0.3]
    dataFileName = "data" + str(convertPosUnits) + str(attType) + str(checkThruster) + str(checkRW) + ".txt"
    dataFileName = os.path.join(path, dataFileName)
    delimiter = ","
    fDataFile = open(dataFileName, "w+")
    for i in range(0, int(simTimeSeconds/dtSeconds)+2):
        t = round(i*dtSeconds, 4)

        # sc1
        lineString = str(t) + delimiter + str(rB1N)[1:-1] + delimiter + str(vB1N)[1:-1] + delimiter
        if attType == 1:
            lineString += str(betaB1N)[1:-1] + delimiter
        else:
            lineString += str(sigmaB1N)[1:-1] + delimiter
        lineString += str(omega)[1:-1] + delimiter
        if checkThruster:
            th1ACS = 1.
            th1DV = 100.
            numACS1 = 1
            numDV1 = 1
            lineString += str(th1ACS) + delimiter + str(th1DV) + delimiter
        if checkRW:
            Omega1sc1 = 100.*macros.RPM
            u1sc1 = 0.1
            Omega2sc1 = 500. * macros.RPM
            u2sc1 = -0.1
            lineString += str(Omega1sc1) + delimiter + str(u1sc1) + delimiter
            lineString += str(Omega2sc1) + delimiter + str(u2sc1) + delimiter

        # sc2
        lineString += str(rB2N)[1:-1] + delimiter + str(vB2N)[1:-1] + delimiter
        if attType == 1:
            lineString += str(betaB2N)[1:-1] + delimiter
        else:
            lineString += str(sigmaB2N)[1:-1] + delimiter
        lineString += str(omega)[1:-1]
        if checkThruster:
            th2ACS = 2.
            th2DV = 200.
            numACS2 = 1
            numDV2 = 2
            lineString += delimiter + str(th2ACS) + delimiter + str(th2DV) + delimiter + str(th2DV)
        if checkRW:
            Omega1sc2 = 1000.*macros.RPM
            u1sc2 = 0.3
            lineString += delimiter + str(Omega1sc2) + delimiter + str(u1sc2)

        lineString += '\n'
        fDataFile.write(lineString)
    fDataFile.close()

    # Construct algorithm and associated C++ container
    testModule = dataFileToViz.DataFileToViz()
    testModule.ModelTag = "testModule"
    testModule.numSatellites = 2
    # load the data path from the same folder where this python script is
    testModule.dataFileName = dataFileName
    scNames = ["test1", "test2"]
    testModule.scStateOutMsgNames = dataFileToViz.StringVector(scNames)
    testModule.delimiter = delimiter
    if convertPosUnits > 0:
        testModule.convertPosToMeters = convertPosUnits
    else:
        convertPosUnits = 1000.
    if attType >= 0:
        testModule.attitudeType = attType

    if checkThruster:
        # sc1
        thrModelTagAdcs1 = scNames[0] + "_adcs"
        thSetAdcs1 = dataFileToViz.ThrClusterMap()
        thSetAdcs1.thrCount = numACS1
        thSetAdcs1.thrTag = thrModelTagAdcs1
        thSetAdcs1.color = vizSupport.toRGBA255("red")

        thrModelTagDv1 = scNames[0] + "_dv"
        thSetDV1 = dataFileToViz.ThrClusterMap()
        thSetDV1.thrCount = numDV1
        thSetDV1.thrTag = thrModelTagDv1
        thSetDV1.color = vizSupport.toRGBA255("blue")

        thList1 = [thSetAdcs1, thSetDV1]
        testModule.appendThrClusterMap(dataFileToViz.VizThrConfig(thList1))

        # set ACS thruster position and direction states
        testModule.appendThrPos([0, 0, 3.])  # thr location in B frame, meters
        testModule.appendThrDir([0, 0, -1])  # thr force direction
        testModule.appendThrForceMax(th1ACS)

        # set DV thruster position and direction states
        testModule.appendThrPos([0., 0., -3.])
        testModule.appendThrDir([0, 0, 1])
        testModule.appendThrForceMax(th1DV)


        # sc2
        thrModelTagAdcs2 = scNames[1] + "_adcs"
        thSetAdcs2 = dataFileToViz.ThrClusterMap()
        thSetAdcs2.thrCount = numACS2
        thSetAdcs2.thrTag = thrModelTagAdcs2

        thrModelTagDv2 = scNames[1] + "_dv"
        thSetDV2 = dataFileToViz.ThrClusterMap()
        thSetDV2.thrCount = numDV2
        thSetDV2.thrTag = thrModelTagDv2

        thList2 = [thSetAdcs2, thSetDV2]
        testModule.appendThrClusterMap(dataFileToViz.VizThrConfig(thList2))

        # set ACS thruster position and direction states
        testModule.appendThrPos([0, 0, 3.])
        testModule.appendThrDir([0, 0, -1])
        testModule.appendThrForceMax(th2ACS)

        # set DV thruster position and direction states
        testModule.appendThrPos([0., 0., -3.])
        testModule.appendThrDir([0, 0, 1])
        testModule.appendThrForceMax(th2DV)
        testModule.appendThrPos([0., 2., -3.])
        testModule.appendThrDir([0, 0, 1])
        testModule.appendThrForceMax(th2DV)

    if checkRW:
        # SC1
        rwSc1MsgList = ["data_rw0_test1", "data_rw1_test1"]
        testModule.appendRwMsgNames(dataFileToViz.StringVector(rwSc1MsgList))
        # RW 1
        testModule.appendRwPos([0, 0, 0])
        testModule.appendRwDir([1, 0, 0])
        testModule.appendOmegaMax(3000.*macros.RPM)
        testModule.appendUMax(0.5)
        # RW2
        testModule.appendRwPos([0, 0, 0])
        testModule.appendRwDir([0, 1, 0])
        testModule.appendOmegaMax(3000.*macros.RPM)
        testModule.appendUMax(0.5)

        # SC2
        rwSc2MsgList = ["data_rw0_test2"]
        testModule.appendRwMsgNames(dataFileToViz.StringVector(rwSc2MsgList))
        # RW 1
        testModule.appendRwPos([0, 0, 0])
        testModule.appendRwDir([0, 1, 0])
        testModule.appendOmegaMax(3000.*macros.RPM)
        testModule.appendUMax(0.5)

    # Add module to the task
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body

    viz = vizSupport.enableUnityVisualization(unitTestSim, unitTaskName, unitProcessName, gravBodies=gravFactory,
                                              saveFile=__file__,
                                              scName=scNames)
    if vizFound:
        # delete any existing list of vizInterface spacecraft data
        viz.scData.clear()
        scCounter = 0
        if checkThruster:
            viz.settings.defaultThrusterColor = vizSupport.toRGBA255("yellow")
        for item in scNames:
            # create a chief spacecraft info container
            scData = vizInterface.VizSpacecraftData()
            scData.spacecraftName = item
            scData.scPlusInMsgName = item
            if checkThruster:
                if scCounter == 0:
                    scData.thrMsgData = vizInterface.VizThrConfig(thList1)
                else:
                    scData.thrMsgData = vizInterface.VizThrConfig(thList2)
            if checkRW:
                if scCounter == 0:
                    scData.numRW = len(rwSc1MsgList)
                    scData.rwInMsgName = vizInterface.StringVector(rwSc1MsgList)
                else:
                    scData.numRW = len(rwSc2MsgList)
                    scData.rwInMsgName = vizInterface.StringVector(rwSc2MsgList)
            viz.scData.push_back(scData)
            scCounter += 1

    # Setup logging on the test module output message so that we get all the writes to it
    for msgName in scNames:
        unitTestSim.TotalSim.logThisMessage(msgName, testProcessRate)
    if checkThruster:
        thrMsgName = []
        for i in range(numACS1):
            thrMsgName.append("thruster_" + thrModelTagAdcs1 + "_" + str(i) + "_data")
        for i in range(numDV1):
            thrMsgName.append("thruster_" + thrModelTagDv1 + "_" + str(i) + "_data")
        for i in range(numACS2):
            thrMsgName.append("thruster_" + thrModelTagAdcs2 + "_" + str(i) + "_data")
        for i in range(numDV2):
            thrMsgName.append("thruster_" + thrModelTagDv2 + "_" + str(i) + "_data")
        for name in thrMsgName:
            unitTestSim.TotalSim.logThisMessage(name, testProcessRate)
    if checkRW:
        unitTestSim.TotalSim.logThisMessage(rwSc1MsgList[0], testProcessRate)
        unitTestSim.TotalSim.logThisMessage(rwSc1MsgList[1], testProcessRate)
        unitTestSim.TotalSim.logThisMessage(rwSc2MsgList[0], testProcessRate)

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
    if checkThruster:
        thrData = []
        for name in thrMsgName:
            thrData.append(unitTestSim.pullMessageLogData(name + ".thrustForce", list(range(1))))
    if checkRW:
        rw1Sc1OmegaData = unitTestSim.pullMessageLogData(rwSc1MsgList[0] + ".Omega", list(range(1)))
        rw1Sc1uData = unitTestSim.pullMessageLogData(rwSc1MsgList[0] + ".u_current", list(range(1)))
        rw2Sc1OmegaData = unitTestSim.pullMessageLogData(rwSc1MsgList[1] + ".Omega", list(range(1)))
        rw2Sc1uData = unitTestSim.pullMessageLogData(rwSc1MsgList[1] + ".u_current", list(range(1)))
        rw1Sc2OmegaData = unitTestSim.pullMessageLogData(rwSc2MsgList[0] + ".Omega", list(range(1)))
        rw1Sc2uData = unitTestSim.pullMessageLogData(rwSc2MsgList[0] + ".u_current", list(range(1)))

    # set input data
    pos1In = np.array(rB1N)
    pos2In = np.array(rB2N)
    if attType == 1:
        att1In = rbk.EP2MRP(np.array(betaB1N))
        att2In = rbk.EP2MRP(np.array(betaB2N))
    else:
        att1In = np.array(sigmaB1N)
        att2In = np.array(sigmaB2N)

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
    if checkThruster:
        if not unitTestSupport.isDoubleEqualRelative(thrData[0][0][1], th1ACS, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed th1ACS check.")
        if not unitTestSupport.isDoubleEqualRelative(thrData[1][0][1], th1DV, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed th1ACS check.")
        if not unitTestSupport.isDoubleEqualRelative(thrData[2][0][1], th2ACS, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed th2ACS check.")
        if not unitTestSupport.isDoubleEqualRelative(thrData[3][0][1], th2DV, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed th2DV (1st) check.")
        if not unitTestSupport.isDoubleEqualRelative(thrData[4][0][1], th2DV, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed th2DV (2nd) check.")
    if checkRW:
        if not unitTestSupport.isDoubleEqualRelative(rw1Sc1OmegaData[0][1], Omega1sc1, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed Omega1sc1 check.")
        if not unitTestSupport.isDoubleEqualRelative(rw1Sc1uData[0][1], u1sc1, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed u1sc1 check.")
        if not unitTestSupport.isDoubleEqualRelative(rw2Sc1OmegaData[0][1], Omega2sc1, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed Omega1sc1 check.")
        if not unitTestSupport.isDoubleEqualRelative(rw2Sc1uData[0][1], u2sc1, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed u1sc1 check.")
        if not unitTestSupport.isDoubleEqualRelative(rw1Sc2OmegaData[0][1], Omega1sc2, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed Omega1sc2 check.")
        if not unitTestSupport.isDoubleEqualRelative(rw1Sc2uData[0][1], u1sc2, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed u1sc2 check.")

    # print out success or failure message
    if testFailCount == 0:
        print("PASSED: " + testModule.ModelTag)
    else:
        print("Failed: " + testModule.ModelTag)
        print(testMessages)

    if os.path.exists(dataFileName):
        os.remove(dataFileName)
    else:
        print("Couldn't close file.")

    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
         False,     # showplots
         -1,        # convertPosUnits
         0,        # attType (-1 -> default, 0 -> MRP, 1 -> quaternions, 2 -> 3-2-1 Euler Angles)
         True,      # checkThruster
         False,      # checkRW
         True       # verbose
       )
