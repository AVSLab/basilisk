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

import numpy as np
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.simulation import dataFileToViz
from Basilisk.simulation import spacecraft
from Basilisk.utilities import RigidBodyKinematics as rbk
# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import vizSupport

try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

path = os.path.dirname(os.path.abspath(__file__))

dataFileName = None
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
    global dataFileName
    if os.path.exists(dataFileName):
        os.remove(dataFileName)





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
    global dataFileName
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
            th2ACS = 0.001
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

    # set number of satellites
    testModule.setNumOfSatellites(2)

    # load the data path from the same folder where this python script is
    testModule.dataFileName = dataFileName
    testModule.delimiter = delimiter
    if convertPosUnits > 0:
        testModule.convertPosToMeters = convertPosUnits
    else:
        convertPosUnits = 1000.
    if attType >= 0:
        testModule.attitudeType = attType

    scNames = ["test1", "test2"]

    if checkThruster:
        # sc1
        thSetAdcs1 = dataFileToViz.ThrClusterMap()
        thSetAdcs1.thrTag = "adcs"
        thSetAdcs1.color = vizSupport.toRGBA255("red")

        thSetDV1 = dataFileToViz.ThrClusterMap()
        thSetDV1.thrTag = "dv"
        thSetDV1.color = vizSupport.toRGBA255("blue")

        thList1 = [thSetAdcs1, thSetDV1]
        numTh1 = [1, 1]
        testModule.appendThrClusterMap(dataFileToViz.VizThrConfig(thList1), dataFileToViz.IntVector(numTh1))

        # set ACS thruster position and direction states
        testModule.appendThrPos([0, 0, 3.])  # thr location in B frame, meters
        testModule.appendThrDir([0, 0, -1])  # thr force direction
        testModule.appendThrForceMax(th1ACS)

        # set DV thruster position and direction states
        testModule.appendThrPos([0., 0., -3.])
        testModule.appendThrDir([0, 0, 1])
        testModule.appendThrForceMax(th1DV)

        # sc2
        thSetAdcs2 = dataFileToViz.ThrClusterMap()
        thSetAdcs2.thrTag = "adcs"

        thSetDV2 = dataFileToViz.ThrClusterMap()
        thSetDV2.thrTag = "dv"

        thList2 = [thSetAdcs2, thSetDV2]
        numTh2 = [1, 2]
        testModule.appendThrClusterMap(dataFileToViz.VizThrConfig(thList2), dataFileToViz.IntVector(numTh2))

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

        thrNumList = [numTh1, numTh2]

    if checkRW:
        # set number of RW for SC1
        testModule.appendNumOfRWs(2)
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

        # set number of RW for SC2
        testModule.appendNumOfRWs(1)
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

    # create SC dummy objects to setup basic Vizard settings.  Only one has to have the Grav Bodies attached
    # to show up in Vizard
    scObject1 = spacecraft.Spacecraft()
    scObject1.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    scObject2 = spacecraft.Spacecraft()

    viz = vizSupport.enableUnityVisualization(unitTestSim, unitTaskName, [scObject1, scObject2]
                                              # , saveFile=__file__
                                              )
    if vizFound:
        # over-ride the default to not read the SC states from scObjects, but set them directly
        # to read from the dataFileToFiz output message
        viz.scData.clear()
        for c in range(len(scNames)):
            scData = vizInterface.VizSpacecraftData()
            scData.spacecraftName = scNames[c]
            scData.scStateInMsg.subscribeTo(testModule.scStateOutMsgs[c])

            if checkThruster:
                thrList = []
                thrInfo = []
                for thrLogMsg in testModule.thrScOutMsgs[c]:  # loop over the THR cluster log message
                    thrList.append(thrLogMsg.addSubscriber())
                k = 0
                for info in testModule.thrMsgDataSC[c]:
                    for i in range(thrNumList[c][k]):
                        thrInfo.append(info)
                    k += 1
                scData.thrInMsgs = messaging.THROutputMsgInMsgsVector(thrList)
                scData.thrInfo = vizInterface.ThrClusterVector(thrInfo)

            if checkRW:
                rwList = []
                for rwLogMsg in testModule.rwScOutMsgs[c]:
                    rwList.append(rwLogMsg.addSubscriber())
                scData.rwInMsgs = messaging.RWConfigLogMsgInMsgsVector(rwList)

            viz.scData.push_back(scData)

        if checkThruster:
            viz.settings.defaultThrusterColor = vizSupport.toRGBA255("yellow")

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = []
    for scCounter in range(2):
        dataLog.append(testModule.scStateOutMsgs[scCounter].recorder())
        unitTestSim.AddModelToTask(unitTaskName, dataLog[-1])

    if checkThruster:
        dataThrLog = []
        # SC1
        for i in range(numACS1 + numDV1):
            dataThrLog.append(testModule.thrScOutMsgs[0][i].recorder())
            unitTestSim.AddModelToTask(unitTaskName, dataThrLog[-1])
        # SC2
        for i in range(numACS2 + numDV2):
            dataThrLog.append(testModule.thrScOutMsgs[1][i].recorder())
            unitTestSim.AddModelToTask(unitTaskName, dataThrLog[-1])

    if checkRW:
        dataSc1RW1Log = testModule.rwScOutMsgs[0][0].recorder()
        dataSc1RW2Log = testModule.rwScOutMsgs[0][1].recorder()
        dataSc2RW1Log = testModule.rwScOutMsgs[1][0].recorder()
        unitTestSim.AddModelToTask(unitTaskName, dataSc1RW1Log)
        unitTestSim.AddModelToTask(unitTaskName, dataSc1RW2Log)
        unitTestSim.AddModelToTask(unitTaskName, dataSc2RW1Log)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    unitTestSim.ConfigureStopTime(simulationTime) 

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    pos1 = dataLog[0].r_BN_N
    pos2 = dataLog[1].r_BN_N
    att1 = dataLog[0].sigma_BN
    att2 = dataLog[1].sigma_BN

    if checkThruster:
        thrData = []
        for item in dataThrLog:
            thrData.append(item.thrustForce)
    if checkRW:
        rw1Sc1OmegaData = dataSc1RW1Log.Omega
        rw1Sc1uData = dataSc1RW1Log.u_current
        rw2Sc1OmegaData = dataSc1RW2Log.Omega
        rw2Sc1uData = dataSc1RW2Log.u_current
        rw1Sc2OmegaData = dataSc2RW1Log.Omega
        rw1Sc2uData = dataSc2RW1Log.u_current

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

    if not unitTestSupport.isVectorEqual(pos1[0], pos1In*convertPosUnits, 0.1):
        testFailCount += 1
        testMessages.append("FAILED: " + testModule.ModelTag + " Module failed pos1 check.")
    if not unitTestSupport.isVectorEqual(pos2[0], pos2In*convertPosUnits, 0.1):
        testFailCount += 1
        testMessages.append("FAILED: " + testModule.ModelTag + " Module failed pos2 check.")
    if not unitTestSupport.isVectorEqual(att1[0], att1In, 0.1):
        testFailCount += 1
        testMessages.append("FAILED: " + testModule.ModelTag + " Module failed att1 check.")
    if not unitTestSupport.isVectorEqual(att2[0], att2In, 0.1):
        testFailCount += 1
        testMessages.append("FAILED: " + testModule.ModelTag + " Module failed att2 check.")
    if checkThruster:
        if not unitTestSupport.isDoubleEqualRelative(thrData[0][0], th1ACS, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed th1ACS check.")
        if not unitTestSupport.isDoubleEqualRelative(thrData[1][0], th1DV, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed th1ACS check.")
        if not unitTestSupport.isDoubleEqualRelative(thrData[2][0], th2ACS, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed th2ACS check.")
        if not unitTestSupport.isDoubleEqualRelative(thrData[3][0], th2DV, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed th2DV (1st) check.")
        if not unitTestSupport.isDoubleEqualRelative(thrData[4][0], th2DV, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed th2DV (2nd) check.")
    if checkRW:
        if not unitTestSupport.isDoubleEqualRelative(rw1Sc1OmegaData[0], Omega1sc1, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed Omega1sc1 check.")
        if not unitTestSupport.isDoubleEqualRelative(rw1Sc1uData[0], u1sc1, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed u1sc1 check.")
        if not unitTestSupport.isDoubleEqualRelative(rw2Sc1OmegaData[0], Omega2sc1, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed Omega1sc1 check.")
        if not unitTestSupport.isDoubleEqualRelative(rw2Sc1uData[0], u2sc1, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed u1sc1 check.")
        if not unitTestSupport.isDoubleEqualRelative(rw1Sc2OmegaData[0], Omega1sc2, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed Omega1sc2 check.")
        if not unitTestSupport.isDoubleEqualRelative(rw1Sc2uData[0], u1sc2, 0.001):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Module failed u1sc2 check.")


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
         0,        # attType (-1 -> default, 0 -> MRP, 1 -> quaternions, 2 -> 3-2-1 Euler Angles)
         True,      # checkThruster
         True,      # checkRW
         True       # verbose
       )
    if os.path.exists(dataFileName):
        os.remove(dataFileName)

