
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
# Bore Angle Calculation Test
#
# Purpose:  Test the proper function of the ore Angle Calculation module.
#           Proper function is tested by
#
# Author:   Rachel Mamich
# Creation Date:  Jun. 30, 2017
#

import os

import numpy
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import boreAngCalc
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

path = os.path.dirname(os.path.abspath(__file__))

class ResultsStore:
    def __init__(self):
        self.PassFail = []
    def texSnippet(self):
        for i in range(len(self.PassFail)):
            snippetName = 'Result' + str(i)
            if self.PassFail[i] == 'PASSED':
                textColor = 'ForestGreen'
            elif self.PassFail[i] == 'FAILED':
                textColor = 'Red'
            texSnippet =  r'\textcolor{' + textColor + '}{'+ self.PassFail[i] + '}'
            unitTestSupport.writeTeXSnippet(snippetName, texSnippet, path)

@pytest.fixture(scope="module")
def testFixture():
    listRes = ResultsStore()
    yield listRes
    listRes.texSnippet()

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("boresightLoc, eulerLoc",
                         [([1.0 / numpy.sqrt(3), 1.0 / numpy.sqrt(3), 1.0 / numpy.sqrt(3)], [0.0, 0.0, 0.0]),
                          ([-1.0 / numpy.sqrt(3), 1.0 / numpy.sqrt(3), 1.0 / numpy.sqrt(3)], [0.0, 0.0, 0.0]),
                          ([1.0 / numpy.sqrt(3), -1.0 / numpy.sqrt(3), 1.0 / numpy.sqrt(3)], [0.0, 0.0, 0.0]),
                          ([-1.0 / numpy.sqrt(3), -1.0 / numpy.sqrt(3), 1.0 / numpy.sqrt(3)], [0.0, 0.0, 0.0]),
                          ([1.0 / numpy.sqrt(3), 1.0 / numpy.sqrt(3), -1.0 / numpy.sqrt(3)], [0.0, 0.0, 0.0]),
                          ([-1.0 / numpy.sqrt(3), 1.0 / numpy.sqrt(3), -1.0 / numpy.sqrt(3)], [0.0, 0.0, 0.0]),
                          ([1.0 / numpy.sqrt(3), -1.0 / numpy.sqrt(3), -1.0 / numpy.sqrt(3)], [0.0, 0.0, 0.0]),
                          ([-1.0 / numpy.sqrt(3), -1.0 / numpy.sqrt(3), -1.0 / numpy.sqrt(3)], [0.0, 0.0, 0.0]),
                          ([0.0, 0.0, 1.0], [numpy.pi / 4, numpy.pi / 4, 0.0]),
                          ([0.0, 0.0, 1.0], [3 * numpy.pi / 4, numpy.pi / 4, 0.0]),
                          ([0.0, 0.0, 1.0], [5 * numpy.pi / 4, numpy.pi / 4, 0.0]),
                          ([0.0, 0.0, 1.0], [-numpy.pi / 4, numpy.pi / 4, 0.0]),
                          ([0.0, 0.0, 1.0], [numpy.pi / 4, -numpy.pi / 4, 0.0]),
                          ([0.0, 0.0, 1.0], [3 * numpy.pi / 4, -numpy.pi / 4, 0.0]),
                          ([0.0, 0.0, 1.0], [5 * numpy.pi / 4, -numpy.pi / 4, 0.0]),
                          ([0.0, 0.0, 1.0], [-numpy.pi / 4, -numpy.pi / 4, 0.0]),
                          ([1.0, 0.0, 0.0], [0.0, 0.0, 0.0])])
# # provide a unique test method name, starting with test_
def test_bore_ang_calc(testFixture, show_plots, boresightLoc, eulerLoc):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = bore_ang_calc_func(testFixture, show_plots, boresightLoc, eulerLoc)
    assert testResults < 1, testMessage

# Run unit test
def bore_ang_calc_func(testFixture, show_plots, boresightLoc, eulerLoc):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    TotalSim = SimulationBaseClass.SimBaseClass()

    DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    spiceMessage = messaging.SpicePlanetStateMsgPayload()
    stateMessage = messaging.SCStatesMsgPayload()
    angMessage = messaging.BoreAngleMsgPayload()
    vehPosition = [10000.0, 0.0, 0.0]
    sunPosition = [10000.0, 1000.0, 0.0]
    stateMessage.r_BN_N = vehPosition
    stateMessage.v_BN_N = [-365052.0511, 0.0, 0.0]
    if eulerLoc[0] == 0.0:
        stateMessage.sigma_BN = [0.0, 0.0, 0.0]
    else:
        stateMessage.sigma_BN = RigidBodyKinematics.euler3212MRP(eulerLoc)
    spiceMessage.PositionVector = sunPosition
    spiceMessage.PlanetName = "sun"
    # Inertial State output Message
    scMsg = messaging.SCStatesMsg().write(stateMessage)

    # Sun Planet Data Message
    sunMsg = messaging.SpicePlanetStateMsg().write(spiceMessage)

    # Initialize the spice modules that we are using.
    BACObject = boreAngCalc.BoreAngCalc()
    BACObject.ModelTag = "solarArrayBoresight"
    BACObject.boreVec_B = boresightLoc  # boresight in body frame
    BACObject.scStateInMsg.subscribeTo(scMsg)
    BACObject.celBodyInMsg.subscribeTo(sunMsg)

    TotalSim.AddModelToTask(unitTaskName, BACObject)
    #
    # Configure simulation
    TotalSim.ConfigureStopTime(int(1.0 * 1E9))

    dataLog = BACObject.angOutMsg.recorder()
    TotalSim.AddModelToTask(unitTaskName, dataLog)

    BACObjectLog = BACObject.logger("boreVec_Po")
    TotalSim.AddModelToTask(unitTaskName, BACObjectLog)

    # Execute simulation
    TotalSim.InitializeSimulation()
    TotalSim.ExecuteSimulation()
    ###################################################################################################################
    #
    # Begin testing module results to truth values

    simMiss = dataLog.missAngle
    simAz = dataLog.azimuth
    simBoreVecPt = BACObjectLog.boreVec_Po

    # Truth values
    dcm_BN = RigidBodyKinematics.MRP2C(stateMessage.sigma_BN)
    relPosVector = numpy.subtract(spiceMessage.PositionVector, stateMessage.r_BN_N)
    relVelVector = numpy.subtract(spiceMessage.VelocityVector, stateMessage.v_BN_N)
    magRelVelVec = numpy.sqrt(relVelVector[0] ** 2 + relVelVector[1] ** 2 + relVelVector[2] ** 2)
    if magRelVelVec == 0:
        secPointVector = numpy.zeros((1, 3))
        magSecPtVec = 0
    else:
        secPointVector = numpy.cross(relPosVector, relVelVector) / numpy.linalg.norm(numpy.cross(relPosVector,
                                                                                                 relVelVector))
        magSecPtVec = 1
    primPointVector = relPosVector / numpy.linalg.norm(relPosVector)  # r_p/b_N
    dcm_PoN = numpy.zeros((3, 3))
    dcm_PoN[0, 0:2] = primPointVector[0:2]
    magPrimPtVec = numpy.sqrt(primPointVector[0] ** 2 + primPointVector[1] ** 2 + primPointVector[2] ** 2)
    if magPrimPtVec != 0 and magSecPtVec != 0:
        dcm_PoN_2 = numpy.cross(primPointVector, secPointVector) / numpy.linalg.norm(
            numpy.cross(primPointVector, secPointVector))
        for i in range(3):
            dcm_PoN[2, i] = dcm_PoN_2[i]
    dcm_PoN_1 = numpy.cross(dcm_PoN_2, primPointVector)
    for i in range(3):
        dcm_PoN[1, i] = dcm_PoN_1[i]
    dcm_BPo = numpy.dot(dcm_BN, dcm_PoN.transpose())
    vecBore_B = numpy.zeros((3, 1))
    for i in range(3):
        vecBore_B[i, 0] = BACObject.boreVec_B[i][0]
    boreVecPoint = numpy.dot(numpy.transpose(dcm_BPo), vecBore_B)
    boreVecPoint_1 = []
    for i in range(3):
        boreVecPoint_1.append(boreVecPoint[i, 0])
    boreVecPoint_1 = numpy.array(boreVecPoint_1)

    ####################################################################################################################
    # attempt calculation in body frame
    r_B = numpy.dot(dcm_BN, stateMessage.r_BN_N)  # BN * N = B

    # Set tolersnce
    AllowTolerance = 1E-10
    boreVecPoint_final = [numpy.ndarray.tolist(boreVecPoint_1)]
    simBoreVecPt_final = [simBoreVecPt[0]]

    testFailCount, testMessages = unitTestSupport.compareArray(boreVecPoint_final, simBoreVecPt_final,
                                                               AllowTolerance,
                                                               "Calculating the vector boreVec_Po.",
                                                               testFailCount, testMessages)
    # Truth values
    #boreVecPoint_1 = [0.0, 1.0, 0.0]

    baselinePoint = [1.0, 0.0, 0.0]
    baselinePoint = numpy.array(baselinePoint)
    dotValue = numpy.dot(boreVecPoint_1, baselinePoint)
    r_N = numpy.dot(numpy.transpose(dcm_BN), BACObject.boreVec_B)
    r_N = [item for sublist in r_N for item in sublist]
    baselineProj = numpy.dot(numpy.transpose(dcm_PoN), baselinePoint)
    dotValue_2 = numpy.dot(r_N, baselineProj)
    boresightMissAng = numpy.arccos(dotValue)
    boresightMissAng_2 = numpy.arccos(dotValue_2)  # boresight calc using body frame
    if boresightMissAng == numpy.pi / 2:
        simAz_final = numpy.array(simAz[-1])
        boresightAzimuth = simAz_final
        print("The miss angle is 0, therefore the miss angle is ill defined!")
    else:
        boresightAzimuth = numpy.arctan2(boreVecPoint_1[2], boreVecPoint_1[1])

    # Next Check
    AllowTolerance = 1E-10
    simMiss_final = numpy.array(simMiss[-1])
    if (boresightMissAng - simMiss_final) > AllowTolerance:  # Skip test days that are Sunday because of the end of a GPS week
        testFailCount += 1
        testMessages.append(
            "FAILED: Calculating the miss angle of the boresight failed with difference of: %(DiffVal)f \n" % \
            {"DiffVal": boresightMissAng - simMiss_final})
    simAz_final = numpy.array(simAz[-1])
    if (boresightAzimuth - simAz_final) > AllowTolerance:  # Skip test days that are Sunday because of the end of a GPS week
        testFailCount += 1
        testMessages.append(
            "FAILED: Calculating the azimuth angle of the boresight failed with difference of: %(DiffVal)f \n" % \
            {"DiffVal": boresightAzimuth - simAz_final})

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED")
        testFixture.PassFail.append("PASSED")
    else:
        print(testMessages)
        testFixture.PassFail.append("FAILED")

    # each test method requires a single assert method to be called
    #   this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    bore_ang_calc_func(ResultsStore(), False,  # show_plots
                       [1.0 / numpy.sqrt(3), 1.0 / numpy.sqrt(3), 1.0 / numpy.sqrt(3)], [0.0, 0.0, 0.0])
