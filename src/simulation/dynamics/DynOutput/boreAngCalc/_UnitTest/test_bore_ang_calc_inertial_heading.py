# ISC License
#
# Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# Purpose:  Test the proper function of the Bore Angle Calculation module using the inertial heading option.
#
# Author:   João Vaz Carneiro
# Creation Date:  Jan. 12, 2023
#
import math
import os

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import boreAngCalc
from Basilisk.utilities import SimulationBaseClass, macros as mc, RigidBodyKinematics as rbk, unitTestSupport

path = os.path.dirname(os.path.abspath(__file__))


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("inertialHeading, eulerLoc",
                         [([1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                          ([0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
                          ([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),
                          ([1.0, 0.0, 0.0], [np.pi / 4, 0.0, - np.pi / 4]),
                          ([0.0, 1.0, 0.0], [np.pi / 4, 0.0, - np.pi / 4]),
                          ([0.0, 0.0, 1.0], [np.pi / 4, 0.0, - np.pi / 4]),
                          ([1 / np.sqrt(2), - 1 / np.sqrt(2), 0.0], [np.pi / 4, 0.0, - np.pi / 4]),
                          ([0.0, 1 / np.sqrt(2), 1 / np.sqrt(2)], [np.pi / 4, 0.0, - np.pi / 4]),
                          ([1 / np.sqrt(2), 0.0, - 1 / np.sqrt(2)], [np.pi / 4, 0.0, - np.pi / 4])])
def test_bore_ang_calc_inertial_heading(show_plots, inertialHeading, eulerLoc):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = bore_ang_calc_inertial_heading_func(show_plots, inertialHeading, eulerLoc)
    assert testResults < 1, testMessage


# Run unit test
def bore_ang_calc_inertial_heading_func(show_plots, inertialHeading, eulerLoc):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # Assign task and process names
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "unitProcess"  # arbitrary name (don't change)

    # Create the sim module, process and task
    TotalSim = SimulationBaseClass.SimBaseClass()
    UnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    UnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, mc.sec2nano(1.0)))

    # Create the state message and populate it
    stateMessage = messaging.SCStatesMsgPayload()
    stateMessage.sigma_BN = rbk.euler3212MRP(eulerLoc)
    scMsg = messaging.SCStatesMsg().write(stateMessage)

    # Initialize the bac module
    BACObject = boreAngCalc.BoreAngCalc()
    BACObject.ModelTag = "solarArrayBoresight"
    boreVec_B = [1.0, 0.0, 0.0]
    BACObject.boreVec_B = boreVec_B  # boresight in body frame
    BACObject.scStateInMsg.subscribeTo(scMsg)
    BACObject.inertialHeadingVec_N = inertialHeading
    TotalSim.AddModelToTask(unitTaskName, BACObject)

    # Configure the recorder
    dataLog = BACObject.angOutMsg.recorder()
    TotalSim.AddModelToTask(unitTaskName, dataLog)

    # Execute simulation
    TotalSim.InitializeSimulation()
    TotalSim.TotalSim.SingleStepProcesses()

    # Configure the tests

    # Compute the inertial heading in B frame
    dcm_BN = rbk.MRP2C(stateMessage.sigma_BN)
    inertialHeading_B = dcm_BN.dot(np.array(inertialHeading))

    # Compute the miss angle
    missAngle = math.acos(np.array(boreVec_B).dot(inertialHeading_B))

    # Extract the miss angle from data
    simMissAngle = dataLog.missAngle[0]

    # Compare the results
    tol = 1E-10
    if not unitTestSupport.isDoubleEqual(missAngle, simMissAngle, tol):
        testFailCount += 1
        testMessages.append("FAILED: Calculating the miss angle of the boresight failed \n")

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED")
    else:
        print(testMessages)

    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_bore_ang_calc_inertial_heading(False,  # show_plots
                                        [1.0 / np.sqrt(3), 1.0 / np.sqrt(3), 1.0 / np.sqrt(3)],
                                        [np.pi, 0.0, 0.0])
