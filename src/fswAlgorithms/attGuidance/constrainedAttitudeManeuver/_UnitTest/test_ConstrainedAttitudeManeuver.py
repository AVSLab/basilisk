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
#   Module Name:        constrainedAttitudeManeuver
#   Author:             Riccardo Calaon
#   Creation Date:      March 14, 2021
#


import os
import pytest
import numpy as np

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.fswAlgorithms import constrainedAttitudeManeuver
from Basilisk.utilities import macros
from Basilisk.architecture import bskLogging
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging

import matplotlib.pyplot as plt

path = os.path.dirname(os.path.abspath(__file__))
dataFileName = None

def CAMTestFunction(N):

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
    
    CAM = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver(N)
    CAM.sigma_BN_goal = [0.75, 0, 0]
    CAM.omega_BN_B_goal = [0, 0, 0]
    CAM.keepOutFov = np.pi/9
    CAM.keepOutBore_B = [1,0,0]
    CAM.ModelTag = "CAMTestModule"
    unitTestSim.AddModelToTask(unitTaskName, CAM)

    # connect messages
    SCStatesMsgData = messaging.SCStatesMsgPayload()
    SCStatesMsgData.r_BN_N = [1,0,0]
    SCStatesMsgData.sigma_BN = [0,1,0]
    SCStatesMsg = messaging.SCStatesMsg().write(SCStatesMsgData)
    PlanetStateMsgData = messaging.SpicePlanetStateMsgPayload()
    PlanetStateMsgData.PositionVector = [10,0,0]
    PlanetStateMsg = messaging.SpicePlanetStateMsg().write(PlanetStateMsgData)
    CAM.scStateInMsg.subscribeTo(SCStatesMsg)
    CAM.celBodyInMsg.subscribeTo(PlanetStateMsg)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()


    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    CAMTestFunction(
        11)
	   