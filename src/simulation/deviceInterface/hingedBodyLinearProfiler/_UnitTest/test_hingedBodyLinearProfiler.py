# 
#  ISC License
# 
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

from math import pi

import numpy as np
import pytest
from Basilisk.simulation import hingedBodyLinearProfiler
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport


@pytest.mark.parametrize("startTime, endTime, startTheta, endTheta", [
     (macros.sec2nano(1), macros.sec2nano(2), 0, pi/180)
])

def test_hingedBodyLinearProfiler(show_plots, startTime, endTime, startTheta, endTheta):
    r"""
    **Validation Test Description**

    For a given deployment, checks that the theta before, during, and after deployment are correct.

    **Test Parameters**

    Discuss the test parameters used.

    Args:
        startTime (uint64_t): starting time in nanoseconds
        endTime (uint64_t): ending time in nanoseconds
        startTheta (double): starting angle of deployment in radians
        endTheta (double): ending angle of deployment in radians

    **Description of Variables Being Tested**
    
    For a deployment from 0 to 1 degree, starting at 1 second and ending at 2 seconds into the simulation, checks that the angle and angle rates are as expected before, during, and after deployment.
    
    Before deployment, theta should be 0 and ``thetaDot`` 0. During deployment, ``thetaDot`` should be 1 degree per second, with theta varying linearly. After deployment, theta should be 1 degree and ``thetaDot`` 0.

    """
    [testResults, testMessage] = hingedBodyLinearProfilerTestFunction(show_plots, startTime, endTime, startTheta, endTheta)
    assert testResults < 1, testMessage


def hingedBodyLinearProfilerTestFunction(show_plots, startTime, endTime, startTheta, endTheta):
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    
    # accuracy to which double values compared
    accuracy = 1e-12

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = hingedBodyLinearProfiler.HingedBodyLinearProfiler()
    module.ModelTag = "hingedBodyLinearProfilerTag"
    unitTestSim.AddModelToTask(unitTaskName, module)
    module.startTime = startTime
    module.endTime = endTime
    module.startTheta = startTheta
    module.endTheta = endTheta

    # set up output message recorder objects
    dataLog = module.hingedRigidBodyReferenceOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(3))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    trueTheta = np.array([0, 0, 0, pi/360, pi/180, pi/180, pi/180]);
    trueThetaDot = np.array([0, 0, pi/180, pi/180, pi/180, 0, 0])
    
    testFailCount, testMessages = unitTestSupport.compareVector(trueTheta, dataLog.theta, accuracy, "theta", testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareVector(trueThetaDot, dataLog.thetaDot, accuracy, "thetaDot", testFailCount, testMessages)

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_hingedBodyLinearProfiler(False, macros.sec2nano(1), macros.sec2nano(2), 0, pi/180)


