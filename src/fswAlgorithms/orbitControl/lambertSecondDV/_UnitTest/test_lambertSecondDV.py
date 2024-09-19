#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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
import itertools

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import lambertSecondDV
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# parameters
DVs = np.array([[1., 3., 4.], [500., -100., 200.]])
time_maneuver = [1.e3, 2.e3]
validFlag = [1]

paramArray = [DVs, time_maneuver, validFlag]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))
# add a case where lambert valid flag is false (0) to make sure module returns zeroed DV
paramList.append((DVs[0], 1.e3, 0))

@pytest.mark.parametrize("accuracy", [1e-4])
@pytest.mark.parametrize("p1_dv, p2_tm, p3_valid", paramList)

def test_lambertSecondDV(show_plots, p1_dv, p2_tm, p3_valid, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the Lambert Second DV module works correctly for different Delta-V vectors and maneuver times.
    If the Lambert solution valid flag is false (0), the commanded Delta-V should be zero.

    **Test Parameters**

    Args:
        show_plots: flag if plots should be shown
        p1_dv: Delta-V vector
        p2_tm: maneuver time
        p3_valid: Lambert solution valid flag
        accuracy: accuracy of the test

    **Description of Variables Being Tested**

    The content of the DvBurnCmdMsg output message (DV vector and burn start time) is compared with the true values.
    """
    lambertSecondDVTestFunction(show_plots, p1_dv, p2_tm, p3_valid, accuracy)


def lambertSecondDVTestFunction(show_plots, p1_dv, p2_tm, p3_valid, accuracy):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(100)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    dv_N = p1_dv
    tm = p2_tm
    validLambert = p3_valid
    muBody = 3.986004418e14
    ecc = 0.1

    # set up orbit using classical orbit elements
    oe1 = orbitalMotion.ClassicElements()
    r = 10000. * 1000  # meters
    if ecc < 1.0:
        # elliptic case
        oe1.a = r
    else:
        # parabolic and hyperbolic case
        oe1.a = -r
    oe1.e = ecc
    oe1.i = 10. * macros.D2R
    oe1.Omega = 20. * macros.D2R
    oe1.omega = 30. * macros.D2R
    oe1.f = 40. * macros.D2R
    # spacecraft state at maneuver time, just before maneuver
    r1_BN_N, v1_BN_N = orbitalMotion.elem2rv_parab(muBody, oe1)

    # spacecraft state at maneuver time, just after maneuver
    v2_BN_N = v1_BN_N + dv_N

    # setup module to be tested
    module = lambertSecondDV.LambertSecondDV()
    module.ModelTag = "lambertSecondDV"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure input messages
    lambertSolutionInMsgData = messaging.LambertSolutionMsgPayload()
    lambertSolutionInMsgData.v2_N = v1_BN_N
    lambertSolutionInMsgData.valid = validLambert
    lambertSolutionInMsg = messaging.LambertSolutionMsg().write(lambertSolutionInMsgData)

    desiredVelocityInMsgData = messaging.DesiredVelocityMsgPayload()
    desiredVelocityInMsgData.vDesired_N = v2_BN_N
    desiredVelocityInMsgData.maneuverTime = tm
    desiredVelocityInMsg = messaging.DesiredVelocityMsg().write(desiredVelocityInMsgData)

    # subscribe input messages to module
    module.lambertSolutionInMsg.subscribeTo(lambertSolutionInMsg)
    module.desiredVelocityInMsg.subscribeTo(desiredVelocityInMsg)

    # setup output message recorder objects
    dvBurnCmdOutMsgRec = module.dvBurnCmdOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dvBurnCmdOutMsgRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    dv = dvBurnCmdOutMsgRec.dvInrtlCmd[0]  # commanded Delta-V
    burnStartTime = dvBurnCmdOutMsgRec.burnStartTime[0]

    if validLambert == 1:
        dvTrue = dv_N
        burnStartTimeTrue = macros.sec2nano(tm)
    else:
        dvTrue = np.array([0., 0., 0.])
        burnStartTimeTrue = 0

    # make sure module output data is correct
    paramsString = ' for DV={}, maneuver time={}, valid flag={}, accuracy={}'.format(
        str(p1_dv),
        str(p2_tm),
        str(p3_valid),
        str(accuracy))

    np.testing.assert_allclose(dv,
                               dvTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: dv_N,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(burnStartTime,
                               burnStartTimeTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: burnStartTime,' + paramsString),
                               verbose=True)


if __name__ == "__main__":
    test_lambertSecondDV(False, DVs[1], time_maneuver[0], 1, 1e-4)
