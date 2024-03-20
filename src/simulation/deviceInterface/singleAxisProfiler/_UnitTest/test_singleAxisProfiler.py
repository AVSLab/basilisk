#
#  ISC License
#
#  Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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
#   Module Name:        singleAxisProfiler
#   Author:             Leah Kiner
#   Created:            March 20, 2024
#

import inspect
import matplotlib.pyplot as plt
import numpy as np
import os
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.simulation import singleAxisProfiler
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("theta", [0.0 * macros.D2R, 5.2 * macros.D2R, -10.1 * macros.D2R]) # [rad]
@pytest.mark.parametrize("thetaDot", [0.0 * macros.D2R, 0.1 * macros.D2R, -0.1 * macros.D2R])  # [rad/s]
@pytest.mark.parametrize("thetaDDot", [0.0 * macros.D2R, 0.1 * macros.D2R, -0.1 * macros.D2R])  # [rad/s^2]
@pytest.mark.parametrize("rotAxis_MAngle", [0.0 * macros.D2R,  -5.2 * macros.D2R, 10.1 * macros.D2R])  # [rad]
@pytest.mark.parametrize("accuracy", [1e-12])
def test_singleAxisProfiler(show_plots, theta, thetaDot, thetaDDot, rotAxis_MAngle, accuracy):
    r"""
    **Validation Test Description**

    The unit test for this module ensures that the single-axis hub-relative rotational scalar states ``theta``,
    ``thetaDot``, and ``thetaDDot`` are properly converted to prescribed hub-relative rotational states ``sigma_FM``,
    ``omega_FM_F``, and ``omegaPrime_FM_F``. The scalar states are varied in this test, along with a single angle
    ``rotAxis_MAngle`` that is used to vary the axis of rotation ``rotAxis_M`` associated with the given
    scalar information. To verify the module, the final prescribed rotational states are logged from the module. The
    final attitude is checked with the computed true attitude. The dot product between the given rotation axis and
    the final angular velocity and angular acceleration is checked to match the given scalar angle rate and angle
    acceleration.

    **Test Parameters**

    Args:
        show_plots (bool): Variable for choosing whether plots should be displayed
        theta (float): [rad] Current hub-relative scalar angle about the given rotation axis rotAxis_M
        thetaDot (float): [rad/s] Current hub-relative scalar angle rate about the given rotation axis rotAxis_M
        thetaDDot (float): [rad/s^2] Current hub-relative scalar angular acceleration about the given rotation axis rotAxis_M
        accuracy (float): Absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This unit test checks that the prescribed rotational module output states ``sigma_FM``, ``omega_FM_F``,
    and ``omegaPrime_FM_F`` are correctly determined. The output state ``sigma_FM`` is checked to match the initial
    attitude. The dot product between the given rotation axis and the module output states ``omega_FM_F`` and
    ``omegaPrime_FM_F`` is checked to match the given scalar states ``thetaDot`` and ``thetaDDot``.
    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testTimeStepSec = 0.1  # [s]
    testProcessRate = macros.sec2nano(testTimeStepSec)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the singleAxisProfiler module to be tested
    rotAxis_M = np.array([np.cos(rotAxis_MAngle), np.sin(rotAxis_MAngle), 0.0])
    singleAxisRotProfiler = singleAxisProfiler.SingleAxisProfiler()
    singleAxisRotProfiler.ModelTag = "singleAxisProfiler"
    singleAxisRotProfiler.setRotHat_M(rotAxis_M)
    unitTestSim.AddModelToTask(unitTaskName, singleAxisRotProfiler)

    # Create the stepper motor input message
    stepperMotorMessageData = messaging.StepperMotorMsgPayload()
    stepperMotorMessageData.theta = theta  # [rad]
    stepperMotorMessageData.thetaDot = thetaDot  # [rad/s]
    stepperMotorMessageData.thetaDDot = thetaDDot  # [rad/s^2]
    stepperMotorMessage = messaging.StepperMotorMsg().write(stepperMotorMessageData)
    singleAxisRotProfiler.stepperMotorInMsg.subscribeTo(stepperMotorMessage)

    # Log module data for module unit test validation
    prescribedStatesDataLog = singleAxisRotProfiler.prescribedRotationOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, prescribedStatesDataLog)

    # Execute the first translation segment
    simTime = 1.0  # [s]
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    sigma_FM = prescribedStatesDataLog.sigma_FM
    omega_FM_F = prescribedStatesDataLog.omega_FM_F  # [rad/s]
    omegaPrime_FM_F = prescribedStatesDataLog.omegaPrime_FM_F  # [rad/s^2]

    # Unit test validation: Check that the module prescribed rotational output states are correct
    sigma_FMTrue = [0.0, 0.0, 0.0]
    if theta != 0.0:
        sigma_FMTrue = rbk.PRV2MRP(theta * rotAxis_M)
    np.testing.assert_allclose(sigma_FMTrue,
                               sigma_FM[-1],
                               atol=accuracy,
                               verbose=True)

    thetaDotCheck = omega_FM_F[-1].dot(rotAxis_M)  # [rad/s]
    np.testing.assert_allclose(thetaDot,
                               thetaDotCheck,
                               atol=accuracy,
                               verbose=True)

    thetaDDotCheck = omegaPrime_FM_F[-1].dot(rotAxis_M)  # [rad/s^2]
    np.testing.assert_allclose(thetaDDot,
                               thetaDDotCheck,
                               atol=accuracy,
                               verbose=True)

if __name__ == "__main__":
    test_singleAxisProfiler(
        True,  # show_plots
        5.2 * macros.D2R,  # [rad] theta
        0.5 * macros.D2R,  # [rad/s] thetaDot
        0.1 * macros.D2R,  # [rad/s^2] thetaDDot
        -10.0 * macros.D2R,   # [rad] rotAxis_MAngle
        1e-12  # accuracy
    )
    