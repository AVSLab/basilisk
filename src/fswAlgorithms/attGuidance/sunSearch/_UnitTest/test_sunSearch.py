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
import inspect
import os

import numpy as np
import pytest
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


from Basilisk.utilities import SimulationBaseClass
from Basilisk.fswAlgorithms import sunSearch
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging


def computeKinematicProperties(theta_R, T_R, u_M, I, omega_M):

    alpha_M = u_M / I

    # Computing the fastest bang-bang slew with no coasting arc
    alpha = 4 * theta_R / T_R**2
    omega = 2 * theta_R / T_R
    T = T_R
    t_c = T_R / 2

    # If angular acceleration exceeds limit, decrease acceleration and increase slew time
    if alpha > alpha_M:
        alpha = alpha_M
        T = 2 * (theta_R / alpha)**0.5
        t_c = T / 2
        omega = alpha * t_c

    # If angular rate exceeds limit, increase slew time adding a coasting arc
    if omega > omega_M:
        omega = omega_M
        T = theta_R / omega + omega / alpha
        t_c = omega / alpha

    return alpha, omega, T, t_c


@pytest.mark.parametrize("axis1", [1, 2, 3])
@pytest.mark.parametrize("axis2", [1, 2, 3])
@pytest.mark.parametrize("axis3", [1, 2, 3])
@pytest.mark.parametrize("omega_BN_B", [[0, 0, 0], [0.01, -0.02, 0.03]])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_rateDamp(show_plots, axis1, axis2, axis3, omega_BN_B, accuracy):

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(1.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    theta1 = np.pi/2
    theta2 = np.pi
    theta3 = 2*np.pi
    T_R = 1
    u_M = 1
    omega_M = np.pi / 18

    # Construct algorithm and associated C++ container
    attGuidance = sunSearch.SunSearch()
    attGuidance.setSlewTime(T_R, T_R, T_R)
    attGuidance.setSlewAngle(theta1, theta2, theta3)
    attGuidance.setMaxRate(omega_M, omega_M, omega_M)
    attGuidance.setMaxTorque(u_M, u_M, u_M)
    attGuidance.setRotAxis(axis1, axis2, axis3)
    attGuidance.ModelTag = "sunSearch"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, attGuidance)

    # Initialize the test module configuration data
    # These will eventually become input messages

    # Create input navigation message
    NavAttMessageData = messaging.NavAttMsgPayload()
    NavAttMessageData.omega_BN_B = omega_BN_B
    NavAttMsg = messaging.NavAttMsg().write(NavAttMessageData)
    attGuidance.attNavInMsg.subscribeTo(NavAttMsg)

    I = [100, 200, 300]

    # Create input vehicle configuration message
    VehConfMessageData = messaging.VehicleConfigMsgPayload()
    VehConfMessageData.ISCPntB_B = [I[0],  0.0,  0.0,
                                     0.0, I[1],  0.0,
                                     0.0,  0.0, I[2]]
    VehConfMessage = messaging.VehicleConfigMsg().write(VehConfMessageData)
    attGuidance.vehConfigInMsg.subscribeTo(VehConfMessage)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = attGuidance.attGuidOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    dataLogC = attGuidance.attGuidOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLogC)

    alpha1, omega1, T1, tc1 = computeKinematicProperties(theta1, T_R, u_M, I[axis1-1], omega_M)
    alpha2, omega2, T2, tc2 = computeKinematicProperties(theta2, T_R, u_M, I[axis2-1], omega_M)
    alpha3, omega3, T3, tc3 = computeKinematicProperties(theta3, T_R, u_M, I[axis3-1], omega_M)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(T1+T2+T3))

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    time = dataLog.times() * macros.NANO2SEC
    omega_BR_B = dataLog.omega_BR_B
    omega_RN_B = dataLog.omega_RN_B
    omegaDot_RN_B = dataLog.domega_RN_B
    omegaC_BR_B = dataLogC.omega_BR_B
    omegaC_RN_B = dataLogC.omega_RN_B
    omegaDotC_RN_B = dataLogC.domega_RN_B

    timeVector = [0, tc1, T1-tc1, T1, T1+tc2, T1+T2-tc2, T1+T2, T1+T2+tc3, T1+T2+T3-tc3, T1+T2+T3]

    omega_BR_B_truth = np.zeros((len(time), 3))
    omega_RN_B_truth = np.zeros((len(time), 3))
    omegaDot_RN_B_truth = np.zeros((len(time), 3))
    for i in range(len(time)):
        t = time[i]
        if t < timeVector[1]:
            omega_RN_B_truth[i, axis1-1] = omega1 * t / tc1
            omegaDot_RN_B_truth[i, axis1-1] = alpha1
        elif t < timeVector[2]:
            omega_RN_B_truth[i, axis1-1] = omega1
        elif t < timeVector[3]:
            omega_RN_B_truth[i, axis1-1] = omega1 * (T1-t) / tc1
            omegaDot_RN_B_truth[i, axis1-1] = -alpha1
        elif t < timeVector[4]:
            omega_RN_B_truth[i, axis2-1] = omega2 * (t-T1) / tc2
            omegaDot_RN_B_truth[i, axis2-1] = alpha2
        elif t < timeVector[5]:
            omega_RN_B_truth[i, axis2-1] = omega2
        elif t < timeVector[6]:
            omega_RN_B_truth[i, axis2-1] = omega2 * (T1+T2-t) / tc2
            omegaDot_RN_B_truth[i, axis2-1] = -alpha2
        elif t < timeVector[7]:
            omega_RN_B_truth[i, axis3-1] = omega3 * (t-T1-T2) / tc3
            omegaDot_RN_B_truth[i, axis3-1] = alpha3
        elif t < timeVector[8]:
            omega_RN_B_truth[i, axis3-1] = omega3
        elif t < timeVector[9]:
            omega_RN_B_truth[i, axis3-1] = omega3 * (T1+T2+T3-t) / tc3
            omegaDot_RN_B_truth[i, axis3-1] = -alpha3
        omega_BR_B_truth[i] = omega_BN_B - omega_RN_B_truth[i]

    # set the filtered output truth states
    np.testing.assert_allclose(omega_BR_B, omega_BR_B_truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omega_RN_B, omega_RN_B_truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omegaDot_RN_B, omegaDot_RN_B_truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omegaC_BR_B, omega_BR_B_truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omegaC_RN_B, omega_RN_B_truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omegaDotC_RN_B, omegaDot_RN_B_truth, rtol=0, atol=accuracy, verbose=True)

    return



if __name__ == "__main__":
    test_rateDamp(False, 1, 2, 3, [0, 0, 0], 1e-12)
