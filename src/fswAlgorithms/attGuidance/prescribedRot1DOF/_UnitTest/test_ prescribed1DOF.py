#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        prescribedRot1DOF
#   Author:             Leah Kiner
#   Creation Date:      Nov 14, 2022
#

import pytest
import os, inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the required module for testing the prescribedRot1DOF module
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import prescribedRot1DOF                # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging

# Use LaTeX for plotting
plt.rcParams['text.usetex'] = True

# Vary the initial angle, reference angle, and maximum angular acceleration for pytest
@pytest.mark.parametrize("thetaInit", [0, 2*np.pi/3])
@pytest.mark.parametrize("thetaRef", [0, 2*np.pi/3])
@pytest.mark.parametrize("thetaDDotMax", [0.008, 0.1])
@pytest.mark.parametrize("accuracy", [1e-12])


def test_prescribedRot1DOFTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy):

    # Each test method requires a single assert method to be called
    [testResults, testMessage] = prescribedRot1DOFTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy)

    assert testResults < 1, testMessage


def prescribedRot1DOFTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy):

    testFailCount = 0                                        # Zero the unit test result counter
    testMessages = []                                        # Create an empty array to store the test log messages
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(0.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the prescribedRot1DOF module to be tested
    PrescribedRot1DOFConfig = prescribedRot1DOF.PrescribedRot1DOFConfig()
    PrescribedWrap = unitTestSim.setModelDataWrap(PrescribedRot1DOFConfig)
    PrescribedWrap.ModelTag = "prescribedRot1DOF"

    # Add the prescribedRot1DOF test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, PrescribedWrap, PrescribedRot1DOFConfig)

    # Initialize the prescribedRot1DOF test module configuration data
    rotAxisM = np.array([1.0, 0.0, 0.0])
    prvInit_FM = thetaInit * rotAxisM
    PrescribedRot1DOFConfig.r_FM_M = np.array([1.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.rotAxis_M = rotAxisM
    PrescribedRot1DOFConfig.thetaDDotMax = thetaDDotMax
    PrescribedRot1DOFConfig.omega_FM_F = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.sigma_FM = rbk.PRV2MRP(prvInit_FM)

    # Create the prescribedRot1DOF input message
    thetaDotRef = 0.0  # [rad/s]
    SpinningBodyMessageData = messaging.SpinningBodyMsgPayload()
    SpinningBodyMessageData.theta = thetaRef
    SpinningBodyMessageData.thetaDot = thetaDotRef
    SpinningBodyMessage = messaging.SpinningBodyMsg().write(SpinningBodyMessageData)
    PrescribedRot1DOFConfig.spinningBodyInMsg.subscribeTo(SpinningBodyMessage)

    # Log the test module output message for data comparison
    dataLog = PrescribedRot1DOFConfig.prescribedMotionOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Set the simulation time
    simTime = np.sqrt(((0.5 * np.abs(thetaRef - thetaInit)) * 8) / thetaDDotMax) + 1
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    omega_FM_F = dataLog.omega_FM_F
    sigma_FM = dataLog.sigma_FM
    timespan = dataLog.times()

    thetaDot_Final = np.linalg.norm(omega_FM_F[-1, :])
    sigma_FM_Final = sigma_FM[-1, :]
    theta_FM_Final = 4 * np.arctan(np.linalg.norm(sigma_FM_Final))

    # Convert the logged sigma_FM MRPs to a scalar theta_FM array
    n = len(timespan)
    theta_FM = []
    for i in range(n):
        theta_FM.append(4 * np.arctan(np.linalg.norm(sigma_FM[i, :])))

    # Plot omega_FB_F
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, omega_FM_F[:, 0], label=r'${}^F \omega_{1_{\mathcal{F}/\mathcal{M}}}$')
    plt.plot(timespan * 1e-9, omega_FM_F[:, 1], label=r'${}^F \omega_{2_{\mathcal{F}/\mathcal{M}}}$')
    plt.plot(timespan * 1e-9, omega_FM_F[:, 2], label=r'${}^F \omega_{3_{\mathcal{F}/\mathcal{M}}}$')
    plt.title(r'${}^F \omega_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory')
    plt.ylabel(r'(rad/s)')
    plt.xlabel(r'Time (s)')
    plt.legend(loc='upper right', prop={'size': 12})

    # Plot theta_FM
    thetaRef_plotting = np.ones(len(timespan)) * thetaRef
    thetaInit_plotting = np.ones(len(timespan)) * thetaInit
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, theta_FM, label=r'$\theta_{\mathcal{F}/\mathcal{M}}$')
    plt.plot(timespan * 1e-9, thetaRef_plotting, '--', label=r'$\theta_{Ref}$')
    plt.plot(timespan * 1e-9, thetaInit_plotting, '--', label=r'$\theta_{0}$')
    plt.title(r'$\theta_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory')
    plt.ylabel(r'(rad)')
    plt.xlabel(r'Time (s)')
    plt.legend(loc='upper right', prop={'size': 12})

    if show_plots:
        plt.show()
    plt.close("all")

    # Check to ensure the initial angle rate converged to the reference angle rate
    if not unitTestSupport.isDoubleEqual(thetaDot_Final, thetaDotRef, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + PrescribedWrap.ModelTag + "thetaDot_Final and thetaDotRef do not match")

    # Check to ensure the initial angle converged to the reference angle
    if not unitTestSupport.isDoubleEqual(theta_FM_Final, thetaRef, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + PrescribedWrap.ModelTag + "theta_FM_Final and thetaRef do not match")
        #testMessages.append("theta_FM_Final: " + str(theta_FM_Final) + " thetaRef: " + str(thetaRef))
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    prescribedRot1DOFTestFunction(
                 True,
                 np.pi/6,     # thetaInit
                 2*np.pi/3,     # thetaRef
                 0.008,       # thetaDDotMax
                 1e-12        # accuracy
               )
