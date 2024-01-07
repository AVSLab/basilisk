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
#   Module Name:        prescribedTrans
#   Author:             Leah Kiner
#   Creation Date:      Jan 2, 2023
#   Last Updated:       Jan 6, 2024

import inspect
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest

from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import prescribedTrans  # import the module that is to be tested
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("transPosInit", [0, -0.75])  # [m]
@pytest.mark.parametrize("transPosRef1", [0, -0.5])  # [m]
@pytest.mark.parametrize("transPosRef2", [-0.75, 1.0])  # [m]
@pytest.mark.parametrize("transAccelMax", [0.01, 0.005])  # [m/s^2]
@pytest.mark.parametrize("accuracy", [1e-12])
def test_prescribedTransTestFunction(show_plots,
                                     transPosInit,
                                     transPosRef1,
                                     transPosRef2,
                                     transAccelMax,
                                     accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that a profiled 1 DOF translation for a secondary rigid body connected to a spacecraft hub
    is properly computed for several different simulation configurations. This unit test profiles two successive
    translations to ensure the module is correctly configured. The body's initial scalar translational position
    relative to the spacecraft hub is varied, along with the two final reference positions and the maximum translational
    acceleration. A pure bang-bang acceleration profile is used for profiling the translation. To validate the module,
    the final position at the end of each translation is checked to match the specified reference position.

    **Test Parameters**

    Args:
        transPosInit (float): [m] Initial translational body position from M to F frame origin along transAxis_M
        transPosRef1 (float): [m] First reference position from M to F frame origin along transAxis_M
        transPosRef2 (float): [m] Second reference position from M to F frame origin along transAxis_M
        transAccelMax (float): [m/s^2] Maximum translational acceleration
        accuracy (float): Absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This unit test checks that the final translational body position at the end of each translation converge to the
    specified reference values ``transPosRef1`` and ``transPosRef2``.
    """
    [testResults, testMessage] = prescribedTransTestFunction(show_plots,
                                                             transPosInit,
                                                             transPosRef1,
                                                             transPosRef2,
                                                             transAccelMax,
                                                             accuracy)

    assert testResults < 1, testMessage


def prescribedTransTestFunction(show_plots,
                                transPosInit,
                                transPosRef1,
                                transPosRef2,
                                transAccelMax,
                                accuracy):
    """Call this routine directly to run the unit test."""
    testFailCount = 0                                        # Zero unit test result counter
    testMessages = []                                        # Create an empty array to store the test log messages
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testTimeStepSec = 0.01  # [s]
    testProcessRate = macros.sec2nano(testTimeStepSec)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the prescribedTrans module to be tested
    PrescribedTrans = prescribedTrans.prescribedTrans()
    PrescribedTrans.ModelTag = "prescribedTrans"
    transAxis_M = np.array([0.5, 0.0, 0.5 * np.sqrt(3)])
    PrescribedTrans.transAxis_M = transAxis_M
    PrescribedTrans.transAccelMax = transAccelMax  # [m/s^2]
    PrescribedTrans.omega_FM_F = np.array([0.0, 0.0, 0.0])  # [rad/s]
    PrescribedTrans.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])  # [rad/s^2]
    PrescribedTrans.sigma_FM = np.array([0.0, 0.0, 0.0])
    PrescribedTrans.transPosInit = transPosInit  # [m]

    # Add the prescribedTrans test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, PrescribedTrans)

    # Create the prescribedTrans input reference position message for the first translation
    PrescribedTransMessageData = messaging.PrescribedTransMsgPayload()
    PrescribedTransMessageData.scalarPos = transPosRef1
    PrescribedTransMessageData.scalarVel = 0.0
    PrescribedTransMessage = messaging.PrescribedTransMsg().write(PrescribedTransMessageData)
    PrescribedTrans.prescribedTransInMsg.subscribeTo(PrescribedTransMessage)

    # Log module data for module unit test validation
    prescribedStatesDataLog = PrescribedTrans.prescribedMotionOutMsg.recorder()
    transPosLog = PrescribedTrans.logger("transPos", testProcessRate)
    transVelLog = PrescribedTrans.logger("transVel", testProcessRate)
    transAccelLog = PrescribedTrans.logger("transAccel", testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, prescribedStatesDataLog)
    unitTestSim.AddModelToTask(unitTaskName, transPosLog)
    unitTestSim.AddModelToTask(unitTaskName, transVelLog)
    unitTestSim.AddModelToTask(unitTaskName, transAccelLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Determine the required simulation time for the first rotation
    translation1ReqTime = np.sqrt(((0.5 * np.abs(transPosRef1 - transPosInit)) * 8) / transAccelMax) + 5  # [s]

    translation1ExtraTime = 5  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(translation1ReqTime + translation1ExtraTime))

    # Execute the first translation
    unitTestSim.ExecuteSimulation()

    # Create the prescribedTrans input reference position message for the second translation
    PrescribedTransMessageData = messaging.PrescribedTransMsgPayload()
    PrescribedTransMessageData.scalarPos = transPosRef2
    PrescribedTransMessageData.scalarVel = 0.0
    PrescribedTransMessage = messaging.PrescribedTransMsg().write(PrescribedTransMessageData)
    PrescribedTrans.prescribedTransInMsg.subscribeTo(PrescribedTransMessage)

    # Determine the required simulation time for the second rotation
    translation2ReqTime = np.sqrt(((0.5 * np.abs(transPosRef2 - transPosRef1)) * 8) / transAccelMax) + 5  # [s]

    translation2ExtraTime = 5  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(translation1ReqTime
                                                  + translation1ExtraTime
                                                  + translation2ReqTime
                                                  + translation2ExtraTime))

    # Execute the second translation
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * prescribedStatesDataLog.times()  # [s]
    transPos = transPosLog.transPos  # [m]
    transVel = transVelLog.transVel  # [m/s]
    transAccel = transAccelLog.transAccel  # [m/s^2]
    r_FM_M = prescribedStatesDataLog.r_FM_M  # [m]
    rPrime_FM_M = prescribedStatesDataLog.rPrime_FM_M  # [m/s]
    rPrimePrime_FM_M = prescribedStatesDataLog.rPrimePrime_FM_M  # [m/s^2]

    # Unit test validation
    # Store the truth data used to validate the module in two lists
    # Compute tf for the first translation, and tInit tf for the second translation
    tf_1 = translation1ReqTime
    tInit_2 = translation1ReqTime + translation1ExtraTime
    tf_2 = tInit_2 + translation2ReqTime

    # Compute the timespan indices for each check
    tf_1_index = int(round(tf_1 / testTimeStepSec)) + 1
    tInit_2_index = int(round(tInit_2 / testTimeStepSec)) + 1
    tf_2_index = int(round(tf_2 / testTimeStepSec)) + 1

    # Store the timespan indices in a list
    timeCheckIndicesList = [tf_1_index,
                            tInit_2_index,
                            tf_2_index]

    # Store the positions to check in a list
    transPosCheckList = [transPosRef1, transPosRef1, transPosRef2]

    # Use the two truth data lists to compare with the module-extracted data
    for i in range(len(timeCheckIndicesList)):
        index = timeCheckIndicesList[i]
        if not unitTestSupport.isDoubleEqual(transPos[index], transPosCheckList[i], accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + PrescribedTrans.ModelTag + " TRANSLATING BODY POSITION CHECK FAILED")
            print("\nAt Simulation Time [s]: ")
            print(timespan[index])
            print("Module Computed Position [m]: ")
            print(transPos[index])
            print("Python Computed TRUTH Position [m]: ")
            print(transPosCheckList[i])

    # 1. Plot the scalar translational states
    # 1A. Plot transPos
    transPosInit_plotting = np.ones(len(timespan)) * transPosInit
    transPosRef1_plotting = np.ones(len(timespan)) * transPosRef1
    transPosRef2_plotting = np.ones(len(timespan)) * transPosRef2
    plt.figure()
    plt.clf()
    plt.plot(timespan, transPos, label=r"$l$")
    plt.plot(timespan, transPosInit_plotting, '--', label=r'$l_{0}$')
    plt.plot(timespan, transPosRef1_plotting, '--', label=r'$l_{Ref_1}$')
    plt.plot(timespan, transPosRef2_plotting, '--', label=r'$l_{Ref_2}$')
    plt.title(r'Translational Position $l_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(m)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # 1B. Plot transVel
    plt.figure()
    plt.clf()
    plt.plot(timespan, transVel, label=r"$\dot{l}$")
    plt.title(r'Translational Velocity $\dot{l}_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(m/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # 1C. Plot transAccel
    plt.figure()
    plt.clf()
    plt.plot(timespan, transAccel, label=r"$\ddot{l}$")
    plt.title(r'Translational Acceleration $\ddot{l}_{\mathcal{F}/\mathcal{M}}$ ', fontsize=14)
    plt.ylabel('(m/s$^2$)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # 2. Plot the prescribed translational states
    # 2A. Plot r_FM_M
    r_FM_M_Ref1 = transPosRef1 * transAxis_M
    r_FM_M_1_Ref1 = np.ones(len(timespan[0:timeCheckIndicesList[0]])) * r_FM_M_Ref1[0]
    r_FM_M_2_Ref1 = np.ones(len(timespan[0:timeCheckIndicesList[0]])) * r_FM_M_Ref1[1]
    r_FM_M_3_Ref1 = np.ones(len(timespan[0:timeCheckIndicesList[0]])) * r_FM_M_Ref1[2]
    r_FM_M_Ref2 = transPosRef2 * transAxis_M
    r_FM_M_1_Ref2 = np.ones(len(timespan[timeCheckIndicesList[0]:-1])) * r_FM_M_Ref2[0]
    r_FM_M_2_Ref2 = np.ones(len(timespan[timeCheckIndicesList[0]:-1])) * r_FM_M_Ref2[1]
    r_FM_M_3_Ref2 = np.ones(len(timespan[timeCheckIndicesList[0]:-1])) * r_FM_M_Ref2[2]
    plt.figure()
    plt.clf()
    plt.plot(timespan, r_FM_M[:, 0], label=r'$r_{1}$')
    plt.plot(timespan, r_FM_M[:, 1], label=r'$r_{2}$')
    plt.plot(timespan, r_FM_M[:, 2], label=r'$r_{3}$')
    plt.plot(timespan[0:timeCheckIndicesList[0]], r_FM_M_1_Ref1, '--', label=r'$r_{1 Ref_{1}}$')
    plt.plot(timespan[0:timeCheckIndicesList[0]], r_FM_M_2_Ref1, '--', label=r'$r_{2 Ref_{1}}$')
    plt.plot(timespan[0:timeCheckIndicesList[0]], r_FM_M_3_Ref1, '--', label=r'$r_{3 Ref_{1}}$')
    plt.plot(timespan[timeCheckIndicesList[0]:-1], r_FM_M_1_Ref2, '--', label=r'$r_{1 Ref_{2}}$')
    plt.plot(timespan[timeCheckIndicesList[0]:-1], r_FM_M_2_Ref2, '--', label=r'$r_{2 Ref_{2}}$')
    plt.plot(timespan[timeCheckIndicesList[0]:-1], r_FM_M_3_Ref2, '--', label=r'$r_{3 Ref_{2}}$')
    plt.title(r'${}^\mathcal{M} r_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
    plt.ylabel('(m)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='center left', prop={'size': 12})
    plt.grid(True)

    # Plot rPrime_FM_F
    plt.figure()
    plt.clf()
    plt.plot(timespan, rPrime_FM_M[:, 0], label='1')
    plt.plot(timespan, rPrime_FM_M[:, 1], label='2')
    plt.plot(timespan, rPrime_FM_M[:, 2], label='3')
    plt.title(r'${}^\mathcal{M} r$Prime$_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
    plt.ylabel('(m/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper left', prop={'size': 12})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    prescribedTransTestFunction(
        True,       # show_plots
        0.0,        # [m] transPosInit
        -0.5,                  # [m] transPosRef1
        -0.75,                 # [m] transPosRef2
        0.01,    # [m/s^2] transAccelMax
        1e-12        # accuracy
    )
