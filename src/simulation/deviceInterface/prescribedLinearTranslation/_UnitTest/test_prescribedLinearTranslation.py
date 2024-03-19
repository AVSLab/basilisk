#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        prescribedLinearTranslation
#   Author:             Leah Kiner
#   Last Updated:       March 18, 2024
#

import inspect
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest

from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.simulation import prescribedLinearTranslation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


@pytest.mark.parametrize("coastOptionBangDuration", [0.0, 2.0])  # [s]
@pytest.mark.parametrize("smoothingDuration", [0.0, 2.0])  # [s]
@pytest.mark.parametrize("transPosInit", [0.0, -0.5])  # [m]
@pytest.mark.parametrize("transPosRef1", [0.0, -1.0])  # [m]
@pytest.mark.parametrize("transPosRef2", [0.5])  # [m]
@pytest.mark.parametrize("transAccelMax", [0.01, 0.005])  # [m/s^2]
@pytest.mark.parametrize("accuracy", [1e-8])
def test_prescribedLinearTranslation(show_plots,
                                     coastOptionBangDuration,
                                     smoothingDuration,
                                     transPosInit,
                                     transPosRef1,
                                     transPosRef2,
                                     transAccelMax,
                                     accuracy):
    r"""
    **Validation Test Description**

    The unit test for this module ensures that the profiled linear translation for a secondary rigid body relative to
    the spacecraft hub is properly computed for several different simulation configurations. This unit test profiles
    two successive translations to ensure the module is correctly configured. The secondary body's initial scalar
    translational position relative to the spacecraft hub is varied, along with the two final reference positions
    and the maximum translational acceleration.

    This unit test also tests four different methods of profiling the translation. Two profilers prescribe a pure
    bang-bang or bang-coast-bang linear acceleration profile for the translation. The bang-bang option results in
    the fastest possible translation; while the bang-coast-bang option includes a coast period with zero acceleration
    between the acceleration segments. The other two profilers apply smoothing to the bang-bang and bang-coast-bang
    acceleration profiles so that the secondary body hub-relative rates start and end at zero.

    **Test Parameters**

    Args:
        show_plots (bool): Variable for choosing whether plots should be displayed
        coastOptionBangDuration: (float): [s] Time the acceleration is applied during the bang segments
        (Variable must be nonzero to select the bang-coast-bang option)
        smoothingDuration (float) [s] Time the acceleration is smoothed to the given maximum acceleration value
        transPosInit (float): [m] Initial translational body position from M to F frame origin along transAxis_M
        transPosRef1 (float): [m] First reference position from M to F frame origin along transAxis_M
        transPosRef2 (float): [m] Second reference position from M to F frame origin along transAxis_M
        transAccelMax (float): [m/s^2] Maximum translational acceleration
        accuracy (float): Absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    To verify the module functionality, the final position at the end of each translation segment is checked to match
    the specified reference position (``transPosRef1`` and ``transPosRef2``). Additionally, for the smoothed profiler
    options, the numerical derivative of the profiled displacements and velocities is determined across the entire
    simulation in this test script. These numerical derivatives are checked with the module's acceleration and velocity
    profiles to ensure the profiled acceleration is correctly integrated in the module to obtain the displacements and
    velocities.
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

    # Create an instance of the prescribedLinearTranslation module to be tested
    transAxis_M = np.array([1.0, 0.0, 0.0])  # Axis of translation
    prescribedTrans = prescribedLinearTranslation.PrescribedLinearTranslation()
    prescribedTrans.ModelTag = "prescribedTrans"
    prescribedTrans.setCoastOptionBangDuration(coastOptionBangDuration)
    prescribedTrans.setSmoothingDuration(smoothingDuration)
    prescribedTrans.setTransHat_M(transAxis_M)
    prescribedTrans.setTransAccelMax(transAccelMax)
    prescribedTrans.setTransPosInit(transPosInit)
    unitTestSim.AddModelToTask(unitTaskName, prescribedTrans)

    # Create the reference position input message for the first translation
    linearTransRigidBodyMessageData = messaging.LinearTranslationRigidBodyMsgPayload()
    linearTransRigidBodyMessageData.rho = transPosRef1  # [m]
    linearTransRigidBodyMessageData.rhoDot = 0.0  # [m/s]
    linearTransRigidBodyMessage = messaging.LinearTranslationRigidBodyMsg().write(linearTransRigidBodyMessageData)
    prescribedTrans.linearTranslationRigidBodyInMsg.subscribeTo(linearTransRigidBodyMessage)

    # Log module data for module unit test validation
    prescribedStatesDataLog = prescribedTrans.prescribedTranslationOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, prescribedStatesDataLog)

    # Execute the first translation segment
    simTime = 5 * 60  # [s]
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
    unitTestSim.ExecuteSimulation()

    # Create the reference position input message for the second translation
    linearTransRigidBodyMessageData = messaging.LinearTranslationRigidBodyMsgPayload()
    linearTransRigidBodyMessageData.rho = transPosRef2  # [m]
    linearTransRigidBodyMessageData.rhoDot = 0.0  # [m/s]
    linearTransRigidBodyMessage = messaging.LinearTranslationRigidBodyMsg().write(linearTransRigidBodyMessageData)
    prescribedTrans.linearTranslationRigidBodyInMsg.subscribeTo(linearTransRigidBodyMessage)

    # Execute the second translation segment
    unitTestSim.ConfigureStopTime(macros.sec2nano(2 * simTime))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * prescribedStatesDataLog.times()  # [s]
    r_FM_M = prescribedStatesDataLog.r_FM_M  # [m]
    rPrime_FM_M = prescribedStatesDataLog.rPrime_FM_M  # [m/s]
    rPrimePrime_FM_M = prescribedStatesDataLog.rPrimePrime_FM_M  # [m/s^2]

    # Unit test validation 1: Check that the profiler converges to the required final positions
    tf_1_index = int(round(simTime / testTimeStepSec)) + 1
    transPosFinal1 = r_FM_M[tf_1_index].dot(transAxis_M)
    transPosFinal2 = r_FM_M[-1].dot(transAxis_M)
    transPosFinalList = [transPosFinal1, transPosFinal2]  # [m]
    transPosRefList = [transPosRef1, transPosRef2]  # [m]
    np.testing.assert_allclose(transPosRefList,
                               transPosFinalList,
                               atol=accuracy,
                               verbose=True)

    # Unit test validation 2: Numerically check that the profiled accelerations,
    # velocities, and displacements are correct
    if (smoothingDuration > 0.0):
        transAccel = rPrimePrime_FM_M.dot(transAxis_M)
        transVel = rPrime_FM_M.dot(transAxis_M)
        transPos = r_FM_M.dot(transAxis_M)
        transAccelNumerical = []
        transVelNumerical = []
        for i in range(len(timespan) - 1):
            # First order forward difference
            transAccelNumerical.append((transVel[i+1] - transVel[i]) / testTimeStepSec)
            transVelNumerical.append((transPos[i+1] - transPos[i]) / testTimeStepSec)

        np.testing.assert_allclose(transAccel[0:-1],
                                   transAccelNumerical,
                                   atol=1e-2,
                                   verbose=True)
        np.testing.assert_allclose(transVel[0:-1],
                                   transVelNumerical,
                                   atol=1e-2,
                                   verbose=True)
        if show_plots:
            # Plot the difference between the numerical and profiled results
            plt.figure()
            plt.clf()
            plt.plot(timespan[0:-1], transAccelNumerical - transAccel[0:-1], label=r'$\ddot{\rho}$')
            plt.plot(timespan[0:-1], transVelNumerical - transVel[0:-1], label=r"$\dot{\rho}$")
            plt.title(r'Profiled vs Numerical Difference', fontsize=14)
            plt.legend(loc='upper right', prop={'size': 12})
            plt.grid(True)

    if show_plots:
        # 1. Plot the scalar translational states
        # 1A. Plot transPos
        transPosInitPlotting = np.ones(len(timespan)) * transPosInit
        transPosRef1Plotting = np.ones(len(timespan)) * transPosRef1
        transPosRef2Plotting = np.ones(len(timespan)) * transPosRef2
        plt.figure()
        plt.clf()
        plt.plot(timespan, r_FM_M.dot(transAxis_M), label=r"$l$")
        plt.plot(timespan, transPosInitPlotting, '--', label=r'$\rho_{0}$')
        plt.plot(timespan, transPosRef1Plotting, '--', label=r'$\rho_{Ref_1}$')
        plt.plot(timespan, transPosRef2Plotting, '--', label=r'$\rho_{Ref_2}$')
        plt.title(r'Profiled Translational Position $\rho_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
        plt.ylabel('(m)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)

        # 1B. Plot transVel
        plt.figure()
        plt.clf()
        plt.plot(timespan, rPrime_FM_M.dot(transAxis_M), label=r"$\dot{\rho}$")
        plt.title(r'Profiled Translational Velocity $\dot{\rho}_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
        plt.ylabel('(m/s)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)

        # 1C. Plot transAccel
        plt.figure()
        plt.clf()
        plt.plot(timespan, rPrimePrime_FM_M.dot(transAxis_M), label=r"$\ddot{\rho}$")
        plt.title(r'Profiled Translational Acceleration $\ddot{\rho}_{\mathcal{F}/\mathcal{M}}$ ', fontsize=14)
        plt.ylabel('(m/s$^2$)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)

        # 2. Plot the prescribed translational states
        # 2A. Plot r_FM_M
        transPosRef1Plotting = np.ones(len(timespan)) * transPosRef1  # [m]
        transPosRef2Plotting = np.ones(len(timespan)) * transPosRef2  # [m]
        plt.figure()
        plt.clf()
        plt.plot(timespan, r_FM_M[:, 0], label=r'$r_{1}$')
        plt.plot(timespan, r_FM_M[:, 1], label=r'$r_{2}$')
        plt.plot(timespan, r_FM_M[:, 2], label=r'$r_{3}$')
        plt.plot(timespan, transPosRef1Plotting, '--', label=r'$\rho_{Ref_1}$')
        plt.plot(timespan, transPosRef2Plotting, '--', label=r'$\rho_{Ref_2}$')
        plt.title(r'${}^\mathcal{M} r_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
        plt.ylabel('(m)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='center left', prop={'size': 12})
        plt.grid(True)

        # 2B. Plot rPrime_FM_F
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

        # 2C. Plot rPrimePrime_FM_F
        plt.figure()
        plt.clf()
        plt.plot(timespan, rPrimePrime_FM_M[:, 0], label='1')
        plt.plot(timespan, rPrimePrime_FM_M[:, 1], label='2')
        plt.plot(timespan, rPrimePrime_FM_M[:, 2], label='3')
        plt.title(r'${}^\mathcal{M} r$PrimePrime$_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
        plt.ylabel('(m/s$^2$)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper left', prop={'size': 12})
        plt.grid(True)

        plt.show()
    plt.close("all")

if __name__ == "__main__":
    test_prescribedLinearTranslation(
        True,  # show_plots
        2.0,  # [s] coastOptionBangDuration
        2.0,  # [s] smoothingDuration
        -0.5,  # [m] transPosInit
        -1.0,  # [m] transPosRef1
        0.5,  # [m] transPosRef2
        0.01,  # [m/s^2] transAccelMax
        1e-8  # accuracy
    )
    