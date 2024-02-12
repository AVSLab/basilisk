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
#   Author:             Patrick Kenneally and Leah Kiner
#   Creation Date:      Feb 12, 2024
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


@pytest.mark.parametrize("coastOptionRampDuration", [0.0, 2.0, 5.0])  # [s]
@pytest.mark.parametrize("transPosInit", [0, -0.75])  # [m]
@pytest.mark.parametrize("transPosRef1", [0, -0.5])  # [m]
@pytest.mark.parametrize("transPosRef2", [-0.75, 1.0])  # [m]
@pytest.mark.parametrize("transAccelMax", [0.01, 0.005])  # [m/s^2]
@pytest.mark.parametrize("accuracy", [1e-4])
def test_prescribedLinearTranslation(show_plots,
                                     coastOptionRampDuration,
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
    acceleration. This unit test also tests both methods of profiling the translation, where either a pure bang-bang
    acceleration profile can be selected for the translation, or a coast option can be selected where the accelerations
    are only applied for a specified ramp time and a coast segment with zero acceleration is applied between the two
    acceleration periods. To validate the module, the final hub-relative position at the end of each translation is
    checked to match the specified reference position.

    **Test Parameters**

    Args:
        show_plots (bool): Variable for choosing whether plots should be displayed
        coastOptionRampDuration: (double): [s] Ramp duration used for the coast option
        transPosInit (float): [m] Initial translational body position from M to F frame origin along transHat_M
        transPosRef1 (float): [m] First reference position from M to F frame origin along transHat_M
        transPosRef2 (float): [m] Second reference position from M to F frame origin along transHat_M
        transAccelMax (float): [m/s^2] Maximum translational acceleration
        accuracy (float): Absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This unit test checks that the final translational body position at the end of each translation converge to the
    specified reference values ``transPosRef1`` and ``transPosRef2``.
    """

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

    # Create an instance of the prescribedLinearTranslation module to be tested
    prescribedTrans = prescribedLinearTranslation.PrescribedLinearTranslation()
    prescribedTrans.ModelTag = "prescribedTrans"
    transHat_M = np.array([0.5, 0.0, 0.5 * np.sqrt(3)])
    prescribedTrans.setCoastOptionRampDuration(coastOptionRampDuration)
    prescribedTrans.setTransHat_M(transHat_M)
    prescribedTrans.setTransAccelMax(transAccelMax)  # [m/s^2]
    prescribedTrans.setTransPosInit(transPosInit)  # [m]

    # Add the prescribedTrans test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, prescribedTrans)

    # Create the input reference position message for the first translation
    linearTranslationRigidBodyMessageData = messaging.LinearTranslationRigidBodyMsgPayload()
    linearTranslationRigidBodyMessageData.rho = transPosRef1
    linearTranslationRigidBodyMessageData.rhoDot = 0.0
    linearTranslationRigidBodyMessage = messaging.LinearTranslationRigidBodyMsg().write(linearTranslationRigidBodyMessageData)
    prescribedTrans.linearTranslationRigidBodyInMsg.subscribeTo(linearTranslationRigidBodyMessage)

    # Log module data for module unit test validation
    prescribedStatesDataLog = prescribedTrans.prescribedTranslationOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, prescribedStatesDataLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Determine the required simulation time for the first translation
    transVelInit = 0.0  # [m/s]
    tCoast_1 = 0.0  # [s]
    if coastOptionRampDuration > 0.0:
        # Determine the position and velocity at the end of the ramp segment/start of the coast segment
        if transPosInit < transPosRef1:
            transPos_tr_1 = ((0.5 * transAccelMax * coastOptionRampDuration * coastOptionRampDuration)
                             + (transVelInit * coastOptionRampDuration)
                             + transPosInit)  # [m]
            transVel_tr_1 = transAccelMax * coastOptionRampDuration + transVelInit  # [m/s]
        else:
            transPos_tr_1 = (- ((0.5 * transAccelMax * coastOptionRampDuration * coastOptionRampDuration)
                                + (transVelInit * coastOptionRampDuration))
                             + transPosInit)  # [m]
            transVel_tr_1 = - transAccelMax * coastOptionRampDuration + transVelInit  # [m/s]

        # Determine the distance traveled during the coast period
        deltaPosCoast_1 = transPosRef1 - transPosInit - 2 * (transPos_tr_1 - transPosInit)  # [m]

        # Determine the time duration of the coast segment
        tCoast_1 = np.abs(deltaPosCoast_1) / np.abs(transVel_tr_1)  # [s]
        translation1ReqTime = (2 * coastOptionRampDuration) + tCoast_1  # [s]
    else:
        translation1ReqTime = np.sqrt(((0.5 * np.abs(transPosRef1 - transPosInit)) * 8) / transAccelMax) + 5  # [s]

    translation1ExtraTime = 5  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(translation1ReqTime + translation1ExtraTime))

    # Execute the first translation
    unitTestSim.ExecuteSimulation()

    # Create the input reference position message for the second translation
    linearTranslationRigidBodyMessageData = messaging.LinearTranslationRigidBodyMsgPayload()
    linearTranslationRigidBodyMessageData.rho = transPosRef2
    linearTranslationRigidBodyMessageData.rhoDot = 0.0
    linearTranslationRigidBodyMessage = messaging.LinearTranslationRigidBodyMsg().write(linearTranslationRigidBodyMessageData)
    prescribedTrans.linearTranslationRigidBodyInMsg.subscribeTo(linearTranslationRigidBodyMessage)

    # Determine the required simulation time for the second translation
    tCoast_2 = 0.0  # [s]
    if coastOptionRampDuration > 0.0:
        # Determine the position and velocity at the end of the ramp segment/start of the coast segment
        if transPosRef1 < transPosRef2:
            transPos_tr_2 = ((0.5 * transAccelMax * coastOptionRampDuration * coastOptionRampDuration)
                             + (transVelInit * coastOptionRampDuration)
                             + transPosRef1)  # [m]
            transVel_tr_2 = transAccelMax * coastOptionRampDuration + transVelInit  # [m/s]
        else:
            transPos_tr_2 = (- ((0.5 * transAccelMax * coastOptionRampDuration * coastOptionRampDuration)
                             + (transVelInit * coastOptionRampDuration))
                             + transPosRef1)  # [m]
            transVel_tr_2 = - transAccelMax * coastOptionRampDuration + transVelInit  # [m/s]

        # Determine the distance traveled during the coast period
        deltaPosCoast_2 = transPosRef2 - transPosRef1 - 2 * (transPos_tr_2 - transPosRef1)  # [m]

        # Determine the time duration of the coast segment
        tCoast_2 = np.abs(deltaPosCoast_2) / np.abs(transVel_tr_2)  # [s]
        translation2ReqTime = (2 * coastOptionRampDuration) + tCoast_2  # [s]
    else:
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
    r_FM_M = prescribedStatesDataLog.r_FM_M  # [m]
    rPrime_FM_M = prescribedStatesDataLog.rPrime_FM_M  # [m/s]
    rPrimePrime_FM_M = prescribedStatesDataLog.rPrimePrime_FM_M  # [m/s^2]

    # Unit test validation
    # Store the truth data used to validate the module in two lists
    if coastOptionRampDuration > 0.0:
        # Compute tf for the first translation, and tInit tf for the second translation
        tf_1 = 2 * coastOptionRampDuration + tCoast_1  # [s]
        tInit_2 = translation1ReqTime + translation1ExtraTime  # [s]
        tf_2 = tInit_2 + (2 * coastOptionRampDuration) + tCoast_2  # [s]

        # Compute the timespan indices for each check
        tf_1_index = int(round(tf_1 / testTimeStepSec)) + 1
        tInit_2_index = int(round(tInit_2 / testTimeStepSec)) + 1
        tf_2_index = int(round(tf_2 / testTimeStepSec)) + 1

        # Store the timespan indices in a list
        timeCheckIndicesList = [tf_1_index, tInit_2_index, tf_2_index]

        # Store the positions to check in a list
        transPosCheckList = [transPosRef1, transPosRef1, transPosRef2]

    else:
        # Compute tf for the first translation, and tInit tf for the second translation
        tf_1 = translation1ReqTime  # [s]
        tInit_2 = translation1ReqTime + translation1ExtraTime  # [s]
        tf_2 = tInit_2 + translation2ReqTime  # [s]

        # Compute the timespan indices for each check
        tf_1_index = int(round(tf_1 / testTimeStepSec)) + 1
        tInit_2_index = int(round(tInit_2 / testTimeStepSec)) + 1
        tf_2_index = int(round(tf_2 / testTimeStepSec)) + 1

        # Store the timespan indices in a list
        timeCheckIndicesList = [tf_1_index, tInit_2_index, tf_2_index]

        # Store the positions to check in a list
        transPosCheckList = [transPosRef1, transPosRef1, transPosRef2]

    # Use the two truth data lists to compare with the module-extracted data
    np.testing.assert_allclose(r_FM_M.dot(transHat_M)[timeCheckIndicesList],
                               transPosCheckList,
                               atol=accuracy,
                               verbose=True)

    # 1. Plot the scalar translational states
    # 1A. Plot transPos
    transPosInitPlotting = np.ones(len(timespan)) * transPosInit
    transPosRef1Plotting = np.ones(len(timespan)) * transPosRef1
    transPosRef2Plotting = np.ones(len(timespan)) * transPosRef2
    plt.figure()
    plt.clf()
    plt.plot(timespan, r_FM_M.dot(transHat_M), label=r"$l$")
    plt.plot(timespan, transPosInitPlotting, '--', label=r'$\rho_{0}$')
    plt.plot(timespan, transPosRef1Plotting, '--', label=r'$\rho_{Ref_1}$')
    plt.plot(timespan, transPosRef2Plotting, '--', label=r'$\rho_{Ref_2}$')
    plt.title(r'Translational Position $\rho_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(m)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # 1B. Plot transVel
    plt.figure()
    plt.clf()
    plt.plot(timespan, rPrime_FM_M.dot(transHat_M), label=r"$\dot{\rho}$")
    plt.title(r'Translational Velocity $\dot{\rho}_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(m/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # 1C. Plot transAccel
    plt.figure()
    plt.clf()
    plt.plot(timespan, rPrimePrime_FM_M.dot(transHat_M), label=r"$\ddot{\rho}$")
    plt.title(r'Translational Acceleration $\ddot{\rho}_{\mathcal{F}/\mathcal{M}}$ ', fontsize=14)
    plt.ylabel('(m/s$^2$)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # 2. Plot the prescribed translational states
    # 2A. Plot r_FM_M
    r_FM_M_Ref1 = transPosRef1 * transHat_M
    r_FM_M_1_Ref1 = np.ones(len(timespan[0:timeCheckIndicesList[0]])) * r_FM_M_Ref1[0]
    r_FM_M_2_Ref1 = np.ones(len(timespan[0:timeCheckIndicesList[0]])) * r_FM_M_Ref1[1]
    r_FM_M_3_Ref1 = np.ones(len(timespan[0:timeCheckIndicesList[0]])) * r_FM_M_Ref1[2]
    r_FM_M_Ref2 = transPosRef2 * transHat_M
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


if __name__ == "__main__":
    test_prescribedLinearTranslation(True,  # show_plots
                               2.0,
                               0.0,  # [m] transPosInit
                               -0.5,  # [m] transPosRef1
                               1.0,  # [m] transPosRef2
                               0.01,  # [m/s^2] transAccelMax
                               1e-4  # accuracy
                               )
