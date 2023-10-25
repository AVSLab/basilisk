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
#   Module Name:        flybyPoint
#   Author:             Riccardo Calaon
#   Creation Date:      May 26, 2023
#

import os

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import flybyPoint
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


@pytest.mark.parametrize("initPos", [[-5e7, 7.5e6, 5e5]])  # m - r_CN_N
@pytest.mark.parametrize("initVel", [[2e4, 0, 0]])  # m/s - v_CN_N
@pytest.mark.parametrize("dTsim", [10, 100])  # s
@pytest.mark.parametrize("dTfilter", [0, 60, 600])  # s
@pytest.mark.parametrize("signOrbitNormal", [1, -1])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_flybyPoint(show_plots, initPos, initVel, dTsim, dTfilter, signOrbitNormal, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the correctness of the reference attitude computed by :ref:`flybyPoint` in a scenario
    where the rectilinear flyby assumption is valid.

    **Test Parameters**

    In this test, there is no gravity body, and the spacecraft is put onto a rectilinear trajectory about the origin.
    With no gravity, the linear momentum of the spacecraft does not change, which means that the spacecraft proceeds
    along a rectilinear trajectory. The input message to the :ref:`flybyPoint` is the relative position and velocity
    of the spacecraft with respect to the body/asteroid, which coincides with the origin and is assumed to be static.
    Correctness is tested assessing whether the computed hill frame moves according to the motion of the spacecraft.

    Args:
        initPos[3] (m): initial position of the spacecraft w.r.t. the body/origin
        initVel[3] (m): initial velocity of the spacecraft w.r.t. the body/origin
        dTsim (s): simulation time step
        dTfilter (s): time between two consecutive reads of the input message
        signOrbitNormal (-): sign of the reference frame "out of plane" vector (orbit normal or anti orbit normal)
        accuracy: tolerance on the result.

    **Description of Variables Being Tested**

    The referene attitude :math:`\sigma_\mathcal{R/N}`, reference angular rates :math:`\omega_\mathcal{R/N}` and
    angular accelerations :math:`\dot{\omega}_\mathcal{R/N}` are tested. These are compared to the analytical results
    expected from the rectilinear motion described in the documentation of :ref:`flybyPoint`.
    The reference attitude is mapped to the corresponding reference frame, and each axis of the reference frame is
    tested for correctness. The angular rate and acceleration vectors are tested against the analytical result,
    expressed in R-frame coordinates.
    """
    # each test method requires a single assert method to be called
    flybyPointTestFunction(show_plots, initPos, initVel, dTsim, dTfilter, signOrbitNormal, accuracy)


def flybyPointTestFunction(show_plots, initPos, initVel, dTsim, dTfilter, signOrbitNormal, accuracy):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(dTsim)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create simulation variable names
    unitTaskName = "unitTask"
    unitProcessName = "unitProcess"

    #  Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    testProcess = unitTestSim.CreateNewProcess(unitProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(dTsim)
    testProcess.addTask(unitTestSim.CreateNewTask(unitTaskName, simulationTimeStep))

    # setup flybyPoint guidance module
    flybyGuid = flybyPoint.FlybyPoint()
    flybyGuid.ModelTag = "flybyPoint"
    flybyGuid.dtFilterData = dTfilter
    flybyGuid.signOfOrbitNormalFrameVector = signOrbitNormal
    unitTestSim.AddModelToTask(unitTaskName, flybyGuid)

    inputData = messaging.NavTransMsgPayload()
    inputData.v_BN_N = np.array(initVel)
    filterInMsg = messaging.NavTransMsg()
    flybyGuid.filterInMsg.subscribeTo(filterInMsg)

    #
    #   Setup data logging before the simulation is initialized
    #
    attRefLog = flybyGuid.attRefOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, attRefLog)

    unitTestSim.InitializeSimulation()
    dataPos = []
    dataVel = []
    for i in range(round(9*600/dTsim)):
        dataPos.append(np.array(initPos) + np.array(initVel) * (i * dTsim))
        dataVel.append(np.array(initVel))
        inputData.timeTag = macros.sec2nano(i * dTsim)
        inputData.r_BN_N = dataPos[i]
        filterInMsg.write(inputData, unitTestSim.TotalSim.CurrentNanos)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1) * dTsim) - 1)
        unitTestSim.ExecuteSimulation()

    #   retrieve the logged data
    refAtt = attRefLog.sigma_RN
    refRate = attRefLog.omega_RN_N
    refAcc = attRefLog.domega_RN_N
    timeData = attRefLog.times() * macros.NANO2MIN

    ur_output = []
    ut_output = []
    uh_output = []
    for i in range(len(refAtt)):
        RN = rbk.MRP2C(refAtt[i])
        ur_output.append(np.matmul(RN.transpose(), [1, 0, 0]))
        ut_output.append(np.matmul(RN.transpose(), [0, 1, 0]))
        uh_output.append(np.matmul(RN.transpose(), [0, 0, 1]))

    ur = initPos / np.linalg.norm(initPos)
    uh = np.cross(initPos, initVel) / np.linalg.norm(np.cross(initPos, initVel))
    ut = np.cross(uh, ur)
    f0 = np.linalg.norm(initVel) / np.linalg.norm(initPos)
    gamma0 = np.arctan(np.dot(initVel, ur) / np.dot(initVel, ut))

    for i in range(len(refAtt)):
        # check a vector values
        ur = dataPos[i] / np.linalg.norm(dataPos[i])
        uh = np.cross(dataPos[i], dataVel[i]) / np.linalg.norm(np.cross(dataPos[i], dataVel[i]))
        ut = np.cross(uh, ur)
        dt = timeData[i]*60
        den = ((f0*dt)**2 + 2*f0*np.sin(gamma0)*dt + 1)
        omega = uh * f0 * np.cos(gamma0) / den
        omegaDot = uh * (-2*f0*f0 * np.cos(gamma0)) * (f0*dt + np.sin(gamma0)) / den / den
        if signOrbitNormal == -1:
            ut = np.cross(ur, uh)
            uh = np.cross(ur, ut)

        # test correctness of frame, angular rate and acceleration
        np.testing.assert_allclose(ur_output[i], ur, rtol=0, atol=accuracy, verbose=True)
        np.testing.assert_allclose(ut_output[i], ut, rtol=0, atol=accuracy, verbose=True)
        np.testing.assert_allclose(uh_output[i], uh, rtol=0, atol=accuracy, verbose=True)
        np.testing.assert_allclose(refRate[i], omega, rtol=0, atol=accuracy, verbose=True)
        np.testing.assert_allclose(refAcc[i], omegaDot, rtol=0, atol=accuracy, verbose=True)

    # plot the results
    plt.close("all")  # clears out plots from earlier test runs
    dataPos = np.array(dataPos)
    dataVel = np.array(dataVel)
    plot_position(timeData, dataPos)
    plot_velocity(timeData, dataVel)
    plot_ref_attitude(timeData, refAtt)
    plot_ref_rates(timeData, refRate)
    plot_ref_accelerations(timeData, refAcc)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    # return [testFailCount, ''.join(testMessages)]
    return


def plot_position(timeData, dataPos):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeData, dataPos[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$r_{BN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Inertial Position [m]')


def plot_velocity(timeData, dataVel):
    """Plot the attitude errors."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataVel[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$v_{BN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Inertial Velocity [m/s]')


def plot_ref_attitude(timeData, sigma_RN):
    """Plot the attitude errors."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, sigma_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_{RN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference attitude')


def plot_ref_rates(timeData, omega_RN):
    """Plot the attitude errors."""
    plt.figure(4)
    for idx in range(3):
        plt.plot(timeData, omega_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{RN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference rates')


def plot_ref_accelerations(timeData, omegaDot_RN):
    """Plot the attitude errors."""
    plt.figure(5)
    for idx in range(3):
        plt.plot(timeData, omegaDot_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\dot{\omega}_{RN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference accelerations')


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_flybyPoint(
        True,                # show_plots
        [-5e7, 7.5e6, 5e5],  # initPos
        [2e4, 0, 0],         # initVel
        100,                  # dTsim
        600,                  # dTfilter
        1,                   # sign Orbit Normal
        1e-12                # accuracy
    )
