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

r"""
Overview
--------

This scenario demonstrates how to use the Lambert solver module package, consisting of
``lambertPlanner()``, ``lambertSolver()`` and ``lambertValidator()``, the computation of a Delta-V maneuver that takes
the spacecraft to a desired locataion in a given time.

In this scenario, the goal is to reach a target position at final time tf by performing a maneuver at time tm.
This is done by solving Lambert's problem. The Lambert problem is set up using :ref:`lambertPlanner`, which provides the
information in the form of :ref:`lambertProblemMsgPayload` to :ref:`lambertSolver`. Lambert's problem is solved within
:ref:`lambertSolver`, which writes the :ref:`lambertSolutionMsgPayload` and :ref:`lambertPerformanceMsgPayload` output
messages. Finally, :ref:`lambertValidator` processes the content of those messages, computes the required Delta-V, and
only writes a non-zero Delta-V message within :ref:`dvBurnCmdMsgPayload` if no constraints are violated
(minimum orbit radius and final distance from targeted location) and the Delta-V solution has converged.

The simulation layout is shown in the following illustration. The simulation and flight software (FSW) are divided into
two different processes. After the maneuver, all tasks of the FSW process are disabled.

.. image:: /_images/static/test_scenarioLambertSolver.svg
   :align: center

The true and measured spacecraft position and velocity are shown in the plots below.

.. image:: /_images/Scenarios/scenarioLambertSolver1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioLambertSolver2.svg
   :align: center

Likewise, the expected spacecraft position and velocity at the time of the maneuver are shown in the plots below.
Due to the noise of the measured spacecraft state, the expected state at maneuver time changes slightly with time.

.. image:: /_images/Scenarios/scenarioLambertSolver3.svg
   :align: center

.. image:: /_images/Scenarios/scenarioLambertSolver4.svg
   :align: center

The next Figure shows the Delta-V that will be required at maneuver time to take the spacecraft to the target location.
Again, due to the noise of the measured spacecraft state, the Delta-V changes slightly with time.

.. image:: /_images/Scenarios/scenarioLambertSolver5.svg
   :align: center

The following three figures show the performance message content of the Lambert solver module,
including the solution of the iteration variable x, the number of iterations it took to find x, and the change in x
between the last and second to last root-finder iteration.

.. image:: /_images/Scenarios/scenarioLambertSolver6.svg
   :align: center

.. image:: /_images/Scenarios/scenarioLambertSolver7.svg
   :align: center

.. image:: /_images/Scenarios/scenarioLambertSolver8.svg
   :align: center

Finally, the last figure shows the failedDvSolutionConvergence flag of the :ref:`lambertValidatorMsgPayload`, which is 1
if the lambert validator returned a zeroed Delta-V if the Delta-V solution is too different from the previous time step,
and 0 otherwise. At the very first time step, the flag is equal to 1, because it is the first time step so the solution
has not converged it. At all subsequent time steps, the flag is equal to 0.

.. image:: /_images/Scenarios/scenarioLambertSolver9.svg
   :align: center

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioLambertSolver.py


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing how to use the Lambert Planner, Lambert Solver and Lambert Validator modules for
#           autonomous Delta-V guidance
# Author:   Julian Hammerl
# Creation Date:  May 8, 2023
#

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.fswAlgorithms import (lambertPlanner, lambertSolver, lambertValidator)
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spacecraft
from Basilisk.utilities import (SimulationBaseClass, macros, simIncludeGravBody, vizSupport)
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport

try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

# The path to the location of Basilisk
# Used to get the location of supporting data.
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    path = os.path.dirname(os.path.abspath(__file__))

    # Create simulation variable names
    dynTaskName = "simTask"
    dynProcessName = "simProcess"
    fswTaskName = "fswTask"
    fswProcessName = "fswProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(dynProcessName)
    fswProcess = scSim.CreateNewProcess(fswProcessName)

    # create the dynamics task and specify the simulation time step information
    simStep = 10.
    fswStep = 30.
    simTimeStep = macros.sec2nano(simStep)
    fswTimeStep = macros.sec2nano(fswStep)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simTimeStep))
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    # Create gravitational bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True

    mu = planet.mu
    rEarth = 6378 * 1e3

    # create SC object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"

    oeSC = orbitalMotion.ClassicElements()
    oeSC.a = 10000. * 1e3
    oeSC.e = 0.001
    oeSC.i = 5. * macros.D2R
    oeSC.Omega = 10. * macros.D2R
    oeSC.omega = 10. * macros.D2R
    oeSC.f = 10. * macros.D2R
    # spacecraft state at initial time
    r_BO_N, v_BO_N = orbitalMotion.elem2rv(mu, oeSC)

    # Set the truth ICs for the spacecraft position and velocity
    scObject.hub.r_CN_NInit = r_BO_N  # m   - r_BN_N
    scObject.hub.v_CN_NInit = v_BO_N  # m/s - v_BN_N
    scObject.hub.mHub = 330.  # kg
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # orbit transfer parameters
    tau = 2*np.pi*np.sqrt(oeSC.a**3/mu)
    tm = round(tau/4./simStep)*simStep  # maneuver time
    tf = round(tau/2./simStep)*simStep  # final time
    r_TN_N = np.array([-(rEarth + 200 * 1e3), 0., 0.])  # targeted position

    # Lambert solution validation parameters
    maxDistanceTarget = 500.
    minOrbitRadius = rEarth

    # Set up simpleNav for s/c "measurements"
    simpleNavMeas = simpleNav.SimpleNav()
    simpleNavMeas.ModelTag = 'SimpleNav'
    simpleNavMeas.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    pos_sigma_sc = 1.
    vel_sigma_sc = 0.01
    att_sigma_sc = 0.0
    rate_sigma_sc = 0.0
    sun_sigma_sc = 0.0
    dv_sigma_sc = 0.0
    p_matrix_sc = np.diag([pos_sigma_sc, pos_sigma_sc, pos_sigma_sc,
                           vel_sigma_sc, vel_sigma_sc, vel_sigma_sc,
                           att_sigma_sc, att_sigma_sc, att_sigma_sc,
                           rate_sigma_sc, rate_sigma_sc, rate_sigma_sc,
                           sun_sigma_sc, sun_sigma_sc, sun_sigma_sc,
                           dv_sigma_sc, dv_sigma_sc, dv_sigma_sc])
    walk_bounds_sc = [[10.], [10.], [10.],
                      [1], [1], [1],
                      [0.], [0.], [0.],
                      [0.], [0.], [0.],
                      [0.], [0.], [0.],
                      [0.], [0.], [0.]]
    simpleNavMeas.PMatrix = p_matrix_sc
    simpleNavMeas.walkBounds = walk_bounds_sc

    # set up Lambert planner
    lamPlanner = lambertPlanner.LambertPlanner()
    lamPlanner.ModelTag = "lambertPlanner"
    lamPlanner.r_TN_N = r_TN_N
    lamPlanner.finalTime = tf
    lamPlanner.maneuverTime = tm
    lamPlanner.mu = mu
    lamPlanner.numRevolutions = 0
    lamPlanner.navTransInMsg.subscribeTo(simpleNavMeas.transOutMsg)

    # set up Lambert Solver
    lamSolver = lambertSolver.LambertSolver()
    lamSolver.ModelTag = "lambertSolver"
    lamSolver.lambertProblemInMsg.subscribeTo(lamPlanner.lambertProblemOutMsg)

    # set up Lambert Validator
    lamValidator = lambertValidator.LambertValidator()
    lamValidator.ModelTag = "lambertValidator"
    lamValidator.finalTime = tf
    lamValidator.maneuverTime = tm
    lamValidator.maxDistanceTarget = maxDistanceTarget
    lamValidator.minOrbitRadius = minOrbitRadius
    lamValidator.uncertaintyStates = np.diag([pos_sigma_sc, pos_sigma_sc, pos_sigma_sc,
                                              vel_sigma_sc, vel_sigma_sc, vel_sigma_sc])
    lamValidator.uncertaintyDV = 0.1
    lamValidator.dvConvergenceTolerance = 1.
    lamValidator.navTransInMsg.subscribeTo(simpleNavMeas.transOutMsg)
    lamValidator.lambertProblemInMsg.subscribeTo(lamPlanner.lambertProblemOutMsg)
    lamValidator.lambertPerformanceInMsg.subscribeTo(lamSolver.lambertPerformanceOutMsg)
    lamValidator.lambertSolutionInMsg.subscribeTo(lamSolver.lambertSolutionOutMsg)

    # Add all models to the task
    scSim.AddModelToTask(dynTaskName, scObject, ModelPriority=100)
    scSim.AddModelToTask(dynTaskName, simpleNavMeas, ModelPriority=99)
    scSim.AddModelToTask(fswTaskName, lamPlanner, ModelPriority=98)
    scSim.AddModelToTask(fswTaskName, lamSolver, ModelPriority=97)
    scSim.AddModelToTask(fswTaskName, lamValidator, ModelPriority=96)

    #   Setup data logging before the simulation is initialized
    sc_truth_recorder = scObject.scStateOutMsg.recorder()
    sc_meas_recorder = simpleNavMeas.transOutMsg.recorder()
    lamProblem_recorder = lamPlanner.lambertProblemOutMsg.recorder()
    lamSolution_recorder = lamSolver.lambertSolutionOutMsg.recorder()
    lamPerf_recorder = lamSolver.lambertPerformanceOutMsg.recorder()
    dvCmd_recorder = lamValidator.dvBurnCmdOutMsg.recorder()
    lamVal_recorder = lamValidator.lambertValidatorOutMsg.recorder()
    scSim.AddModelToTask(dynTaskName, sc_truth_recorder)
    scSim.AddModelToTask(dynTaskName, sc_meas_recorder)
    scSim.AddModelToTask(fswTaskName, lamProblem_recorder)
    scSim.AddModelToTask(fswTaskName, lamSolution_recorder)
    scSim.AddModelToTask(fswTaskName, lamPerf_recorder)
    scSim.AddModelToTask(fswTaskName, dvCmd_recorder)
    scSim.AddModelToTask(fswTaskName, lamVal_recorder)

    # Vizard Visualization Option
    # ---------------------------
    # If you wish to transmit the simulation data to the United based Vizard Visualization application,
    # then uncomment the following
    # line from the python scenario script.  This will cause the BSK simulation data to
    # be stored in a binary file inside the _VizFiles sub-folder with the scenario folder.  This file can be read in by
    # Vizard and played back after running the BSK simulation.
    if vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scObject,
                                                  # saveFile=fileName
                                                  )
        viz.settings.showSpacecraftLabels = 1

    # initialize Simulation
    scSim.InitializeSimulation()
    #  get access to dynManager translational states for future access to the states (in order to apply Delta-V)
    velRef = scObject.dynManager.getStateObject("hubVelocity")

    # configure a simulation stop time and execute the simulation run
    simulationTime = macros.sec2nano(tm)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Next, the state manager objects are called to retrieve the latest inertial position and
    # velocity vector components:
    vm_N = unitTestSupport.EigenVector3d2np(velRef.getState())

    dv_N = dvCmd_recorder.dvInrtlCmd[-1, :]

    # After reading the Delta-V command, the state managers velocity is updated through
    velRef.setState(unitTestSupport.np2EigenVectorXd(vm_N + dv_N))
    # disable flight software after maneuver
    fswProcess.disableAllTasks()

    # To start up the simulation again, note that the total simulation time must be provided,
    # not just the next incremental simulation time.
    simulationTime = macros.sec2nano(tf)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve logged spacecraft position relative to asteroid
    r_BN_N_truth = sc_truth_recorder.r_BN_N
    r_BN_N_meas = sc_meas_recorder.r_BN_N
    v_BN_N_truth = sc_truth_recorder.v_BN_N
    v_BN_N_meas = sc_meas_recorder.v_BN_N
    r1_N = lamProblem_recorder.r1vec
    v1_N = lamSolution_recorder.v1
    xSol = lamPerf_recorder.x
    numIter = lamPerf_recorder.numIter
    errX = lamPerf_recorder.errX
    dv_N = dvCmd_recorder.dvInrtlCmd
    failedDvSolutionConvergence = lamVal_recorder.failedDvSolutionConvergence

    #
    #   plot the results
    #
    time = sc_truth_recorder.times() * macros.NANO2SEC
    timeFSW = lamProblem_recorder.times() * macros.NANO2SEC
    figureList = {}

    # dynamics plots
    plot_position(time, np.array(r_BN_N_truth), np.array(r_BN_N_meas), r_TN_N)
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_velocity(time, np.array(v_BN_N_truth), np.array(v_BN_N_meas))
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    # FSW plots

    # FSW stops after maneuver
    # to plot for entire time span and automatically adjust axes, data is extended for plotting
    plot_rm(np.append(timeFSW, time[-1]), np.append(r1_N, [np.nan * np.ones(3)], axis=0))
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plot_vm(np.append(timeFSW, time[-1]), np.append(v1_N, [np.nan * np.ones(3)], axis=0))
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    plot_dV(np.append(timeFSW, time[-1]), np.append(dv_N, [np.nan * np.ones(3)], axis=0))
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(5)

    plot_x(np.append(timeFSW, time[-1]), np.append(xSol, [np.nan]))
    pltName = fileName + "6"
    figureList[pltName] = plt.figure(6)

    plot_numIter(np.append(timeFSW, time[-1]), np.append(numIter, [np.nan]))
    pltName = fileName + "7"
    figureList[pltName] = plt.figure(7)

    plot_errX(np.append(timeFSW, time[-1]), np.append(errX, [np.nan]))
    pltName = fileName + "8"
    figureList[pltName] = plt.figure(8)

    plot_failedDvConvergence(np.append(timeFSW, time[-1]), np.append(failedDvSolutionConvergence, [np.nan]))
    pltName = fileName + "9"
    figureList[pltName] = plt.figure(9)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


# Plotting functions
def plot_position(time, r_BN_N_truth, r_BN_N_meas, r_TN_N):
    """Plot the position result."""
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, r_BN_N_meas[:, 0], 'k*', label='measurement', markersize=2)
    ax[1].plot(time, r_BN_N_meas[:, 1], 'k*', markersize=2)
    ax[2].plot(time, r_BN_N_meas[:, 2], 'k*', markersize=2)

    ax[0].plot(time, r_BN_N_truth[:, 0], label='truth')
    ax[1].plot(time, r_BN_N_truth[:, 1])
    ax[2].plot(time, r_BN_N_truth[:, 2])

    ax[0].plot(time[-1], r_TN_N[0], 'rx', label='target')
    ax[1].plot(time[-1], r_TN_N[1], 'rx')
    ax[2].plot(time[-1], r_TN_N[2], 'rx')

    plt.xlabel('Time [sec]')
    plt.title('Spacecraft Position')

    ax[0].set_ylabel('${}^Nr_{BN_1}$ [m]')
    ax[1].set_ylabel('${}^Nr_{BN_2}$ [m]')
    ax[2].set_ylabel('${}^Nr_{BN_3}$ [m]')

    ax[0].legend(loc='upper right')


def plot_velocity(time, v_BN_N_truth, v_BN_N_meas):
    """Plot the velocity result."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, v_BN_N_meas[:, 0], 'k*', label='measurement', markersize=2)
    ax[1].plot(time, v_BN_N_meas[:, 1], 'k*', markersize=2)
    ax[2].plot(time, v_BN_N_meas[:, 2], 'k*', markersize=2)

    ax[0].plot(time, v_BN_N_truth[:, 0], label='truth')
    ax[1].plot(time, v_BN_N_truth[:, 1])
    ax[2].plot(time, v_BN_N_truth[:, 2])

    plt.xlabel('Time [sec]')
    plt.title('Spacecraft Velocity')

    ax[0].set_ylabel('${}^Nv_{BN_1}$ [m/s]')
    ax[1].set_ylabel('${}^Nv_{BN_2}$ [m/s]')
    ax[2].set_ylabel('${}^Nv_{BN_3}$ [m/s]')

    ax[0].legend()


def plot_rm(time, rm_BN_N):
    """Plot the expected position at maneuver time."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, rm_BN_N[:, 0], 'k*', label='measurement', markersize=2)
    ax[1].plot(time, rm_BN_N[:, 1], 'k*', markersize=2)
    ax[2].plot(time, rm_BN_N[:, 2], 'k*', markersize=2)

    plt.xlabel('Time [sec]')
    plt.title('Planner: Expected Spacecraft Position at Maneuver Time')

    ax[0].set_ylabel('${}^Nr_{BN,m_1}$ [m]')
    ax[1].set_ylabel('${}^Nr_{BN,m_2}$ [m]')
    ax[2].set_ylabel('${}^Nr_{BN,m_3}$ [m]')

    ax[0].set_xlim([time[0], time[-1]])
    ax[1].set_xlim([time[0], time[-1]])
    ax[2].set_xlim([time[0], time[-1]])


def plot_vm(time, vm_BN_N):
    """Plot the expected velocity at maneuver time."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, vm_BN_N[:, 0], 'k*', label='measurement', markersize=2)
    ax[1].plot(time, vm_BN_N[:, 1], 'k*', markersize=2)
    ax[2].plot(time, vm_BN_N[:, 2], 'k*', markersize=2)

    plt.xlabel('Time [sec]')
    plt.title('Planner: Expected Spacecraft Velocity at Maneuver Time')

    ax[0].set_ylabel('${}^Nv_{BN,m_1}$ [m/s]')
    ax[1].set_ylabel('${}^Nv_{BN,m_2}$ [m/s]')
    ax[2].set_ylabel('${}^Nv_{BN,m_3}$ [m/s]')

    ax[0].set_xlim([time[0], time[-1]])
    ax[1].set_xlim([time[0], time[-1]])
    ax[2].set_xlim([time[0], time[-1]])


def plot_dV(time, dv_N):
    """Plot the required Delta-V for the maneuver."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, dv_N[:, 0], 'k*', markersize=2)
    ax[1].plot(time, dv_N[:, 1], 'k*', markersize=2)
    ax[2].plot(time, dv_N[:, 2], 'k*', markersize=2)

    plt.xlabel('Time [sec]')
    plt.title('Delta-V')

    ax[0].set_ylabel('${}^N\\Delta v_{1}$ [m/s]')
    ax[1].set_ylabel('${}^N\\Delta v_{2}$ [m/s]')
    ax[2].set_ylabel('${}^N\\Delta v_{3}$ [m/s]')

    ax[0].set_xlim([time[0], time[-1]])
    ax[1].set_xlim([time[0], time[-1]])
    ax[2].set_xlim([time[0], time[-1]])


def plot_x(time, x):
    """Plot the x solution results from the Lambert solver."""
    plt.figure()
    plt.gcf()
    plt.ticklabel_format(useOffset=False)

    plt.plot(time, x)

    plt.xlabel('Time [sec]')
    plt.ylabel('x [-]')
    plt.title('Lambert Solver Performance: x Solution')

    plt.xlim([time[0], time[-1]])


def plot_numIter(time, numIter):
    """Plot the number of iterations from the Lambert solver."""
    plt.figure()
    plt.gcf()
    plt.ticklabel_format(useOffset=False)

    plt.plot(time, numIter)

    plt.xlabel('Time [sec]')
    plt.ylabel('number of iterations [-]')
    plt.title('Lambert Solver Performance: Number of Iterations')

    plt.xlim([time[0], time[-1]])


def plot_errX(time, errX):
    """Plot the error in x from the Lambert solver."""
    plt.figure()
    plt.gcf()
    plt.ticklabel_format(useOffset=False)

    plt.plot(time, errX)

    plt.xlabel('Time [sec]')
    plt.ylabel('x error')
    plt.title('Lambert Solver Performance: Error in x')

    plt.xlim([time[0], time[-1]])


def plot_failedDvConvergence(time, failedDvSolutionConvergence):
    """Plot the failedDvSolutionConvergence flag."""
    plt.figure()
    plt.gcf()
    plt.ticklabel_format(useOffset=False)

    plt.plot(time, failedDvSolutionConvergence)

    plt.xlabel('Time [sec]')
    plt.ylabel('Failed Dv Convergence Flag')
    plt.title('Lambert Validator: Failed Delta-V convergence (true if 1)')

    plt.xlim([time[0], time[-1]])


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True  # show_plots
    )
