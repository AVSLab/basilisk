#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This script illustrates how to setup different variable time step integration methods for a basic 3-DOF orbit scenario.
Both a fourth-order (RKF45) and a seventh-order (RKF78) integrators are used. For comparison, an RK4 integrator is also
used.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioVariableTimeStepIntegrators.py

For more information on how to setup different integrators, see :ref:`scenarioIntegrators`. When the simulation
completes, a plot is shown for illustrating both the true and the numerically evaluated orbit.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, integratorCase = {'rk4', 'rkf45', 'rkf78'}

The following figure illustrates the resulting trajectories relative to the true trajectory using a very coarse
integration time step of 2 hours. The variable time step integrators still approximates the true orbit well, while
the RK4 method is starting to show some visible errors, illustrating that much smaller time steps must be used with
this method in this scenario.

.. image:: /_images/Scenarios/scenarioVariableTimeStepIntegrators.svg
   :align: center


Step Size and Tolerances
------------------------

The task rate sets the largest integration step. At each task update the variable time step integrators start
with a trial step equal to the whole task period, and they can shorten the step to meet ``relTol`` and ``absTol``,
but they never take a step longer than the task period. The tolerances therefore govern the accuracy only below
the task period, and they apply to each state in its own units, for example the position in m and the velocity
in m/s.

At the default ``relTol`` of 1e-4, the error estimate can accept long steps even where the dynamics change rapidly,
such as through the periapsis of an eccentric orbit. The function ``runAccuracyComparison()`` illustrates this by
propagating the orbit of this scenario with the RKF45 and RKF78 integrators and comparing the position with the
solution of Kepler's equation. The cases log their states at different times, because the logging interval depends
on the task period, so the errors are compared at a common epoch: the latest time logged by every case, 156 hours
after the start, or about 0.88 orbits. It produces the following results:

.. list-table:: Position error after 156 hours
   :header-rows: 1

   * - Integrator
     - Task period
     - ``relTol``, ``absTol``
     - Position error
   * - RKF45
     - 2 h
     - 1e-4, 1e-8 (default)
     - 3,310 km
   * - RKF45
     - 2 h
     - 1e-8, 1e-12
     - 1.6 km
   * - RKF78
     - 2 h
     - 1e-4, 1e-8 (default)
     - 154 km
   * - RKF78
     - 2 h
     - 1e-8, 1e-12
     - 31 m
   * - RKF78
     - 10 min
     - 1e-4, 1e-8 (default)
     - 0.01 m

::

    runAccuracyComparison(show_plots=True)

.. image:: /_images/Scenarios/scenarioVariableTimeStepIntegratorsAccuracy.svg
   :align: center

Most of the error is incurred during the first periapsis passage, a few hours into the run, and it then grows
slowly. Either tightening the tolerances or shortening the task period reduces the error. The default tolerances
may not be suitable for all orbit regimes, so the user should check the accuracy against a reference before relying
on them.

Creating New Integrator Modules
-------------------------------

New integration modules can be readily created for Basilisk.  They are all stored in the folder
``Basilisk/src/simulation/dynamics/Integrators/``.

The integrators must be created to function on a general state vector and be independent of the particular
dynamics being integrated.  Note that the default integrator is placed inside the ``_GeneralModulesFiles``
folder within the ``dynamics`` folder.

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Demonstration of how to setup and use different variable time step integrators in
#           Basilisk.  The simulation performs a 3-DOF elliptic orbit scenario.
# Author:   João Vaz Carneiro
# Creation Date:  Sep. 26, 2021
#

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.simulation import spacecraft
from Basilisk.simulation import svIntegrators
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
# attempt to import vizard
from Basilisk.utilities import vizSupport
from Basilisk.utilities import simHelpers

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, integratorCase, relTol, absTol, simulationTimeStep=None):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        integratorCase (bool): Specify what type of integrator to use in the sim

            =======  ============================
            String   Definition
            =======  ============================
            'rk4'    RK4
            'rkf45'  RKF45
            'rkf78'  RKF78
            =======  ============================

        relTol (double): Specify the relative tolerance to use in the integration
        absTol (double): Specify the absolute tolerance to use in the integration
        simulationTimeStep (int): Optional dynamics task period in nanoseconds. Defaults to 2 hours.

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # add progress bar
    scSim.SetProgressBar(True)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    if simulationTimeStep is None:
        simulationTimeStep = macros.hour2nano(2.)  # [ns]
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #
    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"

    # set the variable time step integrator
    if integratorCase == "rkf45":
        integratorObject = svIntegrators.svIntegratorRKF45(scObject)
        scObject.setIntegrator(integratorObject)

        # set the relative and absolute tolerances
        integratorObject.relTol = relTol
        integratorObject.absTol = absTol
    elif integratorCase == "rkf78":
        integratorObject = svIntegrators.svIntegratorRKF78(scObject)
        scObject.setIntegrator(integratorObject)

        # set the relative and absolute tolerances
        integratorObject.relTol = relTol
        integratorObject.absTol = absTol

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 16e7
    oe.e = 0.8
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = -90 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)
    #
    #   initialize Spacecraft States with in the initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(0.9 * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simHelpers.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N

    #
    #   plot the results
    #
    np.set_printoptions(precision=16)
    # if integratorCase == "rkf45":
    #     plt.close("all")  # clears out plots from earlier test runs

    # draw orbit in perifocal frame
    b = oe.a * np.sqrt(1 - oe.e * oe.e)
    p = oe.a * (1 - oe.e * oe.e)
    plt.figure(1)
    plt.axis([-50, 10, -20, 20])
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    ax.set_aspect('equal')
    planetColor = '#008800'
    planetRadius = 1.0
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    labelStrings = ("rk4", "rkf45", "rkf78")
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
        rData.append(oeData.rmag/earth.radEquator)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(rData * np.cos(fData), rData * np.sin(fData)
             , color=simHelpers.getLineColor(labelStrings.index(integratorCase), len(labelStrings))
             , label=integratorCase
             , linewidth=3.0
             )
    # draw the full osculating orbit from the initial conditions
    fData = np.linspace(0, 2 * np.pi, 100)
    rData = []
    for idx in range(0, len(fData)):
        rData.append(p / (1 + oe.e * np.cos(fData[idx])))
    plt.plot(rData * np.cos(fData)/earth.radEquator, rData * np.sin(fData)/earth.radEquator
             , '--'
             , color='#555555'
             )
    plt.xlabel('$i_e$ Cord. [DU]')
    plt.ylabel('$i_p$ Cord. [DU]')
    plt.legend(loc='lower right')
    plt.grid()
    figureList = {}
    pltName = fileName
    figureList[pltName] = plt.figure(1)

    if show_plots:
        plt.show()

    if integratorCase == "rkf78":
        plt.close("all")

    # return some information of value for testing
    return dataLog.times(), posData, velData, figureList


def getKeplerPosition(mu, r0, v0, dt):
    """
    Return the inertial position reached after propagating an initial state with Kepler's equation.

    Args:
        mu (double): Gravitational parameter of the central body [m^3/s^2]
        r0 (array): Initial inertial position [m]
        v0 (array): Initial inertial velocity [m/s]
        dt (double): Propagation time [s]

    Returns:
        array: Inertial position after ``dt`` [m]
    """
    oe = orbitalMotion.rv2elem(mu, r0, v0)
    E0 = orbitalMotion.f2E(oe.f, oe.e)
    M0 = orbitalMotion.E2M(E0, oe.e)
    Mf = M0 + np.sqrt(mu / oe.a / oe.a / oe.a) * dt
    Ef = orbitalMotion.M2E(Mf, oe.e)
    oe.f = orbitalMotion.E2f(Ef, oe.e)
    rF, _ = orbitalMotion.elem2rv(mu, oe)
    return np.array(rF)


def runAccuracyComparison(show_plots):
    """
    Compare the position error of the RKF45 and RKF78 integrators against the solution of Kepler's equation,
    using the default and tightened tolerances and two task periods.

    Args:
        show_plots (bool): Determines if the script should display plots

    Returns:
        tuple: A dictionary of position errors in meters at the common evaluation epoch, a dictionary of the
        evaluation time used for each case in seconds, both keyed by case label, and the figure dictionary
    """
    mu = simIncludeGravBody.gravBodyFactory().createEarth().mu  # [m^3/s^2]

    # (label, integrator, relTol [-], absTol [state units: m for position, m/s for velocity], task period [ns])
    cases = [
        ("RKF45, default tolerances", "rkf45", 1e-4, 1e-8, macros.hour2nano(2.)),  # [-], [m, m/s], [ns]
        ("RKF45, tightened tolerances", "rkf45", 1e-8, 1e-12, macros.hour2nano(2.)),  # [-], [m, m/s], [ns]
        ("RKF78, default tolerances", "rkf78", 1e-4, 1e-8, macros.hour2nano(2.)),  # [-], [m, m/s], [ns]
        ("RKF78, tightened tolerances", "rkf78", 1e-8, 1e-12, macros.hour2nano(2.)),  # [-], [m, m/s], [ns]
        ("RKF78, default tolerances, 10 min task", "rkf78", 1e-4, 1e-8, macros.min2nano(10.)),  # [-], [m, m/s], [ns]
    ]

    errorHistory = {}
    loggedTimes = {}
    for label, integratorCase, relTol, absTol, taskStep in cases:
        timeData, posData, velData, _ = run(False, integratorCase, relTol, absTol, taskStep)
        timeNano = np.asarray(timeData - timeData[0], dtype=np.int64)  # [ns]
        timeSec = timeNano * macros.NANO2SEC  # [s]
        errors = np.array([
            np.linalg.norm(posData[idx] - getKeplerPosition(mu, posData[0], velData[0], timeSec[idx]))
            for idx in range(len(timeSec))
        ])  # [m]
        errorHistory[label] = (timeSec, errors)
        loggedTimes[label] = timeNano

    # The logging interval depends on the task period, so the cases do not share a final sample time.
    # Compare them at the latest time that every case has logged.
    commonTimes = loggedTimes[cases[0][0]]
    for timeNano in loggedTimes.values():
        commonTimes = np.intersect1d(commonTimes, timeNano)
    evalTimeNano = commonTimes[-1]  # [ns]

    finalErrors = {}
    evaluationTimes = {}
    for label, timeNano in loggedTimes.items():
        idx = int(np.flatnonzero(timeNano == evalTimeNano)[0])
        finalErrors[label] = errorHistory[label][1][idx]  # [m]
        evaluationTimes[label] = timeNano[idx] * macros.NANO2SEC  # [s]

    # run() leaves its orbit figure open for some cases, so start the comparison figure from a clean state
    plt.close("all")
    fig = plt.figure(2, figsize=(8, 5))
    labels = list(errorHistory.keys())
    for idx, label in enumerate(labels):
        timeSec, errors = errorHistory[label]
        plt.semilogy(timeSec / 3600., np.maximum(errors, 1e-6)
                     , color=simHelpers.getLineColor(idx, len(labels))
                     , label=label
                     )
    plt.xlabel('Time [h]')
    plt.ylabel('Position Error [m]')
    # leave headroom above the curves so the legend does not cover them
    plt.ylim(1e-6, 1e12)  # [m]
    plt.legend(loc='upper left', ncol=2, fontsize='small')
    plt.grid()
    figureList = {fileName + "Accuracy": fig}

    if show_plots:
        plt.show()

    plt.close("all")

    return finalErrors, evaluationTimes, figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        'rkf78',  # integrator case(0 - rk4, 1 - rkf45, 2 - rkf78)
        1e-5,  # relative tolerance
        1e-8)  # absolute tolerance
