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
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, integratorCase, relTol, absTol):
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
    simulationTimeStep = macros.hour2nano(2.)
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
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

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
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
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
             , color=unitTestSupport.getLineColor(labelStrings.index(integratorCase), len(labelStrings))
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

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return posData, figureList


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
