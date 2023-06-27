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

This script illustrates how to setup different integration methods for a basic 3-DOF orbit scenario.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioIntegrators.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains the spacecraft object.  Gravity effectors are attached to the spacecraft dynamics to
simulate the gravitational accelerations.  The spacecraft object provides the states that the integration
module needs to perform the time integration.

.. image:: /_images/static/test_scenarioIntegrators.svg
   :align: center

If :ref:`spacecraft`, or any other dynamics module, is created without specifying a particular
integration type, the fixed time step 4th order Runge-Kutta method is used by default.  To invoke a
different integration scheme, the following code is used before the dynamics module is added to the
python task list:

.. code-block:: python

   integratorObject = svIntegrators.svIntegratorEuler(scObject)
   scObject.setIntegrator(integratorObject)

The first line invokes an instance of the desired state vector integration module, and provides
the dynamics module (spacecraft() in this case) as the input.  This specifies to the integrator
module which other module will provide the ``equationOfMotion()`` function to evaluate the derivatives of
the state vector.  The second line ties the integration module to the dynamics module.  After that we are
done.

The integrator scenario script is setup to evaluate the default integration method (RK4), a fourth-order variable time
step integrator (RKF45), a first order Euler integration method, as well as a second order RK2 method.

Moreover, this scenario illustrates how to set-up your own explicit Runge-Kutta
methods simply by providing the coefficients in their Butcher table. The 3rd order
Runge-Kutta and the (adaptive) Bogacki-Shampine methods are implemented in this way
to illustrate how to create custom integrators:

.. code-block:: python

    # 3rd order Runge-Kutta method
    integratorObject = svIntegrators.svIntegratorRungeKutta(
        scObject,
        a_coefficients=[
            [0,   0, 0],
            [1/2, 0, 0],
            [-1,  2, 0]
        ],
        b_coefficients=[1/6, 2/3, 1/6],
        c_coefficients=[0, 0.5, 1]
    )
    scObject.setIntegrator(integratorObject)

.. code-block:: python

    # Bogacki-Shampine method
    integratorObject = svIntegrators.svIntegratorAdaptiveRungeKutta(
        scObject,
        largest_order=3,
        a_coefficients=[
            [0,   0,   0,   0],
            [1/2, 0,   0,   0],
            [0  , 3/4, 0,   0],
            [2/9, 1/3, 4/9, 0]
        ],
        b_coefficients=[7/24, 1/4, 1/3, 1/8],
        b_star_coefficients=[2/9, 1/3, 4/9, 0],
        c_coefficients=[0, 1/2, 3/4, 1]
    )
    scObject.setIntegrator(integratorObject)


When the simulation completes a plot is shown for illustrating both the true and the numerically
evaluated orbit.


Illustration of Simulation Results
----------------------------------

::

    show_plots = True, integratorCase = {'rk4', 'rkf45', 'rk2', 'euler'}

The following figure illustrates the resulting
trajectories relative to the true trajectory using a very coarse integration time step of 120 seconds.
The RK4 and RKF45 method still approximate the true orbit well, while the RK2 method is starting to show some visible
errors. The first order Euler method provides a horrible estimate of the resulting trajectory, illustrating
that much smaller time steps must be used with this method in this scenario.

.. image:: /_images/Scenarios/scenarioIntegrators.svg
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
# Purpose:  Demonstration of how to setup and use different integrators in
#           Basilisk.  The simulation performs a 3-DOF orbit scenario.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 14, 2016
#

import os

import matplotlib.pyplot as plt
import numpy as np

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import svIntegrators

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import (
    unitTestSupport,
)  # general support file with common unit test functions

# attempt to import vizard
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, integratorCase):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        integratorCase (bool): Specify what type of integrator to use in the simulation

            ================= ============================
            String            Definition
            ================= ============================
            'rk4'             RK4 - default
            'rkf45'           RKF45
            'rkf78'           RKF78
            'rk2'             RK2
            'euler'           Euler or RK1
            'rk3'             RK3
            'bogackiShampine' Bogacki-Shampine adaptive
            ================= ============================

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(120.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #
    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"

    # default case, RK4 is automatically setup, no extra code is needed
    if integratorCase == "rkf45":
        integratorObject = svIntegrators.svIntegratorRKF45(scObject)
        scObject.setIntegrator(integratorObject)
    if integratorCase == "rkf78":
        integratorObject = svIntegrators.svIntegratorRKF78(scObject)
        scObject.setIntegrator(integratorObject)
    elif integratorCase == "euler":
        integratorObject = svIntegrators.svIntegratorEuler(scObject)
        scObject.setIntegrator(integratorObject)
    elif integratorCase == "rk2":
        integratorObject = svIntegrators.svIntegratorRK2(scObject)
        scObject.setIntegrator(integratorObject)
    elif integratorCase == "rk3":
        integratorObject = svIntegrators.svIntegratorRungeKutta(
            scObject,
            a_coefficients=[[0, 0, 0], [1 / 2, 0, 0], [-1, 2, 0]],
            b_coefficients=[1 / 6, 2 / 3, 1 / 6],
            c_coefficients=[0, 0.5, 1],
        )
        scObject.setIntegrator(integratorObject)
    elif integratorCase == "bogackiShampine":
        integratorObject = svIntegrators.svIntegratorAdaptiveRungeKutta(
            scObject,
            largest_order=3,
            a_coefficients=[
                [0, 0, 0, 0],
                [1 / 2, 0, 0, 0],
                [0, 3 / 4, 0, 0],
                [2 / 9, 1 / 3, 4 / 9, 0],
            ],
            b_coefficients=[7 / 24, 1 / 4, 1 / 3, 1 / 8],
            b_star_coefficients=[2 / 9, 1 / 3, 4 / 9, 0],
            c_coefficients=[0, 1 / 2, 3 / 4, 1],
        )
        scObject.setIntegrator(integratorObject)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(
        list(gravFactory.gravBodies.values())
    )

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000.0 * 1000  # meters
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)
    #
    #   initialize Spacecraft States with in the initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2.0 * np.pi / n
    simulationTime = macros.sec2nano(0.75 * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(
        simulationTime, simulationTimeStep, numDataPoints
    )
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    vizSupport.enableUnityVisualization(
        scSim,
        simTaskName,
        scObject
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
    if integratorCase == "rk4":
        plt.close("all")  # clears out plots from earlier test runs

    # draw orbit in perifocal frame
    b = oe.a * np.sqrt(1 - oe.e * oe.e)
    p = oe.a * (1 - oe.e * oe.e)
    plt.figure(1, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
    plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor = "#008800"
    planetRadius = earth.radEquator / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    labelStrings = ("rk4", "rkf45", "rkf78", "euler", "rk2", "rk3", "bogackiShampine")
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(
        rData * np.cos(fData) / 1000,
        rData * np.sin(fData) / 1000
        # , color=unitTestSupport.getLineColor(labelStrings.index(integratorCase), len(labelStrings))
        ,
        label=integratorCase,
        linewidth=3.0,
    )
    # draw the full osculating orbit from the initial conditions
    fData = np.linspace(0, 2 * np.pi, 100)
    rData = []
    for idx in range(0, len(fData)):
        rData.append(p / (1 + oe.e * np.cos(fData[idx])))
    plt.plot(
        rData * np.cos(fData) / 1000,
        rData * np.sin(fData) / 1000,
        "--",
        color="#555555",
    )
    plt.xlabel("$i_e$ Cord. [km]")
    plt.ylabel("$i_p$ Cord. [km]")
    plt.legend(loc="lower right")
    plt.grid()
    figureList = {}
    pltName = fileName
    figureList[pltName] = plt.figure(1)

    if show_plots:
        plt.show()

    # # close the plots being saved off to avoid over-writing old and new figures
    # plt.close("all")

    if integratorCase == "bogackiShampine":
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
        True, "rk4"  # show_plots
    )  # integrator case(0 - rk4, 1 - rkf45, 2 - rkf78, 3 - euler, 4 - rk2, 5 - rk3, 6 - bogackiShampine)
