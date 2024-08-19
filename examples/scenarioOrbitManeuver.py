#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/hkeL50pq0L0" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

This script sets up a 3-DOF spacecraft which is orbiting Earth.  The purpose
is to illustrate how to start and stop the Basilisk simulation to apply
some :math:`\Delta v`'s for simple orbit maneuvers.  Read :ref:`scenarioBasicOrbit`
to learn how to setup an orbit simulation.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioOrbitManeuver.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains the spacecraft object.  The BSK simulation is run for a fixed period.  After stopping, the
states are changed and the simulation is resumed.

.. image:: /_images/static/test_scenarioOrbitManeuver.svg
   :align: center

When the simulation completes 2 plots are shown for each case.  One plot always shows
the inertial position vector components, while the second plot either shows a plot
of the radius time history (Hohmann maneuver), or the
inclination angle time history plot (Inclination change maneuver).

Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a range of script configurations.

::

    show_plots = True, maneuverCase = 0

In this case a classical Hohmann transfer is being
simulated to go from LEO to reach and stay at GEO. The math behind such maneuvers can be found
in textbooks such as `Analytical Mechanics of Space Systems <http://arc.aiaa.org/doi/book/10.2514/4.102400>`__.

.. image:: /_images/Scenarios/scenarioOrbitManeuver10.svg
   :align: center

.. image:: /_images/Scenarios/scenarioOrbitManeuver20.svg
   :align: center

::

    show_plots = True, maneuverCase = 1

In this case a classical plane change is being
simulated to go rotate the orbit plane first 8 degrees, then another 4 degrees after
orbiting 90 degrees. The math behind such maneuvers can be found
in textbooks such as `Analytical Mechanics of Space Systems
<http://arc.aiaa.org/doi/book/10.2514/4.102400>`__.

.. image:: /_images/Scenarios/scenarioOrbitManeuver11.svg
   :align: center

.. image:: /_images/Scenarios/scenarioOrbitManeuver21.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft() and gravity modules illustrating
#           how impulsive Delta_v maneuver can be simulated with stopping and starting the
#           simulation.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 26, 2016
#

import math
import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, maneuverCase):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        maneuverCase (int):

            ======  ============================
            Int     Definition
            ======  ============================
            0       Hohmann maneuver
            1       Inclination change maneuver
            ======  ============================

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
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000  # meters
    rGEO = math.pow(earth.mu / math.pow((2. * np.pi) / (24. * 3600.), 2), 1. / 3.)
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(earth.mu, oe)
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N

    # set the simulation time
    n = np.sqrt(earth.mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(0.25 * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataRec)

    if vizSupport.vizFound:
        # if this scenario is to interface with the BSK Viz, uncomment the following lines
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                  , oscOrbitColorList=[vizSupport.toRGBA255("yellow")]
                                                  , trueOrbitColorList=[vizSupport.toRGBA255("turquoise")]
                                                  # , saveFile=fileName
                                                  )
        viz.settings.mainCameraTarget = "earth"
        viz.settings.trueTrajectoryLinesOn = 1

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #  get access to dynManager translational states for future access to the states
    #
    posRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubPosition)
    velRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubVelocity)

    # The dynamics simulation is setup using a Spacecraft() module with the Earth's
    # gravity module attached.  Note that the rotational motion simulation is turned off to leave
    # pure 3-DOF translation motion simulation.  After running the simulation for 1/4 of a period
    # the simulation is stopped to apply impulsive changes to the inertial velocity vector.
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Next, the state manager objects are called to retrieve the latest inertial position and
    # velocity vector components:
    rVt = unitTestSupport.EigenVector3d2np(posRef.getState())
    vVt = unitTestSupport.EigenVector3d2np(velRef.getState())

    # compute maneuver Delta_v's
    if maneuverCase == 1:
        # inclination change
        Delta_i = 8.0 * macros.D2R
        rHat = rVt / np.linalg.norm(rVt)
        hHat = np.cross(rVt, vVt)
        hHat = hHat / np.linalg.norm(hHat)
        vHat = np.cross(hHat, rHat)
        v0 = np.dot(vHat, vVt)
        vVt = vVt - (1. - np.cos(Delta_i)) * v0 * vHat + np.sin(Delta_i) * v0 * hHat

        # After computing the maneuver specific Delta_v's, the state managers velocity is updated through
        velRef.setState(vVt)
        T2 = macros.sec2nano(P * 0.25)
    else:
        # Hohmann Transfer to GEO
        v0 = np.linalg.norm(vVt)
        r0 = np.linalg.norm(rVt)
        at = (r0 + rGEO) * .5
        v0p = np.sqrt(earth.mu / at * rGEO / r0)
        n1 = np.sqrt(earth.mu / at / at / at)
        T2 = macros.sec2nano((np.pi) / n1)
        vHat = vVt / v0
        vVt = vVt + vHat * (v0p - v0)
        # After computing the maneuver specific Delta_v's, the state managers velocity is updated through
        velRef.setState(vVt)

    # To start up the simulation again, note that the total simulation time must be provided,
    # not just the next incremental simulation time.
    scSim.ConfigureStopTime(simulationTime + T2)
    scSim.ExecuteSimulation()
    # This process is then repeated for the second maneuver.

    # get the current spacecraft states
    rVt = unitTestSupport.EigenVector3d2np(posRef.getState())
    vVt = unitTestSupport.EigenVector3d2np(velRef.getState())
    # compute maneuver Delta_v's
    if maneuverCase == 1:
        # inclination change
        Delta_i = 4.0 * macros.D2R
        rHat = rVt / np.linalg.norm(rVt)
        hHat = np.cross(rVt, vVt)
        hHat = hHat / np.linalg.norm(hHat)
        vHat = np.cross(hHat, rHat)
        v0 = np.dot(vHat, vVt)
        vVt = vVt - (1. - np.cos(Delta_i)) * v0 * vHat + np.sin(Delta_i) * v0 * hHat
        velRef.setState(vVt)
        T3 = macros.sec2nano(P * 0.25)
    else:
        # Hohmann Transfer to GEO
        v1 = np.linalg.norm(vVt)
        v1p = np.sqrt(earth.mu / rGEO)
        n1 = np.sqrt(earth.mu / rGEO / rGEO / rGEO)
        T3 = macros.sec2nano(0.25 * (np.pi) / n1)
        vHat = vVt / v1
        vVt = vVt + vHat * (v1p - v1)
        velRef.setState(vVt)

    # run simulation for 3rd chunk
    scSim.ConfigureStopTime(simulationTime + T2 + T3)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataRec.r_BN_N
    velData = dataRec.v_BN_N

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(3):
        plt.plot(dataRec.times() * macros.NANO2HOUR, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [h]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1" + str(int(maneuverCase))
    figureList[pltName] = plt.figure(1)

    if maneuverCase == 1:
        # show inclination angle
        plt.figure(2)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        iData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(earth.mu, posData[idx], velData[idx])
            iData.append(oeData.i * macros.R2D)
        plt.plot(dataRec.times() * macros.NANO2HOUR, np.ones(len(posData[:, 0])) * 8.93845, '--', color='#444444'
                 )
        plt.plot(dataRec.times() * macros.NANO2HOUR, iData, color='#aa0000'
                 )
        plt.ylim([-1, 10])
        plt.xlabel('Time [h]')
        plt.ylabel('Inclination [deg]')

    else:
        # show SMA
        plt.figure(2)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        rData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem_parab(earth.mu, posData[idx], velData[idx])
            rData.append(oeData.rmag / 1000.)
        plt.plot(dataRec.times() * macros.NANO2HOUR, rData, color='#aa0000',
                 )
        plt.xlabel('Time [h]')
        plt.ylabel('Radius [km]')
    pltName = fileName + "2" + str(int(maneuverCase))
    figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    dataPos = posRef.getState()
    dataPos = [[0.0, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        0  # Maneuver Case (0 - Hohmann, 1 - Inclination)
    )
