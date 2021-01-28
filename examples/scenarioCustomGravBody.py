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

Demonstrates how to convert spacecraft states, stored in a text file from another program, into Basilisk
messages using :ref:`dataFileToViz`.  These messages are red by :ref:`vizInterface` to save a :ref:`Vizard <vizard>`
compatible data play for offline playback and analysis.  In this simulation a servicer is holding a relative
position with respect to an uncontrolled satellite.  Custom spacecraft models are specified for Vizard
in the folder :ref:`Folder_data`.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioDataToViz.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both modules.

.. image:: /_images/static/test_scenarioDataToViz.svg
   :align: center

When the simulation completes several plots are shown for the MRP norm attitude history and the
inertial relative position vector components.  A servicer spacecraft approaches a target and holds a specific
target-frame fixed location even while the target itself is slowly rotating.  The servicer and target orientations
are controlled to be the same to prepare for a final docking maneuver.  If the data is saved to a Vizard file,
then the visualization should look like:

.. image:: /_images/static/vizard-DataFile.jpg
   :align: center

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

.. image:: /_images/Scenarios/scenarioDataToViz1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDataToViz2.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing a servicer (3-axis attitude controlled) and a tumbling debris object.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 29, 2019
#

import os
import matplotlib.pyplot as plt

from Basilisk.utilities import (SimulationBaseClass, macros, simIncludeGravBody, vizSupport)
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import orbitalMotion
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import planetEphemeris

try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

# The path to the location of Basilisk
# Used to get the location of supporting data.
fileName = os.path.basename(os.path.splitext(__file__)[0])



def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    path = os.path.dirname(os.path.abspath(__file__))

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the simulation time step information
    simulationTimeStep = macros.sec2nano(10.0)
    simulationTime = macros.min2nano(120.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # setup grav body orbit
    gravBodyEphem = planetEphemeris.PlanetEphemeris()
    gravBodyEphem.ModelTag = 'planetEphemeris'
    scSim.AddModelToTask(simTaskName, gravBodyEphem)
    gravBodyEphem.setPlanetNames(planetEphemeris.StringVector(["Itokawa", "earth"]))
    oeAsteroid = planetEphemeris.ClassicElementsMsgPayload()
    oeAsteroid.a = 1.3241 * orbitalMotion.AU * 1000  # meters
    oeAsteroid.e = 0.2801
    oeAsteroid.i = 1.6214*macros.D2R
    oeAsteroid.Omega = 69.081*macros.D2R
    oeAsteroid.omega = 162.82*macros.D2R
    oeAsteroid.f = 90.0*macros.D2R

    oeEarth = planetEphemeris.ClassicElementsMsgPayload()
    oeEarth.a = orbitalMotion.AU * 1000  # meters
    oeEarth.e = 0.0167086
    oeEarth.i = 7.155 * macros.D2R
    oeEarth.Omega = 174.9 * macros.D2R
    oeEarth.omega = 288.1 * macros.D2R
    oeEarth.f = 270.0 * macros.D2R

    gravBodyEphem.planetElements = planetEphemeris.classicElementVector([oeAsteroid, oeEarth])

    # setup Earth Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createSun()
    mu = 2.34268    # meters^3/s^2
    asteroid = gravFactory.createCustomGravObject("Itokawa", mu)
    asteroid.isCentralBody = True  # ensure this is the central gravitational body
    asteroid.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[0])

    earth = gravFactory.createCustomGravObject("earth", 0.3986004415E+15, radEquator=6378136.6)
    earth.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[1])

    # create SC dummy objects to setup basic Vizard settings.  Only one has to have the Grav Bodies attached
    # to show up in Vizard
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "bskSat"
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))
    scSim.AddModelToTask(simTaskName, scObject)

    # setup orbit initial conditions
    oe = orbitalMotion.ClassicElements()
    oe.a = 500.0  # meters
    oe.e = 0.0001
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    scRec = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, scRec)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    if vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                  , saveFile=fileName
                                                  )
        viz.settings.showSpacecraftLabels = 1
        # load CAD for target spacecraft
        vizSupport.createCustomModel(viz,
                                     modelPath=os.path.join(path, "data", "Itokawa", "ItokawaHayabusa.obj"),
                                     shader=1,
                                     simBodiesToModify=['Itokawa'],
                                     scale=[962, 962, 962])


    #   initialize Simulation
    scSim.InitializeSimulation()

    #   configure a simulation stop time time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve logged data
    posData = scRec.r_BN_N

    #
    #   plot the results
    #
    timeAxis = scRec.times() * macros.NANO2HOUR
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(3):
        plt.plot(timeAxis, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        False  # show_plots
    )
