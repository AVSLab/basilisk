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

Overview
--------

This scenario demonstrates how to visualize the spacecraft ground tracks on a planet.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioGroundTracks.py

Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a range of script configurations.
When the simulation data is viewed in Vizard the various spacecraft show distinct ground tracks.


.. image:: /_images/Scenarios/scenarioGroundTracks1.svg
   :align: center



"""



#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft() and gravity modules.  Illustrates
#           a 3-DOV spacecraft on a range of orbit types.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 26, 2016
#

import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
# To play with any scenario scripts as tutorials, you should make a copy of them into a custom folder
# outside of the Basilisk directory.
#
# To copy them, first find the location of the Basilisk installation.
# After installing, you can find the installed location of Basilisk by opening a python interpreter and
# running the commands:
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

# Copy the folder `{basiliskPath}/examples` into a new folder in a different directory.
# Now, when you want to use a tutorial, navigate inside that folder, and edit and execute the *copied* integrated tests.


# import simulation related support
from Basilisk.simulation import spacecraft
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport)

# always import the Basilisk messaging support

def run(show_plots):
    """
    Run a scenario with multiple orbital cases..

    Args:
        show_plots (bool): Determines if the script should display plots
    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # (Optional) If you want to see a simulation progress bar in the terminal window, the
    # use the following SetProgressBar(True) statement
    scSim.SetProgressBar(True)

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # setup the simulation tasks/objects
    scList = []
    scLEO = spacecraft.Spacecraft()
    scLEO.ModelTag = "bsk-LEO"
    scList.append(scLEO)

    scGeo = spacecraft.Spacecraft()
    scGeo.ModelTag = "bsk-GEO"
    scList.append(scGeo)

    scGTO = spacecraft.Spacecraft()
    scGTO.ModelTag = "bsk-GTO"
    scList.append(scGTO)

    scHyp = spacecraft.Spacecraft()
    scHyp.ModelTag = "bsk-Hyperbolic"
    scList.append(scHyp)

    scMolniya = spacecraft.Spacecraft()
    scMolniya.ModelTag = "bsk-Molniya"
    scList.append(scMolniya)

    for sc in scList:
        scSim.AddModelToTask(simTaskName, sc)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies('earth', 'sun', 'moon')
    gravBodies['earth'].isCentralBody = True
    mu = gravBodies['earth'].mu
    timeInitString = "2012 MAY 1 00:28:30.0"
    spiceObject = gravFactory.createSpiceInterface(time=timeInitString, epochInMsg=True)
    spiceObject.zeroBase = 'Earth'
    scSim.AddModelToTask(simTaskName, spiceObject)

    # add gravity to spacecraft dynamics
    for sc in scList:
        gravFactory.addBodiesTo(sc)

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    rGEO = 42164. * 1000     # meters
    for sc in scList:
        oe.f = 85.3 * macros.D2R
        if sc.ModelTag == 'bsk-GEO':
            oe.a = rGEO
            oe.e = 0.00001
            oe.i = 10.0 * macros.D2R
            oe.Omega = 0.0 * macros.D2R
            oe.omega = 0.0 * macros.D2R
        elif sc.ModelTag == 'bsk-GTO':
            oe.a = (rLEO + rGEO) / 2.0
            oe.e = 1.0 - rLEO / oe.a
            oe.i = 0.0 * macros.D2R
            oe.Omega = 148.2 * macros.D2R
            oe.omega = 47.8 * macros.D2R
        elif sc.ModelTag == 'bsk-Molniya':
            oe.a = 26000. * 1000   # meters
            oe.e = 0.74
            oe.i = 63.4 * macros.D2R
            oe.Omega = 48.2 * macros.D2R
            oe.omega = 275. * macros.D2R
        elif sc.ModelTag == 'bsk-Hyperbolic':
            oe.a = -40000. * 1000   # meters
            oe.e = 1.2
            oe.i = 33. * macros.D2R
            oe.Omega = 32. * macros.D2R
            oe.omega = 275. * macros.D2R
            oe.f = -120. *macros.D2R
        else:   # LEO case
            oe.a = rLEO
            oe.e = 0.0001
            oe.i = 33.3 * macros.D2R
            oe.Omega = 48.2 * macros.D2R
            oe.omega = 347.8 * macros.D2R
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
        sc.hub.r_CN_NInit = rN  # m   - r_BN_N
        sc.hub.v_CN_NInit = vN  # m/s - v_BN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(2 * P)

    numDataPoints = 300
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataList = []
    for sc in scList:
        dataRec = sc.scStateOutMsg.recorder(samplingTime)
        scSim.AddModelToTask(simTaskName, dataRec)
        dataList.append(dataRec)

    # set spacecraft line colors
    scLineColor = []
    scLineColor.append(vizSupport.toRGBA255("red"))
    scLineColor.append(vizSupport.toRGBA255("blue"))
    scLineColor.append(vizSupport.toRGBA255("cyan"))
    scLineColor.append(vizSupport.toRGBA255("yellow"))
    scLineColor.append(vizSupport.toRGBA255("green"))
    gtLineColor = [row.copy() for row in scLineColor]
    # make ground track color opaque
    for gt in gtLineColor:
        gt[3] = 60


    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scList,
                                                  # saveFile=__file__,
                                                  # liveStream=True,
                                                  groundTrackColorList=gtLineColor,
                                                  oscOrbitColorList=scLineColor,
                                                  )
        viz.settings.osculatingOrbitLineRange.push_back(-160.0 * macros.D2R)
        viz.settings.osculatingOrbitLineRange.push_back(160.0 * macros.D2R)
        viz.settings.osculatingGroundTrackRange.push_back(180.0 * macros.D2R)
        viz.settings.osculatingGroundTrackRange.push_back(180.0 * macros.D2R)
        viz.settings.showSpacecraftLabels = True
        viz.settings.mainCameraTarget = 'earth'
        viz.settings.spacecraftOrbitLineWidth = 2
        viz.settings.showOsculatingGroundTrackLines = True

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   retrieve and plot the logged data
    np.set_printoptions(precision=16)
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    scCounter = 0
    for data in dataList:
        posData = data.r_BN_N
        timeAxis =data.times()
        radii = np.linalg.norm(posData, axis=1)
        plt.semilogy(timeAxis * macros.NANO2HOUR, radii / 1000.,
                 color=np.array(scLineColor[scCounter])/255.,
                 label=scList[scCounter].ModelTag)
        scCounter += 1
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel('Orbit Radius [km]')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


if __name__ == "__main__":
    run(
        True        # show_plots
    )
