#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario demonstrates how to set up a spacecraft orbiting Earth subject to atmospheric drag, causing it to
deorbit. This is achieved using the :ref:`exponentialAtmosphere` or :ref:`msisAtmosphere` environment module and the
:ref:`dragDynamicEffector` dynamics module. The simulation is executed until the altitude falls below some threshold,
using a terminal event handler.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioDragDeorbit.py

Simulation Scenario Setup Details
---------------------------------

A single simulation with a spacecraft object is created, along with the atmosphere, drag, and gravity models.

If the exponential model is selected, the atmosphere module ``ExponentialAtmosphere()`` is initialized as ``atmo``; to
set the model to use Earth-like values,
the utility::

    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")

is invoked.

If the msis model is selected, the atmosphere module ``MsisAtmosphere()`` is initialized as ``atmo``. Solar weather
data must be mocked in a message to the module, where ``sw_msg`` is a dict of reasonable ap and f10.7cm values::

    swMsgList = []
    for c, val in enumerate(sw_msg.values()):
        swMsgData = messaging.SwDataMsgPayload()
        swMsgData.dataValue = val
        swMsgList.append(messaging.SwDataMsg().write(swMsgData))
        atmo.swDataInMsgs[c].subscribeTo(swMsgList[-1])

The drag model ``DragDynamicEffector()`` is initialized, then model parameters are set. In this example, the projected
area ``coreParams.projectedArea`` is set to 10 meters squared and the drag coefficient :math:`C_D`
``coreParams.dragCoeff`` is set to 2.2.

Once the models have been added to the simulation task, the atmosphere, drag model, and spacecraft must be linked.
First, the atmosphere model is given the spacecraft state message so it knows the location for which to calculate
atmospheric conditions::

    atmo.addSpacecraftToModel(scObject.scStateOutMsg)

Then, the drag effector is linked to the spacecraft::

    scObject.addDynamicEffector(dragEffector)

The drag model will calculate zero drag unless it is passed atmospheric conditions. To link the atmosphere model to
the drag model::

    dragEffector.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])

Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a deorbit from 250 to 100 km with the
exponential model.

The orbit is plotted in the orbital plane:

.. image:: /_images/Scenarios/scenarioDragDeorbitexponential1.svg
   :align: center

The altitude as a function of time is plotted.

.. image:: /_images/Scenarios/scenarioDragDeorbitexponential2.svg
   :align: center

The atmospheric density as a function of altitude is plotted in lin-log space. Since this uses the exponential
atmosphere model, the result should be linear.

.. image:: /_images/Scenarios/scenarioDragDeorbitexponential3.svg
   :align: center

The magnitude of drag force over time is plotted in lin-log space.

.. image:: /_images/Scenarios/scenarioDragDeorbitexponential4.svg
   :align: center

The same plots are generated using the MSIS model:

.. image:: /_images/Scenarios/scenarioDragDeorbitmsis1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDragDeorbitmsis2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDragDeorbitmsis3.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDragDeorbitmsis4.svg
   :align: center


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Demonstration of deorbit using exponentialAtmosphere and dragDynamicEffector modules.
# Author:   Mark Stephenson
# Creation Date:  Aug. 31, 2022
#

import os
import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk, used to get the location of supporting data
from Basilisk import __path__
# always import the Basilisk messaging support
from Basilisk.architecture import messaging
# import atmosphere and drag modules
from Basilisk.simulation import exponentialAtmosphere, msisAtmosphere, dragDynamicEffector
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport, simSetPlanetEnvironment)

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, initialAlt=250, deorbitAlt=100, model="exponential"):
    """
    Initialize a satellite with drag and propagate until it falls below a deorbit altitude. Note that an excessively
    low deorbit_alt can lead to intersection with the Earth prior to deorbit being detected, causing some terms to blow
    up and the simulation to terminate.

    Args:
        show_plots (bool): Toggle plotting on/off
        initialAlt (float): Starting altitude in km
        deorbitAlt (float): Terminal altitude in km
        model (str): ["exponential", "msis"]

    Returns:
        Dictionary of figure handles
    """
    # Create simulation and dynamics process
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(15.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Initialize atmosphere model and add to sim
    if model == "exponential":
        atmo = exponentialAtmosphere.ExponentialAtmosphere()
        atmo.ModelTag = "ExpAtmo"
        simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    elif model == "msis":
        atmo = msisAtmosphere.MsisAtmosphere()
        atmo.ModelTag = "MsisAtmo"

        ap = 8
        f107 = 110
        sw_msg = {
            "ap_24_0": ap, "ap_3_0": ap, "ap_3_-3": ap, "ap_3_-6": ap, "ap_3_-9": ap,
            "ap_3_-12": ap, "ap_3_-15": ap, "ap_3_-18": ap, "ap_3_-21": ap, "ap_3_-24": ap,
            "ap_3_-27": ap, "ap_3_-30": ap, "ap_3_-33": ap, "ap_3_-36": ap, "ap_3_-39": ap,
            "ap_3_-42": ap, "ap_3_-45": ap, "ap_3_-48": ap, "ap_3_-51": ap, "ap_3_-54": ap,
            "ap_3_-57": ap, "f107_1944_0": f107, "f107_24_-24": f107
        }

        swMsgList = []
        for c, val in enumerate(sw_msg.values()):
            swMsgData = messaging.SwDataMsgPayload()
            swMsgData.dataValue = val
            swMsgList.append(messaging.SwDataMsg().write(swMsgData))
            atmo.swDataInMsgs[c].subscribeTo(swMsgList[-1])
    else:
        raise ValueError(f"{model} not a valid model!")

    scSim.AddModelToTask(simTaskName, atmo)

    # Initialize drag effector and add to sim
    projArea = 10.0  # drag area in m^2
    dragCoeff = 2.2  # drag coefficient
    dragEffector = dragDynamicEffector.DragDynamicEffector()
    dragEffector.ModelTag = "DragEff"
    dragEffectorTaskName = "drag"
    dragEffector.coreParams.projectedArea = projArea
    dragEffector.coreParams.dragCoeff = dragCoeff
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName, simulationTimeStep))
    scSim.AddModelToTask(dragEffectorTaskName, dragEffector)

    # Set up the spacecraft
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.spacecraft = scObject

    # Link spacecraft to drag model
    atmo.addSpacecraftToModel(scObject.scStateOutMsg)
    scObject.addDynamicEffector(dragEffector)
    # and drag model to atmosphere model
    dragEffector.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])

    # Set up gravity
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    mu = planet.mu
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Set up a circular orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = planet.radEquator + initialAlt * 1000  # meters
    oe.e = 0.0001
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)
    # this stores consistent initial orbit elements; with circular or equatorial orbit, some angles are arbitrary

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    # set the simulation time increments
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(100 * P)
    numDataPoints = 10000
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)

    # Setup data logging before the simulation is initialized
    dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    dataAtmoLog = atmo.envOutMsgs[0].recorder(samplingTime)
    forceLog = dragEffector.logger("forceExternal_B", samplingTime)

    scSim.AddModelToTask(simTaskName, dataRec)
    scSim.AddModelToTask(simTaskName, dataAtmoLog)
    scSim.AddModelToTask(simTaskName, forceLog)

    # Event to terminate the simulation
    scSim.createNewEvent("Deorbited", simulationTimeStep, True,
                         [f"np.linalg.norm(self.spacecraft.scStateOutMsg.read().r_BN_N) < {planet.radEquator + 1000 * deorbitAlt}"],
                         [], terminal=True)

    # Vizard Visualization Option
    # ---------------------------
    # If you wish to transmit the simulation data to the United based Vizard Visualization application,
    # then uncomment the following
    # line from the python scenario script.  This will cause the BSK simulation data to
    # be stored in a binary file inside the _VizFiles sub-folder with the scenario folder.  This file can be read in by
    # Vizard and played back after running the BSK simulation.
    # To enable this, uncomment this line:]
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                                              # saveFile=__file__
                                              # liveStream=True
                                              )

    # initialize Simulation
    scSim.InitializeSimulation()

    # Run the simulation
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve the logged data
    posData = dataRec.r_BN_N
    velData = dataRec.v_BN_N
    dragForce = forceLog.forceExternal_B
    denseData = dataAtmoLog.neutralDensity

    figureList = plotOrbits(dataRec.times(), posData, velData, dragForce, denseData, oe, mu, planet, model)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


def plotOrbits(timeAxis, posData, velData, dragForce, denseData, oe, mu, planet, model):
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    figureList = {}

    def register_fig(i):
        pltName = fileName + model + str(i)
        figureList[pltName] = plt.figure(i)
        fig = plt.gcf()
        ax = fig.gca()
        return fig, ax

    # draw orbit in perifocal frame
    b = oe.a * np.sqrt(1 - oe.e * oe.e)
    plt.figure(1, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
    plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
    # draw the planet
    fig, ax = register_fig(1)
    ax.axis('equal')
    planetColor = '#008800'
    planetRadius = planet.radEquator / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=1.0
             )
    plt.xlabel('$i_e$ Cord. [km]')
    plt.ylabel('$i_p$ Cord. [km]')

    # draw altitude as a function of time
    fig, ax = register_fig(2)
    ax.ticklabel_format(useOffset=False, style='plain')
    alt = np.array(rData) / 1000 - planetRadius
    plt.plot(timeAxis * macros.NANO2HOUR, alt)
    plt.xlabel('$t$ [h]')
    plt.ylabel('Alt. [km]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    # draw density as a function of altitude
    fig, ax = register_fig(3)
    plt.semilogy(alt, denseData)
    plt.xlabel('Alt. [km]')
    plt.ylabel('$\\rho$ [kg/m$^2$]')

    # draw drag as a function of time
    fig, ax = register_fig(4)
    plt.semilogy(timeAxis * macros.NANO2HOUR, np.linalg.norm(dragForce, 2, 1))
    plt.xlabel('$t$ [hr]')
    plt.ylabel('$|F_drag|$ [N]')

    return figureList


if __name__ == "__main__":
    run(
        show_plots=True,
        initialAlt=250,
        deorbitAlt=100,
        model="msis"   # "msis", "exponential"
    )
