#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
This scenario mirrors ``examples/mujoco/scenarioStochasticDrag.py`` but uses
the ``spacecraft.Spacecraft`` dynamics model (``spacecraft.h`` based) together
with ``MeanRevertingNoiseStateEffector``.

The atmospheric density used by drag is:

.. math::

    \rho_{\text{stoch}} = \rho_{\text{exp}}(1 + \delta_\rho)

where :math:`\delta_\rho` is an Ornstein-Uhlenbeck process implemented as a
state in the state effector.

Illustration of Simulation Results
----------------------------------
The following images illustrate a possible simulation result.

The orbit is plotted in the orbital plane:

.. image:: /_images/Scenarios/scenarioStochasticDragSpacecraft_orbit.svg
   :align: center

The altitude as a function of time is plotted.

.. image:: /_images/Scenarios/scenarioStochasticDragSpacecraft_altitude.svg
   :align: center

The atmospheric density as a function of altitude is plotted in lin-log space.
This shows two lines: the deterministic, exponential density (should appear
linear); and the stochastic density.

.. image:: /_images/Scenarios/scenarioStochasticDragSpacecraft_density.svg
   :align: center

The atmospheric density correction, which should have a standard deviation
of 0.15.

.. image:: /_images/Scenarios/scenarioStochasticDragSpacecraft_densityDiff.svg
   :align: center

The magnitude of drag force over time is plotted in lin-log space.

.. image:: /_images/Scenarios/scenarioStochasticDragSpacecraft_drag.svg
   :align: center
"""
import os

import numpy as np
import matplotlib.pyplot as plt
from typing import Optional

from Basilisk.simulation import spacecraft
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.simulation import dragDynamicEffector
from Basilisk.simulation import meanRevertingNoiseStateEffector
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simSetPlanetEnvironment

fileName = os.path.basename(os.path.splitext(__file__)[0])

def run(showPlots: bool = False, rngSeed: Optional[int] = None):
    """
    Run the spacecraft-based stochastic drag scenario.

    Args:
        showPlots: If True, display figures.
        rngSeed: Optional stochastic integrator seed for reproducibility.
    Returns:
        Dict of matplotlib figure handles.
    """
    initialAlt = 250  # [km]
    planet = simIncludeGravBody.BODY_DATA["earth"]

    # Match the same initial orbit used by scenarioStochasticDrag.py
    oe = orbitalMotion.ClassicElements()
    oe.a = planet.radEquator + initialAlt * 1000.0 # [m]
    oe.e = 0.0
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(planet.mu, oe)
    oe = orbitalMotion.rv2elem(planet.mu, rN, vN)
    orbitPeriod = 2.0 * np.pi / np.sqrt(planet.mu / oe.a**3)

    dt = 10.0  # [s]
    tf = 7.1 * orbitPeriod  # [s]

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, macros.sec2nano(dt)))

    # Spacecraft setup (point-mass equivalent to the MuJoCo cannonball body)
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    scObject.hub.mHub = 1.0 # [kg]
    scObject.hub.IHubPntBc_B = np.identity(3) # [kg*m^2]
    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN
    scSim.AddModelToTask(simTaskName, scObject)

    # Stochastic integrator for states with diffusion
    integrator = svIntegrators.svStochasticIntegratorW2Ito2(scObject)
    if rngSeed is not None:
        integrator.setRNGSeed(rngSeed)
    scObject.setIntegrator(integrator)

    # Gravity model (point-mass Earth)
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    gravFactory.addBodiesTo(scObject)

    # Deterministic atmosphere
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.ModelTag = "ExpAtmo"
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    atmo.addSpacecraftToModel(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, atmo)

    # Stochastic atmospheric density correction state effector
    stochasticAtmo = meanRevertingNoiseStateEffector.MeanRevertingNoiseStateEffector()
    stochasticAtmo.ModelTag = "StochasticExpAtmo"
    stochasticAtmo.setStationaryStd(0.15)
    stochasticAtmo.setTimeConstant(1.8 * 60.0)  # [s]
    scObject.addStateEffector(stochasticAtmo)

    # Cannonball drag model
    drag = dragDynamicEffector.DragDynamicEffector()
    drag.ModelTag = "DragEff"
    drag.coreParams.dragCoeff = 2.2
    drag.coreParams.projectedArea = 10.0 # [m^2]
    drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])
    drag.densityCorrectionStateName = stochasticAtmo.getStateName()
    scObject.addDynamicEffector(drag)
    scSim.AddModelToTask(simTaskName, drag)

    # Recorders
    stateRecorder = scObject.scStateOutMsg.recorder()
    deterministicDensityRecorder = atmo.envOutMsgs[0].recorder()
    dragRecorder = drag.logger("forceExternal_B")

    scSim.AddModelToTask(simTaskName, stateRecorder)
    scSim.AddModelToTask(simTaskName, deterministicDensityRecorder)
    scSim.AddModelToTask(simTaskName, dragRecorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    figures = plotOrbits(
        timeAxis=stateRecorder.times(),
        posData=stateRecorder.r_BN_N,
        velData=stateRecorder.v_BN_N,
        dragForce=dragRecorder.forceExternal_B,
        deterministicDenseData=deterministicDensityRecorder.neutralDensity,
        oe=oe,
        mu=planet.mu,
        planetRadius=planet.radEquator,
        dragCoeff=drag.coreParams.dragCoeff,
        dragArea=drag.coreParams.projectedArea,
    )

    if showPlots:
        plt.show()

    return figures


def plotOrbits(timeAxis, posData, velData, dragForce, deterministicDenseData, oe, mu, planetRadius, dragCoeff, dragArea):
    """Plot orbit, altitude, atmosphere, correction, and drag."""
    figureList = {}

    timeHours = timeAxis * macros.NANO2HOUR

    figureList[fileName + "_orbit"], ax = plt.subplots()
    ax.axis("equal")
    ax.add_artist(plt.Circle((0, 0), planetRadius / 1000.0, color="#008800"))
    rData = []
    fData = []
    for idx in range(len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    ax.plot(
        np.array(rData) * np.cos(fData) / 1000.0,
        np.array(rData) * np.sin(fData) / 1000.0,
        color="#aa0000",
        linewidth=1.0
    )
    ax.set_xlabel("$i_e$ Cord. [km]")
    ax.set_ylabel("$i_p$ Cord. [km]")

    figureList[fileName + "_altitude"], ax = plt.subplots()
    ax.ticklabel_format(useOffset=False, style="plain")
    alt = (np.array(rData) - planetRadius) / 1000.0
    ax.plot(timeHours, alt)
    ax.set_xlabel("$t$ [h]")
    ax.set_ylabel("Altitude [km]")

    velNorm = np.linalg.norm(velData, axis=1)
    dragNorm = np.linalg.norm(dragForce, axis=1)
    denseData = 2.0 * dragNorm / (dragCoeff * dragArea * np.maximum(velNorm**2, 1e-30))

    figureList[fileName + "_density"], ax = plt.subplots()
    ax.semilogy(alt[1:], denseData[1:], label="Stochastic")
    ax.semilogy(alt[1:], deterministicDenseData[1:], label="Exponential")
    ax.legend(loc="upper right")
    ax.set_xlabel("Altitude [km]")
    ax.set_ylabel("$\\rho$ [kg/m$^3$]")

    figureList[fileName + "_densityDiff"], ax = plt.subplots()
    ax.plot(timeHours[1:], (denseData / deterministicDenseData)[1:] - 1.0)
    ax.set_xlabel("Time [hr]")
    ax.set_ylabel(r"$(\rho_{stoch} / \rho_{exp}) - 1$ [-]")

    figureList[fileName + "_drag"], ax = plt.subplots()
    ax.semilogy(timeHours[1:], np.linalg.norm(dragForce, 2, 1)[1:])
    ax.set_xlabel("$t$ [hr]")
    ax.set_ylabel("$|F_{drag}|$ [N]")

    return figureList


if __name__ == "__main__":
    run(True)
