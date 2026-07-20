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
r"""
This scenario uses :ref:`MJScene<MJScene>` as its ``DynamicObject``, although it is
not its main focus. For context on dynamics using :ref:`MJScene<MJScene>`, check out:

#. ``examples/mujoco/scenarioReactionWheel.py``

This script shows how to simulate a point-mass spacecraft using :ref:`MJScene<MJScene>`
(mujoco-based dynamics) with cannonball drag force and a stochastic atmospheric
density model. The script illustrates how one defines and handles dynamic states
that are driven by stochastic terms.

The spacecraft definition is given in ``CANNONBALL_SCENE_XML``; it contains a
single body for which we set its mass directly.  A
:ref:`simIncludeGravBody` gravity factory creates the point-mass Earth and
attaches it to the MuJoCo scene.

The drag force acting on the body is computed in the :ref:`CannonballDrag<CannonballDrag>`
module. This takes as input the state of the spacecraft (so that
its velocity and orientation) and the atmospheric density. It outputs a force vector
on the body-fixed reference frame (more accurately, the body's center of mass frame/site).

The atmospheric density used on the drag model is given by:

.. math::

    \rho_{\text{stoch}} = \rho_{\text{exp}} \left(1 + \delta_\rho\right)

where ``densityExponential`` is computed by the :ref:`exponentialAtmosphere` model
while ``densityCorrection`` (written above as :math:`\delta_\rho`) is a stochastic process centered at zero. This process
is modeled as an Ornstein-Uhlenbeck (OU) process, whose Stochastic Differential
Equation (SDE) is given by:

.. math::

    d\delta_\rho = -\theta \delta_\rho\,dt + \sigma\,dW

where ``theta`` and ``sigma`` are the terms that characterize the OU process.
In the SDE above, ``-theta*densityCorrection`` represents the 'drift' term (the
classical derivative). ``sigma``, on the other hand, represents the 'diffusion'
term, which maps the influence of the noise to the state.

The ``densityStochastic`` and ``densityCorrection`` are computed in
:ref:`StochasticAtmDensity<stochasticAtmDensity>`, which is based on
:ref:`MeanRevertingNoise<meanRevertingNoise>`.

Alternatively, running with ``useIgbm=True`` models the density factor with an
inhomogeneous geometric Brownian motion (IGBM) instead, using
:ref:`IgbmAtmDensity<igbmAtmDensity>` (based on
:ref:`InhomogeneousGeometricBrownianMotion<inhomogeneousGeometricBrownianMotion>`):

.. math::

    \rho_{\text{stoch}} = \rho_{\text{exp}}\,x, \qquad
    dx = \frac{1}{\tau}(\mu - x)\,dt + \sigma x\,dW

with mean level :math:`\mu = 1`. The IGBM noise is multiplicative and its exact process
is a positive, mean-reverting factor :math:`x`; :ref:`IgbmAtmDensity<igbmAtmDensity>`
clamps the corrected density to be non-negative to guard against the rare non-positive
:math:`x` an explicit integrator can produce. Both models are configured with the same
stationary standard deviation and time constant, so the two runs are statistically
comparable.

Illustration of Simulation Results
----------------------------------
The following images illustrate a possible simulation result.

The orbit is plotted in the orbital plane:

.. image:: /_images/Scenarios/scenarioStochasticDrag_orbit.svg
   :align: center

The altitude as a function of time is plotted.

.. image:: /_images/Scenarios/scenarioStochasticDrag_altitude.svg
   :align: center

The atmospheric density as a function of altitude is plotted in lin-log space.
This shows two lines: the deterministic, exponential density (should appear
linear); and the stochastic density.

.. image:: /_images/Scenarios/scenarioStochasticDrag_density.svg
   :align: center

The atmospheric density correction, which should have a standard deviation
of 0.15.

.. image:: /_images/Scenarios/scenarioStochasticDrag_densityDiff.svg
   :align: center

The magnitude of drag force over time is plotted in lin-log space.

.. image:: /_images/Scenarios/scenarioStochasticDrag_drag.svg
   :align: center

Illustration of Simulation Results with IGBM
--------------------------------------------
The following images illustrate a possible simulation result with ``useIgbm=True``
(figure names gain an ``igbm`` tag). The overall trajectory is statistically similar to
the OU case since both processes share the same stationary standard deviation and time
constant; the qualitative difference is in the density factor, which is strictly
positive under IGBM.

The atmospheric density as a function of altitude:

.. image:: /_images/Scenarios/scenarioStochasticDrag_igbm_density.svg
   :align: center

The atmospheric density correction :math:`x - 1`, which should have a standard
deviation of 0.15 about a positive mean-reverting factor :math:`x`:

.. image:: /_images/Scenarios/scenarioStochasticDrag_igbm_densityDiff.svg
   :align: center

The magnitude of drag force over time:

.. image:: /_images/Scenarios/scenarioStochasticDrag_igbm_drag.svg
   :align: center

"""
import os

from Basilisk.architecture import messaging
from Basilisk.simulation import mujoco
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.simulation import cannonballDrag
from Basilisk.simulation import MJStochasticAtmDensity
from Basilisk.simulation import MJIgbmAtmDensity
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simSetPlanetEnvironment
from Basilisk.utilities import vizSupport

import numpy as np
import matplotlib.pyplot as plt

"""A cannon-ball spacecraft.

We set its inertial properties directly, instead of defining them
through their geometry. The spacecraft has no sub-bodies or any
moving parts.
"""
CANNONBALL_SCENE_XML = r"""
<mujoco>
	<worldbody>
		<body name="ball">
			<freejoint/>
			<inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
		</body>
	</worldbody>
</mujoco>
"""

fileName = os.path.basename(os.path.splitext(__file__)[0])

def run(showPlots: bool = False, useIgbm: bool = False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): If True, simulation results are plotted and show.
            Defaults to False.
        useIgbm (bool, optional): If True, model the stochastic density factor with an
            inhomogeneous geometric Brownian motion (multiplicative noise, strictly
            positive) instead of the additive Ornstein-Uhlenbeck correction.
            Defaults to False.
    """
    initialAlt = 250 # [km]
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True

    # Set up a circular orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = planet.radEquator + initialAlt * 1000  # [m]
    oe.e = 0
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(planet.mu, oe)
    oe = orbitalMotion.rv2elem(planet.mu, rN, vN)
    orbitPeriod = 2. * np.pi / np.sqrt(planet.mu / oe.a**3)

    dt = 10 # [s]
    tf = 7.1 * orbitPeriod # [s]

    # Create a simulation, process, and task as usual
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create the Mujoco scene (our MuJoCo DynamicObject)
    # Load the XML file that defines the system from a file
    scene = mujoco.MJScene(CANNONBALL_SCENE_XML)
    scSim.AddModelToTask("test", scene)

    # Set a stochastic integrator on the DynamicObject, necessary since we have
    # states (related to density) that are driven by stochastic dynamics.
    # This scenario looks at a single sample trajectory, so a STRONG integrator is used
    # (strong order = pathwise accuracy; weak order only controls statistics across many
    # samples). SOSRA is the recommended strong method for the OU correction (additive
    # noise); the IGBM factor has multiplicative noise, which SOSRA does not support, so
    # it uses SRIW1 (strong order 1.5 for diagonal/scalar noise).
    if useIgbm:
        integ = svIntegrators.svStochasticIntegratorSRIW1(scene)
    else:
        integ = svIntegrators.svStochasticIntegratorSOSRA(scene)
    scene.setIntegrator(integ)

    ### Get mujoco body and site (point, frame) of interest
    body: mujoco.MJBody = scene.getBody("ball")
    dragPoint: mujoco.MJSite = body.getCenterOfMass()

    ### Add Earth gravity to the MuJoCo scene
    # The factory creates the NBodyGravity model and registers the cannonball
    # body as a gravity target.
    gravFactory.addBodiesTo(scene)

    ### Create the density model
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.ModelTag = "ExpAtmo"
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    atmo.addSpacecraftToModel(dragPoint.stateOutMsg)

    # Will be updated with the task period
    scSim.AddModelToTask("test", atmo)

    if useIgbm:
        # Multiplicative IGBM factor reverting to 1 (IgbmAtmDensity clamps the corrected
        # density to be non-negative)
        stochasticAtmo = MJIgbmAtmDensity.IgbmAtmDensity()
        stochasticAtmo.setMean(1.0)
    else:
        # Additive Ornstein-Uhlenbeck correction centered at zero
        stochasticAtmo = MJStochasticAtmDensity.StochasticAtmDensity()
    # Both models are configured in stationary form with the same statistics
    stochasticAtmo.setStationaryStd(0.15)
    stochasticAtmo.setTimeConstant(1.8 * 60)  # [s]
    stochasticAtmo.ModelTag = "StochasticExpAtmo"

    stochasticAtmo.atmoDensInMsg.subscribeTo( atmo.envOutMsgs[0] )

    # StochasticAtmDensity is a StatefulSysModel with a state
    # with both drift (regular derivative) and diffusion (stochastic)
    # so we need to add to both DynamicsTask and DiffusionDynamicsTask
    scene.AddModelToDynamicsTask(stochasticAtmo)
    scene.AddModelToDiffusionDynamicsTask(stochasticAtmo)

    ### Create drag force model
    drag = cannonballDrag.CannonballDrag()
    drag.ModelTag = "DragEff"
    dragGeometry = messaging.DragGeometryMsgPayload()
    dragGeometry.dragCoeff = 2.2
    dragGeometry.projectedArea = 10.0 # [m^2]
    dragGeometry.r_CP_S = [0.0, 0.0, 0.0] # [m]
    dragGeometryMsg = messaging.DragGeometryMsg().write(dragGeometry)

    drag.dragGeometryInMsg.subscribeTo(dragGeometryMsg)
    drag.referenceFrameStateInMsg.subscribeTo(dragPoint.stateOutMsg)
    drag.atmoDensInMsg.subscribeTo( stochasticAtmo.atmoDensOutMsg )

    # The CannonballDrag model will produce a force vector message.
    # We must connect it to an actuator so that the force is applied
    dragActuator: mujoco.MJForceActuator = scene.addForceActuator( "dragForce", dragPoint )
    dragActuator.forceInMsg.subscribeTo( drag.forceOutMsg )

    # Must be added to the dynamics task because it impacts the
    # dynamics of the scene (sets a force on a body)
    scene.AddModelToDynamicsTask(drag)

    ### Add recorders
    # Record the state of the 'ball' body through
    # the ``stateOutMsg`` of its 'origin' site (i.e. frame).
    bodyStateRecorder = body.getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    deterministicDensityRecorder = stochasticAtmo.atmoDensInMsg.recorder()
    scSim.AddModelToTask("test", deterministicDensityRecorder)

    densityRecorder = stochasticAtmo.atmoDensOutMsg.recorder()
    scSim.AddModelToTask("test", densityRecorder)

    dragRecorder = drag.forceOutMsg.recorder()
    scSim.AddModelToTask("test", dragRecorder)

    if vizSupport.vizFound:
        vizSupport.enableUnityVisualization(
            scSim,
            "test",
            scene,
            # saveFile=__file__,
        )

    # Initialize the simulation
    scSim.InitializeSimulation()

    # Set initial position and velocity
    body.setPosition(rN)
    body.setVelocity(vN)

    # Run the simulation
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    figures = plotOrbits(
        timeAxis=bodyStateRecorder.times(),
        posData=bodyStateRecorder.r_BN_N,
        velData=bodyStateRecorder.v_BN_N,
        dragForce=dragRecorder.force_S,
        deterministicDenseData=deterministicDensityRecorder.neutralDensity,
        denseData=densityRecorder.neutralDensity,
        oe=oe,
        mu=planet.mu,
        planetRadius=planet.radEquator,
        figureTag="igbm" if useIgbm else "",
    )

    if showPlots:
        plt.show()

    return figures

def plotOrbits(timeAxis, posData, velData, dragForce, deterministicDenseData, denseData, oe, mu, planetRadius,
               figureTag=""):
    """
    Plot the results of the stochastic drag simulation, including orbit, altitude,
    density, density difference, and drag force over time.

    Args:
        timeAxis: Array of time values.
        posData: Position data array.
        velData: Velocity data array.
        dragForce: Drag force data array.
        deterministicDenseData: Deterministic atmospheric density data.
        denseData: Stochastic atmospheric density data.
        oe: Classical orbital elements object.
        mu: Gravitational parameter.
        planetRadius: Radius of the planet.
        figureTag: Optional tag inserted into figure names (e.g. "igbm"), so the
            variants get their own documentation images.
    Returns:
        Dictionary of matplotlib figure objects.
    """
    # draw the inertial position vector components
    figureList = {}
    baseName = fileName + (f"_{figureTag}" if figureTag else "")

    figureList[baseName + "_orbit"], ax = plt.subplots()
    ax.axis('equal')
    planetColor = '#008800'
    ax.add_artist(plt.Circle((0, 0), planetRadius / 1000, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    ax.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=1.0
             )
    ax.set_xlabel('$i_e$ Cord. [km]')
    ax.set_ylabel('$i_p$ Cord. [km]')

    # draw altitude as a function of time
    figureList[baseName + "_altitude"], ax = plt.subplots()
    ax.ticklabel_format(useOffset=False, style='plain')
    alt = (np.array(rData) - planetRadius) / 1000
    ax.plot(timeAxis * macros.NANO2HOUR, alt)
    ax.set_xlabel('$t$ [h]')
    ax.set_ylabel('Altitude [km]')

    # draw density as a function of altitude
    figureList[baseName + "_density"], ax = plt.subplots()
    ax.semilogy(alt, denseData, label="Stochastic")
    ax.semilogy(alt, deterministicDenseData, label="Exponential")
    ax.legend(loc="upper right")
    ax.set_xlabel('Altitude [km]')
    ax.set_ylabel('$\\rho$ [kg/m$^3$]')

    # draw density as a function of altitude
    figureList[baseName + "_densityDiff"], ax = plt.subplots()
    ax.plot(timeAxis * macros.NANO2HOUR, (denseData / deterministicDenseData) - 1)
    ax.set_xlabel('Time [hr]')
    ax.set_ylabel(r'$(\rho_{stoch} / \rho_{exp}) - 1$ [-]')

    # draw drag as a function of time
    figureList[baseName + "_drag"], ax = plt.subplots()
    ax.semilogy(timeAxis * macros.NANO2HOUR, np.linalg.norm(dragForce, 2, 1))
    ax.set_xlabel('$t$ [hr]')
    ax.set_ylabel('$|F_drag|$ [N]')

    return figureList

if __name__ == "__main__":
    run(True)
