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
Overview
--------

This tutorial demonstrates how to add Earth and Moon gravity to a
:ref:`MJScene<MJScene>` with :ref:`simIncludeGravBody`.  The same MuJoCo
spacecraft and gravity-factory setup can be driven by either:

* :ref:`spiceInterface`, which reads high-fidelity ephemerides from NAIF
  kernels, or
* :ref:`planetEphemeris`, which propagates user-supplied heliocentric
  classical orbit elements with two-body Keplerian motion.

Select the ephemeris source through the ``useSpice`` argument to ``run()``.
The default is ``True``.  For example, the analytic ephemeris case is run with::

    run(showPlots=True, useSpice=False)

Gravity-factory setup
---------------------

Both cases begin by creating the same shared gravity-body descriptors.  Earth
is marked as the central body because the MuJoCo spacecraft state is initialized
relative to Earth::

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    moon = gravFactory.createMoon()

Each descriptor contains the body's gravity model, gravitational parameter,
central-body flag, and a planet-state input message.  The only difference
between the two cases is which ephemeris output is connected to those input
messages.

Using SPICE planet states
-------------------------

The SPICE helper uses the bodies already present in the factory to configure
and connect its output messages::

    spiceObject = gravFactory.createSpiceInterface(
        time="2025 NOVEMBER 15 12:00:00.000",
        epochInMsg=True,
    )
    spiceObject.zeroBase = "Earth"
    scene.AddModelToDynamicsTask(spiceObject, 75)

The factory creates the SPICE output messages in gravity-body insertion order,
so the first output is Earth and the second is the Moon in this example.
``createSpiceInterface()`` subscribes each gravity body's
``planetBodyInMsg`` to the corresponding output automatically.

Setting ``zeroBase`` to Earth makes the planet states and the spacecraft state
Earth-relative.  This is convenient but is not required by
:ref:`NBodyGravity<NBodyGravity>`; it only requires all gravity-source states
to use one consistent inertial origin.

Using analytic ``PlanetEphemeris`` states
-----------------------------------------

The analytic alternative names its output bodies and supplies one
heliocentric classical-element set for each body::

    planetObject = planetEphemeris.PlanetEphemeris()
    planetObject.setPlanetNames(
        planetEphemeris.StringVector(["earth", "moon"])
    )
    planetObject.planetElements = planetEphemeris.classicElementVector(
        [earthElements, moonElements]
    )
    planetObject.zeroBase = "earth"
    earth.planetBodyInMsg.subscribeTo(planetObject.planetOutMsgs[0])
    moon.planetBodyInMsg.subscribeTo(planetObject.planetOutMsgs[1])
    scene.AddModelToDynamicsTask(planetObject, 75)

Unlike the factory's SPICE helper, a separately created ``PlanetEphemeris``
module does not know which gravity descriptors it should drive.  Its output
messages are therefore connected explicitly.

``PlanetEphemeris`` propagates every supplied element set independently about
the Sun.  It does not implement a hierarchical Earth-Moon model.  To provide a
reasonable local approximation, this example constructs an initial
heliocentric Moon state by adding a geocentric lunar state to Earth's
heliocentric state, then converts the result to heliocentric elements.  This is
useful for short examples and deterministic tests.  SPICE should generally be
preferred when accurate Earth-Moon geometry over long intervals is required.

Setting ``zeroBase`` to Earth translates both outputs after propagation.  The
Earth message is therefore zero and the Moon message is Earth-relative.  This
keeps the analytic planet messages in the same frame as the Earth-relative
MuJoCo state, which is required for correct Vizard placement.  The selected
zero-base body must be configured on this same ``PlanetEphemeris`` instance.

Execution ordering and committed-state outputs
----------------------------------------------

The ephemeris execution order is critical.  Basilisk executes models with
higher numeric priorities first; model insertion order in the Python script
does not determine execution order.  This example explicitly enforces the
following sequence::

    Top-level task:
        MJScene (0) -> recorders and Vizard

    MJScene dynamics task:
        forward kinematics (10000) -> ephemeris (75) -> NBodyGravity (-1)

MuJoCo's adaptive integrator evaluates dynamics between top-level task ticks.
The ephemeris must therefore run in the scene dynamics task before
``NBodyGravity`` so that every force evaluation uses planet positions for the
current integrator time::

    scene.AddModelToDynamicsTask(
        ephemeris, EPHEMERIS_PRIORITY
    )

Integrator stages use provisional states.  Without an additional evaluation,
messages written by dynamics-task models can therefore describe the last
integrator stage rather than the state committed at the end of the step.  This
example enables::

    scene.extraEoMCall = True

After integration, ``MJScene`` then executes its dynamics task once more at the
top-level task time using the committed MuJoCo state.  This final evaluation
refreshes the forward-kinematics, ephemeris, and gravity outputs before the
recorders and Vizard execute.  It does not advance or otherwise change the
integrated state.  Because this final dynamics-task evaluation supplies the
current ephemeris messages, the ephemeris module does not also need to be added
to the top-level task.

Attaching gravity to MuJoCo
---------------------------

After either ephemeris has been configured, one factory call completes the
MuJoCo gravity setup::

    gravity = gravFactory.addBodiesTo(scene)

The factory creates one :ref:`NBodyGravity<NBodyGravity>` model, adds Earth and
the Moon as gravity sources, and adds every MuJoCo body as a gravity target.
For the articulated CubeSat used here, gravity is applied at the center of mass
of both the hub and the reaction wheel.

Illustration of Simulation Results
----------------------------------

The scenario test runs both ephemeris cases and saves three figures per case.
The first shows the spacecraft trajectory in its orbital plane, including the
Earth surface and the required :math:`R_E + 400` km minimum-radius boundary.
The initial orbit has a 700 km periapsis altitude, and the scenario checks the
recorded three-dimensional radius to ensure it remains above the 400 km
boundary.  The second figure shows the Earth-Moon geometry during the
simulation, and the third shows osculating spacecraft elements and the
magnitude of the Moon's third-body acceleration.

.. image:: /_images/Scenarios/scenarioMJEarthMoonGravity_spice_orbit.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMJEarthMoonGravity_spice_moonGeometry.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMJEarthMoonGravity_spice_diagnostics.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMJEarthMoonGravity_planetEphemeris_orbit.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMJEarthMoonGravity_planetEphemeris_moonGeometry.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMJEarthMoonGravity_planetEphemeris_diagnostics.svg
   :align: center
"""

import os
from typing import Any, Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.simulation import mujoco
from Basilisk.simulation import planetEphemeris
from Basilisk.simulation import svIntegrators
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import vizSupport


CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = os.path.join(CURRENT_FOLDER, "sat_w_wheel.xml")
FILE_NAME = os.path.basename(os.path.splitext(__file__)[0])

TASK_NAME = "simTask"
PROCESS_NAME = "simProcess"
TASK_STEP_SEC = 30.0  # [s]
SIMULATION_DURATION_SEC = 6.0 * 3600.0  # [s]
MINIMUM_SAFE_ALTITUDE_M = 400_000.0  # [m]
INITIAL_PERIAPSIS_ALTITUDE_M = 700_000.0  # [m]
INITIAL_ECCENTRICITY = 0.001  # [-]
SPICE_EPOCH = "2025 NOVEMBER 15 12:00:00.000"
EPHEMERIS_PRIORITY = 75
SCENE_PRIORITY = 0
RECORDER_PRIORITY = -100


def _copy_classic_elements(
    source: orbitalMotion.ClassicElements,
) -> planetEphemeris.ClassicElements:
    """Copy orbital elements into the SWIG type used by ``PlanetEphemeris``.

    Args:
        source (orbitalMotion.ClassicElements): Elements to copy.

    Returns:
        planetEphemeris.ClassicElements: Equivalent element set accepted by
        :ref:`planetEphemeris`.
    """
    destination = planetEphemeris.ClassicElements()
    destination.a = source.a
    destination.e = source.e
    destination.i = source.i
    destination.Omega = source.Omega
    destination.omega = source.omega
    destination.f = source.f
    return destination


def _create_analytic_ephemeris(
    earth: Any,
    moon: Any,
) -> Tuple[planetEphemeris.PlanetEphemeris, List[Any]]:
    """Create and connect an analytic Earth-Moon ephemeris.

    ``PlanetEphemeris`` expects heliocentric elements for every output.  The
    Moon's initial heliocentric state is therefore formed by adding a simple
    geocentric lunar state to a heliocentric Earth state.

    Args:
        earth (Any): Earth's shared ``GravBodyData`` descriptor.
        moon (Any): The Moon's shared ``GravBodyData`` descriptor.

    Returns:
        Tuple[planetEphemeris.PlanetEphemeris, List[Any]]: Configured module
        and its Earth/Moon output messages.
    """
    sunMu = orbitalMotion.MU_SUN * 1.0e9  # [m^3/s^2]

    earthElements = planetEphemeris.ClassicElements()
    earthElements.a = orbitalMotion.AU * 1000.0  # [m]
    earthElements.e = 0.0167086  # [-]
    earthElements.i = 0.00005 * macros.D2R  # [deg] -> [rad]
    earthElements.Omega = -11.26064 * macros.D2R  # [deg] -> [rad]
    earthElements.omega = 102.94719 * macros.D2R  # [deg] -> [rad]
    earthElements.f = 55.0 * macros.D2R  # [deg] -> [rad]

    moonAboutEarth = orbitalMotion.ClassicElements()
    moonAboutEarth.a = 384_400_000.0  # [m]
    moonAboutEarth.e = 0.0549  # [-]
    moonAboutEarth.i = 5.145 * macros.D2R  # [deg] -> [rad]
    moonAboutEarth.Omega = 125.08 * macros.D2R  # [deg] -> [rad]
    moonAboutEarth.omega = 318.15 * macros.D2R  # [deg] -> [rad]
    moonAboutEarth.f = 35.0 * macros.D2R  # [deg] -> [rad]

    earthPosition, earthVelocity = orbitalMotion.elem2rv(sunMu, earthElements)
    moonRelativePosition, moonRelativeVelocity = orbitalMotion.elem2rv(
        earth.mu, moonAboutEarth
    )
    moonHeliocentricElements = orbitalMotion.rv2elem(
        sunMu,
        earthPosition + moonRelativePosition,
        earthVelocity + moonRelativeVelocity,
    )

    ephemeris = planetEphemeris.PlanetEphemeris()
    ephemeris.ModelTag = "EarthMoonPlanetEphemeris"
    ephemeris.setPlanetNames(
        planetEphemeris.StringVector(["Earth", "Moon"])
    )
    ephemeris.planetElements = planetEphemeris.classicElementVector(
        [earthElements, _copy_classic_elements(moonHeliocentricElements)]
    )

    # A stand-alone PlanetEphemeris module cannot infer which factory body
    # corresponds to each output, so make the two subscriptions explicitly.
    earth.planetBodyInMsg.subscribeTo(ephemeris.planetOutMsgs[0])
    moon.planetBodyInMsg.subscribeTo(ephemeris.planetOutMsgs[1])
    return ephemeris, list(ephemeris.planetOutMsgs)


def _create_figures(
    useSpice: bool,
    earth: Any,
    moon: Any,
    spacecraftRecorder: Any,
    earthRecorder: Any,
    moonRecorder: Any,
) -> Dict[str, plt.Figure]:
    """Create the tutorial's orbit, geometry, and diagnostic figures.

    Args:
        useSpice (bool): If ``True``, label results as the SPICE case.
        earth (Any): Earth's gravity-body descriptor.
        moon (Any): The Moon's gravity-body descriptor.
        spacecraftRecorder (Any): MuJoCo hub state recorder.
        earthRecorder (Any): Earth ephemeris recorder.
        moonRecorder (Any): Moon ephemeris recorder.

    Returns:
        Dict[str, plt.Figure]: Figures keyed by documentation image name.
    """
    modeKey = "spice" if useSpice else "planetEphemeris"
    modeLabel = "SPICE" if useSpice else "PlanetEphemeris"
    figurePrefix = f"{FILE_NAME}_{modeKey}"

    timeHours = spacecraftRecorder.times() * macros.NANO2HOUR
    spacecraftPosition = np.asarray(spacecraftRecorder.r_BN_N)
    spacecraftVelocity = np.asarray(spacecraftRecorder.v_BN_N)
    moonRelativePosition = (
        np.asarray(moonRecorder.PositionVector)
        - np.asarray(earthRecorder.PositionVector)
    )

    semiMajorAxis = np.empty(len(spacecraftPosition))
    eccentricity = np.empty(len(spacecraftPosition))
    orbitRadius = np.empty(len(spacecraftPosition))
    orbitAngle = np.empty(len(spacecraftPosition))
    for index, (position, velocity) in enumerate(
        zip(spacecraftPosition, spacecraftVelocity)
    ):
        elements = orbitalMotion.rv2elem(earth.mu, position, velocity)
        semiMajorAxis[index] = elements.a
        eccentricity[index] = elements.e
        orbitRadius[index] = elements.rmag
        orbitAngle[index] = elements.omega + elements.f

    # This is the Moon's direct acceleration on the spacecraft minus the
    # acceleration it gives Earth, expressed in the Earth-centered frame.
    moonToSpacecraft = moonRelativePosition - spacecraftPosition
    moonDirect = (
        moon.mu
        * moonToSpacecraft
        / np.linalg.norm(moonToSpacecraft, axis=1)[:, None] ** 3
    )
    moonIndirect = (
        moon.mu
        * moonRelativePosition
        / np.linalg.norm(moonRelativePosition, axis=1)[:, None] ** 3
    )
    lunarThirdBodyAcceleration = np.linalg.norm(
        moonDirect - moonIndirect, axis=1
    )

    plt.close("all")
    figures: Dict[str, plt.Figure] = {}

    earthRadiusKm = earth.radEquator / 1000.0  # [km]
    minimumSafeRadiusKm = (
        earth.radEquator + MINIMUM_SAFE_ALTITUDE_M
    ) / 1000.0  # [km]
    orbitRadiusKm = orbitRadius / 1000.0  # [km]

    orbitFigure, orbitAxis = plt.subplots(figsize=(6.0, 5.0))
    earthDisk = plt.Circle(
        (0.0, 0.0),
        earthRadiusKm,
        color="#2a6fbb",
        alpha=0.8,
        label="Earth",
    )
    orbitAxis.add_patch(earthDisk)
    minimumRadiusBoundary = plt.Circle(
        (0.0, 0.0),
        minimumSafeRadiusKm,
        color="#e9c46a",
        fill=False,
        linestyle="--",
        linewidth=2.0,
        label=r"Minimum radius ($R_E+400$ km)",
    )
    orbitAxis.add_patch(minimumRadiusBoundary)
    orbitAxis.plot(
        orbitRadiusKm * np.cos(orbitAngle),
        orbitRadiusKm * np.sin(orbitAngle),
        color="#bb3e03",
        linewidth=1.0,
        label="MuJoCo hub",
    )
    orbitAxis.set_aspect("equal", adjustable="box")
    orbitAxis.set_xlabel(r"$r_x$ [km]")
    orbitAxis.set_ylabel(r"$r_y$ [km]")
    orbitAxis.set_title(
        f"MuJoCo trajectory in the orbital plane ({modeLabel})"
    )
    orbitAxis.legend(loc="best")
    orbitAxis.grid(True, alpha=0.3)
    orbitFigure.tight_layout()
    figures[f"{figurePrefix}_orbit"] = orbitFigure

    geometryFigure, geometryAxes = plt.subplots(
        1,
        2,
        figsize=(10.0, 4.5),
    )
    geometryAxes[0].plot(
        moonRelativePosition[:, 0] / 1.0e6,
        moonRelativePosition[:, 1] / 1.0e6,
        color="#6a4c93",
        linewidth=2.0,
        label="Moon trajectory",
    )
    geometryAxes[0].scatter(
        moonRelativePosition[0, 0] / 1.0e6,
        moonRelativePosition[0, 1] / 1.0e6,
        color="#2a9d8f",
        marker="o",
        label="Start",
        zorder=3,
    )
    geometryAxes[0].scatter(
        moonRelativePosition[-1, 0] / 1.0e6,
        moonRelativePosition[-1, 1] / 1.0e6,
        color="#e76f51",
        marker="x",
        label="End",
        zorder=3,
    )
    geometryAxes[0].scatter(
        0.0,
        0.0,
        color="#2a6fbb",
        marker="o",
        label="Earth",
        zorder=3,
    )
    geometryAxes[0].set_aspect("equal", adjustable="datalim")
    geometryAxes[0].set_xlabel(r"Earth-Moon $x$ [$10^3$ km]")
    geometryAxes[0].set_ylabel(r"Earth-Moon $y$ [$10^3$ km]")
    geometryAxes[0].set_title("Earth-centered geometry")
    geometryAxes[0].legend(loc="best")
    geometryAxes[0].grid(True, alpha=0.3)

    moonPositionChange = (
        moonRelativePosition - moonRelativePosition[0]
    ) / 1000.0  # [m] -> [km]
    geometryAxes[1].plot(
        moonPositionChange[:, 0],
        moonPositionChange[:, 1],
        color="#6a4c93",
        linewidth=2.0,
    )
    geometryAxes[1].scatter(
        moonPositionChange[0, 0],
        moonPositionChange[0, 1],
        color="#2a9d8f",
        marker="o",
        zorder=3,
    )
    geometryAxes[1].scatter(
        moonPositionChange[-1, 0],
        moonPositionChange[-1, 1],
        color="#e76f51",
        marker="x",
        zorder=3,
    )
    geometryAxes[1].set_aspect("equal", adjustable="datalim")
    geometryAxes[1].set_xlabel(r"$\Delta r_x$ [km]")
    geometryAxes[1].set_ylabel(r"$\Delta r_y$ [km]")
    geometryAxes[1].set_title("Lunar motion from initial position")
    geometryAxes[1].grid(True, alpha=0.3)
    geometryFigure.suptitle(
        f"Earth-Moon geometry over 6 hours ({modeLabel})"
    )
    geometryFigure.tight_layout()
    figures[f"{figurePrefix}_moonGeometry"] = geometryFigure

    diagnosticsFigure, diagnosticAxes = plt.subplots(
        3,
        1,
        figsize=(7.0, 7.0),
        sharex=True,
    )
    diagnosticAxes[0].plot(
        timeHours,
        semiMajorAxis - semiMajorAxis[0],
        color="#005f73",
    )
    diagnosticAxes[0].set_ylabel(r"$a-a_0$ [m]")
    diagnosticAxes[1].plot(
        timeHours,
        eccentricity,
        color="#ca6702",
    )
    diagnosticAxes[1].set_ylabel("Eccentricity [-]")
    diagnosticAxes[2].plot(
        timeHours,
        lunarThirdBodyAcceleration,
        color="#6a4c93",
    )
    diagnosticAxes[2].set_ylabel(r"$|\mathbf{a}_{Moon,3B}|$ [m/s$^2$]")
    diagnosticAxes[2].set_xlabel("Time [h]")
    for axis in diagnosticAxes:
        axis.grid(True, alpha=0.3)
    diagnosticsFigure.suptitle(
        f"Osculating orbit and lunar perturbation ({modeLabel})"
    )
    diagnosticsFigure.tight_layout()
    figures[f"{figurePrefix}_diagnostics"] = diagnosticsFigure

    return figures


def run(
    showPlots: bool = False,
    useSpice: bool = True,
) -> Dict[str, plt.Figure]:
    """Run the Earth-Moon gravity tutorial.

    Args:
        showPlots (bool, optional): If ``True``, display the generated figures.
            Defaults to ``False``.
        useSpice (bool, optional): If ``True``, use SPICE planet states. If
            ``False``, use analytic ``PlanetEphemeris`` states. Defaults to
            ``True``.

    Returns:
        Dict[str, plt.Figure]: Generated figures keyed by documentation image
        name.
    """
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess(PROCESS_NAME)
    process.addTask(
        simulation.CreateNewTask(
            TASK_NAME,
            macros.sec2nano(TASK_STEP_SEC),
        )
    )

    scene = mujoco.MJScene.fromFile(XML_PATH)
    scene.ModelTag = "earthMoonSpacecraft"
    # Dynamics-task messages are normally left at the final integrator-stage
    # evaluation, which can use a provisional rather than the committed state.
    # Re-evaluate the dynamics task after integration so the ephemeris, gravity,
    # and forward-kinematics messages are current for recorders and Vizard.
    scene.extraEoMCall = True
    simulation.AddModelToTask(
        TASK_NAME,
        scene,
        SCENE_PRIORITY,
    )

    integrator = svIntegrators.svIntegratorRKF45(scene)
    integrationTolerance = 1.0e-8  # [-]
    integrator.setRelativeTolerance(integrationTolerance)
    integrator.setAbsoluteTolerance(integrationTolerance)
    scene.setIntegrator(integrator)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    moon = gravFactory.createMoon()

    if useSpice:
        ephemeris = gravFactory.createSpiceInterface(
            time=SPICE_EPOCH,
            epochInMsg=True,
        )
        ephemerisMessages = list(ephemeris.planetStateOutMsgs)
    else:
        ephemeris, ephemerisMessages = _create_analytic_ephemeris(
            earth,
            moon,
        )
    ephemeris.zeroBase = "Earth"

    # The adaptive integrator evaluates dynamics between top-level task ticks.
    # Run the ephemeris after forward kinematics (priority 10000) but before the
    # factory-created NBodyGravity model (default priority -1) at every such
    # evaluation.
    scene.AddModelToDynamicsTask(ephemeris, EPHEMERIS_PRIORITY)
    gravFactory.addBodiesTo(scene)

    hub = scene.getBody("hub")
    hubRecorder = hub.getCenterOfMass().stateOutMsg.recorder()
    earthRecorder = ephemerisMessages[0].recorder()
    moonRecorder = ephemerisMessages[1].recorder()
    # Run recorders after MJScene. The extra equations-of-motion call above
    # ensures their samples see messages evaluated at the committed state.
    simulation.AddModelToTask(
        TASK_NAME,
        hubRecorder,
        RECORDER_PRIORITY,
    )
    simulation.AddModelToTask(
        TASK_NAME,
        earthRecorder,
        RECORDER_PRIORITY,
    )
    simulation.AddModelToTask(
        TASK_NAME,
        moonRecorder,
        RECORDER_PRIORITY,
    )

    if vizSupport.vizFound:
        vizSupport.enableUnityVisualization(
            simulation,
            TASK_NAME,
            scene,
            # saveFile=__file__,
        )

    spacecraftElements = orbitalMotion.ClassicElements()
    spacecraftElements.e = INITIAL_ECCENTRICITY
    spacecraftElements.a = (
        earth.radEquator + INITIAL_PERIAPSIS_ALTITUDE_M
    ) / (1.0 - spacecraftElements.e)  # [m]
    spacecraftElements.i = 28.5 * macros.D2R  # [deg] -> [rad]
    spacecraftElements.Omega = 48.2 * macros.D2R  # [deg] -> [rad]
    spacecraftElements.omega = 347.8 * macros.D2R  # [deg] -> [rad]
    spacecraftElements.f = 85.3 * macros.D2R  # [deg] -> [rad]
    initialPosition, initialVelocity = orbitalMotion.elem2rv(
        earth.mu,
        spacecraftElements,
    )

    simulation.InitializeSimulation()
    hub.setPosition(initialPosition)
    hub.setVelocity(initialVelocity)

    try:
        simulation.ConfigureStopTime(
            macros.sec2nano(SIMULATION_DURATION_SEC)
        )
        simulation.ExecuteSimulation()
    finally:
        if useSpice:
            gravFactory.unloadSpiceKernels()

    minimumSafeRadius = (
        earth.radEquator + MINIMUM_SAFE_ALTITUDE_M
    )  # [m]
    minimumRecordedRadius = np.min(
        np.linalg.norm(np.asarray(hubRecorder.r_BN_N), axis=1)
    )  # [m]
    if minimumRecordedRadius <= minimumSafeRadius:
        raise RuntimeError(
            "The MuJoCo spacecraft crossed the required "
            f"{MINIMUM_SAFE_ALTITUDE_M / 1000.0:.0f} km altitude boundary."
        )

    figures = _create_figures(
        useSpice,
        earth,
        moon,
        hubRecorder,
        earthRecorder,
        moonRecorder,
    )
    if showPlots:
        plt.show()
    return figures


if __name__ == "__main__":
    run(showPlots=True, useSpice=True)
