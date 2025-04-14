#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import os
import pytest
import time

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import NBodyGravity

    couldImportMujoco = True
except:
    couldImportMujoco = False


from Basilisk.simulation import pointMassGravityModel
from Basilisk.simulation import sphericalHarmonicsGravityModel
from Basilisk.simulation import spiceInterface
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros, orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities.pyswice_spk_utilities import spkRead
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import spacecraft


import numpy as np
import matplotlib.pyplot as plt

from Basilisk import __path__

bskPath = __path__[0]

import inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

TEST_FOLDER = os.path.dirname(__file__)
XML_PATH_BALL = f"{TEST_FOLDER}/test_ball.xml"
XML_PATH_DUMBBELL = f"{TEST_FOLDER}/test_dumbbell.xml"


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("showPlots", [False])
def test_pointMass(showPlots):
    """Test that the gravity model with point-mass gravity conserves the
    initial orbital elements after two orbits.
    """

    # Orbital parameters for the body
    mu = 0.3986004415e15 # m**3/s**2

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

    period = 2 * np.pi * np.sqrt(oe.a**3 / mu) # s
    tf = 2 * period

    # Because we use an adaptive integrator we can set the
    # time step to be the final time, and let the adaptive
    # integrator pick the most efficient step
    dt = tf if not showPlots else tf / 100

    # Create simulation, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create the MJScene from a simple cannonball body
    scene = mujoco.MJScene.fromFile(XML_PATH_BALL)
    integ = svIntegrators.svIntegratorRKF78(scene)
    integ.absTol = 1e-12
    integ.relTol = 1e-10
    scene.setIntegrator(integ)
    scSim.AddModelToTask("test", scene)

    ### Create the NBodyGravity model
    # add model to the dynamics task of the scene
    gravity = NBodyGravity.NBodyGravity()
    scene.AddModelToDynamicsTask(gravity)

    # Create a point-mass gravity source
    gravityModel = pointMassGravityModel.PointMassGravityModel()
    gravityModel.muBody = mu
    gravity.addGravitySource("earth", gravityModel, True)

    # Create a gravity target from the mujoco body
    body: mujoco.MJBody = scene.getBody("ball")
    gravity.addGravityTarget("ball", body)

    # Create recorders for the state of the body
    bodyStateRecorder = body.getCenterOfMass().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    # Initialize sim
    scSim.InitializeSimulation()

    # Set initial the position and velocity of the body
    body.setPosition(rN)
    body.setVelocity(vN)

    # Add random attitude and attitude rate, which should have no impact
    body.setAttitude(rbk.euler1232MRP([np.pi / 2, np.pi / 6, np.pi / 4]))
    body.setAttitudeRate([0.3, 0.1, 0.2]) # rad/s

    # Run sim
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    if showPlots:
        plt.figure()
        for i, l in enumerate(["x", "y", "z"]):
            plt.plot(
                bodyStateRecorder.times() * macros.NANO2SEC,
                bodyStateRecorder.r_BN_N[:, i],
                label=l,
            )
        plt.legend()
        plt.xlabel("Time [s]")
        plt.ylabel("Position [m]")

        plt.figure()
        plt.plot(bodyStateRecorder.r_BN_N[:, 0], bodyStateRecorder.r_BN_N[:, 1])
        plt.axis("equal")
        plt.xlabel("X-position [m]")
        plt.xlabel("Y-position [m]")

        plt.show()

    if True:
        oePost = orbitalMotion.rv2elem(
            mu, bodyStateRecorder.r_BN_N[-1, :], bodyStateRecorder.v_BN_N[-1, :]
        )

        for attr in ["a", "e", "i", "Omega", "omega"]:
            assert getattr(oePost, attr) == pytest.approx(getattr(oe, attr), 1e-4)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize(
    "showPlots, initialAngularRate", [(False, False), (False, True)]
)
def test_dumbbell(showPlots, initialAngularRate):
    r"""Test gravity on a compound spacecraft made up of two point masses
    rigidly connected but separated by 200 m:

    .. code-block:: text

          _____
         /     \            <----------- 200 m ----------->
        | Earth |          O---------------@---------------O
         \_____/        'left'          'center'         'right'

    The 'left' body starts closer to the Earth. The 'right' body has the same
    mass as the 'left' body and starts further from the Earth. The 'left', 'right',
    and center of the Earth are initially located in the same line. 'center' is
    a "fictitious" (zero-mass) body that we can use to represent the center of
    mass of the spacecraft.

    If ``initialAngularRate`` is True, then the total configuration is given
    an angular velocity equal to 2pi over the period of the orbit, which means
    that the body will tend to rotate about its center once every time the
    spacecraft rotates about the Earth. If ``initialAngularRate`` is False, then
    no initial rotation rate is given

    We can check two things:

    - The center of mass of this spacecraft should maintain constant
      orbital parameters given that this is a point-mass problem and
      all bodies are rigidly connected.
    - The two bodies are far apart enough for the gravity gradient to
      be noticeable over an orbit. The gradient will tend to
      keep the line connecting the two masses parallel to the line connecting
      the Earth to the spacecraft (keep the spacecraft pointing towards Earth).

     - If ``initialAngularRate`` is True, then the initial angular rate
       is such that this pointing is also maintained. Since we have both
       the initial rate and gravity gradient favoring this alignment, we can
       expect the alignment to indeed be maintained exactly through the orbit.
     - If ``initialAngularRate`` is False, then alignment will not be maintained
       exactly, but the gradient is strong enough that we can expect the "inner"
       body to always remain closer to Earth than the "outer" body, since it feels
       a stronger gravity pull.
    """

    # Initial orbital parameters of the body: circular equatorial LEO orbit
    mu = 0.3986004415e15 # m**3/s**2

    oe = orbitalMotion.ClassicElements()
    rLEO = 7000.0 * 1000  # meters
    oe.a = rLEO
    oe.e = 0
    oe.i = 0
    oe.Omega = 0
    oe.omega = 0
    oe.f = 0

    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)

    period = 2 * np.pi * np.sqrt(oe.a**3 / mu) # s

    # Integrate for an orbit, record 50 points through orbit
    tf = period * 1
    dt = period / 50

    # Create simulation, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    task = scSim.CreateNewTask("test", macros.sec2nano(dt))
    dynProcess.addTask(task)

    # Create MJScene, set adaptive integrator
    scene = mujoco.MJScene.fromFile(XML_PATH_DUMBBELL)
    scene.ModelTag = "mujocoScene"
    # We will be logging forces, so we need an extra call to the EoM after each integrator hop
    scene.extraEoMCall = True
    integ = svIntegrators.svIntegratorRKF78(scene)
    integ.absTol = 1e-14
    integ.relTol = 1e-12
    scene.setIntegrator(integ)
    scSim.AddModelToTask("test", scene)

    # Create gravity model and add it to the scene dynamics task
    gravity = NBodyGravity.NBodyGravity()
    gravity.ModelTag = "gravity"
    scene.AddModelToDynamicsTask(gravity)

    # Set a point mass gravity source
    gravityModel = pointMassGravityModel.PointMassGravityModel()
    gravityModel.muBody = mu
    gravity.addGravitySource("earth", gravityModel, True)

    # Add a gravity target for each of the bodies of the spacecraft
    # Note that because 'center' has zero mass, no force should be
    # produced on this body.
    # Also, create recorders for the state of each body and for the
    # gravity force exerted on each body.
    bodies = ["left", "right", "center"]
    forceRecorders = {}
    bodyStateRecorders = {}
    for bodyName in bodies:
        body: mujoco.MJBody = scene.getBody(bodyName)
        target = gravity.addGravityTarget(bodyName, body)

        forceRecorders[bodyName] = target.massFixedForceOutMsg.recorder()
        scSim.AddModelToTask("test", forceRecorders[bodyName])

        bodyStateRecorders[bodyName] = body.getCenterOfMass().stateOutMsg.recorder()
        scSim.AddModelToTask("test", bodyStateRecorders[bodyName])

    # Initialize sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))

    # Set the initial state of the spacecraft
    mainBody = scene.getBody("center")
    mainBody.setPosition(rN)
    mainBody.setVelocity(vN)
    mainBody.setAttitude(rbk.euler1232MRP([0, 0, 0]))
    mainBody.setAttitudeRate([0, 0, 2 * np.pi / period if initialAngularRate else 0]) # rad/s

    # Run sim
    scSim.ExecuteSimulation()

    if showPlots:
        center = bodyStateRecorders["center"].r_BN_N
        relative = (
            bodyStateRecorders["right"].r_BN_N - bodyStateRecorders["left"].r_BN_N
        )
        centerNormalized = center / np.linalg.norm(center, axis=1)[:, np.newaxis]
        relativeNormalized = relative / np.linalg.norm(relative, axis=1)[:, np.newaxis]
        angle = np.rad2deg(
            np.arccos(np.sum(centerNormalized * relativeNormalized, axis=1))
        )

        anomaly = np.rad2deg(np.arctan2(centerNormalized[:, 1], centerNormalized[:, 0]))
        anomaly[anomaly < 0] += 360

        plt.figure()
        plt.plot(anomaly, angle)
        plt.xlabel("True Anomaly [deg]")
        plt.ylabel("Angle between left->right vector and center position vector [deg]")
        plt.tight_layout()

        plt.figure()
        plt.plot(anomaly, np.rad2deg(bodyStateRecorders["center"].omega_BN_B[:, 2]))
        plt.plot(
            anomaly,
            np.rad2deg(2 * np.pi / period) * np.ones_like(anomaly),
            "k--",
            label=r"$\omega_{ref}=2\pi/T_{orbit}$",
        )
        plt.xlabel("True Anomaly [deg]")
        plt.ylabel("Attitude Rate along Z-axis [rad/s]")
        plt.tight_layout()

        plt.figure()
        for body in bodies:
            radius = np.linalg.norm(bodyStateRecorders[body].r_BN_N, axis=1)
            plt.plot(anomaly, radius, label=body)
        plt.xlabel("True Anomaly [deg]")
        plt.ylabel("Orbital Radius [m]")
        plt.legend()
        plt.tight_layout()

        plt.figure()
        for body in ["left", "right"]:
            plt.plot(
                anomaly,
                np.linalg.norm(forceRecorders[body].force_S, axis=1),
                label=body,
            )
        plt.xlabel("True Anomaly [deg]")
        plt.ylabel("Gravitational force acting on body [N]")
        plt.legend()
        plt.tight_layout()

        plt.figure()
        for bodyName in ["left", "right"]:
            plt.plot(
                bodyStateRecorders[bodyName].r_BN_N[:, 0],
                bodyStateRecorders[bodyName].r_BN_N[:, 1],
                label=bodyName,
            )
        plt.xlabel("X-position [m]")
        plt.xlabel("Y-position [m]")
        plt.legend()
        plt.axis("equal")
        plt.tight_layout()

        plt.show()

    if True:
        oePost = orbitalMotion.rv2elem(
            mu,
            bodyStateRecorders["center"].r_BN_N[-1, :],
            bodyStateRecorders["center"].v_BN_N[-1, :],
        )

        # The orbital parameters should be maintained through the orbit
        for attr in ["a", "e", "i"]:
            assert getattr(oePost, attr) == pytest.approx(getattr(oe, attr), 1e-4, 1e-8)

        # If the initial angular rate is set, we expect the gravity gradient to not
        # interfere with this rate (see function docstring), thus it must be
        # constant in time.
        if initialAngularRate:
            for i in range(bodyStateRecorders["center"].omega_BN_B.shape[0]):
                assert bodyStateRecorders["center"].omega_BN_B[i, 2] == pytest.approx(
                    2 * np.pi / period, 1e-6
                ), f"At time index {i}"

        # If no initial rate is given, we can still check that the inner body
        # is kept close to the Earth due to gravity gradient
        else:
            radius = {
                body: np.linalg.norm(bodyStateRecorders[body].r_BN_N, axis=1)
                for body in bodies
            }
            assert np.all(radius["left"] < radius["center"])
            assert np.all(radius["center"] < radius["right"])


def loadSatelliteTrajectory(utcCalInit: str, stopTimeSeconds: float, dtSeconds: float):
    """Loads the trajectory of a GPS satellite from SPICE kernels from the
    given date (``utcCalInit``), for ``stopTimeSeconds`` seconds, with the position
    reported every ``dtSeconds`` seconds."""
    kernels = [
        bskPath + "/supportData/EphemerisData/de430.bsp",
        bskPath + "/supportData/EphemerisData/naif0012.tls",
        path + "/gnss_221111_221123_v01.bsp",
    ]

    # load spice kernels
    for k in kernels:
        pyswice.furnsh_c(k)

    # initial time in UTC to ET
    etArray = pyswice.new_doubleArray(1)
    pyswice.str2et_c(utcCalInit, etArray)
    et0 = pyswice.doubleArray_getitem(etArray, 0)

    # times where the state should be recovered
    t = np.arange(0, stopTimeSeconds + dtSeconds, dtSeconds)

    # Get states
    state = np.zeros([t.size, 6])
    for i in range(t.size):
        et = et0 + t[i]
        tUtc = pyswice.et2utc_c(et, "C", 4, 1024, "Yo")
        state[i, :] = spkRead("-36585", tUtc, "J2000", "EARTH")

    # unload spice kernels
    for k in kernels:
        pyswice.unload_c(k)

    return state


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize(
    "showPlots, useSphericalHarmonics, useThirdBodies",
    [(False, False, False), (False, True, False), (False, True, True)],
)
def test_gps(showPlots: bool, useSphericalHarmonics: bool, useThirdBodies: bool):
    """Compares the trajectory produced by Basilisk with the true trajectory
    of a GPS satellite, as obtained from SPICE kernels.

    The ``useSphericalHarmonics`` and ``useThirdBodies`` entries can be used to
    tune the accuracy of the gravity model, and thus how close should the
    Basilisk result be to the SPICE data.

    The satellite state is initialized from SPICE, then propagated with Basilisk
    for 1 hour. The final state is expected to match the SPICE state at the time
    by:

    - < 300 m if point mass gravity is used.
    - < 30 m if spherical harmonics is used by no third-body effects.
    - < 30 m if spherical harmonics and third-body effects are considered.

    Note that the inclusion of third body effects seems to have no effect on the
    accuracy. Something that should be investigated further.
    """

    # initial date, simulation time, and time step
    utcCalInit = "2022 NOV 14 00:01:10"
    tf = 1 * 3600 # s
    dt = 1 # s

    # load trajectory from SPICE
    spiceSatelliteState = loadSatelliteTrajectory(utcCalInit, tf, dt)

    # Create simulation, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create MJScene (cannonball) and configure integrator
    scene = mujoco.MJScene.fromFile(XML_PATH_BALL)
    integ = svIntegrators.svIntegratorRKF78(scene)
    integ.absTol = 1e-12
    integ.relTol = 1e-10
    scene.setIntegrator(integ)
    scSim.AddModelToTask("test", scene)

    # Create the spice interface module, which reports the state of
    # planetary bodies
    spice = spiceInterface.SpiceInterface()
    spice.ModelTag = "SpiceInterface"
    spice.SPICEDataPath = bskPath + "/supportData/EphemerisData/"
    spice.addPlanetNames(["earth", "moon", "sun"])
    spice.UTCCalInit = utcCalInit
    scene.AddModelToDynamicsTask(spice)

    # Create NBody Gravity
    gravity = NBodyGravity.NBodyGravity()
    scene.AddModelToDynamicsTask(gravity)

    # Create gravity sources (Earth, Moon, Sun)
    if useSphericalHarmonics:
        earthGravityModel = (
            sphericalHarmonicsGravityModel.SphericalHarmonicsGravityModel()
        )
        earthGravityModel.radEquator = 0.6378136300e7
        earthGravityModel.loadFromFile(
            bskPath + "/supportData/LocalGravData/GGM03S.txt", 4
        )  # J4 should be plenty
    else:
        earthGravityModel = pointMassGravityModel.PointMassGravityModel()
    earthGravityModel.muBody = 0.3986004415e15
    earthGravitySource = gravity.addGravitySource(
        "earth", earthGravityModel, isCentralBody=True
    )
    earthGravitySource.stateInMsg.subscribeTo(spice.planetStateOutMsgs[0])

    if useThirdBodies:
        moonGravityModel = pointMassGravityModel.PointMassGravityModel()
        moonGravityModel.muBody = 4.902799e12
        moonGravitySource = gravity.addGravitySource(
            "moon", moonGravityModel, isCentralBody=False
        )
        moonGravitySource.stateInMsg.subscribeTo(spice.planetStateOutMsgs[1])

    if useThirdBodies:
        sunGravityModel = pointMassGravityModel.PointMassGravityModel()
        sunGravityModel.muBody = 1.32712440018e20
        sunGravitySource = gravity.addGravitySource(
            "sun", sunGravityModel, isCentralBody=False
        )
        sunGravitySource.stateInMsg.subscribeTo(spice.planetStateOutMsgs[2])

    # Create gravity target
    body: mujoco.MJBody = scene.getBody("ball")
    target = gravity.addGravityTarget("ball", body)

    # Create gravity force and body state recorders
    forceRecorder = target.massFixedForceOutMsg.recorder()
    scSim.AddModelToTask("test", forceRecorder)

    bodyStateRecorder = body.getCenterOfMass().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    # initialize sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))

    # Initialize the body to the same position and velocity as the SPICE result
    body.setPosition(spiceSatelliteState[0, :3] * 1000) # m
    body.setVelocity(spiceSatelliteState[0, 3:] * 1000) # m

    # Run sim
    scSim.ExecuteSimulation()

    if showPlots:
        diff = np.linalg.norm(
            spiceSatelliteState[:, :3] * 1000 - bodyStateRecorder.r_BN_N, axis=1
        )
        t = bodyStateRecorder.times() * macros.NANO2SEC

        plt.figure()
        plt.plot(t, diff)
        plt.xlabel("Time [s]")
        plt.ylabel("Position diff between SPICE and Basilisk [m]")

        plt.figure()
        plt.plot(
            t,
            spiceSatelliteState[:, :3] * 1000,
            label=["SPICE x", "SPICE y", "SPICE z"],
        )
        plt.plot(t, bodyStateRecorder.r_BN_N, "--", label=["BSK x", "BSK y", "BSK z"])
        plt.legend()
        plt.xlabel("Time [s]")
        plt.ylabel("Position [m]")

        plt.show()

    lastSpice = spiceSatelliteState[-1, :3] * 1000
    lastBsk = bodyStateRecorder.r_BN_N[-1, :]

    # Depending on the gravity model used, we use a different tolerance
    if not useSphericalHarmonics:
        assert lastBsk == pytest.approx(lastSpice, abs=300)

    if useSphericalHarmonics and not useThirdBodies:
        assert lastBsk == pytest.approx(lastSpice, abs=30)

    if useSphericalHarmonics and useThirdBodies:
        assert lastBsk == pytest.approx(lastSpice, abs=30)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize(
    "showPlots, useSpice, useSphericalHarmonics, useThirdBodies",
    [
        (False, False, False, False),
        (False, True, False, False),
        (False, True, True, False),
        (False, True, True, True),
    ],
)
def test_mujocoVsSpacecraft(
    showPlots: bool, useSpice: bool, useSphericalHarmonics: bool, useThirdBodies: bool
):
    """Tests that the same scenario (simple LEO orbit with only gravity)
    produces the same results when running with a ``spacecraft.Spacecraft``
    dynamical object and a ``mujoco.MJScene``.

    If ``showPlots`` is ``True``, the scenario is run for 24 hr and the
    differences are shown in plots. If it's ``False``, then the scenario
    is only run for 10 seconds and an assertion is raised if the results
    disagree on a very tight tolerances.
    """

    if useThirdBodies and not useSpice:
        raise RuntimeError("If using third bodies, we must use spice")

    # Initial time, final time, and time step
    # We run for 24hr only if we want to plot results
    utcCalInit = "2022 NOV 14 00:01:10"
    dt = 1 # s
    tf = 24 * 3600 if showPlots else 10 # s

    # initial state of the body
    mu = 0.3986004415e15 # m**3/s**2

    oe = orbitalMotion.ClassicElements()
    rLEO = 7000.0 * 1000  # meters
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R

    rN, vN = orbitalMotion.elem2rv(mu, oe)

    # Create GravBodies, which will be used for the spacecraft
    # and also the mujoco sim
    bodies = ["earth"]
    if useThirdBodies:
        bodies.extend(["sun", "moon"])
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(bodies)
    gravBodies["earth"].isCentralBody = True
    if useSphericalHarmonics:
        gravBodies["earth"].useSphericalHarmonicsGravityModel(
            bskPath + "/supportData/LocalGravData/GGM03S.txt", 4
        )

    # A decorator that times how long the function takes and
    # prints it to the console
    def timed(fn):
        def wrap():
            tic = time.time()
            result = fn()
            toc = time.time()
            print(f"{fn.__name__} took {toc-tic} seconds")
            return result

        return wrap

    # NOTE: in the two simulation defined below we use the Euler integrator.
    # this is because, for the ``spacecraft`` dynamic object, the spice interface
    # is called once per task step, while for the mujoco dynamic object the
    # interface is called once per integrator step. Euler is the only integrator
    # where one integrator step is taken per task step, which allows us to
    # produce equivalent results.

    @timed
    def spacecraftSim():
        # Create sim, process, and task
        scSim = SimulationBaseClass.SimBaseClass()
        dynProcess = scSim.CreateNewProcess("test")
        dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

        # Create spice interface and add to task
        if useSpice:
            gravFactory.createSpiceInterface(
                bskPath + "/supportData/EphemerisData/", utcCalInit
            )
            gravFactory.spiceObject.zeroBase = "Earth"
            scSim.AddModelToTask("test", gravFactory.spiceObject, 10)

        # Create spacecraft
        scObject = spacecraft.Spacecraft()
        scObject.ModelTag = "spacecraftBody"
        integSc = svIntegrators.svIntegratorEuler(scObject)  # MUST BE EULER!
        scObject.setIntegrator(integSc)
        scSim.AddModelToTask("test", scObject, 9)

        scObject.gravField.gravBodies = spacecraft.GravBodyVector(
            list(gravFactory.gravBodies.values())
        )

        # initialize spacecraft parameters
        scObject.hub.mHub = 100 # kg
        scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] # m
        scObject.hub.IHubPntBc_B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] # kg*m**2
        scObject.hub.r_CN_NInit = rN # m
        scObject.hub.v_CN_NInit = vN # m/s

        # Create recorder
        scStateRecorder = scObject.scStateOutMsg.recorder()
        scSim.AddModelToTask("test", scStateRecorder)

        # initialize and run sim
        scSim.InitializeSimulation()
        scSim.ConfigureStopTime(macros.sec2nano(tf))
        scSim.ExecuteSimulation()

        return scStateRecorder

    @timed
    def mujocoSim():
        # Create sim, process, and task
        scSim = SimulationBaseClass.SimBaseClass()
        dynProcess = scSim.CreateNewProcess("test")
        dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

        # Create the spice interface module, which reports the state of
        # planetary bodies
        spice = spiceInterface.SpiceInterface()
        spice.ModelTag = "SpiceInterface"
        spice.SPICEDataPath = bskPath + "/supportData/EphemerisData/"
        spice.addPlanetNames(["earth", "sun", "moon"])
        spice.UTCCalInit = utcCalInit
        # spice.zeroBase = 'Earth' # Not actually needed for NBodyGravity
        scSim.AddModelToTask("test", spice)

        # Create mujoco scene
        scene = mujoco.MJScene.fromFile(XML_PATH_BALL)
        scene.ModelTag = "mujocoScene"
        integScene = svIntegrators.svIntegratorEuler(scene)  # MUST BE EULER!
        scene.setIntegrator(integScene)
        scSim.AddModelToTask("test", scene)

        # Create N-Body Gravity
        gravity = NBodyGravity.NBodyGravity()
        scene.AddModelToDynamicsTask(gravity)

        for (name, gravBody), stateOuMsg in zip(
            gravBodies.items(), spice.planetStateOutMsgs
        ):
            source = gravity.addGravitySource(
                name, gravBody.gravityModel, isCentralBody=(name == "earth")
            )
            if useSpice:
                source.stateInMsg.subscribeTo(stateOuMsg)

        body: mujoco.MJBody = scene.getBody("ball")
        gravity.addGravityTarget("ball", body)

        bodyStateRecorder = body.getCenterOfMass().stateOutMsg.recorder()
        scSim.AddModelToTask("test", bodyStateRecorder)

        scSim.InitializeSimulation()
        scSim.ConfigureStopTime(macros.sec2nano(tf))

        # Initialize mujoco body to the same position and velocity as the spacecraft
        body.setPosition(rN)
        body.setVelocity(vN)

        scSim.ExecuteSimulation()

        return bodyStateRecorder

    scStateRecorder = spacecraftSim()
    bodyStateRecorder = mujocoSim()

    if showPlots:

        t = bodyStateRecorder.times() * macros.NANO2SEC
        diff = np.linalg.norm(scStateRecorder.r_BN_N - bodyStateRecorder.r_BN_N, axis=1)

        plt.figure()
        plt.plot(t, diff)
        plt.xlabel("Time [s]")
        plt.ylabel("Position Difference ``Spacecraft`` vs. ``MJScene`` [m]")

        plt.figure()
        plt.plot(t, scStateRecorder.r_BN_N, label="Spacecraft")
        plt.plot(t, bodyStateRecorder.r_BN_N, "--", label="MJScene")
        plt.xlabel("Time [s]")
        plt.ylabel("Position [m]")

        plt.show()

    else:
        assert bodyStateRecorder.r_BN_N[-1, :] == pytest.approx(
            scStateRecorder.r_BN_N[-1, :], 1e-10
        )


if __name__ == "__main__":
    if True:
        showPlots: bool = False
        test_pointMass(showPlots)
        test_dumbbell(showPlots, True)
        test_dumbbell(showPlots, False)
        test_gps(showPlots, False, False)
        test_gps(showPlots, True, False)
        test_gps(showPlots, True, True)
        test_mujocoVsSpacecraft(showPlots, True, True, True)

    else:
        pytest.main([__file__])
