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
Multi-body-in-orbit scenario in the dynamics-engine comparison series (see
:ref:`scenarioCompareOrbit` for the introduction to the two engines, and
:ref:`scenarioCompareRwPanels` for the free-floating multi-body it builds on).

This scenario puts the reaction-wheel-and-panel multi-body of
:ref:`scenarioCompareRwPanels` into a Keplerian orbit, adding point-mass gravity to the
internal wheel and flex dynamics. This exposes a difference in how the two engines
apply gravity to an *extended* body, so the scenario runs in two regimes:

**Part A: a compact body (hub + reaction wheels).** All massive parts sit at the hub
origin (the reaction-wheel bodies carry negligible mass), so gravity acts at a single
point and exerts no gravity-gradient torque. The two engines agree closely: both reproduce
the analytic Kepler orbit to the RK4 truncation level (cross-engine orbit difference of
order :math:`10^{-7}` m), and the spinning-up wheels match to round-off. The hub attitude
agrees to about :math:`10^{-4}` rad, several orders coarser than the free-floating
machine-precision floor of :ref:`scenarioCompareRwPanels`. This floor is not a gravity
effect but the MuJoCo gyroscopic-bias round-off characterized in
:ref:`scenarioCompareTorque`, which any body at the ~7.5 km/s orbital speed incurs (the same
size with gravity on or off). Part A therefore sets the noise floor for the extended-body
attitude comparison below.

**Part B: an extended body (hub + reaction wheels + offset flexible panels).** The two
massive solar arrays sit well off the hub origin, and the engines apply gravity by
different models:

#. BSM evaluates the gravitational acceleration **once, at the system center of mass**
   (tracked analytically as the panels flex) and applies it as a single uniform body force.
   A uniform field exerts no net torque about the center of mass, so BSM produces **no
   gravity-gradient torque**.
#. The MuJoCo :ref:`NBodyGravity` evaluates gravity **per body, at each body's own
   position**. The offset panels sit at a slightly different radius than the hub, so they
   feel a slightly different acceleration. Forces at the separated body centers produce a
   net torque about the system center of mass. This model does not resolve gravity variation
   within an individual rigid body.

Part B therefore does not converge to machine precision in attitude. Its magnitude is
consistent with the differential point-force scale over the orbit and builds as a
libration modulated by the hub's tumble.

As in the other attitude-bearing comparisons, the MuJoCo scene enables
``highOrderAttitudeIntegration`` so the hub quaternion is advanced at the integrator's
full order, and attitudes are compared with the principal rotation angle of the relative
direction cosine matrix, :math:`4\,\arctan(|\pmb\sigma_{\rm rel}|)`.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareOrbitMultibody.py

Illustration of Simulation Results
----------------------------------

For the compact body (Part A) each engine tracks the analytic Kepler orbit to its RK4
truncation level (about :math:`10^{-7}` m), and the two engines agree to the same level,
confirming both integrate the same two-body equation of motion through the internal
multi-body coupling.

.. image:: /_images/Scenarios/scenarioCompareOrbitMultibody_compactOrbit.svg
   :align: center

For the extended body (Part B) the hub attitudes separate: forces applied at the
MuJoCo body centers exert a torque that the BSM single-point model omits. Over one orbit,
the cross-engine attitude difference reaches 0.79 rad (46 degrees), nearly four orders of
magnitude above the Part A floor. The response appears as a libration modulated by the
hub's tumble.

.. image:: /_images/Scenarios/scenarioCompareOrbitMultibody_attError.svg
   :align: center

After matching the initial total-center-of-mass position and velocity, both the compact
and extended center-of-mass trajectories agree within :math:`2\times10^{-6}` m. The
attitude, not the translational orbit, carries the differential-force effect.

.. image:: /_images/Scenarios/scenarioCompareOrbitMultibody_regimes.svg
   :align: center

Runtime cost
------------

Wall-clock cost of propagating each regime (one full orbit) with each engine, reported
as the median of five measured trials after one discarded warm-up. Model setup is
excluded. Both the absolute times and speedup ratio are specific to the machine and
build. Generate the optional local CSV with
``make -C docs comparison-runtime-tables``; see
:ref:`scenarioCompareOrbit` for the benchmark requirements and interpretation.

Next comparison: :ref:`scenarioCompareVariableMass` adds fuel depletion, moving mass
properties, thrust, and propellant slosh.

"""

import os

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import spacecraft
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import hingedRigidBodyStateEffector
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import messaging

import _runtimeTable
import _comparePlots
import _comparisonValidation

from Basilisk import hasBuildFeature

couldImportMujoco = hasBuildFeature("mujoco")
if couldImportMujoco:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJSystemCoM

# Paul Tol high-contrast palette shared by the comparison figures.
COLOR_BSM = _comparePlots.COLOR_BSM
COLOR_MUJOCO = _comparePlots.COLOR_MUJOCO
COLOR_REFERENCE = _comparePlots.COLOR_DIFF

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

# ---------------------------------------------------------------------------
# Model parameters (the same medium-class free-flyer as scenarioCompareRwPanels,
# now placed in orbit). Part A omits the panels; Part B includes them.
# ---------------------------------------------------------------------------
HUB_MASS = 600.0  # [kg]
HUB_CORE_INERTIA = (400.0, 380.0, 360.0)  # [kg*m^2] bare hub, before wheel inertia folds in
RW_JS = 0.08  # [kg*m^2] wheel axial inertia
RW_JT = 0.04  # [kg*m^2] wheel transverse inertia
RW_MASS = 1.0e-6  # [kg] negligible, matching the massless balanced-wheel idealization
RW_TORQUE = (0.010, -0.015, 0.008)  # [N*m] constant motor torque per wheel
PANEL_MASS = 50.0  # [kg]
PANEL_INERTIA = (30.0, 20.0, 12.0)  # [kg*m^2] about the panel center of mass
PANEL_D = 1.2  # [m] hinge-axis to panel center-of-mass distance
PANEL_K = 150.0  # [N*m/rad] torsional spring stiffness
PANEL_C = 8.0  # [N*m*s/rad] torsional damping
PANEL_HINGE_X = 0.8  # [m] hinge offset from the hub origin along x
PANEL_THETA0 = 12.0*macros.D2R  # [rad] initial panel deflection
OMEGA0_B = (0.010, -0.020, 0.015)  # [rad/s] initial hub rate


def panelRootDcm(hingeX):
    """Return the root-frame DCM that makes both panels extend away from the hub."""
    if hingeX > 0.0:
        return [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def hubInertiaBSM():
    """BSM hub inertia: bare hub plus the three folded-in wheel tensors.

    Returns:
        numpy.ndarray: 3x3 hub inertia tensor [kg*m^2].
    """
    inertia = np.diag(HUB_CORE_INERTIA).astype(float)
    for axis in range(3):
        diag = [RW_JT, RW_JT, RW_JT]
        diag[axis] = RW_JS
        inertia = inertia + np.diag(diag)
    return inertia


def initialOrbitState(mu):
    """Return the initial inertial position and velocity and the classical elements.

    Args:
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        tuple: ``(rN, vN, oe)`` with position [m], velocity [m/s], and the
        ``ClassicElements`` used to generate them.
    """
    oe = orbitalMotion.ClassicElements()
    oe.a = 7000.0e3  # [m]
    oe.e = 0.01
    oe.i = 33.3*macros.D2R  # [rad]
    oe.Omega = 48.2*macros.D2R  # [rad]
    oe.omega = 347.8*macros.D2R  # [rad]
    oe.f = 85.3*macros.D2R  # [rad]
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    return np.array(rN), np.array(vN), oe


def initialBodyMassDistribution(withPanels):
    """Return the initial body masses, center locations, and total center of mass.

    Locations are expressed in the hub frame. The wheel centers coincide with the hub
    center, while the panel centers follow the common BSM/MJCF hinge convention.

    Args:
        withPanels (bool): include the two hinged panels.

    Returns:
        tuple: ``(masses, centers, systemCoM)`` in kg and m.
    """
    masses = [HUB_MASS, RW_MASS, RW_MASS, RW_MASS]
    centers = [np.zeros(3) for _ in masses]
    if withPanels:
        for sign in (+1.0, -1.0):
            masses.append(PANEL_MASS)
            centers.append(np.array([
                sign*(PANEL_HINGE_X + PANEL_D*np.cos(PANEL_THETA0)),
                0.0,
                PANEL_D*np.sin(PANEL_THETA0),
            ]))
    masses = np.asarray(masses)
    centers = np.asarray(centers)
    systemCoM = np.average(centers, axis=0, weights=masses)
    return masses, centers, systemCoM


def initialMujocoRootState(withPanels, rCN_N, vCN_N):
    """Convert a desired total-CoM state into the MuJoCo hub-origin state.

    The initial attitude is identity and the hinge rates are zero, so the system CoM
    velocity relative to the hub is ``omega_BN_B x c_B``.
    """
    _, _, c_B = initialBodyMassDistribution(withPanels)
    rBN_N = np.asarray(rCN_N) - c_B
    vBN_N = np.asarray(vCN_N) - np.cross(np.asarray(OMEGA0_B), c_B)
    return rBN_N, vBN_N


def keplerTruth(mu, oe0, times):
    """Analytic two-body position history of the system center of mass.

    Args:
        mu (float): gravitational parameter [m^3/s^2]
        oe0 (ClassicElements): initial classical orbital elements
        times (numpy.ndarray): sample times [s]

    Returns:
        numpy.ndarray: inertial position history, shape ``(N, 3)`` [m].
    """
    meanMotion = np.sqrt(mu/oe0.a**3)  # [rad/s]
    meanAnomaly0 = orbitalMotion.E2M(orbitalMotion.f2E(oe0.f, oe0.e), oe0.e)
    truth = np.empty((len(times), 3))
    for k, t in enumerate(times):
        trueAnomaly = orbitalMotion.E2f(
            orbitalMotion.M2E(meanAnomaly0+meanMotion*t, oe0.e), oe0.e)
        oe = orbitalMotion.ClassicElements()
        oe.a, oe.e, oe.i = oe0.a, oe0.e, oe0.i
        oe.Omega, oe.omega, oe.f = oe0.Omega, oe0.omega, trueAnomaly
        rN, _ = orbitalMotion.elem2rv(mu, oe)
        truth[k] = rN
    return truth


def mujocoModel(withPanels):
    """Return the MJCF model: hub and three reaction wheels, optionally two hinged panels.

    Args:
        withPanels (bool): if True, attach the two offset flexible panels (Part B);
            if False, the compact hub-and-wheels body (Part A).

    Returns:
        str: the MJCF XML string.
    """
    ix, iy, iz = HUB_CORE_INERTIA
    ip = PANEL_INERTIA
    panelXml = "" if not withPanels else f"""
      <body name="panelP" pos="{PANEL_HINGE_X} 0 0" quat="0 0 0 1">
        <joint name="panelP" type="hinge" axis="0 1 0" stiffness="{PANEL_K}" damping="{PANEL_C}" springref="0"/>
        <inertial pos="{-PANEL_D} 0 0" mass="{PANEL_MASS}" fullinertia="{ip[0]} {ip[1]} {ip[2]} 0 0 0"/>
      </body>
      <body name="panelM" pos="{-PANEL_HINGE_X} 0 0">
        <joint name="panelM" type="hinge" axis="0 1 0" stiffness="{PANEL_K}" damping="{PANEL_C}" springref="0"/>
        <inertial pos="{-PANEL_D} 0 0" mass="{PANEL_MASS}" fullinertia="{ip[0]} {ip[1]} {ip[2]} 0 0 0"/>
      </body>"""
    return f"""
<mujoco>
  <!-- No geoms, equalities, limits, or frictionloss in this model: disable the
       collision and constraint pipelines so each derivative evaluation skips
       broadphase and constraint-assembly bookkeeping (bit-identical results). -->
  <option gravity="0 0 0">
    <flag contact="disable" constraint="disable"/>
  </option>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" mass="{HUB_MASS}" fullinertia="{ix} {iy} {iz} 0 0 0"/>

      <body name="rw_x" pos="0 0 0">
        <joint name="rw_x" type="hinge" axis="1 0 0"/>
        <inertial pos="0 0 0" mass="{RW_MASS}" fullinertia="{RW_JS} {RW_JT} {RW_JT} 0 0 0"/>
      </body>
      <body name="rw_y" pos="0 0 0">
        <joint name="rw_y" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="{RW_MASS}" fullinertia="{RW_JT} {RW_JS} {RW_JT} 0 0 0"/>
      </body>
      <body name="rw_z" pos="0 0 0">
        <joint name="rw_z" type="hinge" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="{RW_MASS}" fullinertia="{RW_JT} {RW_JT} {RW_JS} 0 0 0"/>
      </body>{panelXml}
    </body>
  </worldbody>
  <actuator>
    <motor name="rw_x" joint="rw_x"/>
    <motor name="rw_y" joint="rw_y"/>
    <motor name="rw_z" joint="rw_z"/>
  </actuator>
</mujoco>
"""


def buildBSM(withPanels, mu, dt, tf, recordDt, record=True):
    """Build the back-substitution orbiting multi-body simulation.

    Args:
        withPanels (bool): include the offset flexible panels (Part B) or not (Part A).
        mu (float): gravitational parameter [m^3/s^2]
        dt (float): integrator time step [s]
        tf (float): final simulation time [s]
        recordDt (float): recorder sampling period [s]
        record (bool, optional): attach state and wheel recorders. Defaults to True.

    Returns:
        tuple: ``(simulation, outputs, handles)``.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "hub"
    scObject.hub.mHub = HUB_MASS  # [kg]
    scObject.hub.IHubPntBc_B = hubInertiaBSM().tolist()  # [kg*m^2]
    scObject.hub.omega_BN_BInit = [[w] for w in OMEGA0_B]  # [rad/s]
    rN, vN, _ = initialOrbitState(mu)
    scObject.hub.r_CN_NInit = rN.tolist()  # [m] initial system center-of-mass position
    scObject.hub.v_CN_NInit = vN.tolist()  # [m/s]
    scSim.AddModelToTask("dynTask", scObject)

    integrator = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integrator)

    rwEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwEffector.ModelTag = "rw"
    rwFactory = simIncludeRW.rwFactory()
    for spinAxis in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
        rwFactory.create("custom", spinAxis, Js=RW_JS, Omega=0.0, rWB_B=[0., 0., 0.],
                         useMaxTorque=False, useMinTorque=False)
    rwFactory.addToSpacecraft("rw", rwEffector, scObject)
    scSim.AddModelToTask("dynTask", rwEffector)
    rwCmdMsg = messaging.ArrayMotorTorqueMsg()
    rwCmdMsg.write(messaging.ArrayMotorTorqueMsgPayload(
        motorTorque=list(RW_TORQUE) + [0.0]*33))
    rwEffector.rwMotorCmdInMsg.subscribeTo(rwCmdMsg)

    # Point-mass Earth gravity. The BSM effector evaluates this once at the system
    # center of mass each step, so it carries no gravity-gradient torque.
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.mu = mu  # [m^3/s^2]
    earth.isCentralBody = True
    gravFactory.addBodiesTo(scObject)

    panels = []
    if withPanels:
        for hingeX in (PANEL_HINGE_X, -PANEL_HINGE_X):
            panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
            panel.mass = PANEL_MASS  # [kg]
            panel.IPntS_S = np.diag(PANEL_INERTIA).tolist()  # [kg*m^2]
            panel.d = PANEL_D  # [m]
            panel.k = PANEL_K  # [N*m/rad]
            panel.c = PANEL_C  # [N*m*s/rad]
            panel.r_HB_B = [[hingeX], [0.0], [0.0]]  # [m]
            panel.dcm_HB = panelRootDcm(hingeX)
            panel.thetaInit = PANEL_THETA0  # [rad]
            panel.thetaDotInit = 0.0  # [rad/s]
            panel.ModelTag = "panel" + ("P" if hingeX > 0 else "M")
            scObject.addStateEffector(panel)
            scSim.AddModelToTask("dynTask", panel)
            panels.append(panel)

    stateRecorder = wheelRecorder = None
    if record:
        stateRecorder = scObject.scStateOutMsg.recorder(macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", stateRecorder)
        wheelRecorder = rwEffector.rwSpeedOutMsg.recorder(macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", wheelRecorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    handles = [scObject, integrator, rwEffector, rwFactory, rwCmdMsg, gravFactory] + panels
    return scSim, (stateRecorder, wheelRecorder), handles


def runBSM(withPanels, mu, dt, tf, recordDt):
    """Propagate the orbiting multi-body with the back-substitution :ref:`spacecraft`."""
    scSim, outputs, handles = buildBSM(withPanels, mu, dt, tf, recordDt)
    scSim.ExecuteSimulation()
    return (*outputs, handles)


def buildMujoco(withPanels, mu, dt, tf, recordDt, record=True):
    """Build the MuJoCo orbiting multi-body simulation without propagating it.

    Gravity is supplied by :ref:`NBodyGravity`, which evaluates the field at each body's
    own position. For the compact Part A body every massive part is at the hub origin, so
    this reduces to a single-point field matching BSM; for the extended Part B body the
    offset panels see a slightly different field, producing the gravity-gradient
    difference.

    Args:
        withPanels (bool): include the offset flexible panels (Part B) or not (Part A).
        mu (float): gravitational parameter [m^3/s^2]
        dt (float): integrator time step [s]
        tf (float): final simulation time [s]
        recordDt (float): recorder sampling period [s]
        record (bool, optional): attach hub, center-of-mass, and wheel recorders.
            Defaults to True.

    Returns:
        tuple: ``(simulation, outputs, handles)``.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(mujocoModel(withPanels))
    scene.ModelTag = "hubMj"
    scene.extraEoMCall = True
    scene.highOrderAttitudeIntegration = True
    scSim.AddModelToTask("dynTask", scene, 1)

    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)

    hub = scene.getBody("hub")

    actuatorMsgs = []
    for i, name in enumerate(["rw_x", "rw_y", "rw_z"]):
        actuator = scene.getSingleActuator(name)
        cmdMsg = messaging.SingleActuatorMsg()
        cmdMsg.write(messaging.SingleActuatorMsgPayload(input=RW_TORQUE[i]))
        actuator.actuatorInMsg.subscribeTo(cmdMsg)
        actuatorMsgs.append(cmdMsg)

    # The factory shares the Earth descriptor with the BSM setup, creates NBodyGravity,
    # and registers every scene body as a target. Including the near-massless wheel
    # bodies keeps the net force equal to the full system weight.
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.mu = mu  # [m^3/s^2]
    earth.isCentralBody = True
    gravity = gravFactory.addBodiesTo(scene)

    hubRecorder = comRecorder = None
    systemCoM = MJSystemCoM.MJSystemCoM()
    systemCoM.ModelTag = "systemCoM"
    systemCoM.scene = scene
    scSim.AddModelToTask("dynTask", systemCoM, 1)
    wheelRecorders = []
    if record:
        hubRecorder = hub.getCenterOfMass().stateOutMsg.recorder(macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", hubRecorder, 0)
        comRecorder = systemCoM.comStatesOutMsg.recorder(macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", comRecorder, 0)
        for name in ["rw_x", "rw_y", "rw_z"]:
            wheelRec = scene.getBody(name).getScalarJoint(name).stateDotOutMsg.recorder(
                macros.sec2nano(recordDt))
            scSim.AddModelToTask("dynTask", wheelRec, 0)
            wheelRecorders.append(wheelRec)

    scSim.InitializeSimulation()
    rN, vN, _ = initialOrbitState(mu)
    if withPanels:
        for name in ["panelP", "panelM"]:
            scene.getBody(name).getScalarJoint(name).setPosition(PANEL_THETA0)
    rBN_N, vBN_N = initialMujocoRootState(withPanels, rN, vN)
    hub.setPosition(rBN_N)
    hub.setVelocity(vBN_N)
    hub.setAttitudeRate(list(OMEGA0_B))

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    handles = [scene, integrator, hub, systemCoM, gravFactory, gravity] + actuatorMsgs
    return scSim, (hubRecorder, comRecorder, wheelRecorders), handles


def runMujoco(withPanels, mu, dt, tf, recordDt):
    """Propagate the orbiting multi-body with the :ref:`MJScene<MJScene>`."""
    scSim, outputs, handles = buildMujoco(withPanels, mu, dt, tf, recordDt)
    scSim.ExecuteSimulation()
    return (*outputs, handles)


def relativePrincipalAngle(sigmaBSM, sigmaMujoco):
    """Per-sample principal rotation angle between two MRP attitude histories.

    Args:
        sigmaBSM (numpy.ndarray): BSM MRP history, shape ``(N, 3)``
        sigmaMujoco (numpy.ndarray): MuJoCo MRP history, shape ``(N, 3)``

    Returns:
        numpy.ndarray: principal angle per sample [rad].
    """
    nSamples = _comparisonValidation.requireEqualLength(
        "orbiting multibody attitude", sigmaBSM, sigmaMujoco)
    angle = np.empty(nSamples)
    for i in range(nSamples):
        dcmRel = rbk.MRP2C(sigmaBSM[i]).dot(rbk.MRP2C(sigmaMujoco[i]).T)
        # 4*atan(|sigma_rel|) is well conditioned to machine precision, unlike
        # arccos((trace-1)/2), which collapses for angles below ~1e-8 rad.
        angle[i] = 4.0*np.arctan(np.linalg.norm(rbk.C2MRP(dcmRel)))
    return angle


def gravityGradientRateEstimate(mu, oe0):
    """Angular-acceleration scale for the modeled differential point gravity.

    The numerator uses only the inertia contribution from the separation of the lumped
    body centers, because the gravity module applies one point force at each body center.
    The denominator includes the full initial system inertia about its center of mass.

    Args:
        mu (float): gravitational parameter [m^3/s^2]
        oe0 (ClassicElements): initial classical orbital elements

    Returns:
        float: angular-acceleration scale [rad/s^2].
    """
    radius = oe0.a  # [m] near-circular orbit
    masses, centers, systemCoM = initialBodyMassDistribution(True)
    pointInertia = np.zeros((3, 3))
    for mass, center in zip(masses, centers):
        offset = center - systemCoM
        pointInertia += mass*(np.dot(offset, offset)*np.eye(3) - np.outer(offset, offset))

    totalInertia = pointInertia + hubInertiaBSM()
    c = np.cos(PANEL_THETA0)
    s = np.sin(PANEL_THETA0)
    dcmPanelToHub = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])
    panelInertiaHub = dcmPanelToHub @ np.diag(PANEL_INERTIA) @ dcmPanelToHub.T
    totalInertia += 2.0*panelInertiaHub

    pointMoments = np.linalg.eigvalsh(pointInertia)
    deltaPointInertia = pointMoments[-1] - pointMoments[0]
    return 3.0*(mu/radius**3)*deltaPointInertia/(
        np.trace(totalInertia)/3.0)  # [rad/s^2]


def run(showPlots=False, saveJson=False, saveTiming=False, resultsDir=None):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write the comparison metrics to
            ``results/scenarioCompareOrbitMultibody.json``. Defaults to False.
        saveTiming (bool, optional): if True, measure the BSM-vs-MJScene wall-clock
            cost of both regimes and write
            ``results/scenarioCompareOrbitMultibody_runtime.csv``. Defaults to False.
        resultsDir (str, optional): explicit artifact directory. Defaults to
            the scenario ``results`` folder.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    dt = 0.1  # [s] resolves the ~5 s panel oscillation for the fixed-step RK4
    recordDt = 60.0  # [s]

    planet = simIncludeGravBody.BODY_DATA["earth"]
    mu = planet.mu  # [m^3/s^2]
    _, _, oe0 = initialOrbitState(mu)
    tf = 2.0*np.pi*np.sqrt(oe0.a**3/mu)  # [s] one full orbital period (~5829 s)
    targetResults = resultsPath if resultsDir is None else resultsDir

    if saveTiming:
        timingRows = []
        for label, withPanels in (("Compact (hub + wheels)", False),
                                  ("Extended (+ offset panels)", True)):
            if couldImportMujoco:
                bsmSeconds, mujocoSeconds = _runtimeTable.pairedPropagationTimes(
                    lambda wp=withPanels: buildBSM(
                        wp, mu, dt, tf, recordDt, record=False),
                    lambda wp=withPanels: buildMujoco(
                        wp, mu, dt, tf, recordDt, record=False),
                )
            else:
                bsmSeconds = _runtimeTable.medianPropagationTime(
                    lambda wp=withPanels: buildBSM(
                        wp, mu, dt, tf, recordDt, record=False))
                mujocoSeconds = None
            timingRows.append((label, bsmSeconds, mujocoSeconds))
        _runtimeTable.saveRuntimeTable(
            fileName, os.path.dirname(__file__), timingRows,
            caseHeader="Regime", resultsDir=targetResults)

    regimes = {}  # label -> dict of per-regime arrays
    for label, withPanels in (("compact", False), ("extended", True)):
        bsmState, bsmWheel, _ = runBSM(withPanels, mu, dt, tf, recordDt)
        timeAxis = np.array(bsmState.times())*macros.NANO2SEC  # [s]
        posBSM = np.array(bsmState.r_CN_N)  # [m] system center of mass
        sigmaBSM = np.array(bsmState.sigma_BN)
        wheelBSM = np.array(bsmWheel.wheelSpeeds)[:, :3]  # [rad/s]
        _comparisonValidation.validateHistory(
            f"orbiting multibody {label} BSM", timeAxis, tf, recordDt,
            position=posBSM, attitude=sigmaBSM, wheels=wheelBSM)

        regime = {"withPanels": withPanels, "time": timeAxis, "posBSM": posBSM,
                  "sigmaBSM": sigmaBSM, "wheelBSM": wheelBSM,
                  "posMujoco": None, "attError": None, "wheelError": None}

        if couldImportMujoco:
            mjHub, mjCoM, mjWheels, _ = runMujoco(withPanels, mu, dt, tf, recordDt)
            mujocoHubTimes = np.array(mjHub.times())*macros.NANO2SEC  # [s]
            mujocoComTimes = np.array(mjCoM.times())*macros.NANO2SEC  # [s]
            _comparisonValidation.validateMatchingHistories(
                f"orbiting multibody {label} MuJoCo hub/CoM",
                mujocoHubTimes, mujocoComTimes, tf, recordDt)
            _comparisonValidation.validateMatchingHistories(
                f"orbiting multibody {label} BSM/MuJoCo",
                timeAxis, mujocoHubTimes, tf, recordDt)
            regime["posMujoco"] = np.array(mjCoM.r_CN_N)  # [m]
            sigmaMujoco = np.array(mjHub.sigma_BN)
            wheelMujoco = np.column_stack([np.squeeze(np.array(rec.state))
                                           for rec in mjWheels])  # [rad/s]
            _comparisonValidation.validateHistory(
                f"orbiting multibody {label} MuJoCo", mujocoHubTimes, tf, recordDt,
                position=regime["posMujoco"], attitude=sigmaMujoco,
                wheels=wheelMujoco)
            regime["attError"] = relativePrincipalAngle(
                regime["sigmaBSM"], sigmaMujoco)  # [rad]
            regime["wheelError"] = np.linalg.norm(
                regime["wheelBSM"] - wheelMujoco, axis=1)  # [rad/s]
            regime["crossOrbit"] = np.linalg.norm(
                regime["posBSM"] - regime["posMujoco"], axis=1)  # [m]

        regime["truth"] = keplerTruth(mu, oe0, regime["time"])  # [m]
        regime["bsmVsKepler"] = np.linalg.norm(
            regime["truth"] - regime["posBSM"], axis=1)  # [m]
        regimes[label] = regime

    ggRate = gravityGradientRateEstimate(mu, oe0)  # [rad/s^2]

    if saveJson:
        writeJsonSummary(regimes, ggRate, targetResults)

    figureList = plotResults(regimes, ggRate)

    _comparePlots.finalizeFigures(figureList)
    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def writeJsonSummary(regimes, ggRate, targetResults):
    """Write a JSON summary of the comparison metrics to the ``results`` folder.

    Args:
        regimes (dict): per-regime result arrays from :func:`run`.
        ggRate (float): gravity-gradient angular-acceleration scale [rad/s^2].
        targetResults (str): directory for the JSON artifact
    """
    import json
    summary = {"scenario": fileName,
               "gravityGradientRateEstimate": float(ggRate)}
    for label, regime in regimes.items():
        entry = {
            "nSamples": int(len(regime["time"])),
            "bsmVsKeplerMax": float(np.max(regime["bsmVsKepler"])),
        }
        if regime["posMujoco"] is not None:
            entry["crossParadigmOrbitMax"] = float(np.max(regime["crossOrbit"]))
            entry["crossParadigmAttitudeMax"] = float(np.max(regime["attError"]))
            entry["crossParadigmWheelSpeedMax"] = float(np.max(regime["wheelError"]))
            # Angular-acceleration scale implied by the peak attitude difference if it
            # were 0.5*alpha*t^2; an order-of-magnitude figure to compare against
            # gravityGradientRateEstimate (the difference is a tumble-modulated libration,
            # not a clean power law).
            tEnd = regime["time"][-1]
            entry["attitudeAccelScaleFromPeak"] = float(2.0*np.max(regime["attError"])/tEnd**2)
        summary[label] = entry
    os.makedirs(targetResults, exist_ok=True)
    with open(os.path.join(targetResults, fileName+".json"), "w") as f:
        json.dump(summary, f, indent=2)


def plotResults(regimes, ggRate):
    """Build the scenario figures.

    Args:
        regimes (dict): per-regime result arrays from :func:`run`.
        ggRate (float): gravity-gradient angular-acceleration scale [rad/s^2].

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    figureList = {}
    compact = regimes["compact"]
    extended = regimes["extended"]

    # Part A: compact-body orbit accuracy. Each engine and their mutual difference track
    # the Kepler solution at the roughly 1e-7 m RK4 truncation scale.
    figureList[fileName+"_compactOrbit"], ax = plt.subplots(layout="constrained")
    ax.semilogy(compact["time"], np.maximum(compact["bsmVsKepler"], 1e-12),
                lw=4, alpha=0.4, color=COLOR_BSM, label="BSM vs Kepler")
    if compact["posMujoco"] is not None:
        mujocoVsKepler = np.linalg.norm(compact["truth"]-compact["posMujoco"], axis=1)
        ax.semilogy(compact["time"], np.maximum(mujocoVsKepler, 1e-12),
                    lw=1.3, color=COLOR_MUJOCO, label="MuJoCo vs Kepler")
        ax.semilogy(compact["time"], np.maximum(compact["crossOrbit"], 1e-12),
                    lw=1.3, ls="--", color=COLOR_REFERENCE, label="BSM vs MuJoCo")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Center-of-mass\nposition error [m]")
    ax.legend(loc="best")

    # Part B: extended-body attitude divergence from differential point forces at the
    # MuJoCo body centers. Over one orbit it builds as a tumble-modulated libration.
    figureList[fileName+"_attError"], ax = plt.subplots(
        figsize=(3.5, 2.25), layout="constrained")
    if extended["attError"] is not None:
        ax.plot(extended["time"], extended["attError"]*macros.R2D,
                color=COLOR_MUJOCO, label="BSM vs MuJoCo hub attitude")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Hub attitude difference [deg]")
    ax.legend(loc="best")

    # Total-center-of-mass orbit agreement for the compact and extended regimes.
    figureList[fileName+"_regimes"], ax = plt.subplots(
        figsize=(3.5, 2.25), layout="constrained")
    if compact["posMujoco"] is not None:
        ax.semilogy(compact["time"], np.maximum(compact["crossOrbit"], 1e-12),
                    color=COLOR_BSM, label="Compact (hub + wheels)")
        ax.semilogy(extended["time"], np.maximum(extended["crossOrbit"], 1e-12),
                    color=COLOR_MUJOCO, label="Extended (+ offset panels)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("CoM position difference [m]")
    ax.legend(loc="best")

    return figureList


if __name__ == "__main__":
    run(True, False)
