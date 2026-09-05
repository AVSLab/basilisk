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
Second scenario in the dynamics-engine comparison series (see
:ref:`scenarioCompareOrbit` for the introduction).

This scenario extends the translational baseline to full six-degree-of-freedom motion
with a constant body-frame torque and a non-zero, off-principal-axis initial angular
velocity, exercising the coupled gyroscopic rotational dynamics
:math:`[I]\dot{\pmb\omega} = -\pmb\omega\times[I]\pmb\omega + {\bf L}`.

On the BSM side the torque is applied with an :ref:`extForceTorque` dynamic effector
attached to the :ref:`spacecraft` hub. On the MuJoCo side the body is free (a single
free joint) and the torque is produced by an ``MJTorqueActuator`` placed at the body
origin site. Because that site rotates with the body, a constant ``torque_S`` is a
constant body-frame torque, matching the BSM side exactly.

Each engine is run twice, both with a fixed-step RK4 integrator at a small enough time step
(0.1 s) that time-stepping truncation is well below the effect of interest:

* **orbiting** -- on a Keplerian orbit with point-mass Earth gravity (:ref:`spacecraft`
  gravity effector; MuJoCo :ref:`NBodyGravity`), carrying the ~7.5 km/s
  orbital velocity; and
* **at rest** -- at the origin with zero linear velocity and no gravity, i.e. pure rotation
  under the body torque.

For this single compact body neither gravity nor the translation it induces can torque
the body about its center of mass, so the attitude must be *identical* in both cases. BSM
passes exactly: its attitude is bit-for-bit the same orbiting or at rest, because it
integrates attitude about the body and never references the inertial state. MuJoCo carries
a small (:math:`\sim 10^{-6}` rad) spurious attitude change between the two cases that does
not shrink as the step is reduced. MuJoCo 3.11.0 forms the spatial bias force in ``mj_rne``
as :math:`{\bf v}\times^*([I]{\bf v})`. For pure translation about the body center of mass,
the rotational part contains the identically-zero term
:math:`{\bf v}_{\rm lin}\times(m{\bf v}_{\rm lin})`, evaluated as the difference of two
rounded :math:`\mathcal{O}(m|{\bf v}_{\rm lin}|^2)` products. Directly recording
``qfrc_bias`` strongly supports this term as the source of the attitude change.

.. note::

    At 7.5 km/s along the generic direction :math:`[2,-3,6]/7`, a nonrotating free body
    has an initial rotational ``qfrc_bias`` of
    :math:`[-3.11\times10^{-7},\,1.56\times10^{-6},\,7.82\times10^{-7}]` N m. The same body moving
    along a coordinate axis has zero bias and zero attitude drift. In a torque-free,
    nonrotating propagation, each factor-of-two increase in speed multiplies the 600 s
    attitude error by approximately four; doubling the mass multiplies it by approximately
    two. These controls match the :math:`m|{\bf v}|^2` cancellation-error scale.

Attitudes are compared by the **principal rotation angle of the relative direction cosine
matrix**, :math:`4\,\arctan(|\pmb\sigma_{\rm rel}|)`, rather than by differencing attitude
parameters: BSM integrates a Modified Rodrigues Parameter set (``sigma_BN``) while MuJoCo
integrates a free-joint quaternion, and the principal angle is parameterization-invariant
and the physically meaningful pointing error. This form stays well conditioned for small
angles, unlike :math:`\arccos((\mathrm{tr}-1)/2)`.

The MuJoCo scene runs with ``highOrderAttitudeIntegration`` so the free-joint quaternion is
advanced at the integrator's full order, matching BSM's MRP attitude order. MuJoCo's default
single exponential map of the stage-averaged body rate is only second-order on SO(3), which
would otherwise dominate the cross-engine attitude difference.

An independent control restricts the initial rate and torque to the body z principal axis.
The gyroscopic cross term then vanishes, giving the exact solution

.. math::

    \omega_z(t) = \omega_{z0} + \frac{L_z}{I_z}t,\qquad
    \theta_z(t) = \omega_{z0}t + \frac{L_z}{2I_z}t^2 .

Both engines are compared directly with these rate and attitude histories.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareTorque.py

Illustration of Simulation Results
----------------------------------

The body angular velocity histories from the two engines overlie one another.

.. image:: /_images/Scenarios/scenarioCompareTorque_rate_x.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareTorque_rate_y.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareTorque_rate_z.svg
   :align: center

Each engine's attitude change between the orbiting body and the same body at rest should be
zero, since motion cannot torque the body about its center of mass. BSM sits at machine
precision (:math:`\sim 10^{-16}` rad), identical orbiting or at rest; MuJoCo sits about ten
orders higher (:math:`\sim 10^{-6}` rad). The gap does not close as the time step is reduced,
identifying it as round-off in MuJoCo's gyroscopic bias rather than a time-stepping error.

.. image:: /_images/Scenarios/scenarioCompareTorque_motionAttError.svg
   :align: center

The cross-engine attitude difference (BSM versus MuJoCo, principal angle of the relative DCM)
shows the same effect directly. **At rest** the two formulations agree at the RK4 truncation
floor (:math:`\sim 10^{-8}` rad); **orbiting**, the difference rises to the
:math:`\sim 10^{-6}` rad round-off the orbital velocity injects into MuJoCo.

.. image:: /_images/Scenarios/scenarioCompareTorque_attError.svg
   :align: center

The principal-axis control compares each engine directly with the closed-form rate and attitude.

.. image:: /_images/Scenarios/scenarioCompareTorque_analyticRateError.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareTorque_analyticAttError.svg
   :align: center

Runtime cost
------------

Wall-clock cost of propagating this scenario with each engine, reported as the median
of five measured trials after one discarded warm-up. Model setup is excluded. Both the
absolute times and speedup ratio are specific to the machine and build.
Generate the optional local CSV with
``make -C docs comparison-runtime-tables``; see
:ref:`scenarioCompareOrbit` for the benchmark requirements and interpretation.

Next comparison: :ref:`scenarioCompareRwPanels` adds reaction wheels and hinged
flexible appendages.

"""

import os

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import spacecraft
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import messaging

import _runtimeTable
import _comparePlots
import _comparisonValidation

from Basilisk import hasBuildFeature

couldImportMujoco = hasBuildFeature("mujoco")
if couldImportMujoco:
    from Basilisk.simulation import mujoco

# Paul Tol high-contrast palette shared by the comparison figures.
COLOR_BSM = _comparePlots.COLOR_BSM
COLOR_MUJOCO = _comparePlots.COLOR_MUJOCO

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

MASS = 750.0  # [kg]
INERTIA_DIAG = (900.0, 800.0, 600.0)  # [kg*m^2] principal inertia about the center of mass
TORQUE_B = (0.2, -0.3, 0.4)  # [N*m] constant body-frame torque
OMEGA0_B = (0.02, -0.01, 0.03)  # [rad/s] initial body rate (off principal axis)
ANALYTIC_AXIS = 2  # body z principal axis
ANALYTIC_TORQUE_B = (0.0, 0.0, 0.4)  # [N*m]
ANALYTIC_OMEGA0_B = (0.0, 0.0, 0.03)  # [rad/s]


def mujocoModel(mass=MASS, inertiaDiag=INERTIA_DIAG):
    """Return the MJCF model string for a single free rigid body.

    Args:
        mass (float, optional): body mass [kg].
        inertiaDiag (tuple, optional): principal inertia values [kg*m^2].
    """
    ix, iy, iz = inertiaDiag
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
      <inertial pos="0 0 0" mass="{mass}" fullinertia="{ix} {iy} {iz} 0 0 0"/>
    </body>
  </worldbody>
</mujoco>
"""


def initialOrbitState(mu):
    """Return the initial inertial position and velocity for the orbit.

    Args:
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        tuple: ``(rN, vN)`` position [m] and velocity [m/s].
    """
    oe = orbitalMotion.ClassicElements()
    oe.a = 7000.0e3  # [m]
    oe.e = 0.01
    oe.i = 33.3*macros.D2R  # [rad]
    oe.Omega = 48.2*macros.D2R  # [rad]
    oe.omega = 347.8*macros.D2R  # [rad]
    oe.f = 85.3*macros.D2R  # [rad]
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    return np.array(rN), np.array(vN)


def buildBSM(mu, dt, tf, recordDt, withGravity=True, record=True,
             torque_B=TORQUE_B, omega0_B=OMEGA0_B):
    """Build the Backsubstitution torque simulation without propagating it.

    Args:
        mu (float): gravitational parameter [m^3/s^2]
        dt (float): integrator time step [s]
        tf (float): final simulation time [s]
        recordDt (float): recorder sampling period [s]
        withGravity (bool, optional): if True, place the body on a Keplerian orbit with
            point-mass Earth gravity (the "orbiting" case). If False, start it at rest at the
            origin with no gravity (the "at rest" case: pure rotation under the body torque).
            Neither gravity nor the induced translation can torque the body about its center
            of mass, so the attitude history is identical either way. Defaults to True.
        record (bool, optional): attach the state recorder. Defaults to True.
        torque_B (tuple, optional): constant body-frame torque [N*m].
        omega0_B (tuple, optional): initial body rate [rad/s].

    Returns:
        tuple: ``(simulation, recorder, handles)``.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scBSM"
    scObject.hub.mHub = MASS  # [kg]
    scObject.hub.IHubPntBc_B = np.diag(INERTIA_DIAG).tolist()  # [kg*m^2]
    scSim.AddModelToTask("dynTask", scObject)

    integrator = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integrator)

    gravFactory = None
    if withGravity:
        gravFactory = simIncludeGravBody.gravBodyFactory()
        earth = gravFactory.createEarth()
        earth.mu = mu  # [m^3/s^2]
        earth.isCentralBody = True
        gravFactory.addBodiesTo(scObject)

    extTorque = extForceTorque.ExtForceTorque()
    extTorque.ModelTag = "extTorque"
    torqueMsg = messaging.CmdTorqueBodyMsg()
    torqueMsg.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=list(torque_B)))
    extTorque.cmdTorqueInMsg.subscribeTo(torqueMsg)
    scObject.addDynamicEffector(extTorque)
    scSim.AddModelToTask("dynTask", extTorque)

    # Orbiting: initial position/velocity on the orbit. At rest: origin, zero velocity.
    rN, vN = initialOrbitState(mu) if withGravity else (np.zeros(3), np.zeros(3))
    scObject.hub.r_CN_NInit = rN.tolist()  # [m]
    scObject.hub.v_CN_NInit = vN.tolist()  # [m/s]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[w] for w in omega0_B]  # [rad/s]

    recorder = None
    if record:
        recorder = scObject.scStateOutMsg.recorder(macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    handles = [scObject, gravFactory, extTorque, torqueMsg, integrator]
    return scSim, recorder, handles


def runBSM(mu, dt, tf, recordDt, withGravity=True,
           torque_B=TORQUE_B, omega0_B=OMEGA0_B):
    """Propagate the torqued body with the Backsubstitution :ref:`spacecraft`."""
    scSim, recorder, handles = buildBSM(
        mu, dt, tf, recordDt, withGravity, torque_B=torque_B, omega0_B=omega0_B)
    scSim.ExecuteSimulation()
    return recorder, handles


def buildMujoco(mu, dt, tf, recordDt, withGravity=True, record=True,
                torque_B=TORQUE_B, omega0_B=OMEGA0_B):
    """Build the MuJoCo torque simulation without propagating it.

    Args:
        mu (float): gravitational parameter [m^3/s^2]
        dt (float): integrator time step [s]
        tf (float): final simulation time [s]
        recordDt (float): recorder sampling period [s]
        withGravity (bool, optional): if True, place the body on the Keplerian orbit with the
            same point-mass Earth field via :ref:`NBodyGravity` (the "orbiting" case). If
            False, start it at rest at the origin with no gravity (the "at rest" case). For
            this compact single body gravity is a pure center-of-mass force with no
            gravity-gradient torque, so the attitude is identical either way. Defaults to True.
        record (bool, optional): attach the state recorder. Defaults to True.
        torque_B (tuple, optional): constant body-frame torque [N*m].
        omega0_B (tuple, optional): initial body rate [rad/s].

    Returns:
        tuple: ``(simulation, recorder, handles)``.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(mujocoModel())
    scene.ModelTag = "scMujoco"
    scene.extraEoMCall = True
    # Advance the free-joint quaternion at the integrator's full order to match BSM's MRP
    # attitude order; MuJoCo's default single exponential map is only second order on SO(3).
    scene.highOrderAttitudeIntegration = True
    scSim.AddModelToTask("dynTask", scene, 1)

    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)

    hub = scene.getBody("hub")

    gravFactory = gravity = None
    if withGravity:
        # The factory creates NBodyGravity and registers every MJScene body as a
        # target, matching the gravity-body setup used by the BSM model.
        gravFactory = simIncludeGravBody.gravBodyFactory()
        earth = gravFactory.createEarth()
        earth.mu = mu  # [m^3/s^2]
        earth.isCentralBody = True
        gravity = gravFactory.addBodiesTo(scene)

    torqueActuator = scene.addTorqueActuator("bodyTorque", hub.getOrigin())
    torqueMsg = messaging.TorqueAtSiteMsg()
    torqueMsg.write(messaging.TorqueAtSiteMsgPayload(torque_S=list(torque_B)))
    torqueActuator.torqueInMsg.subscribeTo(torqueMsg)

    recorder = None
    if record:
        recorder = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", recorder, 0)

    scSim.InitializeSimulation()

    # Orbiting: initial position/velocity on the orbit. At rest: origin, zero velocity.
    rN, vN = initialOrbitState(mu) if withGravity else (np.zeros(3), np.zeros(3))
    hub.setPosition(rN)
    hub.setVelocity(vN)
    hub.setAttitude([0.0, 0.0, 0.0])
    hub.setAttitudeRate(list(omega0_B))

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    handles = [scene, gravFactory, gravity, hub, torqueActuator, torqueMsg, integrator]
    return scSim, recorder, handles


def runMujoco(mu, dt, tf, recordDt, withGravity=True,
              torque_B=TORQUE_B, omega0_B=OMEGA0_B):
    """Propagate the same torqued body with the :ref:`MJScene<MJScene>`."""
    scSim, recorder, handles = buildMujoco(
        mu, dt, tf, recordDt, withGravity, torque_B=torque_B, omega0_B=omega0_B)
    scSim.ExecuteSimulation()
    return recorder, handles


def relativePrincipalAngle(sigmaBSM, sigmaMujoco):
    """Per-sample principal rotation angle between two MRP attitude histories.

    Args:
        sigmaBSM (numpy.ndarray): BSM MRP history, shape ``(N, 3)``
        sigmaMujoco (numpy.ndarray): MuJoCo MRP history, shape ``(N, 3)``

    Returns:
        numpy.ndarray: principal angle per sample [rad].
    """
    nSamples = _comparisonValidation.requireEqualLength(
        "torque attitude", sigmaBSM, sigmaMujoco)
    angle = np.empty(nSamples)
    for i in range(nSamples):
        dcmRel = rbk.MRP2C(sigmaBSM[i]).dot(rbk.MRP2C(sigmaMujoco[i]).T)
        # Use 4*atan(|sigma_rel|), which is well conditioned down to machine precision;
        # arccos((trace-1)/2) collapses to zero for angles below ~1e-8 rad.
        angle[i] = 4.0*np.arctan(np.linalg.norm(rbk.C2MRP(dcmRel)))
    return angle


def principalAxisAnalyticSolution(timeAxis):
    """Return the exact body rate and rotation angle for the principal-axis control."""
    alpha = ANALYTIC_TORQUE_B[ANALYTIC_AXIS] / INERTIA_DIAG[ANALYTIC_AXIS]
    omega = np.zeros((len(timeAxis), 3))
    omega[:, ANALYTIC_AXIS] = (
        ANALYTIC_OMEGA0_B[ANALYTIC_AXIS] + alpha*timeAxis)
    theta = ANALYTIC_OMEGA0_B[ANALYTIC_AXIS]*timeAxis + 0.5*alpha*timeAxis**2
    return omega, theta


def principalAxisAttitudeError(sigmaHist, theta):
    """Return the principal-angle error relative to the exact principal-axis attitude."""
    _comparisonValidation.requireEqualLength(
        "principal-axis attitude", sigmaHist, theta)
    axis = np.zeros(3)
    axis[ANALYTIC_AXIS] = 1.0
    error = np.empty(len(theta))
    for i, angle in enumerate(theta):
        dcmRelative = rbk.MRP2C(sigmaHist[i]).dot(
            rbk.PRV2C(axis*angle).T)
        error[i] = 4.0*np.arctan(np.linalg.norm(rbk.C2MRP(dcmRelative)))
    return error


def run(showPlots=False, saveJson=False, saveTiming=False, resultsDir=None):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write the comparison metrics to
            ``results/scenarioCompareTorque.json``. Defaults to False.
        saveTiming (bool, optional): if True, measure the BSM-vs-MJScene wall-clock
            cost of this scenario and write
            ``results/scenarioCompareTorque_runtime.csv``. Defaults to False.
        resultsDir (str, optional): explicit artifact directory. Defaults to
            the scenario ``results`` folder.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    dt = 0.1  # [s]
    tf = 600.0  # [s]
    recordDt = 1.0  # [s]
    mu = simIncludeGravBody.BODY_DATA["earth"].mu  # [m^3/s^2]
    targetResults = resultsPath if resultsDir is None else resultsDir

    if saveTiming:
        if couldImportMujoco:
            bsmSeconds, mujocoSeconds = _runtimeTable.pairedPropagationTimes(
                lambda: buildBSM(mu, dt, tf, recordDt, record=False),
                lambda: buildMujoco(mu, dt, tf, recordDt, record=False),
            )
        else:
            bsmSeconds = _runtimeTable.medianPropagationTime(
                lambda: buildBSM(mu, dt, tf, recordDt, record=False))
            mujocoSeconds = None
        _runtimeTable.saveRuntimeTable(
            fileName, os.path.dirname(__file__),
            [("Torqued rigid body (600 s, RK4 dt=0.1 s)",
              bsmSeconds, mujocoSeconds)],
            resultsDir=targetResults)

    # Run each engine twice: orbiting (point-mass Earth gravity, initial orbital velocity) and
    # at rest (no gravity, zero linear velocity -- pure rotation under the body torque). Neither
    # gravity nor the translation it induces can torque a body about its own center of mass, so
    # the attitude must be identical in both cases; the attitude change between them is therefore
    # a direct per-engine error measure.
    bsmOrbitRec, _ = runBSM(mu, dt, tf, recordDt, withGravity=True)
    bsmRestRec, _ = runBSM(mu, dt, tf, recordDt, withGravity=False)
    timeAxis = np.array(bsmOrbitRec.times())*macros.NANO2SEC  # [s]
    bsmRestTimes = np.array(bsmRestRec.times())*macros.NANO2SEC  # [s]
    _comparisonValidation.validateMatchingHistories(
        "torque BSM orbit/rest", timeAxis, bsmRestTimes, tf, recordDt)
    sigmaBSM = np.array(bsmOrbitRec.sigma_BN)
    sigmaBSMRest = np.array(bsmRestRec.sigma_BN)
    omegaBSM = np.array(bsmOrbitRec.omega_BN_B)  # [rad/s]
    _comparisonValidation.validateHistory(
        "torque BSM orbit", timeAxis, tf, recordDt,
        attitude=sigmaBSM, rate=omegaBSM)
    _comparisonValidation.validateHistory(
        "torque BSM rest", bsmRestTimes, tf, recordDt,
        attitude=sigmaBSMRest)
    bsmMotionAtt = relativePrincipalAngle(sigmaBSM, sigmaBSMRest)

    attError = attErrorRest = mujocoMotionAtt = None
    sigmaMujoco = omegaMujoco = None
    if couldImportMujoco:
        mujocoOrbitRec, _ = runMujoco(mu, dt, tf, recordDt, withGravity=True)
        mujocoRestRec, _ = runMujoco(mu, dt, tf, recordDt, withGravity=False)
        mujocoOrbitTimes = np.array(mujocoOrbitRec.times())*macros.NANO2SEC  # [s]
        mujocoRestTimes = np.array(mujocoRestRec.times())*macros.NANO2SEC  # [s]
        _comparisonValidation.validateMatchingHistories(
            "torque BSM/MuJoCo orbit",
            timeAxis, mujocoOrbitTimes, tf, recordDt)
        _comparisonValidation.validateMatchingHistories(
            "torque MuJoCo orbit/rest",
            mujocoOrbitTimes, mujocoRestTimes, tf, recordDt)
        sigmaMujoco = np.array(mujocoOrbitRec.sigma_BN)
        sigmaMujocoRest = np.array(mujocoRestRec.sigma_BN)
        omegaMujoco = np.array(mujocoOrbitRec.omega_BN_B)  # [rad/s]
        _comparisonValidation.validateHistory(
            "torque MuJoCo orbit", mujocoOrbitTimes, tf, recordDt,
            attitude=sigmaMujoco, rate=omegaMujoco)
        _comparisonValidation.validateHistory(
            "torque MuJoCo rest", mujocoRestTimes, tf, recordDt,
            attitude=sigmaMujocoRest)
        # Per-engine attitude change between the orbiting body and the same body at rest
        # (exact answer: zero -- neither gravity nor the translation it causes can torque it).
        mujocoMotionAtt = relativePrincipalAngle(sigmaMujoco, sigmaMujocoRest)
        # Cross-engine attitude difference, orbiting and at rest. At rest there is no linear
        # velocity, so MuJoCo's velocity round-off is absent and this is the pure truncation
        # floor at which the two rotational formulations agree.
        attError = relativePrincipalAngle(sigmaBSM, sigmaMujoco)  # [rad]
        attErrorRest = relativePrincipalAngle(sigmaBSMRest, sigmaMujocoRest)

    # Independent analytic control: rotation and torque are confined to one principal axis,
    # so the gyroscopic cross term vanishes and both rate and attitude are available in closed
    # form. This complements the generic off-axis case above without weakening it.
    bsmAnalyticRec, _ = runBSM(
        mu, dt, tf, recordDt, withGravity=False,
        torque_B=ANALYTIC_TORQUE_B, omega0_B=ANALYTIC_OMEGA0_B)
    analyticTime = np.array(bsmAnalyticRec.times())*macros.NANO2SEC
    _comparisonValidation.validateMatchingHistories(
        "torque baseline/analytic BSM", timeAxis, analyticTime, tf, recordDt)
    sigmaBSMAnalytic = np.array(bsmAnalyticRec.sigma_BN)
    omegaBSMAnalytic = np.array(bsmAnalyticRec.omega_BN_B)
    omegaExact, thetaExact = principalAxisAnalyticSolution(analyticTime)
    bsmAnalyticRateError = np.linalg.norm(omegaBSMAnalytic-omegaExact, axis=1)
    bsmAnalyticAttError = principalAxisAttitudeError(
        sigmaBSMAnalytic, thetaExact)

    mujocoAnalyticRateError = mujocoAnalyticAttError = None
    if couldImportMujoco:
        mujocoAnalyticRec, _ = runMujoco(
            mu, dt, tf, recordDt, withGravity=False,
            torque_B=ANALYTIC_TORQUE_B, omega0_B=ANALYTIC_OMEGA0_B)
        mujocoAnalyticTime = np.array(mujocoAnalyticRec.times())*macros.NANO2SEC
        _comparisonValidation.validateMatchingHistories(
            "torque analytic BSM/MuJoCo",
            analyticTime, mujocoAnalyticTime, tf, recordDt)
        sigmaMujocoAnalytic = np.array(mujocoAnalyticRec.sigma_BN)
        omegaMujocoAnalytic = np.array(mujocoAnalyticRec.omega_BN_B)
        mujocoAnalyticRateError = np.linalg.norm(
            omegaMujocoAnalytic-omegaExact, axis=1)
        mujocoAnalyticAttError = principalAxisAttitudeError(
            sigmaMujocoAnalytic, thetaExact)

    if saveJson:
        writeJsonSummary(timeAxis, omegaBSM, omegaMujoco, attError, attErrorRest,
                         bsmMotionAtt, mujocoMotionAtt,
                         bsmAnalyticRateError, bsmAnalyticAttError,
                         mujocoAnalyticRateError, mujocoAnalyticAttError,
                         targetResults)

    figureList = plotResults(timeAxis, omegaBSM, omegaMujoco, attError, attErrorRest,
                             bsmMotionAtt, mujocoMotionAtt,
                             analyticTime, bsmAnalyticRateError, bsmAnalyticAttError,
                             mujocoAnalyticRateError, mujocoAnalyticAttError)

    _comparePlots.finalizeFigures(figureList)
    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def writeJsonSummary(timeAxis, omegaBSM, omegaMujoco, attError, attErrorRest,
                     bsmMotionAtt, mujocoMotionAtt,
                     bsmAnalyticRateError, bsmAnalyticAttError,
                     mujocoAnalyticRateError, mujocoAnalyticAttError,
                     targetResults):
    """Write a JSON summary of the comparison metrics to the ``results`` folder.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        omegaBSM (numpy.ndarray): BSM body-rate history [rad/s]
        omegaMujoco (numpy.ndarray): MuJoCo body-rate history [rad/s] (or None)
        attError (numpy.ndarray): cross-engine attitude error, orbiting [rad] (or None)
        attErrorRest (numpy.ndarray): cross-engine attitude error, at rest [rad] (or None)
        bsmMotionAtt (numpy.ndarray): BSM attitude change, orbiting vs at rest [rad]
        mujocoMotionAtt (numpy.ndarray): MuJoCo attitude change, orbiting vs at rest [rad]
            (or None). The exact answer is zero (motion cannot torque the body); the BSM
            value is at machine precision, the MuJoCo value at its gyroscopic-bias round-off.
        bsmAnalyticRateError (numpy.ndarray): BSM rate error in the analytic control [rad/s].
        bsmAnalyticAttError (numpy.ndarray): BSM attitude error in the analytic control [rad].
        mujocoAnalyticRateError (numpy.ndarray): MuJoCo analytic-control rate error [rad/s].
        mujocoAnalyticAttError (numpy.ndarray): MuJoCo analytic-control attitude error [rad].
        targetResults (str): directory for the JSON artifact
    """
    import json
    summary = {
        "scenario": fileName,
        "nSamples": int(len(timeAxis)),
        "analyticAxis": ("x", "y", "z")[ANALYTIC_AXIS],
        "analyticTorque": ANALYTIC_TORQUE_B[ANALYTIC_AXIS],
        "analyticInitialRate": ANALYTIC_OMEGA0_B[ANALYTIC_AXIS],
        "bsmAnalyticRateErrorMax": float(np.max(bsmAnalyticRateError)),
        "bsmAnalyticAttitudeErrorMax": float(np.max(bsmAnalyticAttError)),
        # Attitude change between the orbiting body and the same body at rest: the exact
        # answer is zero (motion cannot torque the body), so this is the pure per-engine error.
        "bsmMotionAttitudeMax": float(np.max(bsmMotionAtt)),
    }
    if attError is not None:
        omegaError = np.linalg.norm(omegaBSM-omegaMujoco, axis=1)
        # Cross-engine attitude difference: at rest (no linear velocity) the engines agree at
        # the RK4 truncation floor; orbiting, the difference rises to MuJoCo's velocity round-off.
        summary["crossEngineAttitudeRestMax"] = float(np.max(attErrorRest))
        summary["crossEngineAttitudeOrbitMax"] = float(np.max(attError))
        summary["bodyRateErrorMax"] = float(np.max(omegaError))
        summary["mujocoMotionAttitudeMax"] = float(np.max(mujocoMotionAtt))
        summary["mujocoAnalyticRateErrorMax"] = float(
            np.max(mujocoAnalyticRateError))
        summary["mujocoAnalyticAttitudeErrorMax"] = float(
            np.max(mujocoAnalyticAttError))
    os.makedirs(targetResults, exist_ok=True)
    with open(os.path.join(targetResults, fileName+".json"), "w") as f:
        json.dump(summary, f, indent=2)


def plotResults(timeAxis, omegaBSM, omegaMujoco, attError, attErrorRest,
                bsmMotionAtt, mujocoMotionAtt,
                analyticTime, bsmAnalyticRateError, bsmAnalyticAttError,
                mujocoAnalyticRateError, mujocoAnalyticAttError):
    """Build the scenario figures.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        omegaBSM (numpy.ndarray): BSM body-rate history [rad/s]
        omegaMujoco (numpy.ndarray): MuJoCo body-rate history [rad/s] (or None)
        attError (numpy.ndarray): cross-engine attitude error, orbiting [rad] (or None)
        attErrorRest (numpy.ndarray): cross-engine attitude error, at rest [rad] (or None)
        bsmMotionAtt (numpy.ndarray): BSM attitude change, orbiting vs at rest [rad]
        mujocoMotionAtt (numpy.ndarray): MuJoCo attitude change, orbiting vs at rest [rad]
            (or None)
        analyticTime (numpy.ndarray): principal-axis control sample times [s].
        bsmAnalyticRateError (numpy.ndarray): BSM analytic rate error [rad/s].
        bsmAnalyticAttError (numpy.ndarray): BSM analytic attitude error [rad].
        mujocoAnalyticRateError (numpy.ndarray): MuJoCo analytic rate error [rad/s].
        mujocoAnalyticAttError (numpy.ndarray): MuJoCo analytic attitude error [rad].

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    timeMin = timeAxis/60.0  # [min]
    figureList = {}

    figureList.update(_comparePlots.componentComparison(
        fileName+"_rate", timeMin, omegaBSM, omegaMujoco,
        r"$\omega_{BN}$", "rad/s", xlabel="Time [min]"))

    # Attitude change of each engine between the orbiting body and the same body at rest.
    # The exact answer is zero; BSM sits at machine precision, MuJoCo at its velocity-driven
    # gyroscopic-bias round-off floor. This is a per-engine error curve, not a matched overlay.
    figureList[fileName+"_motionAttError"], ax = plt.subplots(layout="constrained")
    angleFloor = np.degrees(1e-18)
    ax.semilogy(timeMin, np.maximum(np.degrees(bsmMotionAtt), angleFloor), color=COLOR_BSM,
                label="BSM")
    if mujocoMotionAtt is not None:
        ax.semilogy(timeMin, np.maximum(np.degrees(mujocoMotionAtt), angleFloor),
                    color=COLOR_MUJOCO,
                    label="MuJoCo")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Attitude difference [deg]")
    ax.legend(loc="best")

    # Cross-engine attitude difference, at rest and orbiting. At rest the two formulations
    # agree at the RK4 truncation floor; the orbital velocity raises the difference to
    # MuJoCo's velocity round-off level. This is already a cross-engine residual, not a matched
    # overlay of the same quantity.
    figureList[fileName+"_attError"], ax = plt.subplots(layout="constrained")
    if attError is not None:
        angleFloor = np.degrees(1e-16)
        ax.semilogy(timeMin, np.maximum(np.degrees(attErrorRest), angleFloor),
                    color=COLOR_BSM,
                    label="At rest (no linear velocity)")
        ax.semilogy(timeMin, np.maximum(np.degrees(attError), angleFloor),
                    color=COLOR_MUJOCO,
                    label="Orbiting (~7.5 km/s)")
        ax.legend(loc="best")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Attitude difference [deg]")

    analyticTimeMin = analyticTime/60.0
    figureList[fileName+"_analyticRateError"], ax = plt.subplots(
        figsize=(4.8, 2.2), layout="constrained")
    ax.semilogy(analyticTimeMin, np.maximum(bsmAnalyticRateError, 1e-18),
                color=COLOR_BSM, label="BSM")
    if mujocoAnalyticRateError is not None:
        ax.semilogy(analyticTimeMin, np.maximum(mujocoAnalyticRateError, 1e-18),
                    color=COLOR_MUJOCO, label="MuJoCo")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Rate error [rad/s]")
    ax.legend(loc="best")

    figureList[fileName+"_analyticAttError"], ax = plt.subplots(
        figsize=(4.8, 2.2), layout="constrained")
    angleFloor = np.degrees(1e-18)
    ax.semilogy(analyticTimeMin,
                np.maximum(np.degrees(bsmAnalyticAttError), angleFloor),
                color=COLOR_BSM, label="BSM")
    if mujocoAnalyticAttError is not None:
        ax.semilogy(analyticTimeMin,
                    np.maximum(np.degrees(mujocoAnalyticAttError), angleFloor),
                    color=COLOR_MUJOCO, label="MuJoCo")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Attitude error [deg]")
    ax.legend(loc="best")

    return figureList


if __name__ == "__main__":
    run(True, False)
