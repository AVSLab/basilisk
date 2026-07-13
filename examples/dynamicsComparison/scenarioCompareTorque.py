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
not shrink as the step is reduced, so it is round-off in MuJoCo's gyroscopic bias term
rather than truncation: the identically-zero self-term
:math:`m\,({\bf v}_{\rm lin}\times{\bf v}_{\rm lin})` is formed as the difference of two
rounded :math:`\mathcal{O}(m|{\bf v}_{\rm lin}|^2)` products driven by the orbital velocity.
It is negligible here but reported to keep the distinction explicit.

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

As an analytic check, the inertial angular momentum evolves as
:math:`{\bf H}(t) = {\bf H}_0 + \int {\bf L}_N\,dt`, where
:math:`{\bf L}_N(t)=[BN(t)]^{\rm T}{\bf L}_B` rotates with the body (a path integral along
the attitude trajectory, not :math:`{\bf L}\,t`). Both engines' angular-momentum histories
are overlaid on this reference.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareTorque.py

Illustration of Simulation Results
----------------------------------

The body angular velocity histories from the two engines overlie one another.

.. image:: /_images/Scenarios/scenarioCompareTorque_rate.svg
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

The inertial angular-momentum magnitude of each engine is overlaid on the analytic
accumulation :math:`{\bf H}_0+\int{\bf L}_N\,dt`; all three curves coincide, so both engines
track the accumulated applied torque.

.. image:: /_images/Scenarios/scenarioCompareTorque_momentum.svg
   :align: center

Runtime cost
------------

Wall-clock cost of propagating this scenario with each engine, best of several timed
repeats (the speedup ratio is the machine-independent figure). The table is generated
when the scenario or its unit test runs and loaded at documentation-build time, so the
numbers reflect the machine that built these docs.

.. csv-table:: BSM vs MuJoCo runtime for :ref:`scenarioCompareTorque`
   :file: results/scenarioCompareTorque_runtime.csv
   :header-rows: 1
   :align: center

"""

import os

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import spacecraft
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import messaging

import _runtimeTable

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import NBodyGravity
    from Basilisk.simulation import pointMassGravityModel
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

# Standard Basilisk plotting colors.
COLOR_BSM = unitTestSupport.getLineColor(0, 3)
COLOR_MUJOCO = unitTestSupport.getLineColor(1, 3)
COLOR_REFERENCE = unitTestSupport.getLineColor(2, 3)

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

MASS = 750.0  # [kg]
INERTIA_DIAG = (900.0, 800.0, 600.0)  # [kg*m^2] principal inertia about the center of mass
TORQUE_B = (0.2, -0.3, 0.4)  # [N*m] constant body-frame torque
OMEGA0_B = (0.02, -0.01, 0.03)  # [rad/s] initial body rate (off principal axis)


def mujocoModel():
    """Return the MJCF model string for a single free rigid body."""
    ix, iy, iz = INERTIA_DIAG
    return f"""
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" mass="{MASS}" fullinertia="{ix} {iy} {iz} 0 0 0"/>
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


def runBSM(mu, dt, tf, recordDt, withGravity=True):
    """Propagate the torqued body with the back-substitution :ref:`spacecraft`.

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

    Returns:
        message recorder of the spacecraft state output message.
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
        earth.isCentralBody = True
        gravFactory.addBodiesTo(scObject)

    extTorque = extForceTorque.ExtForceTorque()
    extTorque.ModelTag = "extTorque"
    torqueMsg = messaging.CmdTorqueBodyMsg()
    torqueMsg.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=list(TORQUE_B)))
    extTorque.cmdTorqueInMsg.subscribeTo(torqueMsg)
    scObject.addDynamicEffector(extTorque)
    scSim.AddModelToTask("dynTask", extTorque)

    # Orbiting: initial position/velocity on the orbit. At rest: origin, zero velocity.
    rN, vN = initialOrbitState(mu) if withGravity else (np.zeros(3), np.zeros(3))
    scObject.hub.r_CN_NInit = rN.tolist()  # [m]
    scObject.hub.v_CN_NInit = vN.tolist()  # [m/s]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[w] for w in OMEGA0_B]  # [rad/s]

    recorder = scObject.scStateOutMsg.recorder(macros.sec2nano(recordDt))
    scSim.AddModelToTask("dynTask", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()
    return recorder, [scObject, gravFactory, extTorque, torqueMsg, integrator]


def runMujoco(mu, dt, tf, recordDt, withGravity=True):
    """Propagate the same torqued body with the :ref:`MJScene<MJScene>`.

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

    Returns:
        message recorder of the body-origin site state output message.
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

    gravity = gravityModel = None
    if withGravity:
        gravity = NBodyGravity.NBodyGravity()
        gravity.ModelTag = "gravity"
        scene.AddModelToDynamicsTask(gravity)
        gravityModel = pointMassGravityModel.PointMassGravityModel()
        gravityModel.muBody = mu  # [m^3/s^2]
        gravity.addGravitySource("earth", gravityModel, True)
        gravity.addGravityTarget("hub", hub)

    torqueActuator = scene.addTorqueActuator("bodyTorque", hub.getOrigin())
    torqueMsg = messaging.TorqueAtSiteMsg()
    torqueMsg.write(messaging.TorqueAtSiteMsgPayload(torque_S=list(TORQUE_B)))
    torqueActuator.torqueInMsg.subscribeTo(torqueMsg)

    recorder = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(recordDt))
    scSim.AddModelToTask("dynTask", recorder, 0)

    scSim.InitializeSimulation()

    # Orbiting: initial position/velocity on the orbit. At rest: origin, zero velocity.
    rN, vN = initialOrbitState(mu) if withGravity else (np.zeros(3), np.zeros(3))
    hub.setPosition(rN)
    hub.setVelocity(vN)
    hub.setAttitude([0.0, 0.0, 0.0])
    hub.setAttitudeRate(list(OMEGA0_B))

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()
    return recorder, [scene, gravity, gravityModel, hub, torqueActuator, torqueMsg, integrator]


def relativePrincipalAngle(sigmaBSM, sigmaMujoco):
    """Per-sample principal rotation angle between two MRP attitude histories.

    Args:
        sigmaBSM (numpy.ndarray): BSM MRP history, shape ``(N, 3)``
        sigmaMujoco (numpy.ndarray): MuJoCo MRP history, shape ``(N, 3)``

    Returns:
        numpy.ndarray: principal angle per sample [rad].
    """
    nSamples = min(len(sigmaBSM), len(sigmaMujoco))
    angle = np.empty(nSamples)
    for i in range(nSamples):
        dcmRel = rbk.MRP2C(sigmaBSM[i]).dot(rbk.MRP2C(sigmaMujoco[i]).T)
        # Use 4*atan(|sigma_rel|), which is well conditioned down to machine precision;
        # arccos((trace-1)/2) collapses to zero for angles below ~1e-8 rad.
        angle[i] = 4.0*np.arctan(np.linalg.norm(rbk.C2MRP(dcmRel)))
    return angle


def angularMomentumInertial(sigmaHist, omegaHist):
    """Inertial-frame rotational angular momentum history.

    Args:
        sigmaHist (numpy.ndarray): MRP attitude history, shape ``(N, 3)``
        omegaHist (numpy.ndarray): body-rate history, shape ``(N, 3)`` [rad/s]

    Returns:
        numpy.ndarray: inertial angular momentum, shape ``(N, 3)`` [kg*m^2/s].
    """
    inertia = np.diag(INERTIA_DIAG)
    momentum = np.empty((len(sigmaHist), 3))
    for i in range(len(sigmaHist)):
        dcm_BN = rbk.MRP2C(sigmaHist[i])
        momentum[i] = dcm_BN.T.dot(inertia.dot(omegaHist[i]))
    return momentum


def accumulatedMomentumInertial(timeAxis, sigmaHist, momentum0):
    """Analytic inertial angular momentum from accumulating the applied torque.

    The inertial angular momentum about the center of mass must obey
    :math:`\\dot{\\bf H}_N = {\\bf L}_N(t)`, where the body-fixed torque resolved into the
    inertial frame is :math:`{\\bf L}_N(t) = [BN(t)]^{\\rm T}{\\bf L}_B`. Because the torque
    is constant only in the body frame, this is a path integral over the attitude history
    rather than a simple :math:`{\\bf L}\\,t`; here it is accumulated by the trapezoidal rule
    along the recorded attitude trajectory. This is the external reference the two engines'
    momentum histories are checked against.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        sigmaHist (numpy.ndarray): MRP attitude history, shape ``(N, 3)``
        momentum0 (numpy.ndarray): initial inertial angular momentum, shape ``(3,)``

    Returns:
        numpy.ndarray: accumulated inertial angular momentum, shape ``(N, 3)`` [kg*m^2/s].
    """
    torque_B = np.array(TORQUE_B)
    torqueInertial = np.empty((len(sigmaHist), 3))
    for i in range(len(sigmaHist)):
        torqueInertial[i] = rbk.MRP2C(sigmaHist[i]).T.dot(torque_B)  # [N*m] L_N
    accumulated = np.empty((len(sigmaHist), 3))
    accumulated[0] = momentum0
    for i in range(1, len(sigmaHist)):
        dt = timeAxis[i] - timeAxis[i-1]
        accumulated[i] = accumulated[i-1] + 0.5*dt*(torqueInertial[i] + torqueInertial[i-1])
    return accumulated


def run(showPlots=False, saveJson=False, saveTiming=False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write the comparison metrics to
            ``results/scenarioCompareTorque.json``. Defaults to False.
        saveTiming (bool, optional): if True, measure the BSM-vs-MJScene wall-clock
            cost of this scenario and write the runtime table embedded in the docstring
            (``results/scenarioCompareTorque_runtime.csv`` and the documentation copy).
            Defaults to False.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    dt = 0.1  # [s]
    tf = 600.0  # [s]
    recordDt = 1.0  # [s]
    mu = simIncludeGravBody.BODY_DATA["earth"].mu  # [m^3/s^2]

    if saveTiming:
        bsmSeconds = _runtimeTable.bestOfN(lambda: runBSM(mu, dt, tf, recordDt))
        mujocoSeconds = (_runtimeTable.bestOfN(lambda: runMujoco(mu, dt, tf, recordDt))
                         if couldImportMujoco else None)
        _runtimeTable.saveRuntimeTable(
            fileName, os.path.dirname(__file__),
            [("Torqued rigid body (600 s, RK4 dt=0.1 s)", bsmSeconds, mujocoSeconds)])

    # Run each engine twice: orbiting (point-mass Earth gravity, initial orbital velocity) and
    # at rest (no gravity, zero linear velocity -- pure rotation under the body torque). Neither
    # gravity nor the translation it induces can torque a body about its own center of mass, so
    # the attitude must be identical in both cases; the attitude change between them is therefore
    # a direct per-engine error measure.
    bsmOrbitRec, _ = runBSM(mu, dt, tf, recordDt, withGravity=True)
    bsmRestRec, _ = runBSM(mu, dt, tf, recordDt, withGravity=False)
    timeAxis = np.array(bsmOrbitRec.times())*macros.NANO2SEC  # [s]
    sigmaBSM = np.array(bsmOrbitRec.sigma_BN)
    sigmaBSMRest = np.array(bsmRestRec.sigma_BN)
    omegaBSM = np.array(bsmOrbitRec.omega_BN_B)  # [rad/s]
    bsmMotionAtt = relativePrincipalAngle(sigmaBSM, sigmaBSMRest)

    attError = attErrorRest = mujocoMotionAtt = None
    sigmaMujoco = omegaMujoco = None
    if couldImportMujoco:
        mujocoOrbitRec, _ = runMujoco(mu, dt, tf, recordDt, withGravity=True)
        mujocoRestRec, _ = runMujoco(mu, dt, tf, recordDt, withGravity=False)
        nSamples = min(len(timeAxis), len(mujocoOrbitRec.times()))
        timeAxis = timeAxis[:nSamples]
        sigmaBSM, omegaBSM = sigmaBSM[:nSamples], omegaBSM[:nSamples]
        sigmaBSMRest = sigmaBSMRest[:nSamples]
        bsmMotionAtt = bsmMotionAtt[:nSamples]
        sigmaMujoco = np.array(mujocoOrbitRec.sigma_BN)[:nSamples]
        sigmaMujocoRest = np.array(mujocoRestRec.sigma_BN)[:nSamples]
        omegaMujoco = np.array(mujocoOrbitRec.omega_BN_B)[:nSamples]  # [rad/s]
        # Per-engine attitude change between the orbiting body and the same body at rest
        # (exact answer: zero -- neither gravity nor the translation it causes can torque it).
        mujocoMotionAtt = relativePrincipalAngle(sigmaMujoco, sigmaMujocoRest)
        # Cross-engine attitude difference, orbiting and at rest. At rest there is no linear
        # velocity, so MuJoCo's velocity round-off is absent and this is the pure truncation
        # floor at which the two rotational formulations agree.
        attError = relativePrincipalAngle(sigmaBSM, sigmaMujoco)  # [rad]
        attErrorRest = relativePrincipalAngle(sigmaBSMRest, sigmaMujocoRest)

    momentumBSM = angularMomentumInertial(sigmaBSM, omegaBSM)

    # Analytic reference H(t) = H0 + integral L_N dt, accumulated along the BSM
    # attitude path, and the MuJoCo momentum history (when available). This is what lets
    # the momentum figure actually demonstrate that both engines accumulate the applied
    # torque as the analytic law requires, rather than showing one unvalidated curve.
    momentumTruth = accumulatedMomentumInertial(timeAxis, sigmaBSM, momentumBSM[0])
    momentumMujoco = (angularMomentumInertial(sigmaMujoco, omegaMujoco)
                      if sigmaMujoco is not None else None)

    if saveJson:
        writeJsonSummary(timeAxis, omegaBSM, omegaMujoco, attError, attErrorRest,
                         momentumBSM, momentumMujoco, momentumTruth,
                         bsmMotionAtt, mujocoMotionAtt)

    figureList = plotResults(timeAxis, omegaBSM, omegaMujoco, attError, attErrorRest,
                             momentumBSM, momentumMujoco, momentumTruth,
                             bsmMotionAtt, mujocoMotionAtt)

    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def writeJsonSummary(timeAxis, omegaBSM, omegaMujoco, attError, attErrorRest,
                     momentumBSM, momentumMujoco, momentumTruth,
                     bsmMotionAtt, mujocoMotionAtt):
    """Write a JSON summary of the comparison metrics to the ``results`` folder.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        omegaBSM (numpy.ndarray): BSM body-rate history [rad/s]
        omegaMujoco (numpy.ndarray): MuJoCo body-rate history [rad/s] (or None)
        attError (numpy.ndarray): cross-engine attitude error, orbiting [rad] (or None)
        attErrorRest (numpy.ndarray): cross-engine attitude error, at rest [rad] (or None)
        momentumBSM (numpy.ndarray): BSM angular momentum [kg*m^2/s]
        momentumMujoco (numpy.ndarray): MuJoCo angular momentum [kg*m^2/s] (or None)
        momentumTruth (numpy.ndarray): analytic accumulated angular momentum [kg*m^2/s]
        bsmMotionAtt (numpy.ndarray): BSM attitude change, orbiting vs at rest [rad]
        mujocoMotionAtt (numpy.ndarray): MuJoCo attitude change, orbiting vs at rest [rad]
            (or None). The exact answer is zero (motion cannot torque the body); the BSM
            value is at machine precision, the MuJoCo value at its gyroscopic-bias round-off.
    """
    import json
    summary = {
        "scenario": fileName,
        "nSamples": int(len(timeAxis)),
        "momentumScale": float(np.mean(np.linalg.norm(momentumBSM, axis=1))),
        # Max error of each engine's angular momentum against the analytic accumulation
        # H0 + integral L_N dt -- the independent check that both engines conserve/accumulate
        # the applied torque correctly, not merely that they agree with each other.
        "bsmMomentumVsAccumMax": float(
            np.max(np.linalg.norm(momentumBSM-momentumTruth, axis=1))),
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
        summary["mujocoMomentumVsAccumMax"] = float(
            np.max(np.linalg.norm(momentumMujoco-momentumTruth, axis=1)))
        summary["mujocoMotionAttitudeMax"] = float(np.max(mujocoMotionAtt))
    os.makedirs(resultsPath, exist_ok=True)
    with open(os.path.join(resultsPath, fileName+".json"), "w") as f:
        json.dump(summary, f, indent=2)


def plotResults(timeAxis, omegaBSM, omegaMujoco, attError, attErrorRest,
                momentumBSM, momentumMujoco, momentumTruth,
                bsmMotionAtt, mujocoMotionAtt):
    """Build the scenario figures.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        omegaBSM (numpy.ndarray): BSM body-rate history [rad/s]
        omegaMujoco (numpy.ndarray): MuJoCo body-rate history [rad/s] (or None)
        attError (numpy.ndarray): cross-engine attitude error, orbiting [rad] (or None)
        attErrorRest (numpy.ndarray): cross-engine attitude error, at rest [rad] (or None)
        momentumBSM (numpy.ndarray): BSM angular momentum [kg*m^2/s]
        momentumMujoco (numpy.ndarray): MuJoCo angular momentum [kg*m^2/s] (or None)
        momentumTruth (numpy.ndarray): analytic accumulated angular momentum [kg*m^2/s]
        bsmMotionAtt (numpy.ndarray): BSM attitude change, orbiting vs at rest [rad]
        mujocoMotionAtt (numpy.ndarray): MuJoCo attitude change, orbiting vs at rest [rad]
            (or None)

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    timeMin = timeAxis/60.0  # [min]
    figureList = {}

    # Color encodes the vector component; the two engines share it. BSM is a thick
    # translucent underlay, MuJoCo a thin opaque line on top, so both stay visible.
    figureList[fileName+"_rate"], ax = plt.subplots()
    for i in range(3):
        color = unitTestSupport.getLineColor(i, 3)
        ax.plot(timeMin, omegaBSM[:, i], "-", lw=4, alpha=0.4, color=color,
                label=r"BSM $\omega_" + "xyz"[i] + "$")
        if omegaMujoco is not None:
            ax.plot(timeMin, omegaMujoco[:, i], "-", lw=1.3, color=color,
                    label=r"MuJoCo $\omega_" + "xyz"[i] + "$")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel(r"$\omega_{BN}$ [rad/s]")
    ax.legend(ncol=3, fontsize=8, loc="best")

    # Attitude change of each engine between the orbiting body and the same body at rest.
    # The exact answer is zero; BSM sits at machine precision, MuJoCo at its velocity-driven
    # gyroscopic-bias round-off floor.
    figureList[fileName+"_motionAttError"], ax = plt.subplots()
    ax.semilogy(timeMin, np.maximum(bsmMotionAtt, 1e-18), color=COLOR_BSM,
                label="BSM")
    if mujocoMotionAtt is not None:
        ax.semilogy(timeMin, np.maximum(mujocoMotionAtt, 1e-18), color=COLOR_MUJOCO,
                    label="MuJoCo")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Attitude change, orbiting vs at rest [rad]")
    ax.legend(loc="best")

    # Cross-engine attitude difference, at rest and orbiting. At rest the two formulations
    # agree at the RK4 truncation floor; the orbital velocity raises the difference to
    # MuJoCo's velocity round-off level.
    figureList[fileName+"_attError"], ax = plt.subplots()
    if attError is not None:
        ax.semilogy(timeMin, np.maximum(attErrorRest, 1e-16), color=COLOR_BSM,
                    label="At rest (no linear velocity)")
        ax.semilogy(timeMin, np.maximum(attError, 1e-16), color=COLOR_MUJOCO,
                    label="Orbiting (~7.5 km/s)")
        ax.legend(loc="best")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Cross-engine principal angle of relative DCM [rad]")

    # Analytic accumulation, BSM momentum, and MuJoCo momentum overlap: analytic as a thin
    # reference line, BSM a thick translucent underlay, MuJoCo a thin line on top. Confirms
    # both engines accumulate the applied body torque per the angular-momentum law.
    figureList[fileName+"_momentum"], ax = plt.subplots()
    ax.plot(timeMin, np.linalg.norm(momentumTruth, axis=1), "-", lw=1.5,
            color=COLOR_REFERENCE, label=r"Analytic ${\bf H}_0+\int{\bf L}_N\,dt$")
    ax.plot(timeMin, np.linalg.norm(momentumBSM, axis=1), "-", lw=4, alpha=0.4,
            color=COLOR_BSM, label="Back-substitution (BSM)")
    if momentumMujoco is not None:
        ax.plot(timeMin, np.linalg.norm(momentumMujoco, axis=1), "-", lw=1.3,
                color=COLOR_MUJOCO, label="MuJoCo")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel(r"$|{\bf H}_{C/N}|$ [kg m$^2$/s]")
    ax.legend(loc="best")

    return figureList


if __name__ == "__main__":
    run(True, False)
