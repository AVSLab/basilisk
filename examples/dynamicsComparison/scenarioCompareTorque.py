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

This scenario extends the clean translational baseline to full six-degree-of-freedom
motion by adding a constant torque applied in the body frame, together with a non-zero,
off-principal-axis initial angular velocity. This exercises the coupled, gyroscopic
rotational dynamics :math:`[I]\dot{\pmb\omega} = -\pmb\omega\times[I]\pmb\omega + {\bf L}`
rather than reducing to a single-axis spin-up.

On the classic side the torque is applied with an :ref:`extForceTorque` dynamic effector
attached to the :ref:`spacecraft` hub. On the MuJoCo side the body is free (a single
free joint) and the torque is produced by an ``MJTorqueActuator`` placed at the body
origin site. Because that site rotates with the body, a constant ``torque_S`` is a
constant body-frame torque, matching the classic side exactly.

The attitudes are compared using the **principal rotation angle of the relative
direction cosine matrix**, computed from the relative attitude as
:math:`4\,\arctan(|\pmb\sigma_{\rm rel}|)`, rather than by differencing the attitude
parameters directly. This is essential because the classic engine integrates a Modified
Rodrigues Parameter set (``sigma_BN``) while MuJoCo integrates a free-joint quaternion;
the principal angle is invariant to the parameterization and is the physically meaningful
pointing error. (This form is well conditioned to machine precision, unlike
:math:`\arccos((\mathrm{tr}-1)/2)`, which loses all precision for small angles.)

The MuJoCo scene is run with ``highOrderAttitudeIntegration`` enabled so that the
free-joint quaternion is advanced at the integrator's full order, matching the order at
which the classic engine integrates its MRP attitude. Without this, MuJoCo's default
single exponential map of the stage-averaged body rate is only second-order accurate on
SO(3) and the cross-engine attitude difference would be dominated by that
formulation-mismatch rather than by the (much smaller) shared truncation error.

A useful analytic check is that the inertial angular momentum about the centre of mass
must evolve as :math:`{\bf H}(t) = {\bf H}_0 + \int {\bf L}_N\,dt`, where the
inertially-resolved torque rotates with the body. Both engines reproduce this evolution.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareTorque.py

Illustration of Simulation Results
----------------------------------

The body angular velocity histories from the two engines overlie one another.

.. image:: /_images/Scenarios/scenarioCompareTorque_rate.svg
   :align: center

The cross-engine attitude difference (the principal angle of the relative DCM) is tiny:
with matched, full-order attitude integration on both sides it sits near the shared
Runge-Kutta truncation level, confirming the two formulations describe the same rotational
dynamics rather than merely the same averaged motion.

.. image:: /_images/Scenarios/scenarioCompareTorque_attError.svg
   :align: center

The inertial angular-momentum magnitude tracks the analytic accumulation of the applied
torque.

.. image:: /_images/Scenarios/scenarioCompareTorque_momentum.svg
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

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import NBodyGravity
    from Basilisk.simulation import pointMassGravityModel
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

# Consistent palette drawn from the standard Basilisk plotting colors.
COLOR_CLASSIC = unitTestSupport.getLineColor(0, 3)
COLOR_MUJOCO = unitTestSupport.getLineColor(1, 3)

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

MASS = 750.0  # [kg]
INERTIA_DIAG = (900.0, 800.0, 600.0)  # [kg*m^2] principal inertia about the centre of mass
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


def runClassic(mu, dt, tf, recordDt):
    """Propagate the torqued body with the classic :ref:`spacecraft`.

    Args:
        mu (float): gravitational parameter [m^3/s^2]
        dt (float): integrator time step [s]
        tf (float): final simulation time [s]
        recordDt (float): recorder sampling period [s]

    Returns:
        message recorder of the spacecraft state output message.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scClassic"
    scObject.hub.mHub = MASS  # [kg]
    scObject.hub.IHubPntBc_B = np.diag(INERTIA_DIAG).tolist()  # [kg*m^2]
    scSim.AddModelToTask("dynTask", scObject)

    integrator = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integrator)

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

    rN, vN = initialOrbitState(mu)
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


def runMujoco(mu, dt, tf, recordDt):
    """Propagate the same torqued body with the MuJoCo-based :ref:`MJScene<MJScene>`.

    Args:
        mu (float): gravitational parameter [m^3/s^2]
        dt (float): integrator time step [s]
        tf (float): final simulation time [s]
        recordDt (float): recorder sampling period [s]

    Returns:
        message recorder of the body-origin site state output message.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(mujocoModel())
    scene.ModelTag = "scMujoco"
    scene.extraEoMCall = True
    # Integrate the free-joint attitude quaternion at the integrator's full order
    # (rather than MuJoCo's default single exponential map of the stage-averaged body
    # rate, which is only second order on SO(3)). This matches the order at which the
    # classic spacecraft integrates its MRP attitude, so the two engines agree on
    # attitude to the same level as on translation and rate.
    scene.highOrderAttitudeIntegration = True
    scSim.AddModelToTask("dynTask", scene, 1)

    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)

    hub = scene.getBody("hub")

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

    rN, vN = initialOrbitState(mu)
    hub.setPosition(rN)
    hub.setVelocity(vN)
    hub.setAttitude([0.0, 0.0, 0.0])
    hub.setAttitudeRate(list(OMEGA0_B))

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()
    return recorder, [scene, gravity, gravityModel, hub, torqueActuator, torqueMsg, integrator]


def relativePrincipalAngle(sigmaClassic, sigmaMujoco):
    """Per-sample principal rotation angle between two MRP attitude histories.

    Args:
        sigmaClassic (numpy.ndarray): classic MRP history, shape ``(N, 3)``
        sigmaMujoco (numpy.ndarray): MuJoCo MRP history, shape ``(N, 3)``

    Returns:
        numpy.ndarray: principal angle per sample [rad].
    """
    nSamples = min(len(sigmaClassic), len(sigmaMujoco))
    angle = np.empty(nSamples)
    for i in range(nSamples):
        dcmRel = rbk.MRP2C(sigmaClassic[i]).dot(rbk.MRP2C(sigmaMujoco[i]).T)
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


def run(showPlots=False, saveJson=False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write the comparison metrics to
            ``results/scenarioCompareTorque.json``. Defaults to False.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    dt = 0.1  # [s]
    tf = 600.0  # [s]
    recordDt = 1.0  # [s]
    mu = simIncludeGravBody.BODY_DATA["earth"].mu  # [m^3/s^2]

    classicRec, classicHandles = runClassic(mu, dt, tf, recordDt)
    timeAxis = np.array(classicRec.times())*macros.NANO2SEC  # [s]
    sigmaClassic = np.array(classicRec.sigma_BN)
    omegaClassic = np.array(classicRec.omega_BN_B)  # [rad/s]

    attError = None
    sigmaMujoco = omegaMujoco = None
    if couldImportMujoco:
        mujocoRec, mujocoHandles = runMujoco(mu, dt, tf, recordDt)
        nSamples = min(len(timeAxis), len(mujocoRec.times()))
        timeAxis = timeAxis[:nSamples]
        sigmaClassic, omegaClassic = sigmaClassic[:nSamples], omegaClassic[:nSamples]
        sigmaMujoco = np.array(mujocoRec.sigma_BN)[:nSamples]
        omegaMujoco = np.array(mujocoRec.omega_BN_B)[:nSamples]  # [rad/s]
        attError = relativePrincipalAngle(sigmaClassic, sigmaMujoco)  # [rad]

    momentumClassic = angularMomentumInertial(sigmaClassic, omegaClassic)

    if saveJson:
        writeJsonSummary(timeAxis, omegaClassic, omegaMujoco, attError, momentumClassic)

    figureList = plotResults(timeAxis, omegaClassic, omegaMujoco,
                             attError, momentumClassic)

    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def writeJsonSummary(timeAxis, omegaClassic, omegaMujoco, attError, momentumClassic):
    """Write a JSON summary of the comparison metrics to the ``results`` folder.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        omegaClassic (numpy.ndarray): classic body-rate history [rad/s]
        omegaMujoco (numpy.ndarray): MuJoCo body-rate history [rad/s] (or None)
        attError (numpy.ndarray): cross-engine attitude error [rad] (or None)
        momentumClassic (numpy.ndarray): classic angular momentum [kg*m^2/s]
    """
    import json
    summary = {
        "scenario": fileName,
        "nSamples": int(len(timeAxis)),
        "momentumScale": float(np.mean(np.linalg.norm(momentumClassic, axis=1))),
    }
    if attError is not None:
        omegaError = np.linalg.norm(omegaClassic-omegaMujoco, axis=1)
        summary["attitudePrincipalAngleMax"] = float(np.max(attError))
        summary["bodyRateErrorMax"] = float(np.max(omegaError))
    os.makedirs(resultsPath, exist_ok=True)
    with open(os.path.join(resultsPath, fileName+".json"), "w") as f:
        json.dump(summary, f, indent=2)


def plotResults(timeAxis, omegaClassic, omegaMujoco, attError, momentumClassic):
    """Build the scenario figures.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        omegaClassic (numpy.ndarray): classic body-rate history [rad/s]
        omegaMujoco (numpy.ndarray): MuJoCo body-rate history [rad/s] (or None)
        attError (numpy.ndarray): cross-engine attitude error [rad] (or None)
        momentumClassic (numpy.ndarray): classic angular momentum [kg*m^2/s]

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    timeMin = timeAxis/60.0  # [min]
    figureList = {}

    # Color encodes the vector component; the two engines share a color and would sit
    # exactly on top of each other. Classic is drawn as a thick translucent underlay and
    # MuJoCo as a thin opaque line on top, so the halo stays visible under perfect overlap.
    figureList[fileName+"_rate"], ax = plt.subplots()
    for i in range(3):
        color = unitTestSupport.getLineColor(i, 3)
        ax.plot(timeMin, omegaClassic[:, i], "-", lw=4, alpha=0.4, color=color,
                label=r"Classic $\omega_" + "xyz"[i] + "$")
        if omegaMujoco is not None:
            ax.plot(timeMin, omegaMujoco[:, i], "-", lw=1.3, color=color,
                    label=r"MuJoCo $\omega_" + "xyz"[i] + "$")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel(r"$\omega_{BN}$ [rad/s]")
    ax.legend(ncol=3, fontsize=8, loc="best")

    figureList[fileName+"_attError"], ax = plt.subplots()
    if attError is not None:
        ax.semilogy(timeMin, np.maximum(attError, 1e-16), color=COLOR_MUJOCO)
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Principal angle of relative DCM [rad]")

    figureList[fileName+"_momentum"], ax = plt.subplots()
    ax.plot(timeMin, np.linalg.norm(momentumClassic, axis=1), color=COLOR_CLASSIC)
    ax.set_xlabel("Time [min]")
    ax.set_ylabel(r"$|{\bf H}_{C/N}|$ [kg m$^2$/s]")

    return figureList


if __name__ == "__main__":
    run(True, False)
