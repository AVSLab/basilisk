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
Third scenario in the dynamics-engine comparison series (see
:ref:`scenarioCompareOrbit` for the introduction).

This scenario models a free-floating spacecraft: a central hub carrying three
orthogonal reaction wheels (commanded with constant motor torques) and two symmetric
solar arrays modeled as torsional spring-damper hinged rigid bodies, initially
deflected. With gravity off, the comparison isolates the internal multi-body coupling:
wheel reaction torques, panel flexing, and the hub's rigid-body response.

Matching the two engines requires accounting for how each models the components:

#. **Reaction wheels.** Basilisk's back-substitution *balanced* :ref:`reactionWheelStateEffector`
   adds no mass or inertia to the system; it injects only the spin-axis reaction and
   gyroscopic terms. MuJoCo models each wheel as a real spinning rigid body. The two are
   made equivalent by folding each wheel's inertia tensor into the BSM hub inertia,
   setting the BSM ``Js`` to the wheel axial inertia, and giving the MuJoCo wheel bodies
   negligible mass. A non-zero wheel mass would shift the system center of mass
   differently in the two formulations once the offset panels are attached.

#. **Hinged panels.** The :ref:`hingedRigidBodyStateEffector` is a torsional
   spring(``k``)-damper(``c``) hinge with the panel center of mass a distance ``d`` from
   the hinge axis. The equivalent MuJoCo construction is a ``hinge`` joint with matching
   ``stiffness`` and ``damping``, the child body's center of mass offset by ``-d`` along
   its local x-axis (the effector's sign convention), and a matching inertia tensor.

The hub attitude is compared with the principal rotation angle of the relative direction
cosine matrix, as in :ref:`scenarioCompareTorque`. The MuJoCo scene is run with
``highOrderAttitudeIntegration`` enabled so the hub quaternion is advanced at the
integrator's full order, matching BSM's MRP attitude order.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareRwPanels.py

Illustration of Simulation Results
----------------------------------

The hub angular velocities from the two engines lie on top of one another as the wheels
spin up and the panels oscillate.

.. image:: /_images/Scenarios/scenarioCompareRwPanels_rate.svg
   :align: center

The panel hinge angles match between the two engines through the damped oscillation.

.. image:: /_images/Scenarios/scenarioCompareRwPanels_panels.svg
   :align: center

The cross-engine hub attitude difference sits at floating-point round-off for this
eleven-degree-of-freedom system.

.. image:: /_images/Scenarios/scenarioCompareRwPanels_attError.svg
   :align: center

Runtime cost
------------

The wall-clock cost of propagating this system with each engine is tabulated below
(best of several timed repeats; the speedup is the machine-independent figure). The
table is generated when the scenario or its unit test runs and loaded at
documentation-build time, so the numbers reflect the machine that built these docs.

.. csv-table:: BSM vs MuJoCo runtime for :ref:`scenarioCompareRwPanels`
   :file: results/scenarioCompareRwPanels_runtime.csv
   :header-rows: 1
   :align: center

"""

import os

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import spacecraft
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import hingedRigidBodyStateEffector
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import messaging

import _runtimeTable

try:
    from Basilisk.simulation import mujoco
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

# Standard Basilisk plotting colors.
COLOR_BSM = unitTestSupport.getLineColor(0, 3)
COLOR_MUJOCO = unitTestSupport.getLineColor(1, 3)

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

HUB_MASS = 600.0  # [kg]
HUB_CORE_INERTIA = (400.0, 380.0, 360.0)  # [kg*m^2] bare hub, before wheel inertia is folded in
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


def mujocoModel():
    """Return the MJCF model: hub, three reaction wheels, two hinged panels."""
    ix, iy, iz = HUB_CORE_INERTIA
    ip = PANEL_INERTIA
    return f"""
<mujoco>
  <option gravity="0 0 0"/>
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
      </body>

      <body name="panelP" pos="{PANEL_HINGE_X} 0 0">
        <joint name="panelP" type="hinge" axis="0 1 0" stiffness="{PANEL_K}" damping="{PANEL_C}" springref="0"/>
        <inertial pos="{-PANEL_D} 0 0" mass="{PANEL_MASS}" fullinertia="{ip[0]} {ip[1]} {ip[2]} 0 0 0"/>
      </body>
      <body name="panelM" pos="{-PANEL_HINGE_X} 0 0">
        <joint name="panelM" type="hinge" axis="0 1 0" stiffness="{PANEL_K}" damping="{PANEL_C}" springref="0"/>
        <inertial pos="{-PANEL_D} 0 0" mass="{PANEL_MASS}" fullinertia="{ip[0]} {ip[1]} {ip[2]} 0 0 0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="rw_x" joint="rw_x"/>
    <motor name="rw_y" joint="rw_y"/>
    <motor name="rw_z" joint="rw_z"/>
  </actuator>
</mujoco>
"""


def runBSM(dt, tf, recordDt):
    """Propagate the hub-wheel-panel system with the back-substitution :ref:`spacecraft`.

    Args:
        dt (float): integrator time step [s]
        tf (float): final simulation time [s]
        recordDt (float): recorder sampling period [s]

    Returns:
        tuple: ``(stateRecorder, wheelRecorder, panelRecorders, handles)``.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "hub"
    scObject.hub.mHub = HUB_MASS  # [kg]
    scObject.hub.IHubPntBc_B = hubInertiaBSM().tolist()  # [kg*m^2]
    scObject.hub.omega_BN_BInit = [[w] for w in OMEGA0_B]  # [rad/s]
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

    panels = []
    for hingeX in (PANEL_HINGE_X, -PANEL_HINGE_X):
        panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
        panel.mass = PANEL_MASS  # [kg]
        panel.IPntS_S = np.diag(PANEL_INERTIA).tolist()  # [kg*m^2]
        panel.d = PANEL_D  # [m]
        panel.k = PANEL_K  # [N*m/rad]
        panel.c = PANEL_C  # [N*m*s/rad]
        panel.r_HB_B = [[hingeX], [0.0], [0.0]]  # [m]
        panel.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        panel.thetaInit = PANEL_THETA0  # [rad]
        panel.thetaDotInit = 0.0  # [rad/s]
        panel.ModelTag = "panel" + ("P" if hingeX > 0 else "M")
        scObject.addStateEffector(panel)
        scSim.AddModelToTask("dynTask", panel)
        panels.append(panel)

    stateRecorder = scObject.scStateOutMsg.recorder(macros.sec2nano(recordDt))
    scSim.AddModelToTask("dynTask", stateRecorder)
    wheelRecorder = rwEffector.rwSpeedOutMsg.recorder(macros.sec2nano(recordDt))
    scSim.AddModelToTask("dynTask", wheelRecorder)
    panelRecorders = []
    for panel in panels:
        panelRec = panel.hingedRigidBodyOutMsg.recorder(macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", panelRec)
        panelRecorders.append(panelRec)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()
    handles = [scObject, integrator, rwEffector, rwFactory, rwCmdMsg] + panels
    return stateRecorder, wheelRecorder, panelRecorders, handles


def runMujoco(dt, tf, recordDt):
    """Propagate the same system with the :ref:`MJScene<MJScene>`.

    Args:
        dt (float): integrator time step [s]
        tf (float): final simulation time [s]
        recordDt (float): recorder sampling period [s]

    Returns:
        tuple: ``(stateRecorder, wheelRecorders, panelRecorders, handles)``.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(mujocoModel())
    scene.ModelTag = "hubMj"
    scene.extraEoMCall = True
    # Advance the hub quaternion at the integrator's full order; MuJoCo's default single
    # exponential map of the stage-averaged body rate would floor the attitude error at
    # second order.
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

    stateRecorder = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(recordDt))
    scSim.AddModelToTask("dynTask", stateRecorder, 0)
    wheelRecorders = []
    for name in ["rw_x", "rw_y", "rw_z"]:
        wheelRec = scene.getBody(name).getScalarJoint(name).stateDotOutMsg.recorder(
            macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", wheelRec, 0)
        wheelRecorders.append(wheelRec)
    panelRecorders = []
    for name in ["panelP", "panelM"]:
        panelRec = scene.getBody(name).getScalarJoint(name).stateOutMsg.recorder(
            macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", panelRec, 0)
        panelRecorders.append(panelRec)

    scSim.InitializeSimulation()
    hub.setAttitudeRate(list(OMEGA0_B))
    for name in ["panelP", "panelM"]:
        scene.getBody(name).getScalarJoint(name).setPosition(PANEL_THETA0)

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()
    handles = [scene, integrator, hub] + actuatorMsgs
    return stateRecorder, wheelRecorders, panelRecorders, handles


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


def run(showPlots=False, saveJson=False, saveTiming=False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write the comparison metrics to
            ``results/scenarioCompareRwPanels.json``. Defaults to False.
        saveTiming (bool, optional): if True, measure the BSM-vs-MJScene wall-clock
            cost of this scenario and write the runtime table embedded in the docstring
            (``results/scenarioCompareRwPanels_runtime.csv`` and the documentation copy).
            Defaults to False.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    dt = 0.02  # [s]
    tf = 120.0  # [s]
    recordDt = 0.5  # [s]

    if saveTiming:
        bsmSeconds = _runtimeTable.bestOfN(lambda: runBSM(dt, tf, recordDt))
        mujocoSeconds = (_runtimeTable.bestOfN(lambda: runMujoco(dt, tf, recordDt))
                         if couldImportMujoco else None)
        _runtimeTable.saveRuntimeTable(
            fileName, os.path.dirname(__file__),
            [("Hub + 3 RWs + 2 flex panels, 11 DOF (120 s, dt=0.02 s)",
              bsmSeconds, mujocoSeconds)])

    bsmState, bsmWheel, bsmPanels, _ = runBSM(dt, tf, recordDt)
    timeAxis = np.array(bsmState.times())*macros.NANO2SEC  # [s]
    sigmaBSM = np.array(bsmState.sigma_BN)
    omegaBSM = np.array(bsmState.omega_BN_B)  # [rad/s]
    wheelBSM = np.array(bsmWheel.wheelSpeeds)[:, :3]  # [rad/s]
    panelBSM = np.column_stack([np.squeeze(np.array(rec.theta))
                                    for rec in bsmPanels])  # [rad]

    attError = None
    omegaMujoco = wheelMujoco = panelMujoco = None
    if couldImportMujoco:
        mjState, mjWheels, mjPanels, _ = runMujoco(dt, tf, recordDt)
        nSamples = min(len(timeAxis), len(mjState.times()))
        timeAxis = timeAxis[:nSamples]
        sigmaBSM, omegaBSM = sigmaBSM[:nSamples], omegaBSM[:nSamples]
        wheelBSM, panelBSM = wheelBSM[:nSamples], panelBSM[:nSamples]
        sigmaMujoco = np.array(mjState.sigma_BN)[:nSamples]
        omegaMujoco = np.array(mjState.omega_BN_B)[:nSamples]  # [rad/s]
        wheelMujoco = np.column_stack([np.squeeze(np.array(rec.state))
                                       for rec in mjWheels])[:nSamples]  # [rad/s]
        panelMujoco = np.column_stack([np.squeeze(np.array(rec.state))
                                       for rec in mjPanels])[:nSamples]  # [rad]
        attError = relativePrincipalAngle(sigmaBSM, sigmaMujoco)  # [rad]

    if saveJson:
        writeJsonSummary(timeAxis, omegaBSM, omegaMujoco, wheelBSM, wheelMujoco,
                         panelBSM, panelMujoco, attError)

    figureList = plotResults(timeAxis, omegaBSM, omegaMujoco,
                             panelBSM, panelMujoco, attError)

    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def writeJsonSummary(timeAxis, omegaBSM, omegaMujoco, wheelBSM, wheelMujoco,
                     panelBSM, panelMujoco, attError):
    """Write a JSON summary of the comparison metrics to the ``results`` folder.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        omegaBSM (numpy.ndarray): BSM hub-rate history [rad/s]
        omegaMujoco (numpy.ndarray): MuJoCo hub-rate history [rad/s] (or None)
        wheelBSM (numpy.ndarray): BSM wheel-speed history [rad/s]
        wheelMujoco (numpy.ndarray): MuJoCo wheel-speed history [rad/s] (or None)
        panelBSM (numpy.ndarray): BSM panel-angle history [rad]
        panelMujoco (numpy.ndarray): MuJoCo panel-angle history [rad] (or None)
        attError (numpy.ndarray): cross-engine attitude error [rad] (or None)
    """
    import json
    summary = {"scenario": fileName, "nSamples": int(len(timeAxis))}
    if attError is not None:
        summary["hubAttitudePrincipalAngleMax"] = float(np.max(attError))
        summary["hubBodyRateErrorMax"] = float(
            np.max(np.linalg.norm(omegaBSM-omegaMujoco, axis=1)))
        summary["wheelSpeedErrorMax"] = float(
            np.max(np.linalg.norm(wheelBSM-wheelMujoco, axis=1)))
        summary["panelAngleErrorMax"] = float(
            np.max(np.linalg.norm(panelBSM-panelMujoco, axis=1)))
    os.makedirs(resultsPath, exist_ok=True)
    with open(os.path.join(resultsPath, fileName+".json"), "w") as f:
        json.dump(summary, f, indent=2)


def plotResults(timeAxis, omegaBSM, omegaMujoco,
                panelBSM, panelMujoco, attError):
    """Build the scenario figures.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        omegaBSM (numpy.ndarray): BSM hub-rate history [rad/s]
        omegaMujoco (numpy.ndarray): MuJoCo hub-rate history [rad/s] (or None)
        panelBSM (numpy.ndarray): BSM panel-angle history [rad]
        panelMujoco (numpy.ndarray): MuJoCo panel-angle history [rad] (or None)
        attError (numpy.ndarray): cross-engine attitude error [rad] (or None)

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    figureList = {}

    # Color encodes the vector component; the two engines share it. BSM is a thick
    # translucent underlay, MuJoCo a thin opaque line on top, so both stay visible
    # under perfect overlap.
    figureList[fileName+"_rate"], ax = plt.subplots()
    for i in range(3):
        color = unitTestSupport.getLineColor(i, 3)
        ax.plot(timeAxis, omegaBSM[:, i], "-", lw=4, alpha=0.4, color=color,
                label=r"BSM $\omega_" + "xyz"[i] + "$")
        if omegaMujoco is not None:
            ax.plot(timeAxis, omegaMujoco[:, i], "-", lw=1.3, color=color,
                    label=r"MuJoCo $\omega_" + "xyz"[i] + "$")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$\omega_{BN}$ [rad/s]")
    ax.legend(ncol=3, fontsize=8, loc="best")

    figureList[fileName+"_panels"], ax = plt.subplots()
    panelLabels = ("+x array", "-x array")
    for i in range(panelBSM.shape[1]):
        color = unitTestSupport.getLineColor(i, panelBSM.shape[1])
        ax.plot(timeAxis, panelBSM[:, i]*macros.R2D, "-", lw=4, alpha=0.4, color=color,
                label="BSM " + panelLabels[i])
        if panelMujoco is not None:
            ax.plot(timeAxis, panelMujoco[:, i]*macros.R2D, "-", lw=1.3, color=color,
                    label="MuJoCo " + panelLabels[i])
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Panel hinge angle [deg]")
    ax.legend(fontsize=8, loc="best")

    figureList[fileName+"_attError"], ax = plt.subplots()
    if attError is not None:
        ax.semilogy(timeAxis, np.maximum(attError, 1e-16), color=COLOR_MUJOCO)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Hub principal angle of relative DCM [rad]")

    return figureList


if __name__ == "__main__":
    run(True, False)
