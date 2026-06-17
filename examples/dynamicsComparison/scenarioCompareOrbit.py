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
First scenario in the dynamics-engine comparison series!

Basilisk offers two ways to propagate rigid-body dynamics, and this family of
scenarios compares them head-to-head on equivalent problems of increasing
complexity. The two engines are:

#. The classic :ref:`spacecraft` :ref:`DynamicObject<dynamicObject>`, which uses the
   *back-substitution* method. This formulation is hub-centric: one body is the "hub"
   and carries the 6-DOF rigid-body state, while ``StateEffectors`` and
   ``DynamicEffectors`` add their own degrees of freedom and external loads. Both the
   hub and the effectors are described in minimal (generalized) coordinates, and their
   coupled equations of motion are derived analytically and solved each integrator
   stage by back-substituting the hub acceleration into the effector states.
#. The MuJoCo-based :ref:`MJScene<MJScene>` :ref:`DynamicObject<dynamicObject>`, which
   also uses minimal coordinates but with no privileged hub: bodies are connected by
   joints, the state is the set of joint coordinates, and the coupled accelerations are
   produced by MuJoCo's general recursive multi-body algorithm. See
   :ref:`scenarioReactionWheel` for a gentle introduction to :ref:`MJScene<MJScene>`.

The central observation that makes the comparison fair is that **both engines are
``DynamicObject`` subclasses advanced by the same Basilisk Runge-Kutta integrator**.
They expose an ``equationsOfMotion`` that the shared ``integrateState`` steps in time.
Consequently, if we match the integrator, the time step, the initial conditions, the
mass properties, and the applied forces, then any difference in the resulting
trajectories is attributable to the *formulation* (the hub-centric analytical
back-substitution versus MuJoCo's recursive multi-body solver), not to the
time-stepping.

This first scenario is the cleanest possible case: a single point-mass spacecraft on
a Keplerian orbit about a point-mass Earth, with no other forces. On the classic side
this is a :ref:`spacecraft` with a point-mass gravity effector. On the MuJoCo side it
is a single body with three orthogonal sliding joints (pure translation, no rotation),
with gravity supplied by :ref:`NBodyGravity` and :ref:`pointMassGravityModel` on the
scene's dynamics task.

Because both engines integrate the identical ordinary differential equation
:math:`\ddot{\bf r} = -\mu {\bf r}/|{\bf r}|^3` with the same RK4 scheme, they should
agree to near machine precision, and both should match the analytic Kepler propagation
to the level of the shared integrator truncation error.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareOrbit.py

Illustration of Simulation Results
----------------------------------

The inertial trajectory of both engines lies on top of the analytic two-body solution.

.. image:: /_images/Scenarios/scenarioCompareOrbit_trajectory.svg
   :align: center

Each engine matches the analytic Kepler propagation to the shared RK4 truncation level
(centimetres over two orbits at a 10-second step), and the two curves are
indistinguishable from one another.

.. image:: /_images/Scenarios/scenarioCompareOrbit_accuracy.svg
   :align: center

The difference between the two engines is at the level of floating-point round-off,
confirming they integrate the identical equations of motion.

.. image:: /_images/Scenarios/scenarioCompareOrbit_crossError.svg
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
from Basilisk.simulation import spacecraft
from Basilisk.simulation import svIntegrators

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import NBodyGravity
    from Basilisk.simulation import pointMassGravityModel
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

# Consistent palette drawn from the standard Basilisk plotting colors so the classic,
# MuJoCo, and analytic-reference curves are distinguishable across all comparison plots.
COLOR_CLASSIC = unitTestSupport.getLineColor(0, 3)
COLOR_MUJOCO = unitTestSupport.getLineColor(1, 3)
COLOR_REFERENCE = unitTestSupport.getLineColor(2, 3)

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

# Single body free to translate in three axes (no rotational degrees of freedom)
ORBIT_SCENE_XML = r"""
<mujoco>
  <worldbody>
    <body name="sat">
      <joint name="sat_x" axis="1 0 0" type="slide"/>
      <joint name="sat_y" axis="0 1 0" type="slide"/>
      <joint name="sat_z" axis="0 0 1" type="slide"/>
      <inertial pos="0 0 0" mass="750" diaginertia="900 800 600"/>
    </body>
  </worldbody>
</mujoco>
"""


def initialOrbitState(mu):
    """Return the initial inertial position and velocity and the classical elements.

    Args:
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        tuple: ``(rN, vN, oe)`` with position [m], velocity [m/s], and the
        ``ClassicElements`` object used to generate them.
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


def runClassic(mass, mu, dt, tf, recordDt):
    """Propagate the orbit with the classic back-substitution :ref:`spacecraft`.

    Args:
        mass (float): spacecraft mass [kg]
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
    scObject.hub.mHub = mass  # [kg]
    scObject.hub.IHubPntBc_B = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]  # [kg*m^2]
    scSim.AddModelToTask("dynTask", scObject)

    # The integrator must be kept referenced for the lifetime of the simulation,
    # otherwise the Python wrapper is garbage-collected and the underlying object
    # is destroyed while the spacecraft still points at it.
    integrator = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integrator)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    gravFactory.addBodiesTo(scObject)

    rN, vN, _ = initialOrbitState(mu)
    scObject.hub.r_CN_NInit = rN.tolist()  # [m]
    scObject.hub.v_CN_NInit = vN.tolist()  # [m/s]

    recorder = scObject.scStateOutMsg.recorder(macros.sec2nano(recordDt))
    scSim.AddModelToTask("dynTask", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()
    return recorder


def runMujoco(mu, dt, tf, recordDt):
    """Propagate the same orbit with the MuJoCo-based :ref:`MJScene<MJScene>`.

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

    scene = mujoco.MJScene(ORBIT_SCENE_XML)
    scene.ModelTag = "scMujoco"
    scene.extraEoMCall = True
    scSim.AddModelToTask("dynTask", scene, 1)

    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)

    sat = scene.getBody("sat")

    # MuJoCo's built-in gravity is disabled inside MJScene, so we supply the same
    # point-mass field that the classic gravity effector uses.
    gravity = NBodyGravity.NBodyGravity()
    gravity.ModelTag = "gravity"
    scene.AddModelToDynamicsTask(gravity)
    gravityModel = pointMassGravityModel.PointMassGravityModel()
    gravityModel.muBody = mu  # [m^3/s^2]
    gravity.addGravitySource("earth", gravityModel, True)
    gravity.addGravityTarget("sat", sat)

    recorder = sat.getCenterOfMass().stateOutMsg.recorder(macros.sec2nano(recordDt))
    scSim.AddModelToTask("dynTask", recorder, 0)

    scSim.InitializeSimulation()

    # Free-body initial conditions are set after initialization.
    rN, vN, _ = initialOrbitState(mu)
    sat.setPosition(rN)
    sat.setVelocity(vN)

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()
    return recorder


def keplerTruth(mu, semiMajorAxis, oe0, times):
    """Analytic two-body position history at the requested sample times.

    Args:
        mu (float): gravitational parameter [m^3/s^2]
        semiMajorAxis (float): orbit semi-major axis [m]
        oe0 (ClassicElements): initial classical orbital elements
        times (numpy.ndarray): sample times [s]

    Returns:
        numpy.ndarray: inertial position history, shape ``(N, 3)`` [m].
    """
    meanMotion = np.sqrt(mu/semiMajorAxis**3)  # [rad/s]
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


def run(showPlots=False, saveJson=False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write a summary of the comparison metrics
            to ``results/scenarioCompareOrbit.json``. Defaults to False.

    Returns:
        dict: mapping from figure name to matplotlib figure, as expected by the
        scenario unit test.
    """
    mass = 750.0  # [kg]
    dt = 10.0  # [s]
    recordDt = 60.0  # [s]

    planet = simIncludeGravBody.BODY_DATA["earth"]
    mu = planet.mu  # [m^3/s^2]
    _, _, oe0 = initialOrbitState(mu)
    orbitPeriod = 2.*np.pi*np.sqrt(oe0.a**3/mu)  # [s]
    tf = 2.*orbitPeriod  # [s]

    classicRec = runClassic(mass, mu, dt, tf, recordDt)
    timeAxis = np.array(classicRec.times())*macros.NANO2SEC  # [s]
    posClassic = np.array(classicRec.r_CN_N)  # [m]
    velClassic = np.array(classicRec.v_CN_N)  # [m/s]

    posMujoco = velMujoco = None
    if couldImportMujoco:
        mujocoRec = runMujoco(mu, dt, tf, recordDt)
        nSamples = min(len(timeAxis), len(mujocoRec.times()))
        timeAxis = timeAxis[:nSamples]
        posClassic, velClassic = posClassic[:nSamples], velClassic[:nSamples]
        posMujoco = np.array(mujocoRec.r_BN_N)[:nSamples]  # [m]
        velMujoco = np.array(mujocoRec.v_BN_N)[:nSamples]  # [m/s]

    truth = keplerTruth(mu, oe0.a, oe0, timeAxis)
    errClassic = np.linalg.norm(truth-posClassic, axis=1)  # [m]
    errMujoco = (np.linalg.norm(truth-posMujoco, axis=1)
                 if posMujoco is not None else None)  # [m]
    crossError = (np.linalg.norm(posClassic-posMujoco, axis=1)
                  if posMujoco is not None else None)  # [m]

    if saveJson:
        writeJsonSummary(timeAxis, errClassic, errMujoco, crossError,
                         mu, posClassic, velClassic, posMujoco, velMujoco)

    figureList = plotResults(timeAxis, truth, posClassic, posMujoco,
                             errClassic, errMujoco, crossError)

    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def writeJsonSummary(timeAxis, errClassic, errMujoco, crossError,
                     mu, posClassic, velClassic, posMujoco, velMujoco):
    """Write a JSON summary of the comparison metrics to the ``results`` folder.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        errClassic (numpy.ndarray): classic-vs-analytic position error [m]
        errMujoco (numpy.ndarray): MuJoCo-vs-analytic position error [m]
        crossError (numpy.ndarray): classic-vs-MuJoCo position difference [m]
        mu (float): gravitational parameter [m^3/s^2]
        posClassic (numpy.ndarray): classic position history [m]
        velClassic (numpy.ndarray): classic velocity history [m/s]
        posMujoco (numpy.ndarray): MuJoCo position history [m]
        velMujoco (numpy.ndarray): MuJoCo velocity history [m/s]
    """
    import json

    def specificEnergy(pos, vel):
        return 0.5*np.linalg.norm(vel, axis=1)**2 - mu/np.linalg.norm(pos, axis=1)

    energyClassic = specificEnergy(posClassic, velClassic)
    summary = {
        "scenario": fileName,
        "nSamples": int(len(timeAxis)),
        "classicVsAnalyticMax": float(np.max(errClassic)),
        "classicEnergyDrift": float(np.max(np.abs(energyClassic-energyClassic[0]))
                                    / abs(energyClassic[0])),
    }
    if crossError is not None:
        summary["mujocoVsAnalyticMax"] = float(np.max(errMujoco))
        summary["crossParadigmPosMax"] = float(np.max(crossError))
        energyMujoco = specificEnergy(posMujoco, velMujoco)
        summary["mujocoEnergyDrift"] = float(np.max(np.abs(energyMujoco-energyMujoco[0]))
                                             / abs(energyMujoco[0]))
    os.makedirs(resultsPath, exist_ok=True)
    with open(os.path.join(resultsPath, fileName+".json"), "w") as f:
        json.dump(summary, f, indent=2)


def plotResults(timeAxis, truth, posClassic, posMujoco,
                errClassic, errMujoco, crossError):
    """Build the scenario figures.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        truth (numpy.ndarray): analytic position history [m]
        posClassic (numpy.ndarray): classic position history [m]
        posMujoco (numpy.ndarray): MuJoCo position history [m] (or None)
        errClassic (numpy.ndarray): classic-vs-analytic error [m]
        errMujoco (numpy.ndarray): MuJoCo-vs-analytic error [m] (or None)
        crossError (numpy.ndarray): classic-vs-MuJoCo difference [m] (or None)

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    timeHours = timeAxis/3600.0  # [hr]
    figureList = {}

    # The analytic, classic, and MuJoCo trajectories lie on top of one another, so the
    # classic curve is drawn as a thick translucent underlay and MuJoCo as a thin line on
    # top: the halo stays visible even where the two engines overlap exactly.
    figureList[fileName+"_trajectory"], ax = plt.subplots()
    ax.plot(truth[:, 0]/1e3, truth[:, 1]/1e3, "-", lw=1.5,
            color=COLOR_REFERENCE, label="Analytic Kepler")
    ax.plot(posClassic[:, 0]/1e3, posClassic[:, 1]/1e3, "-", lw=4, alpha=0.4,
            color=COLOR_CLASSIC, label="Classic (back-substitution)")
    if posMujoco is not None:
        ax.plot(posMujoco[:, 0]/1e3, posMujoco[:, 1]/1e3, "-", lw=1.3,
                color=COLOR_MUJOCO, label="MuJoCo (MJScene)")
    ax.set_xlabel("Inertial x [km]")
    ax.set_ylabel("Inertial y [km]")
    ax.axis("equal")
    ax.legend(loc="best")

    figureList[fileName+"_accuracy"], ax = plt.subplots()
    ax.semilogy(timeHours, np.maximum(errClassic, 1e-12), lw=4, alpha=0.4,
                color=COLOR_CLASSIC, label="Classic vs Kepler")
    if errMujoco is not None:
        ax.semilogy(timeHours, np.maximum(errMujoco, 1e-12), lw=1.3,
                    color=COLOR_MUJOCO, label="MuJoCo vs Kepler")
    ax.set_xlabel("Time [hr]")
    ax.set_ylabel("Position error [m]")
    ax.legend(loc="best")

    figureList[fileName+"_crossError"], ax = plt.subplots()
    if crossError is not None:
        ax.semilogy(timeHours, np.maximum(crossError, 1e-15), color=COLOR_MUJOCO)
    ax.set_xlabel("Time [hr]")
    ax.set_ylabel(r"$\|{\bf r}_{\rm classic}-{\bf r}_{\rm MuJoCo}\|$ [m]")

    return figureList


if __name__ == "__main__":
    run(True, False)
