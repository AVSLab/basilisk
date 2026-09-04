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
This scenario opens the dynamics-engine comparison series. Basilisk offers two ways
to propagate rigid-body dynamics, and this series compares them on equivalent
problems of increasing complexity. The two engines are:

#. The :ref:`spacecraft` :ref:`DynamicObject<dynamicObject>` (BSM), which uses the
   *Backsubstitution Method*. This formulation is hub-centric: one body is the "hub"
   and carries the 6-DOF rigid-body state, while ``StateEffectors`` and
   ``DynamicEffectors`` add their own degrees of freedom and external loads. Both the
   hub and the effectors are described in minimal (generalized) coordinates, and their
   coupled equations of motion are derived analytically and solved each integrator
   stage by back-substituting the hub acceleration into the effector states.
#. The :ref:`MJScene<MJScene>` :ref:`DynamicObject<dynamicObject>`, which
   also uses minimal coordinates but with no privileged hub: bodies are connected by
   joints, the state is the set of joint coordinates, and the coupled accelerations are
   produced by MuJoCo's general recursive multi-body algorithm. See
   :ref:`scenarioReactionWheel` for an introduction to :ref:`MJScene<MJScene>`.

Both engines are ``DynamicObject`` subclasses advanced by the same Basilisk
Runge-Kutta integrator: each exposes an ``equationsOfMotion`` that the shared
``integrateState`` steps in time. With matched integrator, time step, initial
conditions, mass properties, and applied forces, any difference between the
trajectories is due to the *formulation* (hub-centric analytical Backsubstitution
versus MuJoCo's recursive multi-body solver), not the time-stepping.

This scenario is the simplest case: a single point-mass spacecraft on
a Keplerian orbit about a point-mass Earth, with no other forces. On the BSM side
this is a :ref:`spacecraft` with a point-mass gravity effector and
``pointMassTranslationalOnly`` enabled. On the MuJoCo side it is a single body with
three orthogonal sliding joints (pure translation, no rotation). Both engines receive
the same Earth descriptor through :ref:`simIncludeGravBody`; for MuJoCo,
``gravFactory.addBodiesTo(scene)`` creates the :ref:`NBodyGravity` model and registers
the scene body as its gravity target.

Both engines integrate :math:`\ddot{\bf r} = -\mu {\bf r}/|{\bf r}|^3` with the same
RK4 scheme, so each matches the analytic Kepler propagation to the integrator
truncation error, and they agree with each other many orders of magnitude below it.
The agreement is not bitwise: the BSM effector forms the acceleration directly while
MuJoCo forms a force and solves :math:`[M]\ddot{\bf q}={\bf f}`, leaving a small
round-off-seeded residual.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareOrbit.py

Illustration of Simulation Results
----------------------------------

The inertial trajectory of both engines lies on top of the analytic two-body solution.

.. image:: /_images/Scenarios/scenarioCompareOrbit_trajectory.svg
   :align: center

Each engine matches the analytic Kepler propagation to the RK4 truncation level
(centimeters over an aligned, approximately two-orbit horizon at a 10-second step);
the two curves are
indistinguishable.

.. image:: /_images/Scenarios/scenarioCompareOrbit_accuracy.svg
   :align: center

The difference between the two engines stays many orders of magnitude below the
truncation error, confirming they integrate the same equations of motion.

.. image:: /_images/Scenarios/scenarioCompareOrbit_crossError.svg
   :align: center

Runtime cost
------------

Wall-clock cost of propagating this scenario with each engine, reported as the median
of five measured trials after one discarded warm-up. Model setup is excluded. Both the
absolute times and speedup ratio are specific to the machine and build.

Generate local tables for every accuracy scenario with
``make -C docs comparison-runtime-tables`` from a clean, MuJoCo-enabled
Release build. The CSV files are written under
``examples/dynamicsComparison/results`` and are deliberately not embedded in
the HTML documentation because their absolute values depend on the benchmark
host. The companion paper reports the controlled measurements used for its
performance discussion. Scenario tests validate numerical and figure
behavior; they are not timing benchmarks.

Next comparison: :ref:`scenarioCompareTorque` adds full rotational dynamics and a
body-frame torque.

"""

import os

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import spacecraft
from Basilisk.simulation import svIntegrators

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
COLOR_REFERENCE = _comparePlots.COLOR_DIFF

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

# Single body free to translate in three axes (no rotational degrees of freedom)
# The model has no geoms, equalities, limits, or frictionloss, so the collision and
# constraint pipelines are disabled: every derivative evaluation then skips MuJoCo's
# broadphase and constraint-assembly bookkeeping (bit-identical results, measurably
# faster for this small model).
ORBIT_SCENE_XML = r"""
<mujoco>
  <option>
    <flag contact="disable" constraint="disable"/>
  </option>
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


def buildBSM(mass, mu, dt, tf, recordDt, record=True):
    """Build the Backsubstitution orbit simulation without propagating it.

    Args:
        mass (float): spacecraft mass [kg]
        mu (float): gravitational parameter [m^3/s^2]
        dt (float): integrator time step [s]
        tf (float): final simulation time [s]
        recordDt (float): recorder sampling period [s]
        record (bool, optional): attach the state recorder. Defaults to True.

    Returns:
        tuple: ``(simulation, recorder, handles)``.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scBSM"
    scObject.pointMassTranslationalOnly = True
    scObject.hub.mHub = mass  # [kg]
    scObject.hub.IHubPntBc_B = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]  # [kg*m^2]
    scSim.AddModelToTask("dynTask", scObject)

    # Keep a reference to the integrator; otherwise the Python wrapper is
    # garbage-collected while the spacecraft still points at it.
    integrator = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integrator)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.mu = mu  # [m^3/s^2]
    earth.isCentralBody = True
    gravFactory.addBodiesTo(scObject)

    rN, vN, _ = initialOrbitState(mu)
    scObject.hub.r_CN_NInit = rN.tolist()  # [m]
    scObject.hub.v_CN_NInit = vN.tolist()  # [m/s]

    recorder = None
    if record:
        recorder = scObject.scStateOutMsg.recorder(macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    handles = [scObject, integrator, gravFactory]
    return scSim, recorder, handles


def runBSM(mass, mu, dt, tf, recordDt):
    """Propagate the orbit with the Backsubstitution :ref:`spacecraft` (BSM)."""
    scSim, recorder, _ = buildBSM(mass, mu, dt, tf, recordDt)
    scSim.ExecuteSimulation()
    return recorder


def buildMujoco(mu, dt, tf, recordDt, record=True):
    """Build the MuJoCo orbit simulation without propagating it.

    Args:
        mu (float): gravitational parameter [m^3/s^2]
        dt (float): integrator time step [s]
        tf (float): final simulation time [s]
        recordDt (float): recorder sampling period [s]
        record (bool, optional): attach the state recorder. Defaults to True.

    Returns:
        tuple: ``(simulation, recorder, handles)``.
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

    # Use the same gravity-body descriptor and setup path as the BSM model. For an
    # MJScene, the factory creates NBodyGravity and registers every scene body as a
    # gravity target.
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.mu = mu  # [m^3/s^2]
    earth.isCentralBody = True
    gravity = gravFactory.addBodiesTo(scene)

    recorder = None
    if record:
        recorder = sat.getCenterOfMass().stateOutMsg.recorder(macros.sec2nano(recordDt))
        scSim.AddModelToTask("dynTask", recorder, 0)

    scSim.InitializeSimulation()

    # Free-body initial conditions are set after initialization.
    rN, vN, _ = initialOrbitState(mu)
    sat.setPosition(rN)
    sat.setVelocity(vN)

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    handles = [scene, integrator, sat, gravFactory, gravity]
    return scSim, recorder, handles


def runMujoco(mu, dt, tf, recordDt):
    """Propagate the same orbit with the :ref:`MJScene<MJScene>`."""
    scSim, recorder, _ = buildMujoco(mu, dt, tf, recordDt)
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


def run(showPlots=False, saveJson=False, saveTiming=False, resultsDir=None):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write a summary of the comparison metrics
            to ``results/scenarioCompareOrbit.json``. Defaults to False.
        saveTiming (bool, optional): if True, measure the BSM-vs-MJScene wall-clock
            cost of this scenario and write
            ``results/scenarioCompareOrbit_runtime.csv``. Defaults to False.
        resultsDir (str, optional): explicit artifact directory. Defaults to
            the scenario ``results`` folder.

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
    requestedTf = 2.*orbitPeriod  # [s]
    tf = _comparisonValidation.alignedHorizon(
        requestedTf, (dt, recordDt))  # [s]
    targetResults = resultsPath if resultsDir is None else resultsDir

    if saveTiming:
        if couldImportMujoco:
            bsmSeconds, mujocoSeconds = _runtimeTable.pairedPropagationTimes(
                lambda: buildBSM(mass, mu, dt, tf, recordDt, record=False),
                lambda: buildMujoco(mu, dt, tf, recordDt, record=False),
            )
        else:
            bsmSeconds = _runtimeTable.medianPropagationTime(
                lambda: buildBSM(mass, mu, dt, tf, recordDt, record=False))
            mujocoSeconds = None
        _runtimeTable.saveRuntimeTable(
            fileName, os.path.dirname(__file__),
            [("Two-body orbit (about 2 orbits, dt=10 s)",
              bsmSeconds, mujocoSeconds)],
            resultsDir=targetResults)

    bsmRec = runBSM(mass, mu, dt, tf, recordDt)
    timeAxis = np.array(bsmRec.times())*macros.NANO2SEC  # [s]
    posBSM = np.array(bsmRec.r_CN_N)  # [m]
    velBSM = np.array(bsmRec.v_CN_N)  # [m/s]
    _comparisonValidation.validateHistory(
        "orbit BSM", timeAxis, tf, recordDt, position=posBSM, velocity=velBSM)

    posMujoco = velMujoco = None
    if couldImportMujoco:
        mujocoRec = runMujoco(mu, dt, tf, recordDt)
        mujocoTimes = np.array(mujocoRec.times())*macros.NANO2SEC  # [s]
        _comparisonValidation.validateMatchingHistories(
            "orbit BSM/MuJoCo", timeAxis, mujocoTimes, tf, recordDt)
        posMujoco = np.array(mujocoRec.r_BN_N)  # [m]
        velMujoco = np.array(mujocoRec.v_BN_N)  # [m/s]
        _comparisonValidation.validateHistory(
            "orbit MuJoCo", mujocoTimes, tf, recordDt,
            position=posMujoco, velocity=velMujoco)

    truth = keplerTruth(mu, oe0.a, oe0, timeAxis)
    errBSM = np.linalg.norm(truth-posBSM, axis=1)  # [m]
    errMujoco = (np.linalg.norm(truth-posMujoco, axis=1)
                 if posMujoco is not None else None)  # [m]
    crossError = (np.linalg.norm(posBSM-posMujoco, axis=1)
                  if posMujoco is not None else None)  # [m]

    if saveJson:
        writeJsonSummary(timeAxis, errBSM, errMujoco, crossError,
                         mu, posBSM, velBSM, posMujoco, velMujoco,
                         requestedTf, tf, orbitPeriod, targetResults)

    figureList = plotResults(timeAxis, truth, posBSM, posMujoco,
                             errBSM, errMujoco, crossError)

    _comparePlots.finalizeFigures(figureList)
    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def writeJsonSummary(timeAxis, errBSM, errMujoco, crossError,
                     mu, posBSM, velBSM, posMujoco, velMujoco,
                     requestedTf, tf, orbitPeriod, targetResults):
    """Write a JSON summary of the comparison metrics to the ``results`` folder.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        errBSM (numpy.ndarray): BSM-vs-analytic position error [m]
        errMujoco (numpy.ndarray): MuJoCo-vs-analytic position error [m]
        crossError (numpy.ndarray): BSM-vs-MuJoCo position difference [m]
        mu (float): gravitational parameter [m^3/s^2]
        posBSM (numpy.ndarray): BSM position history [m]
        velBSM (numpy.ndarray): BSM velocity history [m/s]
        posMujoco (numpy.ndarray): MuJoCo position history [m]
        velMujoco (numpy.ndarray): MuJoCo velocity history [m/s]
        requestedTf (float): requested two-orbit horizon [s]
        tf (float): aligned propagation horizon [s]
        orbitPeriod (float): analytic orbital period [s]
        targetResults (str): directory for the JSON artifact
    """
    import json

    def specificEnergy(pos, vel):
        return 0.5*np.linalg.norm(vel, axis=1)**2 - mu/np.linalg.norm(pos, axis=1)

    energyBSM = specificEnergy(posBSM, velBSM)
    summary = {
        "scenario": fileName,
        "nSamples": int(len(timeAxis)),
        "requestedFinalTime": requestedTf,
        "finalTime": tf,
        "orbitPeriod": orbitPeriod,
        "orbitCount": tf/orbitPeriod,
        "bsmVsAnalyticMax": float(np.max(errBSM)),
        "bsmEnergyDrift": float(np.max(np.abs(energyBSM-energyBSM[0]))
                                    / abs(energyBSM[0])),
    }
    if crossError is not None:
        summary["mujocoVsAnalyticMax"] = float(np.max(errMujoco))
        summary["crossParadigmPosMax"] = float(np.max(crossError))
        energyMujoco = specificEnergy(posMujoco, velMujoco)
        summary["mujocoEnergyDrift"] = float(np.max(np.abs(energyMujoco-energyMujoco[0]))
                                             / abs(energyMujoco[0]))
    os.makedirs(targetResults, exist_ok=True)
    with open(os.path.join(targetResults, fileName+".json"), "w") as f:
        json.dump(summary, f, indent=2)


def plotResults(timeAxis, truth, posBSM, posMujoco,
                errBSM, errMujoco, crossError):
    """Build the scenario figures.

    Args:
        timeAxis (numpy.ndarray): sample times [s]
        truth (numpy.ndarray): analytic position history [m]
        posBSM (numpy.ndarray): BSM position history [m]
        posMujoco (numpy.ndarray): MuJoCo position history [m] (or None)
        errBSM (numpy.ndarray): BSM-vs-analytic error [m]
        errMujoco (numpy.ndarray): MuJoCo-vs-analytic error [m] (or None)
        crossError (numpy.ndarray): BSM-vs-MuJoCo difference [m] (or None)

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    timeHours = timeAxis/3600.0  # [hr]
    figureList = {}

    # The analytic, BSM, and MuJoCo trajectories lie on top of one another, so the
    # BSM curve is drawn as a thick translucent underlay and MuJoCo as a thin line on
    # top: the halo stays visible even where the two engines overlap exactly.
    figureList[fileName+"_trajectory"], ax = plt.subplots(layout="constrained")
    ax.plot(truth[:, 0]/1e3, truth[:, 1]/1e3, "-", lw=1.5,
            color=COLOR_REFERENCE, label="Analytic Kepler")
    ax.plot(posBSM[:, 0]/1e3, posBSM[:, 1]/1e3, "-", lw=4, alpha=0.4,
            color=COLOR_BSM, label="Backsubstitution (BSM)")
    if posMujoco is not None:
        ax.plot(posMujoco[:, 0]/1e3, posMujoco[:, 1]/1e3, "-", lw=1.3,
                color=COLOR_MUJOCO, label="MuJoCo")
    ax.set_xlabel("Inertial x [km]")
    ax.set_ylabel("Inertial y [km]")
    ax.axis("equal")
    ax.legend(loc="best")

    figureList[fileName+"_accuracy"], ax = plt.subplots(layout="constrained")
    ax.semilogy(timeHours, np.maximum(errBSM, 1e-12), lw=4, alpha=0.4,
                color=COLOR_BSM, label="BSM vs Kepler")
    if errMujoco is not None:
        ax.semilogy(timeHours, np.maximum(errMujoco, 1e-12), lw=1.3,
                    color=COLOR_MUJOCO, label="MuJoCo vs Kepler")
    ax.set_xlabel("Time [hr]")
    ax.set_ylabel("Position error [m]")
    ax.legend(loc="best")

    figureList[fileName+"_crossError"], ax = plt.subplots(layout="constrained")
    if crossError is not None:
        ax.semilogy(timeHours, np.maximum(crossError, 1e-15), color=COLOR_MUJOCO)
    ax.set_xlabel("Time [hr]")
    ax.set_ylabel(r"$\|{\bf r}_{\rm BSM}-{\bf r}_{\rm MuJoCo}\|$ [m]")

    return figureList


if __name__ == "__main__":
    run(True, False)
