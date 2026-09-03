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
Verification of the effector-branching extension along two complementary axes. Whereas the
branching scenarios exercise the capability at production scale, this script reduces it to a hub,
one moving-platform host state effector, and an ``extForceTorque`` dynamic effector branched onto
the host's tip segment in field-free space, where exact references exist to measure against. The
effector applies a torque about the segment origin together with a force through each of the two
channels a branching parent accepts, one resolved in the host segment frame and one in the inertial
frame.

The first axis checks the impulse-momentum theorems at one fixed integration step. The simulator's
internally tracked angular and linear momentum about the system center of mass are compared
against the integrated external impulse, and any drift away from machine precision indicates a
defect in the equations of motion themselves: a wrong reference frame in the branched force or
torque, a wrong moment arm, or a sign error in a Backsubstitution coupling matrix. The
post-processing integrals use Simpson's 1/3 rule so the residual reports the simulator's error
rather than the quadrature's.

The second axis is the order-of-accuracy check standard in computational-physics code
verification. The answer at a fixed final time is computed at successively halved step sizes, and
the convergence rate of the differences is compared against the integrator's formal order, which
is four for the default RK4. This detects a defect class the first axis cannot: stage
inconsistency, where the branched wrench is evaluated against the wrong intermediate Runge-Kutta
state, leaving each step within tolerance whereas the effective order degrades across the run.

A correct implementation must satisfy both axes. Either alone is necessary but not sufficient.

Three host classes span the moving-platform effectors that support branching, each a chain with
the dynamic effector branched onto its tip body: ``spinningBodyNDOF`` configured as three
two-degree-of-freedom segments, ``nHingedRigidBody`` as three hinged panels, and
``linearTranslationNDOF`` as three translating bodies.

The script is found in the folder ``basilisk/examples`` and executed by::

    python3 scenarioBranchingVerification.py

Illustration of Simulation Results
----------------------------------

The default ``run()`` invocation reproduces the configuration presented in the companion
journal article (citation pending publication). The pytest wrapper in ``src/tests`` drives a
shortened step ladder and horizon to keep wall time low.

The impulse-momentum residuals show the random-walk character of pure floating-point rounding,
about thirteen orders of magnitude below the momentum they bound. All three hosts are run and
their peak residuals printed, and the figure shows the cascading-rotation and translating
hosts. The step-size refinement places every host on a line of slope four, so all three
inherit the integrator's formal order through the branched code path.

.. image:: /_images/Scenarios/scenarioBranchingVerificationConservation.svg
   :align: center

.. image:: /_images/Scenarios/scenarioBranchingVerificationConvergence.svg
   :align: center
"""

#
#   Basilisk Scenario Script
#
#   Purpose:            Verify the effector-branching equations of motion.
#   Author:             Andrew Morell
#   Creation Date:      Aug 12, 2026
#

import os
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import cumulative_simpson

from Basilisk.simulation import (
    extForceTorque,
    linearTranslationNDOFStateEffector,
    nHingedRigidBodyStateEffector,
    spacecraft,
    spinningBodyNDOFStateEffector,
)
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, macros


fileName = os.path.basename(os.path.splitext(__file__)[0])

class HostProps:
    """Information needed for the post-processing impulse computation."""

    def __init__(self, totalMass, r_PcP_P, logAttr, segment):
        self.totalMass = float(totalMass)
        self.r_PcP_P = np.array(r_PcP_P, dtype=np.float64).flatten()
        self.logAttr = logAttr
        self.segment = int(segment)


def setupExtFT():
    extFT = extForceTorque.ExtForceTorque()
    extFT.ModelTag = "extFT"
    extFT.extForce_B = [[10.0], [-5.0], [3.0]]  # [N]
    extFT.extForce_N = [[-4.0], [6.0], [2.0]]  # [N]
    extFT.extTorquePntB_B = [[2.0], [-1.0], [4.0]]  # [N m]
    return extFT


def setupSpinningBodyNDOF():
    """3-segment, 2-DOF-per-segment spinning body. Tip = body 6."""
    sbe = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    sbe.ModelTag = "spinningBodyNDOF"

    numberOfSegments = 3
    massSubPanel = 100.0 / numberOfSegments  # [kg]
    lengthSubPanel = 18.0 / numberOfSegments  # [m]
    widthSubPanel = 3.0  # [m]
    thicknessSubPanel = 0.3  # [m]

    for idx in range(numberOfSegments):
        sb = spinningBodyNDOFStateEffector.SpinningBody()
        sb.setMass(0.0)
        sb.setISPntSc_S([[0.0, 0.0, 0.0],
                         [0.0, 0.0, 0.0],
                         [0.0, 0.0, 0.0]])
        sb.setDCM_S0P([[1.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0],
                       [0.0, 0.0, 1.0]])
        sb.setR_ScS_S([[0.0], [lengthSubPanel / 2], [0.0]])  # [m]
        if idx == 0:
            sb.setR_SP_P([[0.0], [1.5], [1.5 - thicknessSubPanel / 2]])  # [m]
        else:
            sb.setR_SP_P([[0.0], [lengthSubPanel], [0.0]])  # [m]
        sb.setSHat_S([[1], [0], [0]])
        sb.setThetaInit(2.0 * macros.D2R)  # [rad]
        sb.setThetaDotInit(-0.5 * macros.D2R)  # [rad/s]
        sb.setK(10.0)  # [N m/rad]
        sb.setC(8.0)  # [N m s/rad]
        sbe.addSpinningBody(sb)

        sb = spinningBodyNDOFStateEffector.SpinningBody()
        sb.setMass(massSubPanel)
        sb.setISPntSc_S([[massSubPanel / 12 * (lengthSubPanel ** 2 + thicknessSubPanel ** 2), 0.0, 0.0],  # [kg m^2]
                         [0.0, massSubPanel / 12 * (widthSubPanel ** 2 + thicknessSubPanel ** 2), 0.0],
                         [0.0, 0.0, massSubPanel / 12 * (widthSubPanel ** 2 + lengthSubPanel ** 2)]])
        sb.setDCM_S0P([[1.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0],
                       [0.0, 0.0, 1.0]])
        sb.setR_ScS_S([[0.0], [lengthSubPanel / 2], [0.0]])  # [m]
        sb.setR_SP_P([[0.0], [0.0], [0.0]])
        sb.setSHat_S([[0], [1], [0]])
        sb.setThetaInit(2.0 * macros.D2R)  # [rad]
        sb.setThetaDotInit(-0.5 * macros.D2R)  # [rad/s]
        sb.setK(1.0)  # [N m/rad]
        sb.setC(0.8)  # [N m s/rad]
        sbe.addSpinningBody(sb)

    props = HostProps(
        totalMass=massSubPanel * numberOfSegments,
        r_PcP_P=[0.0, lengthSubPanel / 2, 0.0],
        logAttr="spinningBodyConfigLogOutMsgs",
        segment=6,
    )
    return sbe, props


def setupNHingedRigidBody():
    """Three identical hinged panels in a chain. Tip = panel 3."""
    nhb = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
    nhb.ModelTag = "nHingedRigidBody"

    numberOfPanels = 3
    panelMass = 100.0        # [kg]
    panelHalfLength = 0.75   # [m]
    thetaInit = (5.0, -2.0, 3.0)      # [deg]
    thetaDotInit = (-1.0, 0.0, 0.5)   # [deg/s]
    nhb.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    nhb.r_HB_B = [[0.5], [-1.5], [-0.5]]  # [m]
    for idx in range(numberOfPanels):
        panel = nHingedRigidBodyStateEffector.HingedPanel()
        panel.mass = panelMass
        panel.d = panelHalfLength
        panel.k = 100.0  # [N m/rad]
        panel.c = 0.0    # [N m s/rad]
        panel.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]  # [kg m^2]
        panel.thetaInit = thetaInit[idx] * macros.D2R
        panel.thetaDotInit = thetaDotInit[idx] * macros.D2R
        panel.theta_0 = 0.0
        nhb.addHingedPanel(panel)

    props = HostProps(
        totalMass=panelMass * numberOfPanels,
        r_PcP_P=[-panelHalfLength, 0.0, 0.0],   # panel CoM, -d along sHat1 from its hinge
        logAttr="nHingedRigidBodyConfigLogOutMsgs",
        segment=numberOfPanels,
    )
    return nhb, props


def setupLinearTranslationNDOF():
    """Three translating bodies in a chain. Tip = body 3."""
    ltn = linearTranslationNDOFStateEffector.LinearTranslationNDOFStateEffector()
    ltn.ModelTag = "linearTranslationNDOF"

    numberOfBodies = 3
    bodyMass = 20.0  # [kg]
    rhoInit = (1.0, 0.5, -0.25)       # [m]
    rhoDotInit = (0.05, -0.02, 0.03)  # [m/s]
    fHat_P = np.array([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    r_FcF_F = np.array([[-1.0], [1.0], [0.0]])  # [m]
    r_F0P_P = np.array([[-1.0], [1.0], [0.0]])  # [m]
    dcm_F0B = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    for idx in range(numberOfBodies):
        body = linearTranslationNDOFStateEffector.TranslatingBody()
        body.setMass(bodyMass)
        body.setK(100.0)  # [N/m]
        body.setC(0.0)    # [N s/m]
        body.setRhoInit(rhoInit[idx])
        body.setRhoDotInit(rhoDotInit[idx])
        body.setFHat_P(fHat_P)
        body.setR_FcF_F(r_FcF_F)
        body.setR_F0P_P(r_F0P_P)
        body.setIPntFc_F([[50.0, 0.0, 0.0], [0.0, 80.0, 0.0], [0.0, 0.0, 60.0]])  # [kg m^2]
        body.setDCM_FP(dcm_F0B if idx == 0 else np.eye(3))
        ltn.addTranslatingBody(body)

    props = HostProps(
        totalMass=bodyMass * numberOfBodies,
        r_PcP_P=r_FcF_F.flatten().tolist(),
        logAttr="translatingBodyConfigLogOutMsgs",
        segment=numberOfBodies,
    )
    return ltn, props


# -----------------------------------------------------------------------------
# Step-size refinement study
# -----------------------------------------------------------------------------

CONVERGENCE_DT_SWEEP = (0.05, 0.025, 0.0125, 0.00625, 0.003125)  # [s]
CONVERGENCE_FINAL_TIME = 10.0  # [s]
OBSERVABLES = ("sigma_BN", "omega_BN_B", "r_BN_N", "v_BN_N")

HOST_SETUPS = (
    ("spinningBodyNDOF",      setupSpinningBodyNDOF),
    ("nHingedRigidBody",      setupNHingedRigidBody),
    ("linearTranslationNDOF", setupLinearTranslationNDOF),
)

SETUP_SHORT = {
    "spinningBodyNDOF":      "spinningBodyNDOF",
    "nHingedRigidBody":      "nHingedRigidBody",
    "linearTranslationNDOF": "linearTranslationNDOF",
}


def runToFinalState(stateEffFactory, dt, T):
    """Run one field-free sim with a branched extForceTorque on the host's tip.

    Returns the hub state at ``t = T`` extracted from ``scStateOutMsg``.
    """
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)

    proc = sim.CreateNewProcess("proc")
    proc.addTask(sim.CreateNewTask("task", macros.sec2nano(dt)))

    sc = spacecraft.Spacecraft()
    sc.ModelTag = "scObject"
    sc.hub.mHub = 750.0  # [kg]
    sc.hub.IHubPntBc_B = [[900.0, 0.0, 0.0],  # [kg m^2]
                          [0.0, 800.0, 0.0],
                          [0.0, 0.0, 600.0]]
    sc.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    sc.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    sc.hub.sigma_BNInit = [[0.10], [0.05], [-0.08]]  # [-]
    sc.hub.omega_BN_BInit = [[0.05], [0.02], [-0.03]]  # [rad/s]

    stateEff, hostProps = stateEffFactory()
    extFT = setupExtFT()

    sc.addStateEffector(stateEff)
    stateEff.addDynamicEffector(extFT, hostProps.segment)

    sim.AddModelToTask("task", sc)
    sim.AddModelToTask("task", stateEff)
    sim.AddModelToTask("task", extFT)

    rec = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("task", rec)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(T))
    sim.ExecuteSimulation()

    return {
        "sigma_BN": np.array(rec.sigma_BN[-1], dtype=np.float64),
        "omega_BN_B": np.array(rec.omega_BN_B[-1], dtype=np.float64),
        "r_BN_N": np.array(rec.r_BN_N[-1], dtype=np.float64),
        "v_BN_N": np.array(rec.v_BN_N[-1], dtype=np.float64),
    }


# -----------------------------------------------------------------------------
# Convergence sweep
# -----------------------------------------------------------------------------


def runSweep(label, factory, dtSweep, finalTime):
    print(f"\n[{label}]")
    runs = []
    for dt in dtSweep:
        t0 = time.time()
        out = runToFinalState(factory, dt, finalTime)
        wall = time.time() - t0
        print(f"  dt={dt:.5g}  |omega(T)|={np.linalg.norm(out['omega_BN_B']):.6e}  "
              f"|sigma(T)|={np.linalg.norm(out['sigma_BN']):.6e}  ({wall:.2f}s)")
        runs.append(out)
    return runs


def pairResiduals(runs, key):
    """Return ‖x(dt_i) − x(dt_{i+1})‖ for each adjacent pair."""
    return np.array([np.linalg.norm(runs[i][key] - runs[i + 1][key])
                     for i in range(len(runs) - 1)])


def runConvergenceStudy(dtSweep=CONVERGENCE_DT_SWEEP, finalTime=CONVERGENCE_FINAL_TIME):
    """Run the step-size refinement study and render the convergence figure.

    Args:
        dtSweep (tuple): integration steps to refine over [s], halving between entries.
        finalTime (float): fixed final time at which the states are compared [s].

    Returns:
        tuple: the convergence figure and the fitted order per host and observable.
    """
    allRuns = {label: runSweep(label, factory, dtSweep, finalTime)
                for label, factory in HOST_SETUPS}
    pairDts = np.array(tuple(dtSweep)[:-1])

    # Print pair-residual table for every observable, fit slopes
    slopes = {}
    print("\n--- pair-wise self-convergence residuals at t = T ---")
    print(f"{'setup':>26s}  {'observable':>12s}  " +
          "  ".join(f"dt={d:.5g}" for d in pairDts) + "    slope")
    for label in allRuns:
        for obs in OBSERVABLES:
            res = pairResiduals(allRuns[label], obs)
            slope = np.polyfit(np.log(pairDts), np.log(res), 1)[0]
            row = "  ".join(f"{v:8.2e}" for v in res)
            print(f"{label:>26s}  {obs:>12s}  {row}    {slope:+.3f}")
            slopes.setdefault(label, {})[obs] = slope

    # twin axes: omega on the left, v on the right
    fig, axOmega = plt.subplots(figsize=(10.0, 6.0))
    axVel = axOmega.twinx()

    colors = {"spinningBodyNDOF": "C0",
              "nHingedRigidBody": "C1",
              "linearTranslationNDOF": "C2"}

    handles = []
    labels = []
    for label, runs in allRuns.items():
        color = colors[label]
        # omega: solid + circle, left axis
        resOmega = pairResiduals(runs, "omega_BN_B")
        slopeOmega = np.polyfit(np.log(pairDts), np.log(resOmega), 1)[0]
        hOmega, = axOmega.loglog(pairDts, resOmega, color=color, marker="o",
                           linestyle="-", markersize=9, linewidth=1.6)
        handles.append(hOmega)
        labels.append(rf"{SETUP_SHORT[label]}, $\omega$  ($p={slopeOmega:+.2f}$)")
        # v: dashed + square, right axis
        resVel = pairResiduals(runs, "v_BN_N")
        slopeVel = np.polyfit(np.log(pairDts), np.log(resVel), 1)[0]
        hVel, = axVel.loglog(pairDts, resVel, color=color, marker="s",
                           linestyle="--", markersize=8, linewidth=1.6)
        handles.append(hVel)
        labels.append(rf"{SETUP_SHORT[label]}, $v$  ($p={slopeVel:+.2f}$)")

    # slope-4 reference on the omega axis; both log axes share the x-axis
    anchor = max(pairResiduals(allRuns[lab], "omega_BN_B")[0]
                 for lab, _ in HOST_SETUPS) * 3.0
    ref = anchor * (pairDts / pairDts[0]) ** 4
    hRef, = axOmega.loglog(pairDts, ref, "k:", linewidth=1.5, alpha=0.8)
    handles.append(hRef)
    labels.append("slope = 4")

    axOmega.set_xlabel(r"integration step  $\Delta t$  [s]")
    axOmega.set_ylabel(r"$\|\,\omega_{B/N}^{(\Delta t)} - \omega_{B/N}^{(\Delta t/2)}\,\|$ "
                    r"at $t = T$   [rad/s]")
    axVel.set_ylabel(r"$\|\,v_{B/N}^{(\Delta t)} - v_{B/N}^{(\Delta t/2)}\,\|$ "
                    r"at $t = T$   [m/s]")
    axOmega.grid(True, which="both", linewidth=0.3, alpha=0.5)

    # co-align the two log axes
    yLo = min(pairResiduals(allRuns[lab], k)[-1]
               for lab, _ in HOST_SETUPS for k in ("omega_BN_B", "v_BN_N")) * 0.3
    yHi = anchor * 3.0
    axOmega.set_ylim(yLo, yHi)
    axVel.set_ylim(yLo, yHi)

    # legend in the empty top-left corner
    axOmega.legend(handles, labels,
                loc="upper left", ncol=2, framealpha=0.95,
                handlelength=2.2, columnspacing=1.0, fontsize=11)
    fig.subplots_adjust(left=0.13, right=0.88, top=0.96, bottom=0.13)

    return fig, slopes


# -----------------------------------------------------------------------------
# Impulse-momentum residual study
# -----------------------------------------------------------------------------

CONSERVATION_DT = 1.0e-4  # [s] fine enough that residuals sit at the float64 noise floor
CONSERVATION_FINAL_TIME = 4.0  # [s]

# hosts whose residuals are plotted: one cascading-rotation host and one translating host
CONSERVATION_PLOTTED = ("spinningBodyNDOF", "linearTranslationNDOF")

PLOT_STRIDE = 20    # plot every Nth sample to keep the SVG file size in check


def runWithFullLogs(hostFactory, dt, T):
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)

    proc = sim.CreateNewProcess("proc")
    proc.addTask(sim.CreateNewTask("task", macros.sec2nano(dt)))

    sc = spacecraft.Spacecraft()
    sc.ModelTag = "scObject"
    sc.hub.mHub = 750.0  # [kg]
    sc.hub.IHubPntBc_B = [[900.0, 0.0, 0.0],  # [kg m^2]
                          [0.0, 800.0, 0.0],
                          [0.0, 0.0, 600.0]]
    sc.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    sc.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    sc.hub.sigma_BNInit = [[0.10], [0.05], [-0.08]]  # [-]
    sc.hub.omega_BN_BInit = [[0.05], [0.02], [-0.03]]  # [rad/s]

    stateEff, props = hostFactory()
    extFT = setupExtFT()

    sc.addStateEffector(stateEff)
    stateEff.addDynamicEffector(extFT, props.segment)

    sim.AddModelToTask("task", sc)
    sim.AddModelToTask("task", stateEff)
    sim.AddModelToTask("task", extFT)

    datLog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("task", datLog)

    ipLog = getattr(stateEff, props.logAttr)[props.segment - 1].recorder()
    sim.AddModelToTask("task", ipLog)

    scLog = sc.logger(["totRotAngMomPntC_N"])
    sim.AddModelToTask("task", scLog)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(T))
    sim.ExecuteSimulation()

    return {
        "datLog": datLog,
        "ipLog": ipLog,
        "scLog": scLog,
        "extFT": extFT,
        "props": props,
        "hubMass": float(sc.hub.mHub),
        "dt": dt,
    }


# -----------------------------------------------------------------------------
# Conservation residuals (Simpson's rule, consistent indexing)
# -----------------------------------------------------------------------------


def computeResiduals(case):
    """Per-component signed residual time histories.

    Returns
    -------
    times : (N,) array of seconds
    resH : (N, 3) signed angular momentum residual,
            ``H_C(t) - H_C(0) - integral(tau_C dt')``  [N m s]
    resV : (N, 3) signed accumulated DV residual,
            ``v_C(t) - v_C(0) - integral(F/M dt')``  [m/s]
    """
    rotHTruth = np.asarray(case["scLog"].totRotAngMomPntC_N, dtype=np.float64)
    accumDVTruth = np.asarray(case["datLog"].TotalAccumDV_CN_N, dtype=np.float64)
    r_CN_N = np.asarray(case["datLog"].r_CN_N, dtype=np.float64)
    sigma_PN = np.asarray(case["ipLog"].sigma_BN, dtype=np.float64)
    r_PcN = np.asarray(case["ipLog"].r_BN_N, dtype=np.float64)

    F_B = np.asarray(case["extFT"].extForce_B, dtype=np.float64).flatten()
    F_ext_N = np.asarray(case["extFT"].extForce_N, dtype=np.float64).flatten()
    tau_B = np.asarray(case["extFT"].extTorquePntB_B, dtype=np.float64).flatten()
    r_PcP_P = case["props"].r_PcP_P
    mTotal = case["hubMass"] + case["props"].totalMass
    dt = case["dt"]

    n = sigma_PN.shape[0]
    assert r_CN_N.shape[0] == n, (
        f"sc log ({r_CN_N.shape[0]}) and ip log ({n}) lengths disagree")

    F_N = np.empty((n, 3), dtype=np.float64)
    tau_C = np.empty((n, 3), dtype=np.float64)
    for i in range(n):
        # MRP2C(sigma) returns [BN] (inertial-to-body); transpose for [NB] = [NP_j]
        dcm_NP = rbk.MRP2C(sigma_PN[i, :]).T
        F_N[i, :] = dcm_NP @ F_B + F_ext_N
        # Application point P_j = panel COM (Pc) - the offset r_PcP_P expressed in N
        r_P_N = r_PcN[i, :] - dcm_NP @ r_PcP_P
        momentArm = r_P_N - r_CN_N[i, :]
        tau_C[i, :] = dcm_NP @ tau_B + np.cross(momentArm, F_N[i, :])

    # Cumulative Simpson integral, O(dt^4)
    intF_N = cumulative_simpson(F_N, dx=dt, axis=0, initial=0.0)
    intTau_C = cumulative_simpson(tau_C, dx=dt, axis=0, initial=0.0)

    times = np.asarray(case["ipLog"].times(), dtype=np.float64) * macros.NANO2SEC

    # Truth at the same time grid
    deltaHTruth = rotHTruth - rotHTruth[0, :]
    deltaDVTruth = accumDVTruth - accumDVTruth[0, :]

    # Signed component residuals (Allard 2018 Fig. 8 convention)
    resH = deltaHTruth - intTau_C            # (N, 3)
    resV = deltaDVTruth - intF_N / mTotal   # (N, 3)
    return times, resH, resV


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------


def runConservationStudy(dt=CONSERVATION_DT, finalTime=CONSERVATION_FINAL_TIME):
    """Run the impulse-momentum residual study and render the conservation figure.

    Args:
        dt (float): fixed integration step [s].
        finalTime (float): simulation duration [s].

    Returns:
        tuple: the conservation figure and the peak angular and linear residual per host.
    """
    results = {}
    maxResiduals = {}
    for label, factory in HOST_SETUPS:
        t0 = time.time()
        case = runWithFullLogs(factory, dt, finalTime)
        times, resH, resV = computeResiduals(case)
        wall = time.time() - t0
        print(f"[{label}]  dt={dt:.5g}  T={finalTime}  ({wall:.2f}s)")
        print(f"    max ||H_C residual||  = {np.max(np.linalg.norm(resH, axis=1)):.3e}  N m s")
        print(f"    max ||Δv_C residual|| = {np.max(np.linalg.norm(resV, axis=1)):.3e}  m/s")
        results[label] = (times, resH, resV)
        maxResiduals[label] = {
            "angular": float(np.max(np.linalg.norm(resH, axis=1))),   # [N m s]
            "linear": float(np.max(np.linalg.norm(resV, axis=1))),    # [m/s]
        }

    setupShort = {
        "spinningBodyNDOF":      "spinningBodyNDOF",
        "nHingedRigidBody":      "nHingedRigidBody",
        "linearTranslationNDOF": "linearTranslationNDOF",
    }
    componentColors = ("C0", "C1", "C2")
    componentLabels = (r"$\hat{\mathbf{n}}_1$",
                        r"$\hat{\mathbf{n}}_2$",
                        r"$\hat{\mathbf{n}}_3$")

    fig, axes = plt.subplots(len(CONSERVATION_PLOTTED), 2, figsize=(12, 6.5), sharex=True)
    handles = None
    for row, label in enumerate(CONSERVATION_PLOTTED):
        t, rH, rV = results[label]
        axH = axes[row, 0]
        axV = axes[row, 1]
        # decimate for plotting only
        s = PLOT_STRIDE
        for k, (color, comp) in enumerate(zip(componentColors, componentLabels)):
            axH.plot(t[::s], rH[::s, k], color=color, linewidth=0.9, label=comp,
                     rasterized=True)
            axV.plot(t[::s], rV[::s, k], color=color, linewidth=0.9,
                     rasterized=True)
        axH.axhline(0.0, color="0.4", linewidth=0.5)
        axV.axhline(0.0, color="0.4", linewidth=0.5)

        axH.set_ylabel(r"angular residual  [N$\cdot$m$\cdot$s]")
        axV.set_ylabel(r"linear residual  [m/s]")
        axH.ticklabel_format(axis="y", style="sci", scilimits=(0, 0),
                             useMathText=True)
        axV.ticklabel_format(axis="y", style="sci", scilimits=(0, 0),
                             useMathText=True)
        for ax in (axH, axV):
            ax.grid(True, linewidth=0.3, alpha=0.5)
            ax.tick_params(direction="in", length=4)
            ax.set_xlim(0.0, finalTime)

        # Setup identifier as a small in-axes annotation, upper-right corner
        for ax in (axH, axV):
            ax.text(0.985, 0.95, setupShort[label],
                    transform=ax.transAxes, ha="right", va="top",
                    fontsize=12,
                    bbox=dict(boxstyle="round,pad=0.25",
                              facecolor="white", edgecolor="0.7",
                              alpha=0.9))

        if row == 0:
            handles = [axH.lines[k] for k in range(3)]

    for ax in axes[-1, :]:
        ax.set_xlabel(r"$t$ [s]")

    # Single component legend at the top of the figure
    fig.legend(handles, componentLabels,
               loc="upper center", bbox_to_anchor=(0.5, 1.0),
               ncol=3, framealpha=0.95, handlelength=2.0)
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    return fig, maxResiduals


def run(show_plots,
        dtSweep=CONVERGENCE_DT_SWEEP,
        convergenceFinalTime=CONVERGENCE_FINAL_TIME,
        conservationDt=CONSERVATION_DT,
        conservationFinalTime=CONSERVATION_FINAL_TIME):
    """Run both verification studies and return their figures and metrics.

    Args:
        show_plots (bool): display the figures interactively.
        dtSweep (tuple): integration steps to refine over [s], halving between entries.
        convergenceFinalTime (float): fixed final time for the refinement study [s].
        conservationDt (float): fixed integration step for the residual study [s].
        conservationFinalTime (float): duration of the residual study [s].

    Returns:
        tuple: ``figureList`` keyed by figure name, and a metrics dict holding the fitted
        convergence ``slopes`` and the peak impulse-momentum ``maxResiduals`` per host.
    """
    conservationFig, maxResiduals = runConservationStudy(conservationDt, conservationFinalTime)
    convergenceFig, slopes = runConvergenceStudy(dtSweep, convergenceFinalTime)

    figureList = {
        fileName + "Conservation": conservationFig,
        fileName + "Convergence": convergenceFig,
    }

    if show_plots:
        plt.show()
    plt.close("all")

    return figureList, {"slopes": slopes, "maxResiduals": maxResiduals}


if __name__ == "__main__":
    run(True)
