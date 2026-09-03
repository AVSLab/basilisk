"""
scenarioBranchingConservation.py

Companion verification figure to ``scenarioBranchingConvergence.py``: the
single-:math:`\\Delta t` impulse-momentum-theorem residual for a branched
``extForceTorque`` effector. Together the two figures verify the branching
extension along orthogonal axes:

* ``scenarioBranchingConvergence`` shows that the simulator preserves the
  formal RK4 order (slope = 4 in step-size refinement). This rules out
  stage-inconsistency defects that would degrade the integrator order without
  necessarily violating any conservation law.
* This script shows that at one (sufficiently small) step size the simulator's
  internal angular and linear momentum about the system center of mass agree
  with the integrated external impulse to machine precision. This rules out
  EOM-level defects (sign errors, wrong reference frames in ``cTheta`` or
  ``vecRot``, wrong velocity source in ``facetDrag``) that conservation drift
  would expose.

Method
------
For a hub + branched ``extForceTorque`` system in field-free space the
impulse-momentum theorems read

.. math::

    \\boldsymbol{H}_C(t) - \\boldsymbol{H}_C(0)
        \\;=\\; \\int_0^t \\boldsymbol{\\tau}_C^{\\rm ext}(t')\\,dt' ,
    \\qquad
    M\\,\\bigl[\\boldsymbol{v}_C(t) - \\boldsymbol{v}_C(0)\\bigr]
        \\;=\\; \\int_0^t \\boldsymbol{F}^{\\rm ext}(t')\\,dt' ,

where :math:`M` is the system mass and the inertial-frame external wrench at
the application point :math:`P_j` (panel frame origin of the host segment) is

.. math::

    \\boldsymbol{F}^{\\rm ext}(t)
        \\;=\\; [\\mathcal{NP}_j(t)]\\,{}^{\\mathcal{P}_j}\\!\\boldsymbol{F} ,
    \\qquad
    \\boldsymbol{\\tau}_C^{\\rm ext}(t)
        \\;=\\; [\\mathcal{NP}_j(t)]\\,{}^{\\mathcal{P}_j}\\!\\boldsymbol{\\tau}
            + \\bigl(\\boldsymbol{r}_{P_j/N}(t) - \\boldsymbol{r}_{C/N}(t)\\bigr)
              \\times \\boldsymbol{F}^{\\rm ext}(t) .

The post-processing integrals are computed with Simpson's 1/3 rule (Scipy's
``cumulative_simpson``), giving :math:`O(\\Delta t^4)` quadrature error so
the residual is not bookkeeping-limited at the integrator's RK4 order. The
prior in-repo test compares against trapezoidal integration with mixed-index
inputs, which limits its residual to :math:`O(\\Delta t)`; that bookkeeping
issue is *not* a property of the simulator and is corrected here.

Usage
-----
::

    /path/to/basilisk/.venv/bin/python scenarioBranchingConservation.py
"""

import os
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import cumulative_simpson

from Basilisk.simulation import spacecraft
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, macros

from _branchingSetups import (
    setup_extFT,
    setup_linearTranslationOneDOF,
    setup_spinningBodyNDOF,
    setup_spinningBodyTwoDOF,
)


# -----------------------------------------------------------------------------
# Per-host bookkeeping shared with scenarioBranchingConvergence
# -----------------------------------------------------------------------------


# -----------------------------------------------------------------------------
# Single sim with full per-step logs
# -----------------------------------------------------------------------------

def run_one(host_factory, dt, T):
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)

    proc = sim.CreateNewProcess("proc")
    proc.addTask(sim.CreateNewTask("task", macros.sec2nano(dt)))

    sc = spacecraft.Spacecraft()
    sc.ModelTag = "scObject"
    sc.hub.mHub = 750.0
    sc.hub.IHubPntBc_B = [[900.0, 0.0, 0.0],
                          [0.0, 800.0, 0.0],
                          [0.0, 0.0, 600.0]]
    sc.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    sc.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    sc.hub.sigma_BNInit = [[0.10], [0.05], [-0.08]]
    sc.hub.omega_BN_BInit = [[0.05], [0.02], [-0.03]]

    state_eff, props = host_factory()
    extFT = setup_extFT()

    sc.addStateEffector(state_eff)
    state_eff.addDynamicEffector(extFT, props.segment)

    sim.AddModelToTask("task", sc)
    sim.AddModelToTask("task", state_eff)
    sim.AddModelToTask("task", extFT)

    datLog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("task", datLog)

    if props.segment == 1:
        ipLog = getattr(state_eff, props.log_attr).recorder()
    else:
        ipLog = getattr(state_eff, props.log_attr)[props.segment - 1].recorder()
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
        "hub_mass": float(sc.hub.mHub),
        "dt": dt,
    }


# -----------------------------------------------------------------------------
# Conservation residuals (Simpson's rule, consistent indexing)
# -----------------------------------------------------------------------------

def compute_residuals(case):
    """Per-component signed residual time histories.

    Returns
    -------
    times : (N,) array of seconds
    res_H : (N, 3) signed angular momentum residual,
            ``H_C(t) - H_C(0) - integral(tau_C dt')``  [N m s]
    res_V : (N, 3) signed accumulated DV residual,
            ``v_C(t) - v_C(0) - integral(F/M dt')``  [m/s]
    """
    rotH_truth = np.asarray(case["scLog"].totRotAngMomPntC_N, dtype=np.float64)
    accumDV_truth = np.asarray(case["datLog"].TotalAccumDV_CN_N, dtype=np.float64)
    r_CN_sc = np.asarray(case["datLog"].r_CN_N, dtype=np.float64)
    sigma_PN = np.asarray(case["ipLog"].sigma_BN, dtype=np.float64)
    r_PcN = np.asarray(case["ipLog"].r_BN_N, dtype=np.float64)

    F_B = np.asarray(case["extFT"].extForce_B, dtype=np.float64).flatten()
    tau_B = np.asarray(case["extFT"].extTorquePntB_B, dtype=np.float64).flatten()
    r_PcP_P = case["props"].r_PcP_P
    M_total = case["hub_mass"] + case["props"].total_mass
    dt = case["dt"]

    n = sigma_PN.shape[0]
    assert r_CN_sc.shape[0] == n, (
        f"sc log ({r_CN_sc.shape[0]}) and ip log ({n}) lengths disagree")

    F_N = np.empty((n, 3), dtype=np.float64)
    tau_C = np.empty((n, 3), dtype=np.float64)
    for i in range(n):
        # MRP2C(sigma) returns [BN] (inertial-to-body); transpose for [NB] = [NP_j]
        dcm_NP = rbk.MRP2C(sigma_PN[i, :]).T
        F_N[i, :] = dcm_NP @ F_B
        # Application point P_j = panel COM (Pc) - the offset r_PcP_P expressed in N
        r_P_N = r_PcN[i, :] - dcm_NP @ r_PcP_P
        moment_arm = r_P_N - r_CN_sc[i, :]
        tau_C[i, :] = dcm_NP @ tau_B + np.cross(moment_arm, F_N[i, :])

    # Cumulative Simpson integral, O(dt^4)
    int_F_N = cumulative_simpson(F_N, dx=dt, axis=0, initial=0.0)
    int_tau_C = cumulative_simpson(tau_C, dx=dt, axis=0, initial=0.0)

    times = np.asarray(case["ipLog"].times(), dtype=np.float64) * macros.NANO2SEC

    # Truth at the same time grid
    delta_H_truth = rotH_truth - rotH_truth[0, :]
    delta_DV_truth = accumDV_truth - accumDV_truth[0, :]

    # Signed component residuals (Allard 2018 Fig. 8 convention)
    res_H = delta_H_truth - int_tau_C            # (N, 3)
    res_V = delta_DV_truth - int_F_N / M_total   # (N, 3)
    return times, res_H, res_V


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

DT = 1.0e-4    # fine enough that residuals sit at the float64 noise floor,
T_FINAL = 4.0  # giving the random-walk character of pure rounding accumulation

SETUPS = (
    # one cascading-rotation example and one translation example, dropping
    # the two-DOF spinning body since it tells the same story as NDOF
    ("spinningBodyNDOF",        setup_spinningBodyNDOF),
    ("linearTranslationOneDOF", setup_linearTranslationOneDOF),
)

PLOT_STRIDE = 20    # plot every Nth sample to keep the SVG file size in check


def run(showPlots=False, dt=DT, finalTime=T_FINAL, resultsDir=None):
    """Run the impulse-momentum residual study and render the conservation figure.

    Args:
        showPlots (bool): display the figure interactively.
        dt (float): fixed integration step [s].
        finalTime (float): simulation duration [s].
        resultsDir (str, optional): directory to write the figure into. Defaults to this
            script's own directory.

    Returns:
        dict: ``maxResiduals`` (peak angular and linear residual per host) and ``figureList``.
    """
    plt.rc("font", family="serif", size=14)
    plt.rc("axes", labelsize=14, titlesize=14)
    plt.rc("xtick", labelsize=11)
    plt.rc("ytick", labelsize=11)
    plt.rc("legend", fontsize=11)

    results = {}
    maxResiduals = {}
    for label, factory in SETUPS:
        t0 = time.time()
        case = run_one(factory, dt, finalTime)
        times, res_H, res_V = compute_residuals(case)
        wall = time.time() - t0
        print(f"[{label}]  dt={dt:.5g}  T={finalTime}  ({wall:.2f}s)")
        print(f"    max ||H_C residual||  = {np.max(np.linalg.norm(res_H, axis=1)):.3e}  N m s")
        print(f"    max ||Δv_C residual|| = {np.max(np.linalg.norm(res_V, axis=1)):.3e}  m/s")
        results[label] = (times, res_H, res_V)
        maxResiduals[label] = {
            "angular": float(np.max(np.linalg.norm(res_H, axis=1))),   # [N m s]
            "linear": float(np.max(np.linalg.norm(res_V, axis=1))),    # [m/s]
        }

    setup_short = {
        "spinningBodyNDOF":        "spinningBodyNDOF",
        "spinningBodyTwoDOF":      "spinningBodyTwoDOF",
        "linearTranslationOneDOF": "linearTranslationOneDOF",
    }
    component_colors = ("C3", "C2", "C0")    # n_1 red, n_2 green, n_3 blue
    component_labels = (r"$\hat{\mathbf{n}}_1$",
                        r"$\hat{\mathbf{n}}_2$",
                        r"$\hat{\mathbf{n}}_3$")

    fig, axes = plt.subplots(len(SETUPS), 2, figsize=(12, 6.5), sharex=True)
    handles = None
    for row, (label, (t, rH, rV)) in enumerate(results.items()):
        axH = axes[row, 0]
        axV = axes[row, 1]
        # Decimate for plotting; full-resolution data is still used in the
        # residual computation upstream.
        s = PLOT_STRIDE
        for k, (color, comp) in enumerate(zip(component_colors, component_labels)):
            axH.plot(t[::s], rH[::s, k], color=color, linewidth=0.9, label=comp,
                     rasterized=True)
            axV.plot(t[::s], rV[::s, k], color=color, linewidth=0.9,
                     rasterized=True)
        axH.axhline(0.0, color="0.4", linewidth=0.5)
        axV.axhline(0.0, color="0.4", linewidth=0.5)

        # Y-axis: short text label; the residual definition lives in the
        # manuscript caption and equations rather than the label itself
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
            ax.text(0.985, 0.95, setup_short[label],
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
    fig.legend(handles, component_labels,
               loc="upper center", bbox_to_anchor=(0.5, 1.0),
               ncol=3, framealpha=0.95, handlelength=2.0)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    if resultsDir is None:
        resultsDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    os.makedirs(resultsDir, exist_ok=True)
    out = os.path.join(resultsDir, "scenarioBranchingConservation.svg")
    fig.savefig(out, bbox_inches="tight")
    print(f"\nfigure saved: {out}")

    if showPlots:
        plt.show()
    plt.close("all")

    return {"maxResiduals": maxResiduals,
            "figureList": {"scenarioBranchingConservation": fig}}


if __name__ == "__main__":
    run(showPlots=False)
