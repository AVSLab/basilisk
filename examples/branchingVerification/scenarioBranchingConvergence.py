"""
scenarioBranchingConvergence.py

Self-convergence (Richardson) study for the Basilisk effector-branching
extension. Demonstrates that the simulator preserves the integrator's expected
order of accuracy when an external dynamic effector is attached to a state
effector segment instead of to the hub.

Method
------
The metric is the simulation's own answer at successively halved integration
step sizes. For an order-:math:`p` integrator the local truncation error of
any state quantity :math:`x` at fixed final time :math:`T` is
:math:`x(\\Delta t) - x_{\\rm true} = C\\,\\Delta t^p + \\mathcal{O}(\\Delta t^{p+1})`,
so the pair-wise difference between successive refinements satisfies

.. math::

    \\| x(\\Delta t) - x(\\Delta t/2) \\| \\;=\\; C\\,(1 - 2^{-p})\\,\\Delta t^p
    \\;+\\; \\mathcal{O}(\\Delta t^{p+1}) ,

which appears as a line of slope :math:`p` on log-log axes. Spacecraft uses
``svIntegratorRK4`` by default, so the expected slope is 4.

Why this exercises branching
----------------------------
The branched dynamic effector contributes to the hub equations of motion only
through the host state effector's BSM coupling matrices and through the
``cTheta`` and ``vecRot`` terms in the host. The hub state at fixed :math:`T`
is therefore a downstream witness of all of those contributions: if any of
them is mis-specified the simulator integrates the wrong equations, but the
self-convergence slope on the (still consistent) discrete scheme is the
quantity verified here. The companion impulse-momentum-theorem residual at a
single fixed :math:`\\Delta t` (in
``test_effectorBranching_integrated``) verifies that the integrated equations
are also the correct physics.

Why field-free space
--------------------
Putting the spacecraft in orbit at LEO velocities produces a 5-7 km/s frame
drift between consecutive logged samples. Any single-step indexing lag in a
manual moment-arm cross product then injects spurious torques of order
:math:`\\|v_C\\| \\|F\\| \\Delta t` into the bookkeeping, swamping integration
error and producing a misleading slope-1 floor. The branching mechanism does
not depend on gravity, so the convergence study is run in field-free space
with zero initial linear state.

Usage
-----
::

    /path/to/basilisk/.venv/bin/python scenarioBranchingConvergence.py
"""

import os
import time

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass, macros

from _branchingSetups import (
    setup_extFT,
    setup_linearTranslationOneDOF,
    setup_spinningBodyNDOF,
    setup_spinningBodyTwoDOF,
)


# -----------------------------------------------------------------------------
# Parent state effector setups
# -----------------------------------------------------------------------------


# -----------------------------------------------------------------------------
# Single-shot simulation
# -----------------------------------------------------------------------------

def run_one(state_eff_factory, dt, T):
    """Run one field-free sim with a branched extForceTorque on the host's tip.

    Returns the hub state at ``t = T`` extracted from ``scStateOutMsg``.
    """
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

    state_eff, host_props = state_eff_factory()
    ext_ft = setup_extFT()

    sc.addStateEffector(state_eff)
    state_eff.addDynamicEffector(ext_ft, host_props.segment)

    sim.AddModelToTask("task", sc)
    sim.AddModelToTask("task", state_eff)
    sim.AddModelToTask("task", ext_ft)

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

DT_SWEEP = (0.05, 0.025, 0.0125, 0.00625, 0.003125)
FINAL_TIME = 10.0
OBSERVABLES = ("sigma_BN", "omega_BN_B", "r_BN_N", "v_BN_N")

SETUPS = (
    ("spinningBodyNDOF",        setup_spinningBodyNDOF),
    ("spinningBodyTwoDOF",      setup_spinningBodyTwoDOF),
    ("linearTranslationOneDOF", setup_linearTranslationOneDOF),
)

SETUP_SHORT = {
    "spinningBodyNDOF":        "spinningBodyNDOF",
    "spinningBodyTwoDOF":      "spinningBodyTwoDOF",
    "linearTranslationOneDOF": "linearTranslationOneDOF",
}


def run_sweep(label, factory, dtSweep, finalTime):
    print(f"\n[{label}]")
    runs = []
    for dt in dtSweep:
        t0 = time.time()
        out = run_one(factory, dt, finalTime)
        wall = time.time() - t0
        print(f"  dt={dt:.5g}  |omega(T)|={np.linalg.norm(out['omega_BN_B']):.6e}  "
              f"|sigma(T)|={np.linalg.norm(out['sigma_BN']):.6e}  ({wall:.2f}s)")
        runs.append(out)
    return runs


def pair_residuals(runs, key):
    """Return ‖x(dt_i) − x(dt_{i+1})‖ for each adjacent pair."""
    return np.array([np.linalg.norm(runs[i][key] - runs[i + 1][key])
                     for i in range(len(runs) - 1)])


def run(showPlots=False, dtSweep=DT_SWEEP, finalTime=FINAL_TIME, resultsDir=None):
    """Run the step-size refinement study and render the convergence figure.

    Args:
        showPlots (bool): display the figure interactively.
        dtSweep (tuple): integration steps to refine over [s], halving between entries.
        finalTime (float): fixed final time at which the states are compared [s].
        resultsDir (str, optional): directory to write the figure into. Defaults to this
            script's own directory.

    Returns:
        dict: ``slopes`` (fitted order per host and observable) and ``figureList``.
    """
    plt.rc("font", family="serif", size=15)
    plt.rc("axes", labelsize=17)
    plt.rc("xtick", labelsize=13)
    plt.rc("ytick", labelsize=13)
    plt.rc("legend", fontsize=12)

    all_runs = {label: run_sweep(label, factory, dtSweep, finalTime)
                for label, factory in SETUPS}
    pair_dts = np.array(tuple(dtSweep)[:-1])

    # Print pair-residual table for every observable, fit slopes
    slopes = {}
    print("\n--- pair-wise self-convergence residuals at t = T ---")
    print(f"{'setup':>26s}  {'observable':>12s}  " +
          "  ".join(f"dt={d:.5g}" for d in pair_dts) + "    slope")
    for label in all_runs:
        for obs in OBSERVABLES:
            res = pair_residuals(all_runs[label], obs)
            slope = np.polyfit(np.log(pair_dts), np.log(res), 1)[0]
            row = "  ".join(f"{v:8.2e}" for v in res)
            print(f"{label:>26s}  {obs:>12s}  {row}    {slope:+.3f}")
            slopes.setdefault(label, {})[obs] = slope

    # Twin-axis convergence figure: omega_B/N on the left, v_B/N on the right.
    # The plotted quantity is the norm of the difference between two
    # simulations whose only configuration difference is the integration step
    # size. As dt -> 0 both runs converge to the simulator's true continuous
    # solution, so this difference goes to zero at the integrator's order.
    fig, ax_w = plt.subplots(figsize=(10.0, 6.0))
    ax_v = ax_w.twinx()

    colors = {"spinningBodyNDOF": "C0",
              "spinningBodyTwoDOF": "C1",
              "linearTranslationOneDOF": "C2"}

    handles = []
    labels = []
    for label, runs in all_runs.items():
        color = colors[label]
        # omega: solid + circle, left axis
        res_w = pair_residuals(runs, "omega_BN_B")
        slope_w = np.polyfit(np.log(pair_dts), np.log(res_w), 1)[0]
        h_w, = ax_w.loglog(pair_dts, res_w, color=color, marker="o",
                           linestyle="-", markersize=9, linewidth=1.6)
        handles.append(h_w)
        labels.append(rf"{SETUP_SHORT[label]}, $\omega$  ($p={slope_w:+.2f}$)")
        # v: dashed + square, right axis
        res_v = pair_residuals(runs, "v_BN_N")
        slope_v = np.polyfit(np.log(pair_dts), np.log(res_v), 1)[0]
        h_v, = ax_v.loglog(pair_dts, res_v, color=color, marker="s",
                           linestyle="--", markersize=8, linewidth=1.6)
        handles.append(h_v)
        labels.append(rf"{SETUP_SHORT[label]}, $v$  ($p={slope_v:+.2f}$)")

    # Reference slope-4 line drawn on the omega axis (the slope is identical
    # on both axes since they share the same x-axis and both are log-scale).
    anchor = max(pair_residuals(all_runs[lab], "omega_BN_B")[0]
                 for lab, _ in SETUPS) * 3.0
    ref = anchor * (pair_dts / pair_dts[0]) ** 4
    h_ref, = ax_w.loglog(pair_dts, ref, "k:", linewidth=1.5, alpha=0.8)
    handles.append(h_ref)
    labels.append("slope = 4")

    ax_w.set_xlabel(r"integration step  $\Delta t$  [s]")
    ax_w.set_ylabel(r"$\|\,\omega_{B/N}^{(\Delta t)} - \omega_{B/N}^{(\Delta t/2)}\,\|$ "
                    r"at $t = T$   [rad/s]")
    ax_v.set_ylabel(r"$\|\,v_{B/N}^{(\Delta t)} - v_{B/N}^{(\Delta t/2)}\,\|$ "
                    r"at $t = T$   [m/s]")
    ax_w.grid(True, which="both", linewidth=0.3, alpha=0.5)

    # Co-align the two log axes so the slope-4 reference reads cleanly
    # against both data clusters.
    y_lo = min(pair_residuals(all_runs[lab], k)[-1]
               for lab, _ in SETUPS for k in ("omega_BN_B", "v_BN_N")) * 0.3
    y_hi = anchor * 3.0
    ax_w.set_ylim(y_lo, y_hi)
    ax_v.set_ylim(y_lo, y_hi)

    # Legend in the top-left corner of the plot (the data slopes up-and-right
    # so this region is empty). Two columns to keep the box compact.
    ax_w.legend(handles, labels,
                loc="upper left", ncol=2, framealpha=0.95,
                handlelength=2.2, columnspacing=1.0, fontsize=11)
    fig.subplots_adjust(left=0.13, right=0.88, top=0.96, bottom=0.13)

    if resultsDir is None:
        resultsDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    os.makedirs(resultsDir, exist_ok=True)
    out_path = os.path.join(resultsDir, "scenarioBranchingConvergence.svg")
    fig.savefig(out_path, bbox_inches="tight")
    print(f"\nfigure saved: {out_path}")

    if showPlots:
        plt.show()
    plt.close("all")

    return {"slopes": slopes, "figureList": {"scenarioBranchingConvergence": fig}}


if __name__ == "__main__":
    run(showPlots=False)
