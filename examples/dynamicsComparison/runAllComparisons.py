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
Dynamics-engine comparison overview and execution guide.

Every ``scenarioCompare*.py`` script in ``basilisk/examples/dynamicsComparison``
exposes a ``run(showPlots, saveJson, ...)`` function returning a mapping from figure
name to matplotlib figure. This script discovers all of them, runs each with its
default arguments, and persists their artifacts:

#. the JSON summary of the comparison metrics (``saveJson=True``), written to the
   ``results`` folder next to this script;
#. every figure in the returned ``figureList``, saved as a transparent ``.svg`` both
   to ``results`` and to ``basilisk/docs/source/_images/Scenarios`` (via
   :ref:`simHelpers`) so the docs build picks them up; and
#. for the accuracy-comparison scenarios, an optional local BSM-vs-MJScene
   runtime ``.csv`` (``saveTiming=True``).

Before importing the scenarios, this runner replaces ``unitTestSupport.getLineColor``
with the Paul Tol high-contrast palette (colorblind-friendly, prints well). The patch
must precede the imports because the scenarios resolve their palette at import time.

Required build features
-----------------------

The cross-engine comparisons require a Basilisk build configured with MuJoCo. Vizard
and optical-navigation support are not required. Use a Release build for timing
measurements; debug builds and machines under changing thermal or power-management
conditions do not provide comparable performance data. The modules remain importable
without MuJoCo so documentation discovery can still run, but cross-engine execution
and the MuJoCo-specific sweeps then raise a clear ``ImportError`` or omit MuJoCo data.

Study types
-----------

The correctness scenarios progress from analytic rigid-body anchors through reaction
wheels, flexible appendages, differential gravity, fuel slosh, and mass depletion.
They report state agreement and conserved or analytic quantities. The two Pareto
scenarios and the flexible-panel dimensionality study are benchmarks: they compare
error with propagation-only wall time after warm-up. The six ``sweep*.py`` drivers
are research controls for step size, velocity-dependent round-off, reaction-wheel
bookkeeping mass, slosh displacement, torque-artifact scaling, and pendulum-bob
inertia. Benchmark timings are descriptive measurements, not pass/fail guarantees.

Execution order and outputs
---------------------------

Run these commands from the repository root after building Basilisk:

.. code-block:: console

    python3 examples/dynamicsComparison/runAllComparisons.py
    python3 examples/dynamicsComparison/sweepOrbitDt.py
    python3 examples/dynamicsComparison/sweepTorqueArtifact.py
    python3 examples/dynamicsComparison/sweepTorqueMechanismChecks.py
    python3 examples/dynamicsComparison/sweepRwBookkeepingMass.py
    python3 examples/dynamicsComparison/sweepSloshDisplacement.py
    python3 examples/dynamicsComparison/sweepVariableMassPendulumInertia.py
    python3 examples/dynamicsComparison/paperFigures.py --update-provenance

The documentation workflow instead invokes ``runAllComparisons.py
--documentation-figures``. That option uses explicitly reduced dimensionality and
Pareto profiles to bound CI time. The affected scenario pages label those generated
figures and state which conclusions require the complete default runs; the reduced
figures are workflow demonstrations, not publication benchmark evidence.

The runner writes scenario JSON and SVG files to ``results/``, copies documentation
figures to ``docs/source/_images/Scenarios``, and may write optional local runtime
CSV tables. Each sweep writes its own JSON and SVG artifacts. ``paperFigures.py``
reruns every scenario and sweep used by a paper figure, table, or quantitative claim,
writes print PDFs to ``results/paper``, and replaces ``results/provenance.json`` with
source, native-build, dependency, host, input, and output hashes. Run the provenance
update only from a clean worktree whose Release build identifies the current commit.
A later render validates those hashes before accepting the figures.

On a 2026 Apple M4 Pro Release build, the full scenario runner takes roughly five
minutes, the documentation-only timing pass takes two to three minutes, and the
standard sweeps finish in about one minute combined. The direct torque-mechanism
control is intentionally longer at about ten minutes. Paper rendering takes less
than a minute. These values are planning estimates only and vary with hardware,
compiler, background load, and thermal state.

Interpretation limits
---------------------

Agreement demonstrates equivalence only for the modeled force, mass-property, joint,
gravity, and integration conventions documented by each scenario. The engines do not
share every gravity or multibody approximation, so some extended cases intentionally
retain a physical residual. Timing ratios apply to the listed model sizes, integrators,
tolerances, and host; they must not be generalized into an engine-wide performance
claim. Inspect the scenario-specific assumptions and the stored configuration metadata
before reusing any result.

"""

import argparse
import os
import sys
import importlib
import tempfile

import matplotlib.pyplot as plt

from Basilisk import hasBuildFeature
from Basilisk.utilities import simHelpers
from Basilisk.utilities import unitTestSupport

thisFolder = os.path.dirname(os.path.abspath(__file__))

# All JSON summaries and figures for this comparison series are collected here,
# matching the ``resultsPath`` the scenarios use for their JSON output.
resultsPath = os.path.join(thisFolder, "results")

# Make the sibling scenario modules importable when this script is run directly.
if thisFolder not in sys.path:
    sys.path.append(thisFolder)

# Paul Tol high-contrast qualitative palette (colorblind-friendly).
BLUE = "#004488"
YELLOW = "#DDAA33"
RED = "#BB5566"
HIGH_CONTRAST = (BLUE, YELLOW, RED)


def requireMujoco():
    """Require the build feature needed for complete comparison figures."""
    if not hasBuildFeature("mujoco"):
        raise RuntimeError(
            "Dynamics-comparison figures require a MuJoCo-enabled Basilisk "
            "build; rebuild with `python conanfile.py --mujoco True`."
        )


def highContrastLineColor(idx, maxNum):
    """Drop-in replacement for ``unitTestSupport.getLineColor``.

    Returns a color from the Paul Tol high-contrast palette, cycling through it so
    that any index requested by the scenarios maps to a valid color.

    Args:
        idx (int): index of the curve to color.
        maxNum (int): number of curves the caller is coloring (unused, kept for
            signature compatibility with ``unitTestSupport.getLineColor``).

    Returns:
        str: a hex color string from the high-contrast palette.
    """
    return HIGH_CONTRAST[idx % len(HIGH_CONTRAST)]


# Install the palette before the scenarios are imported: several of them resolve
# their module-level colors (e.g. COLOR_BSM) at import time.
unitTestSupport.getLineColor = highContrastLineColor


def discoverScenarios():
    """Return the module names of every comparison scenario in this folder.

    Returns:
        list: sorted module names (without the ``.py`` extension) of the
        ``scenarioCompare*.py`` scripts living next to this runner.
    """
    return sorted(
        scenarioFile[:-3]
        for scenarioFile in os.listdir(thisFolder)
        if scenarioFile.startswith("scenario") and scenarioFile.endswith(".py")
    )


def saveFigures(figureList, resultsDir=None, saveDocumentationFigures=True):
    """Save every figure produced by a scenario as a transparent SVG.

    Each figure is written twice: once into the ``results`` folder, and once into
    the documentation image folder via ``simHelpers.saveScenarioFigure`` (the
    same path the scenario unit test uses) so the docs build can pick it up.

    Args:
        figureList (dict): mapping from figure name to matplotlib figure, as
            returned by a scenario's ``run`` function.
        resultsDir (str, optional): explicit artifact directory. Defaults to
            the comparison ``results`` folder.
        saveDocumentationFigures (bool, optional): if True, also copy each
            figure into the Sphinx scenario-image directory. Defaults to True.
    """
    targetResults = resultsPath if resultsDir is None else resultsDir
    os.makedirs(targetResults, exist_ok=True)
    for figureName, figure in figureList.items():
        figFileName = os.path.join(targetResults, figureName + ".svg")
        print("\tSaving figure: " + figFileName)
        figure.savefig(figFileName, transparent=True)
        if saveDocumentationFigures:
            simHelpers.saveScenarioFigure(figureName, figure, thisFolder)


# Accuracy scenarios that report a runtime table via ``saveTiming``. FlexPanels and the
# two Pareto studies visualize wall-clock directly and do not take the flag.
TIMING_SCENARIOS = (
    "scenarioCompareOrbit",
    "scenarioCompareTorque",
    "scenarioCompareRwPanels",
    "scenarioCompareOrbitMultibody",
    "scenarioCompareVariableMass",
)

# Reduced only where the default study is intentionally expensive. This profile
# generates every image referenced by the Sphinx pages without writing publication
# inputs or timing tables.
DOCUMENTATION_RUN_KWARGS = {
    "scenarioCompareFlexPanels": {"nSegmentsList": (1, 2, 4, 8)},
    "scenarioCompareParetoRwPanels": {
        "sweepConfigs": [
            ("svIntegratorRK4", 0.1, None),
            ("svIntegratorEuler", 0.02, None),
            ("svIntegratorRKF45", 0.5, 1.0e-7),
        ],
        "reference": {
            "integrator": "svIntegratorRKF45",
            "dt": 0.05,
            "tol": 1.0e-9,
        },
        "referenceCheck": {
            "integrator": "svIntegratorRKF45",
            "dt": 0.025,
            "tol": 1.0e-9,
        },
    },
    "scenarioCompareParetoFlexPanels": {
        "sweepConfigs": [
            ("svIntegratorRKF45", 0.1, 1.0e-4),
            ("svIntegratorRKF45", 0.1, 1.0e-6),
        ],
        "simDuration": 1.0,
        "reference": {
            "integrator": "svIntegratorRKF45",
            "dt": 0.2,
            "tol": 1.0e-6,
        },
    },
}


def run(timingOnly=False, scenarios=None, scenarioRunKwargs=None,
        resultsDir=None, saveDocumentationFigures=True, saveJson=None,
        saveTiming=None):
    """Run every comparison scenario with its defaults, saving JSON, figures, and tables.

    Each scenario runs with ``showPlots=False`` and ``saveJson=True``; the accuracy
    scenarios additionally run with ``saveTiming=True`` to regenerate their runtime
    table. Scenarios returning no figures (e.g. built without MuJoCo) are skipped after
    their JSON is written.

    Args:
        timingOnly (bool, optional): if True, run only the scenarios that publish a
            local runtime table, and skip their JSON summaries and figures.
        scenarios (sequence, optional): explicit module names to run. Defaults to the
            timing scenarios when ``timingOnly`` is True and discovered scenarios otherwise.
        scenarioRunKwargs (dict, optional): per-module keyword overrides passed to ``run``.
        resultsDir (str, optional): explicit artifact directory. Defaults to
            the comparison ``results`` folder.
        saveDocumentationFigures (bool, optional): if True, copy generated
            figures into the Sphinx scenario-image directory. Defaults to True.
        saveJson (bool, optional): override JSON generation for every scenario.
            Defaults to True except in timing-only mode.
        saveTiming (bool, optional): override runtime-table generation for every
            timing scenario. Defaults to True.

    Returns:
        tuple: module names completed in execution order.
    """
    selectedScenarios = (
        tuple(scenarios)
        if scenarios is not None
        else TIMING_SCENARIOS if timingOnly else tuple(discoverScenarios())
    )
    runOverrides = {} if scenarioRunKwargs is None else scenarioRunKwargs
    targetResults = resultsPath if resultsDir is None else resultsDir
    print("Running " + str(len(selectedScenarios)) + " dynamics-comparison scenarios:")

    for scenario in selectedScenarios:
        print("Running scenario: " + scenario)
        module = importlib.import_module(scenario)
        kwargs = dict(runOverrides.get(scenario, {}))
        kwargs.setdefault("showPlots", False)
        kwargs.setdefault(
            "saveJson", not timingOnly if saveJson is None else saveJson)
        kwargs.setdefault("resultsDir", targetResults)
        if scenario in TIMING_SCENARIOS:
            kwargs.setdefault(
                "saveTiming", True if saveTiming is None else saveTiming)
        figureList = module.run(**kwargs)

        if timingOnly or figureList is None:
            continue
        saveFigures(
            figureList,
            resultsDir=targetResults,
            saveDocumentationFigures=saveDocumentationFigures,
        )

        # Close the saved figures so they do not accumulate across scenarios.
        plt.close("all")

    print("Done. Results written to " + targetResults)
    return selectedScenarios


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the dynamics-engine comparisons.")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--timing-only", action="store_true",
                      help="regenerate only the optional local runtime tables")
    mode.add_argument(
        "--documentation-figures",
        action="store_true",
        help="regenerate the documented scenario images with the reduced CI profile",
    )
    args = parser.parse_args()
    if args.documentation_figures:
        requireMujoco()
        with tempfile.TemporaryDirectory(
                prefix="bsk-dynamics-comparison-docs-") as resultsDirectory:
            run(
                scenarioRunKwargs=DOCUMENTATION_RUN_KWARGS,
                resultsDir=resultsDirectory,
                saveDocumentationFigures=True,
                saveJson=False,
                saveTiming=False,
            )
    else:
        run(timingOnly=args.timing_only)
