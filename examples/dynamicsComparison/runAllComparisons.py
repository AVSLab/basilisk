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
Convenience runner for the dynamics-engine comparison series.

Every ``scenarioCompare*.py`` script in ``basilisk/examples/dynamicsComparison``
exposes the same entry point: a ``run(showPlots, saveJson, ...)`` function that
propagates the classic back-substitution :ref:`spacecraft` and (when Basilisk is
built with ``--mujoco``) the MuJoCo-based :ref:`MJScene<MJScene>`, and returns a
mapping from figure name to matplotlib figure. This script discovers all of those
scenarios, runs each one with its **default arguments**, and persists the two
artifacts they produce:

#. the JSON summary of the comparison metrics, written by each scenario into the
   ``results`` folder next to this script by passing ``saveJson=True``; and
#. every figure in the returned ``figureList``, saved as a transparent ``.svg``
   both into that same ``results`` folder and into the documentation image folder
   ``basilisk/docs/source/_images/Scenarios`` (the latter via
   :ref:`unitTestSupport`, exactly as the scenario unit test does) so the docs
   build picks them up.

The comparison scenarios color their curves through
``unitTestSupport.getLineColor``. Before importing them, this runner monkey-patches
that helper with the Paul Tol *high-contrast* qualitative palette (blue, yellow,
red), which is colorblind-friendly and reproduces well in print. The patch is
installed first because the scenarios resolve their palette at import time.

Running the whole series is useful for regenerating the published metrics and
figures in one pass, or as a quick smoke test that each comparison still executes
end-to-end after a change to the shared dynamics or integrator code.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and
executed by using::

    python3 runAllComparisons.py

"""

import os
import sys
import importlib

import matplotlib.pyplot as plt

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
# their module-level colors (e.g. COLOR_CLASSIC) at import time.
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


def saveFigures(figureList):
    """Save every figure produced by a scenario as a transparent SVG.

    Each figure is written twice: once into the ``results`` folder, and once into
    the documentation image folder via ``unitTestSupport.saveScenarioFigure`` (the
    same path the scenario unit test uses) so the docs build can pick it up.

    Args:
        figureList (dict): mapping from figure name to matplotlib figure, as
            returned by a scenario's ``run`` function.
    """
    os.makedirs(resultsPath, exist_ok=True)
    for figureName, figure in figureList.items():
        figFileName = os.path.join(resultsPath, figureName + ".svg")
        print("\tSaving figure: " + figFileName)
        # Save into the results folder first; saveScenarioFigure closes the figure below.
        figure.savefig(figFileName, transparent=True)
        # Save into basilisk/docs/source/_images/Scenarios for the documentation.
        unitTestSupport.saveScenarioFigure(figureName, figure, thisFolder)


def run():
    """Run every comparison scenario with its defaults, saving JSON and figures.

    Each scenario is imported and executed with ``showPlots=False`` and
    ``saveJson=True`` so that the JSON summary is written into the ``results`` folder
    and the returned figures are saved into that same folder. Scenarios that return
    no figures (for example, when Basilisk is built without MuJoCo) are skipped after
    their JSON is written.
    """
    scenarios = discoverScenarios()
    print("Running " + str(len(scenarios)) + " dynamics-comparison scenarios:")

    for scenario in scenarios:
        print("Running scenario: " + scenario)
        module = importlib.import_module(scenario)
        figureList = module.run(showPlots=False, saveJson=True)

        if figureList is None:
            continue
        saveFigures(figureList)

        # Close the saved figures so they do not accumulate across scenarios.
        plt.close("all")

    print("Done. JSON summaries and figures written to " + resultsPath)


if __name__ == "__main__":
    run()
