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
Shared runtime-timing helper for the dynamics-engine comparison series.

Lets the accuracy-comparison scenarios (orbit, torque, reaction-wheel-and-panel,
multi-body-in-orbit) time the back-substitution method (BSM, the :ref:`spacecraft` module)
against MuJoCo (the :ref:`MJScene<MJScene>` module) on the same problem and persist a small
CSV table.

The table is written to ``<scenarioFileDir>/results/<scenarioName>_runtime.csv`` and embedded
in the scenario docstring with a ``.. csv-table:: :file: results/<scenarioName>_runtime.csv``
directive, so the published numbers are regenerated whenever the scenario (or its unit test)
runs. The path is module-relative because Sphinx ``autodoc`` resolves the ``csv-table``
``:file:`` option relative to the module that owns the docstring.

Timing methodology
------------------
Each configuration is run ``repeats`` times and the minimum wall-clock time is kept
(best-of-N). The dedicated timing scenarios (:ref:`scenarioCompareFlexPanels`,
:ref:`scenarioCompareParetoRwPanels`) instead report the median over warmed trials.
Absolute times are machine-dependent; the speedup ratio (BSM / MuJoCo) is the portable
figure and is reported alongside.
"""

import csv
import io
import os
import time

# Default number of timed repeats; the reported time is the minimum over the repeats.
DEFAULT_TIMING_REPEATS = 5


def bestOfN(propagate, repeats=DEFAULT_TIMING_REPEATS):
    """Return the minimum wall-clock time over ``repeats`` runs of ``propagate`` [s].

    Args:
        propagate (callable): a zero-argument callable that builds and runs one full
            propagation. It is called ``repeats`` times; its return value is ignored.
        repeats (int, optional): number of timed runs. Defaults to
            :data:`DEFAULT_TIMING_REPEATS`.

    Returns:
        float: minimum wall-clock time over the repeats [s].
    """
    best = float("inf")
    for _ in range(max(1, int(repeats))):
        start = time.perf_counter()
        propagate()
        best = min(best, time.perf_counter() - start)
    return best


def saveRuntimeTable(scenarioName, scenarioFileDir, rows, caseHeader="Case"):
    """Write the BSM-vs-MuJoCo runtime table next to the scenario for its docstring.

    Writes a CSV with a header row (for ``:header-rows: 1``) to
    ``<scenarioFileDir>/results/<scenarioName>_runtime.csv``. The scenario docstring embeds
    it with ``.. csv-table:: :file: results/<name>_runtime.csv``; the path is module-relative
    because Sphinx ``autodoc`` resolves the ``csv-table`` ``:file:`` option relative to the
    module that owns the docstring.

    Args:
        scenarioName (str): the scenario base name (e.g. ``"scenarioCompareOrbit"``); the file
            written is ``<scenarioName>_runtime.csv``.
        scenarioFileDir (str): the directory of the scenario file
            (``os.path.dirname(__file__)``), used to locate the ``results`` folder.
        rows (list): one entry per timed case, each ``(caseLabel, bsmSeconds,
            mujocoSeconds)``. ``mujocoSeconds`` may be ``None`` when Basilisk is built without
            MuJoCo, in which case the MuJoCo and speedup columns report ``n/a``.
        caseHeader (str, optional): header for the first (case-label) column. Defaults to
            ``"Case"``.
    """
    header = (caseHeader, "BSM [s]", "MuJoCo [s]", "Speedup (BSM/MuJoCo)")
    buffer = io.StringIO()
    # Quote case labels containing commas so the csv-table directive sees four columns.
    writer = csv.writer(buffer, quoting=csv.QUOTE_MINIMAL, lineterminator="\n")
    writer.writerow(header)
    for caseLabel, bsmSeconds, mujocoSeconds in rows:
        if mujocoSeconds is not None and mujocoSeconds > 0.0:
            speedup = "{:.2f}".format(bsmSeconds / mujocoSeconds)
            mujocoCell = "{:.4f}".format(mujocoSeconds)
        else:
            speedup = "n/a"
            mujocoCell = "n/a"
        writer.writerow([caseLabel, "{:.4f}".format(bsmSeconds), mujocoCell, speedup])
    content = buffer.getvalue()

    resultsPath = os.path.join(scenarioFileDir, "results")
    os.makedirs(resultsPath, exist_ok=True)
    with open(os.path.join(resultsPath, scenarioName + "_runtime.csv"), "w") as handle:
        handle.write(content)
