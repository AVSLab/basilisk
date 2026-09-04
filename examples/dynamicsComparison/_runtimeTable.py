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
multi-body-in-orbit) time the Backsubstitution Method (BSM, the :ref:`spacecraft` module)
against MuJoCo (the :ref:`MJScene<MJScene>` module) on the same problem and persist a small
CSV table.

The table is written to ``<scenarioFileDir>/results/<scenarioName>_runtime.csv`` when a
scenario runs with ``saveTiming=True``. These CSVs are optional local benchmark artifacts;
the documentation build does not generate or embed machine-dependent timings.

Timing methodology
------------------
Each configuration is built once for a discarded warm-up and then rebuilt for each of
``repeats`` measured trials. Only ``ExecuteSimulation()`` is timed, and the table reports
the median. Absolute times and speedup ratios depend on the machine, compiler, build
configuration, and system load.
"""

import csv
import io
import os
import statistics
import time

# Default number of measured trials after one discarded warm-up.
DEFAULT_TIMING_REPEATS = 5


def _executePropagation(build):
    """Build and execute one simulation, returning its propagation time [s]."""
    built = build()
    simulation = built[0] if isinstance(built, tuple) else built
    start = time.perf_counter()
    simulation.ExecuteSimulation()
    return time.perf_counter() - start


def medianPropagationTime(build, repeats=DEFAULT_TIMING_REPEATS):
    """Return median propagation time after one discarded warm-up [s].

    Args:
        build (callable): zero-argument callable that returns an initialized simulation
            or a tuple whose first item is the simulation. Any remaining tuple items are
            retained while the simulation executes so their Python wrappers stay alive.
        repeats (int, optional): number of measured propagations. Defaults to
            :data:`DEFAULT_TIMING_REPEATS`.

    Returns:
        float: median wall-clock propagation time [s].
    """
    built = build()
    simulation = built[0] if isinstance(built, tuple) else built
    simulation.ExecuteSimulation()
    samples = []
    for _ in range(max(1, int(repeats))):
        samples.append(_executePropagation(build))
    return statistics.median(samples)


def interleavedPropagationTimes(builders, repeats=DEFAULT_TIMING_REPEATS):
    """Return median propagation times for builders measured in alternating order.

    Every configuration receives one discarded warm-up. Measured rounds alternate
    between forward and reverse builder order, reducing thermal and background-load
    drift correlated with a particular engine.

    Args:
        builders (sequence): zero-argument simulation builders.
        repeats (int, optional): measured rounds after warm-up. Defaults to
            :data:`DEFAULT_TIMING_REPEATS`.

    Returns:
        tuple: median propagation time for each builder, in input order [s].
    """
    stats = interleavedPropagationStats(builders, repeats)
    return tuple(values["median"] for values in stats)


def interleavedPropagationStats(builders, repeats=DEFAULT_TIMING_REPEATS,
                                warmupBuilders=None):
    """Return median, minimum, and standard deviation from interleaved propagations.

    Args:
        builders (sequence): zero-argument measured simulation builders.
        repeats (int, optional): measured rounds after warm-up.
        warmupBuilders (sequence, optional): builders for discarded warm-ups. This permits
            a shorter warm-up horizon while keeping the measured builders unchanged.

    Returns:
        tuple: one ``{"median", "min", "std"}`` dictionary per measured builder [s].
    """
    builders = tuple(builders)
    if not builders:
        return ()
    warmupBuilders = builders if warmupBuilders is None else tuple(warmupBuilders)
    if len(warmupBuilders) != len(builders):
        raise ValueError("warmupBuilders must match builders in length.")

    for build in warmupBuilders:
        built = build()
        simulation = built[0] if isinstance(built, tuple) else built
        simulation.ExecuteSimulation()

    samples = [[] for _ in builders]
    for trial in range(max(1, int(repeats))):
        order = (range(len(builders)) if trial % 2 == 0
                 else range(len(builders) - 1, -1, -1))
        for index in order:
            samples[index].append(_executePropagation(builders[index]))
    return tuple({
        "median": statistics.median(values),
        "min": min(values),
        "std": statistics.pstdev(values),
    } for values in samples)


def pairedPropagationTimes(buildBsm, buildMujoco, repeats=DEFAULT_TIMING_REPEATS):
    """Return interleaved median BSM and MuJoCo propagation times [s]."""
    return interleavedPropagationTimes((buildBsm, buildMujoco), repeats)


def saveRuntimeTable(scenarioName, scenarioFileDir, rows, caseHeader="Case",
                     resultsDir=None):
    """Write an optional local BSM-vs-MuJoCo runtime table.

    Writes a CSV with a header row to
    ``<scenarioFileDir>/results/<scenarioName>_runtime.csv``.

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
        resultsDir (str, optional): explicit output directory. Defaults to the
            ``results`` folder beside the scenario file.
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

    targetResults = (
        os.path.join(scenarioFileDir, "results")
        if resultsDir is None
        else resultsDir
    )
    os.makedirs(targetResults, exist_ok=True)
    with open(os.path.join(
            targetResults, scenarioName + "_runtime.csv"), "w") as handle:
        handle.write(content)
