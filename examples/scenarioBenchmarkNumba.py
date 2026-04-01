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
"""
Benchmark attitude-control scenarios across C++, Python, and Numba implementations.

This script measures two key timings for each simulation run:
    init  -- InitializeSimulation (one-time setup; includes Numba cache load)
    exec  -- ExecuteSimulation    (steady-state runtime cost)

The minimum timing is used as the primary performance metric because it best
approximates achievable throughput with minimal OS scheduling noise.

Two scenario groups are evaluated:
    Pointing:
        Compare a single control module implemented in C++, Python, and Numba.

    Feedback:
        Compare full FSW + sensor stacks implemented in C++ vs Numba.
"""

from __future__ import annotations

import contextlib
import io
import os
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Callable, Any, Literal

sys.path.insert(0, os.path.dirname(__file__))

import scenarioAttitudeFeedback
import scenarioAttitudeFeedbackNumba
import scenarioAttitudePointing
import scenarioAttitudePointingNumba
import scenarioAttitudePointingPy

from Basilisk.utilities import SimulationBaseClass

sep = "=" * 72


@dataclass(frozen=True)
class TimingStats:
    """Container for summary statistics of a timing sample."""

    min: float
    median: float
    max: float
    stdev: float


@dataclass(frozen=True)
class BenchmarkResult:
    """Stores timing statistics for a single benchmark configuration."""

    label: str
    init: TimingStats
    exec: TimingStats


@dataclass(frozen=True)
class Scenario:
    """
    Represents a benchmarkable simulation scenario.

    Attributes:
        label: Human-readable name for reporting.
        run: Callable that executes the scenario once.
    """

    label: str
    run: Callable[[], Any]


def computeStats(samples: list[float]) -> TimingStats:
    """
    Compute descriptive statistics for a list of timing samples.

    Args:
        samples: List of timing measurements.

    Returns:
        TimingStats object containing min, median, max, and stdev.

    Raises:
        ValueError: If the input list is empty.
    """
    if not samples:
        raise ValueError("Cannot compute statistics of an empty sample list.")

    return TimingStats(
        min=min(samples),
        median=statistics.median(samples),
        max=max(samples),
        stdev=statistics.stdev(samples) if len(samples) > 1 else 0.0,
    )


@contextlib.contextmanager
def suppressOutput():
    """
    Context manager that suppresses stdout and stderr.

    This prevents scenario print statements from interfering with
    benchmark timing and output readability.
    """
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        yield


@contextlib.contextmanager
def patchedSimTimers():
    """
    Context manager that patches simulation methods to measure execution time.

    Temporarily replaces:
        - InitializeSimulation
        - ExecuteSimulation

    to capture timing information for each run.

    Yields:
        dict: Mutable dictionary with keys 'init' and 'exec' storing durations.

    Guarantees:
        Original methods are restored after exiting the context.
    """
    timings = {"init": 0.0, "exec": 0.0}

    origInit = SimulationBaseClass.SimBaseClass.InitializeSimulation
    origExec = SimulationBaseClass.SimBaseClass.ExecuteSimulation

    def timedInit(self):
        """Wrapped InitializeSimulation that records execution time."""
        t0 = time.perf_counter()
        origInit(self)
        timings["init"] = time.perf_counter() - t0

    def timedExec(self):
        """Wrapped ExecuteSimulation that records execution time."""
        t0 = time.perf_counter()
        origExec(self)
        timings["exec"] = time.perf_counter() - t0

    SimulationBaseClass.SimBaseClass.InitializeSimulation = timedInit
    SimulationBaseClass.SimBaseClass.ExecuteSimulation = timedExec

    try:
        yield timings
    finally:
        SimulationBaseClass.SimBaseClass.InitializeSimulation = origInit
        SimulationBaseClass.SimBaseClass.ExecuteSimulation = origExec


def measureScenario(scenario: Scenario, nRuns: int = 10) -> BenchmarkResult:
    """
    Benchmark a scenario over multiple runs.

    Args:
        scenario: Scenario object containing label and run function.
        nRuns: Number of repetitions.

    Returns:
        BenchmarkResult containing timing statistics for init and exec.
    """
    initTimes: list[float] = []
    execTimes: list[float] = []

    with patchedSimTimers() as timings:
        for _ in range(nRuns):
            with suppressOutput():
                scenario.run()
            initTimes.append(timings["init"])
            execTimes.append(timings["exec"])

    return BenchmarkResult(
        label=scenario.label,
        init=computeStats(initTimes),
        exec=computeStats(execTimes),
    )


def formatSeconds(seconds: float) -> str:
    """
    Format a duration using adaptive time units.

    Args:
        seconds: Duration in seconds.

    Returns:
        Formatted duration string.
    """
    if seconds >= 0.1:
        return f"{seconds:.3f} s"
    if seconds >= 1e-3:
        return f"{seconds * 1e3:.2f} ms"
    return f"{seconds * 1e6:.1f} us"


def printResultsTable(results: list[BenchmarkResult], title: str, labelCol: int = 28) -> None:
    """
    Print a single compact benchmark table for one scenario family.

    Args:
        results: Benchmark results to print.
        title: Title of the scenario family.
        labelCol: Width of the label column.
    """
    execBaseline = results[0].exec.min

    print(title)
    print(
        f"  {'Module':<{labelCol}}  "
        f"{'exec min':>10}  {'exec med':>10}  {'exec max':>10}  {'exec min rel':>12}  "
        f"{'init min':>10}  {'init med':>10}  {'init max':>10}"
    )
    print(
        f"  {'-' * labelCol}  "
        f"{'-' * 10}  {'-' * 10}  {'-' * 10}  {'-' * 12}  "
        f"{'-' * 10}  {'-' * 10}  {'-' * 10}"
    )

    for result in results:
        execRatio = result.exec.min / execBaseline if execBaseline > 0.0 else float("inf")
        print(
            f"  {result.label:<{labelCol}}  "
            f"{formatSeconds(result.exec.min):>10}  "
            f"{formatSeconds(result.exec.median):>10}  "
            f"{formatSeconds(result.exec.max):>10}  "
            f"{execRatio:>11.3f}x  "
            f"{formatSeconds(result.init.min):>10}  "
            f"{formatSeconds(result.init.median):>10}  "
            f"{formatSeconds(result.init.max):>10}  "
        )

    print()


def runGroup(title: str, scenarios: list[Scenario], nRuns: int = 10) -> None:
    """
    Execute and report a group of benchmark scenarios.

    Args:
        title: Title describing the scenario family.
        scenarios: Scenarios to benchmark.
        nRuns: Number of runs per scenario.
    """
    results = [measureScenario(s, nRuns=nRuns) for s in scenarios]
    printResultsTable(results, f"{title} ({nRuns} runs)")

def run(case: Literal["pointing", "feedback"] = "feedback", nRuns: int = 10) -> None:
    """Main scenario entry point"""

    if case == "pointing":
        runGroup(
            title="Pointing  - single control module replaced",
            scenarios=[
                Scenario("C++   (all C++)", lambda: scenarioAttitudePointing.run(False, False)),
                Scenario("Python (1 Py module)", lambda: scenarioAttitudePointingPy.run(False)),
                Scenario("Numba  (1 Nb module)", lambda: scenarioAttitudePointingNumba.run(False)),
            ],
            nRuns=nRuns,
        )
    elif case == "feedback":
        runGroup(
            title="Feedback  - all 4 FSW+sensor modules replaced",
            scenarios=[
                Scenario(
                    "C++   (all C++)",
                    lambda: scenarioAttitudeFeedback.run(False, False, False, False, False),
                ),
                Scenario(
                    "Numba  (4 Nb modules)",
                    lambda: scenarioAttitudeFeedbackNumba.run(False, False, False, False),
                ),
            ],
            nRuns=nRuns,
        )


if __name__ == "__main__":
    """Entry point: run all benchmark scenario groups."""
    for case in ("pointing", "feedback"):
        run(case)
