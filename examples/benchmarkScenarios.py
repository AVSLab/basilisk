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

Two timings are reported for each run:
  init  -- InitializeSimulation (one-time setup; includes Numba cache load on first call)
  exec  -- ExecuteSimulation    (the steady-state per-run cost; the fair comparison)

The minimum is used as the primary metric: it best represents the achievable
throughput, since it is least affected by OS scheduling jitter.

Two scenario families are compared:

  Pointing  -- scenarioAttitudePointing*
    C++  : mrpFeedback, attTrackingError, inertial3D, simpleNav all C++
    Py   : mrpFeedback replaced by PythonMRPPD (SysModel subclass)
    Numba: mrpFeedback replaced by NumbaMRPPD (NumbaModel subclass)

  Feedback  -- scenarioAttitudeFeedback*  (orbital, gravity, integral option)
    C++  : all FSW + sensor modules in C++
    Numba: simpleNav, inertial3D, attTrackingError, mrpFeedback all NumbaModel

Usage:
    python3 benchmarkScenarios.py
"""

import io
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import scenarioAttitudePointing
import scenarioAttitudePointingPy
import scenarioAttitudePointingNumba
import scenarioAttitudeFeedback
import scenarioAttitudeFeedbackNumba

from Basilisk.utilities import SimulationBaseClass

N_RUNS = 10

# ---------------------------------------------------------------------------
# Patch SimBaseClass to capture init / exec times without touching scenarios
# ---------------------------------------------------------------------------
_lastInit = [0.0]
_lastExec = [0.0]

_orig_init = SimulationBaseClass.SimBaseClass.InitializeSimulation
_orig_exec = SimulationBaseClass.SimBaseClass.ExecuteSimulation


def _timedInit(self):
    """Time the original InitializeSimulation and store the result in _lastInit."""
    t0 = time.perf_counter()
    _orig_init(self)
    _lastInit[0] = time.perf_counter() - t0


def _timedExec(self):
    """Time the original ExecuteSimulation and store the result in _lastExec."""
    t0 = time.perf_counter()
    _orig_exec(self)
    _lastExec[0] = time.perf_counter() - t0


SimulationBaseClass.SimBaseClass.InitializeSimulation = _timedInit
SimulationBaseClass.SimBaseClass.ExecuteSimulation    = _timedExec


def _silent(fn):
    """Redirect stdout to a buffer while running fn, to suppress scenario printouts."""
    old = sys.stdout
    sys.stdout = io.StringIO()
    try:
        fn()
    finally:
        sys.stdout = old


def measure(runFn, label):
    """Run runFn N_RUNS times, timing InitializeSimulation and ExecuteSimulation."""
    _silent(runFn)   # warmup: excluded from results

    initTimes, execTimes = [], []
    for _ in range(N_RUNS):
        _silent(runFn)
        initTimes.append(_lastInit[0])
        execTimes.append(_lastExec[0])

    def stats(ts):
        return dict(min=min(ts), median=statistics.median(ts),
                    max=max(ts), stdev=statistics.stdev(ts))

    return {"label": label, "init": stats(initTimes), "exec": stats(execTimes)}


def table(results, key, baseline, title, label_col=28):
    print(f"\n  {title}")
    print(f"  {key.upper()} time  ({key} = "
          f"{'ExecuteSimulation' if key == 'exec' else 'InitializeSimulation'})")
    print(f"  {'Module':<{label_col}} {'min':>8}  {'median':>8}  {'max':>8}  "
          f"{'stdev':>7}  {'vs C++':>7}")
    print(f"  {'-'*label_col} {'-'*8}  {'-'*8}  {'-'*8}  {'-'*7}  {'-'*7}")
    for r in results:
        s     = r[key]
        ratio = s["min"] / baseline
        print(f"  {r['label']:<{label_col}} {s['min']:>7.3f}s  {s['median']:>7.3f}s"
              f"  {s['max']:>7.3f}s  {s['stdev']:>6.4f}s  {ratio:>6.2f}x")


def runPointingScenarios():
    """Compare the pointing scenario with mrpFeedback in C++ vs Python vs Numba."""

    pointing = [
        measure(lambda: scenarioAttitudePointing.run(False, False),   "C++   (all C++)"),
        measure(lambda: scenarioAttitudePointingPy.run(False),         "Python (1 Py module)"),
        measure(lambda: scenarioAttitudePointingNumba.run(False),      "Numba  (1 Nb module)"),
    ]

    sep = "=" * 72
    print(f"\n{sep}")
    print(f"  Attitude benchmark  ({N_RUNS} runs each, 10-min sim, 0.1 s timestep)")
    print(sep)

    p_base_exec = pointing[0]["exec"]["min"]
    p_base_init = pointing[0]["init"]["min"]
    table(pointing, "exec", p_base_exec,
        "Pointing  - single control module replaced")
    table(pointing, "init", p_base_init, "")

    print(f"\n{sep}\n")

# ---------------------------------------------------------------------------
# Feedback scenario: all FSW + sensor modules vs all Numba
# ---------------------------------------------------------------------------
def runFeedbackScenarios():
    """Compare the full feedback scenario with all FSW + sensor modules in C++ vs all in Numba."""

    feedback = [
        measure(lambda: scenarioAttitudeFeedback.run(False,False,False,False,False),
                "C++   (all C++)"),
        measure(lambda: scenarioAttitudeFeedbackNumba.run(False,False,False,False),
                "Numba  (4 Nb modules)"),
    ]

    sep = "=" * 72

    f_base_exec = feedback[0]["exec"]["min"]
    f_base_init = feedback[0]["init"]["min"]
    table(feedback, "exec", f_base_exec,
        "Feedback  - all 4 FSW+sensor modules replaced")
    table(feedback, "init", f_base_init, "")

    print(f"\n{sep}\n")

if __name__ == "__main__":
    runPointingScenarios()
    runFeedbackScenarios()
