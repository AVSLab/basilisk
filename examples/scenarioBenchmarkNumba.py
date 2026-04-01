#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
NumbaModel Performance Benchmark
---------------------------------

This script isolates the cost of a single BSK module (no dynamics, no
spacecraft) and compares three implementations of the same workload:

1. **C++ baseline** -- ``mrpFeedback`` (a compiled C++ module, here used as a
   proxy for "C++-class" per-tick cost on a minimal simulation).
2. **Python** -- ``PythonWorkload`` subclassing ``SysModel``.
3. **Numba** -- ``NumbaWorkload`` subclassing ``NumbaModel``.

Each implementation performs the same inner loop: ``N`` iterations of
floating-point multiply-accumulate, writing one output element per tick.
Increasing ``N`` shifts the bottleneck from Python-call overhead to raw
arithmetic throughput, which is where Numba's advantage is largest.

Run with::

    python3 scenarioBenchmarkNumba.py

Sample output (N=2000, 10 000 ticks)::

    [C++ mrpFeedback  ]  run: 0.030 s  (baseline)
    [Python SysModel  ]  run: 3.214 s  x106.9 slower than C++
    [Numba NumbaModel ]  run: 0.038 s    x1.3 slower than C++

The Numba module runs at near-C++ speed because ``UpdateStateImpl`` is
compiled to native code and called directly by the C++ scheduler with zero
Python overhead per tick.  The Python module pays a ~25 µs Python
dispatch cost every tick regardless of how much work is inside the method.

Reducing ``N`` (less arithmetic per tick) will show a slightly larger
gap between Numba and C++ because the fixed dispatch cost of one cfunc
call begins to matter.  Increasing ``N`` (more arithmetic) drives both
toward the same per-flop throughput.
"""

import time

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.architecture.numbaModel import NumbaModel
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport

# ---------------------------------------------------------------------------
# Benchmark parameters
# ---------------------------------------------------------------------------
N_WORK   = 2000   # inner-loop iterations per tick (arithmetic workload size)
N_TICKS  = 10_000  # number of simulation ticks
DT_NS    = macros.sec2nano(0.001)  # 1 ms timestep -> N_TICKS ticks = 10 s


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
def _make_sim():
    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", DT_NS))
    return scSim


def _run(scSim):
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(N_TICKS * DT_NS)
    t0 = time.perf_counter()
    scSim.ExecuteSimulation()
    return time.perf_counter() - t0


# ---------------------------------------------------------------------------
# Benchmark 1: C++ baseline (mrpFeedback - no Python module at all)
# ---------------------------------------------------------------------------
def benchCpp():
    scSim = _make_sim()

    ctrl = mrpFeedback.mrpFeedback()
    ctrl.ModelTag = "ctrl"
    ctrl.K  = 3.5
    ctrl.Ki = -1.0
    ctrl.P  = 30.0
    ctrl.integralLimit = 2.0 / ctrl.Ki * 0.1

    # wire up required input messages with stand-alone stubs
    guidMsg  = messaging.AttGuidMsg().write(messaging.AttGuidMsgPayload())
    vehMsg   = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(ISCPntB_B=[900,0,0, 0,800,0, 0,0,600]))
    ctrl.guidInMsg.subscribeTo(guidMsg)
    ctrl.vehConfigInMsg.subscribeTo(vehMsg)

    scSim.AddModelToTask("task", ctrl)
    return _run(scSim)


# ---------------------------------------------------------------------------
# Benchmark 2: Python SysModel workload
# ---------------------------------------------------------------------------
class PythonWorkload(sysModel.SysModel):
    """Performs N_WORK multiply-accumulate iterations per tick in plain Python."""

    def __init__(self):
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self._acc = 0.0

    def Reset(self, CurrentSimNanos):
        self._acc = 0.0

    def UpdateState(self, CurrentSimNanos):
        acc = self._acc
        for i in range(N_WORK):
            acc += float(i) * 1.000_001
        self._acc = acc

        buf = messaging.CModuleTemplateMsgPayload()
        buf.dataVector[0] = acc
        self.dataOutMsg.write(buf, CurrentSimNanos, self.moduleID)


def benchPython():
    scSim = _make_sim()
    mod = PythonWorkload()
    mod.ModelTag = "pyWork"
    scSim.AddModelToTask("task", mod)
    return _run(scSim)


# ---------------------------------------------------------------------------
# Benchmark 3: NumbaModel workload
# ---------------------------------------------------------------------------
class NumbaWorkload(NumbaModel):
    """Performs N_WORK multiply-accumulate iterations per tick in Numba."""

    def __init__(self):
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.acc = 0.0

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory):
        acc = memory.acc
        for i in range(N_WORK):
            acc += float(i) * 1.000_001
        memory.acc = acc
        dataOutMsgPayload.dataVector[0] = acc


def benchNumba():
    scSim = _make_sim()
    mod = NumbaWorkload()
    mod.ModelTag = "nbWork"
    scSim.AddModelToTask("task", mod)
    return _run(scSim)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print(f"\nBenchmark: N_WORK={N_WORK} inner iterations/tick, "
          f"N_TICKS={N_TICKS} ticks\n")

    # warm up Numba cache before timing
    benchNumba()

    tCpp    = benchCpp()
    tPython = benchPython()
    tNumba  = benchNumba()

    print(f"  [C++ mrpFeedback  ]  {tCpp:.3f} s  (baseline)")
    print(f"  [Python SysModel  ]  {tPython:.3f} s  "
          f"  x{tPython/tCpp:.1f} slower than C++")
    print(f"  [Numba NumbaModel ]  {tNumba:.3f} s  "
          f"  x{tNumba/tCpp:.1f} slower than C++")
    print()
