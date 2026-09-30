# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Exercise Python simulation lifetimes in subprocesses with a shutdown deadline."""

import gc
from pathlib import Path
import subprocess
import sys
import weakref

import pytest

from Basilisk.architecture import sim_model, sysModel
from Basilisk.utilities import SimulationBaseClass, macros


class _LifecycleModel(sysModel.SysModel):
    """Record scheduled calls and inject failures at a selected lifecycle stage."""

    def __init__(self):
        super().__init__()
        self.failure = None
        self.times = []

    def SelfInit(self):
        """Optionally raise during worker initialization."""
        if self.failure == "selfInit":
            raise RuntimeError("Expected self-init failure")

    def Reset(self, current_time):
        """Clear recorded calls or raise during worker reset."""
        if self.failure == "reset":
            raise RuntimeError("Expected reset failure")
        self.times.clear()

    def UpdateState(self, current_time):
        """Record simulation time in nanoseconds or raise during execution."""
        if self.failure == "update":
            raise RuntimeError("Expected update failure")
        self.times.append(current_time)


def _make_simulation(thread_count):
    """Create two independent processes, including one explicit thread assignment."""
    simulation = SimulationBaseClass.SimBaseClass()
    simulation.TotalSim.resetThreads(thread_count)
    models = []
    period = macros.sec2nano(0.1)  # [ns]
    for index in range(2):
        process = simulation.CreateNewProcess(f"process{index}")
        task_name = f"task{index}"
        process.addTask(simulation.CreateNewTask(task_name, period))
        model = _LifecycleModel()
        simulation.AddModelToTask(task_name, model)
        models.append(model)
        if index == 0:
            simulation.TotalSim.addProcessToThread(process.processData, thread_count - 1)
    return simulation, models


def _execute_and_check(simulation, models):
    """Check exact scheduled output after initialization or reinitialization."""
    stop_time = macros.sec2nano(0.2)  # [ns]
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(stop_time)
    simulation.ExecuteSimulation()
    expected_times = [0, macros.sec2nano(0.1), stop_time]  # [ns]
    for model in models:
        assert model.times == expected_times


def _run_case(case, thread_count):
    """Run one lifetime scenario in an isolated interpreter."""
    if case in ("requestStop", "killThread"):
        for _ in range(thread_count):
            worker = sim_model.SimThreadExecution()
            assert worker.threadValid()
            getattr(worker, case)()
            assert not worker.threadValid()
            # A missing shutdown wake-up fails at the subprocess deadline.
            worker.lockThread()
            worker.requestStop()
            worker.killThread()
        return

    if case == "unstarted":
        for _ in range(10):
            simulation = sim_model.SimModel()
            simulation.resetThreads(thread_count)
            simulation.deleteThreads()
            simulation.deleteThreads()
        return

    simulation, models = _make_simulation(thread_count)
    if case == "reinitialize":
        for count in (thread_count, thread_count + 1, 1):
            if simulation.simulationInitialized:
                simulation.TotalSim.resetThreads(count)
            _execute_and_check(simulation, models)
            assert simulation.TotalSim.getThreadCount() == count
    else:
        models[0].failure = case
        with pytest.raises(RuntimeError):
            _execute_and_check(simulation, models)
        simulation.TotalSim.deleteThreads()
        simulation.TotalSim.deleteThreads()
        assert simulation.TotalSim.getThreadCount() == 0
        models[0].failure = None
        simulation.TotalSim.resetThreads(thread_count)
        _execute_and_check(simulation, models)

    # Destroy an idle, initialized simulation while module proxies remain alive.
    owner_ref = weakref.ref(simulation.TotalSim)
    del simulation
    gc.collect()
    assert owner_ref() is None
    assert all(model.times for model in models)


@pytest.mark.parametrize("thread_count", [1, 3])
@pytest.mark.parametrize(
    "case", ["unstarted", "reinitialize", "selfInit", "reset", "update", "requestStop", "killThread"]
)
def test_thread_ownership(case, thread_count):
    """Preserve Python scheduling and exception handling without shutdown hangs."""
    timeout = 30  # [s]
    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve()), case, str(thread_count)],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


if __name__ == "__main__":
    _run_case(sys.argv[1], int(sys.argv[2]))
