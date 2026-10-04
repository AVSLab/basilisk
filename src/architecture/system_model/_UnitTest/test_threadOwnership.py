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

"""Exercise Python simulation lifetimes with separate startup and shutdown deadlines."""

import faulthandler
import gc
from pathlib import Path
import subprocess
import sys
import time
import weakref

STARTUP_TIMEOUT = 120  # [s]
EXECUTION_TIMEOUT = 30  # [s]
TRACEBACK_MARGIN = 5  # [s]
STARTUP_POLL_INTERVAL = 0.05  # [s]

if __name__ == "__main__":
    # Diagnose native loading and Python imports before the parent startup limit.
    faulthandler.enable()
    faulthandler.dump_traceback_later(STARTUP_TIMEOUT - TRACEBACK_MARGIN)

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


def _run_with_deadlines(command, ready_path, startup_timeout=STARTUP_TIMEOUT,
                        execution_timeout=EXECUTION_TIMEOUT):
    """Allow cold imports while retaining a separate execution and shutdown limit.

    :param command: Child interpreter command and arguments.
    :param ready_path: File the child creates after importing its dependencies.
    :param startup_timeout: Maximum wait for startup in seconds.
    :param execution_timeout: Maximum wait for execution and process exit in seconds.
    :return: Completed process including captured standard output and error.
    """
    stdout_path = ready_path.with_suffix(".stdout")
    stderr_path = ready_path.with_suffix(".stderr")
    timed_out = False
    phase = "startup"
    timeout = startup_timeout
    # Files cannot fill a pipe and block the child while the parent awaits readiness.
    with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open("w", encoding="utf-8") as stderr:
        process = subprocess.Popen(command, stdout=stdout, stderr=stderr)
        try:
            deadline = time.monotonic() + startup_timeout
            while not ready_path.exists() and process.poll() is None:
                if time.monotonic() >= deadline:
                    raise subprocess.TimeoutExpired(command, startup_timeout)
                time.sleep(STARTUP_POLL_INTERVAL)
            phase = "execution/shutdown"
            timeout = execution_timeout
            process.wait(timeout=execution_timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
        finally:
            if process.poll() is None:
                process.kill()
            process.wait()

    output = stdout_path.read_text(encoding="utf-8", errors="replace")
    errors = stderr_path.read_text(encoding="utf-8", errors="replace")
    if timed_out:
        pytest.fail(f"Child {phase} timed out after {timeout} seconds.\nstdout:\n{output}\nstderr:\n{errors}",
                    pytrace=False)
    return subprocess.CompletedProcess(command, process.returncode, output, errors)


@pytest.mark.parametrize("phase", ["success", "startup", "execution"])
def test_subprocess_deadlines_report_phase_and_output(tmp_path, phase):
    """Detect stalls in either phase and retain child output for diagnosis.

    :param tmp_path: Temporary directory supplied by pytest.
    :param phase: Successful completion or the phase in which the child stalls.
    """
    ready_path = tmp_path / "ready"
    code = """import sys, time
from pathlib import Path
if sys.argv[2] == 'startup':
    time.sleep(60)  # [s]
print('child stdout', flush=True)
print('child stderr', file=sys.stderr, flush=True)
Path(sys.argv[1]).touch()
if sys.argv[2] == 'execution':
    time.sleep(60)  # [s]
"""
    command = [sys.executable, "-c", code, str(ready_path), phase]
    if phase == "success":
        result = _run_with_deadlines(command, ready_path)
        assert result.returncode == 0, result.stderr
        assert "child stdout" in result.stdout
        assert "child stderr" in result.stderr
    else:
        # Zero deadlines make failure deterministic without relying on short sleeps.
        startup_timeout = 0 if phase == "startup" else STARTUP_TIMEOUT  # [s]
        execution_timeout = 0 if phase == "execution" else EXECUTION_TIMEOUT  # [s]
        with pytest.raises(pytest.fail.Exception, match=f"Child {phase}") as error:
            _run_with_deadlines(command, ready_path, startup_timeout, execution_timeout)
        if phase == "execution":
            assert "child stdout" in str(error.value)
            assert "child stderr" in str(error.value)


@pytest.mark.parametrize("thread_count", [1, 3])
@pytest.mark.parametrize(
    "case", ["unstarted", "reinitialize", "selfInit", "reset", "update", "requestStop", "killThread"]
)
def test_thread_ownership(case, thread_count, tmp_path):
    """Preserve Python scheduling and exception handling without shutdown hangs.

    :param case: Simulation lifetime or exception scenario.
    :param thread_count: Number of configured simulation workers.
    :param tmp_path: Temporary directory supplied by pytest.
    """
    ready_path = tmp_path / "ready"
    result = _run_with_deadlines(
        [sys.executable, str(Path(__file__).resolve()), case, str(thread_count), str(ready_path)],
        ready_path,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert ready_path.exists(), "The child exited without reaching the ownership check"


if __name__ == "__main__":
    faulthandler.dump_traceback_later(EXECUTION_TIMEOUT - TRACEBACK_MARGIN)
    if len(sys.argv) > 3:
        Path(sys.argv[3]).touch()
    _run_case(sys.argv[1], int(sys.argv[2]))
