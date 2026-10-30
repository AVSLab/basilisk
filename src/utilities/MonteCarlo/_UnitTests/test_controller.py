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

import inspect
import json
import os
import _thread
from collections import Counter
import threading
import multiprocessing as mp
import signal

import numpy as np
import pandas as pd
import pytest

from Basilisk.simulation import motorVoltageInterface, spacecraft
from Basilisk.utilities import simHelpers
from Basilisk.utilities.MonteCarlo import Controller as controller_module
from Basilisk.utilities.MonteCarlo.Controller import SimulationExecutor, SimulationParameters
from Basilisk.utilities.MonteCarlo.DataWriter import DataWriter
from Basilisk.utilities.MonteCarlo.Dispersions import (
    NormalVectorAngleDispersion,
    NormalVectorDispersion,
    SingleVariableDispersion,
    UniformDispersion,
    UniformVectorAngleDispersion,
    VectorVariableDispersion,
)


class DummyHub:
    def __init__(self):
        self.mHub = 1.0  # [kg]
        self.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
        self.unitVector = np.array([[1.0], [0.0], [0.0]])  # [-]


class DummyModel:
    def __init__(self):
        self.RNGSeed = 0
        self.hub = DummyHub()


class DummyModelWithoutSeed:
    pass


class DummyTask:
    def __init__(self):
        self.TaskModels = [DummyModel(), DummyModelWithoutSeed()]


class DummyVectorContainer:
    def __init__(self):
        self.values = [0.0, 0.0, 0.0]


class DummyDynModel:
    def __init__(self):
        self.scObject = DummyModel()


class DummySimulation:
    def __init__(self):
        self.TaskList = [DummyTask()]
        self.vectorContainer = DummyVectorContainer()
        self.dynModel = DummyDynModel()

    def get_DynModel(self):
        return self.dynModel


class DeferredDataWriter(DataWriter):
    """Hold retained data in the queue until the controller starts shutdown."""

    def __init__(self, queue):
        super().__init__(queue)
        self.release = mp.Event()

    def run(self):
        """Drain and consolidate the backlog after all producers finish."""
        timeout = 30.0  # [s]
        assert self.release.wait(timeout), "Controller did not release the writer"
        super().run()


def execute_retention_simulation(sim):
    times = np.arange(32)  # [ns]
    sim.retained = np.column_stack((times, np.random.random((32, 3))))


def retain_simulation_data(sim):
    return {"samples": sim.retained}


@pytest.mark.parametrize("workers", [1, 2, 4])
def test_controller_drains_retained_data_before_return(tmp_path, monkeypatch, workers):
    """Verify a delayed writer saves every run before the controller returns."""
    controller = controller_module.Controller()
    controller.setSimulationFunction(DummySimulation)
    controller.setExecutionFunction(execute_retention_simulation)
    controller.setExecutionCount(8)
    controller.setRetentionQueueSize(8)
    controller.setThreadCount(workers)
    controller.setArchiveDir(str(tmp_path / "archive"))
    controller.setVerbose(False)
    controller.setShowProgressBar(False)
    policy = controller_module.RetentionPolicy()
    policy.addRetentionFunction(retain_simulation_data)
    controller.addRetentionPolicy(policy)
    shutdown = controller_module.Controller._shutdown_data_writer

    def release_and_shutdown(self, abort=False):
        writer = self.dataWriter
        if writer is not None:
            writer.release.set()
        shutdown(self, abort=abort)
        if writer is not None and not abort:
            assert writer.exitcode == 0

    monkeypatch.setattr(controller_module, "DataWriter", DeferredDataWriter)
    monkeypatch.setattr(
        controller_module.Controller, "_shutdown_data_writer", release_and_shutdown
    )
    interrupt_handler = signal.getsignal(signal.SIGINT)
    try:
        assert controller.executeSimulations() == []
    finally:
        signal.signal(signal.SIGINT, interrupt_handler)

    combined = pd.read_pickle(tmp_path / "archive" / "samples.data")
    expected_columns = [(run, component) for run in range(8) for component in range(3)]
    assert list(combined.columns) == expected_columns
    np.testing.assert_array_equal(combined.index, np.arange(32))
    for run in range(8):
        expected = np.random.RandomState(run * 10).random_sample((32, 3))
        retained = controller.getRetainedData(run)
        assert retained["index"] == run
        np.testing.assert_array_equal(retained["custom"]["samples"][:, 1:], expected)
        np.testing.assert_array_equal(combined[run].to_numpy(), expected)


class ExitingDataWriter(DataWriter):
    """Exit abruptly after receiving the first retained result."""

    def run(self):
        """Simulate a writer process dying while producers are active."""
        self._queue.get()
        os._exit(7)


class FailedConsolidationWriter(DataWriter):
    """Fail after accepting all retained results and the end marker."""

    def run(self):
        """Simulate an output error during final consolidation."""
        while not self._queue.get()[2]:
            pass
        raise OSError("Simulated consolidation failure")


def retain_worker_pid(sim):
    return {"worker_pid": np.array([[0, os.getpid()]])}


def fail_simulation(sim):
    raise ValueError("Simulated run failure")


def make_retention_controller(tmp_path, workers, archive=True):
    controller = controller_module.Controller()
    controller.setSimulationFunction(DummySimulation)
    controller.setExecutionFunction(execute_retention_simulation)
    controller.setExecutionCount(9)
    controller.setThreadCount(workers)
    controller.setRetentionQueueSize(1)
    controller.setVerbose(False)
    controller.setShowProgressBar(False)
    if archive:
        controller.setArchiveDir(str(tmp_path / "archive"))
    policy = controller_module.RetentionPolicy()
    policy.addRetentionFunction(retain_simulation_data)
    policy.addRetentionFunction(retain_worker_pid)
    controller.addRetentionPolicy(policy)
    return controller


def assert_controller_closed(controller, original_children):
    assert controller.dataWriter is None
    assert controller.dataOutQueue is None
    assert controller.multiProcManager is None
    assert {child.pid for child in mp.active_children()} == original_children


@pytest.mark.timeout(30)  # [s]
def test_pool_reuse_recycling_and_repeated_execution(tmp_path):
    """Reuse workers, bound their lifetimes, and finish a partial final batch."""
    controller = make_retention_controller(tmp_path, 2)
    original_children = {child.pid for child in mp.active_children()}
    for limit in (None, 2):
        controller.setMaxTasksPerChild(limit)
        assert controller.executeSimulations() == []
        assert_controller_closed(controller, original_children)
        counts = Counter(
            int(controller.getRetainedData(run)["custom"]["worker_pid"][0, 1])
            for run in range(9)
        )
        if limit is None:
            assert len(counts) <= 2
            assert max(counts.values()) > 1
        else:
            assert max(counts.values()) <= limit
            assert len(counts) >= 5
        combined = pd.read_pickle(tmp_path / "archive" / "samples.data")
        for run in range(9):
            expected = np.random.RandomState(run * 10).random_sample((32, 3))
            np.testing.assert_array_equal(combined[run].to_numpy(), expected)


@pytest.mark.timeout(30)  # [s]
@pytest.mark.parametrize("initial_conditions", [False, True])
@pytest.mark.parametrize("failure_stage", ["preparation", "submission"])
def test_task_failures_report_indices_and_preserve_successes(
    tmp_path, monkeypatch, initial_conditions, failure_stage
):
    """Report preparation and submission errors while retaining later runs."""
    controller = make_retention_controller(tmp_path, 2)
    controller.setExecutionCount(6)
    original_children = {child.pid for child in mp.active_children()}
    cases = list(range(6))
    archive = tmp_path / "archive"
    failure_directory = archive
    generator_name = "generateSims"
    if initial_conditions:
        cases = [2, 4, 6, 8, 10, 12]
        failure_directory = tmp_path / "initial"
        failure_directory.mkdir()
        for run in cases:
            (failure_directory / f"run{run}.json").write_text("{}")
        controller.setICDir(str(failure_directory))
        controller.setICRunFlag(True)
        generator_name = "generateICSims"

    rejected = [cases[0], cases[1], cases[3]]
    failed = cases[:4] if failure_stage == "preparation" else rejected
    generate = getattr(controller_module.Controller, generator_name)

    def generate_with_errors(self, case_list):
        for sim in generate(self, case_list):
            if sim.index in rejected:
                if failure_stage == "preparation":
                    raise TypeError("Simulated task-preparation failure")
                sim.configureFunction = lambda sim: None
            yield sim

    monkeypatch.setattr(
        controller_module.Controller, generator_name, generate_with_errors
    )
    if initial_conditions:
        result = controller.runInitialConditions(cases)
    else:
        result = controller.executeSimulations()
    assert result == failed
    assert json.loads((failure_directory / "failures.txt").read_text()) == failed
    assert_controller_closed(controller, original_children)
    combined = pd.read_pickle(archive / "samples.data")
    successful = [run for run in cases if run not in failed]
    assert list(combined.dropna(axis=1, how="all").columns) == [
        (run, component) for run in successful for component in range(3)
    ]
    for run in successful:
        expected = np.random.RandomState(run * 10).random_sample((32, 3))
        np.testing.assert_array_equal(combined[run].to_numpy(), expected)


@pytest.mark.parametrize("method", ["setMaxTasksPerChild", "setRetentionQueueSize"])
@pytest.mark.parametrize("invalid", [0, -1, True, 1.5, "8"])
def test_worker_settings_reject_invalid_values(method, invalid):
    """Reject settings that cannot provide a positive worker or queue limit."""
    with pytest.raises(ValueError, match="positive integer or None"):
        getattr(controller_module.Controller(), method)(invalid)


@pytest.mark.timeout(30)  # [s]
@pytest.mark.parametrize("workers", [1, 2])
@pytest.mark.parametrize("writer", [ExitingDataWriter, FailedConsolidationWriter])
def test_writer_failure_raises_and_cleans_up(tmp_path, monkeypatch, workers, writer):
    """Report writer failure during production or shutdown without hanging."""
    controller = make_retention_controller(tmp_path, workers)
    original_children = {child.pid for child in mp.active_children()}
    monkeypatch.setattr(controller_module, "DataWriter", writer)
    with pytest.raises(RuntimeError, match="Monte Carlo data writer"):
        controller.executeSimulations()
    assert_controller_closed(controller, original_children)


@pytest.mark.timeout(30)  # [s]
@pytest.mark.parametrize("workers", [1, 2])
def test_interrupt_with_full_retention_queue_cleans_up(tmp_path, monkeypatch, workers):
    """Cancel blocked producers and the deferred writer on a keyboard interrupt."""
    controller = make_retention_controller(tmp_path, workers)
    original_children = {child.pid for child in mp.active_children()}
    monkeypatch.setattr(controller_module, "DataWriter", DeferredDataWriter)
    stopped = threading.Event()

    def interrupt_when_full():
        interval = 0.01  # [s]
        while not stopped.wait(interval):
            queue = controller.dataOutQueue
            if queue is not None and queue.full():
                _thread.interrupt_main()
                return

    watcher = threading.Thread(target=interrupt_when_full)
    watcher.start()
    try:
        with pytest.raises(KeyboardInterrupt):
            controller.executeSimulations()
    finally:
        stopped.set()
        watcher.join()
    assert_controller_closed(controller, original_children)


@pytest.mark.timeout(30)  # [s]
@pytest.mark.parametrize("workers", [1, 2])
@pytest.mark.parametrize("archive", [False, True])
def test_initial_conditions_with_and_without_writer(tmp_path, workers, archive):
    """Run saved cases with a bounded writer or only their individual archives."""
    controller = make_retention_controller(tmp_path, workers, archive=archive)
    original_children = {child.pid for child in mp.active_children()}
    initial = tmp_path / "initial"
    initial.mkdir()
    cases = [1, 2, 3, 4, 5]
    for run in cases:
        (initial / f"run{run}.json").write_text("{}")
    controller.setICDir(str(initial))
    controller.setICRunFlag(True)
    controller.setMaxTasksPerChild(2)
    assert controller.runInitialConditions(cases) == []
    assert_controller_closed(controller, original_children)
    for run in cases:
        expected = np.random.RandomState(run * 10).random_sample((32, 3))
        retained = controller.getRetainedData(run)
        np.testing.assert_array_equal(retained["custom"]["samples"][:, 1:], expected)
        if archive:
            combined = pd.read_pickle(tmp_path / "archive" / "samples.data")
            np.testing.assert_array_equal(combined[run].to_numpy(), expected)
    controller.setExecutionFunction(fail_simulation)
    assert controller.runInitialConditions([2, 5]) == [2, 5]
    assert json.loads((initial / "failures.txt").read_text()) == [2, 5]
    assert_controller_closed(controller, original_children)


@pytest.mark.timeout(30)  # [s]
@pytest.mark.parametrize("cases", [[], [0], [0, 2]])
def test_initial_conditions_accept_numpy_indices(tmp_path, cases):
    """Execute NumPy case arrays, including case zero and an empty selection."""
    controller = make_retention_controller(tmp_path, 2)
    original_children = {child.pid for child in mp.active_children()}
    initial = tmp_path / "initial"
    initial.mkdir()
    for run in cases:
        (initial / f"run{run}.json").write_text("{}")
    controller.setICDir(str(initial))
    controller.setICRunFlag(True)

    assert controller.runInitialConditions(np.array(cases, dtype=np.int64)) == []
    assert_controller_closed(controller, original_children)
    for run in cases:
        expected = np.random.RandomState(run * 10).random_sample((32, 3))
        retained = controller.getRetainedData(run)
        np.testing.assert_array_equal(retained["custom"]["samples"][:, 1:], expected)


def test_empty_campaign_and_legacy_controller_settings(tmp_path):
    """Handle empty campaigns and controller archives predating the new settings."""
    controller = make_retention_controller(tmp_path, 2)
    controller.setExecutionCount(0)
    original_children = {child.pid for child in mp.active_children()}
    assert controller.executeSimulations() == []
    assert_controller_closed(controller, original_children)
    del controller.maxTasksPerChild
    del controller.retentionQueueSize
    controller.setExecutionCount(3)
    assert controller.executeSimulations() == []
    loaded = controller_module.Controller.load(str(tmp_path / "archive"))
    loaded.setExecutionCount(1)
    assert loaded.executeSimulations() == []
    worker_pid = loaded.getRetainedData(0)["custom"]["worker_pid"][0, 1]
    assert worker_pid != os.getpid()
    assert_controller_closed(loaded, original_children)


@pytest.mark.parametrize(
    "dispersion_class",
    [SingleVariableDispersion, VectorVariableDispersion],
)
def test_dispersion_base_classes_are_abstract(dispersion_class):
    """Verify Python enforces the Monte Carlo dispersion interfaces."""
    assert inspect.isabstract(dispersion_class)

    with pytest.raises(TypeError, match="abstract method.*generate"):
        dispersion_class("test.variable", None)


def test_apply_modification_updates_nested_attributes():
    """Verify Monte Carlo path strings update the nested object."""
    sim = DummySimulation()

    mass_path = "TaskList[0].TaskModels[0].hub.mHub"
    SimulationExecutor.applyModification(sim, mass_path, "25.0")

    assert sim.TaskList[0].TaskModels[0].hub.mHub == 25.0
    assert not hasattr(sim, mass_path)

    center_of_mass_path = "TaskList[0].TaskModels[0].hub.r_BcB_B"
    SimulationExecutor.applyModification(sim, center_of_mass_path, "[1.0, 2.0, 3.0]")

    assert sim.TaskList[0].TaskModels[0].hub.r_BcB_B == [1.0, 2.0, 3.0]

    indexed_path = "vectorContainer.values[1]"
    SimulationExecutor.applyModification(sim, indexed_path, "7.5")

    assert sim.vectorContainer.values == [0.0, 7.5, 0.0]

    method_path = "get_DynModel().scObject.hub.mHub"
    SimulationExecutor.applyModification(sim, method_path, "35.0")

    assert sim.get_DynModel().scObject.hub.mHub == 35.0
    assert not hasattr(sim, method_path)


def test_indexed_modification_updates_swig_copy_on_read_container():
    """Verify indexed paths write back copy-on-read SWIG containers."""
    sim = DummySimulation()
    sim.rwVoltageIO = motorVoltageInterface.MotorVoltageInterface()
    initial_gains = [0.02, 0.02, 0.02]  # [N*m/V]
    new_gain = 0.123  # [N*m/V]

    sim.rwVoltageIO.setGains(initial_gains)
    SimulationExecutor.applyModification(
        sim,
        "rwVoltageIO.voltage2TorqueGain[0]",
        str(new_gain)
    )

    assert sim.rwVoltageIO.voltage2TorqueGain[0][0] == new_gain


def test_double_indexed_modification_updates_swig_matrix3d():
    """Verify double-indexed paths write back SWIG Matrix3d attributes."""
    sim = DummySimulation()
    sim.scObject = spacecraft.Spacecraft()
    inertia = [
        900.0, 0.0, 0.0,
        0.0, 800.0, 0.0,
        0.0, 0.0, 600.0
    ]  # [kg*m^2]
    new_cross_inertia = 12.5  # [kg*m^2]

    sim.scObject.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(inertia)
    SimulationExecutor.applyModification(
        sim,
        "scObject.hub.IHubPntBc_B[0][1]",
        str(new_cross_inertia)
    )

    assert sim.scObject.hub.IHubPntBc_B[0][1] == new_cross_inertia


def test_vector_angle_dispersion_reads_nested_method_path():
    """Verify vector dispersions read nested zero-argument method paths."""
    sim = DummySimulation()
    vector_path = "get_DynModel().scObject.hub.unitVector"
    phi_bounds = [-0.001, 0.001]  # [rad]
    theta_bounds = [-0.001, 0.001]  # [rad]

    dispersion = UniformVectorAngleDispersion(
        vector_path,
        phiBoundsOffNom=phi_bounds,
        thetaBoundsOffNom=theta_bounds
    )

    dispersed_vector = dispersion.generate(sim)

    assert np.isclose(np.linalg.norm(dispersed_vector), 1.0)


def test_uniform_vector_angle_dispersion_uses_bounds_off_nominal(monkeypatch):
    """Verify angle dispersions use bounds centered on the nominal vector."""
    sim = DummySimulation()
    vector_path = "get_DynModel().scObject.hub.unitVector"
    sim.get_DynModel().scObject.hub.unitVector = np.array([
        [1.0], [1.0], [1.0]
    ])  # [-]
    phi_bounds = [-0.02, 0.03]  # [rad]
    theta_bounds = [-0.04, 0.05]  # [rad]
    uniform_calls = []

    def record_uniform(lower_bound, upper_bound):
        uniform_calls.append((lower_bound, upper_bound))
        return (lower_bound + upper_bound) / 2.0

    monkeypatch.setattr(np.random, "uniform", record_uniform)

    dispersion = UniformVectorAngleDispersion(
        vector_path,
        phiBoundsOffNom=phi_bounds,
        thetaBoundsOffNom=theta_bounds
    )
    dispersion.generate(sim)

    nominal_vector = sim.get_DynModel().scObject.hub.unitVector
    nominal_vector = nominal_vector / np.linalg.norm(nominal_vector)
    nominal_spherical = dispersion.cart2Spherical(nominal_vector)
    expected_phi_bounds = [
        nominal_spherical[1] + phi_bounds[0],
        nominal_spherical[1] + phi_bounds[1]
    ]
    expected_theta_bounds = [
        nominal_spherical[2] + theta_bounds[0],
        nominal_spherical[2] + theta_bounds[1]
    ]

    assert np.allclose(uniform_calls[0], expected_phi_bounds)
    assert np.allclose(uniform_calls[1], expected_theta_bounds)


def test_normal_vector_angle_dispersion_uses_scalar_angle_samples(monkeypatch):
    """Verify normal angle dispersions draw scalar angle samples."""
    sim = DummySimulation()
    vector_path = "get_DynModel().scObject.hub.unitVector"
    sim.get_DynModel().scObject.hub.unitVector = np.array([
        [1.0], [1.0], [1.0]
    ])  # [-]
    phi_std = 0.02  # [rad]
    theta_std = 0.03  # [rad]
    normal_calls = []

    def record_normal(mean, standard_deviation):
        normal_calls.append((mean, standard_deviation))
        return mean

    monkeypatch.setattr(np.random, "normal", record_normal)

    dispersion = NormalVectorAngleDispersion(
        vector_path,
        phiStd=phi_std,
        thetaStd=theta_std,
        phiBoundsOffNom=[-0.1, 0.1],
        thetaBoundsOffNom=[-0.1, 0.1]
    )
    dispersed_vector = dispersion.generate(sim)

    nominal_vector = sim.get_DynModel().scObject.hub.unitVector
    nominal_vector = nominal_vector / np.linalg.norm(nominal_vector)
    nominal_spherical = dispersion.cart2Spherical(nominal_vector)

    assert np.isclose(np.linalg.norm(dispersed_vector), 1.0)
    assert np.allclose(normal_calls[0], [nominal_spherical[1], phi_std])
    assert np.allclose(normal_calls[1], [nominal_spherical[2], theta_std])
    assert dispersion.getDispersionMag() == [r"0.0 $\sigma$", r"0.0 $\sigma$"]


def test_normal_vector_dispersion_uses_configured_statistics():
    """Verify normal vector dispersions retain mean and standard deviation."""
    sim = DummySimulation()
    vector_path = "TaskList[0].TaskModels[0].hub.r_BcB_B"
    mean = 2.5  # [m]
    std_deviation = 0.0  # [m]

    dispersion = NormalVectorDispersion(
        vector_path,
        mean=mean,
        stdDeviation=std_deviation
    )
    dispersed_vector = dispersion.generate(sim)

    assert np.allclose(dispersed_vector, [mean, mean, mean])


def test_populate_seeds_applies_before_configure_function():
    """Verify archived RNGSeed modifications are set before configuration."""
    configured_seeds = []

    def create_sim():
        return DummySimulation()

    def configure_sim(sim):
        configured_seeds.append(sim.TaskList[0].TaskModels[0].RNGSeed)

    def execute_sim(sim):
        assert sim.TaskList[0].TaskModels[0].RNGSeed == 8675309
        assert sim.TaskList[0].TaskModels[0].hub.mHub == 42.0

    sim_params = SimulationParameters(
        creationFunction=create_sim,
        executionFunction=execute_sim,
        configureFunction=configure_sim,
        retentionPolicies=[],
        dispersions=[],
        shouldDisperseSeeds=False,
        shouldArchiveParameters=False,
        filename="",
        icfilename="",
        index=0,
        modifications={
            "TaskList[0].TaskModels[0].RNGSeed": "8675309",
            "TaskList[0].TaskModels[0].hub.mHub": "42.0",
        }
    )

    result = SimulationExecutor()([sim_params, None])

    assert result == (True, 0)
    assert configured_seeds == [8675309]


def test_uniform_dispersion_randomizes_nested_sim_parameter():
    """Verify generated dispersions update the live simulation parameter."""
    mass_path = "TaskList[0].TaskModels[0].hub.mHub"
    mass_bounds = [24.0, 26.0]  # [kg]
    observed_masses = []
    generated_masses = []

    def create_sim():
        return DummySimulation()

    def execute_sim(sim):
        observed_masses.append(sim.TaskList[0].TaskModels[0].hub.mHub)

    for run_index in range(2):
        sim_params = SimulationParameters(
            creationFunction=create_sim,
            executionFunction=execute_sim,
            configureFunction=None,
            retentionPolicies=[],
            dispersions=[UniformDispersion(mass_path, mass_bounds)],
            shouldDisperseSeeds=False,
            shouldArchiveParameters=False,
            filename="",
            icfilename="",
            index=run_index,
            modifications={}
        )

        result = SimulationExecutor()([sim_params, None])
        generated_mass = SimulationExecutor.parseModificationValue(
            sim_params.modifications[mass_path]
        )

        assert result == (True, run_index)
        generated_masses.append(generated_mass)

    assert observed_masses == generated_masses
    assert observed_masses[0] != observed_masses[1]
    for observed_mass in observed_masses:
        assert mass_bounds[0] <= observed_mass <= mass_bounds[1]
        assert observed_mass != DummyHub().mHub


def test_uniform_dispersion_randomizes_method_path_parameter():
    """Verify generated dispersions update a zero-argument method path."""
    mass_path = "get_DynModel().scObject.hub.mHub"
    mass_bounds = [34.0, 36.0]  # [kg]
    observed_masses = []
    generated_masses = []

    def create_sim():
        return DummySimulation()

    def execute_sim(sim):
        observed_masses.append(sim.get_DynModel().scObject.hub.mHub)

    for run_index in range(2):
        sim_params = SimulationParameters(
            creationFunction=create_sim,
            executionFunction=execute_sim,
            configureFunction=None,
            retentionPolicies=[],
            dispersions=[UniformDispersion(mass_path, mass_bounds)],
            shouldDisperseSeeds=False,
            shouldArchiveParameters=False,
            filename="",
            icfilename="",
            index=run_index,
            modifications={}
        )

        result = SimulationExecutor()([sim_params, None])
        generated_mass = SimulationExecutor.parseModificationValue(
            sim_params.modifications[mass_path]
        )

        assert result == (True, run_index)
        generated_masses.append(generated_mass)

    assert observed_masses == generated_masses
    assert observed_masses[0] != observed_masses[1]
    for observed_mass in observed_masses:
        assert mass_bounds[0] <= observed_mass <= mass_bounds[1]
        assert observed_mass != DummyHub().mHub


def test_disperse_seeds_only_records_seeded_models():
    """Verify random seed dispersions are recorded only for seeded models."""
    sim = DummySimulation()

    random_seeds = SimulationExecutor.disperseSeeds(sim)

    seed_path = "TaskList[0].TaskModels[0].RNGSeed"

    assert set(random_seeds.keys()) == {seed_path}
    assert isinstance(
        SimulationExecutor.parseModificationValue(random_seeds[seed_path]),
        int
    )


if __name__ == "__main__":
    test_apply_modification_updates_nested_attributes()
    test_indexed_modification_updates_swig_copy_on_read_container()
    test_double_indexed_modification_updates_swig_matrix3d()
    test_vector_angle_dispersion_reads_nested_method_path()
    test_normal_vector_dispersion_uses_configured_statistics()
    test_populate_seeds_applies_before_configure_function()
    test_uniform_dispersion_randomizes_nested_sim_parameter()
    test_uniform_dispersion_randomizes_method_path_parameter()
    test_disperse_seeds_only_records_seeded_models()
