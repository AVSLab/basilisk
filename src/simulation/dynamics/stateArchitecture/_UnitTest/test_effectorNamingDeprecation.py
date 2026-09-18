# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Verify legacy automatic-naming diagnostics through the public Python interfaces.

Leave the production removal date at September 14, 2027. These tests replace the
deprecation helper's clock to exercise both sides of that deadline. Changing the
production date instead invalidates the expected warning categories and messages.
"""

import datetime
import gc
from pathlib import Path
import subprocess
import sys
import threading
import types
import warnings
import weakref

import numpy as np
import pytest

from Basilisk.architecture import bskLogging, sysModel
from Basilisk.simulation import (
    fuelTank,
    hingedRigidBodyStateEffector,
    igbmNoiseStateEffector,
    linearSpringMassDamper,
    meanRevertingNoiseStateEffector,
    spacecraft,
    sphericalPendulum,
    stateArchitecture,
    thrusterStateEffector,
)

from Basilisk.utilities import SimulationBaseClass, _effectorNaming, deprecated, macros

from test_actuatorEffectorNaming import FIELDS as ACTUATOR_FIELDS, make_effector as make_actuator
from test_effectorNamingIntegration import make_legacy_system
from test_multibodyEffectorNaming import KINDS as MULTIBODY_KINDS, make_model
from test_simpleEffectorNaming import FIELDS as SIMPLE_FIELDS, get_name, set_name


WARNING = "Legacy automatic effector naming"
GUIDE = "https://hanspeterschaub.info/basilisk/Support/User/effectorNaming.html"
PANEL_FIELDS = (
    "nameOfThetaState", "nameOfThetaDotState",
    "nameOfInertialPositionProperty", "nameOfInertialVelocityProperty",
    "nameOfInertialAttitudeProperty", "nameOfInertialAngVelocityProperty",
)
KINDS = ("panel", *SIMPLE_FIELDS, *ACTUATOR_FIELDS, *MULTIBODY_KINDS)


def set_today(monkeypatch, today):
    """Change the deprecation helper's clock without changing the computer's date."""
    class FixedDate(datetime.date):
        @classmethod
        def today(cls):
            return cls.fromisoformat(today)

    monkeypatch.setattr(
        deprecated, "datetime", types.SimpleNamespace(date=FixedDate, datetime=datetime.datetime)
    )


@pytest.fixture(autouse=True)
def warning_environment(monkeypatch, recwarn):
    """Keep tests independent of the real date and prove warnings ignore BSK log verbosity."""
    set_today(monkeypatch, "2027-09-13")
    # Count emissions from each manager even when Python would normally hide
    # identical warnings raised from the same source line.
    warnings.simplefilter("always", deprecated.BSKDeprecationWarning)
    warnings.simplefilter("always", deprecated.BSKUrgentDeprecationWarning)
    previous = bskLogging.getDefaultLogLevel()
    bskLogging.setDefaultLogLevel(bskLogging.BSK_SILENT)
    try:
        yield
    finally:
        bskLogging.setDefaultLogLevel(previous)


def warning_output(recwarn, count, category=deprecated.BSKDeprecationWarning):
    """Check this feature's dated warnings independently of unrelated deprecations."""
    records = [record for record in recwarn if WARNING in str(record.message)]
    output = "\n".join(str(record.message) for record in records)
    recwarn.clear()
    assert len(records) == count, output
    assert all(record.category is category for record in records), output
    assert all(record.filename == __file__ for record in records), output
    if count:
        assert "dynManager.useManagerLocalEffectorNames = True" in output
        assert "2027-09-14" in output
        assert GUIDE in output
    return output


def make_vehicle(local=None):
    """Leave the default policy untouched unless the caller explicitly selects one."""
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.hub.IHubPntBc_B = 100.0 * np.eye(3)  # [kg m^2]
    if local is not None:
        vehicle.dynManager.useManagerLocalEffectorNames = local
    return vehicle


def make_panel(custom=False):
    """Create a panel, optionally assigning every current default as an explicit name."""
    panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    panel.mass = 1.0  # [kg]
    panel.d = 0.5  # [m]
    panel.IPntS_S = np.eye(3)  # [kg m^2]
    if custom:
        for field in PANEL_FIELDS:
            setattr(panel, field, getattr(panel, field))
    return panel


def make_builtin(kind, custom):
    """Create one configured effector and retain any owners needed by its attachments."""
    if kind in MULTIBODY_KINDS:
        model = make_model(kind, kind, custom)
        return model.effector, model
    if kind == "panel":
        return make_panel(custom), None
    if kind in ACTUATOR_FIELDS:
        effector = make_actuator(kind, 1)
        if custom:
            for field in ACTUATOR_FIELDS[kind]:
                setattr(effector, field, getattr(effector, field))
        return effector, None
    factories = {
        "tank": fuelTank.FuelTank,
        "damper": linearSpringMassDamper.LinearSpringMassDamper,
        "pendulum": sphericalPendulum.SphericalPendulum,
        "thruster": thrusterStateEffector.ThrusterStateEffector,
        "ou": meanRevertingNoiseStateEffector.MeanRevertingNoiseStateEffector,
        "igbm": igbmNoiseStateEffector.IgbmNoiseStateEffector,
    }
    effector = factories[kind]()
    owner = None
    if kind == "tank":
        owner = fuelTank.FuelTankModelConstantVolume()
        owner.propMassInit = 2.0  # [kg]
        owner.radiusTankInit = 0.5  # [m]
        effector.setTankModel(owner)
    elif kind == "damper":
        effector.massInit = 0.5  # [kg]
        effector.pHat_B = [1.0, 0.0, 0.0]
    elif kind == "pendulum":
        effector.massInit = 0.5  # [kg]
        effector.pendulumRadius = 0.5  # [m]
    elif kind == "thruster":
        owner = thrusterStateEffector.THRSimConfig()
        owner.thrLoc_B = [0.0, 1.0, 0.0]  # [m]
        owner.thrDir_B = [1.0, 0.0, 0.0]
        owner.MaxThrust = 0.2  # [N]
        owner.steadyIsp = 200.0  # [s]
        owner.cutoffFrequency = 2.0  # [rad/s]
        effector.addThruster(owner)
    if custom:
        for field in SIMPLE_FIELDS[kind]:
            set_name(kind, effector, field, get_name(kind, effector, field))
    return effector, owner


@pytest.mark.parametrize("kind", KINDS)
@pytest.mark.parametrize("custom", (False, True), ids=("automatic", "explicit"))
def test_each_builtin_tracks_automatic_names(kind, custom, recwarn):
    """Check each effector's registration hooks with automatic and explicitly assigned names."""
    vehicle = make_vehicle()
    effector, owner = make_builtin(kind, custom)
    vehicle.addStateEffector(effector)
    vehicle.initializeDynamics()
    warning_output(recwarn, int(not custom))


@pytest.mark.parametrize("first", ("registerStates", "registerProperties", "Reset"))
def test_direct_registration_and_partial_overrides(first, recwarn):
    """Direct registration and spacecraft reset must warn about automatic properties after explicit states."""
    panel = make_panel()
    for field in PANEL_FIELDS[:2]:
        setattr(panel, field, getattr(panel, field))
    if first == "Reset":
        vehicle = make_vehicle()
        vehicle.addStateEffector(panel)
        manager = vehicle.dynManager
        vehicle.Reset(0)  # [ns]
    else:
        manager = stateArchitecture.DynParamManager()
        getattr(panel, first)(manager)
    # Panel state registration also registers its properties.
    warning_output(recwarn, 1)
    second = "registerProperties" if first == "registerStates" else "registerStates"
    getattr(panel, second)(manager)
    warning_output(recwarn, 0)


@pytest.mark.parametrize("kind", ACTUATOR_FIELDS)
@pytest.mark.parametrize("model", (0, 1), ids=("unused-angle", "active-angle"))
def test_optional_automatic_actuator_names(kind, model, recwarn):
    """With all other names explicit, only a registered automatic wheel-angle state should warn."""
    vehicle = make_vehicle()
    effector = make_actuator(kind, model)
    for field in ACTUATOR_FIELDS[kind]:
        if "Thetas" not in field:
            setattr(effector, field, getattr(effector, field))
    vehicle.addStateEffector(effector)
    vehicle.initializeDynamics()
    warning_output(recwarn, int(model != 0))


def test_nested_automatic_names(recwarn):
    """The manager diagnoses automatic names in a child of a fully custom parent."""
    vehicle = make_vehicle()
    parent = make_model("prescribed", "parent", custom=True)
    child = make_model("slide", "child", custom=False)
    parent.effector.addStateEffector(child.effector)
    vehicle.addStateEffector(parent.effector)
    vehicle.initializeDynamics()
    warning_output(recwarn, 1)


def test_independent_and_rebuilt_managers_warn(recwarn):
    """An overlapping simulation or collected manager cannot suppress another manager's warning."""
    live = make_vehicle()
    live_panel = make_panel()
    live.addStateEffector(live_panel)
    live.initializeDynamics()
    warning_output(recwarn, 1)
    for _ in range(2):
        fresh = make_vehicle()
        panel = make_panel()
        fresh.addStateEffector(panel)
        fresh.initializeDynamics()
        warning_output(recwarn, 1)
        del fresh, panel
        gc.collect()
    live.initializeDynamics()
    warning_output(recwarn, 0)


def test_hub_and_manual_names_do_not_warn(recwarn):
    """Hub data and ordinary manager registrations do not imply automatic effector naming."""
    vehicle = make_vehicle()
    vehicle.dynManager.registerState(1, 1, "manualState")
    vehicle.dynManager.createProperty("manualProperty", [[0.0]])
    vehicle.initializeDynamics()
    warning_output(recwarn, 0)


def test_legacy_system_shares_warning_across_units(recwarn):
    """The deprecated system reports once for its shared manager and gives an applicable migration."""
    system, units, panels, loads, ports = make_legacy_system()
    system.initializeDynamics()
    output = warning_output(recwarn, 1)
    assert "SpacecraftSystem users must migrate to Spacecraft" in output
    system.initializeDynamics()
    warning_output(recwarn, 0)


def make_empty_simulation(local=None):
    """Schedule a hub-only spacecraft so a legacy panel can be attached after initialization."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    step = 0.01  # [s]
    process.addTask(simulation.CreateNewTask("task", macros.sec2nano(step)))
    vehicle = make_vehicle(local)
    simulation.AddModelToTask("task", vehicle)
    return simulation, vehicle


def make_simulation(local=None):
    """Schedule a panel and spacecraft for complete initialization through C++ workers."""
    simulation, vehicle = make_empty_simulation(local)
    panel = make_panel()
    vehicle.addStateEffector(panel)
    simulation.AddModelToTask("task", panel)
    return simulation, vehicle, panel


@pytest.mark.parametrize("today,category", (
    ("2027-09-13", deprecated.BSKDeprecationWarning),
    ("2027-09-14", deprecated.BSKUrgentDeprecationWarning),
))
def test_date_selects_standard_warning_category(today, category, monkeypatch, recwarn):
    """Check the migration date and warning category once through the public simulation API."""
    set_today(monkeypatch, today)
    simulation, vehicle, panel = make_simulation()
    simulation.InitializeSimulation()
    warning_output(recwarn, 1, category)


@pytest.mark.parametrize("local", (False, True))
def test_scheduled_initialization_reports_once_per_manager(local, recwarn):
    """Mixed effectors share one warning per manager; opting in suppresses it across the simulation."""
    simulation, vehicle, panel = make_simulation(local)
    spring, owner = make_builtin("damper", False)
    vehicle.addStateEffector(spring)
    other = make_vehicle(local)
    other_panel = make_panel()
    other.addStateEffector(other_panel)
    simulation.AddModelToTask("task", other)
    simulation.AddModelToTask("task", other_panel)
    simulation.InitializeSimulation()
    warning_output(recwarn, 0 if local else 2)
    simulation.InitializeSimulation()
    warning_output(recwarn, 0)


@pytest.mark.parametrize("entry", ("simulation", "task"))
def test_task_reset_reports_new_automatic_names(entry, recwarn):
    """Task resets report newly used legacy names once and preserve the caller's source location."""
    simulation, vehicle = make_empty_simulation()
    simulation.InitializeSimulation()
    panel = make_panel()
    vehicle.addStateEffector(panel)
    simulation.AddModelToTask("task", panel)
    warning_output(recwarn, 0)

    for attempt in range(2):
        if entry == "simulation":
            simulation.ResetTask("task")
        else:
            simulation.TaskList[0].resetTask(simulation.TotalSim.CurrentNanos)
        assert vehicle.dynManager.getStateObject(panel.nameOfThetaState) is not None
        warning_output(recwarn, int(attempt == 0))


class LifecycleCallback(sysModel.SysModel):
    """Run a test action from a real native lifecycle callback."""

    def __init__(self, reset=None, self_init=None):
        super().__init__()
        self.reset_callback = reset
        self.self_init_callback = self_init

    def SelfInit(self):
        """Run the optional self-initialization action."""
        if self.self_init_callback is not None:
            self.self_init_callback()

    def Reset(self, current_time):
        """Run the optional reset action."""
        if self.reset_callback is not None:
            self.reset_callback()

    def UpdateState(self, current_time):
        """Leave physical propagation to the spacecraft."""


def check_callback_warning_errors():
    """Cover each lifecycle boundary once, including direct SWIG calls made from a callback."""
    for entry in ("initialize", "simulation_reset", "task_reset"):
        simulation, vehicle = make_empty_simulation()
        if entry != "initialize":
            simulation.InitializeSimulation()
        panel = make_panel()
        vehicle.addStateEffector(panel)
        simulation.AddModelToTask("task", panel)
        calls = []

        def reset():
            # Use fresh manager proxies to exercise the shared SWIG warning guard.
            vehicle.hub.linkInStates(vehicle.dynManager)
            panel.registerStates(vehicle.dynManager)
            vehicle.initializeDynamics()
            vehicle.Reset(0)  # [ns]
            calls.append("reset")

        model = LifecycleCallback(reset, lambda: panel.registerStates(vehicle.dynManager))
        simulation.AddModelToTask("task", model, ModelPriority=-2)
        if entry == "initialize":
            initialize = simulation.InitializeSimulation
        elif entry == "simulation_reset":
            initialize = lambda: simulation.ResetTask("task")
        else:
            initialize = lambda: simulation.TaskList[0].resetTask(0)  # [ns]
        with pytest.raises(deprecated.BSKUrgentDeprecationWarning, match=WARNING):
            initialize()
        assert simulation.simulationInitialized
        assert calls == ["reset"]
        assert not _effectorNaming.isReportDeferred(vehicle.dynManager)
        initialize()
        assert calls == ["reset", "reset"]
        simulation.ConfigureStopTime(0)  # [ns]
        simulation.ExecuteSimulation()
        del initialize, simulation, model, vehicle, panel
        gc.collect()


def check_nested_simulation_warning_errors():
    """Retained and temporary nested simulations must finish before warning exceptions reach the caller."""
    for temporary in (False, True):
        completed = []
        references = []

        def make_nested(depth):
            simulation, vehicle = make_empty_simulation()
            children = []

            def initialize_child():
                if children:
                    child = children[0]
                else:
                    child = make_nested(depth - 1) if depth > 1 else make_simulation()
                    references.append(weakref.ref(child[1]))
                    if not temporary:
                        children.append(child)
                child[0].InitializeSimulation()
                assert child[0].simulationInitialized
                assert _effectorNaming.isReportDeferred(child[1].dynManager)
                completed.append(depth)

            # Cross both native initialization phases in one nested chain.
            model = (LifecycleCallback(reset=initialize_child) if depth == 2
                     else LifecycleCallback(self_init=initialize_child))
            simulation.AddModelToTask("task", model, ModelPriority=-2)
            return simulation, vehicle, model

        simulation, vehicle, model = make_nested(2)
        for attempt in range(2):
            if temporary or attempt == 0:
                with pytest.raises(deprecated.BSKUrgentDeprecationWarning, match=WARNING):
                    simulation.InitializeSimulation()
            else:
                simulation.InitializeSimulation()
            assert simulation.simulationInitialized
            assert completed == [1, 2] * (attempt + 1)
            assert simulation.TotalSim._getPythonExecutionContext() == 0
            assert not _effectorNaming._deferredManagers
            assert not _effectorNaming._activeScopes
            gc.collect()
            if temporary:
                assert all(reference() is None for reference in references)
        del simulation, vehicle, model
        gc.collect()


def check_concurrent_nested_simulations():
    """An independent caller must receive its warning while another simulation is still blocked."""
    first_waiting = threading.Event()
    second_finished = threading.Event()
    reports = []
    errors = []

    def run(index):
        try:
            outer, outer_vehicle = make_empty_simulation()
            inner, inner_vehicle, panel = make_simulation()

            def wait_for_other():
                if index == 0:
                    first_waiting.set()
                    assert second_finished.wait(5)  # [s]
                else:
                    assert first_waiting.wait(5)  # [s]

            waiter = LifecycleCallback(reset=wait_for_other)
            nested = LifecycleCallback(reset=inner.InitializeSimulation)
            inner.AddModelToTask("task", waiter, ModelPriority=-2)
            outer.AddModelToTask("task", nested, ModelPriority=-2)
            with pytest.raises(deprecated.BSKUrgentDeprecationWarning, match=WARNING):
                outer.InitializeSimulation()
            assert outer.simulationInitialized and inner.simulationInitialized
            assert not _effectorNaming.isReportDeferred(inner_vehicle.dynManager)
            reports.append(index)
            if index == 1:
                second_finished.set()
            outer.InitializeSimulation()
        except BaseException as error:
            errors.append(error)
            second_finished.set()

    threads = [threading.Thread(target=run, args=(index,)) for index in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(10)  # [s]
    assert not any(thread.is_alive() for thread in threads)
    assert not errors, errors
    assert reports == [1, 0]
    gc.collect()


def check_initialization_waits_for_all_workers():
    """Both native lifecycle barriers must wait for outstanding callbacks after another worker fails."""
    for hook in ("self_init", "reset"):
        simulation, vehicle = make_empty_simulation()
        process = simulation.CreateNewProcess("other_process")
        step = 0.01  # [s]
        process.addTask(simulation.CreateNewTask("other_task", macros.sec2nano(step)))
        simulation.TotalSim.resetThreads(2)
        for index, configured in enumerate(simulation.TotalSim.processList):
            simulation.TotalSim.addProcessToThread(configured, index)
        inner, inner_vehicle, panel = make_simulation()
        entered = threading.Event()
        failed = threading.Event()
        release = threading.Event()
        finished = threading.Event()
        errors = []

        def fail():
            assert entered.wait(5)  # [s]
            failed.set()
            raise ValueError("intentional initialization failure")

        def wait():
            entered.set()
            assert release.wait(5)  # [s]
            inner.InitializeSimulation()
            assert _effectorNaming.isReportDeferred(inner_vehicle.dynManager)

        failing = LifecycleCallback(**{hook: fail})
        waiting = LifecycleCallback(**{hook: wait})
        simulation.AddModelToTask("task", failing, ModelPriority=-2)
        simulation.AddModelToTask("other_task", waiting)

        def initialize():
            try:
                simulation.InitializeSimulation()
            except BaseException as error:
                errors.append(error)
            finally:
                finished.set()

        caller = threading.Thread(target=initialize)
        caller.start()
        try:
            assert failed.wait(5)  # [s]
            assert not finished.wait(0.1)  # [s]
        finally:
            release.set()
            caller.join(10)  # [s]
        assert not caller.is_alive()
        assert len(errors) == 1 and isinstance(errors[0], RuntimeError), errors
        assert not simulation.simulationInitialized
        assert inner.simulationInitialized
        # The failed outer call must leave the inner warning unconsumed.
        with pytest.raises(deprecated.BSKUrgentDeprecationWarning, match=WARNING):
            inner.InitializeSimulation()
        errors.clear()
        del simulation, vehicle, inner, inner_vehicle, panel, failing, waiting
        gc.collect()


def run_callback_regressions():
    """Run representative crash regressions using one warning category and one interpreter startup."""
    bskLogging.setDefaultLogLevel(bskLogging.BSK_SILENT)
    with pytest.MonkeyPatch.context() as monkeypatch, warnings.catch_warnings():
        set_today(monkeypatch, "2027-09-14")
        warnings.filterwarnings("error", message=f".*{WARNING}.*", category=deprecated.BSKUrgentDeprecationWarning)
        for check in (check_callback_warning_errors, check_nested_simulation_warning_errors,
                      check_concurrent_nested_simulations, check_initialization_waits_for_all_workers):
            check()
            assert not _effectorNaming._deferredManagers
            assert not _effectorNaming._activeScopes


def test_callback_warning_errors_in_subprocess():
    """Isolate native crash regressions so a failure cannot terminate the pytest worker."""
    code = (
        "import sys, faulthandler\n"
        "faulthandler.enable()\n"
        f"sys.path.insert(0, {str(Path(__file__).parent)!r})\n"
        "from test_effectorNamingDeprecation import run_callback_regressions\n"
        "run_callback_regressions()\n"
    )
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=30)  # [s]
    assert result.returncode == 0, result.stdout + result.stderr


def test_deferral_preserves_unrelated_managers_and_original_errors(recwarn):
    """A failed lifecycle scope releases its guard without consuming warnings or hiding another error."""
    vehicle = make_vehicle()
    panel = make_panel()
    vehicle.addStateEffector(panel)
    other = make_vehicle()
    other_panel = make_panel()
    other.addStateEffector(other_panel)
    with pytest.raises(ValueError, match="initialization failed"):
        with _effectorNaming.deferReports([vehicle]):
            # Distinct SWIG proxies and nested scopes must share the same guard.
            alias = types.SimpleNamespace(dynManager=vehicle.dynManager)
            with _effectorNaming.deferReports([alias]):
                vehicle.initializeDynamics()
            warning_output(recwarn, 0)
            other.initializeDynamics()
            warning_output(recwarn, 1)
            raise ValueError("initialization failed")
    vehicle.initializeDynamics()
    warning_output(recwarn, 1)


def test_warning_error_releases_all_scheduled_managers(recwarn):
    """A warning exception for one manager must leave the others able to report on retry."""
    simulation, vehicle, panel = make_simulation()
    other = make_vehicle()
    other_panel = make_panel()
    other.addStateEffector(other_panel)
    simulation.AddModelToTask("task", other)
    with warnings.catch_warnings():
        warnings.filterwarnings("error", message=f".*{WARNING}.*", category=deprecated.BSKDeprecationWarning)
        for _ in range(2):
            with pytest.raises(deprecated.BSKDeprecationWarning, match=WARNING):
                simulation.InitializeSimulation()
            assert simulation.simulationInitialized
            assert not _effectorNaming.isReportDeferred(vehicle.dynManager)
            assert not _effectorNaming.isReportDeferred(other.dynManager)
        simulation.InitializeSimulation()
    warning_output(recwarn, 0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
