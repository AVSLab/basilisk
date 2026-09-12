# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Exercise hinged-panel naming through complete Python simulation lifetimes."""

import gc
import weakref
from dataclasses import dataclass

import numpy as np
import pytest

from Basilisk.architecture import bskLogging, messaging
from Basilisk.simulation import (
    constraintDynamicEffector,
    facetDragDynamicEffector,
    facetSRPDynamicEffector,
    hingedRigidBodyStateEffector,
    spacecraft,
    spacecraftSystem,
    thrusterDynamicEffector,
)
from Basilisk.utilities import SimulationBaseClass, macros


NAME_FIELDS = (
    "nameOfThetaState",
    "nameOfThetaDotState",
    "nameOfInertialPositionProperty",
    "nameOfInertialVelocityProperty",
    "nameOfInertialAttitudeProperty",
    "nameOfInertialAngVelocityProperty",
)
AUTO_PREFIXES = (
    "hingedRigidBodyTheta",
    "hingedRigidBodyThetaDot",
    "hingedRigidBodyInertialPosition",
    "hingedRigidBodyInertialVelocity",
    "hingedRigidBodyInertialAttitude",
    "hingedRigidBodyInertialAngVelocity",
)


@dataclass
class SimulationBundle:
    """Keep each simulation's Python owners alive for the duration of execution."""

    simulation: object
    vehicle: object
    panels: list
    thrusters: list
    setup_names: list


def panel_names(panel):
    """Read the six existing Python attributes."""
    return tuple(getattr(panel, name) for name in NAME_FIELDS)


def build_simulation(manager_local, custom_names=False):
    """Build two panels, exercising custom assignments after attachment in the new mode."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    step = 0.01  # [s]
    process.addTask(simulation.CreateNewTask("task", macros.sec2nano(step)))
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.hub.IHubPntBc_B = np.eye(3) * 100.0  # [kg m^2]
    vehicle.hub.r_CN_NInit = [10.0, -3.0, 4.0]  # [m]
    vehicle.hub.v_CN_NInit = [0.5, -0.2, 0.3]  # [m/s]
    simulation.AddModelToTask("task", vehicle)
    if manager_local:
        import effectorNamingTestSupport

        effectorNamingTestSupport.enableManagerLocalNaming(vehicle.dynManager)

    # Reverse construction order to distinguish it from collection order.
    panels = [hingedRigidBodyStateEffector.HingedRigidBodyStateEffector() for _ in range(2)]
    panels.reverse()
    thrusters = []
    angles = [0.25, -0.4]  # [rad]
    for index, panel in enumerate(panels):
        panel.mass = 1.0  # [kg]
        panel.thetaInit = angles[index]
        panel.r_HB_B = [3.0 * index, 1.0, 0.0]  # [m]
        thruster = thrusterDynamicEffector.ThrusterDynamicEffector()
        if manager_local:
            panel.addDynamicEffector(thruster)
        thrusters.append(thruster)
        if custom_names:
            for name in NAME_FIELDS:
                setattr(panel, name, f"panel{index}_{name}")
        if not manager_local:
            panel.addDynamicEffector(thruster)
        vehicle.addStateEffector(panel)
        simulation.AddModelToTask("task", panel)
    setup_names = [panel_names(panel) for panel in panels]
    return SimulationBundle(simulation, vehicle, panels, thrusters, setup_names)


def run_simulation(bundle, manager_local, custom_names=False):
    """Check public names and bound property storage, then integrate the model."""
    bundle.simulation.InitializeSimulation()
    manager = bundle.vehicle.dynManager
    for index, (panel, thruster) in enumerate(zip(bundle.panels, bundle.thrusters)):
        names = panel_names(panel)
        expected = (
            tuple(f"{prefix}{index + 1}" for prefix in AUTO_PREFIXES)
            if manager_local and not custom_names else bundle.setup_names[index]
        )
        assert names == expected
        assert manager.getStateObject(names[0]).getName() == names[0]
        assert manager.getStateObject(names[1]).getName() == names[1]
        assert thruster.getPropName_inertialPosition() == names[2]
        assert thruster.getPropName_inertialVelocity() == names[3]
        assert thruster.getPropName_inertialAttitude() == names[4]
        assert thruster.getPropName_inertialAngVelocity() == names[5]
        # A write through the manager must be seen through the child's stored pointer.
        marker = np.array([[index + 101.0], [2.0], [3.0]])  # [m]
        manager.setPropertyValue(names[2], marker)
        np.testing.assert_array_equal(thruster.inertialPositionProperty, marker)
    stop_time = 0.02  # [s]
    bundle.simulation.ConfigureStopTime(macros.sec2nano(stop_time))
    bundle.simulation.ExecuteSimulation()
    return np.concatenate([
        np.asarray(manager.getStateObject(getattr(panel, name)).getState()).ravel()
        for panel in bundle.panels for name in NAME_FIELDS[:2]
    ])


@pytest.mark.parametrize("custom_names", [False, True])
def test_repeated_builds(manager_local, custom_names):
    """Sequential simulations retain physical results and predictable policy-specific names."""
    reference = None
    for _ in range(3):
        bundle = build_simulation(manager_local, custom_names)
        result = run_simulation(bundle, manager_local, custom_names)
        if reference is None:
            reference = result
        np.testing.assert_array_equal(result, reference)
        del bundle
        gc.collect()


@pytest.mark.parametrize("collect_before_build", [False, True])
def test_overlapping_lifetimes_and_garbage_collection(manager_local, collect_before_build):
    """A cyclic old simulation and a live simulation cannot change a new simulation's names."""
    gc_was_enabled = gc.isenabled()
    gc.disable()
    try:
        live = build_simulation(manager_local)
        reference = run_simulation(live, manager_local)
        delayed = build_simulation(manager_local)
        run_simulation(delayed, manager_local)
        delayed.cycle = delayed
        old_reference = weakref.ref(delayed)
        del delayed
        assert old_reference() is not None
        if collect_before_build:
            gc.collect()
            assert old_reference() is None
        current = build_simulation(manager_local)
        np.testing.assert_array_equal(run_simulation(current, manager_local), reference)
        gc.collect()
        assert old_reference() is None
        # Equal resolved strings in two live managers still address separate storage.
        current_name = current.panels[0].nameOfInertialPositionProperty
        live_name = live.panels[0].nameOfInertialPositionProperty
        previous_live = np.array(live.vehicle.dynManager.getPropertyReference(live_name))
        marker = np.array([[901.0], [902.0], [903.0]])  # [m]
        current.vehicle.dynManager.setPropertyValue(current_name, marker)
        np.testing.assert_array_equal(current.thrusters[0].inertialPositionProperty, marker)
        np.testing.assert_array_equal(live.thrusters[0].inertialPositionProperty, previous_live)
    finally:
        if gc_was_enabled:
            gc.enable()
        gc.collect()


@pytest.mark.parametrize("field", NAME_FIELDS)
def test_explicit_constructor_name_is_preserved(naming_support, field):
    """Assignment of the current auto-looking string still records an explicit override."""
    bundle = build_simulation(True)
    panel = bundle.panels[0]
    original = getattr(panel, field)
    setattr(panel, field, original)
    bundle.simulation.InitializeSimulation()
    assert getattr(panel, field) == original


def test_new_and_legacy_physics_agree(naming_support):
    """Changing name allocation does not change the integrated panel states."""
    legacy = build_simulation(False)
    local = build_simulation(True)
    np.testing.assert_array_equal(run_simulation(legacy, False), run_simulation(local, True))


def test_deprecated_spacecraft_system_rejects_new_policy(naming_support):
    """The gated policy fails explicitly in an architecture that has not been migrated."""
    from Basilisk.utilities import deprecated

    with pytest.warns((deprecated.BSKDeprecationWarning, deprecated.BSKUrgentDeprecationWarning)):
        system = spacecraftSystem.SpacecraftSystem()
    naming_support.enableManagerLocalNaming(system.dynManager)
    with pytest.raises(bskLogging.BasiliskError, match="supported only by Spacecraft"):
        system.initializeDynamics()


@pytest.mark.parametrize("kind", ["drag", "srp"])
def test_environmental_effectors_bind_resolved_properties(manager_local, kind):
    """Loads respond to the panel properties like an independently linked reference effector."""
    bundle = build_simulation(manager_local, custom_names=True)
    area = 2.0  # [m^2]
    coefficient = 2.0  # [-]
    location = [0.0, 1.0, 0.0]  # [m]
    normal = [1.0, 0.0, 0.0]
    if kind == "drag":
        child = facetDragDynamicEffector.FacetDragDynamicEffector()
        reference = facetDragDynamicEffector.FacetDragDynamicEffector()
        payload = messaging.AtmoPropsMsgPayload()
        payload.neutralDensity = 1.0e-6  # [kg/m^3]
        message = messaging.AtmoPropsMsg().write(payload)
        for effector in (child, reference):
            effector.atmoDensInMsg.subscribeTo(message)
            effector.addFacet(area, coefficient, normal, location)
    else:
        child = facetSRPDynamicEffector.FacetSRPDynamicEffector()
        reference = facetSRPDynamicEffector.FacetSRPDynamicEffector()
        payload = messaging.SpicePlanetStateMsgPayload()
        payload.PositionVector = [1.0e11, 0.0, 0.0]  # [m]
        message = messaging.SpicePlanetStateMsg().write(payload)
        for effector in (child, reference):
            effector.sunInMsg.subscribeTo(message)
            effector.setNumFacets(1)
            effector.addFacet(area, np.eye(3), normal, normal, location, 0.0, 0.0)
    panel = bundle.panels[0]
    if manager_local:
        panel.addDynamicEffector(child)
        panel.nameOfInertialAttitudeProperty = "lateCustomAttitude"
    if not manager_local:
        panel.addDynamicEffector(child)
    bundle.simulation.InitializeSimulation()
    marker = np.array([[0.1], [0.2], [0.3]])  # [-] MRP coefficients
    bundle.vehicle.dynManager.setPropertyValue(panel.nameOfInertialAttitudeProperty, marker)
    velocity = np.array([[100.0], [0.0], [0.0]])  # [m/s]
    bundle.vehicle.dynManager.setPropertyValue(panel.nameOfInertialVelocityProperty, velocity)
    reference.setPropName_inertialAttitude(panel.nameOfInertialAttitudeProperty)
    reference.setPropName_inertialPosition(panel.nameOfInertialPositionProperty)
    reference.setPropName_inertialVelocity(panel.nameOfInertialVelocityProperty)
    reference.linkInProperties(bundle.vehicle.dynManager)
    for effector in (child, reference):
        effector.UpdateState(0)  # [ns]
        effector.computeForceTorque(0.0, 0.01)  # [s]
    assert np.linalg.norm(reference.forceExternal_B) > 0.0
    np.testing.assert_array_equal(child.forceExternal_B, reference.forceExternal_B)
    np.testing.assert_array_equal(child.torqueExternalPntB_B, reference.torqueExternalPntB_B)


@pytest.mark.parametrize("reverse_attachment", [False, True])
def test_constraint_tracks_both_panel_parents(naming_support, reverse_attachment):
    """A shared constraint refreshes each original attachment without appending parents."""
    bundle = build_simulation(True)
    constraint = constraintDynamicEffector.ConstraintDynamicEffector()
    constraint.setAlpha(1.0)  # [1/s]
    constraint.setBeta(1.0)  # [1/s]
    order = list(reversed(bundle.panels)) if reverse_attachment else bundle.panels
    for panel in order:
        panel.addDynamicEffector(constraint)
    bundle.panels[0].nameOfInertialPositionProperty = "firstPosition"
    bundle.panels[1].nameOfInertialPositionProperty = "secondPosition"
    bundle.simulation.InitializeSimulation()
    assert len(constraint.getPropName_inertialPosition()) == 2
    assert len(constraint.getPropName_inertialAttitude()) == 2
    # Repeating preparation must refresh these attachments without adding parents.
    bundle.vehicle.initializeDynamics()
    assert len(constraint.getPropName_inertialPosition()) == 2
    for index, panel in enumerate(bundle.panels):
        marker = np.array([[index + 201.0], [2.0], [3.0]])  # [m]
        bundle.vehicle.dynManager.setPropertyValue(panel.nameOfInertialPositionProperty, marker)
        for field in NAME_FIELDS[3:]:
            bundle.vehicle.dynManager.setPropertyValue(getattr(panel, field), np.zeros((3, 1)))
    # Two evaluations complete a parent pair and recompute the spring force.
    constraint.computeForceTorque(0.0, 0.01)  # [s]
    constraint.computeForceTorque(0.0, 0.01)  # [s]
    expected_force = [[1.0], [0.0], [0.0]]  # [N]
    np.testing.assert_array_equal(constraint.forceExternal_N, expected_force)


@pytest.mark.parametrize("hub_attached_first", [False, True])
@pytest.mark.parametrize("hub_initialized_first", [False, True])
def test_constraint_with_a_hub_parent(naming_support, hub_attached_first, hub_initialized_first):
    """A shared panel/hub constraint retains its slots across either setup and binding order."""
    bundle = build_simulation(True)
    panel = bundle.panels[0]
    hub = spacecraft.Spacecraft()
    hub.hub.mHub = 100.0  # [kg]
    hub.hub.IHubPntBc_B = np.eye(3) * 100.0  # [kg m^2]
    constraint = constraintDynamicEffector.ConstraintDynamicEffector()
    constraint.setAlpha(1.0)  # [1/s]
    constraint.setBeta(1.0)  # [1/s]
    parents = [hub, panel] if hub_attached_first else [panel, hub]
    for parent in parents:
        parent.addDynamicEffector(constraint)
    panel.nameOfInertialPositionProperty = "latePanelPosition"
    if hub_initialized_first:
        hub.initializeDynamics()
    bundle.simulation.InitializeSimulation()
    if not hub_initialized_first:
        hub.initializeDynamics()
    marker = np.array([[1.0], [0.0], [0.0]])  # [m]
    bundle.vehicle.dynManager.setPropertyValue(panel.nameOfInertialPositionProperty, marker)
    for field in NAME_FIELDS[3:]:
        bundle.vehicle.dynManager.setPropertyValue(getattr(panel, field), np.zeros((3, 1)))
    constraint.computeForceTorque(0.0, 0.01)  # [s]
    constraint.computeForceTorque(0.0, 0.01)  # [s]
    # The magnitude detects stale panel positions regardless of the next parent in the force cycle.
    np.testing.assert_array_equal(np.abs(constraint.forceExternal_N), marker)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
