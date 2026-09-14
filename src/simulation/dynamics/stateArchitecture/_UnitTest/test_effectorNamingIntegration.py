# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Exercise naming preparation across spacecraft, attachment trees, and shared managers."""

from dataclasses import dataclass

import numpy as np
import pytest

from Basilisk.architecture import bskLogging
from Basilisk.simulation import (
    constraintDynamicEffector,
    hingedRigidBodyStateEffector,
    prescribedMotionStateEffector,
    spacecraft,
    spacecraftSystem,
    spinningBodyOneDOFStateEffector,
    thrusterDynamicEffector,
)
from Basilisk.utilities import SimulationBaseClass, deprecated, macros


@dataclass
class Tree:
    """Retain the Python owners of every node until simulation workers stop."""

    vehicle: object
    roots: list
    leaves: list
    loads: list

    def state_names(self):
        """Return state names in physical attachment order."""
        return [name for root, leaf in zip(self.roots, self.leaves) for name in (
            root.nameOfsigma_PMState, leaf.nameOfThetaState, leaf.nameOfThetaDotState
        )]

    def values(self):
        """Read integrated states without depending on alphabetical map order."""
        return np.concatenate([
            np.asarray(self.vehicle.dynManager.getStateObject(name).getState()).ravel()
            for name in self.state_names()
        ])


def make_tree(local):
    """Construct in reverse order and reserve automatic candidates from a later branch."""
    leaves = [spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector() for _ in range(2)][::-1]
    roots = [prescribedMotionStateEffector.PrescribedMotionStateEffector() for _ in range(2)][::-1]
    loads = [thrusterDynamicEffector.ThrusterDynamicEffector() for _ in range(2)]
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.hub.IHubPntBc_B = 100.0 * np.eye(3)  # [kg m^2]
    if local:
        import effectorNamingTestSupport

        effectorNamingTestSupport.enableManagerLocalNaming(vehicle.dynManager)
        leaves[1].nameOfThetaState = "prescribedObjectspinningBodyTheta1"
        leaves[1].nameOfInertialPositionProperty = "spinningBodyInertialPosition1"
        roots[1].nameOfPrescribedPositionProperty = "prescribedObjectPosition1"
    for index, (root, leaf, load) in enumerate(zip(roots, leaves, loads)):
        root.setR_PM_M([0.3 * (index + 1), 0.0, 0.0])  # [m]
        leaf.mass = 1.0  # [kg]
        leaf.sHat_S = [1.0, 0.0, 0.0]
        leaf.thetaInit = 0.125 * (index + 1)  # [rad]
        leaf.thetaDotInit = 0.02  # [rad/s]
        leaf.addDynamicEffector(load)
        root.addStateEffector(leaf)
        vehicle.addStateEffector(root)
    return Tree(vehicle, roots, leaves, loads)


@pytest.mark.parametrize("reverse_schedule", [False, True])
def test_independent_spacecraft_repeat_preparation_in_one_simulation(manager_local, reverse_schedule):
    """Independent managers isolate names and bindings through repeated initialization."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    step = 0.01  # [s]
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(step)))
    trees = [make_tree(manager_local) for _ in range(2)][::-1]
    schedule = trees[::-1] if reverse_schedule else trees
    for tree in schedule:
        sim.AddModelToTask("task", tree.vehicle)
        for model in tree.roots + tree.leaves + tree.loads:
            sim.AddModelToTask("task", model)
    sim.InitializeSimulation()
    names = [tree.state_names() for tree in trees]
    initial = [tree.values() for tree in trees]
    if manager_local:
        assert names[0] == names[1]
        for tree in trees:
            assert tree.leaves[0].nameOfThetaState == "prescribedObjectspinningBodyTheta2"
            assert tree.leaves[1].nameOfThetaState == "prescribedObjectspinningBodyTheta1"
            assert tree.roots[0].nameOfPrescribedPositionProperty == "prescribedObjectPosition2"
    sim.InitializeSimulation()
    for tree, original_names, original_values in zip(trees, names, initial):
        assert tree.state_names() == original_names
        np.testing.assert_array_equal(tree.values(), original_values)
    # Identical property names in independent managers must still address different storage.
    first_name = trees[0].loads[0].getPropName_inertialPosition()
    second_name = trees[1].loads[0].getPropName_inertialPosition()
    first_manager = trees[0].vehicle.dynManager
    second_manager = trees[1].vehicle.dynManager
    first_original = np.array(first_manager.getPropertyReference(first_name))
    second_original = np.array(second_manager.getPropertyReference(second_name))
    marker = np.array([[3.0], [4.0], [5.0]])  # [m]
    first_manager.setPropertyValue(first_name, marker)
    np.testing.assert_array_equal(trees[0].loads[0].inertialPositionProperty, marker)
    np.testing.assert_array_equal(trees[1].loads[0].inertialPositionProperty, second_original)
    first_manager.setPropertyValue(first_name, first_original)
    stop = 0.02  # [s]
    sim.ConfigureStopTime(macros.sec2nano(stop))
    sim.ExecuteSimulation()
    np.testing.assert_allclose(trees[0].values(), trees[1].values(), rtol=0.0, atol=1e-14)
    assert np.all(np.isfinite(trees[0].values()))


@pytest.mark.parametrize("reverse_attachment", [False, True])
@pytest.mark.parametrize("reverse_initialization", [False, True])
def test_constraint_keeps_equal_names_in_independent_managers_distinct(
    naming_support, reverse_attachment, reverse_initialization
):
    """A cross-spacecraft constraint binds each parent's own storage in either order."""
    trees = [make_tree(True) for _ in range(2)]
    constraint = constraintDynamicEffector.ConstraintDynamicEffector()
    constraint.setAlpha(1.0)  # [1/s]
    constraint.setBeta(1.0)  # [1/s]
    parents = trees[::-1] if reverse_attachment else trees
    for tree in parents:
        tree.leaves[0].addDynamicEffector(constraint)
    initialization_order = trees[::-1] if reverse_initialization else trees
    for _ in range(2):
        for tree in initialization_order:
            tree.vehicle.initializeDynamics()
    assert len(constraint.getPropName_inertialPosition()) == 2
    assert len(set(constraint.getPropName_inertialPosition())) == 1
    for index, tree in enumerate(trees):
        manager = tree.vehicle.dynManager
        leaf = tree.leaves[0]
        position = np.array([[4.0 * index], [0.0], [0.0]])  # [m]
        manager.setPropertyValue(leaf.nameOfInertialPositionProperty, position)
        for part in ("Velocity", "Attitude", "AngVelocity"):
            manager.setPropertyValue(getattr(leaf, f"nameOfInertial{part}Property"), np.zeros((3, 1)))
    constraint.computeForceTorque(0.0, 0.01)  # [s]
    constraint.computeForceTorque(0.0, 0.01)  # [s]
    np.testing.assert_allclose(np.abs(constraint.forceExternal_N), [[4.0], [0.0], [0.0]], atol=1e-14)  # [N]


def make_legacy_system():
    """Build docked and undocked units sharing the deprecated system's manager."""
    with pytest.warns((deprecated.BSKDeprecationWarning, deprecated.BSKUrgentDeprecationWarning)):
        system = spacecraftSystem.SpacecraftSystem()
    free = spacecraftSystem.SpacecraftUnit()
    docked = spacecraftSystem.SpacecraftUnit()
    units = [system.primaryCentralSpacecraft, docked, free]
    panels = [hingedRigidBodyStateEffector.HingedRigidBodyStateEffector() for _ in units][::-1]
    loads = [thrusterDynamicEffector.ThrusterDynamicEffector() for _ in units]
    for index, (unit, panel, load) in enumerate(zip(units, panels, loads)):
        unit.spacecraftName = f"vehicle{index}"
        unit.hub.mHub = 100.0  # [kg]
        unit.hub.IHubPntBc_B = 100.0 * np.eye(3)  # [kg m^2]
        panel.mass = 1.0  # [kg]
        panel.IPntS_S = np.eye(3)  # [kg m^2]
        panel.thetaInit = 0.125 * (index + 1)  # [rad]
        panel.nameOfThetaState = "customAngle"
        panel.addDynamicEffector(load)
        unit.addStateEffector(panel)
    ports = [spacecraftSystem.DockingData(), spacecraftSystem.DockingData()]
    for unit, port, name in zip(units, ports, ("primaryPort", "dockedPort")):
        port.portName = name
        unit.addDockingPort(port)
    system.attachSpacecraftToPrimary(docked, "dockedPort", "primaryPort")
    system.addSpacecraftUndocked(free)
    return system, units, panels, loads, ports


def test_deprecated_system_preserves_legacy_names_in_its_shared_manager():
    """All units keep distinct legacy names and bindings through repeated initialization."""
    system, units, panels, loads, ports = make_legacy_system()
    system.initializeDynamics()
    names = [panel.nameOfThetaState for panel in panels]
    hub_names = [unit.hub.nameOfHubPosition for unit in units]
    property_names = [load.getPropName_inertialPosition() for load in loads]
    assert names == [f"vehicle{index}customAngle" for index in range(3)]
    assert len(set(load.getPropName_inertialPosition() for load in loads)) == 3
    system.initializeDynamics()
    assert names == [panel.nameOfThetaState for panel in panels]
    assert hub_names == [unit.hub.nameOfHubPosition for unit in units]
    assert property_names == [load.getPropName_inertialPosition() for load in loads]
    for panel, name in zip(panels, names):
        assert system.dynManager.getStateObject(name).getState()[0][0] == panel.thetaInit
    for index, load in enumerate(loads):
        marker = np.array([[float(index)], [2.0], [3.0]])  # [m]
        system.dynManager.setPropertyValue(load.getPropName_inertialPosition(), marker)
        np.testing.assert_array_equal(load.inertialPositionProperty, marker)


def test_deprecated_unit_rejects_new_policy_before_registering(naming_support):
    """Direct initialization cannot bypass the deprecated system's legacy-only boundary."""
    system, units, panels, loads, ports = make_legacy_system()
    naming_support.enableManagerLocalNaming(system.dynManager)
    with pytest.raises(bskLogging.BasiliskError, match="legacy naming"):
        units[0].initializeDynamicsSC(system.dynManager)


def test_deprecated_unit_rejects_renaming_after_preparation():
    """Changing a unit's prefix cannot leave existing state bindings under stale names."""
    system, units, panels, loads, ports = make_legacy_system()
    system.initializeDynamics()
    original = units[0].hub.nameOfHubPosition
    units[0].spacecraftName = "renamed"
    with pytest.raises(bskLogging.BasiliskError, match="rebuild the unit"):
        units[0].initializeDynamicsSC(system.dynManager)
    assert units[0].hub.nameOfHubPosition == original


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
