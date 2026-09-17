# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Check multibody names and nested attachments through the Python interfaces."""

import gc
import weakref
from dataclasses import dataclass

import numpy as np
import pytest

from Basilisk.architecture import bskLogging
from Basilisk.simulation import (
    dualHingedRigidBodyStateEffector as dual,
    nHingedRigidBodyStateEffector as chain,
    linearTranslationOneDOFStateEffector as slide,
    linearTranslationNDOFStateEffector as slides,
    prescribedMotionStateEffector as prescribed,
    spinningBodyOneDOFStateEffector as spin,
    spinningBodyTwoDOFStateEffector as gimbal,
    spinningBodyNDOFStateEffector as spins,
    spacecraft,
    thrusterDynamicEffector,
)
from Basilisk.utilities import SimulationBaseClass, macros

KINDS = ("spin", "gimbal", "spins", "slide", "slides", "dual", "chain", "prescribed")
PROPERTIES = tuple(f"nameOfInertial{part}Property" for part in ("Position", "Velocity", "Attitude", "AngVelocity"))
DYNAMIC_GETTERS = tuple(f"getPropName_inertial{part}" for part in ("Position", "Velocity", "Attitude", "AngVelocity"))


@dataclass
class Model:
    """Retain an effector, its bodies, and the dynamic effectors attached to each segment."""

    kind: str
    effector: object
    bodies: list
    children: list
    fields: tuple


def make_model(kind, label, custom=False):
    """Configure a nonsingular model and attach a dynamic effector to each body."""
    factories = {
        "spin": spin.SpinningBodyOneDOFStateEffector,
        "gimbal": gimbal.SpinningBodyTwoDOFStateEffector,
        "spins": spins.SpinningBodyNDOFStateEffector,
        "slide": slide.LinearTranslationOneDOFStateEffector,
        "slides": slides.LinearTranslationNDOFStateEffector,
        "dual": dual.DualHingedRigidBodyStateEffector,
        "chain": chain.NHingedRigidBodyStateEffector,
        "prescribed": prescribed.PrescribedMotionStateEffector,
    }
    effector = factories[kind]()
    effector.ModelTag = label
    bodies = []
    if kind == "spin":
        effector.mass = 1.0  # [kg]
        effector.sHat_S = [1.0, 0.0, 0.0]
        effector.thetaInit = 0.15  # [rad]
        effector.thetaDotInit = 0.02  # [rad/s]
    elif kind == "gimbal":
        effector.mass1 = 1.0  # [kg]
        effector.mass2 = 1.0  # [kg]
        effector.s1Hat_S1 = [1.0, 0.0, 0.0]
        effector.s2Hat_S2 = [0.0, 1.0, 0.0]
        effector.theta1Init = 0.15  # [rad]
        effector.theta2DotInit = 0.02  # [rad/s]
    elif kind == "slide":
        effector.setMass(1.0)  # [kg]
        effector.setRhoInit(0.15)  # [m]
        effector.setRhoDotInit(0.02)  # [m/s]
    elif kind == "dual":
        effector.mass1 = 1.0  # [kg]
        effector.mass2 = 1.0  # [kg]
        effector.theta1Init = 0.15  # [rad]
        effector.theta2DotInit = 0.02  # [rad/s]
    elif kind == "prescribed":
        effector.setMass(1.0)  # [kg]
        effector.setR_PM_M([0.2, 0.0, 0.0])  # [m]
        effector.setOmega_PM_P([0.0, 0.01, 0.0])  # [rad/s]
    if kind in ("spins", "slides", "chain"):
        for index in range(2):
            if kind == "spins":
                body = spins.SpinningBody()
                body.setThetaInit(0.1 * (index + 1))  # [rad]
            elif kind == "slides":
                body = slides.TranslatingBody()
                body.setMass(1.0)  # [kg]
                body.setRhoInit(0.1 * (index + 1))  # [m]
            else:
                body = chain.HingedPanel()
                body.IPntS_S = np.eye(3)  # [kg m^2]
                body.thetaInit = 0.1 * (index + 1)  # [rad]
            if custom:
                for field in PROPERTIES:
                    setattr(body, field, f"{label}_body{index}_{field}")
            if kind == "spins":
                effector.addSpinningBody(body)
            elif kind == "slides":
                effector.addTranslatingBody(body)
            else:
                effector.addHingedPanel(body)
            bodies.append(body)
    fields = tuple(
        field for field in dir(effector)
        if field.startswith("nameOf") and field != "nameOfSpacecraftAttachedTo"
    )
    if custom:
        for field in fields:
            setattr(effector, field, f"{label}_{field}")
    children = []
    count = 0 if kind == "prescribed" else 2 if kind in ("gimbal", "spins", "slides", "dual", "chain") else 1
    for segment in range(1, count + 1):
        child = thrusterDynamicEffector.ThrusterDynamicEffector()
        effector.addDynamicEffector(child, segment)
        children.append(child)
    return Model(kind, effector, bodies, children, fields)


def state_names(model):
    """Read the integrated state names through the preserved Python attributes."""
    return tuple(getattr(model.effector, field) for field in model.fields if field.endswith("State"))


def property_names(model):
    """Read all per-body properties through their dependent dynamic effectors."""
    if model.kind == "prescribed":
        return tuple(getattr(model.effector, field) for field in model.fields if "Property" in field)
    return tuple(getattr(child, getter)() for child in model.children for getter in DYNAMIC_GETTERS)


@dataclass
class Simulation:
    """Keep Python owners alive until all simulation workers have stopped."""

    simulation: object
    vehicle: object
    models: list


def build_simulation(local, custom=False, nested=False):
    """Build two mixed model banks, with optional children under prescribed frames."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    step = 0.01  # [s]
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(step)))
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.hub.IHubPntBc_B = 100.0 * np.eye(3)  # [kg m^2]
    sim.AddModelToTask("task", vehicle)
    if local:
        vehicle.dynManager.useManagerLocalEffectorNames = True
    models = []
    for bank in reversed(range(2)):
        for kind in KINDS:
            model = make_model(kind, f"bank{bank}_{kind}", custom)
            vehicle.addStateEffector(model.effector)
            models.append(model)
            if nested and kind == "prescribed":
                for child_kind in ("spin", "gimbal", "slide"):
                    child = make_model(child_kind, f"bank{bank}_nested_{child_kind}", custom)
                    model.effector.addStateEffector(child.effector)
                    models.append(child)
    for model in models:
        sim.AddModelToTask("task", model.effector)
        for child in model.children:
            sim.AddModelToTask("task", child)
    return Simulation(sim, vehicle, models)


def run_simulation(bundle):
    """Verify unique registered names and integrate the coupled multibody system."""
    bundle.simulation.InitializeSimulation()
    states = [name for model in bundle.models for name in state_names(model)]
    properties = [name for model in bundle.models for name in property_names(model)]
    assert len(states) == len(set(states))
    assert len(properties) == len(set(properties))
    for name in states:
        assert bundle.vehicle.dynManager.getStateObject(name).getName() == name
    for name in properties:
        assert np.asarray(bundle.vehicle.dynManager.getPropertyReference(name)).shape == (3, 1)
    for model_index, model in enumerate(bundle.models):
        for segment, child in enumerate(model.children):
            name = child.getPropName_inertialPosition()
            original = np.array(bundle.vehicle.dynManager.getPropertyReference(name))
            marker = np.array([[100.0 + model_index], [float(segment)], [3.0]])  # [m]
            bundle.vehicle.dynManager.setPropertyValue(name, marker)
            np.testing.assert_array_equal(child.inertialPositionProperty, marker)
            bundle.vehicle.dynManager.setPropertyValue(name, original)
    stop = 0.02  # [s]
    bundle.simulation.ConfigureStopTime(macros.sec2nano(stop))
    bundle.simulation.ExecuteSimulation()
    result = np.concatenate([
        np.asarray(bundle.vehicle.dynManager.getStateObject(name).getState()).ravel() for name in states
    ])
    assert np.all(np.isfinite(result))
    return result, (states, properties)


@pytest.mark.parametrize("nested", [False, True])
@pytest.mark.parametrize("custom", [False, True])
def test_multibody_lifetimes(nested, custom):
    """Names and dynamics repeat across overlapping lifetimes and cyclic garbage collection."""
    live = build_simulation(True, custom, nested)
    reference, reference_names = run_simulation(live)
    delayed = build_simulation(True, custom, nested)
    result, names = run_simulation(delayed)
    np.testing.assert_allclose(result, reference, rtol=0.0, atol=1e-14)
    assert names == reference_names
    delayed.cycle = delayed
    ref = weakref.ref(delayed)
    del delayed
    gc.collect()
    assert ref() is None
    fresh = build_simulation(True, custom, nested)
    result, names = run_simulation(fresh)
    np.testing.assert_allclose(result, reference, rtol=0.0, atol=1e-14)
    assert names == reference_names


@pytest.mark.parametrize("nested", [False, True])
def test_multibody_physics_matches_legacy(nested):
    """Explicit state names isolate the naming policy when comparing complete trajectories."""
    legacy = build_simulation(False, True, nested)
    local = build_simulation(True, True, nested)
    expected, _ = run_simulation(legacy)
    actual, _ = run_simulation(local)
    np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-14)


@pytest.mark.parametrize("kind", KINDS)
def test_every_python_name_tracks_explicit_assignments(kind):
    """All scalar name attributes distinguish explicit constructor-looking assignments."""
    unused = make_model(kind, "unused")
    for field in unused.fields:
        model = make_model(kind, "explicit")
        original = getattr(model.effector, field)
        setattr(model.effector, field, original)
        vehicle = spacecraft.Spacecraft()
        vehicle.hub.mHub = 100.0  # [kg]
        vehicle.hub.IHubPntBc_B = np.eye(3)  # [kg m^2]
        vehicle.dynManager.useManagerLocalEffectorNames = True
        vehicle.addStateEffector(model.effector)
        vehicle.initializeDynamics()
        assert getattr(model.effector, field) == original
        with pytest.raises(bskLogging.BasiliskError, match="resolved names cannot be changed"):
            setattr(model.effector, field, "tooLate")


@pytest.mark.parametrize("kind", ["spins", "slides", "chain"])
def test_body_property_custom_names(kind, manager_local):
    """Custom names captured after body attachment remain valid under both policies."""
    model = make_model(kind, "bodyNames", True)
    captured_names = property_names(model)
    expected_names = tuple(
        f"bodyNames_body{index}_{field}"
        for index in range(len(model.bodies)) for field in PROPERTIES
    )
    assert captured_names == expected_names
    if kind != "chain":
        assert tuple(getattr(body, field) for body in model.bodies for field in PROPERTIES) == expected_names
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.hub.IHubPntBc_B = np.eye(3)  # [kg m^2]
    vehicle.dynManager.useManagerLocalEffectorNames = manager_local
    vehicle.addStateEffector(model.effector)
    vehicle.initializeDynamics()
    assert property_names(model) == captured_names
    for name in captured_names:
        assert np.asarray(vehicle.dynManager.getPropertyReference(name)).shape == (3, 1)
    if manager_local and kind != "chain":
        with pytest.raises(bskLogging.BasiliskError, match="resolved names cannot be changed"):
            model.bodies[0].nameOfInertialPositionProperty = "tooLate"


@pytest.mark.parametrize("field", ["nameOfThetaState", "nameOfInertialPositionProperty"])
def test_cross_type_custom_collisions_fail_before_registration(field):
    """A collision between a spinner and a prescribed body publishes no effector states."""
    spinner = make_model("spin", "spinner")
    parent = make_model("prescribed", "parent")
    other_field = "nameOfsigma_PMState" if field.endswith("State") else field
    setattr(spinner.effector, field, "collision")
    setattr(parent.effector, other_field, "collision")
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.dynManager.useManagerLocalEffectorNames = True
    vehicle.addStateEffector(spinner.effector)
    vehicle.addStateEffector(parent.effector)
    with pytest.raises(bskLogging.BasiliskError, match="already in use"):
        vehicle.initializeDynamics()
    setattr(parent.effector, other_field, "corrected")
    vehicle.initializeDynamics()


@pytest.mark.parametrize("also_root", [False, True])
def test_shared_nested_children_rejected(also_root):
    """Reject a child used by two parents or both a parent and the spacecraft."""
    first = make_model("prescribed", "first")
    second = make_model("prescribed", "second")
    child = make_model("spin", "child")
    first.effector.addStateEffector(child.effector)
    vehicle = spacecraft.Spacecraft()
    vehicle.dynManager.useManagerLocalEffectorNames = True
    vehicle.addStateEffector(first.effector)
    if also_root:
        vehicle.addStateEffector(child.effector)
    else:
        second.effector.addStateEffector(child.effector)
        vehicle.addStateEffector(second.effector)
    with pytest.raises(bskLogging.BasiliskError, match="repeated nested"):
        vehicle.initializeDynamics()


@pytest.mark.parametrize("kind", ["spin", "gimbal", "slide", "dual", "spins", "slides"])
def test_rename_after_dynamic_attachment(kind):
    """Refresh dependent property names after a custom assignment made during setup."""
    model = make_model(kind, "renamed")
    if model.bodies:
        model.bodies[1].nameOfInertialPositionProperty = "renamedPosition"
    elif kind in ("gimbal", "dual"):
        model.effector.nameOfInertialPositionProperty2 = "renamedPosition"
    else:
        model.effector.nameOfInertialPositionProperty = "renamedPosition"
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.dynManager.useManagerLocalEffectorNames = True
    vehicle.addStateEffector(model.effector)
    vehicle.initializeDynamics()
    child = model.children[-1]
    assert child.getPropName_inertialPosition() == "renamedPosition"
    marker = np.array([[1.0], [2.0], [3.0]])  # [m]
    vehicle.dynManager.setPropertyValue("renamedPosition", marker)
    np.testing.assert_array_equal(child.inertialPositionProperty, marker)


@pytest.mark.parametrize("kind", ["spins", "slides", "chain"])
def test_body_addition_after_registration_rejected(kind):
    """Freeze the body count once state dimensions and per-body names are registered."""
    model = make_model(kind, "fixedBodies")
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.dynManager.useManagerLocalEffectorNames = True
    vehicle.addStateEffector(model.effector)
    vehicle.initializeDynamics()
    add_body = {
        "spins": "addSpinningBody",
        "slides": "addTranslatingBody",
        "chain": "addHingedPanel",
    }[kind]
    with pytest.raises(bskLogging.BasiliskError, match="bodies cannot change"):
        getattr(model.effector, add_body)(model.bodies[0])


@pytest.mark.parametrize("kind", ["spins", "slides"])
def test_duplicate_body_properties_rejected(kind):
    """Reject two bodies claiming the same custom property within one effector."""
    model = make_model(kind, "duplicateBodies")
    for body in model.bodies:
        body.nameOfInertialPositionProperty = "duplicatePosition"
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.dynManager.useManagerLocalEffectorNames = True
    vehicle.addStateEffector(model.effector)
    with pytest.raises(bskLogging.BasiliskError, match="already in use"):
        vehicle.initializeDynamics()


@pytest.mark.parametrize("kind", ["spins", "slides"])
def test_shared_body_cannot_change_registered_parent_names(kind):
    """Reject sharing a registered body while preserving its original property bindings."""
    first = make_model(kind, "firstOwner")
    second = make_model(kind, "secondOwner")
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.dynManager.useManagerLocalEffectorNames = True
    vehicle.addStateEffector(first.effector)
    vehicle.initializeDynamics()
    original = first.bodies[0].nameOfInertialPositionProperty
    add_body = "addSpinningBody" if kind == "spins" else "addTranslatingBody"
    with pytest.raises(bskLogging.BasiliskError, match="live naming owner"):
        getattr(second.effector, add_body)(first.bodies[0])
    assert first.bodies[0].nameOfInertialPositionProperty == original
    assert first.children[0].getPropName_inertialPosition() == original
