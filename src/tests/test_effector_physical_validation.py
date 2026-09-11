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

"""Check mass and hinge-frame validation through both initialization paths."""

import numpy as np
import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import (
    dualHingedRigidBodyStateEffector,
    hingedRigidBodyStateEffector,
    linearSpringMassDamper,
    nHingedRigidBodyStateEffector,
    prescribedMotionStateEffector,
    spacecraft,
    sphericalPendulum,
)
from Basilisk.utilities import SimulationBaseClass, macros


HINGED_MODELS = ["single", "dual", "chain"]
SLOSH_MODELS = ["linear", "spherical"]
PUBLIC_MASS_MODELS = HINGED_MODELS + SLOSH_MODELS
ALL_MODELS = PUBLIC_MASS_MODELS + ["prescribed"]
INVALID_MASSES = [-1.0, np.nan, np.inf, -np.inf]  # [kg]
ROTATION = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]  # [-]


def make_effector(kind, mass=1.0, second_mass=1.0, second_distance=1.0):  # [kg], [kg], [m]
    """Create a valid model with nonsingular dynamics for initialization tests."""
    constructors = {
        "single": hingedRigidBodyStateEffector.HingedRigidBodyStateEffector,
        "dual": dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector,
        "chain": nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector,
        "linear": linearSpringMassDamper.LinearSpringMassDamper,
        "spherical": sphericalPendulum.SphericalPendulum,
        "prescribed": prescribedMotionStateEffector.PrescribedMotionStateEffector,
    }
    effector = constructors[kind]()
    if kind == "chain":
        for panel_mass, panel_distance in [(mass, 1.0), (second_mass, second_distance)]:  # [kg], [m]
            panel = nHingedRigidBodyStateEffector.HingedPanel()
            panel.mass = panel_mass
            panel.d = panel_distance
            panel.IPntS_S = np.eye(3).tolist()  # [kg*m^2]
            effector.addHingedPanel(panel)
    else:
        set_mass(effector, kind, mass)
    if kind == "dual":
        effector.mass2 = second_mass
    elif kind == "spherical":
        effector.pendulumRadius = 1.0  # [m]
    return effector


def set_mass(effector, kind, mass):
    """Set the selected model's configured mass in kilograms."""
    if kind == "prescribed":
        effector.setMass(mass)
    else:
        field = {"single": "mass", "dual": "mass1"}.get(kind, "massInit")
        setattr(effector, field, mass)


def make_simulation(effector, priority=None, parent=None):
    """Attach the effector, optionally also scheduling it or nesting it under a parent."""
    simulation = SimulationBaseClass.SimBaseClass()
    simulation.SetProgressBar(False)
    process = simulation.CreateNewProcess("process")
    process.addTask(simulation.CreateNewTask("task", macros.sec2nano(0.01)))  # [s]
    body = spacecraft.Spacecraft()
    body.hub.mHub = 100.0  # [kg]
    body.hub.IHubPntBc_B = (10.0 * np.eye(3)).tolist()  # [kg*m^2]
    if parent is None:
        body.addStateEffector(effector)
    else:
        parent.addStateEffector(effector)
        body.addStateEffector(parent)
    simulation.AddModelToTask("task", body)
    if priority is not None:
        simulation.AddModelToTask("task", effector, priority)
    return simulation, body


def validate(effector, path):
    """Exercise standalone Reset or initialization without scheduling the effector."""
    if path == "reset":
        effector.Reset(0)
    else:
        simulation, body = make_simulation(effector)
        simulation.InitializeSimulation()


def state_names(effector):
    """Return public integrated-state names for hinged and slosh models."""
    return [getattr(effector, name) for name in dir(effector)
            if name.startswith("nameOf") and name.endswith("State")]


@pytest.mark.parametrize("kind", PUBLIC_MASS_MODELS)
@pytest.mark.parametrize("mass", INVALID_MASSES)
@pytest.mark.parametrize("path", ["reset", "attached"])
def test_invalid_mass(kind, mass, path):
    """Reject negative and non-finite masses before dynamics initialization."""
    effector = make_effector(kind, mass=mass)
    with pytest.raises(BasiliskError, match="mass.*finite"):
        validate(effector, path)


@pytest.mark.parametrize("kind", ["dual", "chain"])
@pytest.mark.parametrize("mass", INVALID_MASSES)
@pytest.mark.parametrize("path", ["reset", "attached"])
def test_invalid_second_panel_mass(kind, mass, path):
    """Check every panel, including one beyond the first."""
    effector = make_effector(kind, second_mass=mass)
    with pytest.raises(BasiliskError, match="mass.*finite"):
        validate(effector, path)


@pytest.mark.parametrize("kind", PUBLIC_MASS_MODELS)
def test_rejection_precedes_state_registration(kind):
    """Invalid configuration must not leave partially registered effector states."""
    effector = make_effector(kind, mass=-1.0)  # [kg]
    manager = spacecraft.DynParamManager()
    with pytest.raises(BasiliskError, match="mass.*finite"):
        effector.registerStates(manager)
    names = state_names(effector)
    assert names
    for name in names:
        assert manager.getStateObject(name) is None


@pytest.mark.parametrize("mass", INVALID_MASSES)
@pytest.mark.parametrize("initialized", [False, True])
def test_prescribed_mass_setter_is_transactional(mass, initialized):
    """Invalid setter input must raise immediately and preserve the previous mass."""
    effector = make_effector("prescribed")
    if initialized:
        simulation, body = make_simulation(effector)
        simulation.InitializeSimulation()
    with pytest.raises(BasiliskError, match="mass.*finite"):
        effector.setMass(mass)
    assert effector.getMass() == 1.0  # [kg]
    effector.Reset(0)


@pytest.mark.parametrize("kind", HINGED_MODELS)
@pytest.mark.parametrize("path", ["reset", "attached"])
@pytest.mark.parametrize("dcm", [
    np.zeros((3, 3)),
    np.diag([2.0, 1.0, 1.0]),
    [[1.0, 0.2, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    np.diag([-1.0, 1.0, 1.0]),
    np.diag([np.nan, 1.0, 1.0]),
    np.diag([np.inf, 1.0, 1.0]),
], ids=["zero", "scale", "shear", "reflection", "nan", "infinity"])
def test_invalid_hinge_orientation(kind, path, dcm):
    """Reject non-finite, non-orthogonal, and left-handed hinge frames."""
    effector = make_effector(kind)
    setattr(effector, "dcm_H1B" if kind == "dual" else "dcm_HB", np.asarray(dcm).tolist())
    with pytest.raises(BasiliskError, match="dcm_H.*rotation matrix"):
        validate(effector, path)


@pytest.mark.parametrize("kind", ["dual", "chain"])
@pytest.mark.parametrize("path", ["reset", "attached"])
@pytest.mark.parametrize("mass", [0.0, np.finfo(float).max])  # [kg]
def test_invalid_combined_panel_mass(kind, path, mass):
    """Reject undefined center-of-mass division and overflow of combined mass."""
    effector = make_effector(kind, mass=mass, second_mass=mass)
    with pytest.raises(BasiliskError, match="mass"):
        validate(effector, path)


@pytest.mark.parametrize("path", ["reset", "attached"])
@pytest.mark.parametrize("field,value", [
    ("mass", 2.0),  # [kg]
    ("d", 2.0),  # [m]
    ("d", np.nan),  # [m]
    ("d", np.inf),  # [m]
])
def test_chain_uniformity(path, field, value):
    """Require finite, uniform panel data from both initialization paths."""
    parameter = "second_mass" if field == "mass" else "second_distance"
    effector = make_effector("chain", **{parameter: value})
    with pytest.raises(BasiliskError, match="every panel"):
        validate(effector, path)


@pytest.mark.parametrize("kind", ALL_MODELS)
@pytest.mark.parametrize("priority", [None, -100, 100], ids=["attached", "after", "before"])
def test_reset_preserves_states_and_valid_orientation(kind, priority):
    """Accept valid models in every scheduling order without rewinding states on Reset."""
    effector = make_effector(kind)
    if kind in HINGED_MODELS:
        dcm_field = "dcm_H1B" if kind == "dual" else "dcm_HB"
        setattr(effector, dcm_field, ROTATION)
    if kind == "prescribed":
        effector.setOmegaPrime_PM_P([0.1, -0.2, 0.05])  # [rad/s^2]
    simulation, body = make_simulation(effector, priority)
    simulation.InitializeSimulation()
    states = [body.dynManager.getStateObject(name) for name in state_names(effector)]
    for state in states:
        changed_state = np.full_like(state.getState(), 0.25, dtype=float)  # [rad], [rad/s], [m], [m/s], or [kg]
        state.setState(changed_state.tolist())
    if kind == "prescribed":
        simulation.ConfigureStopTime(macros.sec2nano(0.02))  # [s]
        simulation.ExecuteSimulation()
        attitude = np.asarray(effector.getSigma_PM()).copy()
        assert np.linalg.norm(attitude) > 0.0
    snapshots = [np.asarray(state.getState()).copy() for state in states]
    effector.Reset(0)
    effector.Reset(macros.sec2nano(2.0))  # [s]
    for state, snapshot in zip(states, snapshots):
        np.testing.assert_array_equal(state.getState(), snapshot)
    if kind == "prescribed":
        effector.updateEffectorMassProps(0.02)  # [s]; read the integrated attitude after Reset
        np.testing.assert_array_equal(effector.getSigma_PM(), attitude)
    if kind in HINGED_MODELS:
        np.testing.assert_array_equal(getattr(effector, dcm_field), ROTATION)
    simulation.ConfigureStopTime(macros.sec2nano(0.02))  # [s]
    simulation.ExecuteSimulation()
    for state in states:
        assert np.isfinite(state.getState()).all()


@pytest.mark.parametrize("kind", [kind for kind in ALL_MODELS if kind != "chain"])
@pytest.mark.parametrize("priority", [None, -100, 100], ids=["attached", "after", "before"])
def test_supported_zero_mass(kind, priority):
    """Preserve supported zero-mass configurations with usable geometry and inertia."""
    effector = make_effector(kind)
    set_mass(effector, kind, 0.0)  # [kg]; the dual model retains a massive second panel
    simulation, body = make_simulation(effector, priority)
    simulation.InitializeSimulation()
    effector.Reset(0)
    simulation.ConfigureStopTime(macros.sec2nano(0.02))  # [s]
    simulation.ExecuteSimulation()
    for name in state_names(effector):
        assert np.isfinite(body.dynManager.getStateObject(name).getState()).all()


@pytest.mark.parametrize("kind", SLOSH_MODELS)
def test_reset_preserves_depleted_mass(kind):
    """Reset must not restore initial fuel mass after a slosh particle empties."""
    effector = make_effector(kind)
    simulation, body = make_simulation(effector)
    simulation.InitializeSimulation()
    mass_state = body.dynManager.getStateObject(effector.nameOfMassState)
    mass_state.setState([[0.0]])  # [kg]
    effector.Reset(0)
    effector.Reset(0)
    np.testing.assert_array_equal(mass_state.getState(), [[0.0]])
    simulation.ConfigureStopTime(macros.sec2nano(0.02))  # [s]
    simulation.ExecuteSimulation()
    np.testing.assert_array_equal(mass_state.getState(), [[0.0]])
    for name in state_names(effector):
        assert np.isfinite(body.dynManager.getStateObject(name).getState()).all()


@pytest.mark.parametrize("kind", HINGED_MODELS)
@pytest.mark.parametrize("invalid", ["mass", "orientation"])
def test_prescribed_parent_validates_children(kind, invalid):
    """Nested state registration must enforce the same checks as hub attachment."""
    parent = make_effector("prescribed")
    if invalid == "mass":
        effector = make_effector(kind, mass=-1.0)  # [kg]
    else:
        effector = make_effector(kind)
        setattr(effector, "dcm_H1B" if kind == "dual" else "dcm_HB", np.zeros((3, 3)).tolist())
    simulation, body = make_simulation(effector, parent=parent)
    with pytest.raises(BasiliskError, match="mass|dcm_H"):
        simulation.InitializeSimulation()


@pytest.mark.parametrize("damping", [
    np.diag([-1.0, 0.0, 0.0]),
    np.diag([np.nan, 0.0, 0.0]),
    np.diag([np.inf, 0.0, 0.0]),
    [[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
])  # [N*s/m]
def test_spherical_reset_checks_damping(damping):
    """Reset must enforce the same damping validation as state registration."""
    effector = make_effector("spherical")
    effector.D = np.asarray(damping).tolist()
    with pytest.raises(BasiliskError, match="D must be symmetric positive semidefinite"):
        effector.Reset(0)


@pytest.mark.parametrize("damping", [np.zeros((3, 3)), np.diag([1.0, 0.0, 0.0])])  # [N*s/m]
def test_spherical_reset_accepts_semidefinite_damping(damping):
    """Zero and singular positive-semidefinite damping remain supported."""
    effector = make_effector("spherical")
    effector.D = damping.tolist()
    validate(effector, "reset")
    validate(effector, "attached")
