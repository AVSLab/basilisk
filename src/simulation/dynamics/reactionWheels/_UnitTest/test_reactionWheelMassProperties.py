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

"""Validate optional wheel mass accounting against fully coupled and lumped-hub models."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import reactionWheelStateEffector, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, simIncludeRW


BALANCED = reactionWheelStateEffector.BalancedWheels
SIMPLE = reactionWheelStateEffector.JitterSimple
COUPLED = reactionWheelStateEffector.JitterFullyCoupled


def _parallel_axis(position):
    """Return the parallel-axis tensor per unit mass for a body-frame displacement."""
    return np.dot(position, position) * np.eye(3) - np.outer(position, position)


def _make_simulation(models, include=None, lump_into_hub=False, driven=False):
    """Configure offset, nonparallel wheels on a translating and rotating free spacecraft."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    step = 0.002  # [s]
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(step)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "spacecraft"
    hub_mass = 80.0  # [kg]
    hub_center = np.array([0.2, -0.1, 0.3])  # [m]
    hub_inertia = np.diag([20.0, 25.0, 30.0])  # [kg*m^2]
    sc.hub.r_CN_NInit = [1.0, 2.0, -3.0]  # [m]
    sc.hub.v_CN_NInit = [0.2, 0.3, -0.1]  # [m/s]
    sc.hub.sigma_BNInit = [0.1, -0.2, 0.05]  # [-]
    sc.hub.omega_BN_BInit = [0.12, -0.08, 0.2]  # [rad/s]

    factory = simIncludeRW.rwFactory()
    axes = [[1.0, 2.0, -1.0], [-2.0, 1.0, 3.0], [2.0, -3.0, 1.0]]  # [-]
    positions = [[0.7, -0.4, 0.2], [-0.3, 0.8, 0.5], [0.2, 0.1, -0.6]]  # [m]
    masses = [2.0, 3.0, 4.0]  # [kg]
    spin_inertias = [0.2, 0.3, 0.4]  # [kg*m^2]
    transverse_inertias = [0.15, 0.23, 0.31]  # [kg*m^2]
    speeds = [2.0, -3.0, 4.0]  # [rad/s]
    first_moment = hub_mass * hub_center
    inertia_at_origin = hub_inertia + hub_mass * _parallel_axis(hub_center)
    expected_mass = hub_mass
    expected_first_moment = first_moment.copy()
    expected_inertia = inertia_at_origin.copy()
    for i, model in enumerate(models):
        wheel = factory.create(
            "custom", axes[i], Js=spin_inertias[i], rWB_B=positions[i], RWModel=model,
            useMaxTorque=False, useMinTorque=False, useRWfriction=False,
        )
        wheel.mass = masses[i]
        wheel.Jt = transverse_inertias[i]
        wheel.Jg = transverse_inertias[i]
        wheel.Omega = speeds[i]
        wheel.U_s = 0.0  # [kg*m]
        wheel.U_d = 0.0  # [kg*m^2]
        # Transform principal inertias using the factory's independent orthonormal wheel triad.
        frame = np.column_stack([np.array(axis).reshape(3) for axis in
                                 (wheel.gsHat_B, wheel.w2Hat0_B, wheel.w3Hat0_B)])
        tensor = frame @ np.diag([wheel.Js, wheel.Jt, wheel.Jg]) @ frame.T
        position = np.array(positions[i])
        translated_tensor = tensor + wheel.mass * _parallel_axis(position)
        if include or lump_into_hub or model == COUPLED:
            expected_mass += wheel.mass
            expected_first_moment += wheel.mass * position
            expected_inertia += translated_tensor
        if lump_into_hub and model != COUPLED:
            hub_mass += wheel.mass
            first_moment += wheel.mass * position
            inertia_at_origin += translated_tensor

    hub_center = first_moment / hub_mass
    sc.hub.mHub = hub_mass
    sc.hub.r_BcB_B = hub_center.tolist()
    sc.hub.IHubPntBc_B = (inertia_at_origin - hub_mass * _parallel_axis(hub_center)).tolist()
    wheels = reactionWheelStateEffector.ReactionWheelStateEffector()
    assert wheels.includeWheelMassProperties is False
    if include is not None:
        wheels.includeWheelMassProperties = include
    factory.addToSpacecraft("wheels", wheels, sc)
    torques = [0.03, -0.02, 0.01] if driven else [0.0, 0.0, 0.0]  # [N*m]
    command = messaging.ArrayMotorTorqueMsg().write(messaging.ArrayMotorTorqueMsgPayload(motorTorque=torques))
    wheels.rwMotorCmdInMsg.subscribeTo(command)
    sim.AddModelToTask("task", wheels, 2)
    sim.AddModelToTask("task", sc, 1)
    state = sc.scStateOutMsg.recorder()
    mass = sc.scMassOutMsg.recorder()
    speed = wheels.rwSpeedOutMsg.recorder()
    conserved = sc.logger(["totRotEnergy", "totOrbEnergy", "totRotAngMomPntC_N", "totOrbAngMomPntN_N"])
    for recorder in (state, mass, speed, conserved):
        sim.AddModelToTask("task", recorder)
    expected = (expected_mass, expected_first_moment / expected_mass, expected_inertia)
    return sim, sc, wheels, factory, command, state, mass, speed, conserved, expected


def _run(models, include=None, lump_into_hub=False, driven=False):
    """Run the configured system and return histories and independent mass-property expectations."""
    objects = _make_simulation(models, include, lump_into_hub, driven)
    sim, sc, wheels, factory, command, state, mass, speed, conserved, expected = objects
    sim.InitializeSimulation()
    duration = 3.0  # [s]
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    histories = {
        "position": state.r_BN_N,
        "velocity": state.v_BN_N,
        "attitude": state.sigma_BN,
        "rate": state.omega_BN_B,
        "speeds": speed.wheelSpeeds[:, :len(models)],
        "mass": mass.massSC,
        "center": mass.c_B,
        "inertia": mass.ISC_PntB_B,
        "rot_energy": conserved.totRotEnergy,
        "orb_energy": conserved.totOrbEnergy,
        "rot_momentum": conserved.totRotAngMomPntC_N,
        "orb_momentum": conserved.totOrbAngMomPntN_N,
    }
    for name, truth in zip(("mass", "center", "inertia"), expected):
        np.testing.assert_allclose(histories[name] - truth, 0.0, atol=2.0e-13,
                                   err_msg=f"Incorrect {name} accounting")
    np.testing.assert_allclose(wheels.effProps.rEffPrime_CB_B, 0.0, atol=1.0e-14)  # [m/s]
    np.testing.assert_allclose(wheels.effProps.IEffPrimePntB_B, 0.0, atol=1.0e-14)  # [kg*m^2/s]
    return histories


@pytest.mark.parametrize("models", [[BALANCED] * 3, [SIMPLE] * 3, [BALANCED, SIMPLE, COUPLED]],
                         ids=["balanced", "simple", "mixed"])
@pytest.mark.parametrize("driven", [False, True], ids=["free", "motor_torque"])
def test_mass_properties_match_fully_coupled(models, driven):
    """Compare mass properties and trajectories to zero-imbalance fully coupled and lumped-hub systems.

    Offset wheel locations and a nonzero hub center exercise parallel-axis and first-moment terms.
    Free motion must conserve energy and momentum; internal motor torque must conserve momentum.
    The mixed case verifies that the option never double-counts fully coupled wheel properties.
    """
    automatic = _run(models, include=True, driven=driven)
    coupled = _run([COUPLED] * 3, include=True, driven=driven)
    legacy = _run(models, lump_into_hub=True, driven=driven)
    for name, history in automatic.items():
        for reference in (coupled, legacy):
            np.testing.assert_allclose(history, reference[name], rtol=2.0e-11, atol=2.0e-12, err_msg=name)
    for name in ("rot_momentum", "orb_momentum", "orb_energy"):
        np.testing.assert_allclose(automatic[name] - automatic[name][0], 0.0, atol=2.0e-11, err_msg=name)
    if not driven:
        np.testing.assert_allclose(automatic["rot_energy"], automatic["rot_energy"][0], rtol=2.0e-12)
    else:
        assert np.max(np.abs(automatic["speeds"][-1] - automatic["speeds"][0])) > 0.05  # [rad/s]


@pytest.mark.parametrize("model", [BALANCED, SIMPLE, COUPLED])
def test_default_mass_accounting_is_preserved(model):
    """Verify the unset option matches explicit false and fully coupled accounting is unconditional."""
    default = _run([model] * 3)
    disabled = _run([model] * 3, include=False)
    for name in default:
        np.testing.assert_array_equal(default[name], disabled[name])
    if model == COUPLED:
        enabled = _run([model] * 3, include=True)
        for name in default:
            np.testing.assert_array_equal(default[name], enabled[name])


@pytest.mark.parametrize("field,value,expected_error", [
    ("mass", 0.0, "positive wheel mass"),  # [kg]
    ("mass", float("nan"), "positive wheel mass"),  # [kg]
    ("Js", -0.1, "positive Js"),  # [kg*m^2]
    ("Jt", float("inf"), "positive wheel mass"),  # [kg*m^2]
    ("Jg", 0.16, "axisymmetric wheel inertia"),  # [kg*m^2]
    ("Js", 0.4, "axisymmetric wheel inertia"),  # [kg*m^2]
    ("gsHat_B", [2.0, 0.0, 0.0], "unit spin axis"),  # [-]
    ("rWB_B", [float("nan"), 0.0, 0.0], "finite wheel position"),  # [m]
])
@pytest.mark.parametrize("validation_path", ["attachment", "reset"])
def test_automatic_mass_properties_validate_configuration(field, value, expected_error, validation_path):
    """Reject invalid opt-in geometry during state registration even without task scheduling."""
    objects = _make_simulation([BALANCED] * 3, include=True)
    sim, sc, wheels, factory = objects[:4]
    setattr(factory.rwList["RW1"], field, value)
    with pytest.raises(BasiliskError, match=expected_error):
        if validation_path == "attachment":
            wheels.registerStates(sc.dynManager)
        else:
            wheels.Reset(0)


@pytest.mark.parametrize("field,value,expected_error", [
    ("mass", 0.0, "positive wheel mass"),  # [kg]
    ("mass", -1.0, "positive wheel mass"),  # [kg]
    ("mass", float("nan"), "positive wheel mass"),  # [kg]
    ("mass", float("inf"), "positive wheel mass"),  # [kg]
    ("Jt", 0.0, "positive wheel mass"),  # [kg*m^2]
    ("Jg", -0.1, "positive wheel mass"),  # [kg*m^2]
    ("Jg", float("nan"), "positive wheel mass"),  # [kg*m^2]
    ("U_s", float("inf"), "finite U_s and U_d"),  # [kg*m]
    ("U_d", float("nan"), "finite U_s and U_d"),  # [kg*m^2]
    ("U_d", 0.3, "positive definite"),  # [kg*m^2]
    ("U_d", -0.3, "positive definite"),  # [kg*m^2]
    ("gsHat_B", [2.0, 0.0, 0.0], "unit spin axis"),  # [-]
    ("rWB_B", [float("nan"), 0.0, 0.0], "finite wheel position"),  # [m]
])
@pytest.mark.parametrize("validation_path", ["attachment", "reset"])
def test_fully_coupled_mass_properties_validate_configuration(field, value, expected_error, validation_path):
    """Validate coupled wheel inputs before division or inertia use, with the new option left disabled."""
    objects = _make_simulation([COUPLED] * 3)
    sim, sc, wheels, factory = objects[:4]
    setattr(factory.rwList["RW1"], field, value)
    with pytest.raises(BasiliskError, match=expected_error):
        if validation_path == "attachment":
            wheels.registerStates(sc.dynManager)
        else:
            wheels.Reset(0)


@pytest.mark.parametrize("model", [BALANCED, SIMPLE, COUPLED])
@pytest.mark.parametrize("spin_inertia", [0.0, -0.1, float("nan"), float("inf")])  # [kg*m^2]
def test_all_models_require_valid_spin_inertia(model, spin_inertia):
    """Spin inertia is consumed by every model even when constant wheel mass properties are in the hub."""
    objects = _make_simulation([model] * 3)
    sim, sc, wheels, factory = objects[:4]
    factory.rwList["RW1"].Js = spin_inertia
    with pytest.raises(BasiliskError, match="positive Js"):
        wheels.registerStates(sc.dynManager)


@pytest.mark.parametrize("model", [BALANCED, SIMPLE])
def test_legacy_simplified_models_do_not_require_unused_mass_properties(model):
    """Legacy wheel configurations can omit the constant mass and transverse inertia kept in the hub."""
    objects = _make_simulation([model] * 3)
    sim, sc, wheels, factory = objects[:4]
    for wheel in factory.rwList.values():
        wheel.mass = 0.0  # [kg] unused in legacy simplified accounting
        wheel.Jt = 0.0  # [kg*m^2] unused in legacy simplified accounting
        wheel.Jg = 0.0  # [kg*m^2] unused in legacy simplified accounting
    sim.InitializeSimulation()


def test_fully_coupled_dynamic_imbalance_conserves_energy_and_momentum():
    """Verify free-motion conservation with equal transverse moments and a nonzero product of inertia."""
    objects = _make_simulation([COUPLED] * 3)
    sim, sc, wheels, factory = objects[:4]
    conserved = objects[8]
    wheel = factory.rwList["RW1"]
    wheel.Jg = wheel.Jt
    wheel.U_d = 0.01  # [kg*m^2]
    sim.InitializeSimulation()
    assert wheel.J13 == pytest.approx(0.01)  # [kg*m^2]

    duration = 3.0  # [s]
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()

    for energy in (conserved.totRotEnergy, conserved.totOrbEnergy):
        np.testing.assert_allclose(energy, energy[0], rtol=2.0e-12, atol=0.0)
    for momentum in (conserved.totRotAngMomPntC_N, conserved.totOrbAngMomPntN_N):
        np.testing.assert_allclose(momentum - momentum[0], 0.0, atol=2.0e-11)  # [kg*m^2/s]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
