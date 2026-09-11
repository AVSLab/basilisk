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

"""Verify VSCMG attachment initialization and repeatable configuration validation."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import spacecraft, vscmgStateEffector
from Basilisk.utilities import SimulationBaseClass, macros


MODELS = [vscmgStateEffector.vscmgBalancedWheels,
          vscmgStateEffector.vscmgJitterSimple,
          vscmgStateEffector.vscmgJitterFullyCoupled]


def configuration(model):
    """Create a device with finite geometry and nonzero spin and gimbal inertia."""
    config = messaging.VSCMGConfigMsgPayload()
    config.VSCMGModel = model
    config.gsHat0_B = [1.0, 0.0, 0.0]  # [-]
    config.gtHat0_B = [0.0, 1.0, 0.0]  # [-]
    config.ggHat_B = [0.0, 0.0, 1.0]  # [-]
    config.massW = 2.0  # [kg]
    config.massG = 1.0  # [kg]
    config.IW1 = 0.2  # [kg*m^2]
    config.IW2 = config.IW3 = 0.15  # [kg*m^2]
    config.IG1 = 0.1  # [kg*m^2]
    config.IG2 = 0.15  # [kg*m^2]
    config.IG3 = 0.2  # [kg*m^2]
    config.U_s = 0.02  # [kg*m]
    config.U_d = 0.001  # [kg*m^2]
    config.rGcG_G = [0.01, 0.02, 0.0]  # [m]
    config.l = 0.01  # [m]
    config.L = 0.1  # [m]
    config.Omega = 1.0  # [rad/s]
    config.gamma = 0.3  # [rad]
    config.gammaDot = 0.02  # [rad/s]
    return config


def simulation_with_effector(config, priority=None):
    """Attach one device and optionally schedule its command-processing lifecycle."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    process.addTask(simulation.CreateNewTask("task", macros.sec2nano(0.001)))  # [ns]
    parent = spacecraft.Spacecraft()
    parent.hub.mHub = 100.0  # [kg]
    parent.hub.IHubPntBc_B = np.eye(3) * 10.0  # [kg*m^2]
    effector = vscmgStateEffector.VSCMGStateEffector()
    effector.AddVSCMG(config)
    parent.addStateEffector(effector)
    simulation.AddModelToTask("task", parent)
    command = messaging.VSCMGArrayTorqueMsg().write(messaging.VSCMGArrayTorqueMsgPayload())
    if priority is not None:
        effector.cmdsInMsg.subscribeTo(command)
        simulation.AddModelToTask("task", effector, priority)
    return simulation, parent, effector, command


@pytest.mark.parametrize("model", MODELS)
def test_attachment_matches_both_scheduler_orders(model):
    """Derived properties and propagation must agree with either Reset ordering or no scheduled Reset."""
    histories = []
    for priority in [None, 10, -10]:
        simulation, parent, effector, command = simulation_with_effector(configuration(model), priority)
        recorder = parent.scStateOutMsg.recorder()
        simulation.AddModelToTask("task", recorder)
        simulation.InitializeSimulation()
        device = effector.VSCMGData[0]
        assert device.massV == pytest.approx(3.0)  # [kg]
        assert device.rhoW == pytest.approx(2.0 / 3.0)  # [-]
        if model == vscmgStateEffector.vscmgJitterFullyCoupled:
            assert device.d == pytest.approx(0.01)  # [m]
            assert device.IW13 == pytest.approx(0.001)  # [kg*m^2]
        else:
            assert device.IV1 == pytest.approx(0.3)  # [kg*m^2]
            assert device.IV3 == pytest.approx(0.35)  # [kg*m^2]
        simulation.ConfigureStopTime(macros.sec2nano(0.02))  # [ns]
        simulation.ExecuteSimulation()
        histories.append(np.hstack((recorder.r_BN_N, recorder.sigma_BN, recorder.omega_BN_B)))
        assert np.isfinite(histories[-1]).all()
    for history in histories[1:]:
        np.testing.assert_allclose(history, histories[0], rtol=1e-12, atol=1e-14)


@pytest.mark.parametrize("model", MODELS[:2])
def test_massless_device_has_finite_derived_values(model):
    """Balanced and simple jitter devices retain support for zero translational mass."""
    config = configuration(model)
    config.massW = config.massG = 0.0  # [kg]
    simulation, parent, effector, command = simulation_with_effector(config)
    simulation.InitializeSimulation()
    effector.Reset(0)
    device = effector.VSCMGData[0]
    assert device.massV == 0.0  # [kg]
    assert device.rhoW == device.rhoG == 0.0  # [-]
    simulation.ConfigureStopTime(macros.sec2nano(0.002))  # [ns]
    simulation.ExecuteSimulation()
    assert np.isfinite(parent.dynManager.getStateObject("hubOmega").getState()).all()


def test_fully_coupled_device_allows_massless_gimbal():
    """Only wheel and total mass are divisors in fully coupled initialization."""
    config = configuration(vscmgStateEffector.vscmgJitterFullyCoupled)
    config.massG = 0.0  # [kg]
    simulation, parent, effector, command = simulation_with_effector(config)
    simulation.InitializeSimulation()
    assert effector.VSCMGData[0].rhoG == 0.0  # [-]
    assert effector.VSCMGData[0].rhoW == 1.0  # [-]


@pytest.mark.parametrize("model", MODELS[:2])
@pytest.mark.parametrize("field", ["IW1", "IV3"])
def test_uncoupled_models_reject_zero_inertia_divisor(model, field):
    """Reject zero denominators without requiring positive translational mass."""
    config = configuration(model)
    if field == "IW1":
        config.IW1 = 0.0  # [kg*m^2]
    else:
        config.IW3 = config.IG3 = 0.0  # [kg*m^2]
    simulation, parent, effector, command = simulation_with_effector(config)
    with pytest.raises(BasiliskError, match="must be positive"):
        simulation.InitializeSimulation()


@pytest.mark.parametrize("path", ["reset", "attachment"])
@pytest.mark.parametrize("field,value,match", [
    ("massW", -1.0, "massW"),  # [kg]
    ("massG", -1.0, "massG"),  # [kg]
    ("massW", 0.0, "massW"),  # [kg], fully coupled model
    ("massG", np.nan, "massG"),  # [kg]
    ("gsHat0_B", [0.0, 0.0, 0.0], "gsHat0_B"),  # [-]
    ("gsHat0_B", [np.inf, 0.0, 0.0], "gsHat0_B"),  # [-]
    ("gtHat0_B", [1.0, 0.0, 0.0], "orthogonal"),  # [-]
    ("ggHat_B", [0.0, 0.0, -1.0], "right-handed"),  # [-]
    ("U_s", np.inf, "U_s"),  # [kg*m]
    ("gamma", np.nan, "gamma"),  # [rad]
    ("IW1", -1.0, "inertias"),  # [kg*m^2]
    ("IG13", np.nan, "IG13"),  # [kg*m^2]
    ("rGcG_G", [np.nan, 0.0, 0.0], "rGcG_G"),  # [m]
    ("VSCMGModel", 99, "VSCMGModel"),
])
def test_invalid_configuration_rejected(path, field, value, match):
    """Reject each invalid configuration before state registration or derived divisions."""
    config = configuration(vscmgStateEffector.vscmgJitterFullyCoupled)
    setattr(config, field, value)
    simulation, parent, effector, command = simulation_with_effector(config)
    with pytest.raises(BasiliskError, match=match):
        if path == "reset":
            effector.Reset(0)
        else:
            simulation.InitializeSimulation()


@pytest.mark.parametrize("scale", [2.0, 1e300, 1e-300])
def test_repeated_reset_normalizes_axes_and_preserves_states(scale):
    """Repeated normalization is stable and Reset does not rewind integrated states or applied torques."""
    config = configuration(vscmgStateEffector.vscmgJitterFullyCoupled)
    config.gsHat0_B = [scale, 0.0, 0.0]  # [-]
    config.gtHat0_B = [0.0, scale, 0.0]  # [-]
    config.ggHat_B = [0.0, 0.0, scale]  # [-]
    config.u_s_current = 0.1  # [N*m]
    simulation, parent, effector, command = simulation_with_effector(config)
    simulation.InitializeSimulation()
    state = parent.dynManager.getStateObject(effector.nameOfVSCMGOmegasState)
    state.setState([[2.0]])  # [rad/s]
    for _ in range(3):
        pending = messaging.VSCMGCmdMsgPayload()
        pending.u_s_cmd = 0.2  # [N*m]
        effector.newVSCMGCmds[0] = pending
        effector.Reset(0)
        device = effector.VSCMGData[0]
        np.testing.assert_allclose(np.hstack((device.gsHat0_B, device.gtHat0_B, device.ggHat_B)), np.eye(3))
        assert device.d == pytest.approx(0.01)  # [m]
        assert device.u_s_current == pytest.approx(0.1)  # [N*m]
        assert effector.newVSCMGCmds[0].u_s_cmd == 0.0  # [N*m]
        assert state.getState()[0][0] == 2.0  # [rad/s]


@pytest.mark.parametrize("path", ["reset", "attachment"])
def test_device_count_exceeding_payload_capacity_is_rejected(path):
    """Reject oversized device arrays before accessing fixed-size message payloads."""
    capacity = len(messaging.VSCMGArrayTorqueMsgPayload().wheelTorque)
    config = configuration(MODELS[0])
    simulation, parent, effector, command = simulation_with_effector(config)
    for _ in range(capacity):
        effector.AddVSCMG(config)
    with pytest.raises(BasiliskError, match="device count.*MAX_EFF_CNT"):
        if path == "reset":
            effector.Reset(0)
        else:
            simulation.InitializeSimulation()


@pytest.mark.parametrize("model", MODELS)
def test_device_count_at_payload_capacity_processes_last_command(model):
    """The largest supported array must command and publish its last device correctly."""
    payload = messaging.VSCMGArrayTorqueMsgPayload()
    capacity = len(payload.wheelTorque)
    config = configuration(model)
    simulation, parent, effector, command = simulation_with_effector(config, -10)
    for _ in range(capacity - 1):
        effector.AddVSCMG(config)
    payload.wheelTorque = [0.0] * (capacity - 1) + [0.04]  # [N*m]
    payload.gimbalTorque = [0.0] * (capacity - 1) + [-0.01]  # [N*m]
    command.write(payload)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(0.002))  # [ns]
    simulation.ExecuteSimulation()
    assert effector.VSCMGData[-1].u_s_current == pytest.approx(0.04)  # [N*m]
    assert effector.VSCMGData[-1].u_g_current == pytest.approx(-0.01)  # [N*m]
    output = effector.speedOutMsg.read()
    wheel_state = parent.dynManager.getStateObject(effector.nameOfVSCMGOmegasState).getState()
    assert np.isfinite(wheel_state).all()
    np.testing.assert_allclose(output.wheelSpeeds, np.asarray(wheel_state).ravel())


@pytest.mark.parametrize("path", ["reset", "attachment"])
@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf])
@pytest.mark.parametrize("field", [
    "u_s_current", "u_g_current", "u_s_f", "u_g_f",  # [N*m]
    "u_s_max", "u_g_max", "u_s_min", "u_g_min",  # [N*m]
    "Omega_max", "gammaDot_max",  # [rad/s]
    "wheelLinearFrictionRatio", "gimbalLinearFrictionRatio",  # [-]
])
def test_nonfinite_motor_configuration_is_rejected(path, value, field):
    """Reject non-finite motor settings through both configuration entry points."""
    config = configuration(MODELS[0])
    setattr(config, field, value)
    simulation, parent, effector, command = simulation_with_effector(config)
    with pytest.raises(BasiliskError, match=field):
        if path == "reset":
            effector.Reset(0)
        else:
            simulation.InitializeSimulation()


def coupled_configuration_with_invalid_divisor(case):
    """Construct finite inputs that produce a zero or overflowing coupled divisor."""
    config = configuration(MODELS[2])
    config.gamma = 0.0  # [rad]
    config.U_s = 0.0  # [kg*m]
    config.U_d = 0.0  # [kg*m^2]
    config.l = config.L = 0.0  # [m]
    config.rGcG_G = [0.0, 0.0, 0.0]  # [m]
    if case == "zero_spin":
        config.IW1 = 0.0  # [kg*m^2]
    elif case == "tiny_spin":
        config.IW1 = 1e-320  # [kg*m^2]
    elif case == "overflow_spin":
        config.U_s = 1e200  # [kg*m]
    elif case == "tiny_mass":
        config.massW = 1e-320  # [kg]
        config.massG = 0.0  # [kg]
    elif case == "zero_gimbal":
        config.IW3 = config.IG3 = 0.0  # [kg*m^2]
    elif case == "overflow_gimbal":
        config.IW3 = config.IG3 = 1e308  # [kg*m^2]
    elif case == "singular_coupling":
        config.IW1 = config.IW3 = 1.0  # [kg*m^2]
        config.IG3 = 0.0  # [kg*m^2]
        config.U_d = 1.0  # [kg*m^2]
    return config


@pytest.mark.parametrize("case,divisor", [
    ("zero_spin", "eOmega"),
    ("tiny_spin", "eOmega"),
    ("overflow_spin", "eOmega"),
    ("tiny_mass", "massV"),
    ("zero_gimbal", "egamma"),
    ("overflow_gimbal", "egamma"),
    ("singular_coupling", r"1 - cOmega\*cgamma"),
])
def test_invalid_coupled_divisors_stop_initialization(case, divisor):
    """Fail before dividing by a singular coupled inertia or elimination coefficient."""
    config = coupled_configuration_with_invalid_divisor(case)
    simulation, parent, effector, command = simulation_with_effector(config)
    with pytest.raises(BasiliskError, match=divisor):
        simulation.InitializeSimulation()


@pytest.mark.parametrize("case", ["zero_spin", "tiny_spin", "overflow_spin"])
def test_invalid_coupled_spin_divisor_is_rejected_by_reset(case):
    """Validate the configuration-dependent spin divisor without linked parent states."""
    effector = vscmgStateEffector.VSCMGStateEffector()
    effector.AddVSCMG(coupled_configuration_with_invalid_divisor(case))
    with pytest.raises(BasiliskError, match="eOmega"):
        effector.Reset(0)


def test_coupled_divisor_is_checked_again_during_propagation():
    """A valid startup must not bypass guards when a later dynamics evaluation is singular."""
    config = configuration(MODELS[2])
    config.U_s = config.U_d = 0.0  # [kg*m], [kg*m^2]
    simulation, parent, effector, command = simulation_with_effector(config)
    simulation.InitializeSimulation()
    effector.VSCMGData[0].IW1 = 0.0  # [kg*m^2]
    simulation.ConfigureStopTime(macros.sec2nano(0.002))  # [ns]
    with pytest.raises(BasiliskError, match="eOmega"):
        simulation.ExecuteSimulation()


@pytest.mark.parametrize("field", ["wheelTorque", "gimbalTorque"])
@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf])
def test_nonfinite_command_is_rejected_before_motor_processing(field, value):
    """Reject non-finite commanded torques before saturation or friction can mask them."""
    simulation, parent, effector, command = simulation_with_effector(configuration(MODELS[0]), 10)
    simulation.InitializeSimulation()
    payload = messaging.VSCMGArrayTorqueMsgPayload()
    torques = list(getattr(payload, field))
    torques[0] = value  # [N*m]
    setattr(payload, field, torques)
    command.write(payload)
    simulation.ConfigureStopTime(macros.sec2nano(0.002))  # [ns]
    with pytest.raises(BasiliskError, match="u_s_cmd and u_g_cmd must be finite"):
        simulation.ExecuteSimulation()


@pytest.mark.parametrize("limit_field,ratio_field", [
    ("Omega_max", "wheelLinearFrictionRatio"),
    ("gammaDot_max", "gimbalLinearFrictionRatio"),
])
@pytest.mark.parametrize("limit,ratio", [(0.0, 0.1), (-1.0, 0.1), (1e-300, 1e-300), (1e300, 1e300)])
def test_invalid_linear_friction_threshold_is_rejected(limit_field, ratio_field, limit, ratio):
    """Finite settings must not create a zero, negative, or infinite friction divisor."""
    config = configuration(MODELS[0])
    setattr(config, limit_field, limit)  # [rad/s]
    setattr(config, ratio_field, ratio)  # [-]
    config.Omega = config.gammaDot = 0.0  # [rad/s]
    simulation, parent, effector, command = simulation_with_effector(config, 10)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(0.002))  # [ns]
    with pytest.raises(BasiliskError, match=ratio_field):
        simulation.ExecuteSimulation()


def test_negative_motor_limits_and_friction_ratios_remain_supported():
    """Negative sentinels disable limits and friction smoothing without rejecting valid commands."""
    config = configuration(MODELS[0])
    config.u_s_max = config.u_g_max = -1.0  # [N*m]
    config.u_s_min = config.u_g_min = -1.0  # [N*m]
    config.Omega_max = config.gammaDot_max = -1.0  # [rad/s]
    config.wheelLinearFrictionRatio = config.gimbalLinearFrictionRatio = -1.0  # [-]
    simulation, parent, effector, command = simulation_with_effector(config, 10)
    payload = messaging.VSCMGArrayTorqueMsgPayload()
    payload.wheelTorque = [0.04]  # [N*m]
    payload.gimbalTorque = [-0.01]  # [N*m]
    command.write(payload)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(0.002))  # [ns]
    simulation.ExecuteSimulation()
    assert effector.VSCMGData[0].u_s_current == pytest.approx(0.04)  # [N*m]
    assert effector.VSCMGData[0].u_g_current == pytest.approx(-0.01)  # [N*m]


@pytest.mark.parametrize("wheel_ratio", [0.0, 0.1])
def test_gimbal_friction_uses_its_own_smoothing_ratio(wheel_ratio):
    """Gimbal smoothing must be independent of the wheel friction configuration."""
    config = configuration(MODELS[0])
    config.wheelLinearFrictionRatio = wheel_ratio  # [-]
    config.gimbalLinearFrictionRatio = 0.5  # [-]
    config.Omega_max = config.gammaDot_max = 10.0  # [rad/s]
    config.gammaDot = 1.0  # [rad/s]
    config.u_g_f = 0.5  # [N*m]
    effector = vscmgStateEffector.VSCMGStateEffector()
    effector.AddVSCMG(config)
    effector.Reset(0)
    effector.ConfigureVSCMGRequests(0.0)  # [s]
    assert effector.VSCMGData[0].u_g_current == pytest.approx(-0.1)  # [N*m]


def test_motor_arithmetic_overflow_is_rejected_before_applying_torque():
    """Finite commands and friction must not silently overflow the applied torque."""
    config = configuration(MODELS[0])
    config.Omega = -1.0  # [rad/s]
    config.u_s_f = 1e308  # [N*m]
    effector = vscmgStateEffector.VSCMGStateEffector()
    effector.AddVSCMG(config)
    effector.Reset(0)
    pending = messaging.VSCMGCmdMsgPayload()
    pending.u_s_cmd = 1e308  # [N*m]
    effector.newVSCMGCmds[0] = pending
    with pytest.raises(BasiliskError, match="non-finite applied torque"):
        effector.ConfigureVSCMGRequests(0.0)  # [s]
    assert effector.VSCMGData[0].u_s_current == 0.0  # [N*m]


@pytest.mark.parametrize("models", [MODELS, MODELS[::-1]])
def test_mixed_models_publish_only_their_own_jitter_angles(models):
    """Output indexing must follow the compact jitter state array when models are mixed."""
    simulation, parent, effector, command = simulation_with_effector(configuration(models[0]), -10)
    for model in models[1:]:
        effector.AddVSCMG(configuration(model))
    simulation.InitializeSimulation()
    angles = [0.2, 0.4]  # [rad]
    theta_state = parent.dynManager.getStateObject(effector.nameOfVSCMGThetasState)
    theta_state.setState(np.array(angles).reshape(-1, 1))
    effector.WriteOutputMessages(0)
    jitter_index = 0
    for index, model in enumerate(models):
        expected = 0.0  # [rad]
        if model != MODELS[0]:
            expected = angles[jitter_index]
            jitter_index += 1
        assert effector.vscmgOutMsgs[index].read().theta == pytest.approx(expected)  # [rad]
