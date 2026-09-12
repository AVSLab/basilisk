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

"""Exercise non-finite configuration rejection and robust axis/noise arithmetic."""

import numpy as np
import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import (
    extForceTorque,
    igbmNoiseStateEffector,
    linearTranslationNDOFStateEffector,
    linearTranslationOneDOFStateEffector,
    meanRevertingNoiseStateEffector,
    spacecraft,
    spinningBodyNDOFStateEffector,
    spinningBodyOneDOFStateEffector,
    spinningBodyTwoDOFStateEffector,
)
from Basilisk.utilities import RigidBodyKinematics, SimulationBaseClass, macros


NONFINITE = [np.nan, np.inf, -np.inf]
NOISE = [meanRevertingNoiseStateEffector.MeanRevertingNoiseStateEffector,
         igbmNoiseStateEffector.IgbmNoiseStateEffector]


def make_model(kind):
    """Return a valid effector and the object holding its configuration."""
    if kind == "spin1":
        effector = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
        effector.sHat_S = [1.0, 0.0, 0.0]  # [-]
    elif kind == "spin2":
        effector = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
        effector.mass1 = 1.0  # [kg]
        effector.IS1PntSc1_S1 = np.eye(3).tolist()  # [kg*m^2]
    elif kind == "spinN":
        effector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
        body = spinningBodyNDOFStateEffector.SpinningBody()
        effector.addSpinningBody(body)
        return effector, body
    elif kind == "translate1":
        effector = linearTranslationOneDOFStateEffector.LinearTranslationOneDOFStateEffector()
    elif kind == "translateN":
        effector = linearTranslationNDOFStateEffector.LinearTranslationNDOFStateEffector()
        body = linearTranslationNDOFStateEffector.TranslatingBody()
        body.setMass(1.0)  # [kg]
        effector.addTranslatingBody(body)
        return effector, body
    else:
        effector = spacecraft.Spacecraft()
        return effector, effector.hub
    return effector, effector


def simulation_for(effector, kind, priority=None):
    """Attach a configured effector without requiring its scheduled Reset."""
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    sim.CreateNewProcess("process").addTask(sim.CreateNewTask("task", macros.sec2nano(0.01)))  # [s]
    parent = effector if kind == "hub" else spacecraft.Spacecraft()
    if kind != "hub":
        parent.addStateEffector(effector)
    sim.AddModelToTask("task", parent)
    if priority is not None and kind != "hub":
        sim.AddModelToTask("task", effector, priority)
    return sim, parent


PUBLIC_FIELDS = {
    "spin1": ["mass", "k", "c", "thetaInit", "thetaDotInit", "sHat_S", "r_SB_B",
              "r_ScS_S", "IPntSc_S", "dcm_S0B"],
    "spin2": ["mass1", "mass2", "k1", "k2", "c1", "c2", "theta1Init", "theta2Init",
              "theta1DotInit", "theta2DotInit", "s1Hat_S1", "s2Hat_S2", "r_S1B_B",
              "r_S2S1_S1", "r_Sc1S1_S1", "r_Sc2S2_S2", "IS1PntSc1_S1", "IS2PntSc2_S2",
              "dcm_S10B", "dcm_S20S1"],
    "hub": ["mHub", "r_BcB_B", "r_CN_NInit", "v_CN_NInit", "sigma_BNInit", "omega_BN_BInit",
            "IHubPntBc_B"],
}


def set_nonfinite_entry(config, field, bad):
    """Replace a scalar or one vector/matrix entry through its public configuration field."""
    current = np.asarray(getattr(config, field))
    if current.ndim:
        changed = current.copy()
        changed.flat[0] = bad
        setattr(config, field, changed.tolist())
    else:
        setattr(config, field, bad)


@pytest.mark.parametrize("kind,field", [(kind, field) for kind, fields in PUBLIC_FIELDS.items() for field in fields])
@pytest.mark.parametrize("bad", NONFINITE)
@pytest.mark.parametrize("path", ["reset", "attached"])
def test_nonfinite_public_configuration(kind, field, bad, path):
    """Reject non-finite scalars and individual vector/matrix entries through both paths."""
    effector, config = make_model(kind)
    set_nonfinite_entry(config, field, bad)
    with pytest.raises(BasiliskError, match=field):
        if path == "reset":
            effector.Reset(0)
        else:
            sim, parent = simulation_for(effector, kind)
            sim.InitializeSimulation()


@pytest.mark.parametrize("field", PUBLIC_FIELDS["hub"])
@pytest.mark.parametrize("bad", NONFINITE)
@pytest.mark.parametrize("path", ["reset", "initialize"])
@pytest.mark.parametrize("with_dynamic_effector", [False, True])
def test_nonfinite_point_mass_configuration(field, bad, path, with_dynamic_effector):
    """Reject non-finite point-mass inputs before registering any hub states."""
    parent = spacecraft.Spacecraft()
    parent.pointMassTranslationalOnly = True
    force = extForceTorque.ExtForceTorque()
    if with_dynamic_effector:
        parent.addDynamicEffector(force)
    set_nonfinite_entry(parent.hub, field, bad)
    with pytest.raises(BasiliskError, match=field):
        if path == "reset":
            parent.Reset(0)
        else:
            sim, parent = simulation_for(parent, "hub")
            sim.InitializeSimulation()
    assert parent.dynManager.getStateObject(parent.hub.nameOfHubPosition) is None
    assert parent.dynManager.getStateObject(parent.hub.nameOfHubVelocity) is None
    assert parent.dynManager.getStateObject(parent.hub.nameOfHubSigma) is None
    assert parent.dynManager.getStateObject(parent.hub.nameOfHubOmega) is None


@pytest.mark.parametrize("point_mass", [False, True])
@pytest.mark.parametrize("with_dynamic_effector", [False, True])
@pytest.mark.parametrize("inertia", [np.zeros((3, 3)), np.diag([1.0, 0.0, 0.0])])  # [kg*m^2]
def test_point_mass_preserves_finite_singular_inertia_support(point_mass, with_dynamic_effector, inertia):
    """Point-mass propagation accepts finite inertia tensors rejected by full hub validation."""
    parent = spacecraft.Spacecraft()
    parent.pointMassTranslationalOnly = point_mass
    parent.hub.IHubPntBc_B = inertia.tolist()
    parent.hub.mHub = 2.0  # [kg]
    position = np.array([1.0, 2.0, 3.0])  # [m]
    velocity = np.array([0.1, -0.2, 0.3])  # [m/s]
    attitude = [0.1, -0.2, 0.3]  # [-]
    angular_velocity = [0.2, -0.3, 0.1]  # [rad/s]
    parent.hub.r_CN_NInit = position.tolist()
    parent.hub.v_CN_NInit = velocity.tolist()
    parent.hub.sigma_BNInit = attitude
    parent.hub.omega_BN_BInit = angular_velocity
    force = extForceTorque.ExtForceTorque()
    acceleration = np.zeros(3)  # [m/s^2]
    if with_dynamic_effector:
        force.extForce_B = [2.0, -1.0, 3.0]  # [N]
        parent.addDynamicEffector(force)
        acceleration = (RigidBodyKinematics.MRP2C(attitude).T
                        @ np.asarray(force.extForce_B).ravel() / parent.hub.mHub)
    sim, parent = simulation_for(parent, "hub")
    if not point_mass:
        with pytest.raises(BasiliskError, match="IHubPntBc_B"):
            sim.InitializeSimulation()
        return

    sim.InitializeSimulation()
    stop_time = 0.02  # [s]
    sim.ConfigureStopTime(macros.sec2nano(stop_time))
    sim.ExecuteSimulation()
    output = parent.scStateOutMsg.read()
    np.testing.assert_allclose(output.r_BN_N, position + velocity*stop_time + 0.5*acceleration*stop_time**2,
                               rtol=1e-13, atol=1e-14)  # [m]
    np.testing.assert_allclose(output.v_BN_N, velocity + acceleration*stop_time,
                               rtol=1e-13, atol=1e-14)  # [m/s]
    np.testing.assert_array_equal(output.sigma_BN, attitude)
    np.testing.assert_array_equal(output.omega_BN_B, angular_velocity)
    np.testing.assert_array_equal(np.asarray(parent.scMassOutMsg.read().ISC_PntB_B).reshape(3, 3), inertia)


@pytest.mark.parametrize("point_mass", [False, True])
@pytest.mark.parametrize("mass", [-1.0, 0.0])  # [kg]
def test_hub_mass_must_remain_positive(point_mass, mass):
    """Sharing finite-value validation preserves the positive mass bound in both modes."""
    parent = spacecraft.Spacecraft()
    parent.pointMassTranslationalOnly = point_mass
    parent.hub.mHub = mass
    with pytest.raises(BasiliskError, match="mHub"):
        parent.Reset(0)


@pytest.mark.parametrize("offset", [1.0, 1e-200, np.nextafter(0.0, 1.0)])  # [m]
def test_point_mass_rejects_any_nonzero_center_offset(offset):
    """The point-mass zero-offset requirement must not depend on a possibly underflowing norm."""
    parent = spacecraft.Spacecraft()
    parent.pointMassTranslationalOnly = True
    parent.hub.r_BcB_B = [offset, 0.0, 0.0]
    with pytest.raises(BasiliskError, match="r_BcB_B.*zero"):
        parent.Reset(0)


DEFERRED_FIELDS = {
    "spinN": ["ThetaInit", "ThetaDotInit", "R_SP_P", "R_ScS_S", "ISPntSc_S", "DCM_S0P"],
    "translate1": ["RhoInit", "RhoDotInit", "R_FcF_F", "R_F0B_B", "IPntFc_F", "DCM_FB"],
    "translateN": ["RhoInit", "RhoDotInit", "R_FcF_F", "R_F0P_P", "IPntFc_F", "DCM_FP"],
}


@pytest.mark.parametrize("kind,field", [(kind, field) for kind, fields in DEFERRED_FIELDS.items() for field in fields])
@pytest.mark.parametrize("bad", NONFINITE)
@pytest.mark.parametrize("path", ["reset", "attached"])
def test_nonfinite_deferred_configuration(kind, field, bad, path):
    """Validate private configuration before it is used to initialize states or properties."""
    effector, config = make_model(kind)
    if field.startswith(("DCM", "I")):
        value = np.eye(3)
        value[0, 0] = bad
        value = value.tolist()
    elif field.startswith("R_"):
        value = [bad, 0.0, 0.0]
    else:
        value = bad
    getattr(config, "set" + field)(value)
    with pytest.raises(BasiliskError):
        if path == "reset":
            effector.Reset(0)
        else:
            sim, parent = simulation_for(effector, kind)
            sim.InitializeSimulation()


@pytest.mark.parametrize("kind", ["spinN", "translate1", "translateN"])
@pytest.mark.parametrize("field", ["Mass", "K", "C"])
@pytest.mark.parametrize("bad", NONFINITE)
def test_nonfinite_bounded_setters(kind, field, bad):
    """Range-checked setters must reject non-finite values without corrupting valid settings."""
    effector, config = make_model(kind)
    getter = getattr(config, "get" + field, None)
    before = getter() if getter else None
    with pytest.raises(BasiliskError):
        getattr(config, "set" + field)(bad)
    if getter:
        assert getter() == before
    effector.Reset(0)
    sim, parent = simulation_for(effector, kind)
    sim.InitializeSimulation()


AXES = [("spin1", "sHat_S"), ("spin2", "s1Hat_S1"), ("spin2", "s2Hat_S2"),
        ("spinN", "SHat_S"), ("translate1", "FHat_B"), ("translateN", "FHat_P")]


def configure_axis(config, name, value):
    """Set a public axis or call the corresponding configuration setter."""
    if hasattr(config, "set" + name):
        getattr(config, "set" + name)(value)
    else:
        setattr(config, name, value)


def read_axis(config, name):
    """Read a normalized axis through its public API."""
    if hasattr(config, "get" + name):
        return np.asarray(getattr(config, "get" + name)()).ravel()
    return np.asarray(getattr(config, name)).ravel()


@pytest.mark.parametrize("kind,axis", AXES)
@pytest.mark.parametrize("magnitude", [2.0, 1e200, np.finfo(float).max])  # [-]
@pytest.mark.parametrize("priority", [None, -100, 100], ids=["attached", "after", "before"])
def test_large_finite_axes(kind, axis, magnitude, priority):
    """Normalize large finite axes accurately even when their unscaled norms overflow."""
    effector, config = make_model(kind)
    configure_axis(config, axis, [magnitude, -magnitude, magnitude])
    sim, parent = simulation_for(effector, kind, priority)
    sim.InitializeSimulation()
    expected = np.array([1.0, -1.0, 1.0]) / np.sqrt(3.0)  # [-]
    np.testing.assert_allclose(read_axis(config, axis), expected, atol=1e-15)
    effector.Reset(0)
    effector.Reset(0)
    np.testing.assert_allclose(read_axis(config, axis), expected, atol=1e-15)
    sim.ConfigureStopTime(macros.sec2nano(0.02))  # [s]
    sim.ExecuteSimulation()
    hub_state = parent.dynManager.getStateObject(parent.hub.nameOfHubSigma)
    assert np.isfinite(hub_state.getState()).all()


@pytest.mark.parametrize("kind,axis", AXES)
@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf, 0.0, 0.01])  # [-]
def test_invalid_axes_preserve_configuration(kind, axis, bad):
    """Reject non-finite or undersized axes before normalizing or overwriting prior settings."""
    effector, config = make_model(kind)
    previous = read_axis(config, axis).copy()
    if hasattr(config, "set" + axis):
        with pytest.raises(BasiliskError):
            configure_axis(config, axis, [bad, 0.0, 0.0])
        np.testing.assert_array_equal(read_axis(config, axis), previous)
    else:
        configure_axis(config, axis, [bad, 0.0, 0.0])
        with pytest.raises(BasiliskError):
            effector.Reset(0)
        np.testing.assert_array_equal(read_axis(config, axis), [bad, 0.0, 0.0])


@pytest.mark.parametrize("kind", ["spin1", "spin2"])
def test_failed_validation_does_not_normalize_axes(kind):
    """A later validation failure must leave valid but unnormalized axis input intact."""
    effector, config = make_model(kind)
    axis = "sHat_S" if kind == "spin1" else "s1Hat_S1"
    configure_axis(config, axis, [2.0, 0.0, 0.0])  # [-]
    setattr(config, "dcm_S0B" if kind == "spin1" else "dcm_S10B", np.zeros((3, 3)).tolist())
    with pytest.raises(BasiliskError):
        effector.Reset(0)
    np.testing.assert_array_equal(read_axis(config, axis), [2.0, 0.0, 0.0])


@pytest.mark.parametrize("kind", ["spin2", "spinN", "translateN"])
@pytest.mark.parametrize("path", ["reset", "attached"])
def test_combined_mass_overflow(kind, path):
    """Reject overflowing mass sums before center-of-mass or joint-matrix calculations."""
    effector, config = make_model(kind)
    mass = np.finfo(float).max  # [kg]
    if kind == "spin2":
        config.mass1 = mass
        config.mass2 = mass
    else:
        config.setMass(mass)
        if kind == "spinN":
            second = spinningBodyNDOFStateEffector.SpinningBody()
            second.setMass(mass)
            effector.addSpinningBody(second)
        else:
            second = linearTranslationNDOFStateEffector.TranslatingBody()
            second.setMass(mass)
            effector.addTranslatingBody(second)
    with pytest.raises(BasiliskError, match="mass.*finite"):
        if path == "reset":
            effector.Reset(0)
        else:
            sim, parent = simulation_for(effector, kind)
            sim.InitializeSimulation()


@pytest.mark.parametrize("constructor,field", [
    (constructor, field) for constructor in NOISE
    for field in (["TimeConstant", "StationaryStd", "StateValue"]
                  + (["Mean"] if constructor is NOISE[1] else []))
])
@pytest.mark.parametrize("bad", NONFINITE)
@pytest.mark.parametrize("registered", [False, True])
def test_noise_setters_are_transactional(constructor, field, bad, registered):
    """Reject invalid noise parameters before changing stored configuration or a live state."""
    effector = constructor()
    effector.setStateValue(0.25)  # [-]
    manager = spacecraft.DynParamManager()
    if registered:
        effector.registerStates(manager)
    previous = getattr(effector, "get" + field)()
    with pytest.raises(BasiliskError):
        getattr(effector, "set" + field)(bad)
    assert getattr(effector, "get" + field)() == previous
    assert effector.getStateValue() == 0.25  # [-]


@pytest.mark.parametrize("constructor", NOISE)
def test_noise_reset_preserves_current_state(constructor):
    """The inherited Reset remains safe before registration and preserves live noise state."""
    effector = constructor()
    effector.Reset(0)
    manager = spacecraft.DynParamManager()
    effector.registerStates(manager)
    effector.setStateValue(0.25)  # [-]
    effector.Reset(0)
    effector.Reset(0)
    assert effector.getStateValue() == 0.25  # [-]


@pytest.mark.parametrize("constructor", NOISE)
def test_noise_small_time_constant_and_zero_noise(constructor):
    """A zero state at equilibrium with zero diffusion remains finite for tiny positive tau."""
    effector = constructor()
    effector.setTimeConstant(1e-320)  # [s]
    manager = spacecraft.DynParamManager()
    effector.registerStates(manager)
    effector.computeDerivatives(0.0, [0.0]*3, [0.0]*3, [0.0]*3)
    state = manager.getStateObject(effector.getStateName())
    np.testing.assert_array_equal(state.getStateDeriv(), [[0.0]])
    np.testing.assert_array_equal(state.getStateDiffusion(0), [[0.0]])


@pytest.mark.parametrize("mean,std,tau,expected_drift,expected_diffusion", [
    (1.0, 1e300, 1.0, 0.0, np.sqrt(2.0)),
    (1e200, 1e200, 1e200, 1.0, 1e-100),
])  # mean/std: [-], tau: [s], drift: [1/s], diffusion: [1/sqrt(s)]
def test_igbm_large_stationary_parameters(mean, std, tau, expected_drift, expected_diffusion):
    """Avoid overflow of squared stationary parameters when the resulting coefficients are finite."""
    effector = igbmNoiseStateEffector.IgbmNoiseStateEffector()
    effector.setMean(mean)
    effector.setStationaryStd(std)
    effector.setTimeConstant(tau)
    effector.setStateValue(0.0)
    manager = spacecraft.DynParamManager()
    effector.registerStates(manager)
    effector.computeDerivatives(0.0, [0.0]*3, [0.0]*3, [0.0]*3)
    state = manager.getStateObject(effector.getStateName())
    np.testing.assert_allclose(state.getStateDeriv(), [[expected_drift]], rtol=1e-14, atol=0.0)
    np.testing.assert_allclose(state.getStateDiffusion(0), [[expected_diffusion]], rtol=1e-14, atol=0.0)


@pytest.mark.parametrize("constructor", NOISE)
def test_noise_unrepresentable_derivative_fails_before_write(constructor):
    """Report genuine arithmetic overflow without publishing a non-finite derivative."""
    effector = constructor()
    manager = spacecraft.DynParamManager()
    effector.registerStates(manager)
    state = manager.getStateObject(effector.getStateName())
    effector.computeDerivatives(0.0, [0.0]*3, [0.0]*3, [0.0]*3)
    previous = np.asarray(state.getStateDeriv()).copy()
    effector.setStateValue(1.0)  # [-]
    effector.setTimeConstant(1e-320)  # [s]
    with pytest.raises(BasiliskError, match="non-finite drift or diffusion"):
        effector.computeDerivatives(0.0, [0.0]*3, [0.0]*3, [0.0]*3)
    np.testing.assert_array_equal(state.getStateDeriv(), previous)
