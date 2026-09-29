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

"""Verify fixed state layouts and synchronized integration through public APIs."""

import gc
import weakref

import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import linearTranslationNDOFStateEffector
from Basilisk.simulation import prescribedMotionStateEffector
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import spacecraft
from Basilisk.simulation import spacecraftChargingDynamics
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import thrusterStateEffector
from Basilisk.simulation import vscmgStateEffector


def reset_charging_dynamics(dynamic_object):
    """Initialize a compact dynamic object through its public lifecycle."""
    dynamic_object.Reset(0)


def test_primary_retains_bound_secondary_proxy_and_binding():
    """Keep a bound integration group valid after external references vanish."""
    primary = spacecraftChargingDynamics.SpacecraftChargingDynamics()
    secondary = spacecraftChargingDynamics.SpacecraftChargingDynamics()
    primary_ref = weakref.ref(primary)
    secondary_ref = weakref.ref(secondary)

    reset_charging_dynamics(primary)
    reset_charging_dynamics(secondary)
    primary.syncDynamicsIntegration(secondary)
    primary.integrateState(0)

    secondary = None
    gc.collect()

    assert secondary_ref() is not None
    assert primary.integrator.getDynamicsCount() == 2
    primary.integrateState(0)

    primary = None
    gc.collect()
    assert primary_ref() is None
    assert secondary_ref() is None


def test_object_tolerances_survive_external_secondary_reference_release():
    """Keep tolerance overrides attached to primary-retained secondaries."""
    primary = spacecraft.Spacecraft()
    secondary = spacecraft.Spacecraft()
    secondary_ref = weakref.ref(secondary)
    integrator = svIntegrators.svIntegratorRKF45(primary)
    primary.setIntegrator(integrator)
    primary.syncDynamicsIntegration(secondary)

    state_name = "retainedState"
    relative_tolerance = 2.0e-6  # [-]
    absolute_tolerance = 3.0e-9  # [-]
    integrator.setRelativeTolerance(
        secondary, state_name, relative_tolerance
    )
    integrator.setAbsoluteTolerance(
        secondary, state_name, absolute_tolerance
    )

    secondary = None
    gc.collect()

    retained_secondary = secondary_ref()
    assert retained_secondary is not None
    assert (
        integrator.getRelativeTolerance(retained_secondary, state_name)
        == relative_tolerance
    )
    assert (
        integrator.getAbsoluteTolerance(retained_secondary, state_name)
        == absolute_tolerance
    )

    replacement_secondary = spacecraft.Spacecraft()
    primary.syncDynamicsIntegration(replacement_secondary)
    assert integrator.getRelativeTolerance(replacement_secondary, state_name) is None
    assert integrator.getAbsoluteTolerance(replacement_secondary, state_name) is None


def test_synchronization_rejects_bound_integrator():
    """Reject synchronization after either participating integrator has bound."""
    primary = spacecraftChargingDynamics.SpacecraftChargingDynamics()
    secondary = spacecraftChargingDynamics.SpacecraftChargingDynamics()

    reset_charging_dynamics(primary)
    primary.integrateState(0)

    with pytest.raises(BasiliskError, match="before their first integration step"):
        primary.syncDynamicsIntegration(secondary)
    assert primary.integrator.getDynamicsCount() == 1
    assert secondary.getIntegrationOwner() is None


def test_repeated_spacecraft_reset_freezes_topology():
    """Keep buffer topology stable and reject later collection mutation."""
    dynamic_object = spacecraft.Spacecraft()
    dynamic_object.pointMassTranslationalOnly = True
    dynamic_object.hub.mHub = 1.0  # [kg]

    dynamic_object.Reset(0)
    dynamic_object.Reset(0)

    assert dynamic_object.dynManager.statesAreFinalized()

    late_effector = (
        linearTranslationNDOFStateEffector
        .LinearTranslationNDOFStateEffector()
    )
    late_effector.addTranslatingBody(
        linearTranslationNDOFStateEffector.TranslatingBody()
    )
    with pytest.raises(RuntimeError, match="finalized state topology"):
        dynamic_object.addStateEffector(late_effector)


def test_nested_effector_collection_rejects_post_registration_mutation():
    """Freeze a state effector's dimension-defining collection before mutation."""
    dynamic_object = spacecraft.Spacecraft()
    dynamic_object.hub.mHub = 1.0  # [kg]
    dynamic_object.hub.IHubPntBc_B = [[1.0, 0.0, 0.0],  # [kg*m^2]
                                      [0.0, 1.0, 0.0],
                                      [0.0, 0.0, 1.0]]
    effector = (
        linearTranslationNDOFStateEffector
        .LinearTranslationNDOFStateEffector()
    )
    body = linearTranslationNDOFStateEffector.TranslatingBody()
    body.setMass(1.0)  # [kg]
    effector.addTranslatingBody(body)
    dynamic_object.addStateEffector(effector)
    dynamic_object.Reset(0)

    with pytest.raises(
        RuntimeError,
        match="cannot change a state effector after state registration",
    ):
        effector.addTranslatingBody(
            linearTranslationNDOFStateEffector.TranslatingBody()
        )

    dynamic_object.Reset(0)


def test_recursive_effector_collection_freezes_after_registration():
    """Freeze dimension-defining collections on recursively registered children."""
    parent = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    child = (
        linearTranslationNDOFStateEffector
        .LinearTranslationNDOFStateEffector()
    )
    child.addTranslatingBody(
        linearTranslationNDOFStateEffector.TranslatingBody()
    )
    parent.addStateEffector(child)
    parent.freezeTopology()

    with pytest.raises(
        RuntimeError,
        match="cannot change a state effector after state registration",
    ):
        child.addTranslatingBody(
            linearTranslationNDOFStateEffector.TranslatingBody()
        )


@pytest.mark.parametrize(
    ("effector", "config", "add", "collection", "field"),
    [
        (
            reactionWheelStateEffector.ReactionWheelStateEffector(),
            reactionWheelStateEffector.RWConfigPayload(),
            "addReactionWheel",
            "ReactionWheelData",
            "Omega",
        ),
        (
            thrusterStateEffector.ThrusterStateEffector(),
            thrusterStateEffector.THRSimConfig(),
            "addThruster",
            "thrusterData",
            "MaxThrust",
        ),
        (
            vscmgStateEffector.VSCMGStateEffector(),
            vscmgStateEffector.VSCMGConfigMsgPayload(),
            "AddVSCMG",
            "VSCMGData",
            "Omega",
        ),
    ],
)
def test_dimension_defining_collection_bindings_are_guarded_live_views(
    effector,
    config,
    add,
    collection,
    field,
):
    """Preserve pre-reset collection mutation and guard it after finalization."""
    setattr(config, field, 1.0)  # [rad/s] Omega; [N] MaxThrust
    getattr(effector, add)(config)
    exposed = getattr(effector, collection)
    exposed.append(config)
    assert len(getattr(effector, collection)) == 2

    replacement = type(config)()
    setattr(replacement, field, 2.0)  # [rad/s] Omega; [N] MaxThrust
    exposed[0] = replacement
    assert [getattr(item, field) for item in exposed] == [2.0, 1.0]

    setattr(effector, collection, [config, replacement])
    assert [getattr(item, field) for item in exposed] == [1.0, 2.0]

    exposed[:] = [replacement, config]
    assert [getattr(item, field) for item in exposed[::-1]] == [1.0, 2.0]
    with pytest.raises(ValueError, match="cannot change"):
        exposed[:] = [config]
    assert [getattr(item, field) for item in exposed] == [2.0, 1.0]

    effector.freezeTopology()
    with pytest.raises(
        RuntimeError,
        match="cannot change a state effector after state registration",
    ):
        exposed.append(config)
    with pytest.raises(
        RuntimeError,
        match="cannot change a state effector after state registration",
    ):
        exposed[0] = replacement


def test_vscmg_collection_elements_remain_live():
    """Propagate field edits through the VSCMG compatibility collection."""
    effector = vscmgStateEffector.VSCMGStateEffector()
    config = vscmgStateEffector.VSCMGConfigMsgPayload()
    config.Omega = 1.0  # [rad/s]
    effector.AddVSCMG(config)

    effector.VSCMGData[0].Omega = 2.0  # [rad/s]

    assert effector.VSCMGData[0].Omega == 2.0


@pytest.mark.parametrize("accessor", ("getVSCMGAt", "collection"))
def test_vscmg_element_keeps_effector_alive(accessor):
    """Retain the native owner until its last borrowed element is released."""
    effector = vscmgStateEffector.VSCMGStateEffector()
    config = vscmgStateEffector.VSCMGConfigMsgPayload()
    config.Omega = 1.0  # [rad/s]
    effector.AddVSCMG(config)
    item = (
        effector.getVSCMGAt(0)
        if accessor == "getVSCMGAt"
        else effector.VSCMGData[0]
    )
    owner = weakref.ref(effector)
    del effector
    gc.collect()

    # Check the owner before dereferencing potentially destroyed native storage.
    assert owner() is not None
    item.Omega = 2.0  # [rad/s]
    assert owner().getVSCMGAt(0).Omega == 2.0

    del item
    gc.collect()
    assert owner() is None


@pytest.mark.parametrize(
    ("effector", "config", "add"),
    [
        (
            reactionWheelStateEffector.ReactionWheelStateEffector(),
            reactionWheelStateEffector.RWConfigPayload(),
            "addReactionWheel",
        ),
        (
            thrusterStateEffector.ThrusterStateEffector(),
            thrusterStateEffector.THRSimConfig(),
            "addThruster",
        ),
        (
            vscmgStateEffector.VSCMGStateEffector(),
            vscmgStateEffector.VSCMGConfigMsgPayload(),
            "AddVSCMG",
        ),
    ],
)
def test_dimension_defining_mutators_translate_topology_errors(
    effector,
    config,
    add,
):
    """Translate native topology errors at each derived SWIG boundary."""
    effector.freezeTopology()
    with pytest.raises(
        RuntimeError,
        match="cannot change a state effector after state registration",
    ):
        getattr(effector, add)(config)


def test_reaction_wheel_model_cannot_change_after_registration():
    """Reject payload mutations that would change the theta-state topology."""
    dynamic_object = spacecraft.Spacecraft()
    dynamic_object.hub.mHub = 1.0  # [kg]
    dynamic_object.hub.IHubPntBc_B = [[1.0, 0.0, 0.0],  # [kg*m^2]
                                      [0.0, 1.0, 0.0],
                                      [0.0, 0.0, 1.0]]
    effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    wheel = reactionWheelStateEffector.RWConfigPayload()
    effector.addReactionWheel(wheel)
    dynamic_object.addStateEffector(effector)
    dynamic_object.Reset(0)

    wheel.RWModel = reactionWheelStateEffector.JitterSimple
    with pytest.raises(BasiliskError, match="wheel models cannot change"):
        dynamic_object.Reset(0)


def test_integrator_replacement_preserves_synchronized_dynamics():
    """A replacement method advances the same primary and secondary states."""
    primary = spacecraft.Spacecraft()
    secondary = spacecraft.Spacecraft()
    primary.hub.v_CN_NInit = [[1.0], [0.0], [0.0]]  # [m/s]
    secondary.hub.v_CN_NInit = [[2.0], [0.0], [0.0]]  # [m/s]
    primary.syncDynamicsIntegration(secondary)
    primary.Reset(0)
    secondary.Reset(0)
    primary.integrateState(1_000_000_000)  # [ns]
    replacement = svIntegrators.svIntegratorRK2(primary)
    primary.setIntegrator(replacement)
    primary.integrateState(2_000_000_000)  # [ns]
    assert replacement.getDynamicsCount() == 2
    assert primary.dynManager.getStateObject("hubPosition").getState()[0][0] == pytest.approx(2.0)
    assert secondary.dynManager.getStateObject("hubPosition").getState()[0][0] == pytest.approx(4.0)
