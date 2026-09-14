#
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
#
"""Verify that C++ naming infrastructure preserves the existing Python API."""

import pytest

from Basilisk.architecture import bskLogging
from Basilisk.simulation import hingedRigidBodyStateEffector, spacecraft, stateArchitecture


@pytest.mark.parametrize("custom_names", [False, True])
def test_setup_time_names_remain_valid(custom_names):
    """Names captured during setup still identify the original states after registration."""
    manager = stateArchitecture.DynParamManager()
    panels = [hingedRigidBodyStateEffector.HingedRigidBodyStateEffector() for _ in range(2)]
    initial_angles = [0.125, -0.25]  # [rad]
    for index, panel in enumerate(panels):
        panel.thetaInit = initial_angles[index]
        if custom_names:
            panel.nameOfThetaState = f"panel{index}Angle"
            panel.nameOfThetaDotState = f"panel{index}Rate"
    captured_names = [panel.nameOfThetaState for panel in panels]
    assert captured_names[0] != captured_names[1]

    for panel in panels:
        panel.registerStates(manager)

    assert captured_names == [panel.nameOfThetaState for panel in panels]
    for name, angle in zip(captured_names, initial_angles):
        state = manager.getStateObject(name)
        assert state.getName() == name
        assert state.getState()[0][0] == angle


def test_public_policy_flag_keeps_preparation_internal():
    """Python selects either policy while the preparation protocol remains internal."""
    manager = stateArchitecture.DynParamManager()
    panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    assert manager.useManagerLocalEffectorNames is False
    manager.useManagerLocalEffectorNames = True
    assert manager.useManagerLocalEffectorNames is True
    manager.useManagerLocalEffectorNames = False
    assert manager.useManagerLocalEffectorNames is False
    assert not hasattr(manager, "setEffectorNamingPolicy")
    assert not hasattr(manager, "requestEffectorNames")
    assert not hasattr(manager, "resolveEffectorNames")
    assert not hasattr(manager, "registerEffectorState")
    assert not hasattr(manager, "createEffectorProperty")
    assert not hasattr(manager, "cancelEffectorNames")
    assert not hasattr(panel, "collectEffectorNames")
    assert not hasattr(panel, "cancelEffectorNames")
    assert not hasattr(panel, "bindAttachedDynamicEffectors")


@pytest.mark.parametrize("local", [False, True])
def test_public_policy_cannot_change_after_initialization(local):
    """Changing the flag cannot invalidate registered states or resolved names."""
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    vehicle.dynManager.useManagerLocalEffectorNames = local
    vehicle.addStateEffector(panel)
    vehicle.initializeDynamics()
    original = panel.nameOfThetaState
    vehicle.dynManager.useManagerLocalEffectorNames = local
    with pytest.raises(bskLogging.BasiliskError, match="select the naming policy before"):
        vehicle.dynManager.useManagerLocalEffectorNames = not local
    assert vehicle.dynManager.useManagerLocalEffectorNames is local
    assert panel.nameOfThetaState == original


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
