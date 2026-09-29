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

import gc
import inspect
import subprocess
import sys
import weakref

import pytest

from Basilisk.simulation import dynParamManager, spacecraft, stateArchitecture
from Basilisk import hasBuildFeature


EXPECTED_COMPATIBILITY_EXPORTS = frozenset(
    {
        "DynParamManager",
        "StateData",
        "MatrixShape",
        "StateSpec",
        "ErrorControlMode_WholeState",
        "ErrorControlMode_PerComponent",
        "StateUpdateKind_Euclidean",
        "StateUpdateKind_Special",
    }
)



def matrix_shape(rows, cols):
    shape = stateArchitecture.MatrixShape()
    shape.rows = rows
    shape.cols = cols
    return shape


def state_spec(rows, cols, noise_count=0):
    spec = stateArchitecture.StateSpec()
    spec.state = matrix_shape(rows, cols)
    spec.derivative = matrix_shape(rows, cols)
    spec.diffusionTangent = matrix_shape(rows, cols)
    spec.noiseCount = noise_count
    return spec


def register_reference_topology(manager):
    alpha = manager.registerStateSpec("alpha", state_spec(2, 1, noise_count=2))
    beta = manager.registerStateSpec("beta", state_spec(1, 2, noise_count=1))
    manager.registerSharedNoiseSource([(alpha, 1), (beta, 0)])
    return alpha, beta


def finalized_manager():
    manager = stateArchitecture.DynParamManager()
    alpha, beta = register_reference_topology(manager)
    alpha.setState([[1.0], [2.0]])
    alpha.setDerivative([[3.0], [4.0]])
    alpha.setDiffusion([[5.0], [6.0]], 0)
    alpha.setDiffusion([[7.0], [8.0]], 1)
    beta.setState([[9.0, 10.0]])
    manager.finalizeStates()
    return manager, alpha, beta


def assert_committed_values(alpha, beta):
    assert alpha.getState() == [[1.0], [2.0]]
    assert alpha.getStateDeriv() == [[3.0], [4.0]]
    assert alpha.getStateDiffusion(0) == [[5.0], [6.0]]
    assert alpha.getStateDiffusion(1) == [[7.0], [8.0]]
    assert beta.getState() == [[9.0, 10.0]]


def test_finalization_and_repeated_registration_preserve_values():
    manager = stateArchitecture.DynParamManager()

    # Native buffer access belongs to integration code, not the Python module API.
    assert not hasattr(manager, "getStateRegistry")
    assert not hasattr(dynParamManager, "StateRegistry")

    assert not hasattr(
        stateArchitecture, "_dyn_param_manager_register_state"
    )
    assert not hasattr(
        stateArchitecture, "_dyn_param_manager_get_state_object"
    )
    assert not manager.statesAreFinalized()

    alpha, beta = register_reference_topology(manager)
    alpha.setState([[1.0], [2.0]])
    beta.setState([[3.0, 4.0]])
    manager.finalizeStates()

    assert manager.statesAreFinalized()
    assert manager.getStateObject("alpha").getState() == [[1.0], [2.0]]

    repeated_alpha, repeated_beta = register_reference_topology(manager)
    repeated_alpha.setState([[11.0], [12.0]])
    repeated_beta.setState([[13.0, 14.0]])
    manager.finalizeStates()

    assert alpha.getState() == [[11.0], [12.0]]
    assert beta.getState() == [[13.0, 14.0]]


def test_registration_overloads_preserve_legacy_keyword_names():
    manager = stateArchitecture.DynParamManager()
    legacy = manager.registerState(nRow=1, nCol=1, stateName="legacy")
    modern = manager.registerStateSpec(
        stateName="modern", spec=state_spec(1, 1)
    )
    manager.finalizeStates()

    assert legacy.getName() == "legacy"
    assert modern.getName() == "modern"
    assert manager.registerState.__name__ == "registerState"
    assert manager.registerState.__qualname__.endswith(
        "DynParamManager.registerState"
    )
    assert inspect.signature(manager.registerState)


def test_state_architecture_reexports_canonical_proxy_classes():
    assert (
        set(dynParamManager._COMPATIBILITY_EXPORTS)
        == EXPECTED_COMPATIBILITY_EXPORTS
    )

    for name in EXPECTED_COMPATIBILITY_EXPORTS:
        assert getattr(stateArchitecture, name) is getattr(
            dynParamManager, name
        )


def test_spacecraft_uses_canonical_state_proxy_classes():
    dynamic_object = spacecraft.Spacecraft()
    manager = dynamic_object.dynManager

    for name in EXPECTED_COMPATIBILITY_EXPORTS:
        assert getattr(spacecraft, name) is getattr(dynParamManager, name)

    assert type(manager) is dynParamManager.DynParamManager
    assert isinstance(manager, stateArchitecture.DynParamManager)


@pytest.mark.parametrize(
    "modules",
    (
        (),
        ("stateArchitecture",),
        ("spacecraft",),
        ("StatefulSysModel",),
    ),
)
def test_canonical_proxy_protection_is_import_order_independent(modules):
    if "StatefulSysModel" in modules and not hasBuildFeature("mujoco"):
        pytest.skip("Requires Basilisk built with --mujoco True")
    worker = """
import importlib
import sys

for module_name in sys.argv[1:]:
    importlib.import_module(f"Basilisk.simulation.{module_name}")

from Basilisk.simulation import dynParamManager

shape = dynParamManager.MatrixShape()
try:
    shape.unregistered_attribute = 1
except ValueError:
    pass
else:
    raise AssertionError("canonical proxy unexpectedly accepted a new attribute")

assert (
    dynParamManager.MatrixShape.__setattr__.__module__
    == "Basilisk.architecture.swig_common_model"
)
"""
    result = subprocess.run(
        [sys.executable, "-c", worker, *modules],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("module", [dynParamManager, stateArchitecture])
def test_state_data_legacy_properties_forward_to_current_api(module):
    manager = module.DynParamManager()
    state = manager.registerStateSpec(
        "legacyProperties", state_spec(2, 1, noise_count=2)
    )
    manager.finalizeStates()

    state.state = [[1.0], [2.0]]
    state.stateDeriv = [[3.0], [4.0]]

    assert state.state == [[1.0], [2.0]]
    assert state.stateDeriv == [[3.0], [4.0]]
    assert state.stateName == "legacyProperties"
    assert state.perComponentErrorControl is False
    state.stateDiffusion[0] = [[5.0], [6.0]]
    state.stateDiffusion = (
        [[7.0], [8.0]],
        [[9.0], [10.0]],
    )
    assert len(state.stateDiffusion) == 2
    assert list(state.stateDiffusion) == [
        [[7.0], [8.0]],
        [[9.0], [10.0]],
    ]
    with pytest.raises(AttributeError):
        state.stateDiffusion.append([[11.0], [12.0]])
    with pytest.raises(AttributeError):
        state.stateName = "renamed"
    with pytest.raises(AttributeError):
        state.perComponentErrorControl = True






@pytest.mark.parametrize("accessor", ["register", "lookup"])
def test_state_proxy_keeps_manager_alive(accessor):
    manager = stateArchitecture.DynParamManager()
    manager_ref = weakref.ref(manager)
    registered = manager.registerStateSpec("owned", state_spec(1, 1))
    registered.setState([[4.0]])
    manager.finalizeStates()

    state = (
        registered
        if accessor == "register"
        else manager.getStateObject("owned")
    )
    del registered
    del manager
    gc.collect()

    assert manager_ref() is not None
    assert state.getName() == "owned"
    assert state.getState() == [[4.0]]

    del state
    gc.collect()
    assert manager_ref() is None


def test_legacy_noise_count_registration_and_repeated_epoch():
    manager = stateArchitecture.DynParamManager()
    state = manager.registerState(2, 1, "legacy")
    state.setNumNoiseSources(2)
    state.setDiffusion([[1.0], [2.0]], 0)
    state.setDiffusion([[3.0], [4.0]], 1)
    manager.finalizeStates()

    assert state.getNumNoiseSources() == 2
    assert state.getStateDiffusion(0) == [[1.0], [2.0]]
    assert state.getStateDiffusion(1) == [[3.0], [4.0]]

    state.setNumNoiseSources(2)
    with pytest.raises(RuntimeError, match="cannot change finalized topology"):
        state.setNumNoiseSources(1)

    repeated = manager.registerState(2, 1, "legacy")
    assert repeated.getName() == state.getName()
    assert repeated.getNumNoiseSources() == 2
    repeated.setNumNoiseSources(2)
    repeated.setDiffusion([[5.0], [6.0]], 1)
    manager.finalizeStates()
    assert state.getStateDiffusion(1) == [[5.0], [6.0]]

    repeated = manager.registerState(2, 1, "legacy")
    with pytest.raises(RuntimeError, match="cannot change finalized topology"):
        repeated.setNumNoiseSources(3)
    assert state.getNumNoiseSources() == 2
    assert state.getStateDiffusion(1) == [[5.0], [6.0]]


def test_shared_noise_rejects_state_from_another_manager():
    manager = stateArchitecture.DynParamManager()
    local = manager.registerStateSpec("local", state_spec(1, 1, noise_count=1))

    other = stateArchitecture.DynParamManager()
    foreign = other.registerStateSpec(
        "foreign", state_spec(1, 1, noise_count=1)
    )

    with pytest.raises(RuntimeError, match="does not belong"):
        manager.registerSharedNoiseSource([(foreign, 0)])

    manager.registerSharedNoiseSource([(local, 0)])
    manager.finalizeStates()


def test_shared_noise_rejects_multiple_sources_from_same_state():
    manager = stateArchitecture.DynParamManager()
    state = manager.registerStateSpec(
        "duplicateEndpointState", state_spec(1, 1, noise_count=2)
    )
    manager.registerSharedNoiseSource([(state, 0), (state, 1)])

    with pytest.raises(RuntimeError, match="same state"):
        manager.finalizeStates()







def test_state_setters_require_exact_matrix_dimensions():
    manager, alpha, _ = finalized_manager()

    with pytest.raises(RuntimeError, match=r"expected 2x1, observed 1x2"):
        alpha.setState([[11.0, 12.0]])
    with pytest.raises(RuntimeError, match=r"expected 2x1, observed 1x2"):
        alpha.setDerivative([[13.0, 14.0]])
    with pytest.raises(RuntimeError, match=r"expected 2x1, observed 1x2"):
        alpha.setDiffusion([[15.0, 16.0]], 0)
    with pytest.raises(RuntimeError, match="outside its 2 noise sources"):
        alpha.setDiffusion([[15.0], [16.0]], 2)

    assert_committed_values(alpha, manager.getStateObject("beta"))




def test_repeated_declarations_are_name_based_and_update_live_values():
    """Reordered and repeated declarations reuse named state handles."""
    manager, alpha, beta = finalized_manager()
    repeated_beta = manager.registerStateSpec("beta", state_spec(1, 2, noise_count=1))
    repeated_alpha = manager.registerStateSpec("alpha", state_spec(2, 1, noise_count=2))
    repeated_alpha.setState([[11.0], [12.0]])
    repeated_beta.setState([[13.0, 14.0]])
    assert alpha.getState() == [[11.0], [12.0]]
    assert beta.getState() == [[13.0, 14.0]]
    manager.finalizeStates()
    manager.finalizeStates()
    assert alpha.getState() == [[11.0], [12.0]]
    with pytest.raises(RuntimeError, match="Topology mismatch"):
        manager.registerStateSpec("alpha", state_spec(1, 2, noise_count=2))
    with pytest.raises(RuntimeError, match="Cannot add state"):
        manager.registerState(1, 1, "newState")


def test_shared_noise_connections_remain_fixed_after_finalization():
    """Reordered shared endpoints reuse the original independent process."""
    manager, alpha, beta = finalized_manager()
    manager.registerSharedNoiseSource([(beta, 0), (alpha, 1)])
    manager.finalizeStates()
    with pytest.raises(RuntimeError, match="cannot change after finalization"):
        manager.registerSharedNoiseSource([(alpha, 0)])
    assert_committed_values(alpha, beta)


def test_empty_finalization_fixes_an_empty_layout():
    """A zero-state object can finalize repeatedly without allocating states."""
    manager = stateArchitecture.DynParamManager()
    manager.finalizeStates()
    manager.finalizeStates()
    assert manager.statesAreFinalized()
    with pytest.raises(RuntimeError, match="Cannot add state"):
        manager.registerState(1, 1, "lateState")
