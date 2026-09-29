#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import gc
import inspect
import weakref
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from Basilisk import hasBuildFeature
from Basilisk.utilities import SimulationBaseClass, macros

mujocoEnabled = hasBuildFeature("mujoco")
pytestmark = pytest.mark.skipif(
    not mujocoEnabled,
    reason="Requires Basilisk built with --mujoco True",
)
if mujocoEnabled:
    from Basilisk.architecture.messaging import MJSceneStateMsgPayload
    from Basilisk.architecture.messaging import SCStatesMsgPayload
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import stateArchitecture
    from Basilisk.simulation import StatefulSysModel

def test_stateful():
    """Tests that ``StatefulSysModel`` works as expected.

    We use a simple ``StatefulSysModel`` with a single state. We check
    that said state is registered with the expected name and that its
    value evolves as we would expect.
    """

    # Declared inside, since StatefulSysModel may be undefined if not running with mujoco
    class ExponentialStateModel(StatefulSysModel.StatefulSysModel):
        """A simple model with one state, whose derivative is dx/dt = x*t."""

        def registerStates(self, registerer: StatefulSysModel.DynParamRegisterer):
            """Called once during InitializeSimulation"""
            self.xState = registerer.registerState(1, 1, "x")

        def UpdateState(self, CurrentSimNanos):
            """Called at every integrator step"""
            t = macros.NANO2SEC * CurrentSimNanos
            x = self.xState.getState()[0][0]
            self.xState.setDerivative( [[t*x]] )


    dt = 0.01 # s
    tf = 1 # s

    # Create sim, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene("<mujoco/>") # empty scene, no multi-body dynamics
    scSim.AddModelToTask("test", scene)

    expState = ExponentialStateModel()
    expState.ModelTag = "testModel"

    scene.AddModelToDynamicsTask(expState)

    # Run the sim
    scSim.InitializeSimulation()
    expState.xState.setState([[1]]) # initialize state to 1

    # Run for tf seconds
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    # Check that the state name has the model tag and ID prepended
    expected_name = f"{expState.ModelTag}_{expState.moduleID}_x"
    assert expState.xState.getName() == expected_name, f"{expState.xState.getName()} != {expected_name}"

    # The state follows dx/dt=x*t for x(0) = 1
    # So we expect x(tf=1) to be e^(tf^2/2)
    expected = np.exp( tf**2 / 2 )
    assert expState.xState.getState()[0][0] == pytest.approx(expected)


def test_register_state_preserves_legacy_keyword_arguments():

    class KeywordRegistrationModel(StatefulSysModel.StatefulSysModel):
        def registerStates(self, registerer):
            self.xState = registerer.registerState(
                nRow=1,
                nCol=1,
                stateName="x",
            )

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])

    scene = mujoco.MJScene("<mujoco/>")
    model = KeywordRegistrationModel()
    model.ModelTag = "keywordModel"
    scene.AddModelToDynamicsTask(model)
    scene.Reset(0)

    assert model.xState.getName().endswith("_x")
    method = StatefulSysModel.DynParamRegisterer.registerState
    assert method.__name__ == "registerState"
    assert method.__qualname__.endswith(
        "DynParamRegisterer.registerState"
    )
    assert inspect.signature(method)


def test_empty_model_tags_receive_unique_state_namespaces():
    class UntaggedModel(StatefulSysModel.StatefulSysModel):
        def registerStates(self, registerer):
            self.xState = registerer.registerState(1, 1, "x")

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])

    scene = mujoco.MJScene("<mujoco/>")
    first = UntaggedModel()
    second = UntaggedModel()
    assert first.ModelTag == second.ModelTag == ""

    scene.AddModelToDynamicsTask(first)
    scene.AddModelToDynamicsTask(second)
    scene.Reset(0)

    assert first.xState.getName() == f"model_{first.moduleID}_x"
    assert second.xState.getName() == f"model_{second.moduleID}_x"
    assert first.xState.getName() != second.xState.getName()


def test_initialize_dynamics_is_not_a_public_scene_lifecycle_entry():
    scene = mujoco.MJScene("<mujoco/>")
    assert not hasattr(scene, "initializeDynamics")


def test_self_init_rejects_task_mutation_before_iteration_changes():
    class MutatingSelfInitModel(StatefulSysModel.StatefulSysModel):
        def __init__(self):
            super().__init__()
            self.mutation_error = None

        def SelfInit(self):
            try:
                scene.AddFwdKinematicsToDynamicsTask(0)
            except RuntimeError as error:
                self.mutation_error = str(error)
                raise

        def registerStates(self, registerer):
            self.xState = registerer.registerState(1, 1, "x")

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])

    simulation = SimulationBaseClass.SimBaseClass()
    simulation.CreateNewProcess("process").addTask(
        simulation.CreateNewTask("task", 1)
    )
    scene = mujoco.MJScene("<mujoco/>")
    model = MutatingSelfInitModel()
    scene.AddModelToDynamicsTask(model)
    simulation.AddModelToTask("task", scene)

    with pytest.raises(RuntimeError, match="director method error"):
        simulation.InitializeSimulation()

    assert "cannot run from an MJScene reset or dynamics callback" in (
        model.mutation_error
    )


def test_register_state_spec_preserves_keywords_and_borrowed_access():
    """Register through the public specification API while the scene remains alive."""
    class SpecRegistrationModel(StatefulSysModel.StatefulSysModel):
        def registerStates(self, registerer):
            shape = stateArchitecture.MatrixShape()
            shape.rows = 1
            shape.cols = 1
            spec = stateArchitecture.StateSpec()
            spec.state = shape
            spec.derivative = shape
            spec.diffusionTangent = shape
            self.xState = registerer.registerStateSpec(
                stateName="x",
                spec=spec,
            )

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])

    scene = mujoco.MJScene("<mujoco/>")
    model = SpecRegistrationModel()
    model.ModelTag = "specModel"
    scene.AddModelToDynamicsTask(model)
    scene.Reset(0)

    assert model.xState.getName().endswith("_x")
    assert model.xState.thisown is False
    assert model.xState.getState() == [[0.0]]


def test_late_stateful_model_is_rejected_before_task_mutation():
    class LateStateModel(StatefulSysModel.StatefulSysModel):
        def registerStates(self, registerer):
            self.xState = registerer.registerState(1, 1, "x")

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])

    scene = mujoco.MJScene("<mujoco/>")
    scene.Reset(0)

    with pytest.raises(RuntimeError, match="finalized state topology"):
        scene.AddModelToDynamicsTask(LateStateModel())

    scene.Reset(0)


def test_python_reset_failure_is_translated_and_recoverable():
    class FailingResetModel(StatefulSysModel.StatefulSysModel):
        def __init__(self):
            super().__init__()
            self.fail = True

        def registerStates(self, registerer):
            self.xState = registerer.registerState(1, 1, "x")

        def Reset(self, CurrentSimNanos):
            if self.fail:
                raise RuntimeError("deliberate Python reset failure")

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])

    scene = mujoco.MJScene("<mujoco/>")
    model = FailingResetModel()
    scene.AddModelToDynamicsTask(model)

    with pytest.raises(RuntimeError):
        scene.Reset(0)

    model.fail = False
    scene.Reset(0)
    assert scene.dynManager.statesAreFinalized()




def test_model_shared_by_both_dynamics_tasks_is_reset_once():
    class SharedModel(StatefulSysModel.StatefulSysModel):
        def __init__(self):
            super().__init__()
            self.reset_count = 0

        def registerStates(self, registerer):
            self.xState = registerer.registerState(1, 1, "x")

        def Reset(self, CurrentSimNanos):
            self.reset_count += 1

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])

    scene = mujoco.MJScene("<mujoco/>")
    model = SharedModel()
    scene.AddModelToDynamicsTask(model)
    scene.AddModelToDiffusionDynamicsTask(model)

    scene.Reset(0)

    assert model.reset_count == 1


def test_reset_callback_cannot_invalidate_compiled_model_pointers():
    class MutatingResetModel(StatefulSysModel.StatefulSysModel):
        def __init__(self):
            super().__init__()
            self.mutate = True
            self.mutation_error = None

        def registerStates(self, registerer):
            self.xState = registerer.registerState(1, 1, "x")

        def Reset(self, CurrentSimNanos):
            if self.mutate:
                try:
                    scene.addSingleActuator(
                        "lateActuator",
                        "missingSite",
                        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    )
                except RuntimeError as error:
                    self.mutation_error = str(error)
                    raise

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])

    scene = mujoco.MJScene("<mujoco/>")
    model = MutatingResetModel()
    scene.AddModelToDynamicsTask(model)

    with pytest.raises(RuntimeError, match="director method error"):
        scene.Reset(0)
    assert "cannot run from an MJScene reset or dynamics callback" in (
        model.mutation_error
    )

    model.mutate = False
    scene.Reset(0)


def test_dynamics_callback_cannot_invalidate_compiled_model_pointers():
    class MutatingDynamicsModel(StatefulSysModel.StatefulSysModel):
        def __init__(self):
            super().__init__()
            self.mutate = True
            self.mutation_error = None

        def registerStates(self, registerer):
            self.xState = registerer.registerState(1, 1, "x")

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])
            if self.mutate:
                try:
                    scene.addSingleActuator(
                        "lateActuator",
                        "missingSite",
                        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    )
                except RuntimeError as error:
                    self.mutation_error = str(error)
                    raise

    scene = mujoco.MJScene("<mujoco/>")
    model = MutatingDynamicsModel()
    scene.AddModelToDynamicsTask(model)
    scene.Reset(0)

    with pytest.raises(RuntimeError, match="director method error"):
        scene.integrateState(1)
    assert "cannot run from an MJScene reset or dynamics callback" in (
        model.mutation_error
    )

    model.mutate = False
    scene.Reset(0)
    scene.integrateState(1)


def test_diffusion_callback_cannot_recompile_or_mutate_scene():
    from Basilisk.simulation import svIntegrators

    shape = stateArchitecture.MatrixShape()
    shape.rows = 1
    shape.cols = 1
    state_spec = stateArchitecture.StateSpec()
    state_spec.state = shape
    state_spec.derivative = shape
    state_spec.diffusionTangent = shape
    state_spec.noiseCount = 1

    class MutatingDiffusionModel(StatefulSysModel.StatefulSysModel):
        def __init__(self):
            super().__init__()
            self.mutation_error = None

        def registerStates(self, registerer):
            self.xState = registerer.registerStateSpec("x", state_spec)

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDiffusion([[1.0]], index=0)
            try:
                scene.addSingleActuator(
                    "lateDiffusionActuator",
                    "missingSite",
                    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                )
            except RuntimeError as error:
                self.mutation_error = str(error)
                raise

    scene = mujoco.MJScene("<mujoco/>")
    integrator = svIntegrators.svStochasticIntegratorMayurama(scene)
    scene.setIntegrator(integrator)
    model = MutatingDiffusionModel()
    scene.AddModelToDiffusionDynamicsTask(model)
    scene.Reset(0)

    with pytest.raises(RuntimeError, match="director method error"):
        scene.integrateState(1)

    assert "cannot run from an MJScene reset or dynamics callback" in (
        model.mutation_error
    )


def test_diffusion_synchronizes_candidate_state_and_mass_before_callbacks():
    xml = """
    <mujoco>
      <worldbody>
        <body name="slider">
          <joint name="slide" type="slide" axis="1 0 0"/>
          <geom type="sphere" size="1" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """

    scene = mujoco.MJScene(xml)
    scene.AddFwdKinematicsToDiffusionDynamicsTask(0)
    scene.Reset(0)

    body = scene.getBody("slider")
    body.getScalarJoint("slide").setPosition(2.0)
    mass_state = scene.getMassState()
    mass_state.setState([[0.0], [2.0]])

    origin_reader = body.getOrigin().stateOutMsg.addSubscriber()
    evaluation_time = 0.25
    scene.equationsOfMotionDiffusion(evaluation_time, 0.0)

    assert origin_reader.isWritten()
    assert origin_reader.timeWritten() == macros.sec2nano(evaluation_time)
    np.testing.assert_allclose(
        body.getOrigin().stateOutMsg.read().r_BN_N,
        [2.0, 0.0, 0.0],
        atol=1e-14,
    )

    mass_state.setState([[0.0], [-1.0]])
    with pytest.raises(RuntimeError, match="finite and nonnegative"):
        scene.equationsOfMotionDiffusion(evaluation_time, 0.0)


def test_reset_from_dynamics_callback_is_rejected_without_invalidating_scene():
    class NestedResetModel(StatefulSysModel.StatefulSysModel):
        def __init__(self):
            super().__init__()
            self.attempted_reset = False
            self.reset_error = None

        def registerStates(self, registerer):
            self.xState = registerer.registerState(1, 1, "x")

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])
            if self.attempted_reset:
                return
            self.attempted_reset = True
            try:
                scene.Reset(CurrentSimNanos)
            except RuntimeError as error:
                self.reset_error = str(error)

    scene = mujoco.MJScene("<mujoco/>")
    model = NestedResetModel()
    scene.AddModelToDynamicsTask(model)
    scene.Reset(0)

    scene.integrateState(1)

    assert "cannot run from an MJScene reset or dynamics callback" in (
        model.reset_error
    )
    scene.integrateState(2)


def test_failed_initial_reset_does_not_size_or_publish_state_output():
    xml = """
    <mujoco>
      <worldbody>
        <body name="slider">
          <joint name="slide" type="slide"/>
          <geom type="sphere" size="1" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """

    class FailingInitialResetModel(StatefulSysModel.StatefulSysModel):
        def __init__(self):
            super().__init__()
            self.fail = True

        def registerStates(self, registerer):
            self.xState = registerer.registerState(1, 1, "x")

        def Reset(self, CurrentSimNanos):
            if self.fail:
                raise RuntimeError("deliberate initial reset failure")

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])

    scene = mujoco.MJScene(xml)
    model = FailingInitialResetModel()
    scene.AddModelToDynamicsTask(model)
    initial_output = scene.stateOutMsg.read()
    initial_qpos = np.asarray(initial_output.qpos).copy()
    initial_qvel = np.asarray(initial_output.qvel).copy()
    initial_act = np.asarray(initial_output.act).copy()

    with pytest.raises(RuntimeError, match="director method error"):
        scene.Reset(0)

    failed_output = scene.stateOutMsg.read()
    np.testing.assert_array_equal(failed_output.qpos, initial_qpos)
    np.testing.assert_array_equal(failed_output.qvel, initial_qvel)
    np.testing.assert_array_equal(failed_output.act, initial_act)

    model.fail = False
    scene.Reset(0)
    assert len(scene.stateOutMsg.read().qpos) == 1






def test_callback_state_borrows_scene_without_retaining_registration_helper():
    """The helper is temporary; the state remains usable for the scene lifetime."""
    class OwnershipModel(StatefulSysModel.StatefulSysModel):
        def registerStates(self, registerer):
            self.registerer_ref = weakref.ref(registerer)
            self.xState = registerer.registerState(1, 1, "x")

        def UpdateState(self, CurrentSimNanos):
            self.xState.setDerivative([[0.0]])

    scene = mujoco.MJScene("<mujoco/>")
    model = OwnershipModel()
    scene.AddModelToDynamicsTask(model)
    scene.Reset(0)
    gc.collect()
    assert model.registerer_ref() is None
    assert not model.xState.thisown
    model.xState.setState([[3.0]])
    scene.integrateState(0)
    assert model.xState.getState() == [[3.0]]

    # Releasing an expired borrowed proxy is safe; using it after the scene is not.
    state = model.xState
    scene_ref = weakref.ref(scene)
    del model, scene
    gc.collect()
    assert scene_ref() is None
    del state


def test_is_dynamics_synced_is_read_only_compatibility_property():
    primary = mujoco.MJScene("<mujoco/>")
    secondary = mujoco.MJScene("<mujoco/>")

    assert primary.isDynamicsSynced is False
    assert secondary.isDynamicsSynced is False
    primary.syncDynamicsIntegration(secondary)
    assert primary.isDynamicsSynced is False
    assert secondary.isDynamicsSynced is True

    with pytest.raises(AttributeError):
        secondary.isDynamicsSynced = False


def test_synchronized_scene_survives_reference_drop_during_callback():
    holder = {}
    references = {}

    class ReferenceDroppingModel(StatefulSysModel.StatefulSysModel):
        def __init__(self):
            super().__init__()
            self.dropped_reference = False

        def registerStates(self, registerer):
            pass

        def UpdateState(self, CurrentSimNanos):
            if self.dropped_reference:
                return
            self.dropped_reference = True
            holder["secondary"] = None
            gc.collect()
            assert references["secondary"]() is not None

    primary = mujoco.MJScene("<mujoco/>")
    secondary = mujoco.MJScene("<mujoco/>")
    model = ReferenceDroppingModel()
    primary.AddModelToDynamicsTask(model)
    primary.Reset(0)
    secondary.Reset(0)
    primary.syncDynamicsIntegration(secondary)

    primary_ref = weakref.ref(primary)
    references["secondary"] = weakref.ref(secondary)
    holder["secondary"] = secondary
    secondary = None

    primary.integrateState(macros.sec2nano(0.01))

    assert model.dropped_reference
    assert references["secondary"]() is not None
    assert primary.integrator.getDynamicsCount() == 2

    primary = None
    gc.collect()
    assert primary_ref() is None
    assert references["secondary"]() is None


if __name__ == "__main__":
    if True:
        test_stateful()
    else:
        pytest.main([__file__])
