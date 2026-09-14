# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Exercise naming for reaction-wheel and VSCMG arrays through the public flag."""

import numpy as np
import pytest

from Basilisk.architecture import bskLogging, messaging
from Basilisk.simulation import reactionWheelStateEffector, spacecraft, vscmgStateEffector
from Basilisk.utilities import SimulationBaseClass, macros


FIELDS = {
    "rw": ("nameOfReactionWheelOmegasState", "nameOfReactionWheelThetasState"),
    "vscmg": ("nameOfVSCMGOmegasState", "nameOfVSCMGThetasState",
              "nameOfVSCMGGammasState", "nameOfVSCMGGammaDotsState"),
}
PREFIXES = {
    "rw": ("reactionWheelOmegas", "reactionWheelThetas"),
    "vscmg": ("VSCMGOmegas", "VSCMGThetas", "VSCMGGammas", "VSCMGGammaDots"),
}


def make_effector(kind, model):
    """Create one balanced, simple-jitter, or fully coupled device with finite geometry."""
    if kind == "rw":
        effector = reactionWheelStateEffector.ReactionWheelStateEffector()
        config = reactionWheelStateEffector.RWConfigPayload()
        config.RWModel = (reactionWheelStateEffector.BalancedWheels,
                          reactionWheelStateEffector.JitterSimple,
                          reactionWheelStateEffector.JitterFullyCoupled)[model]
        config.gsHat_B = [1.0, 0.0, 0.0]
        config.w2Hat0_B = [0.0, 1.0, 0.0]
        config.w3Hat0_B = [0.0, 0.0, 1.0]
        config.Js = 0.2  # [kg m^2]
        config.Jt = config.Jg = 0.15  # [kg m^2]
        config.mass = 2.0  # [kg]
        config.U_s = 0.02  # [kg m]
        config.U_d = 0.001  # [kg m^2]
        config.Omega = 1.0  # [rad/s]
        config.betaStatic = -1.0  # [-]
        effector.addReactionWheel(config)
    else:
        effector = vscmgStateEffector.VSCMGStateEffector()
        config = messaging.VSCMGConfigMsgPayload()
        config.VSCMGModel = (vscmgStateEffector.vscmgBalancedWheels,
                             vscmgStateEffector.vscmgJitterSimple,
                             vscmgStateEffector.vscmgJitterFullyCoupled)[model]
        config.gsHat0_B = [1.0, 0.0, 0.0]
        config.gtHat0_B = [0.0, 1.0, 0.0]
        config.ggHat_B = [0.0, 0.0, 1.0]
        config.massW = 2.0  # [kg]
        config.massG = 1.0  # [kg]
        config.IW1 = 0.2  # [kg m^2]
        config.IW2 = config.IW3 = 0.15  # [kg m^2]
        config.IG1 = 0.1  # [kg m^2]
        config.IG2 = 0.15  # [kg m^2]
        config.IG3 = 0.2  # [kg m^2]
        config.U_s = 0.02  # [kg m]
        config.U_d = 0.001  # [kg m^2]
        config.Omega = 1.0  # [rad/s]
        config.gamma = 0.3  # [rad]
        config.gammaDot = 0.02  # [rad/s]
        effector.AddVSCMG(config)
    return effector


def make_vehicle(local):
    """Configure a stationary hub with a selected naming policy."""
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.hub.IHubPntBc_B = 100.0 * np.eye(3)  # [kg m^2]
    vehicle.dynManager.useManagerLocalEffectorNames = local
    return vehicle


@pytest.mark.parametrize("kind", FIELDS)
@pytest.mark.parametrize("model", range(3))
def test_actuator_array_names_and_dynamics(kind, model):
    """Two arrays have independent names and equivalent trajectories under both policies."""
    histories = []
    for local in (False, True):
        sim = SimulationBaseClass.SimBaseClass()
        process = sim.CreateNewProcess("process")
        step = 0.01  # [s]
        process.addTask(sim.CreateNewTask("task", macros.sec2nano(step)))
        vehicle = make_vehicle(local)
        effectors = [make_effector(kind, model) for _ in range(2)][::-1]
        for index, effector in enumerate(effectors):
            # Legacy array defaults are fixed strings, so two arrays require explicit names.
            if not local:
                for field, prefix in zip(FIELDS[kind], PREFIXES[kind]):
                    setattr(effector, field, prefix + str(index + 1))
            vehicle.addStateEffector(effector)
        sim.AddModelToTask("task", vehicle)
        recorder = vehicle.scStateOutMsg.recorder()
        sim.AddModelToTask("task", recorder)
        sim.InitializeSimulation()
        active_fields = [field for field in FIELDS[kind] if model or "Thetas" not in field]
        names = [getattr(effector, field) for effector in effectors for field in active_fields]
        states = [vehicle.dynManager.getStateObject(name) for name in names]
        initial = [np.asarray(state.getState()).copy() for state in states]
        sim.InitializeSimulation()
        assert names == [getattr(effector, field) for effector in effectors for field in active_fields]
        for state, original in zip(states, initial):
            np.testing.assert_array_equal(state.getState(), original)
        assert len(set(names)) == len(names)
        # A marker proves the second array does not alias the first array's speed state.
        states[0].setState([[2.0]])  # [rad/s]
        np.testing.assert_array_equal(states[len(active_fields)].getState(), [[1.0]])  # [rad/s]
        states[0].setState(initial[0])
        duration = 0.1  # [s]
        sim.ConfigureStopTime(macros.sec2nano(duration))
        sim.ExecuteSimulation()
        histories.append(np.column_stack((recorder.sigma_BN, recorder.omega_BN_B)))
        assert np.all(np.isfinite(histories[-1]))
    np.testing.assert_allclose(histories[0], histories[1], rtol=0.0, atol=1e-13)


@pytest.mark.parametrize("kind,field,prefix", [
    (kind, field, prefix) for kind in FIELDS for field, prefix in zip(FIELDS[kind], PREFIXES[kind])
])
def test_custom_actuator_names_reserve_candidates_and_freeze(kind, field, prefix):
    """Every public name tracks assignments, reserves automatic candidates, and freezes."""
    for custom in (prefix, prefix + "1"):
        vehicle = make_vehicle(True)
        first = make_effector(kind, 1)
        last = make_effector(kind, 1)
        setattr(last, field, custom)
        vehicle.addStateEffector(first)
        vehicle.addStateEffector(last)
        vehicle.initializeDynamics()
        assert getattr(last, field) == custom
        assert getattr(first, field) == prefix + ("2" if custom.endswith("1") else "1")
        setattr(last, field, custom)
        with pytest.raises(bskLogging.BasiliskError, match="resolved names cannot be changed"):
            setattr(last, field, "changed")


def test_actuator_custom_name_collision_across_types():
    """Reaction-wheel and VSCMG declarations share the manager's state namespace."""
    vehicle = make_vehicle(True)
    wheel = make_effector("rw", 0)
    gimbal = make_effector("vscmg", 0)
    wheel.nameOfReactionWheelOmegasState = "sharedSpeed"
    gimbal.nameOfVSCMGOmegasState = "sharedSpeed"
    vehicle.addStateEffector(wheel)
    vehicle.addStateEffector(gimbal)
    with pytest.raises(bskLogging.BasiliskError, match="sharedSpeed.*already in use"):
        vehicle.initializeDynamics()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
