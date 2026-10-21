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

"""Configuration and reset regressions for the general single body effector."""

import numpy as np
import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.architecture import messaging
from Basilisk.simulation import generalSingleBodyStateEffector as general, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros


def make_effector(rotational=False):
    """Create a valid effector with one independent degree of freedom."""
    effector = general.GeneralSingleBodyStateEffector()
    effector.setMass(20.0)  # [kg]
    effector.setIPntGc_G(np.eye(3))  # [kg*m^2]
    dof = general.DOF()
    dof.setDOFAxis([1.0, 0.0, 0.0])  # [-]
    dof.setBetaInit(0.1)  # [rad] or [m]
    dof.setBetaDotInit(0.2)  # [rad/s] or [m/s]
    (effector.addRotDOF if rotational else effector.addTransDOF)(dof)
    return effector, dof


def make_simulation(effector, priority=None, time_step=0.01):  # [s]
    """Attach the effector, optionally scheduling it before or after its parent.

    :param effector: Effector to attach.
    :param priority: Optional task priority; the spacecraft priority is zero.
    :param time_step: Task period in seconds.
    :return: Simulation and spacecraft, whose Python owners must remain alive.
    """
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    sim.CreateNewProcess("process").addTask(sim.CreateNewTask("task", macros.sec2nano(time_step)))
    parent = spacecraft.Spacecraft()
    parent.hub.mHub = 100.0  # [kg]
    parent.hub.IHubPntBc_B = np.eye(3) * 10.0  # [kg*m^2]
    parent.addStateEffector(effector)
    sim.AddModelToTask("task", parent)
    if priority is not None:
        sim.AddModelToTask("task", effector, priority)
    return sim, parent


def validate(effector, path):
    """Exercise direct reset or spacecraft initialization without effector scheduling."""
    if path == "reset":
        effector.Reset(0)
    else:
        sim, parent = make_simulation(effector)
        sim.InitializeSimulation()


FIELDS = [
    ("mass", False, "setMass", "getMass"),
    ("IPntGc_G", False, "setIPntGc_G", "getIPntGc_G"),
    ("r_GcG_G", False, "setR_GcG_G", "getR_GcG_G"),
    ("r_G0B_B", False, "setR_G0B_B", "getR_G0B_B"),
    ("dcm_G0B", False, "setDCM_G0B", "getDCM_G0B"),
    ("axis_G", True, "setDOFAxis", "getDOFAxis"),
    ("betaInit", True, "setBetaInit", "getBetaInit"),
    ("betaDotInit", True, "setBetaDotInit", "getBetaDotInit"),
    ("k", True, "setSpringConstantK", "getSpringConstantK"),
    ("c", True, "setDampingConstantC", "getDampingConstantC"),
]


@pytest.mark.parametrize("path", ["reset", "attached"])
@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
@pytest.mark.parametrize("field,on_dof,setter,getter", FIELDS, ids=[entry[0] for entry in FIELDS])
def test_rejects_nonfinite_configuration(path, bad, field, on_dof, setter, getter):
    """Reject non-finite scalars and vector/matrix entries through both initialization paths."""
    effector, dof = make_effector()
    target = dof if on_dof else effector
    value = np.asarray(getattr(target, getter)()).copy()
    value.flat[0] = bad
    getattr(target, setter)(value.tolist() if value.ndim else value.item())
    with pytest.raises(BasiliskError, match=field):
        validate(effector, path)


@pytest.mark.parametrize("path", ["reset", "attached"])
@pytest.mark.parametrize("case,expected", [
    ("zero_mass", "mass"), ("negative_mass", "mass"),
    ("negative_spring", "k"), ("negative_damping", "c"),
    ("zero_axis", "axis_G"), ("nonunit_axis", "axis_G"),
    ("invalid_dcm", "dcm_G0B"), ("invalid_inertia", "IPntGc_G"),
    ("empty", "at least one"),
])
def test_rejects_invalid_configuration(path, case, expected):
    """Reject invalid physical bounds, geometry, and empty configurations."""
    effector, dof = make_effector()
    mutations = {
        "zero_mass": lambda: effector.setMass(0.0),  # [kg]
        "negative_mass": lambda: effector.setMass(-1.0),  # [kg]
        "negative_spring": lambda: dof.setSpringConstantK(-1.0),  # [N/m]
        "negative_damping": lambda: dof.setDampingConstantC(-1.0),  # [N*s/m]
        "zero_axis": lambda: dof.setDOFAxis([0.0, 0.0, 0.0]),
        "nonunit_axis": lambda: dof.setDOFAxis([2.0, 0.0, 0.0]),  # [-]
        "invalid_dcm": lambda: effector.setDCM_G0B(np.diag([1.0, 1.0, -1.0])),  # [-]
        "invalid_inertia": lambda: effector.setIPntGc_G(np.diag([-1.0, 1.0, 1.0])),  # [kg*m^2]
        "empty": effector.clearDOFs,
    }
    mutations[case]()
    with pytest.raises(BasiliskError, match=expected):
        validate(effector, path)


@pytest.mark.parametrize("path", ["reset", "attached"])
@pytest.mark.parametrize("rotational", [False, True])
def test_rejects_redundant_coordinates(path, rotational):
    """Distinct DOF objects must not introduce a singular joint mass matrix."""
    effector, dof = make_effector(rotational)
    another = general.DOF()
    another.setDOFAxis(dof.getDOFAxis())
    (effector.addRotDOF if rotational else effector.addTransDOF)(another)
    with pytest.raises(BasiliskError, match="joint mass matrix.*singular"):
        validate(effector, path)


@pytest.mark.parametrize("path", ["reset", "register"])
@pytest.mark.parametrize("singular", [False, True])
def test_large_angle_initial_geometry(path, singular):
    """Validate the same rotation-vector geometry that the dynamics evaluates."""
    effector = general.GeneralSingleBodyStateEffector()
    effector.setMass(20.0)  # [kg]
    effector.setIPntGc_G(np.eye(3))  # [kg*m^2]
    axis_scale = 1.0 + 5e-10  # [-], within the accepted unit-vector tolerance
    angle = 2*np.pi*100000 + np.pi/2  # [rad]
    if singular:
        angle /= axis_scale
    for index, axis in enumerate(np.eye(3)):
        dof = general.DOF()
        dof.setDOFAxis(axis*axis_scale if index == 1 else axis)
        if index == 1:
            dof.setBetaInit(angle)
        effector.addRotDOF(dof)
    sim, parent = make_simulation(effector)
    initialize = (lambda: effector.Reset(0)) if path == "reset" else lambda: effector.registerStates(parent.dynManager)
    if singular:
        with pytest.raises(BasiliskError, match="joint mass matrix.*singular"):
            initialize()
    else:
        initialize()
        sim.InitializeSimulation()
        reader = effector.spinningBodyOutMsgs[1].addSubscriber()
        sim.ConfigureStopTime(macros.sec2nano(0.01))  # [s]
        sim.ExecuteSimulation()
        assert reader().theta == pytest.approx(angle)
        assert reader().thetaDot == pytest.approx(0.0)  # [rad/s]


@pytest.mark.parametrize("inertia", [1e-16, 1e-8, 1.0])  # [kg*m^2]
@pytest.mark.parametrize("rotation_first", [False, True])
def test_independent_coordinates_with_different_inertia_scales(inertia, rotation_first):
    """Mixed coordinate scales must preserve analytical force and torque responses."""
    effector = general.GeneralSingleBodyStateEffector()
    effector.setMass(1.0)  # [kg]
    effector.setIPntGc_G(inertia*np.eye(3))  # [kg*m^2]
    for rotational in (rotation_first, not rotation_first):
        dof = general.DOF()
        (effector.addRotDOF if rotational else effector.addTransDOF)(dof)
    force = 0.4  # [N]
    torque = 0.6*inertia  # [N*m]
    force_payload = messaging.ArrayMotorForceMsgPayload()
    force_payload.motorForce = [force] + [0.0]*(len(force_payload.motorForce) - 1)  # [N]
    force_message = messaging.ArrayMotorForceMsg().write(force_payload)
    effector.motorForceInMsg[0].subscribeTo(force_message)
    torque_payload = messaging.ArrayMotorTorqueMsgPayload()
    torque_payload.motorTorque = [torque] + [0.0]*(len(torque_payload.motorTorque) - 1)  # [N*m]
    torque_message = messaging.ArrayMotorTorqueMsg().write(torque_payload)
    effector.motorTorqueInMsg[0].subscribeTo(torque_message)
    translation = effector.translatingBodyOutMsgs[0].addSubscriber()
    rotation = effector.spinningBodyOutMsgs[0].addSubscriber()
    sim, parent = make_simulation(effector, priority=1)
    sim.InitializeSimulation()
    duration = 0.01  # [s]
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    acceleration = force*(1/effector.getMass() + 1/parent.hub.mHub)  # [m/s^2]
    angular_acceleration = torque*(1/inertia + 1/np.asarray(parent.hub.IHubPntBc_B)[0, 0])  # [rad/s^2]
    assert translation().rho == pytest.approx(acceleration*duration**2/2, rel=1e-12, abs=1e-14)
    assert translation().rhoDot == pytest.approx(acceleration*duration, rel=1e-12, abs=1e-14)
    assert rotation().theta == pytest.approx(angular_acceleration*duration**2/2, rel=1e-12, abs=1e-14)
    assert rotation().thetaDot == pytest.approx(angular_acceleration*duration, rel=1e-12, abs=1e-14)


@pytest.mark.parametrize("path", ["reset", "attached"])
def test_rejects_redundancy_with_different_inertia_scales(path):
    """Scaling must not hide redundant rotations in a mixed coordinate system."""
    effector = general.GeneralSingleBodyStateEffector()
    effector.setMass(1.0)  # [kg]
    effector.setIPntGc_G(1e-16*np.eye(3))  # [kg*m^2]
    effector.addTransDOF(general.DOF())
    effector.addRotDOF(general.DOF())
    effector.addRotDOF(general.DOF())
    with pytest.raises(BasiliskError, match="joint mass matrix.*singular"):
        validate(effector, path)


@pytest.mark.parametrize("priority", [None, 1, -1], ids=["attached", "effector_first", "parent_first"])
def test_repeated_reset_preserves_live_states(priority):
    """Effector reset preserves live states; spacecraft reset restores configured initial values."""
    effector, dof = make_effector()
    reader = effector.translatingBodyOutMsgs[0].addSubscriber()
    sim, parent = make_simulation(effector, priority)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.02))  # [ns]
    sim.ExecuteSimulation()
    expected = 0.104  # [m], initial displacement plus constant relative rate times elapsed time
    assert reader().rho == pytest.approx(expected)
    dof.setBetaInit(0.5)  # [m], configuration changes must not overwrite the live state during effector reset
    for _ in range(2):
        effector.Reset(0)
        effector.updateEffectorMassProps(0.0)  # [s], refresh from registered states
        effector.writeOutputStateMessages(0)
        assert reader().rho == pytest.approx(expected)
    parent.Reset(0)
    assert reader().rho == pytest.approx(0.5)  # [m]
    assert reader().rhoDot == pytest.approx(0.2)  # [m/s]


def test_rejects_coordinate_singularity_during_motion():
    """A valid XYZ rotational chain must stop when its middle angle reaches pi/2."""
    effector = general.GeneralSingleBodyStateEffector()
    effector.setMass(20.0)  # [kg]
    effector.setIPntGc_G(np.eye(3))  # [kg*m^2], spherical inertia gives constant rotation about Y
    for index, axis in enumerate(np.eye(3)):
        dof = general.DOF()
        dof.setDOFAxis(axis)
        if index == 1:
            dof.setBetaDotInit(np.pi / 2.0)  # [rad/s]
        effector.addRotDOF(dof)
    sim, parent = make_simulation(effector, time_step=1.0)  # [s]
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))  # [ns]
    with pytest.raises(BasiliskError, match="joint mass matrix.*singular"):
        sim.ExecuteSimulation()
