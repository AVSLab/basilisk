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

import numpy as np
import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import constraintDynamicEffector, spacecraft, spinningBodyOneDOFStateEffector
from Basilisk.utilities import SimulationBaseClass, macros


def _simulationWithConstraint(attachToBranch):
    """Create a spacecraft-only simulation with a constraint attached to a hub or branch."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    timeStep = macros.sec2nano(0.1)  # [ns]
    process.addTask(simulation.CreateNewTask("task", timeStep))

    spacecraftObject = spacecraft.Spacecraft()
    spacecraftObject.hub.mHub = 100.0  # [kg]
    spacecraftObject.hub.IHubPntBc_B = [[10.0, 0.0, 0.0],
                                        [0.0, 20.0, 0.0],
                                        [0.0, 0.0, 30.0]]  # [kg m^2]
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    spinningBody = None
    if attachToBranch:
        spinningBody = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
        spinningBody.mass = 1.0  # [kg]
        spinningBody.sHat_S = [[1.0], [0.0], [0.0]]  # [-]
        spinningBody.dcm_S0B = [[1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0]]  # [-]
        spinningBody.IPntSc_S = [[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]]  # [kg m^2]
        spinningBody.addDynamicEffector(constraintEffector)
        spacecraftObject.addStateEffector(spinningBody)
    else:
        spacecraftObject.addDynamicEffector(constraintEffector)
    simulation.AddModelToTask("task", spacecraftObject)
    return simulation, spacecraftObject, constraintEffector, spinningBody


@pytest.mark.parametrize("attachToBranch", [False, True], ids=["hub", "branch"])
@pytest.mark.parametrize("validationPath", ["attachment", "reset"])
@pytest.mark.parametrize("missingParameter", ["Alpha", "Beta"])
def test_constraintEffector_validation(attachToBranch, validationPath, missingParameter):
    """Hub, branch, and direct Reset paths must reject missing tuning parameters."""
    simulation, spacecraftObject, constraintEffector, spinningBody = _simulationWithConstraint(
        attachToBranch)
    if missingParameter == "Beta":
        constraintEffector.setAlpha(1.0)  # [-]

    with pytest.raises(BasiliskError, match=missingParameter):
        if validationPath == "reset":
            constraintEffector.Reset(0)
        else:
            simulation.InitializeSimulation()


@pytest.mark.parametrize("attachToBranch", [False, True], ids=["hub", "branch"])
@pytest.mark.parametrize("initializationPath", ["attachment", "reset"])
def test_constraintEffector_gainInitialization(attachToBranch, initializationPath):
    """Hub, branch, and direct Reset paths must derive unspecified individual gains."""
    simulation, spacecraftObject, constraintEffector, spinningBody = _simulationWithConstraint(
        attachToBranch)
    constraintEffector.setAlpha(3.0)  # [-]
    constraintEffector.setBeta(2.0)  # [-]

    if initializationPath == "reset":
        constraintEffector.Reset(0)
    else:
        simulation.InitializeSimulation()

    assert constraintEffector.getK_d() == pytest.approx(9.0)  # [N/m]
    assert constraintEffector.getC_d() == pytest.approx(4.0)  # [N*s/m]
    assert constraintEffector.getK_a() == pytest.approx(9.0)  # [N*m]
    assert constraintEffector.getC_a() == pytest.approx(4.0)  # [N*m*s]


@pytest.mark.parametrize("path", ["reset", "hub", "branch"])
@pytest.mark.parametrize("explicit_mask", range(16))
@pytest.mark.parametrize("override_time", ["before", "after"])
def test_retuning_refreshes_only_derived_gains(path, explicit_mask, override_time):
    """Explicit gains remain fixed even when their values match the original derived gains."""
    sim, parent, effector, body = _simulationWithConstraint(path == "branch")
    effector.setAlpha(3.0)  # [-]
    effector.setBeta(2.0)  # [-]
    fields = ["K_d", "C_d", "K_a", "C_a"]
    initial = [9.0, 4.0, 9.0, 4.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    retuned = [25.0, 14.0, 25.0, 14.0]  # [N/m], [N*s/m], [N*m], [N*m*s]

    def set_overrides():
        """Set each selected gain to the value it would otherwise derive."""
        for index, field in enumerate(fields):
            if explicit_mask & (1 << index):
                getattr(effector, "set" + field)(initial[index])

    if override_time == "before":
        set_overrides()
    if path == "reset":
        effector.Reset(0)
    else:
        sim.InitializeSimulation()
    if override_time == "after":
        set_overrides()
    effector.setAlpha(5.0)  # [-]
    effector.setBeta(7.0)  # [-]
    for _ in range(3):
        effector.Reset(macros.sec2nano(1.0))  # [s]
        for index, field in enumerate(fields):
            expected = initial[index] if explicit_mask & (1 << index) else retuned[index]
            assert getattr(effector, "get" + field)() == expected


@pytest.mark.parametrize("field", ["Alpha", "Beta", "K_d", "C_d", "K_a", "C_a"])
@pytest.mark.parametrize("bad", [
    np.nan, np.inf, -np.inf, 0.0, -1.0,  # K_d: [N/m], C_d: [N*s/m], K_a: [N*m], C_a: [N*m*s], Alpha/Beta: [-]
])
def test_invalid_gain_setters_preserve_derived_status(field, bad):
    """Rejected setters must not change a value or turn a derived gain into an explicit one."""
    effector = constraintDynamicEffector.ConstraintDynamicEffector()
    effector.setAlpha(3.0)  # [-]
    effector.setBeta(2.0)  # [-]
    effector.Reset(0)
    prior = getattr(effector, "get" + field)()
    with pytest.raises(BasiliskError):
        getattr(effector, "set" + field)(bad)
    assert getattr(effector, "get" + field)() == prior
    effector.setAlpha(5.0)  # [-]
    effector.setBeta(7.0)  # [-]
    effector.Reset(0)
    expected = [25.0, 14.0, 25.0, 14.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    assert [effector.getK_d(), effector.getC_d(), effector.getK_a(), effector.getC_a()] == expected


@pytest.mark.parametrize("path", ["reset", "hub", "branch"])
@pytest.mark.parametrize("field", ["Alpha", "Beta"])
def test_derived_gain_overflow_preserves_prior_gains(path, field):
    """Overflow while retuning must fail before any active gain is replaced."""
    sim, parent, effector, body = _simulationWithConstraint(path == "branch")
    effector.setAlpha(3.0)  # [-]
    effector.setBeta(2.0)  # [-]
    effector.Reset(0)
    getattr(effector, "set" + field)(np.finfo(float).max)  # [-] Numerical alpha/beta tuning parameter
    with pytest.raises(BasiliskError, match="finite derived gains"):
        if path == "reset":
            effector.Reset(0)
        else:
            sim.InitializeSimulation()
    expected = [9.0, 4.0, 9.0, 4.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    assert [effector.getK_d(), effector.getC_d(), effector.getK_a(), effector.getC_a()] == expected


def test_partial_explicit_gains_preserve_zero_unspecified_gains():
    """An existing partial explicit setup may intentionally leave other gains at zero."""
    effector = constraintDynamicEffector.ConstraintDynamicEffector()
    effector.setK_d(3.0)  # [N/m]
    effector.Reset(0)
    expected = [3.0, 0.0, 0.0, 0.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    assert [effector.getK_d(), effector.getC_d(), effector.getK_a(), effector.getC_a()] == expected
    effector.setAlpha(5.0)  # [-]
    effector.setBeta(7.0)  # [-]
    effector.Reset(0)
    expected = [3.0, 14.0, 25.0, 14.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    assert [effector.getK_d(), effector.getC_d(), effector.getK_a(), effector.getC_a()] == expected


def test_fully_explicit_gains_do_not_derive_unused_tuning_parameters():
    """Unused tuning parameters need not be squared or doubled when every gain is explicit."""
    effector = constraintDynamicEffector.ConstraintDynamicEffector()
    effector.setAlpha(np.finfo(float).max)  # [-]
    effector.setBeta(np.finfo(float).max)  # [-]
    effector.setK_d(3.0)  # [N/m]
    effector.setC_d(4.0)  # [N*s/m]
    effector.setK_a(5.0)  # [N*m]
    effector.setC_a(6.0)  # [N*m*s]
    effector.Reset(0)
    expected = [3.0, 4.0, 5.0, 6.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    assert [effector.getK_d(), effector.getC_d(), effector.getK_a(), effector.getC_a()] == expected
