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

"""Tests for reaction-wheel initialization through spacecraft attachment."""

import pytest

from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW


def test_fully_coupled_wheel_initializes_without_task_scheduling():
    """Verify attachment initializes fully coupled wheel parameters without an effector ``Reset()`` call."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(0.1)))  # [ns]

    spacecraft_model = spacecraft.Spacecraft()
    spacecraft_model.hub.mHub = 10.0  # [kg]
    spacecraft_model.hub.IHubPntBc_B = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]  # [kg*m^2]

    wheel_factory = simIncludeRW.rwFactory()
    wheel = wheel_factory.create(
        "Honeywell_HR16",
        [1.0, 0.0, 0.0],
        maxMomentum=50.0,  # [N*m*s]
        RWModel=reactionWheelStateEffector.JitterFullyCoupled,
    )
    wheel.mass = 2.0  # [kg]
    wheel.U_s = 0.6  # [kg*m]
    wheel.U_d = 0.4  # [kg*m^2]

    wheel_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    wheel_factory.addToSpacecraft("reactionWheels", wheel_effector, spacecraft_model)

    # The effector is intentionally not added to the task. Its required dynamics initialization must occur when the
    # attached spacecraft registers the effector states.
    sim.AddModelToTask("task", spacecraft_model)
    sim.InitializeSimulation()

    assert wheel.d == pytest.approx(0.3)  # [m]
    assert wheel.J13 == pytest.approx(0.4)  # [kg*m^2]
