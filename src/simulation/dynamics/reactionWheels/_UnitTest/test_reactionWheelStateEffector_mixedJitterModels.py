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

"""Unit tests for mixed reaction wheel jitter model output indexing."""

import numpy as np

from Basilisk.architecture import messaging
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW


def test_mixed_rw_models_theta_output_indexing():
    r"""Verify theta outputs map only to jitter-model wheels in mixed wheel sets.

    The test configures one balanced wheel and one simple-jitter wheel. It then
    verifies that the balanced wheel's output theta stays at zero while the jitter
    wheel theta advances.
    """
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("proc")
    task_period_ns = macros.sec2nano(0.1)  # [ns]
    process.addTask(sim.CreateNewTask("task", task_period_ns))

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "scObject"
    sc_object.hub.mHub = 10.0  # [kg]
    sc_object.hub.IHubPntBc_B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]  # [kg*m^2]
    sc_object.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]  # [m]
    sc_object.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]  # [m/s]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # [-]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # [rad/s]

    rw_factory = simIncludeRW.rwFactory()
    rw_factory.create(
        "Honeywell_HR16",
        [1.0, 0.0, 0.0],
        maxMomentum=50.0,  # [N*m*s]
        Omega=100.0,  # [RPM]
        rWB_B=[0.1, 0.0, 0.0],  # [m]
        RWModel=reactionWheelStateEffector.BalancedWheels,
    )
    rw_factory.create(
        "Honeywell_HR16",
        [0.0, 1.0, 0.0],
        maxMomentum=50.0,  # [N*m*s]
        Omega=50.0,  # [RPM]
        rWB_B=[0.0, 0.1, 0.0],  # [m]
        RWModel=reactionWheelStateEffector.JitterSimple,
    )

    rw_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_effector.ModelTag = "rwEffector"
    rw_factory.addToSpacecraft("RWs", rw_effector, sc_object)

    cmd_payload = messaging.ArrayMotorTorqueMsgPayload()
    cmd_payload.motorTorque = [0.0, 0.0]  # [N*m]
    cmd_msg = messaging.ArrayMotorTorqueMsg().write(cmd_payload)
    rw_effector.rwMotorCmdInMsg.subscribeTo(cmd_msg)

    rw0_rec = rw_effector.rwOutMsgs[0].recorder()
    rw1_rec = rw_effector.rwOutMsgs[1].recorder()

    sim.AddModelToTask("task", rw_effector)
    sim.AddModelToTask("task", sc_object)
    sim.AddModelToTask("task", rw0_rec)
    sim.AddModelToTask("task", rw1_rec)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.3))  # [ns]
    sim.ExecuteSimulation()

    assert np.allclose(rw0_rec.theta, np.zeros_like(rw0_rec.theta))
    assert np.isclose(rw1_rec.theta[2], np.pi / 6.0)
    assert np.isclose(rw1_rec.theta[3], np.pi / 3.0)
