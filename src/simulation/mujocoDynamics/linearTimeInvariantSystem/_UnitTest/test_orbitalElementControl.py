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

import pytest
import numpy as np

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import meanOEFeedback
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJLinearTimeInvariantSystem
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("integral", (True, False))
def test_orbitalElementControl(integral: bool):
    """Checks that the MJLinearTimeInvariantSystem.OrbitalElementControl behaves
    the same way as meanOEFeedback with J2 = 0.

    Both the proportional and integral gains are tested.
    """
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess(unitProcessName)
    dt = macros.sec2nano(0.1)
    proc.addTask(sim.CreateNewTask(unitTaskName, dt))

    # MuJoCo scene just to host OrbitalElementControl
    scene = mujoco.MJScene("<mujoco/>")
    sim.AddModelToTask(unitTaskName, scene)

    # Gravitational parameter and J2
    mu = orbitalMotion.MU_EARTH * 1e9       # [m^3/s^2]
    req = orbitalMotion.REQ_EARTH * 1e3     # [m]
    J2_small = 1e-12                        # effectively zero J2 for "truth"

    # Chief and deputy classic elements (same as meanOEFeedback test)
    oe_c = orbitalMotion.ClassicElements()
    oe_c.a = 20000e3
    oe_c.e = 0.1
    oe_c.i = 0.2
    oe_c.Omega = 0.3
    oe_c.omega = 0.4
    oe_c.f = 0.5
    r_c, v_c = orbitalMotion.elem2rv(mu, oe_c)

    oe_d = orbitalMotion.ClassicElements()
    oe_d.a = (1 + 0.0006) * 7000e3
    oe_d.e = 0.2 + 0.0005
    oe_d.i = 0.0 + 0.0004
    oe_d.Omega = 0.0 + 0.0003
    oe_d.omega = 0.0 + 0.0002
    oe_d.f = 0.0001
    r_d, v_d = orbitalMotion.elem2rv(mu, oe_d)

    # meanOEFeedback module (to generate truth values)
    mean_module = meanOEFeedback.meanOEFeedback()
    mean_module.ModelTag = "meanOEFeedback_orbitElemControlTruth"
    mean_module.targetDiffOeMean = [0.0] * 6
    mean_module.mu = mu
    mean_module.req = req
    mean_module.J2 = J2_small
    mean_module.K = [
        1e7, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1e7, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e7, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e7, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e7, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1e7
    ]

    sim.AddModelToTask(unitTaskName, mean_module)

    # Chief NavTrans message
    chiefNavPayload = messaging.NavTransMsgPayload()
    chiefNavPayload.timeTag = 0
    chiefNavPayload.r_BN_N = r_c
    chiefNavPayload.v_BN_N = v_c
    chiefNavPayload.vehAccumDV = [0.0, 0.0, 0.0]
    chiefNavMsg = messaging.NavTransMsg().write(chiefNavPayload)

    # Deputy NavTrans message
    deputyNavPayload = messaging.NavTransMsgPayload()
    deputyNavPayload.timeTag = 0
    deputyNavPayload.r_BN_N = r_d
    deputyNavPayload.v_BN_N = v_d
    deputyNavPayload.vehAccumDV = [0.0, 0.0, 0.0]
    deputyNavMsg = messaging.NavTransMsg().write(deputyNavPayload)

    mean_module.chiefTransInMsg.subscribeTo(chiefNavMsg)
    mean_module.deputyTransInMsg.subscribeTo(deputyNavMsg)

    # Truth recorder
    meanLog = mean_module.forceOutMsg.recorder()
    sim.AddModelToTask(unitTaskName, meanLog)

    # OrbitalElementControl module under test
    ctrl = MJLinearTimeInvariantSystem.OrbitalElementControl()
    ctrl.ModelTag = "OrbitalElementControl"
    ctrl.mu = mu

    # the proportional gain is applied to the instantaenous difference
    # in orbital elements, the integral gain is applied to a continuous
    # state whose derivative is the difference in orbital elements.
    # The difference in orbital elements is constant, and we run this
    # scenario for 10 seconds, so the integral gain / 10 should be equivalent
    # to the proportional gain.
    K_mat = np.diag([1e7] * 6)
    if integral:
        ctrl.setIntegralGain(K_mat / 10)
    else:
        ctrl.setProportionalGain(K_mat)

    # ClassicElements messages for target and current.
    # Use the same osculating elements as the meanOEFeedback test, since J2 is tiny.
    targetPayload = messaging.ClassicElementsMsgPayload()
    targetPayload.a = oe_c.a
    targetPayload.e = oe_c.e
    targetPayload.i = oe_c.i
    targetPayload.Omega = oe_c.Omega
    targetPayload.omega = oe_c.omega
    targetPayload.f = oe_c.f

    currentPayload = messaging.ClassicElementsMsgPayload()
    currentPayload.a = oe_d.a
    currentPayload.e = oe_d.e
    currentPayload.i = oe_d.i
    currentPayload.Omega = oe_d.Omega
    currentPayload.omega = oe_d.omega
    currentPayload.f = oe_d.f

    targetMsg = messaging.ClassicElementsMsg().write(targetPayload)
    currentMsg = messaging.ClassicElementsMsg().write(currentPayload)

    ctrl.targetOEInMsg.subscribeTo(targetMsg)
    ctrl.currentOEInMsg.subscribeTo(currentMsg)

    # Add controller under MuJoCo scene
    scene.AddModelToDynamicsTask(ctrl)

    # Controller recorder
    ctrlLog = ctrl.forceOutMsg.recorder()
    sim.AddModelToTask(unitTaskName, ctrlLog)

    # Initialize and run a single step
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(10))
    sim.ExecuteSimulation()

    # Extract outputs
    mean_force = meanLog.forceRequestInertial[-1]
    ctrl_force = ctrlLog.forceRequestInertial[-1]

    # Compare forces
    np.testing.assert_allclose(ctrl_force, mean_force, rtol=1e-6, atol=1e-6)


if __name__ == "__main__":
    assert couldImportMujoco
    test_orbitalElementControl(True)
