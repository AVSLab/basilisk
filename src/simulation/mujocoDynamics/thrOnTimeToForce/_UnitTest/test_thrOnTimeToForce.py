#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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
#

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.utilities import SimulationBaseClass, macros

try:
    from Basilisk.simulation import thrOnTimeToForce
    couldImportThrOnTimeToForce = True
except Exception:
    couldImportThrOnTimeToForce = False


@pytest.mark.skipif(not couldImportThrOnTimeToForce, reason="Compiled Basilisk without thrOnTimeToForce")
def test_thrOnTimeToForceCountdown():
    """
    Validate pulse realization and timer countdown with fixed-step updates.
    """
    unitTaskName = "unitTask"
    unitProcessName = "unitProcess"
    dt = 0.1

    sim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(dt)
    proc = sim.CreateNewProcess(unitProcessName)
    proc.addTask(sim.CreateNewTask(unitTaskName, testProcessRate))

    module = thrOnTimeToForce.ThrOnTimeToForce()
    module.ModelTag = "thrOnTimeToForceTag"
    thrMag = [2.0, 3.0]
    module.setThrMag(thrMag)
    for _ in range(len(thrMag)):
        module.addThruster()
    sim.AddModelToTask(unitTaskName, module)

    onTimeData = messaging.THRArrayOnTimeCmdMsgPayload()
    onTimeData.OnTimeRequest = [0.2, 0.3]
    onTimeMsg = messaging.THRArrayOnTimeCmdMsg().write(onTimeData)
    module.onTimeInMsg.subscribeTo(onTimeMsg)

    out0 = module.thrusterForceOutMsgs[0].recorder()
    out1 = module.thrusterForceOutMsgs[1].recorder()
    sim.AddModelToTask(unitTaskName, out0)
    sim.AddModelToTask(unitTaskName, out1)

    sim.InitializeSimulation()

    sim.TotalSim.SingleStepProcesses()
    sim.TotalSim.SingleStepProcesses()
    sim.TotalSim.SingleStepProcesses()
    sim.TotalSim.SingleStepProcesses()
    sim.TotalSim.SingleStepProcesses()

    thr0 = np.array(out0.input)
    thr1 = np.array(out1.input)

    np.testing.assert_allclose(thr0, [2.0, 2.0, 0.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(thr1, [3.0, 3.0, 3.0, 0.0, 0.0], atol=1e-12)


@pytest.mark.skipif(not couldImportThrOnTimeToForce, reason="Compiled Basilisk without thrOnTimeToForce")
def test_thrOnTimeToForceGapThenFire():
    """
    Validate behavior when module is disabled during a gap between message write time and
    module call time
    """
    unitTaskName = "unitTask"
    unitProcessName = "unitProcess"
    dt = 0.1

    sim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(dt)
    proc = sim.CreateNewProcess(unitProcessName)
    proc.addTask(sim.CreateNewTask(unitTaskName, testProcessRate))

    module = thrOnTimeToForce.ThrOnTimeToForce()
    module.ModelTag = "thrOnTimeToForceTag"
    module.setThrMag([1.5])
    module.addThruster()
    sim.AddModelToTask(unitTaskName, module)

    onTimeData = messaging.THRArrayOnTimeCmdMsgPayload()
    onTimeData.OnTimeRequest = [0.0]
    onTimeMsg = messaging.THRArrayOnTimeCmdMsg()
    onTimeMsg.write(onTimeData)
    module.onTimeInMsg.subscribeTo(onTimeMsg)

    out = module.thrusterForceOutMsgs[0].recorder()
    sim.AddModelToTask(unitTaskName, out)

    sim.InitializeSimulation()

    # Phase 1: run for 1.0 sec with the module disabled
    sim.disableTask(unitTaskName)
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Write a new thr on time message
    onTimeData.OnTimeRequest = [0.2]
    onTimeMsg.write(onTimeData)

    # Keep module inactive for additional gap time before firing phase starts
    sim.ConfigureStopTime(macros.sec2nano(1.5))
    sim.ExecuteSimulation()

    # Phase 2: thruster firing (module enabled)
    sim.enableTask(unitTaskName)
    sim.TotalSim.SingleStepProcesses()
    sim.TotalSim.SingleStepProcesses()
    sim.TotalSim.SingleStepProcesses()
    sim.TotalSim.SingleStepProcesses()

    thr = np.array(out.input)
    np.testing.assert_allclose(thr, [1.5, 1.5, 0.0, 0.0], atol=1e-12)


if __name__ == "__main__":
    test_thrOnTimeToForceCountdown()
    test_thrOnTimeToForceGapThenFire()
