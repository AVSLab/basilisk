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
#

import numpy as np
from Basilisk.architecture.numbaModel import NumbaModel
from Basilisk.architecture import messaging, bskLogging
from Basilisk.utilities import SimulationBaseClass, macros


def run():
    """
    Illustration of NumbaModel: two JIT-compiled modules exchanging messages.
    """

    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", macros.sec2nano(1.0)))

    # AccumulatorModel tracks elapsed time in memory; it has no input messages.
    prod = AccumulatorModel()
    prod.ModelTag = "accumulator"
    scSim.AddModelToTask("task", prod)

    # FilterModel reads from AccumulatorModel and scales by a configurable gain.
    filt = FilterModel()
    filt.ModelTag = "filter"
    filt.memory.gain = 2.0                  # override default before Reset
    filt.dataInMsg.subscribeTo(prod.dataOutMsg)
    scSim.AddModelToTask("task", filt)

    recorder = filt.dataOutMsg.recorder()
    scSim.AddModelToTask("task", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(3.0))  # 4 ticks: t = 0, 1, 2, 3 s
    scSim.ExecuteSimulation()

    print("ticks recorded by accumulator:", int(prod.memory.ticks))
    print("filtered dataVector at t=3s:  ", recorder.dataVector[-1])


# =============================================================================
# Module 1: AccumulatorModel
#
# Demonstrates: memory namespace, CurrentSimNanos, no readers.
# =============================================================================
class AccumulatorModel(NumbaModel):
    """Outputs the cumulative simulation time (seconds) and a tick counter."""

    def __init__(self):
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()

        # Fields declared here become a C-level structured buffer after Reset.
        # The dtype is inferred from the value: 0.0 -> float64, 0 -> int64,
        # np.int32(0) -> int32, np.array(...) -> array field.
        self.memory.total_s = 0.0
        self.memory.ticks   = 0

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, CurrentSimNanos, memory):
        # Everything in this method is compiled by Numba in nopython mode.
        # See the "What is allowed" section in the documentation.
        memory.total_s += np.float64(CurrentSimNanos) * 1e-9
        memory.ticks   += 1
        dataOutMsgPayload.dataVector[0] = memory.total_s
        dataOutMsgPayload.dataVector[1] = float(memory.ticks)


# =============================================================================
# Module 2: FilterModel
#
# Demonstrates: scalar reader with IsLinked guard, bskLogger, memory gain.
# =============================================================================
class FilterModel(NumbaModel):
    """Scales each element of the input dataVector by a configurable gain."""

    def __init__(self):
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.gain = 1.0               # default; caller may override

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked,
                        dataOutMsgPayload, bskLogger, memory):
        if not dataInMsgIsLinked:
            bskLogger.bskLog(bskLogging.BSK_WARNING, "FilterModel: input not linked")
            return
        for i in range(3):
            dataOutMsgPayload.dataVector[i] = dataInMsgPayload.dataVector[i] * memory.gain


if __name__ == "__main__":
    run()
