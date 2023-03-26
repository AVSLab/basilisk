#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.architecture import sysModel
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging

import numpy as np

def run():
    """
    Illustration of adding Basilisk Python modules to a task
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(5.)))

    # create copies of the Basilisk modules
    mod1 = cModuleTemplate.cModuleTemplateConfig()
    mod1Wrap = scSim.setModelDataWrap(mod1)
    mod1Wrap.ModelTag = "cModule1"
    scSim.AddModelToTask("dynamicsTask", mod1Wrap, mod1, 0)

    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.ModelTag = "cppModule2"
    scSim.AddModelToTask("dynamicsTask", mod2, None, 5)

    mod3 = cModuleTemplate.cModuleTemplateConfig()
    mod3Wrap = scSim.setModelDataWrap(mod3)
    mod3Wrap.ModelTag = "cModule3"
    scSim.AddModelToTask("dynamicsTask", mod3Wrap, mod3, 15)

    # The following is a Python module, which has a higher priority
    # then some of the C++/C modules. Observe in the script output
    # how the Python module is called in the order that respects
    # its priority with respect to the rest of the modules.
    mod4 = TestPythonModule()
    mod4.ModelTag = "pythonModule4"
    scSim.AddModelToTask("dynamicsTask", mod4, None, 10)

    mod2.dataInMsg.subscribeTo(mod4.dataOutMsg)
    mod4.dataInMsg.subscribeTo(mod3.dataOutMsg)

    # Set up recording
    mod2MsgRecorder = mod2.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", mod2MsgRecorder)

    #  initialize Simulation:
    scSim.InitializeSimulation()
    print("InitializeSimulation() completed...")

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(5.0))
    scSim.ExecuteSimulation()

    print("Recorded mod2.dataOutMsg.dataVector: ", mod2MsgRecorder.dataVector)

    return

class TestPythonModule(sysModel.SysModel):

    def __init__(self, *args):
        super().__init__(*args)
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    def Reset(self, CurrentSimNanos):
        # Ensure that self.dataInMsg is linked
        if not self.dataInMsg.isLinked():
            self.bskLogger.bskLog(bskLogging.BSK_ERROR, "TestPythonModule.dataInMsg is not linked.")

        # Initialiazing self.dataOutMsg
        payload = self.dataOutMsg.zeroMsgPayload
        payload.dataVector = np.array([0,0,0])
        self.dataOutMsg.write(payload, CurrentSimNanos, self.moduleID)

        self.bskLogger.bskLog(bskLogging.BSK_INFORMATION, "Reset in TestPythonModule")

    def UpdateState(self, CurrentSimNanos):
        # Read input message
        inPayload = self.dataInMsg()
        inputVector = inPayload.dataVector

        # Set output message
        payload = self.dataOutMsg.zeroMsgPayload
        payload.dataVector = self.dataOutMsg.read().dataVector + np.array([0,1,0]) + inputVector
        self.dataOutMsg.write(payload, CurrentSimNanos, self.moduleID)

        self.bskLogger.bskLog(bskLogging.BSK_INFORMATION, f"Python Module ID {self.moduleID} ran Update at {CurrentSimNanos*1e-9}s")

if __name__ == "__main__":
    run()
