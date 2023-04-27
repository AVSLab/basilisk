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

def test_PySysModel():
    testResults, testMessage = 0, []

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

    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.ModelTag = "cppModule2"

    mod3 = cModuleTemplate.cModuleTemplateConfig()
    mod3Wrap = scSim.setModelDataWrap(mod3)
    mod3Wrap.ModelTag = "cModule3"

    mod4 = PythonModule()
    mod4.ModelTag = "pythonModule4"

    mod2.dataInMsg.subscribeTo(mod4.dataOutMsg)

    scSim.AddModelToTask("dynamicsTask", mod1Wrap, mod1, 0)
    scSim.AddModelToTask("dynamicsTask", mod2, None, 5)
    scSim.AddModelToTask("dynamicsTask", mod3Wrap, mod3, 15)
    scSim.AddModelToTask("dynamicsTask", mod4, None, 10)

    # Set up recording
    mod2MsgRecorder = mod2.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", mod2MsgRecorder)

    # initialize Simulation:
    scSim.InitializeSimulation()

    # configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(5.0))
    scSim.ExecuteSimulation()

    if mod4.CallCounts != 2:
        testResults += 1
        testMessage.append("TestPythonModule::UpdateState was not called")

    if mod2MsgRecorder.dataVector[1,1] == 0:
        testResults += 1
        testMessage.append("Message from TestPythonModule was not connected to message in mod2")
    elif mod2MsgRecorder.dataVector[1,1] == 1:
        testResults += 1
        testMessage.append("TestPythonModule does not run before mod2 despite having greater priority")

    assert testResults < 1, testMessage

class PythonModule(sysModel.SysModel):

    def __init__(self, *args):
        super().__init__(*args)
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    def Reset(self, CurrentSimNanos):
        payload = self.dataOutMsg.zeroMsgPayload
        payload.dataVector = np.array([0,0,0])
        self.dataOutMsg.write(payload, CurrentSimNanos, self.moduleID)
        self.bskLogger.bskLog(bskLogging.BSK_INFORMATION, "Reset in TestPythonModule")

    def UpdateState(self, CurrentSimNanos):
        payload = self.dataOutMsg.zeroMsgPayload
        payload.dataVector = self.dataOutMsg.read().dataVector + np.array([0,1,0])
        self.dataOutMsg.write(payload, CurrentSimNanos, self.moduleID)
        self.bskLogger.bskLog(bskLogging.BSK_INFORMATION, f"Python Module ID {self.moduleID} ran Update at {CurrentSimNanos*1e-9}s")

if __name__ == "__main__":
    test_PySysModel()
