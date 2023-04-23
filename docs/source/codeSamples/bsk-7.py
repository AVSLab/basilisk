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

from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def run():
    """
    Illustration of re-directing module output message to stand-alone messages
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(1.)))

    # create modules
    mod1 = cModuleTemplate.cModuleTemplateConfig()
    mod1Wrap = scSim.setModelDataWrap(mod1)
    mod1Wrap.ModelTag = "cModule1"
    scSim.AddModelToTask("dynamicsTask", mod1Wrap, mod1)

    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.ModelTag = "cppModule2"
    scSim.AddModelToTask("dynamicsTask", mod2)

    # create stand-alone message with a C interface and re-direct
    # the C module output message writing to this stand-alone message
    cMsg = messaging.CModuleTemplateMsg_C()
    cMsg.write(messaging.CModuleTemplateMsgPayload())
    messaging.CModuleTemplateMsg_C_addAuthor(mod1.dataOutMsg, cMsg)

    # create stand-along message with a C++ interface and re-direct
    # the C++ module output message writing to this stand-alone message
    cppMsg = messaging.CModuleTemplateMsg()
    mod2.dataOutMsg = cppMsg

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()

    # read the message values and print them to the terminal
    print("mod1.dataOutMsg:")
    print(mod1.dataOutMsg.read().dataVector)
    print("cMsg:")
    print(cMsg.read().dataVector)
    print("mod2.dataOutMsg:")
    print(mod2.dataOutMsg.read().dataVector)
    print("cppMsg:")
    print(cppMsg.read().dataVector)

    return


if __name__ == "__main__":
    run()
