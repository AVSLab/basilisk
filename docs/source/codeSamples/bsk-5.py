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

import sys

import matplotlib.pyplot as plt
from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport


def run():
    """
    Illustration of creating stand-alone messages
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(1.)))

    # create modules
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.ModelTag = "cppModule1"
    scSim.AddModelToTask("dynamicsTask", mod1)

    # create stand-alone input message
    msgData = messaging.CModuleTemplateMsgPayload()
    msgData.dataVector = [1., 2., 3.]
    msg = messaging.CModuleTemplateMsg().write(msgData)

    # connect to stand-alone msg
    mod1.dataInMsg.subscribeTo(msg)

    # setup message recording
    msgRec = mod1.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", msgRec)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(10.0))
    scSim.ExecuteSimulation()

    # change input message and continue simulation
    msgData.dataVector = [-1., -2., -3.]
    msg.write(msgData)
    scSim.ConfigureStopTime(macros.sec2nano(20.0))
    scSim.ExecuteSimulation()

    # plot recorded data
    plt.close("all")
    figureList = {}
    plt.figure(1)
    for idx in range(3):
        plt.plot(msgRec.times() * macros.NANO2SEC, msgRec.dataVector[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('Module Data [units]')
    figureList["bsk-5"] = plt.figure(1)
    if "pytest" not in sys.modules:
        plt.show()
    plt.close("all")

    return figureList


if __name__ == "__main__":
    run()
