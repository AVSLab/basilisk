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
from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport


def run():
    """
    Illustration of recording messages
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(1.)))

    # create modules
    mod1 = cModuleTemplate.cModuleTemplate()
    mod1.ModelTag = "cModule1"
    scSim.AddModelToTask("dynamicsTask", mod1)
    mod1.dataInMsg.subscribeTo(mod1.dataOutMsg)

    # setup message recording
    msgRec = mod1.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", msgRec)
    msgRec2 = mod1.dataOutMsg.recorder(macros.sec2nano(20.))
    scSim.AddModelToTask("dynamicsTask", msgRec2)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(60.0))
    scSim.ExecuteSimulation()

    # plot recorded data
    plt.close("all")
    plt.figure(1)
    figureList = {}
    for idx in range(3):
        plt.plot(msgRec.times() * macros.NANO2SEC, msgRec.dataVector[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$x_{' + str(idx) + '}$')
        plt.plot(msgRec2.times() * macros.NANO2SEC, msgRec2.dataVector[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\hat x_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('Module Data [units]')
    if "pytest" not in sys.modules:
        plt.show()
    figureList["bsk-4"] = plt.figure(1)
    plt.close("all")

    return figureList


if __name__ == "__main__":
    run()
