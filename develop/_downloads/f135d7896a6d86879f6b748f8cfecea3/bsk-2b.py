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

from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def run():
    """
    Controlling the simulation time
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")
    fswProcess = scSim.CreateNewProcess("fswProcess", 10)

    # create the dynamics task and specify the integration update time
    fswProcess.addTask(scSim.CreateNewTask("fswTask1", macros.sec2nano(1.)))
    fswProcess.addTask(scSim.CreateNewTask("fswTask2", macros.sec2nano(2.)))
    fswProcess.addTask(scSim.CreateNewTask("fswTask3", macros.sec2nano(3.)), 10)
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask1", macros.sec2nano(1.)))
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask2", macros.sec2nano(5.)), 10)
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask3", macros.sec2nano(10.)))

    # create modules
    mod1 = cModuleTemplate.cModuleTemplate()
    mod1.ModelTag = "cModule1"

    mod2 = cModuleTemplate.cModuleTemplate()
    mod2.ModelTag = "cModule2"

    # add modules to various task lists
    scSim.AddModelToTask("dynamicsTask1", mod1, 4)
    scSim.AddModelToTask("dynamicsTask1", mod2, 5)
    scSim.AddModelToTask("dynamicsTask2", mod2)
    scSim.AddModelToTask("dynamicsTask2", mod1)
    scSim.AddModelToTask("dynamicsTask3", mod1)
    scSim.AddModelToTask("dynamicsTask3", mod2)

    scSim.AddModelToTask("fswTask1", mod1)
    scSim.AddModelToTask("fswTask1", mod2, 2)
    scSim.AddModelToTask("fswTask2", mod2)
    scSim.AddModelToTask("fswTask2", mod1)
    scSim.AddModelToTask("fswTask3", mod1)
    scSim.AddModelToTask("fswTask3", mod2)

    # print to the terminal window the execution order of the processes, task lists and modules
    scSim.ShowExecutionOrder()

    # uncomment this code to show the execution order figure and save it off
    # fig = scSim.ShowExecutionFigure(False)
    # fig.savefig("qs-bsk-2b-order.svg", transparent=True, bbox_inches = 'tight', pad_inches = 0)

    return


if __name__ == "__main__":
    run()
