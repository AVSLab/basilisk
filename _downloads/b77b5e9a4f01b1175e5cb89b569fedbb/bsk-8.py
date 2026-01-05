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
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def run():
    """
    Illustration of enabling and disabling tasks
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("cTask", macros.sec2nano(1.)))
    dynProcess.addTask(scSim.CreateNewTask("cppTask", macros.sec2nano(1.)))

    # create modules
    mod1 = cModuleTemplate.cModuleTemplate()
    mod1.ModelTag = "cModule1"
    scSim.AddModelToTask("cTask", mod1)

    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.ModelTag = "cppModule2"
    scSim.AddModelToTask("cppTask", mod2)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    # execute BSK for a single step
    scSim.TotalSim.SingleStepProcesses()

    dynProcess.disableAllTasks()
    print("all tasks disabled")
    scSim.TotalSim.SingleStepProcesses()
    print("BSK executed a single simulation step")

    scSim.enableTask("cTask")
    scSim.TotalSim.SingleStepProcesses()
    print("BSK executed a single simulation step")

    scSim.enableTask("cppTask")
    scSim.TotalSim.SingleStepProcesses()
    print("BSK executed a single simulation step")

    scSim.disableTask("cppTask")
    scSim.TotalSim.SingleStepProcesses()
    print("BSK executed a single simulation step")

    return


if __name__ == "__main__":
    run()
