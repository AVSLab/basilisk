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
from Basilisk.fswAlgorithms import fswModuleTemplate


def run():
    """
    Illustration of adding Basilisk modules to a task
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(5.)))

    # create copies of the Basilisk modules
    mod1 = fswModuleTemplate.fswModuleTemplateConfig()
    mod1Wrap = scSim.setModelDataWrap(mod1)
    mod1Wrap.ModelTag = "Module1"

    mod2 = fswModuleTemplate.fswModuleTemplateConfig()
    mod2Wrap = scSim.setModelDataWrap(mod1)
    mod2Wrap.ModelTag = "Module2"

    mod3 = fswModuleTemplate.fswModuleTemplateConfig()
    mod3Wrap = scSim.setModelDataWrap(mod3)
    mod3Wrap.ModelTag = "Module3"

    scSim.AddModelToTask("dynamicsTask", mod1Wrap, mod1)
    scSim.AddModelToTask("dynamicsTask", mod2Wrap, mod2, 10)
    scSim.AddModelToTask("dynamicsTask", mod3Wrap, mod3, 5)

    #  initialize Simulation:
    scSim.InitializeSimulation()
    print("InitializeSimulation() completed...")

    #   configure a simulation stop time time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(5.0))
    scSim.ExecuteSimulation()

    return


if __name__ == "__main__":
    run()
