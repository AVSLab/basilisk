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
    Illustration of setting and recording module variables
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

    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.ModelTag = "cppModule2"
    scSim.AddModelToTask("dynamicsTask", mod2)

    # set module variables
    mod1.dummy = 1
    mod1.dumVector = [1., 2., 3.]
    mod2.dummy = 1
    mod2.dumVector = [1., 2., 3.]

    # request these module variables to be recorded
    scSim.AddVariableForLogging(mod1.ModelTag + ".dummy", macros.sec2nano(1.))
    scSim.AddVariableForLogging(mod1.ModelTag + ".dumVector", macros.sec2nano(1.), 0, 2)
    scSim.AddVariableForLogging(mod2.ModelTag + ".dummy", macros.sec2nano(1.))
    scSim.AddVariableForLogging(mod2.ModelTag + ".dumVector", macros.sec2nano(1.), 0, 2)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()

    print("mod1.dummy:")
    print(scSim.GetLogVariableData(mod1.ModelTag + ".dummy"))
    print("mod1.dumVector:")
    print(scSim.GetLogVariableData(mod1.ModelTag + ".dumVector"))
    print("mod2.dummy:")
    print(scSim.GetLogVariableData(mod2.ModelTag + ".dummy"))
    print("mod2.dumVector:")
    print(scSim.GetLogVariableData(mod2.ModelTag + ".dumVector"))

    return


if __name__ == "__main__":
    run()
