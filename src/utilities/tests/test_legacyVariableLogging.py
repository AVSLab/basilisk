# ISC License
#
# Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from numpy.testing import assert_array_equal

from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
import warnings
from Basilisk.utilities import deprecated

def test_legacy_variable_logging(show_plots):
    __tracebackhide__ = True

    warnings.filterwarnings("ignore", category=deprecated.BSKDeprecationWarning)

    simulation = SimulationBaseClass.SimBaseClass()

    process = simulation.CreateNewProcess("testProcess")

    task1Name = "task1"
    process.addTask(simulation.CreateNewTask(task1Name, macros.sec2nano(1.0)))
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.r_CN_NInit = [0.0, 0.0, 0.0]
    scObject.hub.v_CN_NInit = [1.0, 0.0, 0.0]
    simulation.AddModelToTask(task1Name, scObject)

    # Adding this second task and module are necessary for creating a runtime task/module graph that requires
    # the AddVariableForLogging function to find the correct module and task for the variable being logged.
    # The task is then disabled to ensure that the underlying logger does not execute on the second task
    # which masks the fact that the function has not found the correct task and module.
    task2Name = "task2"
    process.addTask(simulation.CreateNewTask(task2Name, macros.sec2nano(1.0)))
    scObject1 = spacecraft.Spacecraft()
    scObject1.ModelTag = "spacecraftBody1"
    simulation.AddModelToTask(task2Name, scObject1)
    simulation.disableTask(task2Name)

    # Log a single variable via both the legacy and current mechanism
    simulation.AddVariableForLogging("spacecraftBody.totOrbEnergy")

    scObjectLogger = scObject.logger(["totOrbEnergy"])
    simulation.AddModelToTask(task1Name, scObjectLogger)

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(2.0))
    simulation.ExecuteSimulation()

    assert_array_equal(scObjectLogger.totOrbEnergy, simulation.GetLogVariableData('spacecraftBody.totOrbEnergy')[:, 1])
