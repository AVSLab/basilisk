#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import os
import shutil

import matplotlib.pyplot as plt
from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simHelpers


def make_graphviz_figure(scSim, inputMsg, show_plots=False):
    """Create the optional Graphviz tutorial output if Graphviz is available."""
    if shutil.which("dot") is None:
        return None

    return simHelpers.saveScenarioGraphvizFigure(
        "bsk-5a-graphviz",
        scSim,
        os.path.dirname(os.path.abspath(__file__)),
        show_plots=show_plots,
        graphvizLayout="vertical",
        extraMessages={"inputMsg": inputMsg},
        includeUnlinked=False,
        includeRecorders=True,
    )


def run(show_plots=False):
    """Illustrate Basilisk message connection visualization."""

    # Create a simulation container.
    scSim = SimulationBaseClass.SimBaseClass()

    # Create the simulation process and task.
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", simulationTimeStep))

    # Create a module with one input message and one output message.
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.ModelTag = "cppModule1"
    scSim.AddModelToTask("dynamicsTask", mod1)

    # Create a stand-alone input message and connect it to the module input.
    msgData = messaging.CModuleTemplateMsgPayload(dataVector=[1.0, 2.0, 3.0])
    inputMsg = messaging.CModuleTemplateMsg().write(msgData)
    mod1.dataInMsg.subscribeTo(inputMsg)

    # Add a recorder so the diagram also shows a regular module output link.
    msgRec = mod1.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", msgRec)

    # The message connection figure is optional and inspects the setup without
    # changing the simulation execution or logging behavior.
    figureList = {}
    figureList["bsk-5a"] = scSim.ShowMessageConnectionFigure(
        show_plots=show_plots,
        extraMessages={"inputMsg": inputMsg},
        includeUnlinked=False,
        includeRecorders=True,
    )

    # Graphviz writes a file rather than returning a Matplotlib figure.  The
    # tutorial uses this optional file to show the Graphviz renderer result.
    make_graphviz_figure(scSim, inputMsg, show_plots)
    plt.close("all")

    return figureList


if __name__ == "__main__":
    run(True)
