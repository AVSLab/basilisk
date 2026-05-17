from unittest.mock import MagicMock, patch
import shutil

import matplotlib
import numpy as np
import pytest

matplotlib.use("Agg", force=True)

import matplotlib.pyplot as plt

from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cModuleTemplate, cppModuleTemplate
from Basilisk.simulation import simpleNav
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport


@pytest.mark.parametrize("stopTime1", [0.0, 8.0, 10.0, 12.0])
@pytest.mark.parametrize("stopTime2", [0.0, 8.0, 10.0, 12.0, 95.0, 100.0, 105.0])
@pytest.mark.parametrize("StopCondition", ["<=", ">="])
def test_ConfigureStopTime(stopTime1, stopTime2, StopCondition):
    sim_rate = 10.0

    # Skip nonsense tests
    if stopTime1 > stopTime2:
        return

    # Create a simulation
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("TestProcess")
    simulationTimeStep = macros.sec2nano(sim_rate)
    dynProcess.addTask(scSim.CreateNewTask("TestTask", simulationTimeStep))
    scSim.InitializeSimulation()

    for stopTime in [stopTime1, stopTime2]:
        scSim.ConfigureStopTime(macros.sec2nano(stopTime), StopCondition)
        scSim.ExecuteSimulation()

        if StopCondition == "<=":
            assert scSim.TotalSim.CurrentNanos == macros.sec2nano(
                np.floor(stopTime / sim_rate) * sim_rate
            )
        elif StopCondition == ">=":
            assert scSim.TotalSim.CurrentNanos == macros.sec2nano(
                np.ceil(stopTime / sim_rate) * sim_rate
            )


@pytest.mark.parametrize(
    "creationTime,eventRate,expectedCallsExactRate,expectedCallsInexactRate",
    [
        (0.0, 10.0, 10, 10),
        (10.0, 10.0, 9, 9),
        (0.0, 40.0, 3, 3),
        (10.0, 40.0, 2, 3),
        (0.0, np.pi, 1, 10),
        (0.0, 5.0, 10, 10),
        (0.0, 7.5, 4, 10),
        (0.0, 12.5, 2, 5),
        (0.0, 15.0, 4, 5),
    ],
)
@pytest.mark.parametrize("exactRateMatch", [True, False])
def test_Events(
    creationTime,
    eventRate,
    expectedCallsExactRate,
    expectedCallsInexactRate,
    exactRateMatch,
):
    sim_rate = 10.0
    sim_duration = 100.0

    # Create a simulation
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("TestProcess")
    simulationTimeStep = macros.sec2nano(sim_rate)
    dynProcess.addTask(scSim.CreateNewTask("TestTask", simulationTimeStep))
    scSim.InitializeSimulation()

    conditionFunction = MagicMock(return_value=False)

    # Step to event creation time
    scSim.ConfigureStopTime(macros.sec2nano(creationTime))
    scSim.ExecuteSimulation()

    # Create event and step through the rest of the sim
    scSim.createNewEvent(
        "TestEvent",
        conditionFunction=conditionFunction,
        eventRate=macros.sec2nano(eventRate),
        eventActive=True,
        exactRateMatch=exactRateMatch,
    )
    scSim.ConfigureStopTime(macros.sec2nano(sim_duration))
    scSim.ExecuteSimulation()

    if exactRateMatch:
        assert conditionFunction.call_count == expectedCallsExactRate
    else:
        assert conditionFunction.call_count == expectedCallsInexactRate


def test_GetMessageConnectionGraph_with_extra_messages():
    """Check message graph extraction for module and stand-alone message links."""
    scSim = SimulationBaseClass.SimBaseClass()
    testProcess = scSim.CreateNewProcess("testProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    testProcess.addTask(scSim.CreateNewTask("testTask", simulationTimeStep))

    cModule = cModuleTemplate.cModuleTemplate()
    cModule.ModelTag = "cModule"
    cppModule = cppModuleTemplate.CppModuleTemplate()
    cppModule.ModelTag = "cppModule"

    extraMsg = messaging.CModuleTemplateMsg().write(
        messaging.CModuleTemplateMsgPayload()
    )
    cModule.dataInMsg.subscribeTo(extraMsg)
    cppModule.dataInMsg.subscribeTo(cModule.dataOutMsg)

    scSim.AddModelToTask("testTask", cModule)
    scSim.AddModelToTask("testTask", cppModule)

    graph = scSim.GetMessageConnectionGraph(extraMessages={"extraMsg": extraMsg})
    edgePairs = {(edge["sourceName"], edge["targetOwner"]) for edge in graph["edges"]}
    targetNames = {edge["targetName"] for edge in graph["edges"]}
    moduleOutputs = {
        (output["moduleTag"], output["name"])
        for output in graph["outputs"]
        if output["ownerType"] == "module"
    }

    assert len(graph["modules"]) == 2
    assert ("extraMsg", "module:0") in edgePairs
    assert ("dataOutMsg", "module:1") in edgePairs
    assert ("cppModule", "dataOutMsg") in moduleOutputs
    assert targetNames == {"dataInMsg"}
    assert graph["unlinkedInputs"] == []
    assert graph["unresolvedInputs"] == []


def test_ShowMessageConnectionFigure_returns_figure():
    """Check that the message graph figure can be generated headlessly."""
    scSim = SimulationBaseClass.SimBaseClass()
    testProcess = scSim.CreateNewProcess("testProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    testProcess.addTask(scSim.CreateNewTask("testTask", simulationTimeStep))

    cModule = cModuleTemplate.cModuleTemplate()
    cModule.ModelTag = "cModule"
    cppModule = cppModuleTemplate.CppModuleTemplate()
    cppModule.ModelTag = "cppModule"
    cppModule.dataInMsg.subscribeTo(cModule.dataOutMsg)

    scSim.AddModelToTask("testTask", cModule)
    scSim.AddModelToTask("testTask", cppModule)

    fig = scSim.ShowMessageConnectionFigure(show_plots=False)

    assert len(fig.axes) == 1
    assert fig.number not in plt.get_fignums()
    plt.close(fig)


def test_GetMessageConnectionDot_contains_graphviz_edges():
    """Check that the message graph can be exported as Graphviz DOT text."""
    scSim = SimulationBaseClass.SimBaseClass()
    testProcess = scSim.CreateNewProcess("testProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    testProcess.addTask(scSim.CreateNewTask("testTask", simulationTimeStep))

    cModule = cModuleTemplate.cModuleTemplate()
    cModule.ModelTag = "cModule"
    cppModule = cppModuleTemplate.CppModuleTemplate()
    cppModule.ModelTag = "cppModule"

    extraMsg = messaging.CModuleTemplateMsg().write(
        messaging.CModuleTemplateMsgPayload()
    )
    cModule.dataInMsg.subscribeTo(extraMsg)
    cppModule.dataInMsg.subscribeTo(cModule.dataOutMsg)

    scSim.AddModelToTask("testTask", cModule)
    scSim.AddModelToTask("testTask", cppModule)

    dotText = scSim.GetMessageConnectionDot(
        extraMessages={"extraMsg": extraMsg},
        includeUnlinked=False,
    )

    assert "digraph BSKMessageConnections" in dotText
    assert 'rankdir="TB"' in dotText
    assert "extraMsg" in dotText
    assert 'style="dashed"' in dotText
    assert 'style="solid"' in dotText

    horizontalDotText = scSim.GetMessageConnectionDot(
        extraMessages={"extraMsg": extraMsg},
        includeUnlinked=False,
        graphvizLayout="horizontal",
    )
    assert 'rankdir="LR"' in horizontalDotText


def test_ShowMessageConnectionFigure_graphviz_writes_file(tmp_path):
    """Check that the Graphviz renderer writes a rendered file when available."""
    if shutil.which("dot") is None:
        pytest.skip("Graphviz dot executable is not available.")

    scSim = SimulationBaseClass.SimBaseClass()
    testProcess = scSim.CreateNewProcess("testProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    testProcess.addTask(scSim.CreateNewTask("testTask", simulationTimeStep))

    cModule = cModuleTemplate.cModuleTemplate()
    cModule.ModelTag = "cModule"
    cppModule = cppModuleTemplate.CppModuleTemplate()
    cppModule.ModelTag = "cppModule"
    cppModule.dataInMsg.subscribeTo(cModule.dataOutMsg)

    scSim.AddModelToTask("testTask", cModule)
    scSim.AddModelToTask("testTask", cppModule)

    outputPath = tmp_path / "messageConnections.svg"
    renderedPath = scSim.ShowMessageConnectionFigure(
        renderer="graphviz",
        fileName=str(outputPath),
        includeUnlinked=False,
    )

    assert renderedPath == str(outputPath)
    assert outputPath.exists()
    assert outputPath.read_text().lstrip().startswith("<?xml")


def test_saveScenarioGraphvizFigure_uses_documentation_image_path(tmp_path):
    """Check that Graphviz scenario figures are saved to the docs image path."""
    if shutil.which("dot") is None:
        pytest.skip("Graphviz dot executable is not available.")

    docsPath = tmp_path / "docs" / "source"
    docsPath.mkdir(parents=True)
    scenarioPath = tmp_path / "examples"
    scenarioPath.mkdir()
    expectedPath = docsPath / "_images" / "Scenarios" / "messageFlow.svg"
    scSim = MagicMock()
    scSim.ShowMessageConnectionFigure.return_value = str(expectedPath)

    renderedPath = unitTestSupport.saveScenarioGraphvizFigure(
        "messageFlow",
        scSim,
        str(scenarioPath),
        includeUnlinked=False,
    )

    assert renderedPath == str(expectedPath)
    assert expectedPath.parent.exists()
    scSim.ShowMessageConnectionFigure.assert_called_once_with(
        renderer="graphviz",
        fileName=str(expectedPath),
        show_plots=False,
        graphvizFormat="svg",
        includeUnlinked=False,
    )


def test_saveScenarioGraphvizFigure_skips_without_graphviz(tmp_path):
    """Check that optional Graphviz scenario figures do not require Graphviz."""
    docsPath = tmp_path / "docs" / "source"
    docsPath.mkdir(parents=True)
    scenarioPath = tmp_path / "examples"
    scenarioPath.mkdir()
    scSim = MagicMock()

    with patch("Basilisk.utilities.unitTestSupport.shutil.which", return_value=None):
        renderedPath = unitTestSupport.saveScenarioGraphvizFigure(
            "messageFlow",
            scSim,
            str(scenarioPath),
            includeUnlinked=False,
        )

    assert renderedPath is None
    scSim.ShowMessageConnectionFigure.assert_not_called()


def test_GetMessageConnectionGraph_with_recorder():
    """Check that recorder modules show their source message connection."""
    scSim = SimulationBaseClass.SimBaseClass()
    testProcess = scSim.CreateNewProcess("testProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    testProcess.addTask(scSim.CreateNewTask("testTask", simulationTimeStep))

    cModule = cModuleTemplate.cModuleTemplate()
    cModule.ModelTag = "cModule"
    dataRec = cModule.dataOutMsg.recorder()

    scSim.AddModelToTask("testTask", cModule)
    scSim.AddModelToTask("testTask", dataRec)

    graph = scSim.GetMessageConnectionGraph(includeUnlinked=False)
    edgePairs = {(edge["sourceName"], edge["targetName"]) for edge in graph["edges"]}

    assert ("dataOutMsg", "recordedMsg") in edgePairs


def test_GetMessageConnectionGraph_with_recorder_before_sim_creation():
    """Check recorder source tracking before creating the simulation object."""
    cModule = cModuleTemplate.cModuleTemplate()
    cModule.ModelTag = "cModule"
    dataRec = cModule.dataOutMsg.recorder()

    scSim = SimulationBaseClass.SimBaseClass()
    testProcess = scSim.CreateNewProcess("testProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    testProcess.addTask(scSim.CreateNewTask("testTask", simulationTimeStep))

    scSim.AddModelToTask("testTask", cModule)
    scSim.AddModelToTask("testTask", dataRec)

    graph = scSim.GetMessageConnectionGraph(includeUnlinked=False)
    edgePairs = {(edge["sourceName"], edge["targetName"]) for edge in graph["edges"]}

    assert ("dataOutMsg", "recordedMsg") in edgePairs


def test_GetMessageConnectionGraph_with_input_message_recorder():
    """Check that input message recorders show their recorded input."""
    scSim = SimulationBaseClass.SimBaseClass()
    testProcess = scSim.CreateNewProcess("testProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    testProcess.addTask(scSim.CreateNewTask("testTask", simulationTimeStep))

    cModule = cModuleTemplate.cModuleTemplate()
    cModule.ModelTag = "cModule"
    cppModule = cppModuleTemplate.CppModuleTemplate()
    cppModule.ModelTag = "cppModule"
    cppModule.dataInMsg.subscribeTo(cModule.dataOutMsg)
    dataRec = cppModule.dataInMsg.recorder()

    scSim.AddModelToTask("testTask", cModule)
    scSim.AddModelToTask("testTask", cppModule)
    scSim.AddModelToTask("testTask", dataRec)

    graph = scSim.GetMessageConnectionGraph(includeUnlinked=False)
    edgePairs = {
        (edge["sourceName"], edge["sourceOwner"], edge["targetName"], edge["targetOwner"])
        for edge in graph["edges"]
    }

    assert ("dataInMsg", "module:1", "recordedMsg", "module:2") in edgePairs
    assert ("dataOutMsg", "module:0", "recordedMsg", "module:2") not in edgePairs


def test_GetMessageConnectionGraph_can_skip_recorders():
    """Check that recorder modules can be omitted from the message graph."""
    scSim = SimulationBaseClass.SimBaseClass()
    testProcess = scSim.CreateNewProcess("testProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    testProcess.addTask(scSim.CreateNewTask("testTask", simulationTimeStep))

    cModule = cModuleTemplate.cModuleTemplate()
    cModule.ModelTag = "cModule"
    dataRec = cModule.dataOutMsg.recorder()

    scSim.AddModelToTask("testTask", cModule)
    scSim.AddModelToTask("testTask", dataRec)

    graph = scSim.GetMessageConnectionGraph(
        includeUnlinked=False,
        includeRecorders=False,
    )
    moduleTags = {module["tag"] for module in graph["modules"]}
    edgePairs = {(edge["sourceName"], edge["targetName"]) for edge in graph["edges"]}

    assert moduleTags == {"cModule"}
    assert ("dataOutMsg", "recordedMsg") not in edgePairs


def test_GetMessageConnectionGraph_handles_duplicate_model_tags():
    """Check that duplicate model tags do not mix message endpoints."""
    scSim = SimulationBaseClass.SimBaseClass()
    testProcess = scSim.CreateNewProcess("testProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    testProcess.addTask(scSim.CreateNewTask("testTask", simulationTimeStep))

    cModule = cModuleTemplate.cModuleTemplate()
    cModule.ModelTag = "duplicateTag"
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "duplicateTag"

    scSim.AddModelToTask("testTask", cModule, 1)
    scSim.AddModelToTask("testTask", sNavObject, 2)

    graph = scSim.GetMessageConnectionGraph(includeUnlinked=False)

    assert graph["modules"][0]["tag"] == "duplicateTag"
    assert {output["name"] for output in graph["modules"][0]["outputs"]} == {
        "attOutMsg",
        "transOutMsg",
    }
    assert graph["modules"][1]["tag"] == "duplicateTag"
    assert {output["name"] for output in graph["modules"][1]["outputs"]} == {
        "dataOutMsg",
    }
