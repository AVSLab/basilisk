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

"""Regression tests for Monte Carlo variable retention."""

import numpy as np
import pytest

from Basilisk.architecture import sysModel
from Basilisk.utilities import (
    SimulationBaseClass,
    deprecated,
    macros,
    pythonVariableLogger,
    simHelpers,
)
from Basilisk.utilities.MonteCarlo.RetentionPolicy import RetentionPolicy


class RetainedVariableModel(sysModel.SysModel):
    """Provide deterministic module variables for retention tests."""

    def __init__(self, modelTag):
        super().__init__()
        self.ModelTag = modelTag
        self.scalarValue = 0.0  # [-]
        self.vectorOffsets = np.array([0.0, 10.0, 20.0])  # [-]
        self.vectorValue = self.vectorOffsets.copy()  # [-]
        self.matrixOffsets = np.array([[0.0, 1.0], [10.0, 11.0]])  # [-]
        self.matrixValue = self.matrixOffsets.copy()  # [-]
        self.emptyValue = np.array([])  # [-]
        self.times = 0.0  # [-]
        self._getterValue = 0.0  # [-]

    def getGetterValue(self):
        """Return a value exposed through the standard getter convention."""
        return self._getterValue

    def UpdateState(self, CurrentSimNanos):
        """Advance the deterministic values once per task execution."""
        self.scalarValue += 1.0  # [-]
        self.vectorValue = self.scalarValue+self.vectorOffsets
        self.matrixValue = self.scalarValue+self.matrixOffsets
        self.times = self.scalarValue+30.0  # [-]
        self._getterValue = self.scalarValue+40.0  # [-]


def createVariableSimulation(
    modelTag="retainedModel",
    modelPriority=-1,
    firstStart=0,  # [ns]
):
    """Create a short simulation containing one retained-variable model."""
    simulation = SimulationBaseClass.SimBaseClass()
    taskName = "retentionTask"
    taskRate = macros.sec2nano(1.0)  # [ns]
    process = simulation.CreateNewProcess("retentionProcess")
    task = simulation.CreateNewTask(taskName, taskRate, FirstStart=firstStart)
    process.addTask(task)

    model = RetainedVariableModel(modelTag)
    simulation.AddModelToTask(taskName, model, ModelPriority=modelPriority)
    stopTime = macros.sec2nano(2.0)  # [ns]
    simulation.ConfigureStopTime(stopTime)
    return simulation, task, model, taskRate


def executeSimulation(simulation):
    """Initialize and execute a configured test simulation."""
    simulation.InitializeSimulation()
    simulation.ExecuteSimulation()


def retainComputedVariable(simulation):
    """Return a computed logger value in the standard time-column format."""
    variableLogger = simulation.computedVariableLogger
    retainedValue = simHelpers.addTimeColumn(
        variableLogger.times(),
        variableLogger["doubleScalarValue"],
    )
    return {"doubleScalarValue": retainedValue}


def test_variable_retention_records_complete_values():
    """Verify complete scalar, vector, and matrix values include time."""
    simulation, task, model, taskRate = createVariableSimulation()
    policy = RetentionPolicy()
    policy.addVariableLog(f"{model.ModelTag}.scalarValue", logRate=taskRate)
    policy.addVariableLog(f"{model.ModelTag}.vectorValue", logRate=taskRate)
    policy.addVariableLog(f"{model.ModelTag}.matrixValue", logRate=taskRate)
    policy.addVariableLog(f"{model.ModelTag}.times", logRate=taskRate)
    policy.addVariableLog(f"{model.ModelTag}.getterValue", logRate=taskRate)

    policy.addLogsToSim(simulation)
    assert len(task.TaskModels) == 6
    assert all(variable.logger in task.TaskModels for variable in policy.varLogList)

    executeSimulation(simulation)
    retainedData = RetentionPolicy.getDataForRetention(simulation, [policy])

    expectedTimes = np.array([0, taskRate, 2*taskRate])  # [ns]
    expectedScalars = np.array([1.0, 2.0, 3.0])  # [-]
    expectedVectors = expectedScalars[:, None]+model.vectorOffsets
    expectedMatrices = (
        expectedScalars[:, None]+model.matrixOffsets.reshape(1, -1)
    )
    np.testing.assert_array_equal(
        retainedData["variables"][f"{model.ModelTag}.scalarValue"],
        np.column_stack((expectedTimes, expectedScalars)),
    )
    np.testing.assert_array_equal(
        retainedData["variables"][f"{model.ModelTag}.vectorValue"],
        np.column_stack((expectedTimes, expectedVectors)),
    )
    np.testing.assert_array_equal(
        retainedData["variables"][f"{model.ModelTag}.matrixValue"],
        np.column_stack((expectedTimes, expectedMatrices)),
    )
    np.testing.assert_array_equal(
        retainedData["variables"][f"{model.ModelTag}.times"],
        np.column_stack((expectedTimes, expectedScalars+30.0)),
    )
    np.testing.assert_array_equal(
        retainedData["variables"][f"{model.ModelTag}.getterValue"],
        np.column_stack((expectedTimes, expectedScalars+40.0)),
    )


def test_variable_retention_honors_deprecated_component_range():
    """Verify legacy inclusive slicing works during its deprecation period."""
    simulation, _, model, taskRate = createVariableSimulation()
    policy = RetentionPolicy()
    with pytest.warns(deprecated.BSKDeprecationWarning, match="complete values"):
        policy.addVariableLog(
            f"{model.ModelTag}.vectorValue",
            startIndex=1,
            stopIndex=2,
            varType="double",
            logRate=taskRate,
        )

    policy.addLogsToSim(simulation)
    executeSimulation(simulation)
    retainedData = RetentionPolicy.getDataForRetention(simulation, [policy])

    expectedTimes = np.array([0, taskRate, 2*taskRate])  # [ns]
    expectedScalars = np.array([1.0, 2.0, 3.0])  # [-]
    expectedVectors = expectedScalars[:, None]+model.vectorOffsets[1:3]
    np.testing.assert_array_equal(
        retainedData["variables"][f"{model.ModelTag}.vectorValue"],
        np.column_stack((expectedTimes, expectedVectors)),
    )


@pytest.mark.parametrize(
    ("startIndex", "stopIndex", "errorType"),
    [
        (-1, 0, ValueError),
        (2, 1, ValueError),
        (1.5, 2, TypeError),
        (True, 1, TypeError),
    ],
)
def test_variable_retention_rejects_invalid_component_range(
    startIndex,
    stopIndex,
    errorType,
):
    """Verify deprecated component ranges are valid inclusive indices."""
    policy = RetentionPolicy()

    with pytest.raises(errorType, match="startIndex and stopIndex"):
        policy.addVariableLog(
            "retainedModel.vectorValue",
            startIndex=startIndex,
            stopIndex=stopIndex,
        )


def test_variable_retention_handles_no_samples():
    """Verify a logger whose task never runs returns an empty time-column array."""
    firstStart = macros.sec2nano(10.0)  # [ns]
    simulation, _, model, taskRate = createVariableSimulation(
        firstStart=firstStart
    )
    policy = RetentionPolicy()
    variableName = f"{model.ModelTag}.vectorValue"
    policy.addVariableLog(variableName, logRate=taskRate)

    policy.addLogsToSim(simulation)
    executeSimulation(simulation)
    retainedData = RetentionPolicy.getDataForRetention(simulation, [policy])

    assert retainedData["variables"][variableName].shape == (0, 1)


def test_variable_retention_handles_zero_component_value():
    """Verify sampled empty arrays retain their time column without failing."""
    simulation, _, model, taskRate = createVariableSimulation()
    policy = RetentionPolicy()
    variableName = f"{model.ModelTag}.emptyValue"
    policy.addVariableLog(variableName, logRate=taskRate)

    policy.addLogsToSim(simulation)
    executeSimulation(simulation)
    retainedData = RetentionPolicy.getDataForRetention(simulation, [policy])

    expectedTimes = np.array([0, taskRate, 2*taskRate])  # [ns]
    np.testing.assert_array_equal(
        retainedData["variables"][variableName],
        expectedTimes[:, None],
    )


def test_custom_variable_logger_retention():
    """Verify the documented fallback retains a computed variable history."""
    simulation, task, model, taskRate = createVariableSimulation()
    simulation.computedVariableLogger = pythonVariableLogger.PythonVariableLogger(
        {"doubleScalarValue": lambda _: 2.0*model.scalarValue},
        taskRate,
    )
    simulation.AddModelToTask(task.Name, simulation.computedVariableLogger)

    policy = RetentionPolicy()
    policy.addRetentionFunction(retainComputedVariable)

    executeSimulation(simulation)
    retainedData = RetentionPolicy.getDataForRetention(simulation, [policy])

    expectedTimes = np.array([0, taskRate, 2*taskRate])  # [ns]
    expectedValues = np.array([2.0, 4.0, 6.0])  # [-]
    np.testing.assert_array_equal(
        retainedData["custom"]["doubleScalarValue"],
        np.column_stack((expectedTimes, expectedValues)),
    )


def test_variable_logger_is_added_only_to_owning_task():
    """Verify model-tag lookup schedules a logger on the matching task."""
    simulation = SimulationBaseClass.SimBaseClass()
    taskRate = macros.sec2nano(1.0)  # [ns]
    process = simulation.CreateNewProcess("retentionProcess")
    firstTask = simulation.CreateNewTask("firstTask", taskRate)
    secondTask = simulation.CreateNewTask("secondTask", taskRate)
    process.addTask(firstTask)
    process.addTask(secondTask)

    firstModel = RetainedVariableModel("firstModel")
    secondModel = RetainedVariableModel("secondModel")
    simulation.AddModelToTask(firstTask.Name, firstModel)
    simulation.AddModelToTask(secondTask.Name, secondModel)

    policy = RetentionPolicy()
    policy.addVariableLog("secondModel.scalarValue", logRate=taskRate)
    policy.addLogsToSim(simulation)

    variableLogger = policy.varLogList[0].logger
    assert variableLogger not in firstTask.TaskModels
    assert variableLogger in secondTask.TaskModels


def test_variable_logger_runs_after_negative_priority_model():
    """Verify retained values are sampled after a low-priority model update."""
    modelPriority = -10
    simulation, task, model, taskRate = createVariableSimulation(
        modelPriority=modelPriority
    )
    policy = RetentionPolicy()
    policy.addVariableLog(f"{model.ModelTag}.scalarValue", logRate=taskRate)
    policy.addLogsToSim(simulation)

    assert task.TaskModelPriorities[-1] == modelPriority
    executeSimulation(simulation)
    retainedData = RetentionPolicy.getDataForRetention(simulation, [policy])

    expectedTimes = np.array([0, taskRate, 2*taskRate])  # [ns]
    expectedScalars = np.array([1.0, 2.0, 3.0])  # [-]
    np.testing.assert_array_equal(
        retainedData["variables"][f"{model.ModelTag}.scalarValue"],
        np.column_stack((expectedTimes, expectedScalars)),
    )


def test_compatible_duplicate_variable_requests_share_logger():
    """Verify identical requests across policies use one scheduled logger."""
    simulation, task, model, taskRate = createVariableSimulation()
    firstPolicy = RetentionPolicy()
    secondPolicy = RetentionPolicy()
    variableName = f"{model.ModelTag}.scalarValue"
    firstPolicy.addVariableLog(variableName, logRate=taskRate)
    secondPolicy.addVariableLog(variableName, logRate=taskRate)

    RetentionPolicy.addRetentionPoliciesToSim(
        simulation,
        [firstPolicy, secondPolicy],
    )

    firstLogger = firstPolicy.varLogList[0].logger
    secondLogger = secondPolicy.varLogList[0].logger
    assert firstLogger is secondLogger
    assert task.TaskModels.count(firstLogger) == 1


def test_conflicting_duplicate_variable_requests_fail():
    """Verify conflicting settings fail before any logger is scheduled."""
    simulation, task, model, taskRate = createVariableSimulation()
    firstPolicy = RetentionPolicy()
    secondPolicy = RetentionPolicy()
    variableName = f"{model.ModelTag}.scalarValue"
    firstPolicy.addVariableLog(variableName, logRate=taskRate)
    secondPolicy.addVariableLog(variableName, logRate=2*taskRate)
    initialModelCount = len(task.TaskModels)

    with pytest.raises(ValueError, match="conflicting retention settings"):
        RetentionPolicy.addRetentionPoliciesToSim(
            simulation,
            [firstPolicy, secondPolicy],
        )

    assert len(task.TaskModels) == initialModelCount
    assert firstPolicy.varLogList[0].logger is None
    assert secondPolicy.varLogList[0].logger is None


def test_variable_retention_rejects_invalid_identifier_syntax():
    """Verify invalid identifiers fail before a Monte Carlo run starts."""
    policy = RetentionPolicy()

    with pytest.raises(ValueError, match="<ModelTag>.<variableName>"):
        policy.addVariableLog("missingSeparator")


def test_variable_retention_supports_dotted_model_tag():
    """Verify the last period separates a dotted model tag from its variable."""
    simulation, _, model, taskRate = createVariableSimulation("group.retainedModel")
    policy = RetentionPolicy()
    variableName = f"{model.ModelTag}.scalarValue"
    policy.addVariableLog(variableName, logRate=taskRate)

    policy.addLogsToSim(simulation)
    executeSimulation(simulation)
    retainedData = RetentionPolicy.getDataForRetention(simulation, [policy])

    expectedTimes = np.array([0, taskRate, 2*taskRate])  # [ns]
    expectedScalars = np.array([1.0, 2.0, 3.0])  # [-]
    np.testing.assert_array_equal(
        retainedData["variables"][variableName],
        np.column_stack((expectedTimes, expectedScalars)),
    )


def test_variable_retention_rejects_duplicate_model_tags():
    """Verify a variable identifier resolves to exactly one scheduled model."""
    simulation, task, model, taskRate = createVariableSimulation()
    duplicateModel = RetainedVariableModel(model.ModelTag)
    simulation.AddModelToTask(task.Name, duplicateModel)
    policy = RetentionPolicy()
    policy.addVariableLog(f"{model.ModelTag}.scalarValue", logRate=taskRate)

    with pytest.raises(ValueError, match="not unique"):
        policy.addLogsToSim(simulation)


def test_variable_retention_rejects_nested_variable():
    """Verify dotted paths below a matching model tag are rejected clearly."""
    simulation, _, _, _ = createVariableSimulation()
    policy = RetentionPolicy()
    policy.addVariableLog("retainedModel.child.value")

    with pytest.raises(ValueError, match="direct module variables only"):
        policy.addLogsToSim(simulation)


@pytest.mark.parametrize(
    ("logRate", "errorType"),
    [
        (-1, ValueError),
        (1.5, TypeError),
        (True, TypeError),
    ],
)
def test_variable_retention_rejects_invalid_log_rate(logRate, errorType):
    """Verify recording periods are nonnegative integer nanoseconds."""
    policy = RetentionPolicy()

    with pytest.raises(errorType, match="logRate"):
        policy.addVariableLog("retainedModel.scalarValue", logRate=logRate)


@pytest.mark.parametrize(
    ("variableName", "errorMatch"),
    [
        ("missingModel.scalarValue", "Could not find a model"),
        ("retainedModel.missingVariable", "Cannot log missingVariable"),
    ],
)
def test_variable_retention_rejects_unavailable_model_or_variable(
    variableName,
    errorMatch,
):
    """Verify unavailable models and variables fail with actionable errors."""
    simulation, _, _, _ = createVariableSimulation()
    policy = RetentionPolicy()
    policy.addVariableLog(variableName)

    with pytest.raises(ValueError, match=errorMatch):
        policy.addLogsToSim(simulation)
