from numbers import Integral

import numpy as np

from Basilisk.utilities import deprecated, simHelpers


_DIRECT_VARIABLE_ERROR = (
    "addVariableLog supports direct module variables only; create a "
    "PythonVariableLogger during simulation setup for nested or computed time "
    "histories and retain its output with addRetentionFunction()"
)


class VariableRetentionParameters:
    """Store the configuration and runtime logger for a retained variable.

    :param str varName: Variable identifier in ``<ModelTag>.<variableName>`` format.
    :param int varRate: Minimum variable recording period in nanoseconds.
    :param int startIndex: Deprecated first retained component index.
    :param int stopIndex: Deprecated last retained component index, inclusive.
    :param str varType: Deprecated legacy C-array type name.
    """

    def __init__(
        self,
        varName,
        varRate,
        startIndex=None,
        stopIndex=None,
        varType=None,
    ):
        self.varName = varName
        self.varRate = varRate
        self.startIndex = startIndex
        self.stopIndex = stopIndex
        self.varType = varType
        self.logger = None


class MessageRetentionParameters:
    """Store the configuration for retained message fields.

    :param str name: Name of the message recorder.
    :param list retainedVars: Message payload fields to retain.
    """

    def __init__(self, name, retainedVars):
        self.msgRecName = name
        self.retainedVars = retainedVars


class RetentionPolicy:
    """Control which simulation data is retained and how it is stored.

    The first column of each retained message or variable array contains the
    corresponding simulation time in nanoseconds.
    """

    def __init__(self, rate=10_000_000_000):  # [ns]
        self.logRate = rate
        self.messageLogList = []
        self.varLogList = []
        self.dataCallback = None
        self.retentionFunctions = []

    def addMessageLog(self, name, retainedVars):
        self.messageLogList.append(MessageRetentionParameters(name, retainedVars))

    def addVariableLog(
        self,
        variableName,
        startIndex=None,
        stopIndex=None,
        varType=None,
        logRate=None,
    ):
        """Add a module variable to the retained Monte Carlo data.

        The variable identifier must use ``<ModelTag>.<variableName>`` format
        and refer to a direct public or getter-backed variable on a uniquely
        tagged module that supports the standard ``logger()`` API. The final
        period separates the model tag from the variable name, so model tags may
        contain periods. The complete variable is retained by default;
        multidimensional samples are flattened in row-major order.
        ``startIndex``, ``stopIndex``, and ``varType`` are compatibility
        arguments for the removed legacy variable-logging API and will be
        removed after 2027-08-26. New code should select component columns from
        the retained NumPy array instead.

        :param str variableName: Model tag and direct module variable name.
        :param int startIndex: Deprecated first retained component index.
        :param int stopIndex: Deprecated last retained component index, inclusive.
        :param str varType: Deprecated legacy C-array type name; ignored.
        :param int logRate: Minimum recording period in nanoseconds. The policy
            default is used when this argument is ``None``.
        """
        self._splitVariableName(variableName)
        startIndex, stopIndex = self._normalizeLegacyIndices(
            startIndex,
            stopIndex,
        )
        if startIndex is not None or stopIndex is not None or varType is not None:
            deprecated.deprecationWarn(
                "RetentionPolicy.addVariableLog legacy arguments",
                "2027/08/26",
                "Variable loggers now retain complete values directly. Select "
                "components from the retained NumPy array instead.",
            )

        if logRate is None:
            logRate = self.logRate
        logRate = self._normalizeLogRate(logRate)
        variableParameters = VariableRetentionParameters(
            variableName,
            logRate,
            startIndex,
            stopIndex,
            varType,
        )
        self.varLogList.append(variableParameters)

    @staticmethod
    def _normalizeLegacyIndices(startIndex, stopIndex):
        """Normalize and validate a deprecated inclusive component range."""
        if startIndex is None and stopIndex is None:
            return None, None
        if startIndex is None:
            startIndex = 0
        if stopIndex is None:
            stopIndex = startIndex
        if any(
            isinstance(index, bool) or not isinstance(index, Integral)
            for index in (startIndex, stopIndex)
        ):
            raise TypeError("startIndex and stopIndex must be integers")
        if startIndex < 0 or stopIndex < startIndex:
            raise ValueError(
                "startIndex and stopIndex must define a nonnegative inclusive range"
            )
        return int(startIndex), int(stopIndex)

    @staticmethod
    def _normalizeLogRate(logRate):
        """Validate and normalize a variable recording period in nanoseconds."""
        if isinstance(logRate, bool) or not isinstance(logRate, Integral):
            raise TypeError("logRate must be an integer number of nanoseconds")
        if logRate < 0:
            raise ValueError("logRate must be nonnegative")
        return int(logRate)

    @staticmethod
    def _splitVariableName(variableName):
        """Split and validate a retained module variable identifier."""
        if not isinstance(variableName, str) or variableName == "":
            raise ValueError("variableName must be a non-empty string")

        modelTag, separator, moduleVariableName = variableName.rpartition(".")
        if separator == "" or modelTag == "" or moduleVariableName == "":
            raise ValueError(
                "variableName must use '<ModelTag>.<variableName>' format"
            )
        if any(token in moduleVariableName for token in ("[", "]", "(", ")")):
            raise ValueError(_DIRECT_VARIABLE_ERROR)
        return modelTag, moduleVariableName

    @staticmethod
    def _findModelAndTask(simInstance, modelTag, retainedVariableName):
        """Return the uniquely tagged model and its owning simulation task."""
        matches = []
        prefixModelTags = set()
        for task in simInstance.TaskList:
            for model in task.TaskModels:
                candidateTag = getattr(model, "ModelTag", None)
                if candidateTag == modelTag:
                    matches.append((model, task))
                elif (
                    isinstance(candidateTag, str)
                    and candidateTag != ""
                    and retainedVariableName.startswith(candidateTag + ".")
                ):
                    prefixModelTags.add(candidateTag)

        if not matches:
            if prefixModelTags:
                matchingTags = ", ".join(sorted(prefixModelTags))
                raise ValueError(
                    f"Could not find a model with ModelTag '{modelTag}'. The "
                    f"identifier extends existing ModelTag(s): {matchingTags}. "
                    f"{_DIRECT_VARIABLE_ERROR}."
                )
            raise ValueError(f"Could not find a model with ModelTag '{modelTag}'")
        if len(matches) > 1:
            raise ValueError(
                f"ModelTag '{modelTag}' is not unique across simulation tasks"
            )
        return matches[0]

    @staticmethod
    def _getLoggerPriority(task):
        """Return a priority that schedules a logger after existing task models."""
        priorities = getattr(task, "TaskModelPriorities", None)
        if priorities is None or len(priorities) != len(task.TaskModels):
            raise RuntimeError(
                f"Task '{task.Name}' has inconsistent model-priority metadata"
            )
        if not priorities:
            raise RuntimeError(f"Task '{task.Name}' does not contain any models")
        # SysModelTask preserves insertion order for models with equal priority.
        # Reusing the lowest priority therefore places the new logger last.
        return min(priorities)

    def addLogsToSim(self, simInstance):
        """Create and schedule the variable loggers required by this policy."""
        RetentionPolicy.addRetentionPoliciesToSim(simInstance, [self])

    def _addLogsToSim(self, simInstance, loggerCache):
        """Create variable loggers, sharing compatible duplicate requests."""
        for variable in self.varLogList:
            modelTag, moduleVariableName = self._splitVariableName(variable.varName)
            cachedLogger = loggerCache.get(variable.varName)
            if cachedLogger is not None:
                variable.logger = cachedLogger
                continue

            model, task = self._findModelAndTask(
                simInstance,
                modelTag,
                variable.varName,
            )
            loggerFactory = getattr(model, "logger", None)
            if not callable(loggerFactory):
                raise TypeError(
                    f"Model '{modelTag}' does not provide the standard logger() "
                    "API; create a PythonVariableLogger during simulation setup "
                    "and retain its output with addRetentionFunction()"
                )

            variableLogger = loggerFactory(moduleVariableName, variable.varRate)
            variableLogger.ModelTag = f"RetentionLogger:{variable.varName}"
            loggerPriority = self._getLoggerPriority(task)
            simInstance.AddModelToTask(
                task.Name,
                variableLogger,
                ModelPriority=loggerPriority,
            )
            variable.logger = variableLogger
            loggerCache[variable.varName] = variableLogger

    def addRetentionFunction(self, function):
        """Add a callback that returns custom data after simulation execution.

        The callback receives the completed simulation instance and must return
        a dictionary. Its entries are merged into the retained ``custom`` data.

        :param callable function: Post-simulation data extraction callback.
        """
        self.retentionFunctions.append(function)

    def setDataCallback(self, dataCallback):
        self.dataCallback = dataCallback

    def executeCallback(self, data):
        if self.dataCallback is not None:
            self.dataCallback(data, self)

    @staticmethod
    def addRetentionPoliciesToSim(simInstance, retentionPolicies):
        """Add the variable loggers from a list of policies to a simulation.

        Compatible duplicate requests share one logger. Requests for the same
        retained-data key with different rates or component ranges are rejected.

        :param simInstance: Simulation instance receiving the loggers.
        :param list retentionPolicies: Retention policies defining data to log.
        """
        retentionPolicies = list(retentionPolicies)
        requestedSignatures = {}
        for retentionPolicy in retentionPolicies:
            for variable in retentionPolicy.varLogList:
                loggerSignature = (
                    variable.varRate,
                    variable.startIndex,
                    variable.stopIndex,
                )
                existingSignature = requestedSignatures.get(variable.varName)
                if (
                    existingSignature is not None
                    and existingSignature != loggerSignature
                ):
                    raise ValueError(
                        f"Variable '{variable.varName}' has conflicting retention "
                        "settings"
                    )
                requestedSignatures[variable.varName] = loggerSignature

        loggerCache = {}
        for retentionPolicy in retentionPolicies:
            retentionPolicy._addLogsToSim(simInstance, loggerCache)

    @staticmethod
    def _getRetainedVariableData(variable):
        """Return one runtime variable logger as a time-column data array."""
        if variable.logger is None:
            raise RuntimeError(
                f"Variable logger '{variable.varName}' was not added to the simulation"
            )

        _, moduleVariableName = RetentionPolicy._splitVariableName(variable.varName)
        logTimes = variable.logger.times()
        variableData = np.asarray(variable.logger[moduleVariableName])
        sampleCount = len(logTimes)
        if sampleCount == 0:
            componentCount = (
                variable.stopIndex-variable.startIndex+1
                if variable.startIndex is not None
                else 0
            )
            emptyData = np.empty((0, componentCount))
            return simHelpers.addTimeColumn(logTimes, emptyData)

        if variableData.ndim == 0 or variableData.shape[0] != sampleCount:
            raise RuntimeError(
                f"Variable logger '{variable.varName}' returned inconsistent sample data"
            )

        componentCount = variableData.size//sampleCount
        componentData = variableData.reshape(sampleCount, componentCount)
        if variable.startIndex is not None:
            if variable.stopIndex >= componentData.shape[1]:
                raise IndexError(
                    f"Legacy component range [{variable.startIndex}, "
                    f"{variable.stopIndex}] exceeds the width of '{variable.varName}'"
                )
            componentData = componentData[
                :, variable.startIndex:variable.stopIndex+1
            ]

        return simHelpers.addTimeColumn(logTimes, componentData)

    @staticmethod
    def getDataForRetention(simInstance, retentionPolicies):
        """Return the data selected by a list of retention policies.

        The returned dictionary contains ``messages``, ``variables``, and
        ``custom`` sub-dictionaries. Message and variable arrays have simulation
        time prepended as their first column. Multidimensional variable samples
        are flattened in row-major order.

        :param simInstance: Simulation instance containing completed recorders.
        :param list retentionPolicies: Policies defining the data to retain.
        :return: Retained simulation data grouped by source.
        :rtype: dict
        """
        data = {"messages": {}, "variables": {}, "custom": {}}
        for retentionPolicy in retentionPolicies:
            for msgParam in retentionPolicy.messageLogList:
                recorder = simInstance.msgRecList[msgParam.msgRecName]
                msgTimes = recorder.times()
                for varName in msgParam.retainedVars:
                    msgData = getattr(recorder, varName)
                    msgData = simHelpers.addTimeColumn(msgTimes, msgData)
                    messageKey = f"{msgParam.msgRecName}.{varName}"
                    data["messages"][messageKey] = msgData

            for variable in retentionPolicy.varLogList:
                retainedVariableData = RetentionPolicy._getRetainedVariableData(
                    variable
                )
                data["variables"][variable.varName] = retainedVariableData

            for retentionFunction in retentionPolicy.retentionFunctions:
                customData = retentionFunction(simInstance)
                data["custom"].update(customData)
        return data
