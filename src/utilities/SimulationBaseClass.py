# ISC License
#
# Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


# Import some architectural stuff that we will probably always use
import html
import math
import os
import shutil
import subprocess
import tempfile
import webbrowser
import warnings
import xml.etree.ElementTree as ET
from collections import OrderedDict
from typing import Literal

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch
from Basilisk.architecture import alg_contain, bskLogging, sim_model
from Basilisk.utilities import deprecated, simulationArchTypes
from Basilisk.utilities.pythonVariableLogger import PythonVariableLogger
from Basilisk.utilities.simulationProgessBar import SimulationProgressBar

# Point the path to the module storage area


# define ASCI color codes
processColor = "\u001b[32m"
taskColor = "\u001b[33m"
moduleColor = "\u001b[36m"
endColor = "\u001b[0m"

connectedMessageColor = "#F58518"
inactiveMessageColor = "#8A8F98"
connectionColor = "#5F6B73"


def methodizeCondition(conditionList):
    """Methodize a condition list to a function"""
    if conditionList is None or len(conditionList) == 0:
        return lambda _: False

    funcString = "def EVENT_check_condition(self):\n"
    funcString += "    if("
    for condValue in conditionList:
        funcString += " " + condValue + " and"
    funcString = funcString[:-3] + "):\n"
    funcString += "        return True\n"
    funcString += "    return False"

    local_namespace = {}
    exec(funcString, globals(), local_namespace)
    return local_namespace["EVENT_check_condition"]


def methodizeAction(actionList):
    """Methodize an action list to a function"""
    if actionList is None or len(actionList) == 0:
        return lambda _: None

    funcString = "def EVENT_operate_action(self):\n"
    for actionValue in actionList:
        funcString += "    " + actionValue + "\n"
    funcString += "    return None"

    local_namespace = {}
    exec(funcString, globals(), local_namespace)
    return local_namespace["EVENT_operate_action"]


class EventHandlerClass:
    """
    Class for defining event checking behavior, conditions, and actions.

    Three checking strategies are supported:

        1. **Exact Interval Checking**: (default) The event is checked only when the
           current time is an exact multiple of the ``eventRate``. This behavior is similar
           to how tasks are scheduled in Basilisk. Note that if no task leads to a timestep
           at a checking time, the event will not be checked.
        2. **Elapsed Interval Checking**: The event is checked whenever the ``eventRate``
           has elapsed since the last check. This is enabled by setting ``exactRateMatch``
           to ``False``. This behavior is similar to how Basilisk loggers operate.
        3. **Condition Time Checking**: An alternative to interval-based checking when
           an event should occur at a specific time. This is enabled by setting
           ``conditionTime``, and will lead to the event being triggered at the first
           timestep at or after the specified time.

    When an event is checked, the ``conditionFunction`` is called to determine if the
    event should occur. If the condition returns ``True``, the ``actionFunction`` is executed.
    and the event is deactivated. To continue checking the event, it must be reactivated.
    If the event is marked as ``terminal``, the simulation will be instructed to terminate
    when the event condition occurs.

    Args:
        eventName (str): Name of the event
        eventRate (int): Rate at which the event is checked in nanoseconds
        eventActive (bool): Whether the event is active or not
        terminal (bool): Whether this event should terminate the simulation when it occurs
        conditionFunction (function): Function to check if the event should occur. The
            function should take the simulation object as an argument and return a boolean.
            This is the preferred manner to set conditions as it enables the use of arbitrary
            packages and objects in events and allows for event code to be parsed by IDE tools.
        conditionTime (int): Alternative to conditionFunction, a time in nanoseconds to trigger
            the event. Does not depend on eventRate for checking.
        actionFunction (function): Function to execute when the event occurs. The
            function should take the simulation object as an argument.
            This is the preferred manner to set conditions as it enables the use of arbitrary
            packages and objects in events and allows for event code to be parsed by IDE tools.
        exactRateMatch (bool): If True, the event is checked only when the current time is an
            exact multiple of the eventRate. If False, the event is checked whenever the
            ``eventRate`` has elapsed since the last check.
        conditionList (list): (deprecated) List of conditions to check for the event,
            expressed as strings of code to execute within the class.
        actionList (list): (deprecated) List of actions to perform when the event occurs,
            expressed as strings of code to execute within the class.
    """

    def __init__(
        self,
        eventName,
        eventRate=int(1e9),
        eventActive=False,
        conditionList=None,
        actionList=None,
        conditionFunction=None,
        actionFunction=None,
        conditionTime=None,
        terminal=False,
        exactRateMatch=True,
    ):
        self.eventName = eventName
        self.eventActive = eventActive
        self.eventRate = eventRate
        self.occurCounter = 0
        self.prevCheckTime = None
        self.terminal = terminal
        self.exactRateMatch = exactRateMatch

        self.conditionTime = conditionTime
        if conditionTime is not None:
            if conditionFunction is not None:
                raise ValueError(
                    "Only specify a conditionFunction or a conditionTime, not both"
                )
            conditionFunction = (
                lambda sim: sim.TotalSim.CurrentNanos >= self.conditionTime
            )

        self.conditionFunction = conditionFunction or (lambda _: False)
        self.actionFunction = actionFunction or (lambda _: None)

        if conditionList is not None:
            if conditionFunction is not None:
                raise ValueError(
                    "Only specify a conditionFunction or a conditionList, not both"
                )
            else:
                self.conditionFunction = methodizeCondition(conditionList)

        if actionList is not None:
            if actionFunction is not None:
                raise ValueError(
                    "Only specify an actionFunction or am actionList, not both."
                )
            else:
                self.actionFunction = methodizeAction(actionList)

    def shouldBeChecked(self, currentTime):
        """See if the event should be checked at the current time."""
        if not self.eventActive:
            return False

        if self.conditionTime is not None:
            return currentTime >= self.conditionTime

        if self.exactRateMatch:
            return currentTime % self.eventRate == 0
        else:
            return (
                self.prevCheckTime is None
                or currentTime >= self.prevCheckTime + self.eventRate
            )

    def checkEvent(self, parentSim):
        """Check the condition and execute the action if condition is met."""
        if self.conditionFunction(parentSim):
            self.eventActive = False
            # This event just deactivated; the parent's active-event cache is now
            # stale. Invalidate it before the action runs so that an action which
            # re-activates events rebuilds a correct list on the next query.
            parentSim._invalidateActiveEventCache()
            self.actionFunction(parentSim)
            self.occurCounter += 1
            if self.terminal:
                parentSim.terminate = True
        self.prevCheckTime = parentSim.TotalSim.CurrentNanos

    def nextCheckTime(self, currentTime):
        """Get the earliest upcoming time this event should be checked."""
        if self.conditionTime is not None:
            return self.conditionTime
        if self.exactRateMatch:
            return (currentTime // self.eventRate + 1) * self.eventRate
        else:
            return (
                self.prevCheckTime + self.eventRate
                if self.prevCheckTime is not None
                else currentTime
            )


class StructDocData:
    """Structure data documentation class"""

    class StructElementDef:
        def __init__(self, type, name, argstring, desc=""):
            self.type = type
            self.name = name
            self.argstring = argstring
            self.desc = desc

    def __init__(self, strName):
        self.strName = strName
        self.structPopulated = False
        self.structElements = {}

    def clearItem(self):
        self.structPopulated = False
        self.structElements = {}

    def populateElem(self, xmlSearchPath):
        if self.structPopulated == True:
            return
        xmlFileUse = xmlSearchPath + "/" + self.strName + ".xml"
        try:
            xmlData = ET.parse(xmlFileUse)
        except:
            print("Failed to parse the XML structure for: " + self.strName)
            print("This file does not exist most likely: " + xmlFileUse)
            return
        root = xmlData.getroot()
        validElement = root.find("./compounddef[@id='" + self.strName + "']")
        for newVariable in validElement.findall(".//memberdef[@kind='variable']"):
            typeUse = (
                newVariable.find("type").text
                if newVariable.find("type") is not None
                else None
            )
            nameUse = (
                newVariable.find("name").text
                if newVariable.find("type") is not None
                else None
            )
            argstringUse = (
                newVariable.find("argsstring").text
                if newVariable.find("argsstring") is not None
                else None
            )
            descUse = (
                newVariable.find("./detaileddescription/para").text
                if newVariable.find("./detaileddescription/para") is not None
                else None
            )
            if descUse == None:
                descUse = (
                    newVariable.find("./briefdescription/para").text
                    if newVariable.find("./briefdescription/para") is not None
                    else None
                )
            newElement = StructDocData.StructElementDef(
                typeUse, nameUse, argstringUse, descUse
            )
            self.structElements.update({nameUse: newElement})
            self.structPopulated = True

    def printElem(self):
        print("    " + self.strName + " Structure Elements:")
        for key, value in self.structElements.items():
            outputString = ""
            outputString += value.type + " " + value.name
            outputString += value.argstring if value.argstring is not None else ""
            outputString += ": " + value.desc if value.desc is not None else ""
        print("      " + outputString)


class DataPairClass:
    def __init__(self):
        self.outputMessages = set([])
        self.inputMessages = set([])
        self.name = ""
        self.outputDict = {}


def _hasCallable(obj, name):
    """Return ``True`` if ``obj`` has a callable attribute named ``name``."""
    return callable(getattr(obj, name, None))


def _hasBskMessageShape(obj):
    """Return ``True`` if ``obj`` looks like a Basilisk message wrapper."""
    if not hasattr(obj, "this"):
        return False
    methodNames = [
        "subscribeTo",
        "isSubscribedTo",
        "addSubscriber",
        "getMsgPointers",
        "recorder",
    ]
    return any(_hasCallable(obj, methodName) for methodName in methodNames)


def _isSimpleValue(obj):
    """Return ``True`` if ``obj`` is a scalar-like value that should not be walked."""
    return obj is None or isinstance(obj, (str, bytes, int, float, bool, complex))


def _shouldInspectMessageAttribute(obj, name):
    """Return ``True`` if an attribute may contain a Basilisk message endpoint."""
    if name.startswith("_") or name in ("this", "thisown", "bskLogger"):
        return False
    if not hasattr(obj, "this"):
        return True
    messageContainerNames = {
        "gravBodies",
        "gravField",
    }
    return "Msg" in name or name in messageContainerNames


def _iterCollection(obj):
    """Yield collection items from Python and SWIG containers when possible."""
    if isinstance(obj, dict):
        for key, value in obj.items():
            yield f"[{key}]", value
        return

    if isinstance(obj, (list, tuple, set)):
        for index, value in enumerate(obj):
            yield f"[{index}]", value
        return

    if _isSimpleValue(obj) or _hasBskMessageShape(obj):
        return

    try:
        iterator = iter(obj)
    except TypeError:
        return
    except Exception:
        return

    for index, value in enumerate(iterator):
        yield f"[{index}]", value


def _messageIdentity(message):
    """Return a stable identity key for a SWIG-wrapped message object."""
    if hasattr(message, "this"):
        try:
            pointerAddress = int(message.this)
        except Exception:
            pointerAddress = str(message.this)
        return type(message).__module__, type(message).__name__, pointerAddress
    return type(message).__module__, type(message).__name__, str(id(message))


def _messagePayloadType(message):
    """Return the best available payload type name for a Basilisk message."""
    moduleName = type(message).__module__.split(".")[-1]
    if moduleName.endswith("Payload"):
        return moduleName

    typeName = type(message).__name__
    for suffix in ("Reader", "_C", "Msg"):
        if typeName.endswith(suffix):
            typeName = typeName[: -len(suffix)]
            break
    if typeName.endswith("Payload"):
        return typeName
    return typeName + "Payload"


def _messageIsLinked(message):
    """Return ``True`` if a Basilisk message endpoint reports a link."""
    if _hasCallable(message, "isLinked"):
        try:
            return bool(message.isLinked())
        except Exception:
            pass

    header = getattr(message, "header", None)
    if header is not None and hasattr(header, "isLinked"):
        try:
            return bool(header.isLinked)
        except Exception:
            pass

    return False


def _messageDirection(name, message):
    """Infer whether a message endpoint is an input or output endpoint."""
    nameTail = name.split(".")[-1]
    if "InMsg" in nameTail:
        return "input"
    if "OutMsg" in nameTail:
        return "output"

    typeName = type(message).__name__
    if typeName.endswith("Reader"):
        return "input"
    if _hasCallable(message, "addSubscriber") or _hasCallable(message, "getMsgPointers"):
        return "output"

    return None


def _safeGetAttribute(obj, name):
    """Return an object attribute or ``None`` if SWIG rejects the access."""
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", deprecated.BSKDeprecationWarning)
            return getattr(obj, name)
    except Exception:
        return None


def _walkMessageLeaves(obj, name, depthRemaining, visited):
    """Yield ``(path, message)`` pairs found below ``obj``."""
    if _isSimpleValue(obj):
        return

    if _hasBskMessageShape(obj):
        yield name, obj
        return

    objIdentity = id(obj)
    if objIdentity in visited:
        return
    visited.add(objIdentity)

    if depthRemaining <= 0:
        return

    for collectionName, value in _iterCollection(obj):
        yield from _walkMessageLeaves(
            value,
            name + collectionName,
            depthRemaining - 1,
            visited,
        )

    if not hasattr(obj, "__dict__") and not hasattr(obj, "this"):
        return

    for attrName in dir(obj):
        if not _shouldInspectMessageAttribute(obj, attrName):
            continue
        value = _safeGetAttribute(obj, attrName)
        if value is None:
            continue
        if callable(value) and not _hasBskMessageShape(value):
            continue
        yield from _walkMessageLeaves(
            value,
            attrName if name == "" else name + "." + attrName,
            depthRemaining - 1,
            visited,
        )


def _iterMessageLeaves(obj, name="", maxDepth=4):
    """Yield ``(path, message)`` pairs found below ``obj``."""
    yield from _walkMessageLeaves(obj, name, maxDepth, set())


def _isSubscribedTo(inputMessage, sourceMessage):
    """Return ``True`` if ``inputMessage`` is subscribed to ``sourceMessage``."""
    if not _hasCallable(inputMessage, "isSubscribedTo"):
        return False

    try:
        return bool(inputMessage.isSubscribedTo(sourceMessage))
    except Exception:
        return False


def _shortText(text, maxLength=26):
    """Shorten text so that port labels remain legible in compact figures."""
    if len(text) <= maxLength:
        return text
    return text[: maxLength - 3] + "..."


def _graphvizId(text):
    """Return a DOT-safe identifier from arbitrary graph text."""
    return "".join(character if character.isalnum() else "_" for character in text)


def _graphvizEscape(text):
    """Escape text for Graphviz HTML-like labels."""
    escapedText = html.escape(str(text), quote=True)
    if escapedText.strip() == "":
        return "&#160;"
    return escapedText


def _attachRecorderSource(recorder, sourceMessage):
    """Attach source message metadata to a recorder if the wrapper allows it."""
    try:
        recorder._bskRecordedMessage = sourceMessage
    except Exception:
        pass
    return recorder


def _ensureRecorderSourceHooks():
    """Wrap generated ``recorder()`` methods so recorders remember their source."""
    try:
        from Basilisk.architecture import messaging
    except Exception:
        return

    for attrName in dir(messaging):
        messageClass = getattr(messaging, attrName)
        recorderMethod = getattr(messageClass, "recorder", None)
        if not callable(recorderMethod):
            continue
        if getattr(messageClass, "_bskRecorderSourceHooked", False):
            continue

        def recorderWithSource(self, *args, _recorderMethod=recorderMethod, **kwargs):
            recorder = _recorderMethod(self, *args, **kwargs)
            return _attachRecorderSource(recorder, self)

        try:
            messageClass.recorder = recorderWithSource
            messageClass._bskRecorderSourceHooked = True
        except Exception:
            pass


_ensureRecorderSourceHooks()


class SimBaseClass:
    """Simulation Base Class"""

    def __init__(self):
        _ensureRecorderSourceHooks()
        self.TotalSim = sim_model.SimModel()
        self.TaskList = []
        self.procList = []
        self.StopTime = 0
        self.nextEventTime = 0
        self.terminate = False
        self.eventMap = {}
        self._activeEventCache = None
        self.simBasePath = os.path.dirname(os.path.realpath(__file__)) + "/../"
        self.dataStructIndex = self.simBasePath + "/xml/index.xml"
        self.indexParsed = False
        self.simulationInitialized = False
        self.simulationFinished = False
        self.bskLogger = bskLogging.BSKLogger()
        self.showProgressBar = False
        self.progressBarTargetUpdates = 1000  # [-]
        self.progressBarUpdateInterval = None
        self.allModules = set()

    def SetProgressBar(self, value, targetUpdates=None, updateInterval=None):
        """
        Shows a dynamic progress in the terminal while the simulation is executing.

        Args:
            value (bool): Flag indicating if progress should be displayed.
            targetUpdates (int, optional): Approximate number of progress updates
                to produce over the simulation run.
            updateInterval (int, optional): Fixed progress update interval in
                nanoseconds.
        """
        if targetUpdates is not None and updateInterval is not None:
            raise ValueError("Only specify targetUpdates or updateInterval, not both.")
        if targetUpdates is not None:
            if targetUpdates <= 0:
                raise ValueError("targetUpdates must be positive.")
            self.progressBarTargetUpdates = targetUpdates
            self.progressBarUpdateInterval = None
        if updateInterval is not None:
            if updateInterval <= 0:
                raise ValueError("updateInterval must be positive.")
            self.progressBarUpdateInterval = int(updateInterval)
        self.showProgressBar = value

    def ShowExecutionOrder(self):
        """
        Shows in what order the Basilisk processes, task lists and modules are executed
        """

        for processData in self.TotalSim.processList:
            print(
                f"{processColor}Process Name: {endColor}{processData.processName}, "
                f"{processColor}priority: {endColor}{processData.processPriority}"
            )
            for task in processData.processTasks:
                print(
                    f"{taskColor}Task Name: {endColor}{task.TaskPtr.TaskName}, "
                    f"{taskColor}priority: {endColor}{task.taskPriority}, "
                    f"{taskColor}TaskPeriod: {endColor}{task.TaskPtr.TaskPeriod / 1.0e9}s"
                )
                for module in task.TaskPtr.TaskModels:
                    print(
                        f"{moduleColor}ModuleTag: {endColor}{module.ModelPtr.ModelTag}, "
                        f"{moduleColor}priority: {endColor}{module.CurrentModelPriority}"
                    )
            print()

    def ShowExecutionFigure(self, show_plots=False):
        """
        Shows in what order the Basilisk processes, task lists and modules are executed
        """
        processList = OrderedDict()
        for processData in self.TotalSim.processList:
            taskList = OrderedDict()
            for task in processData.processTasks:
                moduleList = []
                for module in task.TaskPtr.TaskModels:
                    moduleList.append(
                        f"{module.ModelPtr.ModelTag} ({module.CurrentModelPriority})"
                    )
                taskList[
                    f"{task.TaskPtr.TaskName} ({task.taskPriority}, {task.TaskPtr.TaskPeriod / 1.0e9}s)"
                ] = moduleList
            processList[
                processData.processName + " (" + str(processData.processPriority) + ")"
            ] = taskList

        fig = plt.figure()
        plt.rcParams.update({"font.size": 8})
        plt.axis("off")

        processNo = 0
        processWidth = 6
        lineHeight = 0.5
        textBuffer = lineHeight * 0.75
        textIndent = lineHeight * 0.25
        processGap = 0.5
        for process in processList:
            # Draw process box + priority
            rectangle = plt.Rectangle(
                ((processWidth + processGap) * processNo, 0),
                processWidth,
                -lineHeight,
                ec="g",
                fc="g",
            )
            plt.gca().add_patch(rectangle)
            plt.text(
                (processWidth + processGap) * processNo + textIndent,
                -textBuffer,
                process,
                color="w",
            )

            taskNo = 0
            currentLine = -lineHeight - textIndent
            for task in processList[process]:
                # Draw task box + priority + task rate
                rectangle = plt.Rectangle(
                    ((processWidth + processGap) * processNo + textIndent, currentLine),
                    processWidth - 2 * textIndent,
                    -(1 + len(processList[process][task])) * (lineHeight + textIndent),
                    ec="y",
                    fc=(1, 1, 1, 0),
                )
                plt.gca().add_patch(rectangle)
                rectangle = plt.Rectangle(
                    ((processWidth + processGap) * processNo + textIndent, currentLine),
                    processWidth - 2 * textIndent,
                    -lineHeight,
                    ec="y",
                    fc="y",
                )
                plt.gca().add_patch(rectangle)
                plt.text(
                    (processWidth + processGap) * processNo + 2 * textIndent,
                    currentLine - textBuffer,
                    task,
                    color="black",
                )

                for module in processList[process][task]:
                    # Draw modules + priority
                    currentLine -= lineHeight + textIndent
                    rectangle = plt.Rectangle(
                        (
                            (processWidth + processGap) * processNo + 2 * textIndent,
                            currentLine,
                        ),
                        processWidth - 4 * textIndent,
                        -lineHeight,
                        ec="c",
                        fc=(1, 1, 1, 0),
                    )
                    plt.gca().add_patch(rectangle)
                    plt.text(
                        (processWidth + processGap) * processNo + 3 * textIndent,
                        currentLine - textBuffer,
                        module,
                        color="black",
                    )

                taskNo += 1
                currentLine -= lineHeight + 2 * textIndent

            rectangle = plt.Rectangle(
                ((processWidth + processGap) * processNo, 0),
                processWidth,
                currentLine,
                ec="g",
                fc=(1, 1, 1, 0),
            )
            plt.gca().add_patch(rectangle)
            processNo += 1

        plt.axis("scaled")

        if show_plots:
            plt.show()

        return fig

    def _normalizeExtraMessages(self, extraMessages):
        """Normalize stand-alone messages into an ordered name/object dictionary."""
        normalizedMessages = OrderedDict()
        if extraMessages is None:
            return normalizedMessages

        messageCollections = [extraMessages]
        if not isinstance(extraMessages, dict):
            messageCollections = extraMessages

        for messageCollection in messageCollections:
            if not isinstance(messageCollection, dict):
                raise TypeError("extraMessages must be a dictionary or a list of dictionaries.")
            for messageName, message in messageCollection.items():
                normalizedMessages[str(messageName)] = message
        return normalizedMessages

    def _getOrderedTaskModels(self, task):
        """Return Python-side task models in simulation execution order."""
        priorities = getattr(task, "TaskModelPriorities", None)
        if priorities is None or len(priorities) != len(task.TaskModels):
            return []
        indexedModels = list(enumerate(zip(task.TaskModels, priorities)))
        indexedModels.sort(key=lambda item: (-item[1][1], item[0]))
        return [model for _, (model, _) in indexedModels]

    def _getTaskModelCandidates(self, taskName, modelTag, modelPointer, taskModelIndex):
        """Return Python-side objects that may own message attributes for a model."""
        candidates = [modelPointer]
        for task in self.TaskList:
            if task.Name != taskName:
                continue
            orderedTaskModels = self._getOrderedTaskModels(task)
            if taskModelIndex < len(orderedTaskModels):
                taskModel = orderedTaskModels[taskModelIndex]
                if taskModel is not modelPointer:
                    candidates.append(taskModel)
                return candidates

            for taskModel in task.TaskModels:
                if taskModel is modelPointer:
                    continue
                if getattr(taskModel, "ModelTag", "") == modelTag:
                    candidates.append(taskModel)
        return candidates

    def _candidateListHasRecorder(self, candidates):
        """Return ``True`` if any model candidate is a message recorder."""
        for candidate in candidates:
            if getattr(candidate, "_bskRecordedMessage", None) is not None:
                return True
            modelTag = getattr(candidate, "ModelTag", "")
            if isinstance(modelTag, str) and modelTag.startswith("Rec:"):
                return True
        return False

    def _collectModuleMessageEndpoints(self, moduleId, moduleRecord, candidates):
        """Collect input and output message endpoints from module candidates."""
        seenEndpoints = set()
        for candidate in candidates:
            recordedMessage = getattr(candidate, "_bskRecordedMessage", None)
            if recordedMessage is not None:
                endpoint = {
                    "id": f"{moduleId}:input:{len(moduleRecord['inputs'])}",
                    "ownerId": moduleId,
                    "ownerType": "module",
                    "moduleTag": moduleRecord["tag"],
                    "name": "recordedMsg",
                    "direction": "input",
                    "payloadType": _messagePayloadType(recordedMessage),
                    "message": candidate,
                    "recordedMessage": recordedMessage,
                    "isLinked": True,
                }
                moduleRecord["inputs"].append(endpoint)

            for messageName, message in _iterMessageLeaves(candidate):
                direction = _messageDirection(messageName, message)
                if direction is None:
                    continue
                endpointKey = (direction, _messageIdentity(message), messageName)
                if endpointKey in seenEndpoints:
                    continue
                seenEndpoints.add(endpointKey)

                endpointId = f"{moduleId}:{direction}:{len(moduleRecord[direction + 's'])}"
                endpoint = {
                    "id": endpointId,
                    "ownerId": moduleId,
                    "ownerType": "module",
                    "moduleTag": moduleRecord["tag"],
                    "name": messageName,
                    "direction": direction,
                    "payloadType": _messagePayloadType(message),
                    "message": message,
                    "isLinked": _messageIsLinked(message),
                }
                moduleRecord[direction + "s"].append(endpoint)

    def GetMessageConnectionGraph(
        self,
        extraMessages=None,
        includeUnlinked=True,
        includeRecorders=True,
    ):
        """
        Extract Basilisk message connections from the configured simulation.

        Args:
            extraMessages (dict or list[dict], optional): Stand-alone messages to
                include as possible source messages.  The dictionary keys are used
                as labels in the returned graph and any generated figure.
            includeUnlinked (bool): If ``True``, include unlinked input message
                endpoints in the returned graph.
            includeRecorders (bool): If ``True``, include message recorder modules
                in the returned graph.

        Returns:
            dict: A graph dictionary containing module records, stand-alone
            message records, endpoint records, connection edge records, unlinked
            inputs, and linked inputs whose source was not found.
        """
        graph = {
            "modules": [],
            "standaloneMessages": [],
            "inputs": [],
            "outputs": [],
            "edges": [],
            "unlinkedInputs": [],
            "unresolvedInputs": [],
        }

        moduleIndex = 0
        for processData in self.TotalSim.processList:
            for task in processData.processTasks:
                taskPeriod = task.TaskPtr.TaskPeriod / 1.0e9  # [s]
                for taskModelIndex, module in enumerate(task.TaskPtr.TaskModels):
                    modelPointer = module.ModelPtr
                    modelTag = modelPointer.ModelTag
                    moduleId = f"module:{moduleIndex}"
                    moduleRecord = {
                        "id": moduleId,
                        "tag": modelTag,
                        "processName": processData.processName,
                        "processPriority": processData.processPriority,
                        "taskName": task.TaskPtr.TaskName,
                        "taskPriority": task.taskPriority,
                        "taskPeriod": taskPeriod,
                        "modelPriority": module.CurrentModelPriority,
                        "executionIndex": moduleIndex,
                        "inputs": [],
                        "outputs": [],
                    }
                    candidates = self._getTaskModelCandidates(
                        task.TaskPtr.TaskName,
                        modelTag,
                        modelPointer,
                        taskModelIndex,
                    )
                    if not includeRecorders and self._candidateListHasRecorder(candidates):
                        continue
                    self._collectModuleMessageEndpoints(moduleId, moduleRecord, candidates)
                    graph["modules"].append(moduleRecord)
                    graph["inputs"].extend(moduleRecord["inputs"])
                    graph["outputs"].extend(moduleRecord["outputs"])
                    moduleIndex += 1

        for messageIndex, (messageName, messageObject) in enumerate(
            self._normalizeExtraMessages(extraMessages).items()
        ):
            for sourceName, sourceMessage in _iterMessageLeaves(messageObject, messageName):
                sourceId = f"standalone:{messageIndex}:{len(graph['standaloneMessages'])}"
                endpoint = {
                    "id": sourceId + ":output:0",
                    "ownerId": sourceId,
                    "ownerType": "standalone",
                    "moduleTag": "",
                    "name": sourceName,
                    "direction": "output",
                    "payloadType": _messagePayloadType(sourceMessage),
                    "message": sourceMessage,
                    "isLinked": _messageIsLinked(sourceMessage),
                }
                standaloneRecord = {
                    "id": sourceId,
                    "name": sourceName,
                    "outputs": [endpoint],
                }
                graph["standaloneMessages"].append(standaloneRecord)
                graph["outputs"].append(endpoint)

        seenEdges = set()

        def addEdge(sourceEndpoint, targetEndpoint):
            """Add a graph edge unless it was already recorded."""
            edgeKey = (sourceEndpoint["id"], targetEndpoint["id"])
            if edgeKey in seenEdges:
                return
            seenEdges.add(edgeKey)
            graph["edges"].append(
                {
                    "source": sourceEndpoint["id"],
                    "target": targetEndpoint["id"],
                    "sourceName": sourceEndpoint["name"],
                    "targetName": targetEndpoint["name"],
                    "sourceOwner": sourceEndpoint["ownerId"],
                    "targetOwner": targetEndpoint["ownerId"],
                    "payloadType": targetEndpoint["payloadType"],
                    "sourceType": sourceEndpoint["ownerType"],
                }
            )

        for inputEndpoint in graph["inputs"]:
            recordedMessage = inputEndpoint.get("recordedMessage", None)
            if recordedMessage is not None:
                for sourceEndpoint in graph["inputs"]:
                    if inputEndpoint["id"] == sourceEndpoint["id"]:
                        continue
                    if _messageIdentity(recordedMessage) == _messageIdentity(
                        sourceEndpoint["message"]
                    ):
                        addEdge(sourceEndpoint, inputEndpoint)
                        break
                else:
                    for outputEndpoint in graph["outputs"]:
                        if _messageIdentity(recordedMessage) == _messageIdentity(
                            outputEndpoint["message"]
                        ):
                            addEdge(outputEndpoint, inputEndpoint)
                            break
                continue

            for outputEndpoint in graph["outputs"]:
                if inputEndpoint["message"] is outputEndpoint["message"]:
                    continue
                if not _isSubscribedTo(inputEndpoint["message"], outputEndpoint["message"]):
                    continue
                addEdge(outputEndpoint, inputEndpoint)

        connectedInputIds = {edge["target"] for edge in graph["edges"]}
        for inputEndpoint in graph["inputs"]:
            if inputEndpoint["id"] in connectedInputIds:
                continue
            if inputEndpoint["isLinked"]:
                graph["unresolvedInputs"].append(inputEndpoint)
            elif includeUnlinked:
                graph["unlinkedInputs"].append(inputEndpoint)

        if not includeUnlinked:
            hiddenInputIds = {
                endpoint["id"]
                for endpoint in graph["inputs"]
                if endpoint["id"] not in connectedInputIds and not endpoint["isLinked"]
            }
            graph["inputs"] = [
                endpoint for endpoint in graph["inputs"] if endpoint["id"] not in hiddenInputIds
            ]
            for moduleRecord in graph["modules"]:
                moduleRecord["inputs"] = [
                    endpoint
                    for endpoint in moduleRecord["inputs"]
                    if endpoint["id"] not in hiddenInputIds
                ]

        return graph

    def ShowMessageConnectionFigure(
        self,
        show_plots=False,
        extraMessages=None,
        includeUnlinked=True,
        includeRecorders=True,
        renderer="matplotlib",
        fileName=None,
        graphvizFormat="svg",
        graphvizLayout="vertical",
    ):
        """
        Show how Basilisk messages are connected between simulation modules.

        Args:
            show_plots (bool): If ``True``, display the Matplotlib figure or
                open the rendered Graphviz output file.
            extraMessages (dict or list[dict], optional): Stand-alone messages to
                draw as source messages.  The dictionary keys are used as labels.
            includeUnlinked (bool): If ``True``, draw unlinked input message
                endpoints.
            includeRecorders (bool): If ``True``, draw message recorder modules.
            renderer (str): Rendering backend.  Use ``"matplotlib"`` to return a
                Matplotlib figure, or ``"graphviz"`` to render through the
                Graphviz ``dot`` executable.
            fileName (str, optional): Output path for the Graphviz renderer.  If
                no extension is provided, ``graphvizFormat`` is appended.
            graphvizFormat (str): Graphviz output format, such as ``"svg"``,
                ``"png"``, ``"pdf"``, or ``"dot"``.
            graphvizLayout (str): Graphviz module layout direction.  Use
                ``"vertical"`` for a top-to-bottom layout or ``"horizontal"``
                for a left-to-right layout.

        Returns:
            matplotlib.figure.Figure or str: Matplotlib figure for the
            ``"matplotlib"`` renderer, or the Graphviz output file path for the
            ``"graphviz"`` renderer.
        """
        graph = self.GetMessageConnectionGraph(
            extraMessages=extraMessages,
            includeUnlinked=includeUnlinked,
            includeRecorders=includeRecorders,
        )

        if renderer == "graphviz":
            return self._renderMessageConnectionGraphviz(
                graph,
                fileName=fileName,
                graphvizFormat=graphvizFormat,
                graphvizLayout=graphvizLayout,
                show_plots=show_plots,
            )
        if renderer != "matplotlib":
            raise ValueError("renderer must be either 'matplotlib' or 'graphviz'.")

        fig = self._drawMessageConnectionFigure(graph)

        if show_plots:
            plt.show()
        else:
            plt.close(fig)

        return fig

    def GetMessageConnectionDot(
        self,
        extraMessages=None,
        includeUnlinked=True,
        includeRecorders=True,
        graphvizLayout="vertical",
    ):
        """
        Return a Graphviz DOT description of the simulation message graph.

        Args:
            extraMessages (dict or list[dict], optional): Stand-alone messages to
                include as possible source messages.
            includeUnlinked (bool): If ``True``, include unlinked input message
                endpoints.
            includeRecorders (bool): If ``True``, include message recorder modules.
            graphvizLayout (str): Graphviz module layout direction.  Use
                ``"vertical"`` for a top-to-bottom layout or ``"horizontal"``
                for a left-to-right layout.

        Returns:
            str: Graphviz DOT source text.
        """
        graph = self.GetMessageConnectionGraph(
            extraMessages=extraMessages,
            includeUnlinked=includeUnlinked,
            includeRecorders=includeRecorders,
        )
        return self._messageConnectionGraphToDot(
            graph,
            graphvizLayout=graphvizLayout,
        )

    def _renderMessageConnectionGraphviz(
        self,
        graph,
        fileName=None,
        graphvizFormat="svg",
        graphvizLayout="vertical",
        show_plots=False,
    ):
        """Render a message connection graph through the Graphviz executable."""
        outputFormat = graphvizFormat.lower().lstrip(".")
        if outputFormat == "":
            outputFormat = "svg"

        outputFileName, dotFileName, outputFormat = self._resolveGraphvizFileNames(
            fileName,
            outputFormat,
        )
        dotText = self._messageConnectionGraphToDot(
            graph,
            graphvizLayout=graphvizLayout,
        )

        dotDirectory = os.path.dirname(dotFileName)
        if dotDirectory:
            os.makedirs(dotDirectory, exist_ok=True)
        with open(dotFileName, "w") as dotFile:
            dotFile.write(dotText)

        if outputFormat == "dot":
            outputFileName = dotFileName
        else:
            dotExecutable = shutil.which("dot")
            if dotExecutable is None:
                raise RuntimeError(
                    "Graphviz renderer requires the 'dot' executable to be available."
                )
            outputDirectory = os.path.dirname(outputFileName)
            if outputDirectory:
                os.makedirs(outputDirectory, exist_ok=True)
            try:
                subprocess.run(
                    [
                        dotExecutable,
                        "-T" + outputFormat,
                        dotFileName,
                        "-o",
                        outputFileName,
                    ],
                    check=True,
                    capture_output=True,
                    text=True,
                )
            except subprocess.CalledProcessError as error:
                raise RuntimeError(
                    "Graphviz 'dot' failed to render the message connection graph: "
                    + error.stderr
                ) from error

        if show_plots:
            webbrowser.open("file://" + os.path.abspath(outputFileName))

        return outputFileName

    def _resolveGraphvizFileNames(self, fileName, graphvizFormat):
        """Resolve Graphviz DOT and rendered output paths."""
        if fileName is None:
            outputDirectory = tempfile.mkdtemp(prefix="bskMessageConnections_")
            outputFileName = os.path.join(
                outputDirectory,
                "messageConnections." + graphvizFormat,
            )
            dotFileName = os.path.join(outputDirectory, "messageConnections.dot")
            return outputFileName, dotFileName, graphvizFormat

        fileRoot, fileExtension = os.path.splitext(fileName)
        if fileExtension:
            extensionFormat = fileExtension[1:].lower()
            if extensionFormat == "dot":
                dotFileName = fileName
                outputFileName = fileRoot + "." + graphvizFormat
            else:
                graphvizFormat = extensionFormat
                outputFileName = fileName
                dotFileName = fileRoot + ".dot"
        else:
            outputFileName = fileName + "." + graphvizFormat
            dotFileName = fileName + ".dot"

        return outputFileName, dotFileName, graphvizFormat

    def _resolveGraphvizLayout(self, graphvizLayout):
        """Resolve the requested Graphviz layout into a rank direction."""
        layoutName = str(graphvizLayout).lower()
        if layoutName in ["vertical", "tb", "top-bottom", "top_to_bottom"]:
            return "vertical", "TB"
        if layoutName in ["horizontal", "lr", "left-right", "left_to_right"]:
            return "horizontal", "LR"
        raise ValueError(
            "graphvizLayout must be either 'vertical' or 'horizontal'."
        )

    def _messageConnectionGraphToDot(self, graph, graphvizLayout="vertical"):
        """Convert a message connection graph dictionary to Graphviz DOT text."""
        graphvizLayout, rankDirection = self._resolveGraphvizLayout(graphvizLayout)
        if graphvizLayout == "vertical":
            nodeSeparation = "0.28"
            rankSeparation = "0.45"
            sourceCompass = "s"
            targetCompass = "n"
        else:
            nodeSeparation = "0.35"
            rankSeparation = "0.65"
            sourceCompass = "e"
            targetCompass = "w"
        endpointNodePorts = {}
        connectedEndpointIds = {
            endpointId
            for edge in graph["edges"]
            for endpointId in (edge["source"], edge["target"])
        }

        lines = [
            "digraph BSKMessageConnections {",
            f'  graph [rankdir="{rankDirection}", bgcolor="transparent", pad="0.15",',
            f'         nodesep="{nodeSeparation}", ranksep="{rankSeparation}", splines="spline"];',
            '  node [shape="plain", fontname="Helvetica"];',
            '  edge [fontname="Helvetica", fontsize="9", arrowsize="0.7",',
            f'        color="{connectionColor}"];',
        ]

        for standaloneRecord in graph["standaloneMessages"]:
            nodeId = _graphvizId(standaloneRecord["id"])
            outputEndpoint = standaloneRecord["outputs"][0]
            outputPort = _graphvizId(outputEndpoint["id"])
            endpointNodePorts[outputEndpoint["id"]] = (nodeId, outputPort)
            connected = outputEndpoint["id"] in connectedEndpointIds
            lines.extend(
                self._makeGraphvizStandaloneNode(
                    nodeId,
                    standaloneRecord,
                    outputEndpoint,
                    outputPort,
                    connected,
                )
            )

        for moduleRecord in graph["modules"]:
            nodeId = _graphvizId(moduleRecord["id"])
            for endpoint in moduleRecord["inputs"] + moduleRecord["outputs"]:
                endpointNodePorts[endpoint["id"]] = (nodeId, _graphvizId(endpoint["id"]))
            lines.extend(
                self._makeGraphvizModuleNode(
                    nodeId,
                    moduleRecord,
                    connectedEndpointIds,
                )
            )

        for edge in graph["edges"]:
            if edge["source"] not in endpointNodePorts or edge["target"] not in endpointNodePorts:
                continue
            sourceNode, sourcePort = endpointNodePorts[edge["source"]]
            targetNode, targetPort = endpointNodePorts[edge["target"]]
            style = "dashed" if edge["sourceType"] == "standalone" else "solid"
            lines.append(
                f'  "{sourceNode}":"{sourcePort}":{sourceCompass} -> '
                f'"{targetNode}":"{targetPort}":{targetCompass} [style="{style}"];'
            )

        for index in range(len(graph["modules"]) - 1):
            sourceNode = _graphvizId(graph["modules"][index]["id"])
            targetNode = _graphvizId(graph["modules"][index + 1]["id"])
            lines.append(
                f'  "{sourceNode}" -> "{targetNode}" [style="invis", weight="8"];'
            )

        lines.extend(self._makeGraphvizLegendNode(graphvizLayout))
        if graphvizLayout == "vertical" and graph["modules"]:
            lastNode = _graphvizId(graph["modules"][-1]["id"])
            lines.append(
                f'  "{lastNode}" -> "legend" [style="invis", weight="1"];'
            )
        lines.append("}")
        return "\n".join(lines) + "\n"

    def _makeGraphvizStandaloneNode(
        self,
        nodeId,
        standaloneRecord,
        outputEndpoint,
        outputPort,
        connected,
    ):
        """Create DOT lines for a stand-alone message node."""
        color = connectedMessageColor if connected else inactiveMessageColor
        fontColor = "white" if connected else "#222222"
        return [
            f'  "{nodeId}" [label=<',
            '    <TABLE BORDER="1" CELLBORDER="0" CELLSPACING="0" CELLPADDING="5"',
            f'           COLOR="{color}" BGCOLOR="#FFFFFF">',
            "      <TR>",
            f'        <TD PORT="{outputPort}" BGCOLOR="{color}">'
            f'<FONT POINT-SIZE="9" COLOR="{fontColor}"><B>'
            f'{_graphvizEscape(_shortText(standaloneRecord["name"], 28))}'
            "</B></FONT></TD>",
            "      </TR>",
            "      <TR>",
            '        <TD><FONT POINT-SIZE="8" COLOR="#555555">'
            f'{_graphvizEscape(_shortText(outputEndpoint["payloadType"], 34))}'
            "</FONT></TD>",
            "      </TR>",
            "    </TABLE>",
            "  >];",
        ]

    def _makeGraphvizModuleNode(self, nodeId, moduleRecord, connectedEndpointIds):
        """Create DOT lines for a module node with input and output ports."""
        lines = [
            f'  "{nodeId}" [label=<',
            '    <TABLE BORDER="1" CELLBORDER="0" CELLSPACING="0" CELLPADDING="4"',
            '           COLOR="#2F3B45" BGCOLOR="#F8FAFF">',
            "      <TR>",
            '        <TD COLSPAN="3"><FONT POINT-SIZE="10"><B>'
            f'{_graphvizEscape(_shortText(moduleRecord["tag"], 30))}'
            "</B></FONT></TD>",
            "      </TR>",
            "      <TR>",
            '        <TD COLSPAN="3"><FONT POINT-SIZE="8" COLOR="#555555">'
            f'{_graphvizEscape(_shortText(moduleRecord["processName"] + " / " + moduleRecord["taskName"], 42))}'
            "</FONT></TD>",
            "      </TR>",
        ]

        rowCount = max(len(moduleRecord["inputs"]), len(moduleRecord["outputs"]), 1)
        for index in range(rowCount):
            inputEndpoint = (
                moduleRecord["inputs"][index]
                if index < len(moduleRecord["inputs"])
                else None
            )
            outputEndpoint = (
                moduleRecord["outputs"][index]
                if index < len(moduleRecord["outputs"])
                else None
            )
            inputCell = self._makeGraphvizPortCell(
                inputEndpoint,
                connectedEndpointIds,
                "LEFT",
            )
            outputCell = self._makeGraphvizPortCell(
                outputEndpoint,
                connectedEndpointIds,
                "RIGHT",
            )
            lines.extend(
                [
                    "      <TR>",
                    "        " + inputCell,
                    '        <TD WIDTH="12"></TD>',
                    "        " + outputCell,
                    "      </TR>",
                ]
            )

        lines.extend(
            [
                "    </TABLE>",
                "  >];",
            ]
        )
        return lines

    def _makeGraphvizPortCell(self, endpoint, connectedEndpointIds, align):
        """Create one Graphviz HTML table cell for an input or output port."""
        if endpoint is None:
            return '<TD WIDTH="80"></TD>'

        color = (
            connectedMessageColor
            if endpoint["id"] in connectedEndpointIds
            else inactiveMessageColor
        )
        fontColor = "white" if endpoint["id"] in connectedEndpointIds else "#222222"
        portName = _graphvizId(endpoint["id"])
        label = _shortText(endpoint["name"].split(".")[-1], 24)
        return (
            f'<TD PORT="{portName}" ALIGN="{align}" BGCOLOR="{color}">'
            f'<FONT POINT-SIZE="8" COLOR="{fontColor}">'
            f'{_graphvizEscape(label)}</FONT></TD>'
        )

    def _makeGraphvizLegendNode(self, graphvizLayout):
        """Create DOT lines for the Graphviz legend node."""
        if graphvizLayout == "vertical":
            return [
                '  "legend" [label=<',
                '    <TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="4">',
                "      <TR>",
                f'        <TD BGCOLOR="{inactiveMessageColor}"><FONT POINT-SIZE="8" COLOR="white">message</FONT></TD>',
                f'        <TD BGCOLOR="{connectedMessageColor}"><FONT POINT-SIZE="8" COLOR="white">connected</FONT></TD>',
                "      </TR>",
                "      <TR>",
                '        <TD><FONT POINT-SIZE="8">solid: module link</FONT></TD>',
                '        <TD><FONT POINT-SIZE="8">dashed: extraMessages</FONT></TD>',
                "      </TR>",
                "    </TABLE>",
                "  >];",
            ]
        return [
            '  "legend" [label=<',
            '    <TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="4">',
            "      <TR>",
            f'        <TD BGCOLOR="{inactiveMessageColor}"><FONT POINT-SIZE="8" COLOR="white">message</FONT></TD>',
            f'        <TD BGCOLOR="{connectedMessageColor}"><FONT POINT-SIZE="8" COLOR="white">connected</FONT></TD>',
            '        <TD><FONT POINT-SIZE="8">solid: module link</FONT></TD>',
            '        <TD><FONT POINT-SIZE="8">dashed: extraMessages</FONT></TD>',
            "      </TR>",
            "    </TABLE>",
            "  >];",
        ]

    def _drawMessageConnectionFigure(self, graph):
        """Draw a module message connection figure from a graph dictionary."""
        moduleCount = max(len(graph["modules"]), 1)
        maxModulesPerRow = min(moduleCount, 4)
        moduleWidth = 2.7
        moduleGap = 0.95
        moduleStep = moduleWidth + moduleGap
        maxPortCount = 1
        for moduleRecord in graph["modules"]:
            maxPortCount = max(
                maxPortCount,
                len(moduleRecord["inputs"]),
                len(moduleRecord["outputs"]),
            )
        moduleHeight = max(1.55, 0.3 * (maxPortCount + 3))
        rowStep = moduleHeight + 1.7
        moduleRows = int(math.ceil(moduleCount / maxModulesPerRow))
        standaloneRows = int(math.ceil(len(graph["standaloneMessages"]) / maxModulesPerRow))
        standaloneBandHeight = 1.35 * standaloneRows

        figWidth = max(6.0, maxModulesPerRow * 2.0)
        figHeight = max(3.2, moduleRows * 1.5 + standaloneBandHeight * 0.75 + 0.9)
        fig = plt.figure(figsize=(figWidth, figHeight))
        ax = fig.add_subplot(111)
        ax.axis("off")

        moduleLayout = {}
        endpointCoordinates = {}
        for moduleRecord in graph["modules"]:
            row = moduleRecord["executionIndex"] // maxModulesPerRow
            column = moduleRecord["executionIndex"] % maxModulesPerRow
            xPosition = column * moduleStep
            yPosition = -row * rowStep
            moduleLayout[moduleRecord["id"]] = (xPosition, yPosition)

            self._assignModulePortCoordinates(
                moduleRecord,
                xPosition,
                yPosition,
                moduleWidth,
                moduleHeight,
                endpointCoordinates,
            )

        standaloneLayout = {}
        standaloneTop = moduleHeight * 0.5 + standaloneBandHeight
        for index, standaloneRecord in enumerate(graph["standaloneMessages"]):
            row = index // maxModulesPerRow
            column = index % maxModulesPerRow
            xPosition = column * moduleStep
            yPosition = standaloneTop - 1.35 * row
            standaloneLayout[standaloneRecord["id"]] = (xPosition, yPosition)
            endpointCoordinates[standaloneRecord["outputs"][0]["id"]] = (
                xPosition + moduleWidth,
                yPosition,
            )

        connectedEndpointIds = {
            endpointId
            for edge in graph["edges"]
            for endpointId in (edge["source"], edge["target"])
        }
        unresolvedInputIds = {endpoint["id"] for endpoint in graph["unresolvedInputs"]}
        unlinkedInputIds = {endpoint["id"] for endpoint in graph["unlinkedInputs"]}

        self._drawMessageEdges(ax, graph["edges"], endpointCoordinates)
        self._drawStandaloneMessageNodes(
            ax,
            graph["standaloneMessages"],
            standaloneLayout,
            moduleWidth,
            connectedEndpointIds,
        )
        self._drawModuleMessageNodes(
            ax,
            graph["modules"],
            moduleLayout,
            moduleWidth,
            moduleHeight,
            connectedEndpointIds,
            unresolvedInputIds,
            unlinkedInputIds,
        )
        self._drawMessageFigureLegend(ax, moduleLayout, moduleHeight, rowStep)

        xLimit = maxModulesPerRow * moduleStep - moduleGap + 0.5
        yTop = standaloneTop + 1.1 if graph["standaloneMessages"] else moduleHeight
        yBottom = -max(moduleRows - 1, 0) * rowStep - moduleHeight - 0.9
        ax.set_xlim(-0.8, xLimit)
        ax.set_ylim(yBottom, yTop)
        fig.tight_layout(pad=0.2)

        return fig

    def _assignModulePortCoordinates(
        self,
        moduleRecord,
        xPosition,
        yPosition,
        moduleWidth,
        moduleHeight,
        endpointCoordinates,
    ):
        """Assign input and output port coordinates for one module node."""
        headerSpace = 0.65
        usableHeight = moduleHeight - headerSpace
        for direction, xOffset in (("inputs", 0.0), ("outputs", moduleWidth)):
            endpoints = moduleRecord[direction]
            if len(endpoints) == 0:
                continue
            portSpacing = usableHeight / (len(endpoints) + 1)
            firstPortY = yPosition + moduleHeight * 0.5 - headerSpace
            for index, endpoint in enumerate(endpoints):
                endpointCoordinates[endpoint["id"]] = (
                    xPosition + xOffset,
                    firstPortY - portSpacing * (index + 1),
                )

    def _drawMessageEdges(self, ax, edges, endpointCoordinates):
        """Draw message connection arrows."""
        for edgeIndex, edge in enumerate(edges):
            if edge["source"] not in endpointCoordinates or edge["target"] not in endpointCoordinates:
                continue
            sourcePoint = endpointCoordinates[edge["source"]]
            targetPoint = endpointCoordinates[edge["target"]]
            directionSign = 1 if sourcePoint[1] >= targetPoint[1] else -1
            curve = 0.12 * directionSign * (1 + edgeIndex % 3)
            lineStyle = "--" if edge["sourceType"] == "standalone" else "-"
            arrow = FancyArrowPatch(
                sourcePoint,
                targetPoint,
                arrowstyle="-|>",
                mutation_scale=9,
                linewidth=1.15,
                linestyle=lineStyle,
                color=connectionColor,
                alpha=0.72,
                connectionstyle=f"arc3,rad={curve}",
                zorder=1,
            )
            ax.add_patch(arrow)

    def _drawStandaloneMessageNodes(
        self,
        ax,
        standaloneRecords,
        standaloneLayout,
        moduleWidth,
        connectedEndpointIds,
    ):
        """Draw stand-alone message source nodes."""
        nodeHeight = 0.72
        for standaloneRecord in standaloneRecords:
            xPosition, yPosition = standaloneLayout[standaloneRecord["id"]]
            outputEndpoint = standaloneRecord["outputs"][0]
            color = (
                connectedMessageColor
                if outputEndpoint["id"] in connectedEndpointIds
                else inactiveMessageColor
            )
            rectangle = plt.Rectangle(
                (xPosition, yPosition - nodeHeight * 0.5),
                moduleWidth,
                nodeHeight,
                ec=color,
                fc=(1, 1, 1, 0.95),
                linewidth=1.2,
                zorder=3,
            )
            ax.add_patch(rectangle)
            ax.scatter(
                [xPosition + moduleWidth],
                [yPosition],
                s=55,
                color=color,
                edgecolors="white",
                linewidths=0.8,
                zorder=4,
            )
            ax.text(
                xPosition + 0.14,
                yPosition + 0.12,
                _shortText(standaloneRecord["name"]),
                fontsize=7,
                fontweight="bold",
                va="center",
                zorder=4,
            )
            ax.text(
                xPosition + 0.14,
                yPosition - 0.16,
                _shortText(outputEndpoint["payloadType"], 32),
                fontsize=6.5,
                color="#555555",
                va="center",
                zorder=4,
            )

    def _drawModuleMessageNodes(
        self,
        ax,
        moduleRecords,
        moduleLayout,
        moduleWidth,
        moduleHeight,
        connectedEndpointIds,
        unresolvedInputIds,
        unlinkedInputIds,
    ):
        """Draw module boxes with colored input and output message ports."""
        for moduleRecord in moduleRecords:
            xPosition, yPosition = moduleLayout[moduleRecord["id"]]
            rectangle = plt.Rectangle(
                (xPosition, yPosition - moduleHeight * 0.5),
                moduleWidth,
                moduleHeight,
                ec="#2F3B45",
                fc=(0.98, 0.99, 1.0, 0.7),
                linewidth=1.2,
                zorder=3,
            )
            ax.add_patch(rectangle)
            ax.text(
                xPosition + moduleWidth * 0.5,
                yPosition + moduleHeight * 0.5 - 0.22,
                _shortText(moduleRecord["tag"], 24),
                fontsize=7.0,
                fontweight="bold",
                ha="center",
                va="center",
                zorder=4,
            )
            ax.text(
                xPosition + moduleWidth * 0.5,
                yPosition + moduleHeight * 0.5 - 0.48,
                _shortText(
                    f"{moduleRecord['processName']} / {moduleRecord['taskName']}",
                    36,
                ),
                fontsize=6.3,
                color="#555555",
                ha="center",
                va="center",
                zorder=4,
            )
            self._drawModulePorts(
                ax,
                moduleRecord,
                xPosition,
                yPosition,
                moduleWidth,
                moduleHeight,
                connectedEndpointIds,
                unresolvedInputIds,
                unlinkedInputIds,
            )

    def _drawModulePorts(
        self,
        ax,
        moduleRecord,
        xPosition,
        yPosition,
        moduleWidth,
        moduleHeight,
        connectedEndpointIds,
        unresolvedInputIds,
        unlinkedInputIds,
    ):
        """Draw the input and output message ports for one module."""
        headerSpace = 0.65
        usableHeight = moduleHeight - headerSpace
        portSize = 40

        for endpointListName, xOffset, labelOffset, textAlign in (
            ("inputs", 0.0, 0.12, "left"),
            ("outputs", moduleWidth, -0.12, "right"),
        ):
            endpoints = moduleRecord[endpointListName]
            if len(endpoints) == 0:
                continue
            portSpacing = usableHeight / (len(endpoints) + 1)
            firstPortY = yPosition + moduleHeight * 0.5 - headerSpace
            for index, endpoint in enumerate(endpoints):
                portX = xPosition + xOffset
                portY = firstPortY - portSpacing * (index + 1)
                color = (
                    connectedMessageColor
                    if endpoint["id"] in connectedEndpointIds
                    else inactiveMessageColor
                )

                ax.scatter(
                    [portX],
                    [portY],
                    s=portSize,
                    color=color,
                    edgecolors="white",
                    linewidths=0.75,
                    zorder=5,
                )
                ax.text(
                    portX + labelOffset,
                    portY,
                    _shortText(endpoint["name"].split(".")[-1], 22),
                    fontsize=4.4,
                    color="#222222",
                    ha=textAlign,
                    va="center",
                    zorder=5,
                )

    def _drawMessageFigureLegend(self, ax, moduleLayout, moduleHeight, rowStep):
        """Draw a compact color legend for message connection figures."""
        if len(moduleLayout) == 0:
            legendY = -moduleHeight
        else:
            lastRow = min(yPosition for _, yPosition in moduleLayout.values())
            legendY = lastRow - rowStep * 0.35

        pointEntries = [
            ("message", inactiveMessageColor),
            ("connected", connectedMessageColor),
        ]
        for index, (label, color) in enumerate(pointEntries):
            xPosition = index * 1.45
            ax.scatter(
                [xPosition],
                [legendY],
                s=40,
                color=color,
                edgecolors="white",
                linewidths=0.7,
                zorder=5,
            )
            ax.text(
                xPosition + 0.14,
                legendY,
                label,
                fontsize=6.8,
                va="center",
                zorder=5,
            )

        lineEntries = [
            ("solid: module link", "-"),
            ("dashed: extraMessages", "--"),
        ]
        for index, (label, lineStyle) in enumerate(lineEntries):
            xPosition = 3.25 + index * 2.05
            ax.plot(
                [xPosition, xPosition + 0.42],
                [legendY, legendY],
                linestyle=lineStyle,
                linewidth=1.15,
                color=connectionColor,
                alpha=0.72,
                zorder=5,
            )
            ax.text(
                xPosition + 0.5,
                legendY,
                label,
                fontsize=6.8,
                va="center",
                zorder=5,
            )

    def AddModelToTask(self, TaskName, NewModel, ModelData=None, ModelPriority=-1):
        """
        This function is responsible for passing on the logger to a module instance (model), adding the
        model to a particular task, and defining
        the order/priority that the model gets updated within the task.

        :param TaskName (str): Name of the task
        :param NewModel (obj): Model to add to the task
        :param ModelData: None or struct containing, only used for C BSK modules
        :param ModelPriority (int): Priority that determines when the model gets updated. (Higher number = Higher priority)
        :return:
        """
        # Supports calling AddModelToTask(TaskName, NewModel, ModelPriority)
        if isinstance(ModelData, int):
            ModelPriority = ModelData
            ModelData = None

        for Task in self.TaskList:
            if Task.Name == TaskName:
                Task.TaskData.AddNewObject(NewModel, ModelPriority)
                if ModelData is not None:
                    try:
                        ModelData.bskLogger = self.bskLogger
                    except:
                        pass
                    Task.TaskModels.append(ModelData)
                    Task.TaskModelPriorities.append(ModelPriority)
                else:
                    try:
                        NewModel.bskLogger = self.bskLogger
                    except:
                        pass
                    Task.TaskModels.append(NewModel)
                    Task.TaskModelPriorities.append(ModelPriority)
                return
        raise ValueError(f"Could not find a Task with name: {TaskName}")

    def CreateNewProcess(self, procName, priority=-1):
        """
        Creates a process and adds it to the sim

        :param procName (str): Name of process
        :param priority (int): Priority that determines when the model gets updated. (Higher number = Higher priority)
        :return: simulationArchTypes.ProcessBaseClass object
        """
        proc = simulationArchTypes.ProcessBaseClass(procName, priority)
        self.procList.append(proc)
        self.TotalSim.addNewProcess(proc.processData)
        return proc

    def CreateNewTask(self, TaskName, TaskRate, InputDelay=None, FirstStart=0):
        """
        Creates a simulation task on the C-level with a specific update-frequency (TaskRate), an optional delay, and
        an optional start time.

        Args:
            TaskName (str): Name of Task
            TaskRate (int): Number of nanoseconds to elapse before update() is called
            InputDelay (int): (deprecated, unimplemented) Number of nanoseconds simulating a lag of the particular task
            FirstStart (int): Number of nanoseconds to elapse before task is officially enabled

        Returns:
            simulationArchTypes.TaskBaseClass object
        """

        if InputDelay is not self.CreateNewTask.__defaults__[0]:
            deprecated.deprecationWarn(
                "InputDelay",
                "2024/12/13",
                "This input variable is non-functional and now deprecated.",
            )

        Task = simulationArchTypes.TaskBaseClass(TaskName, TaskRate, FirstStart)
        self.TaskList.append(Task)
        return Task

    def ResetTask(self, taskName):
        for Task in self.TaskList:
            if Task.Name == taskName:
                Task.resetTask(self.TotalSim.CurrentNanos)

    def InitializeSimulation(self):
        """
        Initialize the BSK simulation.  This runs the SelfInit() and Reset() methods on each module.
        """
        if self.simulationInitialized:
            self.TotalSim.resetThreads(self.TotalSim.getThreadCount())
        self.TotalSim.assignRemainingProcs()
        self.TotalSim.ResetSimulation()
        self.TotalSim.selfInitSimulation()
        self.TotalSim.resetInitSimulation()
        self.simulationInitialized = True

    def ConfigureStopTime(self, TimeStop, StopCondition: Literal["<=", ">="] = "<="):
        """
        Set the simulation stop time in nano-seconds.

        Args:
            TimeStop (int): Time to stop the simulation in nanoseconds
            StopCondition (str): Condition for meeting the stop time. Two behaviors
                are supported:

                * ``<=``: (default) The simulation will run as far as it can such that the
                  ``StopTime`` is met if possible, but not exceeded.
                * ``>=``: The simulation will run as far as it can such that the
                  ``StopTime`` is minimally exceeded, but met if possible.
        """
        self.StopTime = TimeStop
        assert StopCondition in ("<=", ">="), "StopCondition must be '<=' or '>='"
        self.StopCondition = StopCondition

    def CheckStopCondition(self):
        if self.StopCondition == "<=":
            return self.TotalSim.NextTaskTime <= self.StopTime
        elif self.StopCondition == ">=":
            return (
                self.TotalSim.CurrentNanos < self.StopTime
                or self.TotalSim.NextTaskTime == self.StopTime
            )

    def _progressBarUpdateInterval(self):
        if not self.showProgressBar or self.StopTime <= self.TotalSim.CurrentNanos:
            return None
        if self.progressBarUpdateInterval is not None:
            return self.progressBarUpdateInterval
        remainingTime = self.StopTime - self.TotalSim.CurrentNanos
        return max(1, math.ceil(remainingTime / self.progressBarTargetUpdates))

    def _limitStopTimeForProgress(self, nextStopTime, nextProgressUpdateTime):
        if nextProgressUpdateTime is None:
            return nextStopTime
        progressStopTime = max(nextProgressUpdateTime, self.TotalSim.NextTaskTime)
        return min(nextStopTime, progressStopTime)

    def ExecuteSimulation(self):
        """
        run the simulation until the prescribed stop time or termination.
        """

        progressBar = SimulationProgressBar(self.StopTime, self.showProgressBar)
        progressUpdateInterval = self._progressBarUpdateInterval()
        nextProgressUpdateTime = None
        if progressUpdateInterval is not None:
            nextProgressUpdateTime = self.TotalSim.CurrentNanos + progressUpdateInterval
        while self.CheckStopCondition():
            # Check events. Iterate by index rather than over a snapshot so that
            # an event action which activates another event that is already due
            # this cycle still gets that event checked before the pass ends
            # (matching the historical lazy-generator behaviour, see issue #455).
            # ``checkedEvents`` guarantees each event is checked at most once per
            # cycle, even when a triggered event rebuilds the cache mid-pass.
            checkedEvents = set()
            activeEvents = self._ensureActiveEventCache()
            i = 0
            while i < len(activeEvents):
                event = activeEvents[i]
                i += 1
                if id(event) in checkedEvents:
                    continue
                if event.shouldBeChecked(self.TotalSim.CurrentNanos):
                    checkedEvents.add(id(event))
                    event.checkEvent(self)
                    if self._activeEventCache is not activeEvents:
                        # A triggered event invalidated and rebuilt the cache
                        # (and may have activated further events). Continue from
                        # the fresh list; ``checkedEvents`` prevents re-checking
                        # anything already handled this cycle.
                        activeEvents = self._ensureActiveEventCache()
                        i = 0

            if self.terminate:
                break

            # Find the next time to stop the sim
            eventCheckTimes = [
                event.nextCheckTime(self.TotalSim.CurrentNanos)
                for event in self._ensureActiveEventCache()
            ]
            if len(eventCheckTimes) > 0:
                # Stop at next event, if any
                nextStopTime = min(eventCheckTimes)
                # But must at least reach the next task
                nextStopTime = max(nextStopTime, self.TotalSim.NextTaskTime)
                # But don't pass stop
                nextStopTime = min(nextStopTime, self.StopTime)
            else:
                nextStopTime = self.StopTime  # Otherwise stop at the stop time

            # Must at least step to the next task time if StopCondition is ">="
            if self.StopCondition == ">=":
                nextStopTime = max(nextStopTime, self.TotalSim.NextTaskTime)

            nextStopTime = self._limitStopTimeForProgress(
                nextStopTime, nextProgressUpdateTime
            )

            # Execute the sim
            nextPriority = -1
            self.TotalSim.StepUntilStop(int(nextStopTime), nextPriority)
            progressBar.update(self.TotalSim.NextTaskTime)
            if nextProgressUpdateTime is not None:
                while nextProgressUpdateTime <= self.TotalSim.NextTaskTime:
                    nextProgressUpdateTime += progressUpdateInterval
        self.terminate = False
        progressBar.markComplete()
        progressBar.close()

    def disableTask(self, TaskName):
        """
        Disable this particular task from being executed.
        """
        for Task in self.TaskList:
            if Task.Name == TaskName:
                Task.disable()

    def enableTask(self, TaskName):
        """
        Enable this particular task to be executed.
        """
        for Task in self.TaskList:
            if Task.Name == TaskName:
                Task.enable()

    def parseDataIndex(self):
        self.dataStructureDictionary = {}
        try:
            xmlData = ET.parse(self.dataStructIndex)
        except:
            print("Failed to parse the XML index.  Likely that it isn't present")
            return
        root = xmlData.getroot()
        for child in root:
            newStruct = StructDocData(child.attrib["refid"])
            self.dataStructureDictionary.update({child.find("name").text: newStruct})
        self.indexParsed = True

    def createNewEvent(self, eventName, *args, **kwargs):
        """
        Create an event sequence that contains a series of tasks to be executed.

        Args:
            eventName (str): Name of the event
            *args: Arguments to pass to the :class:`EventHandlerClass` constructor
            **kwargs: Keyword arguments to pass to the :class:`EventHandlerClass` constructor
        """
        if eventName in list(self.eventMap.keys()):
            warnings.warn(f"Skipping event creation since {eventName} already exists.")
            return
        newEvent = EventHandlerClass(eventName, *args, **kwargs)
        self.eventMap[eventName] = newEvent
        self._invalidateActiveEventCache()

    def _invalidateActiveEventCache(self):
        """Mark the cached active-event list stale. Must be called whenever event
        membership (``eventMap``) or any event's ``eventActive`` flag changes."""
        self._activeEventCache = None

    def _ensureActiveEventCache(self):
        """Return the internal cached list of active events, rebuilding it from
        ``eventMap`` if it has been invalidated.

        This is the hot path used by the simulation loop. It returns the actual
        cached list (not a copy); external callers must use :meth:`activeEvents`
        instead, which hands out an immutable snapshot. The cache is only rebuilt
        when it has been invalidated (an event was added, its activity was
        toggled, or an event triggered), so the per-check-cycle cost stays
        proportional to the number of *active* events rather than the total
        number of events (see issue #455)."""
        if self._activeEventCache is None:
            self._activeEventCache = [
                event for event in self.eventMap.values() if event.eventActive
            ]
        return self._activeEventCache

    def activeEvents(self):
        """Return an immutable snapshot of the currently active events.

        A tuple, not the internal cache list, is returned so that external
        callers cannot mutate the cache and silently change which events fire.
        See :meth:`_ensureActiveEventCache` for the caching behaviour."""
        return tuple(self._ensureActiveEventCache())

    def setEventActivity(self, eventName, activityCommand):
        """Enable or disable the named event.

        ``activityCommand`` is the boolean value assigned to the event's
        ``eventActive`` flag. The cached active-event list (see
        :meth:`activeEvents`) is invalidated only when the flag actually
        changes, so setting an event to the activity state it already has does
        not force an unnecessary rebuild."""
        if eventName not in list(self.eventMap.keys()):
            print("You asked me to set the status of an event that I don't have.")
            return
        event = self.eventMap[eventName]
        if event.eventActive != activityCommand:
            event.eventActive = activityCommand
            self._invalidateActiveEventCache()

    def setAllButCurrentEventActivity(
        self, currentEventName, activityCommand, useIndex=False
    ):
        """Set all event activity variables except for the currentEventName event. The ``useIndex`` flag can be used to
        prevent enabling or disabling every task, and instead only alter the ones that belong to the same group (for
        example, the same spacecraft). The distinction is made through an index set after the ``_`` symbol in the event
        name. All events of the same group must have the same index."""

        if useIndex:
            index = currentEventName.partition("_")[2]  # save the current event's index

        # Invalidate the active-event cache only if at least one event's activity
        # actually changed, so a no-op blanket set does not force a rebuild.
        activityChanged = False
        for eventName in list(self.eventMap.keys()):
            if currentEventName != eventName:
                if useIndex:
                    if eventName.partition("_")[2] == index:
                        if self.eventMap[eventName].eventActive != activityCommand:
                            self.eventMap[eventName].eventActive = activityCommand
                            activityChanged = True
                else:
                    if self.eventMap[eventName].eventActive != activityCommand:
                        self.eventMap[eventName].eventActive = activityCommand
                        activityChanged = True
        if activityChanged:
            self._invalidateActiveEventCache()


def SetCArray(InputList, VarType, ArrayPointer):
    if isinstance(ArrayPointer, (list, tuple)):
        raise TypeError(
            "Cannot set a C array if it is actually a python list.  Just assign the variable to the list directly."
        )
    arraySetter = getattr(sim_model, VarType + "Array_setitem")
    for currIndex, currElem in enumerate(InputList):
        arraySetter(ArrayPointer, currIndex, currElem)


def getCArray(varType, arrayPointer, arraySize):
    arrayGetter = getattr(sim_model, varType + "Array_getitem")
    return [arrayGetter(arrayPointer, currIndex) for currIndex in range(arraySize)]


def synchronizeTimeHistories(arrayList):
    returnArrayList = arrayList
    timeCounter = 0
    for i in range(len(returnArrayList)):
        while returnArrayList[i][0, 0] > returnArrayList[0][timeCounter, 0]:
            timeCounter += 1
    for i in range(len(returnArrayList)):
        while returnArrayList[i][1, 0] < returnArrayList[0][timeCounter, 0]:
            returnArrayList[i] = np.delete(returnArrayList[i], 0, 0)

    timeCounter = -1
    for i in range(len(returnArrayList)):
        while returnArrayList[i][-1, 0] < returnArrayList[0][timeCounter, 0]:
            timeCounter -= 1
    for i in range(len(returnArrayList)):
        while returnArrayList[i][-2, 0] > returnArrayList[0][timeCounter, 0]:
            returnArrayList[i] = np.delete(returnArrayList[i], -1, 0)

    timeNow = returnArrayList[0][
        0, 0
    ]  # Desirement is to have synched arrays match primary time
    outputArrayList = []
    indexPrev = [0] * len(returnArrayList)
    outputArrayList = [[]] * len(returnArrayList)
    timeNow = returnArrayList[0][0, 0]

    outputArrayList[0] = returnArrayList[0][0:-2, :]
    for i in range(1, returnArrayList[0].shape[0] - 1):
        for j in range(1, len(returnArrayList)):
            while returnArrayList[j][indexPrev[j] + 1, 0] < returnArrayList[0][i, 0]:
                indexPrev[j] += 1

            dataProp = (
                returnArrayList[j][indexPrev[j] + 1, 1:]
                - returnArrayList[j][indexPrev[j], 1:]
            )
            dataProp *= (timeNow - returnArrayList[j][indexPrev[j], 0]) / (
                returnArrayList[j][indexPrev[j] + 1, 0]
                - returnArrayList[j][indexPrev[j], 0]
            )
            dataProp += returnArrayList[j][indexPrev[j], 1:]
            dataRow = [timeNow]
            dataRow.extend(dataProp.tolist())
            outputArrayList[j].append(dataRow)
        timePrevious = timeNow
        timeNow = returnArrayList[0][i, 0]
    for j in range(1, len(returnArrayList)):
        outputArrayList[j] = np.array(outputArrayList[j])

    return outputArrayList
