
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
import os
import warnings
import xml.etree.ElementTree as ET
from collections import OrderedDict

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.architecture import alg_contain, bskLogging, sim_model
from Basilisk.utilities import deprecated, simulationArchTypes
from Basilisk.utilities.pythonVariableLogger import PythonVariableLogger
from Basilisk.utilities.simulationProgessBar import SimulationProgressBar

# Point the path to the module storage area


# define ASCI color codes
processColor = '\u001b[32m'
taskColor = '\u001b[33m'
moduleColor = '\u001b[36m'
endColor = '\u001b[0m'

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
    """Event Handler Class"""

    def __init__(
        self,
        eventName,
        eventRate=int(1e9),
        eventActive=False,
        conditionFunction=None,
        actionFunction=None,
        conditionList=None,
        actionList=None,
        terminal=False,
    ):
        self.eventName = eventName
        self.eventActive = eventActive
        self.eventRate = eventRate
        self.occurCounter = 0
        self.prevTime = -1
        self.terminal = terminal

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



    def checkEvent(self, parentSim):
        nextTime = int(-1)
        if not self.eventActive:
            return(nextTime)
        nextTime = self.prevTime + self.eventRate - (self.prevTime%self.eventRate)
        if self.prevTime < 0 or (parentSim.TotalSim.CurrentNanos%self.eventRate == 0):
            nextTime = parentSim.TotalSim.CurrentNanos + self.eventRate
            eventOccurred = self.conditionFunction(parentSim)
            self.prevTime = parentSim.TotalSim.CurrentNanos
            if eventOccurred:
                self.eventActive = False
                self.actionFunction(parentSim)
                self.occurCounter += 1
                if self.terminal:
                    parentSim.terminate = True
        return(nextTime)


class StructDocData:
    """Structure data documentation class"""
    class StructElementDef:
        def __init__(self, type, name, argstring, desc=''):
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
        xmlFileUse = xmlSearchPath + '/' + self.strName + '.xml'
        try:
            xmlData = ET.parse(xmlFileUse)
        except:
            print("Failed to parse the XML structure for: " + self.strName)
            print("This file does not exist most likely: " + xmlFileUse)
            return
        root = xmlData.getroot()
        validElement = root.find("./compounddef[@id='" + self.strName + "']")
        for newVariable in validElement.findall(".//memberdef[@kind='variable']"):
            typeUse = newVariable.find('type').text if newVariable.find('type') is not None else \
                None
            nameUse = newVariable.find('name').text if newVariable.find('type') is not None else \
                None
            argstringUse = newVariable.find('argsstring').text if newVariable.find('argsstring') is not None else \
                None
            descUse = newVariable.find('./detaileddescription/para').text if newVariable.find(
                './detaileddescription/para') is not None else \
                None
            if descUse == None:
                descUse = newVariable.find('./briefdescription/para').text if newVariable.find(
                    './briefdescription/para') is not None else \
                    None
            newElement = StructDocData.StructElementDef(typeUse, nameUse, argstringUse, descUse)
            self.structElements.update({nameUse: newElement})
            self.structPopulated = True

    def printElem(self):
        print("    " + self.strName + " Structure Elements:")
        for key, value in self.structElements.items():
            outputString = ''
            outputString += value.type + " " + value.name
            outputString += value.argstring if value.argstring is not None else ''
            outputString += ': ' + value.desc if value.desc is not None else ''
        print("      " + outputString)

class DataPairClass:
    def __init__(self):
        self.outputMessages = set([])
        self.inputMessages = set([])
        self.name = ""
        self.outputDict = {}

class SimBaseClass:
    """Simulation Base Class"""
    def __init__(self):
        self.TotalSim = sim_model.SimModel()
        self.TaskList = []
        self.procList = []
        self.StopTime = 0
        self.nextEventTime = 0
        self.terminate = False
        self.eventMap = {}
        self.simBasePath = os.path.dirname(os.path.realpath(__file__)) + '/../'
        self.dataStructIndex = self.simBasePath + '/xml/index.xml'
        self.indexParsed = False
        self.simulationInitialized = False
        self.simulationFinished = False
        self.bskLogger = bskLogging.BSKLogger()
        self.showProgressBar = False
        self.allModules = set()

    def SetProgressBar(self, value):
        """
        Shows a dynamic progress in the terminal while the simulation is executing.
        """
        self.showProgressBar = value

    def ShowExecutionOrder(self):
        """
        Shows in what order the Basilisk processes, task lists and modules are executed
        """

        for processData in self. TotalSim.processList:
            print(f"{processColor}Process Name: {endColor}" + processData.processName +
                  " , " + processColor + "priority: " + endColor + str(processData.processPriority))
            for task in processData.processTasks:
                print(f"{taskColor}Task Name: {endColor}" + task.TaskPtr.TaskName +
                      ", " + taskColor + "priority: " + endColor + str(task.taskPriority) +
                      ", " + taskColor + "TaskPeriod: " + endColor + str(task.TaskPtr.TaskPeriod/1.0e9) + "s")
                for module in task.TaskPtr.TaskModels:
                    print(moduleColor + "ModuleTag: " + endColor + module.ModelPtr.ModelTag +
                          ", " + moduleColor + "priority: " + endColor + str(module.CurrentModelPriority))
            print("")


    def ShowExecutionFigure(self, show_plots=False):
        """
        Shows in what order the Basilisk processes, task lists and modules are executed
        """
        processList = OrderedDict()
        for processData in self. TotalSim.processList:
            taskList = OrderedDict()
            for task in processData.processTasks:
                moduleList = []
                for module in task.TaskPtr.TaskModels:
                    moduleList.append(module.ModelPtr.ModelTag + " (" + str(module.CurrentModelPriority) + ")")
                taskList[task.TaskPtr.TaskName + " (" + str(task.taskPriority) + ", " + str(task.TaskPtr.TaskPeriod/1.0e9) + "s)"] = moduleList
            processList[processData.processName + " (" + str(processData.processPriority) + ")"] = taskList

        fig = plt.figure()
        plt.rcParams.update({'font.size': 8})
        plt.axis('off')

        processNo = 0
        processWidth = 6
        lineHeight = 0.5
        textBuffer = lineHeight*0.75
        textIndent = lineHeight*0.25
        processGap = 0.5
        for process in processList:
            # Draw process box + priority
            rectangle = plt.Rectangle(((processWidth+processGap)*processNo, 0), processWidth, -lineHeight, ec='g', fc='g')
            plt.gca().add_patch(rectangle)
            plt.text((processWidth+processGap)*processNo + textIndent, -textBuffer, process, color='w')

            taskNo = 0
            currentLine = -lineHeight - textIndent
            for task in processList[process]:
                # Draw task box + priority + task rate
                rectangle = plt.Rectangle(((processWidth + processGap) * processNo + textIndent, currentLine)
                                          , processWidth - 2 * textIndent
                                          , - (1+len(processList[process][task])) * (lineHeight + textIndent),
                                          ec='y', fc=(1,1,1,0))
                plt.gca().add_patch(rectangle)
                rectangle = plt.Rectangle(((processWidth + processGap) * processNo + textIndent, currentLine)
                                          , processWidth - 2 * textIndent, -lineHeight,
                                          ec='y', fc='y')
                plt.gca().add_patch(rectangle)
                plt.text((processWidth + processGap) * processNo + 2*textIndent,
                         currentLine-textBuffer, task, color='black')

                for module in processList[process][task]:
                    # Draw modules + priority
                    currentLine -= lineHeight + textIndent
                    rectangle = plt.Rectangle(((processWidth + processGap) * processNo + 2*textIndent, currentLine)
                                              , processWidth - 4 * textIndent, -lineHeight,
                                              ec='c', fc=(1,1,1,0))
                    plt.gca().add_patch(rectangle)
                    plt.text((processWidth + processGap) * processNo + 3*textIndent,
                             currentLine-textBuffer, module, color='black')

                taskNo += 1
                currentLine -=  lineHeight + 2 * textIndent

            rectangle = plt.Rectangle(((processWidth+processGap)*processNo, 0), processWidth, currentLine, ec='g', fc=(1,1,1,0))
            plt.gca().add_patch(rectangle)
            processNo += 1

        plt.axis('scaled')

        if show_plots:
            plt.show()

        return fig

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
                else:
                    try:
                        NewModel.bskLogger = self.bskLogger
                    except:
                        pass
                    Task.TaskModels.append(NewModel)
                return
        raise ValueError(f"Could not find a Task with name: {TaskName}")

    def CreateNewProcess(self, procName, priority = -1):
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
            InputDelay (int): (depreciated, unimplemented) Number of nanoseconds simulating a lag of the particular task
            FirstStart (int): Number of nanoseconds to elapse before task is officially enabled

        Returns:
            simulationArchTypes.TaskBaseClass object
        """

        if InputDelay is not self.CreateNewTask.__defaults__[0]:
            deprecated.deprecationWarn("InputDelay", "2024/12/13",
                                       "This input variable is non-functional and now depreciated.")

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
        if(self.simulationInitialized):
            self.TotalSim.resetThreads(self.TotalSim.getThreadCount())
        self.TotalSim.assignRemainingProcs()
        self.TotalSim.ResetSimulation()
        self.TotalSim.selfInitSimulation()
        self.TotalSim.resetInitSimulation()
        self.simulationInitialized = True


    def ConfigureStopTime(self, TimeStop):
        """
        Set the simulation stop time in nano-seconds.
        """
        self.StopTime = TimeStop

    def ExecuteSimulation(self):
        """
        run the simulation until the prescribed stop time or termination.
        """
        self.initializeEventChecks()

        nextStopTime = self.TotalSim.NextTaskTime
        nextPriority = -1
        progressBar = SimulationProgressBar(self.StopTime, self.showProgressBar)
        while self.TotalSim.NextTaskTime <= self.StopTime and not self.terminate:
            if self.TotalSim.CurrentNanos >= self.nextEventTime >= 0:
                self.nextEventTime = self.checkEvents()
                self.nextEventTime = self.nextEventTime if self.nextEventTime >= self.TotalSim.NextTaskTime else self.TotalSim.NextTaskTime
            if 0 <= self.nextEventTime < nextStopTime:
                nextStopTime = self.nextEventTime
                nextPriority = -1
            if self.terminate:
                break
            self.TotalSim.StepUntilStop(nextStopTime, nextPriority)
            progressBar.update(self.TotalSim.NextTaskTime)
            nextPriority = -1
            nextStopTime = self.StopTime
            nextStopTime = nextStopTime if nextStopTime >= self.TotalSim.NextTaskTime else self.TotalSim.NextTaskTime
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
            newStruct = StructDocData(child.attrib['refid'])
            self.dataStructureDictionary.update({child.find('name').text:
                                                     newStruct})
        self.indexParsed = True

    def createNewEvent(
        self,
        eventName,
        eventRate=int(1e9),
        eventActive=False,
        conditionList=None,
        actionList=None,
        terminal=False,
        conditionFunction=None,
        actionFunction=None,
    ):
        """
        Create an event sequence that contains a series of tasks to be executed.

        Args:
            eventName (str): Name of the event
            eventRate (int): Rate at which the event is checked in nanoseconds
            eventActive (bool): Whether the event is active or not
            conditionList (list): List of conditions to check for the event,
                expressed as strings of code to execute within the class.
            actionList (list): List of actions to perform when the event occurs,
                expressed as strings of code to execute within the class.
            terminal (bool): Whether this event should terminate the simulation when it occurs
            conditionFunction (function): Function to check if the event should occur. The
                function should take the simulation object as an argument and return a boolean.
                This is the preferred manner to set conditions as it enables the use of arbitrary
                packages and objects in events and allows for event code to be parsed by IDE tools.
            actionFunction (function): Function to execute when the event occurs. The
                function should take the simulation object as an argument.
                This is the preferred manner to set conditions as it enables the use of arbitrary
                packages and objects in events and allows for event code to be parsed by IDE tools.
        """
        if (eventName in list(self.eventMap.keys())):
            warnings.warn(f"Skipping event creation since {eventName} already exists.")
            return
        newEvent = EventHandlerClass(
            eventName,
            eventRate,
            eventActive,
            conditionFunction=conditionFunction,
            actionFunction=actionFunction,
            conditionList=conditionList,
            actionList=actionList,
            terminal=terminal,
        )
        self.eventMap[eventName] = newEvent

    def initializeEventChecks(self):
        self.eventList = list(self.eventMap.values())
        self.nextEventTime = 0

    def checkEvents(self):
        nextTime = -1
        for localEvent in self.eventList:
            localNextTime = localEvent.checkEvent(self)
            if(localNextTime >= 0 and (localNextTime < nextTime or nextTime <0)):
                nextTime = localNextTime
        return nextTime

    def setEventActivity(self, eventName, activityCommand):
        if eventName not in list(self.eventMap.keys()):
            print("You asked me to set the status of an event that I don't have.")
            return
        self.eventMap[eventName].eventActive = activityCommand

    def setAllButCurrentEventActivity(self, currentEventName, activityCommand, useIndex=False):
        """Set all event activity variables except for the currentEventName event. The ``useIndex`` flag can be used to
        prevent enabling or disabling every task, and instead only alter the ones that belong to the same group (for
        example, the same spacecraft). The distinction is made through an index set after the ``_`` symbol in the event
        name. All events of the same group must have the same index."""

        if useIndex:
            index = currentEventName.partition('_')[2]  # save the current event's index

        for eventName in list(self.eventMap.keys()):
            if currentEventName != eventName:
                if useIndex:
                    if eventName.partition('_')[2] == index:
                        self.eventMap[eventName].eventActive = activityCommand
                else:
                    self.eventMap[eventName].eventActive = activityCommand

def SetCArray(InputList, VarType, ArrayPointer):
    if(isinstance(ArrayPointer, (list, tuple))):
        raise TypeError('Cannot set a C array if it is actually a python list.  Just assign the variable to the list directly.')
    CmdString = 'sim_model.' + VarType + 'Array_setitem(ArrayPointer, CurrIndex, CurrElem)'
    CurrIndex = 0
    for CurrElem in InputList:
        exec (CmdString)
        CurrIndex += 1

def getCArray(varType, arrayPointer, arraySize):
    CmdString = 'outputList.append(sim_model.' + varType + 'Array_getitem(arrayPointer, currIndex))'
    outputList = []
    currIndex = 0
    for currIndex in range(arraySize):
        exec (CmdString)
        currIndex += 1
    return outputList

def synchronizeTimeHistories(arrayList):
    returnArrayList = arrayList
    timeCounter = 0
    for i in range(len(returnArrayList)):
        while returnArrayList[i][0,0] > returnArrayList[0][timeCounter,0]:
            timeCounter += 1
    for i in range(len(returnArrayList)):
        while(returnArrayList[i][1,0] < returnArrayList[0][timeCounter,0]):
            returnArrayList[i] = np.delete(returnArrayList[i], 0, 0)

    timeCounter = -1
    for i in range(len(returnArrayList)):
        while returnArrayList[i][-1,0] < returnArrayList[0][timeCounter,0]:
                timeCounter -= 1
    for i in range(len(returnArrayList)):
        while(returnArrayList[i][-2,0] > returnArrayList[0][timeCounter,0]):
            returnArrayList[i] = np.delete(returnArrayList[i], -1, 0)

    timeNow = returnArrayList[0][0,0] #Desirement is to have synched arrays match primary time
    outputArrayList = []
    indexPrev = [0]*len(returnArrayList)
    outputArrayList = [[]]*len(returnArrayList)
    timeNow = returnArrayList[0][0,0]

    outputArrayList[0] = returnArrayList[0][0:-2, :]
    for i in range(1, returnArrayList[0].shape[0]-1):
        for j in range(1, len(returnArrayList)):
            while(returnArrayList[j][indexPrev[j]+1,0] < returnArrayList[0][i,0]):
                indexPrev[j] += 1

            dataProp = returnArrayList[j][indexPrev[j]+1,1:] - returnArrayList[j][indexPrev[j],1:]
            dataProp *= (timeNow - returnArrayList[j][indexPrev[j],0])/(returnArrayList[j][indexPrev[j]+1,0] - returnArrayList[j][indexPrev[j],0])
            dataProp += returnArrayList[j][indexPrev[j],1:]
            dataRow = [timeNow]
            dataRow.extend(dataProp.tolist())
            outputArrayList[j].append(dataRow)
        timePrevious = timeNow
        timeNow = returnArrayList[0][i,0]
    for j in range(1, len(returnArrayList)):
        outputArrayList[j] = np.array(outputArrayList[j])

    return outputArrayList
