
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



import array
import inspect

# Import some architectural stuff that we will probably always use
import os
import sys
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

class EventHandlerClass:
    """Event Handler Class"""
    def __init__(self, eventName, eventRate=int(1E9), eventActive=False,
                 conditionList=[], actionList=[], terminal=False):
        self.eventName = eventName
        self.eventActive = eventActive
        self.eventRate = eventRate
        self.conditionList = conditionList
        self.actionList = actionList
        self.occurCounter = 0
        self.prevTime = -1
        self.checkCall = None
        self.operateCall = None
        self.terminal = terminal

    def methodizeEvent(self):
        if self.checkCall != None:
            return
        funcString = 'def EVENT_check_' + self.eventName + '(self):\n'
        funcString += '    if('
        for condValue in self.conditionList:
            funcString += ' ' + condValue + ' and'
        funcString = funcString[:-3] + '):\n'
        funcString += '        return 1\n'
        funcString += '    return 0'

        exec (funcString)
        self.checkCall = eval('EVENT_check_' + self.eventName)
        funcString = 'def EVENT_operate_' + self.eventName + '(self):\n'
        for actionValue in self.actionList:
            funcString += '    '
            funcString += actionValue + '\n'
        funcString += '    return 0'
        exec (funcString)
        self.operateCall = eval('EVENT_operate_' + self.eventName)

    def checkEvent(self, parentSim):
        nextTime = int(-1)
        if self.eventActive == False:
            return(nextTime)
        nextTime = self.prevTime + self.eventRate - (self.prevTime%self.eventRate)
        if self.prevTime < 0 or (parentSim.TotalSim.CurrentNanos%self.eventRate == 0):
            nextTime = parentSim.TotalSim.CurrentNanos + self.eventRate
            eventCount = self.checkCall(parentSim)
            self.prevTime = parentSim.TotalSim.CurrentNanos
            if eventCount > 0:
                self.eventActive = False
                self.operateCall(parentSim)
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
        self.oldSyntaxVariableLog = {}
        self.allModels = []
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
                self.allModels.append((NewModel, ModelData, Task) )
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

    # When this method is removed, remember to delete the 'oldSyntaxVariableLog' and
    # 'allModels' attributes (as well as any mention of them) as they are no longer needed
    @deprecated.deprecated("2024/09/06", 
        "Use the 'logger' function or 'PythonVariableLogger' instead of 'AddVariableForLogging'."
        " See 'http://hanspeterschaub.info/basilisk/Learn/bskPrinciples/bskPrinciples-6.html'"
    )
    def AddVariableForLogging(self, VarName: str, LogPeriod: int = 0, *_, **__):
        """Generates a logger and adds it to the same task as the module
        in `VarName`.

        Args:
            VarName (str): The variable to log in the format "<ModelTag>.<variable_name>"
            LogPeriod (int, optional): The minimum time between logs. Defaults to 0.
        """        
        if "." not in VarName:
            raise ValueError('The variable to log must be given in the format '
                             '"<ModelTag>.<variable_name>"')
        
        modelTag = VarName.split('.')[0]

        # Calling eval on a pre-compiled string is faster than
        # eval-ing the string (by a large factor)
        compiledExpr = compile(VarName, "<logged-variable>", "eval")

        # Find the model object that corresponds to the given tag, as well as the 
        # task where this model was added
        modelOrConfig = task = None
        for model, modelData, task in self.allModels:
            if model.ModelTag == modelTag:
                modelOrConfig = modelData or model
                break

        if task is None or modelOrConfig is None:
            raise ValueError(f"Could not find model with tag {modelTag}")

        # The callback logging function 'fun' simply evaluates the given
        # expression. We pass a dictionary '{modelTag: modelOrConfig}'
        # that allows the expression to substitute the modelTag by the
        # actual model object
        def fun(_): 
            val = eval(compiledExpr, globals(), {modelTag: modelOrConfig})
            val = np.array(val).squeeze()
            return val

        logger = PythonVariableLogger({"variable": fun}, LogPeriod)
        logger.ModelTag = f"Logger:{VarName}"
        self.AddModelToTask(task.Name, logger)

        self.oldSyntaxVariableLog[VarName] = logger

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

    @deprecated.deprecated("2024/09/06", 
        "Calling 'RecordLogVars' is deprecated and unnecessary."
    )
    def RecordLogVars(self):
        pass

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

    @deprecated.deprecated("2024/09/06", 
        "Deprecated way to access logged variables."
        " See 'http://hanspeterschaub.info/basilisk/Learn/bskPrinciples/bskPrinciples-6.html'"
    )
    def GetLogVariableData(self, LogName):
        """
        Pull the recorded module recorded variable.  The first column is the variable recording time in
        nano-seconds, the additional column(s) are the message data columns.
        """        
        if LogName not in self.oldSyntaxVariableLog:
            raise ValueError(f'"{LogName}" is not being logged. Check the spelling.')
        
        logger = self.oldSyntaxVariableLog[LogName]
        return np.column_stack([logger.times(), logger.variable])

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

    def createNewEvent(self, eventName, eventRate=int(1E9), eventActive=False,
                       conditionList=[], actionList=[], terminal=False):
        """
        Create an event sequence that contains a series of tasks to be executed.
        """
        if (eventName in list(self.eventMap.keys())):
            return
        newEvent = EventHandlerClass(eventName, eventRate, eventActive,
                                     conditionList, actionList, terminal)
        self.eventMap.update({eventName: newEvent})

    def initializeEventChecks(self):
        self.eventList = []
        for key, value in self.eventMap.items():
            value.methodizeEvent()
            self.eventList.append(value)
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

    def setModelDataWrap(self, modelData):
        """
        Takes a module and returns an object that provides access to said module's SelfInit, Update, and Reset
        methods.

        Takes the module instance, collects all SwigPyObjects generated from the .i file (SelfInit,
        Update and Reset), and attaches it to a alg_contain model instance so the modules standard functions can be
        run at the python level.

        :param modelData: model to gather functions for
        :return: An alg_contain model that provides access to the original model's core functions
        """
        deprecationId = f"{SimBaseClass.setModelDataWrap.__module__}.{SimBaseClass.setModelDataWrap.__qualname__}"
        removalDate = "2024/07/30"

        if hasattr(modelData, "createWrapper"):
            deprecated.deprecationWarn(
                deprecationId,
                removalDate,
                "C modules no longer require having separate 'Config' and 'Wrap' objects. "
                "Treat C modules like C++ modules. For example, instead of:\n"
                "\tinertial3DConfig = inertial3D.inertial3DConfig()\n"
                "\tinertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)\n"
                "\tinertial3DWrap.ModelTag = 'inertial3D'\n"
                "\tscSim.AddModelToTask(simTaskName, inertial3DWrap, inertial3DConfig, 10)\n"
                "Do:\n"
                "\tinertial3D = inertial3D.inertial3D()\n"
                "\tinertial3D.ModelTag = 'inertial3D'\n"
                "\tscSim.AddModelToTask(simTaskName, inertial3D, 10)\n"
            )
            return modelData.createWrapper()
    
        deprecated.deprecationWarn(
            deprecationId, 
            removalDate, 
            "This C module has not been converted yet to the new way of defining C "
            "modules, which makes using them more intuitive. Take the time to see how "
            "the new C module '.i' file looks by checking out a default Basilisk module"
            " and adapt your module to use a similar format."
        )

        algDict = {}
        STR_SELFINIT = 'SelfInit'
        STR_UPDATE = 'Update'
        STR_RESET = 'Reset'

        # SwigPyObject's Parsing:
        # Collect all the SwigPyObjects present in the list. Only the methods SelfInit, Update and Restart
        # are wrapped by Swig in the .i files. Therefore they are the only SwigPyObjects
        def parseDirList(dirList):
            algNames = []
            for methodName in dirList:
                methodObject = eval('sys.modules["' + module + '"].' + methodName)
                if type(methodObject).__name__ == "SwigPyObject":
                    algNames.append(methodName)
            return algNames

        # Check the type of the algorithm, i.e. SelfInit, Update or Reset,
        # and return the key to create a new dictionary D[str_method] = method
        def checkMethodType(methodName):
            if methodName[0:len(STR_SELFINIT)] == STR_SELFINIT:
                return STR_SELFINIT
            elif methodName[0:len(STR_UPDATE)] == STR_UPDATE:
                return STR_UPDATE
            elif methodName[0:len(STR_RESET)] == STR_RESET:
                return STR_RESET
            else:
                raise ValueError('Cannot recognize the method'
                                 '(I only assess SelfInit, Update and Reset methods). '
                                 'Parse better.')

        module = modelData.__module__
        sysMod = sys.modules[module]
        dirList = dir(sysMod)
        algList = parseDirList(dirList)

        # if the package has different levels we need to access the correct level of the package
        currMod = __import__(module, globals(), locals(), [], 0)

        moduleString = "currMod."
        moduleNames = module.split(".")
        if len(moduleNames) > 1:
            moduleString += ".".join(moduleNames[1:]) + "."

        for alg in algList:
            key = checkMethodType(alg)
            algDict[key] = alg

        update = eval(moduleString + algDict[STR_UPDATE])
        selfInit = eval(moduleString + algDict[STR_SELFINIT])
        try:
            resetArg = algDict[STR_RESET]
            reset = eval(moduleString + resetArg)
            modelWrap = alg_contain.AlgContain(modelData, update, selfInit, reset)
        except:
            modelWrap = alg_contain.AlgContain(modelData, update, selfInit)
        return modelWrap


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
