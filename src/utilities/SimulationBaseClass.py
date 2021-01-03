
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
import sys, os, ast
import six

# Point the path to the module storage area


from Basilisk.architecture import sim_model
from Basilisk.architecture import alg_contain
from Basilisk.utilities import MessagingAccess
import numpy as np
import array
import xml.etree.ElementTree as ET
import inspect
import threading
from time import sleep
try:
   set
except NameError:
   from sets import Set as set

from Basilisk.utilities import simulationArchTypes
from Basilisk.architecture import bskLogging
from Basilisk.utilities.simulationProgessBar import SimulationProgressBar
import warnings


class LogBaseClass:
    def __init__(self, ReplaceName, LogPeriod, RefFunction, DataCols=1):
        self.Period = LogPeriod
        self.Name = ReplaceName
        self.PrevLogTime = None
        self.PrevValue = None
        self.TimeValuePairs = array.array('d')
        self.ArrayDim = DataCols + 1
        self.CallableFunction = RefFunction

    def clearItem(self):
        self.TimeValuePairs = array.array('d')
        self.PrevLogTime = None
        self.PrevValue = None


class EventHandlerClass:
    def __init__(self, eventName, eventRate=int(1E9), eventActive=False,
                 conditionList=[], actionList=[]):
        self.eventName = eventName
        self.eventActive = eventActive
        self.eventRate = eventRate
        self.conditionList = conditionList
        self.actionList = actionList
        self.occurCounter = 0
        self.prevTime = -1
        self.checkCall = None
        self.operateCall = None

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
        return(nextTime)


class StructDocData:
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
    def __init__(self):
        self.TotalSim = sim_model.SimModel()
        self.TotalSim.terminateSimulation()
        self.TaskList = []
        self.procList = []
        self.pyProcList = []
        self.StopTime = 0
        self.nextEventTime = 0
        self.NameReplace = {}
        self.VarLogList = {}
        self.eventMap = {}
        self.simModules = set()
        self.simBasePath = os.path.dirname(os.path.realpath(__file__)) + '/../'
        self.dataStructIndex = self.simBasePath + '/xml/index.xml'
        self.indexParsed = False
        self.simulationInitialized = False
        self.simulationFinished = False
        self.bskLogger = bskLogging.BSKLogger()
        self.showProgressBar = False
        self.allModules = set()

    def SetProgressBar(self, value):
        self.showProgressBar = value

    def AddModelToTask(self, TaskName, NewModel, ModelData=None, ModelPriority=-1):
        """
        This function is responsible for passing on the logger to a module instance (model), adding the
        model to a particular task, and defining
        the order/priority that the model gets updated within the task.
        :param TaskName (str): Name of the task
        :param NewModel (obj): Model to add to the task
        :param ModelData: None or struct containing
        :param ModelPriority (int): Priority that determines when the model gets updated. (Higher number = Higher priority)
        :return:
        """
        i = 0
        for Task in self.TaskList:
            if Task.Name == TaskName:
                Task.TaskData.AddNewObject(NewModel, ModelPriority)
                TaskReplaceTag = 'self.TaskList[' + str(i) + ']'
                TaskReplaceTag += '.TaskModels[' + str(len(Task.TaskModels)) + ']'
                self.NameReplace[TaskReplaceTag] = NewModel.ModelTag
                if (ModelData != None):
                    try:
                        ModelData.bskLogger = self.bskLogger
                    except:
                        pass
                    Task.TaskModels.append(ModelData)
                    self.simModules.add(inspect.getmodule(ModelData))
                else:
                    try:
                        NewModel.bskLogger = self.bskLogger
                    except:
                        pass
                    Task.TaskModels.append(NewModel)
                    self.simModules.add(inspect.getmodule(NewModel))
                return
            i += 1
        print("Could not find a Task with name: %(TaskName)s" % \
              {"TaskName": TaskName})

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

    def CreateNewPythonProcess(self, procName, priority = -1):
        """
        Creates the python analog of a sim-level process, that exists only on the python level in self.pyProcList
        :param procName (str): Name of process
        :param priority (int): Priority that determines when the model gets updated. (Higher number = Higher priority)
        :return: simulationArchTypes.PythonProcessClass object
        """
        proc = simulationArchTypes.PythonProcessClass(procName, priority)
        i=0;
        for procLoc in self.pyProcList:
            if priority > procLoc.pyProcPriority:
                self.pyProcList.insert(i, proc)
                return proc
            i+=1
        self.pyProcList.append(proc)
        return proc

    def CreateNewTask(self, TaskName, TaskRate, InputDelay=0, FirstStart=0):
        """
        Creates a simulation task on the C-level with a specific update-frequency (TaskRate), an optional delay, and
        an optional start time.
        :param TaskName (str): Name of Task
        :param TaskRate (int): Number of nanoseconds to elapse before update() is called
        :param InputDelay (int): Number of nanoseconds simulating a lag of the particular task# TODO: Check that this is [ns]
        :param FirstStart (int): Number of nanoseconds to elapse before task is officially enabled
        :return: simulationArchTypes.TaskBaseClass object
        """
        Task = simulationArchTypes.TaskBaseClass(TaskName, TaskRate, InputDelay, FirstStart)
        self.TaskList.append(Task)
        return Task

    def AddVariableForLogging(self, VarName, LogPeriod=0, StartIndex=0, StopIndex=0, VarType=None):
        """
        Informs the sim to log a particular variable within a message
        :param VarName:
        :param LogPeriod:
        :param StartIndex:
        :param StopIndex:
        :param VarType:
        :return:
        """
        SplitName = VarName.split('.')
        Subname = '.'
        Subname = Subname.join(SplitName[1:])
        NoDotName = ''
        NoDotName = NoDotName.join(SplitName)
        if six.PY2:
            NoDotName = NoDotName.translate(None, "[]'()")
        else:
            tr = str.maketrans("","", "[]'()")
            NoDotName = NoDotName.translate(tr)
            #NoDotName = NoDotName.translate({ord(c): None for c in "[]'()"})
        inv_map = {v: k for k, v in list(self.NameReplace.items())}
        if SplitName[0] in inv_map:
            LogName = inv_map[SplitName[0]] + '.' + Subname
            if (LogName in self.VarLogList):
                return
            if (type(eval(LogName)).__name__ == 'SwigPyObject'):
                RefFunctionString = 'def Get' + NoDotName + '(self):\n'
                RefFunctionString += '   return ['
                LoopTerminate = False
                i = 0
                while not LoopTerminate:
                    RefFunctionString += 'sim_model.' + VarType + 'Array_getitem('
                    RefFunctionString += LogName + ', ' + str(StartIndex + i) + '),'
                    i += 1
                    if (i > StopIndex - StartIndex):
                        LoopTerminate = True
                RefFunctionString = RefFunctionString[:-1] + ']'
                exec (RefFunctionString)
                methodHandle = eval('Get' + NoDotName)

            elif (type(eval(LogName)).__name__ == 'list'):
                RefFunctionString = 'def Get' + NoDotName + '(self):\n'
                RefFunctionString += '   if isinstance(' + LogName + '[0], list):\n'
                RefFunctionString += '      localList = sum(' + LogName + ',[])\n'
                RefFunctionString += '   else:\n      localList = ' + LogName + '\n'
                RefFunctionString += '   return ['
                LoopTerminate = False
                i = 0
                while not LoopTerminate:
                    RefFunctionString +=  'localList[' + str(StartIndex + i) + '],'
                    i += 1
                    if (i > StopIndex - StartIndex):
                        LoopTerminate = True
                RefFunctionString = RefFunctionString[:-1] + ']'
                exec (RefFunctionString)
                methodHandle = eval('Get' + NoDotName)
            else:
                RefFunctionString = 'def Get' + NoDotName + '(self):\n'
                RefFunctionString += '   return ' + LogName
                exec (RefFunctionString)
                methodHandle = eval('Get' + NoDotName)
            self.VarLogList[VarName] = LogBaseClass(LogName, LogPeriod,
                                                    methodHandle, StopIndex - StartIndex + 1)
        else:
            print("Could not find a structure that has the ModelTag: %(ModName)s" % \
                  {"ModName": SplitName[0]})

    def ResetTask(self, taskName):
        for Task in self.TaskList:
            if Task.Name == taskName:
                Task.resetTask(self.TotalSim.CurrentNanos)

    def InitializeSimulation(self):
        self.TotalSim.ResetSimulation()
        self.TotalSim.selfInitSimulation()
        for proc in self.pyProcList:
            proc.selfInitProcess()
        self.TotalSim.crossInitSimulation()
        for proc in self.pyProcList:
            proc.crossInitProcess()
        self.TotalSim.resetInitSimulation()
        for proc in self.pyProcList:
            proc.resetProcess(0)
        for LogItem, LogValue in self.VarLogList.items():
            LogValue.clearItem()
        self.simulationInitialized = True


    def ConfigureStopTime(self, TimeStop):
        self.StopTime = TimeStop

    def RecordLogVars(self):
        CurrSimTime = self.TotalSim.CurrentNanos
        minNextTime = -1
        for LogItem, LogValue in self.VarLogList.items():
            LocalPrev = LogValue.PrevLogTime
            if (LocalPrev != None and (CurrSimTime -
                                           LocalPrev) < LogValue.Period):
                if(minNextTime < 0 or LocalPrev + LogValue.Period < minNextTime):
                    minNextTime = LocalPrev + LogValue.Period
                continue
            CurrentVal = LogValue.CallableFunction(self)
            LocalTimeVal = LogValue.TimeValuePairs
            if (LocalPrev != CurrentVal):
                LocalTimeVal.append(CurrSimTime)
                try:
                    temp = (len(CurrentVal))
                    for Value in CurrentVal:
                        LocalTimeVal.append(Value)
                except TypeError:
                    LocalTimeVal.append(CurrentVal)
                LogValue.PrevLogTime = CurrSimTime
                LogValue.PrevValue = CurrentVal
                if(minNextTime < 0 or CurrSimTime + LogValue.Period < minNextTime):
                    minNextTime = CurrSimTime + LogValue.Period
        return minNextTime

    def ExecuteSimulation(self, showPlots=None, livePlots=None, simComm=None, plottingFunc=None, plotArgs=None):
        self.initializeEventChecks()

        #Live Plotting Thread
        if livePlots:
            shareDataThread = threading.Thread(target=self.shareData, args=(simComm,))
            shareDataThread.daemon = True
            shareDataThread.start()
        nextStopTime = self.TotalSim.NextTaskTime
        nextPriority = -1
        pyProcPresent = False
        if(len(self.pyProcList) > 0):
            nextPriority = self.pyProcList[0].pyProcPriority
            pyProcPresent = True
            nextStopTime = self.pyProcList[0].nextCallTime()
        progressBar = SimulationProgressBar(self.StopTime, self.showProgressBar)
        while (self.TotalSim.NextTaskTime <= self.StopTime):
            if(self.nextEventTime <= self.TotalSim.CurrentNanos and self.nextEventTime >= 0):
                self.nextEventTime = self.checkEvents()
                self.nextEventTime = self.nextEventTime if self.nextEventTime >= self.TotalSim.NextTaskTime else self.TotalSim.NextTaskTime
            if(self.nextEventTime >= 0 and self.nextEventTime < nextStopTime):
                nextStopTime = self.nextEventTime
                nextPriority = -1
            self.TotalSim.StepUntilStop(nextStopTime, nextPriority)
            progressBar.update(self.TotalSim.NextTaskTime)
            nextPriority = -1
            nextStopTime = self.StopTime
            nextLogTime = self.RecordLogVars()
            procStopTimes = []
            for pyProc in self.pyProcList:
                nextCallTime = pyProc.nextCallTime()
                if(nextCallTime<=self.TotalSim.CurrentNanos):
                    pyProc.executeTaskList(self.TotalSim.CurrentNanos)
                nextCallTime = pyProc.nextCallTime()
                procStopTimes.append(nextCallTime)

            if(pyProcPresent and nextStopTime >= min(procStopTimes)):
                nextStopTime = min(procStopTimes)
                nextPriority = self.pyProcList[procStopTimes.index(nextStopTime)].pyProcPriority
            if(nextLogTime >= 0 and nextLogTime < nextStopTime):
                nextStopTime = nextLogTime
                nextPriority = -1
            nextStopTime = nextStopTime if nextStopTime >= self.TotalSim.NextTaskTime else self.TotalSim.NextTaskTime
        progressBar.markComplete()
        progressBar.close()
        if simComm is not None:
            simComm.send("TERM")
        if showPlots:
            plottingFunc(*plotArgs)

    def shareData(self, simComm):
        while True:
            inComm = simComm.recv()
            outData = []
            outComm = {}
            for req in inComm["dataReq"]:
                outData.append(self.pullMessageLogData(req, list(range(3))))
            if len(outData) != 0:
                outComm["plotID"] = inComm["plotID"]; outComm["dataResp"] = outData; outComm["plotFun"] = inComm["plotFun"]
                simComm.send(outComm)

    def shareDataTest(self, simComm):
        while True:
            inComm = simComm.recv()
            outData = []
            outComm = {}
            for req in inComm["dataReq"]:
                outData.append(self.pullMessageLogData(req, list(range(3))))
            if len(outData) != 0:
                outComm["dataResp"] = outData
                simComm.send(outComm)

    def GetLogVariableData(self, LogName):
        TheArray = np.array(self.VarLogList[LogName].TimeValuePairs)
        ArrayDim = self.VarLogList[LogName].ArrayDim
        TheArray = np.reshape(TheArray, (TheArray.shape[0] // ArrayDim, ArrayDim))
        return TheArray

    def disableTask(self, TaskName):
        for Task in self.TaskList:
            if Task.Name == TaskName:
                Task.disable()

    def enableTask(self, TaskName):
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

    def findSimulationMessage(self, searchString):
        searchComplete = False
        msgMatchList = MessagingAccess.findMessageMatches(searchString,
                                                          self.TotalSim)
        exactMessage = -1
        for message in msgMatchList:
            if message == searchString:
                exactMessage = message
                continue
            print(message)

        if (exactMessage == searchString):
            searchComplete = True
            headerData = sim_model.MessageHeaderData()
            self.TotalSim.populateMessageHeader(exactMessage, headerData)
            print(headerData.MessageName + ": " + headerData.messageStruct)
            if self.indexParsed == False:
                self.parseDataIndex()
            if headerData.messageStruct in self.dataStructureDictionary:
                xmlDataPath = os.path.dirname(self.dataStructIndex)
                self.dataStructureDictionary[
                    headerData.messageStruct].populateElem(xmlDataPath)
                self.dataStructureDictionary[
                    headerData.messageStruct].printElem()
        return searchComplete

    def pullMessageLogData(self, varName, indices=[], numRecords=-1, varType='double'):
        """
        Retrieves logged message data from TotalSim by querying over message and variable names.

        :param varName: string: A message name with the variable specified (i.e., 'scMessage.r_BN_N')
        :param indices: list: For multidimensional message attributes, this specifies how many columns to pull from (ex. a 3 vector should be called with list(range(3)))
        :param numRecords: The number of messages to pull from the history. Defaults to -1, which pulls all logged message values.
        """
        splitName = varName.split('.')
        messageID = self.TotalSim.getMessageID(splitName[0])
        if not (messageID.itemFound):
            print("Failed to pull log due to invalid ID for this message: " + splitName[0])
            return []
        headerData = sim_model.MessageHeaderData()
        self.TotalSim.populateMessageHeader(splitName[0], headerData)
        moduleFound = ''

        if len(self.allModules) == 0:
            # Create a new set into which we add the SWIG'd simMessages definitions
            # and union it with the simulation's modules set. We do this so that
            # python modules can have message structs resolved
            self.allModules = set()
            self.allModules.add(simMessages)
            self.allModules = self.allModules | self.simModules

        #   Search for the specific module
        for moduleData in self.allModules:
            if moduleFound != '':
                break
            for name, obj in inspect.getmembers(moduleData):
                if inspect.isclass(obj):
                    if obj.__name__ == headerData.messageStruct:
                        moduleFound = moduleData.__name__
                        break
        if moduleFound == '':
            print("Failed to find valid message structure for: " + headerData.messageStruct)
            return []
        messageCount = self.TotalSim.messageLogs.getLogCount(messageID.processBuffer, messageID.itemID)
        resplit = varName.split(splitName[0] + '.')
        bufferUse = sim_model.logBuffer if messageCount > 0 else sim_model.messageBuffer
        maxCountMessager = headerData.UpdateCounter if headerData.UpdateCounter < headerData.MaxNumberBuffers else headerData.MaxNumberBuffers
        messageCount = messageCount if messageCount > 0 else maxCountMessager
        messageCount = messageCount if numRecords < 0 else numRecords
        if (len(indices) <= 0):
            indices_use = [0]
        else:
            indices_use = indices

        dataUse = MessagingAccess.obtainMessageVector(splitName[0], moduleFound,
                                                      headerData.messageStruct, messageCount, self.TotalSim, resplit[1],
                                                      varType,
                                                      indices_use[0], indices_use[-1], bufferUse)

        indicesLocal = [0]
        if (len(dataUse) == 0):
            return dataUse
        for indexUse in indices_use:
            indicesLocal.append(indexUse + 1)
        return (dataUse[:, indicesLocal])

    def pullMultiMessageLogData(self, varNames, indices, types, numRecords = -1):
        """
        Pulls the log data from multiple messages at the same time. When pulling multiple messages, or start/stopping the sim often,
        produces faster performance than pullMessageLogData.

        Inputs:
        @param varNames: list : list of message names and parameters.
        @param indices: list : list of message indices to be pulled.
        @param types: list: list of strings describing types of data to be pulled (i.e. "bool" or "double").
        @param numRecords : int : number of logged messages to pull. Defaults to -1, which returns all logged messages.

        Outputs:
        @param pullDict : dict : dict using names from varNames as keys and the corresponding pulled message data as the values.
        """

        attributeDict = {}
        structToName = {}
        indexDict = {}
        typeDict = {}
        headerDict = {}
        foundDict = {}
        pullDict = {}

        #   Create entrices in header/index/id/foundDict corresponding to each varName
        for idx, varName in enumerate(varNames):
            #   Work out the message, attribute names:
            splitName = varName.split('.')
            msgName = splitName[0]
            msgAttribute = splitName[1]

            if msgName in attributeDict.keys():
                attributeDict[msgName].append(msgAttribute)
            else:
                attributeDict[msgName] = [msgAttribute]

            #   Find some identifying information we'll need to pull it
            messageID = self.TotalSim.getMessageID(splitName[0])
            if not (messageID.itemFound):
                print("Failed to pull log due to invalid ID for this message: " + splitName[0])
                return []
            #   Populate a header for the message
            headerData = sim_model.MessageHeaderData()
            self.TotalSim.populateMessageHeader(splitName[0], headerData)

            if headerData.messageStruct in headerDict.keys():
                headerDict[headerData.messageStruct].append(headerData)
                structToName[headerData.messageStruct].append(msgName)
            else:
                headerDict[headerData.messageStruct] = [headerData]
                structToName[headerData.messageStruct] = [msgName]

            foundDict[varName] = False
            indexDict[varName] = indices[idx]
            typeDict[varName] = types[idx]


        headerList = list(headerDict.keys())

        if len(self.allModules) == 0:
            # Create a new set into which we add the SWIG'd simMessages definitions
            # and union it with the simulation's modules set. We do this so that
            # python modules can have message structs resolved
            self.allModules = set()
            self.allModules.add(simMessages)
            self.allModules = self.allModules | self.simModules

        #   Search for any of the modules in the list:
        for module in self.allModules:
            #   If all the values are "true", we found everything. Kill it
            if all(value == True for value in foundDict.values()):
                break
            for name, obj in inspect.getmembers(module):
                if inspect.isclass(obj):
                    #   Check to see if the module has a
                    if obj.__name__ in headerList:
                        #   Get all the local variables corresponding to this module, since we aren't garunteed what order
                        #   we'll find this header name:
                        structName = obj.__name__
                        moduleFound = module.__name__
                        msgNames = structToName[structName]
                        headerDatas = headerDict[obj.__name__]

                        for idx,msgName in enumerate(msgNames):
                            headerData = headerDatas[idx]
                            msgAttributes = attributeDict[msgName]
                            messageID = self.TotalSim.getMessageID(msgName)
                            messageCount = self.TotalSim.messageLogs.getLogCount(messageID.processBuffer, messageID.itemID)
                            bufferUse = sim_model.logBuffer if messageCount > 0 else sim_model.messageBuffer

                            maxCountMessager = headerData.UpdateCounter if headerData.UpdateCounter < headerData.MaxNumberBuffers else headerData.MaxNumberBuffers
                            messageCount = messageCount if messageCount > 0 else maxCountMessager
                            messageCount = messageCount if numRecords < 0 else numRecords

                            #   Opportunistically grab all the attributes we can before we clear this message struct:
                            for attr in msgAttributes:
                                indices = indexDict[msgName+'.'+attr]
                                type = typeDict[msgName+'.'+attr]
                                if (len(indices) <= 0):
                                    indices_use = [0]
                                else:
                                    indices_use = indices

                                dataUse = MessagingAccess.obtainMessageVector(msgName, moduleFound,
                                                                              structName, messageCount,
                                                                              self.TotalSim,
                                                                              attr,
                                                                              type,
                                                                              indices_use[0], indices_use[-1], bufferUse)
                                pullDict.update({msgName+'.'+attr:dataUse})

                                foundDict.update({msgName+'.'+attr:True})

        return pullDict

    def createNewEvent(self, eventName, eventRate=int(1E9), eventActive=False,
                       conditionList=[], actionList=[]):
        if (eventName in list(self.eventMap.keys())):
            return
        newEvent = EventHandlerClass(eventName, eventRate, eventActive,
                                     conditionList, actionList)
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
    def terminateSimulation(self):
        self.TotalSim.terminateSimulation()


    # Only instances of following three functions is within SimBaseClass
    # TODO: Investigate origin and need for these functions
    def findMessagePairs(self, messageName, processList):
        dataPairs = self.TotalSim.getMessageExchangeData(messageName, processList)
        outputDataPairs = []
        for messagePair in dataPairs:
            namePairs = [None, None]
            for proc in self.procList:
                for intDef in proc.processData.intRefs:
                    for singInt in intDef.interfaceDef:
                        for i in range(2):
                            if singInt.moduleID == messagePair[i]:
                                if singInt.ModelTag != "":
                                    namePairs[i] = singInt.ModelTag
                                else:
                                    namePairs[i] = singInt.moduleID
            for task in self.TaskList:
                for module in task.TaskData.TaskModels:
                    for i in range(2):
                        if module.ModelPtr.moduleID == messagePair[i]:
                            if namePairs[i] == None:
                                if module.ModelPtr.ModelTag != "":
                                    namePairs[i] = module.ModelPtr.ModelTag
                                else:
                                    namePairs[i] = module.ModelPtr.moduleID
            outputDataPairs.append(namePairs)
        return(outputDataPairs)
    def getDataMap(self, processList):
        mapDict = {}
        messNames = self.TotalSim.getUniqueMessageNames()
        for name in messNames:
            dataPairs = self.findMessagePairs(name, processList)
            for transPair in dataPairs:
                if transPair[0] in mapDict:
                    if name not in mapDict[transPair[0]].outputMessages and name is not None:
                        mapDict[transPair[0]].outputDict[name] = [transPair[1]]
                    elif name is not None:
                        mapDict[transPair[0]].outputDict[name].append(transPair[1])
                    mapDict[transPair[0]].outputMessages.add(name)
                else:
                    newElem = DataPairClass()
                    newElem.outputMessages.add(name)
                    newElem.name = transPair[0]
                    newElem.outputDict[name] = [transPair[1]]
                    mapDict[transPair[0]] = newElem
                if transPair[1] in mapDict:
                    mapDict[transPair[1]].inputMessages.add(name)
                else:
                    newElem = DataPairClass()
                    newElem.inputMessages.add(name)
                    newElem.name = transPair[1]
                    mapDict[transPair[1]] = newElem
        return(mapDict)
    def writeDataMapDot(self, processList = [], outputFileName='SimDataMap.dot'):
        fDesc = open(outputFileName, 'w')
        messageDataMap = self.getDataMap(processList)
        fDesc.write('digraph messages {\n')
        fDesc.write('node [shape=record];\n')
        for key, value in messageDataMap.items():
            if(str(key) == 'None'):
                continue
            fDesc.write('    ' + str(key))
            fDesc.write('[shape=record,label="{')
            i=1
            for input in value.inputMessages:
                fDesc.write('{<' + input + 'In> ' + input + '}')
                if i < len(value.inputMessages):
                    fDesc.write(' | ')
                i += 1
            fDesc.write('} | ' + str(key) + ' | {')
            i=1
            for output in value.outputMessages:
                fDesc.write('{<' + output + 'Out> ' + output + '}')
                if i < len(value.outputMessages):
                    fDesc.write(' | ')
                i += 1
            fDesc.write('}"];\n')
            for outputConn, ConnValue in value.outputDict.items():
                for outputModule in ConnValue:
                    if(outputModule == None):
                        continue
                    fDesc.write('    ' + str(key) + ':' + outputConn + 'Out')
                    fDesc.write(' -> ' + str(outputModule) + ':' + outputConn +'In;\n')


        fDesc.write('\n}')
        fDesc.close()

    def setModelDataWrap(self, modelData):
        """
        Takes a module and returns an object that provides access to said module's SelfInit, CrossInit, Update, and Reset
        methods.

        Takes the module instance, collects all SwigPyObjects generated from the .i file (SelfInit, CrossInit,
        Update and Reset), and attaches it to a alg_contain model instance so the modules standard functions can be
        run at the python level.

        :param modelData: model to gather functions for
        :return: An alg_contain model that provides access to the original model's core functions
        """
        algDict = {}
        STR_SELFINIT = 'SelfInit'
        STR_CROSSINIT = 'CrossInit'
        STR_UPDATE = 'Update'
        STR_RESET = 'Reset'

        # SwigPyObject's Parsing:
        # Collect all the SwigPyObjects present in the list. Only the methods SelfInit, CrossInit, Update and Restart
        # are wrapped by Swig in the .i files. Therefore they are the only SwigPyObjects
        def parseDirList(dirList):
            algNames = []
            for methodName in dirList:
                methodObject = eval('sys.modules["' + module + '"].' + methodName)
                if type(methodObject).__name__ == "SwigPyObject":
                    algNames.append(methodName)
            return algNames
        # Check the type of the algorithm, i.e. SelfInit, CrossInit, Update or Reset,
        # and return the key to create a new dictionary D[str_method] = method
        def checkMethodType(methodName):
            if methodName[0:len(STR_SELFINIT)] == STR_SELFINIT:
                return STR_SELFINIT
            elif methodName[0:len(STR_CROSSINIT)] == STR_CROSSINIT:
                return STR_CROSSINIT
            elif methodName[0:len(STR_UPDATE)] == STR_UPDATE:
                return STR_UPDATE
            elif methodName[0:len(STR_RESET)] == STR_RESET:
                return STR_RESET
            else:
                raise ValueError('Cannot recognize the method'
                                 '(I only assess SelfInit, CrossInit, Update and Reset methods). '
                                 'Parse better.')

        module = modelData.__module__
        sysMod = sys.modules[module]
        dirList = dir(sysMod)
        algList = parseDirList(dirList)

        # if the package has different levels we need to access the correct level of the package
        if six.PY2:
            level = -1
        else:
            level = 0
        currMod = __import__(module, globals(), locals(), [], level)

        moduleString = "currMod."
        moduleNames = module.split(".")
        if len(moduleNames) > 1:
            moduleString += ".".join(moduleNames[1:]) + "."

        for alg in algList:
            key = checkMethodType(alg)
            algDict[key] = alg

        update = eval(moduleString + algDict[STR_UPDATE])
        selfInit = eval(moduleString + algDict[STR_SELFINIT])
        crossInit = eval(moduleString + algDict[STR_CROSSINIT])
        try:
            resetArg = algDict[STR_RESET]
            reset = eval(moduleString + resetArg)
            modelWrap = alg_contain.AlgContain(modelData, update, selfInit, crossInit, reset)
        except:
            modelWrap = alg_contain.AlgContain(modelData, update, selfInit, crossInit)
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
