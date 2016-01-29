
#Import some architectural stuff that we will probably always use
import sys, os, ast
#Point the path to the module storage area
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../modules')
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import sim_model
import sys_model_task
import MessagingAccess
import types
import numpy
import array
import xml.etree.ElementTree as ET
import inspect

class ProcessBaseClass:
 def __init__(self, procName):
    self.Name = procName
    self.processData = sim_model.SysProcess(procName)
 def addTask(self, newTask):
    self.processData.addNewTask(newTask.TaskData)
 def addInterfaceRef(self, newInt):
    self.processData.addInterfaceRef(newInt)
 def discoverAllMessages(self):
    self.processData.discoverAllMessages()
 def disableAllTasks(self):
    self.processData.disableAllTasks()
 def enableAllTasks(self):
    self.processData.enableAllTasks()

class TaskBaseClass:
 def __init__(self, TaskName, TaskRate, InputDelay = 0, FirstStart=0):
   self.Name = TaskName
   self.TaskData = sys_model_task.SysModelTask(TaskRate, InputDelay,
      FirstStart)
   self.TaskModels = []
 def disable(self):
     self.TaskData.disableTask();
 def enable(self):
     self.TaskData.enableTask();
 def updatePeriod(self, newPeriod):
     self.TaskData.updatePeriod(newPeriod)
 def resetTask(self):
     self.TaskData.ResetTaskList()

class LogBaseClass:
 def __init__(self, ReplaceName, LogPeriod, RefFunction, DataCols = 1):
   self.Period = LogPeriod
   self.Name = ReplaceName
   self.PrevLogTime = None
   self.PrevValue = None
   self.TimeValuePairs = array.array('d')
   self.ArrayDim = DataCols+1
   self.CallableFunction = RefFunction
 def clearItem(self):
   self.TimeValuePairs = array.array('d')
   self.PrevLogTime = None
   self.PrevValue = None
 
class EventHandlerClass:
  def __init__(self, eventName, eventRate = int(1E9),  eventActive = False,
                conditionList = [], actionList = []):
    self.eventName = eventName
    self.eventActive = eventActive
    self.eventRate = eventRate
    self.conditionList = conditionList
    self.actionList = actionList
    self.occurCounter = 0
    self.prevTime = -1
    self.methodCall = None
  def methodizeEvent(self):
     if(self.methodCall != None):
        return
     funcString = 'def EVENT_check_' + self.eventName + '(self):\n'
     funcString += '    if('
     for condValue in self.conditionList:
         funcString += ' ' + condValue + ' and'
         funcString = funcString[:-3] + '):\n'
     for actionValue in self.actionList:
         funcString += '        ';
         funcString += actionValue + '\n'
     funcString += '        return 1\n'
     funcString += '    return 0'
     exec(funcString)
     self.methodCall = eval('EVENT_check_' + self.eventName)
  def checkEvent(self, parentSim):
    if(self.eventActive == False):
        return
    if(self.prevTime < 0 or parentSim.TotalSim.CurrentNanos - self.prevTime >= self.eventRate):
        eventCount = self.methodCall(parentSim)
        self.prevTime = parentSim.TotalSim.CurrentNanos
        if eventCount > 0:
            self.eventActive = False
            self.occurCounter += 1


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
     if(self.structPopulated == True):
         return
     xmlFileUse = xmlSearchPath + '/' + self.strName + '.xml'
     try:
          xmlData = ET.parse(xmlFileUse)
     except:
          print "Failed to parse the XML structure for: " + self.strName
          print "This file does not exist most likely: " + xmlFileUse
          return
     root = xmlData.getroot()
     validElement = root.find("./compounddef[@id='" + self.strName + "']")
     for newVariable in validElement.findall(".//memberdef[@kind='variable']"):
        typeUse = newVariable.find('type').text if newVariable.find('type') != None else \
            None
        nameUse = newVariable.find('name').text if newVariable.find('type') != None else \
            None
        argstringUse = newVariable.find('argsstring').text if newVariable.find('argsstring') != None else \
            None
        descUse = newVariable.find('./detaileddescription/para').text if newVariable.find('./detaileddescription/para') != None else \
            None
        if(descUse == None):
            descUse = newVariable.find('./briefdescription/para').text if newVariable.find('./briefdescription/para') != None else \
            None
        newElement = StructDocData.StructElementDef(typeUse, nameUse, argstringUse, descUse)
        self.structElements.update({nameUse: newElement})
        self.structPopulated = True
 def printElem(self):
     print "    " + self.strName + " Structure Elements:"
     for key, value in self.structElements.iteritems():
         outputString = ''
         outputString += value.type + " " + value.name
         outputString += value.argstring if value.argstring != None else ''
         outputString += ': ' + value.desc if value.desc != None else ''
         print "      " + outputString


class SimBaseClass:
 def __init__(self):
   self.TotalSim = sim_model.SimModel()
   self.TaskList = []
   self.procList = []
   self.StopTime = 0
   self.NameReplace = {}
   self.VarLogList = {}
   self.eventMap = {}
   self.simModules = set()
   self.simBasePath = os.path.dirname(os.path.realpath(__file__)) + '/../'
   self.dataStructIndex = self.simBasePath+'/xml/index.xml'
   self.indexParsed = False

 def AddModelToTask(self, TaskName, NewModel, ModelData = None):
   i=0
   for Task in self.TaskList:
       if Task.Name == TaskName:
          Task.TaskData.AddNewObject(NewModel)
          TaskReplaceTag = 'self.TaskList['+str(i) + ']'
          TaskReplaceTag += '.TaskModels[' + str(len(Task.TaskModels)) + ']'
          self.NameReplace[NewModel.ModelTag] = TaskReplaceTag
          if(ModelData != None):
             Task.TaskModels.append(ModelData)
             self.simModules.add(inspect.getmodule(ModelData))
          else:
             Task.TaskModels.append(NewModel)
             self.simModules.add(inspect.getmodule(NewModel))
          return
       i+=1
   print "Could not find a Task with name: %(TaskName)s"  % \
      {"TaskName": TaskName}

 def CreateNewProcess(self, procName):
    proc = ProcessBaseClass(procName)
    self.procList.append(proc)
    self.TotalSim.addNewProcess(proc.processData)
    return proc

 def CreateNewTask(self, TaskName, TaskRate, InputDelay=0, FirstStart=0):
   Task = TaskBaseClass(TaskName, TaskRate, InputDelay, FirstStart)
   self.TaskList.append(Task)
   return Task

 def AddVectorForLogging(self, VarName, VarType, StartIndex, StopIndex=0, LogPeriod=0):
   SplitName = VarName.split('.')
   Subname = '.'
   Subname = Subname.join(SplitName[1:])
   NoDotName = ''
   NoDotName = NoDotName.join(SplitName)
   NoDotName = NoDotName.translate(None, '[]')
   if SplitName[0] in self.NameReplace:
      LogName = self.NameReplace[SplitName[0]] + '.' + Subname
      if(LogName in self.VarLogList):
         return
      if(type(eval(LogName)).__name__ == 'SwigPyObject'):
         RefFunctionString = 'def Get' + NoDotName + '(self):\n'
         RefFunctionString += '   return ['
         LoopTerminate = False
         i=0
         while not LoopTerminate:
            RefFunctionString += 'sim_model.' + VarType + 'Array_getitem('
            RefFunctionString += LogName + ', ' + str(StartIndex + i) + '),'
            i+=1
            if(i > StopIndex-StartIndex):
               LoopTerminate = True
      else:
         RefFunctionString = 'def Get' + NoDotName + '(self):\n'
         RefFunctionString += '   return ['
         LoopTerminate = False
         i=0
         while not LoopTerminate:
            RefFunctionString += LogName + '[' +str(StartIndex+i) +'],'
            i+=1
            if(i > StopIndex-StartIndex):
               LoopTerminate = True
      RefFunctionString = RefFunctionString[:-1] + ']'
      exec(RefFunctionString)
      methodHandle = eval('Get' + NoDotName)
      self.VarLogList[VarName] = LogBaseClass(LogName, LogPeriod,
         methodHandle, StopIndex - StartIndex+1)
   else:
      print "Could not find a structure that has the ModelTag: %(ModName)s" % \
         {"ModName": SplitName[0]}

 def AddVariableForLogging(self, VarName, LogPeriod = 0):
   i=0
   SplitName = VarName.split('.')
   Subname = '.'
   Subname = Subname.join(SplitName[1:])
   NoDotName = ''
   NoDotName = NoDotName.join(SplitName)
   NoDotName = NoDotName.translate(None, '[]')
   if SplitName[0] in self.NameReplace:
      LogName = self.NameReplace[SplitName[0]] + '.' + Subname
      if(LogName not in self.VarLogList):
         RefFunctionString = 'def Get' + NoDotName + '(self):\n'
         RefFunctionString += '   return '+ LogName
         exec(RefFunctionString)
         methodHandle = eval('Get' + NoDotName)
         self.VarLogList[VarName] = LogBaseClass(LogName, LogPeriod, 
            methodHandle )
   else:
      print "Could not find a structure that has the ModelTag: %(ModName)s" % \
         {"ModName": SplitName[0]}

 def ResetTask(self, taskName):
     for Task in self.TaskList:
       if Task.Name == taskName:
           Task.resetTask()
 def InitializeSimulation(self):
   self.TotalSim.ResetSimulation()
   self.TotalSim.InitSimulation()
   for LogItem, LogValue in self.VarLogList.iteritems():
      LogValue.clearItem()

 def ConfigureStopTime(self, TimeStop):
   self.StopTime = TimeStop

 def RecordLogVars(self):
   CurrSimTime = self.TotalSim.CurrentNanos;
   for LogItem, LogValue in self.VarLogList.iteritems():
      LocalPrev = LogValue.PrevLogTime
      if(LocalPrev != None and (CurrSimTime -
         LocalPrev) < LogValue.Period):
         continue
      CurrentVal = LogValue.CallableFunction(self)
      LocalTimeVal = LogValue.TimeValuePairs
      if(LocalPrev != CurrentVal):
         LocalTimeVal.append(CurrSimTime)
         if(isinstance(CurrentVal, (list, tuple))):
            for Value in CurrentVal:
               LocalTimeVal.append(Value)
         else:
            LocalTimeVal.append(CurrentVal)
         LogValue.PrevLogTime = CurrSimTime
         LogValue.PrevValue = CurrentVal
  
 def ExecuteSimulation(self):
   self.initializeEventChecks()
   while(self.TotalSim.CurrentNanos < self.StopTime):
      self.checkEvents()
      self.TotalSim.SingleStepProcesses()
      self.RecordLogVars()

 def GetLogVariableData(self, LogName):
   TheArray = numpy.array(self.VarLogList[LogName].TimeValuePairs)
   ArrayDim = self.VarLogList[LogName].ArrayDim
   TheArray = numpy.reshape(TheArray, (TheArray.shape[0]/ArrayDim, ArrayDim))
   return TheArray

 def disableTask(self, TaskName):
     for Task in self.TaskList:
       if Task.Name == TaskName:
           Task.disable()

 def enableTask(self, TaskName):
     for Task in self.TaskList:
       if Task.Name == TaskName:
           Task.enable()

 def updateTaskPeriod(self, TaskName, newPeriod):
     for Task in self.TaskList:
         if Task.Name == TaskName:
             Task.updatePeriod(newPeriod)

 def parseDataIndex(self):
    self.dataStructureDictionary = {}
    try:
        xmlData = ET.parse(self.dataStructIndex)
    except:
        print "Failed to parse the XML index.  Likely that it isn't present"
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
        print message

    if(exactMessage == searchString):
        searchComplete = True
        headerData = sim_model.MessageHeaderData()
        self.TotalSim.populateMessageHeader(exactMessage, headerData)
        print headerData.MessageName + ": " + headerData.messageStruct
        if self.indexParsed == False:
            self.parseDataIndex()
        if headerData.messageStruct in self.dataStructureDictionary:
            xmlDataPath = os.path.dirname(self.dataStructIndex)
            self.dataStructureDictionary[
            headerData.messageStruct].populateElem(xmlDataPath)
            self.dataStructureDictionary[
             headerData.messageStruct].printElem()
    return searchComplete

 def pullMessageLogData(self, varName, indices = [], numRecords = -1):
    splitName = varName.split('.')
    messageID = self.TotalSim.getMessageID(splitName[0])
    if not(messageID.itemFound):
        print "Failed to pull log due to invalid ID for this message: " + splitName[0]
        return []
    headerData = sim_model.MessageHeaderData()
    self.TotalSim.populateMessageHeader(splitName[0], headerData)
    moduleFound = ''
    for moduleData in self.simModules:
        if moduleFound != '':
            break
        for name, obj in inspect.getmembers(moduleData):
            if inspect.isclass(obj):
                if obj.__name__ == headerData.messageStruct:
                   moduleFound = moduleData.__name__
                   break
    if moduleFound == '':
        print "Failed to find valid message structure for: " + headerData.messageStruct
        return []
    messageCount = self.TotalSim.messageLogs.getLogCount(messageID.processBuffer, messageID.itemID)
    resplit = varName.split(splitName[0] + '.')
    bufferUse = sim_model.logBuffer if messageCount > 0 else sim_model.messageBuffer
    messageCount = messageCount if messageCount > 0 else headerData.UpdateCounter
    messageCount = messageCount if numRecords < 0 else numRecords
    if(len(indices) <= 0):
        indices_use = [0]
    else:
        indices_use = indices
    
    dataUse = MessagingAccess.obtainMessageVector(splitName[0], moduleFound,
        headerData.messageStruct, messageCount, self.TotalSim, resplit[1], 'double',
        indices_use[0], indices_use[-1], bufferUse)

    indicesLocal = [0]
    if(len(dataUse) == 0):
       return dataUse
    for indexUse in indices_use:
        indicesLocal.append(indexUse+1)
    return(dataUse[:, indicesLocal])
    
 def createNewEvent(self, eventName, eventRate = int(1E9), eventActive = False,
                    conditionList = [], actionList = []):
    if(eventName in self.eventMap.keys()):
       return
    newEvent = EventHandlerClass(eventName, eventRate, eventActive,
                            conditionList, actionList)
    self.eventMap.update({eventName: newEvent})

 def initializeEventChecks(self):
    self.eventList = []
    for key, value in self.eventMap.iteritems():
        value.methodizeEvent()
        self.eventList.append(value)
 def checkEvents(self):
    for localEvent in self.eventList:
        localEvent.checkEvent(self)
 def setEventActivity(self, eventName, activityCommand):
    if eventName not in self.eventMap.keys():
        print "You asked me to set the status of an event that I don't have."
        return
    self.eventMap[eventName].eventActive = activityCommand

def SetCArray(InputList, VarType, ArrayPointer):
   CmdString = 'sim_model.' + VarType + 'Array_setitem(ArrayPointer, CurrIndex, CurrElem)'
   CurrIndex = 0
   for CurrElem in InputList:
      exec(CmdString)
      CurrIndex += 1
   

