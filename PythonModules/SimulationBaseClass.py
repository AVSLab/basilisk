
#Import some architectural stuff that we will probably always use
import sys, os, ast
#Point the path to the module storage area
sys.path.append(os.environ['SIMULATION_BASE']+'/modules')
import sim_model
import sys_model_thread
import types
import numpy
import gc
import array

class ThreadBaseClass:
 def __init__(self, ThreadName, ThreadRate, InputDelay = 0, FirstStart=0):
   self.Name = ThreadName
   self.ThreadData = sys_model_thread.SysModelThread(ThreadRate, InputDelay, 
      FirstStart)
   self.ThreadModels = []

class LogBaseClass:
 def __init__(self, ReplaceName, LogPeriod, RefFunction, DataCols = 1):
   self.Period = LogPeriod
   self.Name = ReplaceName
   self.PrevLogTime = None
   self.PrevValue = None
   self.TimeValuePairs = array.array('d')
   self.ArrayDim = DataCols+1
   self.CallableFunction = RefFunction

class SimBaseClass:
 def __init__(self):
   self.TotalSim = sim_model.SimModel()
   self.ThreadList = []
   self.StopTime = 0
   self.NameReplace = {}
   self.VarLogList = {}

 def AddModelToThread(self, ThreadName, NewModel):
   i=0
   for Thread in self.ThreadList:
       if Thread.Name == ThreadName:
          Thread.ThreadData.AddNewObject(NewModel)
          ThreadReplaceTag = 'self.ThreadList['+str(i) + ']'
          ThreadReplaceTag += '.ThreadModels[' + str(len(Thread.ThreadModels)) + ']'
          self.NameReplace[NewModel.ModelTag] = ThreadReplaceTag
          Thread.ThreadModels.append(NewModel)
          return
       i+=1
   print "Could not find a Thread with name: %(ThreadName)s"  % \
      {"ThreadName": ThreadName}

 def CreateNewThread(self, ThreadName, ThreadRate, InputDelay=0, FirstStart=0):
   Thread = ThreadBaseClass(ThreadName, ThreadRate, InputDelay, FirstStart)
   self.ThreadList.append(Thread)
   self.TotalSim.AddNewThread(Thread.ThreadData)

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
         RefFunctionString = 'def Get' + NoDotName + '(self, ActSelf):\n'
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
         RefFunctionString = 'def Get' + NoDotName + '(self, ActSelf):\n'
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
         RefFunctionString = 'def Get' + NoDotName + '(self, ActSelf):\n'
         RefFunctionString += '   return '+ LogName
         exec(RefFunctionString)
         methodHandle = eval('Get' + NoDotName)
         self.VarLogList[VarName] = LogBaseClass(LogName, LogPeriod, 
            methodHandle )
   else:
      print "Could not find a structure that has the ModelTag: %(ModName)s" % \
         {"ModName": SplitName[0]}

 def InitializeSimulation(self):
   self.TotalSim.InitThreads()

 def ConfigureStopTime(self, TimeStop):
   self.StopTime = TimeStop

 def RecordLogVars(self):
   for LogItem, LogValue in self.VarLogList.iteritems():
      if(LogValue.PrevLogTime != None and self.TotalSim.CurrentNanos - 
         LogValue.PrevLogTime < LogValue.Period):
         continue
      CurrentVal = LogValue.CallableFunction(self, self)
      if(LogValue.PrevValue != CurrentVal):
         LogValue.TimeValuePairs.append(self.TotalSim.CurrentNanos)
         if(isinstance(CurrentVal, (list, tuple))):
            for Value in CurrentVal:
               LogValue.TimeValuePairs.append(Value)
         else:
            LogValue.TimeValuePairs.append(CurrentVal)
         LogValue.PrevLogTime = self.TotalSim.CurrentNanos
         LogValue.PrevValue = CurrentVal
  
 def ExecuteSimulation(self):
   while(self.TotalSim.CurrentNanos < self.StopTime):
      self.TotalSim.SingleStepNextThread()
      self.RecordLogVars()

 def GetLogVariableData(self, LogName):
   TheArray = numpy.array(self.VarLogList[LogName].TimeValuePairs)
   ArrayDim = self.VarLogList[LogName].ArrayDim
   TheArray = numpy.reshape(TheArray, (TheArray.shape[0]/ArrayDim, ArrayDim))
   return TheArray
