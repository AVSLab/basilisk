import sys
import copy
import sim_model
import numpy
import array
import pdb

def getMessageContainers(MessageName, MessageModule, MessageObj):
   ## Begin Method steps here
   ## - Import the module in question to get access to structures
   LocalObj = __import__(MessageModule, globals(), locals(), [], -1)

   MessageList = []
   ## - Find the structure of the message from python/SWIG
   LocalContainer = eval('LocalObj.' + MessageObj + '()')
   FirstKeys = dir(LocalContainer)
   FilteredKeys = []
   TotalDict = {"MessageTime": [] } ## - MessageTime is time of write for messag
   for LocalKey in FirstKeys:
      ## - Filter out private and the this pointer because we don't need them
      if LocalKey[0] != '_' and LocalKey != 'this':
         FilteredKeys.append(LocalKey)
         TotalDict.update({LocalKey: []})

   return LocalContainer, TotalDict

## This function gets a specified number of messages for MessageName.
#
#  We currently have to get the module and object that we are logging.  
#  We might be able to cut that down in the future as we get to good XML 
#  coverage of our source.
#  @param MessageName Name of the message that we want to get data on
#  @param MessageModule The Name of the python module that contains the struct
#  @param MessageObj The python module that defines the structure of message
#  @param MessageCount The number of messages that we want to pull
#  @param SimContainer The simulation that was run to get the messages
def ObtainMessageList(MessageName, MessageModule, MessageObj, MessageCount, 
  SimContainer, messageType = sim_model.messageBuffer):

   ## Begin Method steps here
   LocalContainer, TotalDict = getMessageContainers(MessageName, MessageModule,
      MessageObj)
   ## - For each message, pull the buffer, and update the keys of the dictionary
   LocalCount = 0
   while(LocalCount < MessageCount):
      WriteTime = SimContainer.GetWriteData(MessageName, sys.getsizeof(LocalContainer), 
         LocalContainer, messageType, LocalCount)
      ## - For each message append time and the message elements to lists
      for Element, Value in TotalDict.items():
         if Element == 'MessageTime':
            LocalVal = WriteTime
         else :
            LocalVal =  eval('LocalContainer.'+Element)
         TotalDict[Element].append(LocalVal)
      LocalCount += 1
   ## - All of the lists are backwards (latest to oldest), reverse for ease of use
   for Element, Value in TotalDict.items():
      Value.reverse()
      Value = numpy.array(Value)
   return(TotalDict)


def obtainMessageVector(MessageName, MessageModule, MessageObj, MessageCount,
   SimContainer, VarName, VarType, startIndex, stopIndex, 
   messageType = sim_model.messageBuffer):
   
   ## Begin Method steps here
   LocalContainer, TotalDict = getMessageContainers(MessageName, MessageModule,
      MessageObj)
   ## - For each message, pull the buffer, and update the keys of the dictionary
   LocalCount = 0
   swigObject = eval('LocalContainer.' + VarName);
   swigObjectGood = type(swigObject).__name__ == 'SwigPyObject'
   TimeValues = array.array('d')
   if swigObject:
      functionCall = eval('sim_model.' + VarType + 'Array_getitem')
   else:
      functionCall = eval('LocalContainer.'+VarName)
   while(LocalCount < MessageCount):
      WriteTime = SimContainer.GetWriteData(MessageName, 10000, 
         LocalContainer, messageType, LocalCount)
      currentIndex = stopIndex
      executeLoop = True
      while executeLoop:
         if swigObjectGood:
            TimeValues.append(functionCall(swigObject, currentIndex))
         else:
            TimeValues.append(functionCall[currentIndex])
         currentIndex -= 1
         if currentIndex < startIndex:
            executeLoop = False
      TimeValues.append(WriteTime)
      LocalCount += 1
   TimeValues.reverse()
   TheArray = numpy.array(TimeValues)
   ArrayDim = MessageCount
   TheArray = numpy.reshape(TheArray, (ArrayDim, TheArray.shape[0]/ArrayDim))
   return TheArray

