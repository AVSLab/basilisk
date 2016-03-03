'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
import sys
import copy
import sim_model
import numpy
import array
import pdb

def getMessageContainers(MessageModule, MessageObj):
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

def obtainMessageVector(MessageName, MessageModule, MessageObj, MessageCount,
   SimContainer, VarName, VarType, startIndex, stopIndex, 
   messageType = sim_model.messageBuffer):
   ## Begin Method steps here
   LocalContainer, TotalDict = getMessageContainers(MessageModule, MessageObj)
   ## - For each message, pull the buffer, and update the keys of the dictionary
   LocalCount = 0
   swigObject = eval('LocalContainer.' + VarName);
   swigObjectGood = type(swigObject).__name__ == 'SwigPyObject'
   TimeValues = array.array('d')
   if swigObject:
      functionCall = eval('sim_model.' + VarType + 'Array_getitem')
   else: #So this is weird, but weirdly we need to punch a duck now
      RefFunctionString = 'def GetMessage' + MessageName + VarName + '(self):\n'
      RefFunctionString += '   return self.' + VarName
      exec(RefFunctionString)
      functionCall = eval('GetMessage'+MessageName + VarName)
   while(LocalCount < MessageCount):
      WriteTime = SimContainer.GetWriteData(MessageName, 10000, 
         LocalContainer, messageType, LocalCount)
      currentIndex = stopIndex
      executeLoop = True
      while executeLoop:
         if swigObjectGood:
            TimeValues.append(functionCall(swigObject, currentIndex))
         else:
            TimeValues.append(functionCall(LocalContainer))
         currentIndex -= 1
         if currentIndex < startIndex:
            executeLoop = False
      TimeValues.append(WriteTime)
      LocalCount += 1
   TimeValues.reverse()
   TheArray = numpy.array(TimeValues)
   ArrayDim = MessageCount
   if ArrayDim > 0:
      TheArray = numpy.reshape(TheArray, (ArrayDim, TheArray.shape[0]/ArrayDim))
   return TheArray

def findMessageMatches(searchString, SimContainer):
    i=0
    matchList = []
    totalList = SimContainer.getUniqueMessageNames()
    for localName in totalList:
        if(localName.find(searchString) >= 0):
           matchList.append(localName)
        i+=1
    return matchList

