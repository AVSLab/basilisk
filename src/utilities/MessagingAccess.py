''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


from Basilisk.simulation import sim_model
import numpy
import array
import importlib


def getMessageContainers(messageModule, messageObj):
    # Import the module in question to get access to structures
    localObj = importlib.import_module(messageModule)
    moduleString = "localObj."
    messageModuleExecString = moduleString + messageObj + "()"

    localContainer = eval(messageModuleExecString)
    firstKeys = dir(localContainer)
    filteredKeys = []
    totalDict = {"MessageTime": []}  # MessageTime is time of write for message
    for localKey in firstKeys:
        # Filter out private and the this pointer because we don't need them
        if localKey[0] != '_' and localKey != 'this':
            filteredKeys.append(localKey)
            totalDict.update({localKey: []})

    return localContainer, totalDict


def obtainMessageVector(messageName, messageModule, messageObj, messageCount,
                        simContainer, varName, varType, startIndex, stopIndex,
                        messageType=sim_model.messageBuffer):
    localContainer, totalDict = getMessageContainers(messageModule, messageObj)

    # For each message, pull the buffer, and update the keys of the dictionary
    count = 0
    swigObject = eval('localContainer.' + varName)
    swigObjectGood = type(swigObject).__name__ == 'SwigPyObject'
    timeValues = array.array('d')
    messageNoSpace = messageName.translate(None, "[]'() .")
    varNameClean = varName.translate(None, "[]'() .")

    if swigObjectGood:
        functionCall = eval('sim_model.' + varType + 'Array_getitem')
    else:  # So this is weird, but weirdly we need to punch a duck now
        refFunctionString = 'def GetMessage' + messageNoSpace + varNameClean + '(self):\n'
        refFunctionString += '   return self.' + varName
        exec refFunctionString
        functionCall = eval('GetMessage' + messageNoSpace + varNameClean)

    while count < messageCount:
        writeTime = simContainer.GetWriteData(messageName, 10000, localContainer, messageType, count)
        currentIndex = stopIndex
        executeLoop = True
        dataOut = 0
        if not swigObjectGood:
            dataOut = functionCall(localContainer)
            try:
                dataOut = [y for x in dataOut for y in x]
            except TypeError:
                placeHold = 1
        while executeLoop:
            if swigObjectGood:
                timeValues.append(functionCall(swigObject, currentIndex))
            elif startIndex != stopIndex:
                timeValues.append(dataOut[currentIndex])
            else:
                timeValues.append(dataOut)
            currentIndex -= 1
            if currentIndex < startIndex:
                executeLoop = False
        timeValues.append(writeTime)
        count += 1

    timeValues.reverse()
    resultArray = numpy.array(timeValues)
    arrayDim = messageCount
    if arrayDim > 0:
        resultArray = numpy.reshape(resultArray, (arrayDim, resultArray.shape[0] / arrayDim))
    return resultArray


def findMessageMatches(searchString, simContainer):
    i = 0
    matchList = []
    totalList = simContainer.getUniqueMessageNames()
    for localName in totalList:
        if localName.find(searchString) >= 0:
            matchList.append(localName)
        i += 1
    return matchList
