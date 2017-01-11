import os
import numpy as np
import math
import scipy as sci
from collections import defaultdict

def linearInterp(x0, x1, y0, y1, xdes):

    ydes = y0 + (xdes - x0) * (y1 - y0)/(x1-x0)

    return ydes

class fileParams:
    def __init__(self, fileFreq, columnNames, sepChar):
        self.fileFreq = fileFreq
        self.columnNames = columnNames
        self.sepChar = sepChar

class options:
    def __init__(self, outputCaching):
        self.outputCaching = outputCaching

class dataFactory:
    def __init__(self, baseFileName, baseFileParams, desiredParams, options):
        self.baseFileName = baseFileName
        self.options = options
        self.baseParams = baseFileParams
        self.desParams = desiredParams
        #   Presumably we want to open up the base file before doing anything else
        self.baseFileHandle = open(self.baseFileName, 'r')
        #   Find the file length and reset the iterator head
        self.baseFileLen = sum(1 for line in self.baseFileHandle)
        self.baseFileHandle.seek(0)

    def getBaseData(self):
        #   Split the string of column names to use as variables later
        self.varNames = self.baseParams.columnNames.split()
        self.numVar = len(self.varNames)
        self.baseDict = defaultdict(list)

        #   Iterate over each line in the file
        for ind in range(0, self.baseFileLen):
            currLine = self.baseFileHandle.readline()
            currVals = currLine.split()
            #print currVals
            #   Iterate over each seperated value in the line
            for valInd in range(0, self.numVar):
                #print self.varNames[valInd]
                self.baseDict[self.varNames[valInd]].append(float(currVals[valInd]))
                #print self.baseDict[self.varNames[valInd]]
        self.baseFileHandle.seek(0)

    def baseToDes(self):
        self.getBaseData()
        #   Figure out the frequency of data in the base file and in the desired output
        #   Example: "3 hour" to "1 day"
        baseVals = self.baseParams.fileFreq.split()
        baseFileQuant = float(baseVals[0])
        baseFileUnit = baseVals[1]
        desVals = self.desParams.fileFreq.split()
        desQuant = float(desVals[0])
        desUnit = desVals[1]

        unitList = ['second','minute','hour','day','year']
        convList = [60, 60, 24, 365]

        convDict = dict(zip(unitList, range(0,5)))
        baseInd = convDict[baseFileUnit]
        desInd = convDict[desUnit]
        indDiff = desInd - baseInd

        convFactor = 1
        if not indDiff == 0:
            for ind in range(baseInd, desInd):
                convFactor = convFactor * convList[ind]

        indsToAvg = (desQuant / baseFileQuant * convFactor)

        indsToAvg = int(indsToAvg)

        self.desDict = defaultdict(list)

        for ind in range(0,self.baseFileLen,indsToAvg):
            for nameInd in range(0,self.numVar):
                self.desDict[self.varNames[nameInd]].append(np.mean(self.baseDict[self.varNames[nameInd]][ind:ind+indsToAvg]))

    def getVarVal(self, doy, hour, min, sec, varName):
        #   If we haven't already generated the desired dictionary of converted values, do that first.
        if not hasattr(self, 'desDict'):
            self.baseToDes()

        fracHour = hour + min/60.0 + sec/ 3600.0

        for ind in range(0,len(self.desDict["Year"])):
            if self.desDict["Day"][ind] == doy:
                #   If the time at this index is GREATER than the requested time,
                if self.desDict["Hour"][ind] >= fracHour:
                    returnVal = linearInterp(self.desDict["Hour"][ind-1],self.desDict["Hour"][ind], self.desDict[varName][ind-1], self.desDict[varName][ind], fracHour)

        if 'returnVal' in vars():
            return returnVal
        else:
            return 1













