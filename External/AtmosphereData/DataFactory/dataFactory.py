import os
import numpy as np
import math
import scipy as sci
from collections import defaultdict

class fileParams:
    def __init__(self, fileFreq, columnNames):
        self.fileFreq = fileFreq
        self.columnNames = columnNames

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
            currLine = self.baseFileName.readline()
            currVals = currLine.split(self.baseFileParams.sepChar)
            #   Iterate over each seperated value in the line
            for valInd in range(0, self.numVar):
                self.baseDict[self.varNames[valInd]].append(currVals[valInd])

    def baseToDes(self):
        self.getBaseData()
        #   Figure out the frequency of data in the base file and in the desired output
        #   Example: "3 hour" to "1 day"
        baseVals = self.baseFileParams.fileFreq.split()
        baseFileQuant = float(baseVals[0])
        baseFileUnit = baseVals[1]
        desVals = self.desiredParams.fileFreq.split()
        desQuant = float(desVals[0])
        desUnit = baseVals[2]

        unitList = ['second','minute','hour','day','year']
        convList = [60, 60, 24, 365]

        convDict = dict(zip(unitList, range(0,5)))

        baseInd = convDict(baseFileUnit)
        desInd = convDict(desUnit)
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
                self.desDict(self.varNames[nameInd]).append(np.mean(self.baseDict[self.varNames[nameInd]][ind:indsToAvg]))








