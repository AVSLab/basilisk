import os
import numpy as np
import math
import scipy as sci
from collections import defaultdict

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
                self.baseDict[valInd].append(currVals[valInd])



