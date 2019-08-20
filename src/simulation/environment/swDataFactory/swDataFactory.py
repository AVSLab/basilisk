
import sys, os, inspect
import numpy as np
import ctypes
import math
import csv
import collections as co

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
# if this script is run from a custom folder outside of the Basilisk folder, then uncomment the
# following line and specify the absolute bath to the Basilisk folder
#bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')
# @endcond

# import FSW Algorithm related support
import simulationArchTypes
import nrlmsiseAtmosphere

class fileParams:
    def __init__(self, fileFreq, columnNames, sepChar):
        self.fileFreq = fileFreq
        self.columnNames = columnNames
        self.sepChar = sepChar

class options:
    def __init__(self, outputCaching):
        self.outputCaching = outputCaching

class swDataFactory(simulationArchTypes.PythonModelClass):
    def __init__(self, modelName, modelActive=True, modelPriority=-1):
        super(swDataFactory, self).__init__(modelName, modelActive, modelPriority)

        ## Output body torque message name
        self.outputDataMessageNames = []
        self.outputDataMessageIds = []
        self.outputData = []

        self.baseFileName = " "
        self.options = options
        #self.baseParams = baseFileParams
        #self.desParams = desiredParams

        #   Presumably we want to open up the base file before doing anything else
        self.baseFileHandle = open(self.baseFileName, 'r')

        #   Find the file length and reset the iterator head
        self.baseFileLen = sum(1 for line in self.baseFileHandle)
        self.baseFileHandle.seek(0)

        self.dataDict = co.defaultdict(dict)

    ## The selfInit method is used to initialze all of the output messages of a class.
    # It is important that ALL outputs are initialized here so that other models can
    # subscribe to these messages in their crossInit method.
    def addMessageType(self, dataType, dataAvg, dataOffset):
        tmpMsgName = str(dataType) + "_" + str(dataAvg) + "_" + str(dataOffset)

        if tmpMsgName != self.outputDataNameList:
            self.outputDataMessageNames.append(tmpMsgName)
            self.outputData.append(nrlmsiseAtmosphere.swDataSimMsg())
            self.desVarList.append(dataType)
            self.dataDict[dataType] = co.defaultdict(data)
            self.desFreqList.append(dataAvg)
        else:
            print "Warning: Redundant SWData message requested."

        return

    def selfInit(self):

        for ind in range(0,len(self.outputDataNameList)):
            self.outputDataMessageIds.append(simulationArchTypes.CreateNewMessage(self.outputDataName[ind], self.outputData[ind],
                                                                 self.moduleID))
        return

    ## The crossInit method is used to initialize all of the input messages of a class.
    #  This subscription assumes that all of the other models present in a given simulation
    #  instance have initialized their messages during the selfInit step.
    def crossInit(self):
        return

    ## The reset method is used to clear out any persistent variables that need to get changed
    #  when a task is restarted.  This method is typically only called once after selfInit/crossInit,
    #  but it should be written to allow the user to call it multiple times if necessary.
    def reset(self, currentTime):
        return

    ## The updateState method is the cyclical worker method for a given Basilisk class.  It
    # will get called periodically at the rate specified in the Python task that the model is
    # attached to.  It persists and anything can be done inside of it.  If you have realtime
    # requirements though, be careful about how much processing you put into a Python updateState
    # method.  You could easily detonate your sim's ability to run in realtime.
    def updateState(self, currentTime):

        for messageInd in range(0,len(self.outputDataMessageIds)):
            currVar = self.desVarList[messageInd]
            currFreq=  self.desFreqList[messageInd]
            self.outputData[messageInd] = computeDesDict(desVar, desFreq, currentTime)
            simulationArchTypes.WriteMessage(self.outputDataMessageIds[messageInd], currentTime, self.outputData[messageInd], self.moduleID)
        return

    ##  linearInterp is a simple linear interpolation function.
    #   INPUTS
    #   x0 - initial x value of the data set
    #   x1 - final x value of the data set
    #   y0 - initial y (dependent) variable of the data set
    #   y1 - final y (dependent) variable of the data set
    #   xdes - x value at which y should be found
    #   OUTPUTS
    #   ydes - desired y value at xdes
    def linearInterp(x0, x1, y0, y1, xdes):
        ydes = y0 + (xdes - x0) * (y1 - y0) / (x1 - x0)

        return ydes

    ##  computeDesVal(self, desVar, desFreq, currentTime) computes a desired variable with a specified averaging frequency and offset at the current time
    #   from the set base dataset.
    def computeDesVal(self, desVar, desFreq, desOffset, currentTime):

        #   Compute the current fractional hour
        fracHour = self.baseHour + self.baseMin/60.0 + self.BaseSec/ 3600.0 + currentTime / (1e9*3600.0)


    ##  getBaseData() loads into a dict the selected base dataset.
    def getBaseData(self):
        #   Split the string of column names to use as variables later
        self.varNames = self.baseParams.columnNames.split()
        self.numVar = len(self.varNames)
        self.baseDict = co.defaultdict(list)

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

    ##  baseToDes() converts from the base data set into a desired data format, i.e. converts from 1-hour indices to 3-hour averages.
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

        self.desDict = co.defaultdict(list)

        for ind in range(0,self.baseFileLen,indsToAvg):
            for nameInd in range(0,self.numVar):
                self.desDict[self.varNames[nameInd]].append(np.mean(self.baseDict[self.varNames[nameInd]][ind:ind+indsToAvg]))

    ##  getVarVal(doy, hour, min, sec, varName) either computes the desired value of a given variable at a specific instant or fetches
    #   it from an existing dict.
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











