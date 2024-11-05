import pandas as pd
from Basilisk.utilities import unitTestSupport


class VariableRetentionParameters:
    """
    Represents a variable's logging parameters.
    """

    def __init__(self, varName, varRate, startIndex=0, stopIndex=0, varType='double'):
        self.varName = varName
        self.varRate = varRate
        self.startIndex = startIndex
        self.stopIndex = stopIndex
        self.varType = varType


class MessageRetentionParameters:
    """
    Represents a message's logging parameters.
    Args:
        name: name of the message recorder
        retainedVars: the message variable to record
    """

    def __init__(self, name, retainedVars):
        self.msgRecName = name
        self.retainedVars = retainedVars


class RetentionPolicy:
    """
    This policy controls what simulation data is saved and how it is stored.  Note that the simulation data
    array will have the message time prepended as the first column.
    """

    def __init__(self, rate=int(1E10)):
        self.logRate = rate
        self.messageLogList = []
        self.varLogList = []
        self.dataCallback = None
        self.retentionFunctions = []

    def addMessageLog(self, name, retainedVars):
        self.messageLogList.append(MessageRetentionParameters(name, retainedVars))

    def addVariableLog(self, variableName, startIndex=0, stopIndex=0, varType='double', logRate=None):
        if logRate is None:
            logRate = self.logRate
        varContainer = VariableRetentionParameters(variableName, logRate, startIndex, stopIndex, varType)
        self.varLogList.append(varContainer)

    def addLogsToSim(self, simInstance):
        for variable in self.varLogList:
            simInstance.AddVariableForLogging(variable.varName, variable.varRate,
                                              variable.startIndex, variable.stopIndex, variable.varType)

    def addRetentionFunction(self, function):
        self.retentionFunctions.append(function)

    def setDataCallback(self, dataCallback):
        self.dataCallback = dataCallback

    def executeCallback(self, data):
        if self.dataCallback is not None:
            self.dataCallback(data, self)

    @staticmethod
    def addRetentionPoliciesToSim(simInstance, retentionPolicies):
        """ Adds logs for variables and messages to a simInstance
        Args:
            simInstance: The simulation instance to add logs to.
            retentionPolicies: RetentionPolicy[] list that defines the data to log.
        """

        for retentionPolicy in retentionPolicies:
            retentionPolicy.addLogsToSim(simInstance)

        # TODO handle duplicates somehow?

    @staticmethod
    def getDataForRetention(simInstance, retentionPolicies):
        """ Returns the data that should be retained given a simInstance and the retentionPolicies

        Args:
            simInstance: The simulation instance to retrieve data from
            retentionPolicies: A list of RetentionPolicy objects defining the data to retain

        Returns:
            Retained Data in the form of a dictionary with two sub-dictionaries for messages and variables::

                {
                    "messages": {
                        "messageName": [value1,value2,value3]
                    },
                    "variables": {
                        "variableName": [value1,value2,value3]
                    }
                }
        """
        data = {"messages": {}, "variables": {}, "custom": {}}
        df = pd.DataFrame()
        dataFrames = []
        for retentionPolicy in retentionPolicies:
            for msgParam in retentionPolicy.messageLogList:

                # record the message recording times
                msgTimes = simInstance.msgRecList[msgParam.msgRecName].times()

                # record the message variables
                for varName in msgParam.retainedVars:
                    msgData = getattr(simInstance.msgRecList[msgParam.msgRecName], varName)
                    msgData = unitTestSupport.addTimeColumn(msgTimes, msgData)
                    data["messages"][msgParam.msgRecName + "." + varName] = msgData

            for variable in retentionPolicy.varLogList:
                data["variables"][variable.varName] = simInstance.GetLogVariableData(variable.varName)

            for func in retentionPolicy.retentionFunctions:
                tmpModuleData = func(simInstance)
                for (key, value) in tmpModuleData.items():
                    data["custom"][key] = value
        return data
