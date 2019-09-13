import pandas as pd

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
        messageName: the name of the message to log
        messageRate: rate to log the message at.
        dataType: the dataType to pull
    """

    def __init__(self, messageName, messageRate, retainedVars):
        self.messageName = messageName
        self.messageRate = messageRate
        self.retainedVars = retainedVars


class RetentionPolicy():

    def __init__(self, rate=int(1E10)):
        self.logRate = rate
        self.messageLogList = []
        self.varLogList = []
        self.dataCallback = None
        self.retentionFunctions = []

    def addMessageLog(self, messageName, retainedVars, logRate=None):
        if logRate is None:
            logRate = self.logRate
        self.messageLogList.append(MessageRetentionParameters(messageName, logRate, retainedVars))

    def addVariableLog(self, variableName, startIndex=0, stopIndex=0, varType='double', logRate=None):
        if logRate is None:
            logRate = self.logRate
        varContainer = VariableRetentionParameters(variableName, logRate, startIndex, stopIndex, varType)
        self.varLogList.append(varContainer)

    def addLogsToSim(self, simInstance):
        for message in self.messageLogList:
            simInstance.TotalSim.logThisMessage(message.messageName, message.messageRate)
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
            simInstance: The simulation instance to retrive data from
            retentionPolicies: A list of RetentionPolicy objects defining the data to retain
        Returns:
            Retained Data: In the form of a dictionary with two sub-dictionaries for messages and variables:
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
            for message in retentionPolicy.messageLogList:
                for (param, dataType) in message.retainedVars:
                    name = message.messageName + "." + param
                    if dataType is None:
                        msg = simInstance.pullMessageLogData(name)
                    else:
                        msg = simInstance.pullMessageLogData(name, dataType)
                    data["messages"][name] = msg

            for variable in retentionPolicy.varLogList:
                data["variables"][variable.varName] = simInstance.GetLogVariableData(variable.varName)

            for func in retentionPolicy.retentionFunctions:
                tmpModuleData = func(simInstance)
                for (key, value) in tmpModuleData.items():
                    data["custom"][key] = value
        return data