import SimulationBaseClass as sbc
import json
import sys
import MessagingAccess
import sim_model
import inspect
import time


class DataDelegate:
    def __init__(self, simulation):
        self.simulation = simulation
        self.isNewConnection = True
        self.encoder = VisJSONEncoder()
        self.processName = 'DynamicsProcess'
        self.visMessages = []
        self.visSimModules = []

    def packageData(self):
        tmpObjects = []
        if self.isNewConnection:
            self.isNewConnection = False
            self.initDataSource()
        # package up grabbed (updated) messages into parsed structures
        # leave stale values in structures
        messageData = self.collectMessages()

        for messageStruct in messageData:
            tmpClassType = type(messageStruct).__name__
            if tmpClassType == "ThrusterOutputData":
                tmpObjects.append(VisThruster(messageStruct))
            elif tmpClassType == "ReactionWheelConfigData":
                tmpObjects.append(VisReactionWheel(messageStruct))
            elif tmpClassType == "OutputStateData":
                tmpObjects.append(VisSpacecraft(messageStruct))
            elif tmpClassType == "SpicePlanetState":
                tmpObjects.append(VisPlanetState(messageStruct))
        return tmpObjects

    def initDataSource(self):
        msgNames = self.simulation.TotalSim.getUniqueMessageNames()

        for name in msgNames:
            msgIdentData = self.simulation.TotalSim.getMessageID(name)
            if msgIdentData.bufferName == self.processName:
                self.visMessages.append({'msgName': name, 'msgData': msgIdentData})

        for msg in self.visMessages:
            headerData = sim_model.MessageHeaderData()
            self.simulation.TotalSim.populateMessageHeader(msg['msgName'], headerData)

            for moduleData in self.simulation.simModules:
                for key, obj in inspect.getmembers(moduleData):
                    if inspect.isclass(obj):
                        if obj.__name__ == headerData.messageStruct:
                            self.visSimModules.append([msg['msgName'], moduleData.__name__, headerData.messageStruct])
                            break

            # if self.visSimModules.has_key(msg['msgName']) is False:
            #     print "Failed to find valid message structure for: "+headerData.messageStruct
            #     continue

    def collectMessages(self):
        messages = []
        for msgData in self.visSimModules:
            localContainer, totalDict = MessagingAccess.getMessageContainers(msgData[1], msgData[2])
            messageType = sim_model.messageBuffer
            writeTime = self.simulation.TotalSim.GetWriteData(msgData[0], 10000,
                                                                  localContainer, messageType, 0)
            messages.append(localContainer)
        return messages

    def encodeDataToJSON(self, tmpObjects):
        return self.encoder.encode(tmpObjects)


class VisJSONEncoder(json.JSONEncoder):
    def default(self, obj):
        return {type(obj).__name__: obj.__dict__}


class VisReactionWheel(object):
    def __init__(self, reactionWheelConfigDataStruct):
        self.omega = reactionWheelConfigDataStruct.Omega
        self.u = reactionWheelConfigDataStruct.u_current


class VisThruster(object):
    def __init__(self, messageDataDict):
        self.position = messageDataDict.thrusterLocation
        self.thrusterDirection = messageDataDict.thrusterDirection
        self.maxThrust = messageDataDict.maxThrust
        self.thrustFactor = messageDataDict.thrustFactor


class VisSpacecraft(object):
    def __init__(self, outputStateDataBuffer):
        self.r_N = outputStateDataBuffer.r_N
        self.v_N = outputStateDataBuffer.v_N
        self.omega = outputStateDataBuffer.omega
        self.sigma = outputStateDataBuffer.sigma


class VisPlanetState(object):
    def __init__(self, spicePlanetStateStruct):
        self.position = spicePlanetStateStruct.PositionVector
        self.velocity = spicePlanetStateStruct.VelocityVector
        self.planetName = spicePlanetStateStruct.PlanetName