import SimulationBaseClass as sbc
import json
import sys
import MessagingAccess
import sim_model
import inspect
import threading
import copy

class DataDelegate:
    def __init__(self, simulation):
        self.simulation = simulation
        self.isNewConnection = True
        self.encoder = VisJSONEncoder()
        self.processName = 'DynamicsProcess'
        self.visMessages = []

    def packageData(self):
        print "Stage 1: "+sys._getframe().f_code.co_name
        tmpObjects = [] # Clear the list
        if self.isNewConnection:
            self.isNewConnection = False
            self.initDataSource()
        # package up grabbed (updated) messages into parsed structures
        # leave stale values in structures
        messages = self.collectMessages()

        for messageStruct in messages:
            for function in (VisReactionWheel, VisPlanetState, VisThruster):
                try:
                    tmpObjects.append(function(messageStruct))
                except:
                    continue
        return tmpObjects

    def encodeDataToJSON(self, tmpObjects):
        print sys._getframe().f_code.co_name
        return self.encoder.encode(tmpObjects)

    def initDataSource(self):
        # dataMap = self.simulation.writeDataMapDot([1])
        msgNames = self.simulation.TotalSim.getUniqueMessageNames()

        for name in msgNames:
            msgIdentData = self.simulation.TotalSim.getMessageID(name)
            if msgIdentData.bufferName == self.processName:
                # self.visMessages.append({'msgName': name, 'msgId': int(msgIdentData.itemID)})
                self.visMessages.append({'msgName': name, 'msgData': msgIdentData})

    def collectMessages(self):
        # lock = threading.Lock()
        # try:
        print "In: "+sys._getframe().f_code.co_name
        messages = []
        # lock.acquire()
        for msg in self.visMessages:
            headerData = sim_model.MessageHeaderData()
            self.simulation.TotalSim.populateMessageHeader(msg['msgName'], headerData)

            moduleFound = ''
            for moduleData in self.simulation.simModules:
                if moduleFound != '':
                    break
                for name, obj in inspect.getmembers(moduleData):
                    if inspect.isclass(obj):
                        if obj.__name__ == headerData.messageStruct:
                            moduleFound = moduleData.__name__
                            break

            if moduleFound == '':
                print "Failed to find valid message structure for: "+headerData.messageStruct
                continue

            # print name
            localContainer, totalDict = MessagingAccess.getMessageContainers(moduleFound, headerData.messageStruct)
            # print localContainer.this.__repr__()

            # test_container = copy.deepcopy(localContainer)
            messageType = sim_model.messageBuffer
            # print "msg[msgName] " + msg['msgName']
            # print msg['msgName']
            # print "localContainer " + str(localContainer)
            # print messageType
            try:
                writeTime = self.simulation.TotalSim.GetWriteData(msg['msgName'], 10000,
                                                                  localContainer, messageType, 0)
            except:
                 print "except"
            messages.append(localContainer)
        # finally:
            # lock.release()
        print "Out: "+sys._getframe().f_code.co_name
        return messages


class VisJSONEncoder(json.JSONEncoder):
    def default(self, obj):
        return {type(obj).__name__: obj.__dict__}


class VisReactionWheel(object):
    def __init__(self, reactionWheelConfigDataStruct):
        self.omega = reactionWheelConfigDataStruct['Omega']


class VisThruster(object):
    def __init__(self, messageDataDict):
        self.position = messageDataDict.thrusterLocation
        self.thrusterDirection = messageDataDict.thrusterDirection
        self.maxThrust = messageDataDict.maxThrust
        self.thrustFactor = messageDataDict.thrustFactor
        # etc.


class VisSpacecraft(object):
    def __init__(self, outputStateDataBuffer):
        self.r_N = sbc.getCArray('double', outputStateDataBuffer.r_N, 3)
        self.omega = sbc.getCArray('double', outputStateDataBuffer.omega, 3)
        self.sigma = sbc.getCArray('double', outputStateDataBuffer.sigma, 3)


class VisPlanetState(object):
    def __init__(self, spicePlanetStateStruct):
        self.position = spicePlanetStateStruct.stuff