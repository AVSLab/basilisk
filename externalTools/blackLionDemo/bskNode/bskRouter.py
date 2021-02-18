import binascii
import sys, os

path = os.path.dirname(os.path.abspath(__file__))
#sys.path.append(path + '/../../../dist/')
from Basilisk.simulation import sim_model
from Basilisk.utilities import simulationArchTypes
from Basilisk.utilities import macros as mc

sys.path.append(path + '/../')
from blackLion import constants


# ----------------------------- High-level classes ----------------------------- #
class TargetProcess_class():
    """Target Process"""
    def __init__(self, timeStep, procName):
        self.targetTimeStep = timeStep
        self.targetProcName = procName


class BSK_router_class():
    def __init__(self, SimBase, targetProcess):
        self.processName = SimBase.RouterProcessName
        self.taskName = "RouterTask"
        self.processTasksTimeStep = targetProcess.targetTimeStep
        SimBase.routerProc.createPythonTask(self.taskName, self.processTasksTimeStep, True, -1)
        self.router = PyRouter("RouterModel", True, )
        self.router.initialize_source(targetProcess.targetProcName)
        SimBase.routerProc.addModelToTask(self.taskName, self.router)


# ----------------------------- Low-level classes ----------------------------- #
class DataExchangeObject(object):
    def __init__(self):
        self.packet_type = ""
        self.message_ID = -1
        self.last_update_counter = 0
        self.payload_size = 0
        self.payload = []

    def updateData(self, last_update_counter, payload_size, payload):
        self.last_update_counter = last_update_counter
        self.payload_size = payload_size
        self.payload = payload


class PyRouter(simulationArchTypes.PythonModelClass):
    """
    test
    """
    def __init__(self, modelName, modelActive=True, modelPriority=-1):
        super(PyRouter, self).__init__(modelName, modelActive, modelPriority)
        self.localSubscriptions = {}  # dict[msg name] = msg ID of messages to receive
        self.localPublications = {}  # dict[msg name] = DataExchangeObject, with all msgs matched
        self.currentDataExchangeOut = {}  # dict[msg name] = DataExchangeObject, with all msgs to publish @current t
        self.messageSource = ""
        self.source = -1

    def initialize_source(self, sourceProcName):
        self.messageSource = sourceProcName
        self.source = sim_model.SystemMessaging_GetInstance().findMessageBuffer(sourceProcName)

    def selfInit(self):
        return
    def crossInit(self):
        return
    def reset(self, currentTime):
        return
    def updateState(self, currentTime):
        """
            For every msg_name, exchange_object in the dictionary self.localPublications,
            it reads the payload as stored in the internal Messaging System @ currentTime.
            If the msg has been internally updated, the dictionary value exchange_object is also updated
            (with  the new update_counter, payload_size and payload information).
            The empty dictionary self.currentDataExchangeOut is filled with the entries in self.localPublications
            that have been updated @ currentTime.
        """
        self.updateVizState(currentTime)
        sim_model.SystemMessaging_GetInstance().selectMessageBuffer(self.source)
        local_header = sim_model.SingleMessageHeader()
        for msg_name, exchange_object in self.localPublications.items():
            msgHeader = sim_model.SystemMessaging_GetInstance().FindMsgHeader(exchange_object.message_ID)
            msgBuffer = sim_model.new_cByteArray(msgHeader.CurrentReadSize)
            # print 'header.UpdateCounter = %d, router.last_update_count = %d'%(msgHeader.UpdateCounter, exchange_object.last_update_counter)
            if msgHeader.UpdateCounter > exchange_object.last_update_counter:
                sim_model.SystemMessaging_GetInstance().ReadMessage(
                    exchange_object.message_ID, local_header, msgHeader.CurrentReadSize, msgBuffer, self.moduleID)
                payload = self.serialize_payload(msgHeader.CurrentReadSize, msgBuffer)
                # print 'update_payload = %s' % payload
                exchange_object.updateData(msgHeader.UpdateCounter, int(local_header.WriteSize), payload)
                self.currentDataExchangeOut[msg_name] = exchange_object
        # print 'currentDataExchangeOut = %s'%self.currentDataExchangeOut.keys()
        return

    def updateVizState(self, currentTime):
        sim_model.SystemMessaging_GetInstance().selectMessageBuffer(self.source)
        local_header = sim_model.SingleMessageHeader()
        if "viz_message" in self.localPublications.keys():
            exchange_object = self.localPublications["viz_message"]
            msgHeader = sim_model.SystemMessaging_GetInstance().FindMsgHeader(exchange_object.message_ID)
            msgBuffer = sim_model.new_cByteArray(msgHeader.CurrentReadSize)
            # print "UpdateCounter = ", msgHeader.UpdateCounter, "last_update_counter = ", exchange_object.last_update_counter
            if msgHeader.UpdateCounter > exchange_object.last_update_counter:
                sim_model.SystemMessaging_GetInstance().ReadMessage(
                    exchange_object.message_ID, local_header, msgHeader.CurrentReadSize, msgBuffer, self.moduleID)

                cdata_payload = sim_model.cdata(msgBuffer, msgHeader.CurrentReadSize)
                hex_byte_count = binascii.hexlify(cdata_payload[0:4])  # sizeof(uint32_t)
                byte_count = "0x" + hex_byte_count[4:6] + hex_byte_count[2:4] + hex_byte_count[0:2]
                int_count = int(byte_count, 0)
                viz_payload = cdata_payload[4:int_count+4]
                # print "byte_count_new = ", int(byte_count, 0)
                # print "len(viz_payload) = ", len(viz_payload)
                # print "viz_msg size = ", int(local_header.WriteSize)
                exchange_object.updateData(msgHeader.UpdateCounter, int_count, viz_payload)
                # exchange_object.updateData(msgHeader.UpdateCounter, int(local_header.WriteSize), cdata_payload)
                self.currentDataExchangeOut["viz_message"] = exchange_object

    def routeMessage(self, current_sim_time, msg_name, payload_size, payload):
        sim_model.SystemMessaging_GetInstance().selectMessageBuffer(self.source)
        msg_ID = self.localSubscriptions[msg_name]
        write_clock_nanos = mc.sec2nano(current_sim_time)
        msg_buffer = self.deserialize_payload(payload_size, payload)
        sim_model.SystemMessaging_GetInstance().WriteMessage(msg_ID, write_clock_nanos, payload_size, msg_buffer, self.moduleID)
        return

    def serialize_payload(self, payload_size, msg_buffer):
        """
        Converts BSK message bytes into a hex encoded string.
        @param: payload_size: size of the BSK internal message struct.
        @param: msg_buffer: SWIG new_cByteArray filled with data as read from the BSK internal msg system.
        @return: hex_payload: hex encoded string.
        """
        cdata_payload = sim_model.cdata(msg_buffer, payload_size).encode("utf-16", "surrogatepass")
        return cdata_payload

    def deserialize_payload(self, payload_size, payload):
        """
        Converts the payload (a hex encoded string, as received from ZMQ) into BSK message bytes.
        @param: payload_size: size of the BSK internal message struct.
        @param: payload: hex encoded string.
        @return: msg_buffer: SWIG new_cByteArray filled with data to be written down into  the BSK internal msg system.
        """
        msg_buffer = sim_model.new_cByteArray(payload_size)
        cdata = payload.decode("iso-8859-1").encode("utf-8").decode("utf-8")
        dataOrd = [ord(x) for x in cdata]
        for i in range(0, payload_size):
                sim_model.cByteArray_setitem(msg_buffer, i, int(dataOrd[i]))
        def print_stuff():
            print('\nDESERIALIZATION: UNHEXLIFY')
            print('payload = %s' % payload)
            print('payload_type = %s' % type(payload))
            print('cdata =%s' % cdata)
            print('cdata_type = %s' % type(cdata))
            print('dataOrd = %s' % dataOrd)
            print('dataOrd_type = %s' % type(dataOrd))
            print('msg_buffer = %s\n' % msg_buffer)
        # print_stuff()
        return msg_buffer

    def collect_localRouteMessages(self):
        """
        Looks in its own process buffer (i.e. source) for the local messages whose publisher is unknown.
        Fills a dictionary (self.localSubscriptions) where keys are the message names and values are the message IDs.
        @return: self.localSubscriptions.keys() : names of messages to subscribe to
        """
        sim_model.SystemMessaging_GetInstance().selectMessageBuffer(self.source)
        unknownPublisherList = sim_model.SystemMessaging_GetInstance().getUnpublishedMessages()
        for localMessageName in unknownPublisherList:
            msg_ID = int(sim_model.SystemMessaging_GetInstance().FindMessageID(localMessageName))
            sim_model.SystemMessaging_GetInstance().obtainWriteRights(msg_ID, self.moduleID)
            self.localSubscriptions[localMessageName] = msg_ID
        return self.localSubscriptions.keys()

    def match_externalRouteMessages(self, externalMessageList):
        """
        Receives a list of message names and looks in its own process buffer (i.e. source) if there is any match.
        Fills an internal dictionary (self.localPublications) where keys are the msg_name of potential publications,
        and values are objects of type DataExchangeObject().
        @param: externalMessageList: list of message names to check for a match
        @return: self.localPublications.keys() :  names of messages to publish
        """
        sim_model.SystemMessaging_GetInstance().selectMessageBuffer(self.source)
        for externalMessageName in externalMessageList:
            local_msgID = sim_model.SystemMessaging_GetInstance().FindMessageID(externalMessageName)
            if local_msgID >= 0:
                if not (externalMessageName in self.localPublications.keys()):
                    local_dataExchange = DataExchangeObject()
                    local_dataExchange.message_ID = local_msgID
                    local_dataExchange.packet_type = constants.BSK
                    self.localPublications[externalMessageName] = local_dataExchange
                sim_model.SystemMessaging_GetInstance().obtainReadRights(local_msgID,
                                                                         self.moduleID)  # ROUTER-INTERFACE READ RIGHTS
        return self.localPublications.keys()
