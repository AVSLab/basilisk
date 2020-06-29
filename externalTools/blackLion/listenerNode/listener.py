import sys, os
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(path + '/../')
from blackLion import workerProcess, constants

import zmq
from zmq.eventloop import ioloop
import binascii
import argparse
import warnings

try:
    from Basilisk.simulation import sim_model
except:
    print("PyListerner running without Basilisk")


class ListenerRouter(object):
    def __init__(self):
        self.localSubscriptions = set()

    def collect_localRouteMessages(self):
        self.localSubscriptions.add("reactionwheel_output_states")
        self.localSubscriptions.add("viz_interface_msg")
        return self.localSubscriptions

    def routeMessage(self, packet_type, payload_size, payload):
        if packet_type == constants.VIZ:
            print("\nVIZ ARCH PACKET")
            self.deserialize_raw_payload(payload_size, payload)
        elif packet_type == constants.BSK:
            #print "\nBSK PACKET"
            #self.deserialize_raw_payload(payload_size, payload)
            #self.deserialize_payload(payload_size, payload)
            pass
        return

    def deserialize_payload(self, payload_size, payload):
        """
        Converts the payload (a hex encoded string, as received from ZMQ) into list of int bytes.
        @param: payload_size: size of the payload.
        @param: payload: hex encoded string.
        @return: msg_buffer:.
        """
        msg_buffer = sim_model.new_cByteArray(payload_size)
        cdata = binascii.unhexlify(payload)
        dataOrd = [ord(x) for x in cdata]
        for i in range(0, payload_size):
            sim_model.cByteArray_setitem(msg_buffer, i, int(dataOrd[i]))
        #print 'payload = %s' % payload
        #print 'cdata =%s' % cdata
        #print 'dataOrd = %s' % dataOrd
        print('msg_buffer = %s\n' % msg_buffer)
        print('Py Listener node. Unserialized dataOrd: ', dataOrd)
        return dataOrd

    def deserialize_raw_payload(self, payload_size, payload):
        """
        Converts the payload (a hex encoded string, as received from ZMQ) into list of int bytes.
        @param: payload_size: size of the payload.
        @param: payload: hex encoded string.
        @return: msg_buffer:.
        """
        msg_buffer = sim_model.new_cByteArray(payload_size)
        hexArray = []
        for i in range(0, payload_size):
            sim_model.cByteArray_setitem(msg_buffer, i, ord(payload[i]))
            hexArray.append(hex(ord(payload[i])))
        print('hexArray = %s' % hexArray)
        return


class Listener(workerProcess.WorkerProcess):
    def __init__(self, name, proc_args, master_address, verbosity_level):
        super(Listener, self).__init__(name, proc_args, master_address, verbosity_level)
        self.Router = proc_args[0]

    def step_process(self):
        self.accum_sim_time += self.frame_time

    def run(self):
        self.context = zmq.Context()
        self.init_command_socket()
        ioloop.IOLoop.instance().start()

    def initialize(self):
        pass

    def match_published_msgs(self, message_names):
        localPublications = []
        return localPublications

    def get_unpublished_msgs(self):
        localSubscriptions = list(self.Router.collect_localRouteMessages())
        return localSubscriptions

    def next_pub_update(self):
        next_pub_list = []
        return next_pub_list

    def publish(self):
        pass

    def route_message_in(self, msg_name, packet_type, payload_size, payload):
        #self.logger.debug(
        # 'Py Listener node. Received packet: %s %s %s %s' % (msg_name, packet_type, payload_size, payload))
        self.logger.debug('Py Listener node. Received packet: %s %s %s %s' % (msg_name, packet_type, payload_size, payload))
        self.Router.routeMessage(packet_type, payload_size, payload)
        if msg_name == "coarse_time_masterRead":
            dataOrd = self.Router.deserialize_payload(payload_size, payload)
            self.logger.debug("Py Listener node. Unserialized clock dataOrd: ", dataOrd)
        return

def add_arg_definitions(parser):
    parser.add_argument('--master_address', nargs='?', required=True,
                        help='Address string to connect to the controller')
    parser.add_argument('--node_name', nargs='?', default="",
                        help='Address string to connect to the controller')
    parser.add_argument('--verbosity_level', nargs='?', default="",
                        help='Verbosity level of the PyListener logger')


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description='EMM Simulation Standalone Test.')
    add_arg_definitions(arg_parser)
    parsed_args, unknown_args = arg_parser.parse_known_args()

    if unknown_args:
        warnings.warn("PyListener unrecognised args parsed: %s" % unknown_args, RuntimeWarning)

    node_name = "PyListner"
    if parsed_args.node_name:
        node_name = parsed_args.node_name
    verbosity_level = "DEBUG"
    if parsed_args.verbosity_level:
        verbosity_level = parsed_args.verbosity_level

    master_address = None
    try:
        master_address = parsed_args.master_address
    except ValueError('Node %s needs to know the master address' % node_name):
        print("FAULT")

    PyListener_process = Listener(
        name=node_name,
        proc_args=[ListenerRouter()],
        master_address=master_address,
        verbosity_level=verbosity_level)

    print("Node %s: STARTING " % node_name)
    PyListener_process.run()

