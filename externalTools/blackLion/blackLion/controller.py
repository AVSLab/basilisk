''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
import sys, os
import zmq
import constants
import utilities
import blLogging


sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
import time
import warnings
import argparse

path = os.path.dirname(os.path.abspath(__file__))


class CentralDataExchangeObject(object):
    def __init__(self):
        self.msg_subscribers = set()
        self.msg_publishers = set()

    def add_subscriber(self, nodeName):
        self.msg_subscribers.add(nodeName)

    def add_publisher(self, nodeName):
        self.msg_publishers.add(nodeName)

    def is_node_subscribed(self, nodeName):
        if nodeName in self.msg_subscribers:
            return True
        return False


class PayloadExchangeObject(object):
    def __init__(self):
        self.payload_size = 0
        self.payload = []


class MasterController(object):
    def __init__(self, host_name="127.0.0.1", verbosity_level="DEBUG"):
        self.host_name = host_name
        self.nodePyProcesses = []  # this should be a set
        self.context = zmq.Context()
        self.logs_socket = self.context.socket(zmq.SUB)
        self.backend = self.context.socket(zmq.XPUB)
        self.frontend = self.context.socket(zmq.SUB)

        self.logger = blLogging.createLogger(name=__name__, verbosity_level=verbosity_level)
        tmp_uid = str(time.time())
        self.run_uid = tmp_uid.replace(".", "")
        self.log_file_name = "black_lion_logs_%s.txt" % self.run_uid
        self.log_file_path = None
        self.log_file_handle = None

        self.nodeReqSockets = {}
        self.num_frames = 0
        self.frame_length = 0
        self.dataExchangeRecord = {}  # dictionary with keys= msg_name, values = CentralDataExchangeObject's

        self.backend_protocol = utilities.TcpProtocol(host_name=self.host_name)
        self.frontend_protocol = utilities.TcpProtocol(host_name=self.host_name)
        self.bind_backend()
        self.bind_frontend()

        self.current_payload_routing = []  # vectors with zmq received publications
        self.total_pub_msg_next_count = 0
        self.total_sub_msg_next_list = []
        self.sock_sub_msg_next = {}  # dictionary with keys = node_name, values = # msg to send @ next frame
        self.foreign_node_ports = {}

        self.empty_plots_folder()

    def empty_plots_folder(self):
        plot_path = "%s/../plots" % path
        if os.path.exists(plot_path):
            self.logger.debug("Cleaning '../plots' directory for new run")
            for plot_file in os.listdir(plot_path):
                os.remove(plot_path + '/' + plot_file)
        else:
            self.logger.debug("Creating new '../plots' directory")
            os.makedirs(plot_path)


    def bind_frontend(self):
        self.frontend_protocol.port = self.frontend.bind_to_random_port(self.frontend_protocol.get_bind_address())
        self.logger.debug('Controller binding to frontend address: %s' % self.frontend_protocol.get_bind_address())

    def bind_backend(self):
        self.backend_protocol.port = self.backend.bind_to_random_port(self.backend_protocol.get_bind_address())
        self.logger.debug('Controller binding to backend address: %s' % self.backend_protocol.get_bind_address())

    def subscribe_frontend_to_all_messages(self):
        self.logger.info('Frontend subscribing to msg = %s' % constants.HANDSHAKE)
        self.frontend.setsockopt(zmq.SUBSCRIBE, constants.HANDSHAKE)
        for msg_name in self.dataExchangeRecord.keys():
            self.logger.warn('Frontend subscribing to msg = %s. '
                             'Current publishers = %s. Current subscribers = %s' %
                             (msg_name,
                              self.dataExchangeRecord[msg_name].msg_publishers,
                              self.dataExchangeRecord[msg_name].msg_subscribers))
            self.frontend.setsockopt(zmq.SUBSCRIBE, msg_name)

    def publish_at_backend(self):
        # self.logger.info("Backend publishing")
        for publication in self.current_payload_routing:
            self.backend.send_multipart(publication)
        # self.logger.info("Backend closing")

    def listen_at_frontend(self):
        # stdout.write('Frontend listening. Expecting %d messages.' % self.total_pub_msg_next_count)
        local_msg_count = 0
        self.current_payload_routing = []
        self.logger.debug('Frontend received:')
        while not (local_msg_count >= self.total_pub_msg_next_count):
            # self.logger.debug('local_msg_count = %s' % local_msg_count)
            publication = self.frontend.recv_multipart()
            self.logger.debug('%s; ' % publication[0])
            self.current_payload_routing.append(publication)
            local_msg_count += 1
        # self.logger.debug('Frontend closing.')

    def receive_logs(self):
        reading = True
        logs = []
        log_buffer = ""
        while reading:
            try:
                logs = self.logs_socket.recv_multipart(zmq.NOBLOCK)
                if logs:
                    log_buffer += logs[1]
                    log_buffer += "\n"
            except zmq.ZMQError:
                reading = False
        if self.log_file_handle:
            self.log_file_handle.write(log_buffer)

    def parse_started_reply(self, reply, node_name):
        if len(reply) >= 2:
            try:
                self.logs_socket.connect(reply[1])
                self.logs_socket.setsockopt(zmq.SUBSCRIBE, constants.LOGS)
                self.logger.info("Connected to log %s node data: %s" % (node_name, reply[1]))
            except:
                self.logger.warn("Logs will not be saved for node  %s in  %s. "
                                 "Logging connection couldn't be established." % (node_name, self.log_file_name))

    def set_log_path(self, path):
        self.logger.warning("Logs will be stored in file: %s" % self.log_file_name)
        self.log_file_path = path
        self.log_file_handle = open(self.log_file_path + self.log_file_name, "w+")

    def set_frames(self, totalSimTime, frameTime):
        self.frame_length = frameTime
        self.num_frames = int(totalSimTime / frameTime)

    def run(self):
        self.logger.info('STARTING CHILDREN')
        self.initializeAllProcesses()
        self.logger.info('COLLECTING NODE SUBS')
        self.collectSubscribers()
        self.logger.info('COLLECTING NODE PUBS')
        self.collectPublishers()
        self.logger.info('CONTROLLER FRONTEND')
        self.subscribe_frontend_to_all_messages()
        self.logger.info('NODE HANDSHAKES')
        self.handshake_all_nodes()
        self.logger.info('RUNNING CHILD PROCESSES')
        for i in range(0, self.num_frames):
            self.logger.info("Controller stepping for dt=%s. Accumulated SimTime = %s seconds" % (
                self.frame_length, i * self.frame_length))
            self.advanceClock()
        self.logger.info('POSTPROCESSING CHILD PROCESSES')
        self.stopAllProcesses()
        self.shutdown_controller()

    def shutdown_controller(self):
        self.logger.info("Shutting down controller")
        if self.log_file_handle:
            self.log_file_handle.close()

    def sig_handler(self, sig, frame):
        self.logger.warning('Process: {} , caught signal: {}'.format(self.name, sig))
        self.shutdown_controller()

    def handshake_all_nodes(self):
        nodeHandshakes = self.nodeReqSockets.copy()
        self.frontend.RCVTIMEO = 2000  # [milliseconds]
        while any(nodeHandshakes):
            for nodeName, socket in nodeHandshakes.items():
                self.logger.info("Sending HANDSHAKE to node %s." % (nodeName))
                socket.send(constants.HANDSHAKE)
                try:
                    handshake_msg = self.frontend.recv()
                    nodeHandshakes.pop(nodeName)
                    self.logger.info("Received publication %s message from node %s." % (handshake_msg, nodeName))
                except:
                    self.logger.info('Will try handshaking again node: %s' % nodeName)
                reply = socket.recv_multipart()
                self.logger.info("Received reply %s message from node %s." % (reply, nodeName))
        self.frontend.RCVTIMEO = -1



    def collectPublishers(self):
        """
            For every message for which there is a subscriber node in the controller's record,
            request the remaining non-subscribed nodes if they have a match to become publishers.
        """
        for nodeName, socket in self.nodeReqSockets.items():
            local_record = [constants.MATCH_MSGS]
            for msg_name, exchange_obj in self.dataExchangeRecord.items():
                if not (exchange_obj.is_node_subscribed(nodeName)):
                    local_record.append(msg_name)
            socket.send_multipart(local_record)
            self.logger.warning("Sending match list to node %s." % nodeName)

        for nodeName, socket in self.nodeReqSockets.items():
            reply = socket.recv_multipart()
            self.logger.warning("Received from node %s matched list = %s" % (nodeName, reply))
            py_list = self.convert_list_zmq2py(reply)
            for msg_name in py_list:
                if msg_name in self.dataExchangeRecord:
                    self.dataExchangeRecord[msg_name].add_publisher(nodeName)
                else:
                    self.logger.warning('Node %s is trying to publish message_name = %s, which nobody has asked for. '
                                        'Controller will ignore this msg.' % (nodeName,msg_name))

        return

    def parse_tock_reply(self, reply):  # reply[0] = "TOCK"
        self.total_pub_msg_next_count += len(reply[1:])  # used in the frontend
        for msg in reply[1:]:
            self.total_sub_msg_next_list.append(msg)  # used in the backend
            # self.logger.debug("total_sub_msg_next_list: appending msg = %s" % msg)

    def update_backend_data(self):
        for node_name in self.sock_sub_msg_next.keys():
            self.sock_sub_msg_next[node_name] = 0
        for msg_name in self.total_sub_msg_next_list:  # total_sub_msg_next_set:
            subscriber_nodes = self.dataExchangeRecord[msg_name].msg_subscribers
            for node_name in subscriber_nodes:
                self.sock_sub_msg_next[node_name] += 1
            # self.logger.debug('subscriber_nodes to msg %s: %s.' % (msg_name, subscriber_nodes))
        # self.logger.debug('sub_sock_count = %s. ' % self.sock_sub_msg_next.items())
        return

    def prepare_tick_command(self, nodeName):
        tick_command = [str(self.frame_length), str(self.sock_sub_msg_next[nodeName])]
        return tick_command

    def prepare_start_command(self):
        start_command = [self.backend_protocol.get_connect_address(),
                         self.frontend_protocol.get_connect_address()]
        # self.logger.debug('Backend connect address: %s' % self.backend_protocol.connect_address)
        return start_command

    def advanceClock(self):
        for nodeName, socket in self.nodeReqSockets.iteritems():
            node_tick_command = self.prepare_tick_command(nodeName)
            socket.send_multipart([constants.TICK] + node_tick_command)
        # self.logger.debug('Sent all ticks')

        self.listen_at_frontend()
        self.publish_at_backend()

        self.total_pub_msg_next_count = 0
        self.total_sub_msg_next_list = []

        self.receive_logs()

        for nodeName, socket in self.nodeReqSockets.iteritems():
            reply = socket.recv_multipart()
            self.parse_tock_reply(reply)
            # self.logger.debug("Received reply %s message" % reply)
        # self.logger.debug("Received all TOCKS.")
        # self.logger.debug('total_pub_msg_next_count = %d' % self.total_pub_msg_next_count)
        # self.logger.debug('total_sub_msg_next_set = %s' % self.total_sub_msg_next_set)
        self.update_backend_data()

    def create_node_request_socket(self, name, bind_address):
        reqSocket = self.context.socket(zmq.REQ)
        self.nodeReqSockets[name] = reqSocket
        reqSocket.bind(bind_address)
        self.logger.info('Creating node command socket %s. New binding to %s' % (name, bind_address))
        self.sock_sub_msg_next[name] = 0

    def collectSubscribers(self):
        """
            Request the message subscriptions from all nodes.
        """
        for nodeName, socket in self.nodeReqSockets.items():
            self.logger.warning('Sending collectSubscribers message to node %s' % nodeName)
            socket.send(constants.UNKNOWN_MSGS)

        for nodeName, socket in self.nodeReqSockets.items():
            reply = socket.recv_multipart()
            self.logger.warning("Received reply from node %s. Message = %s." % (nodeName, reply))
            py_list = self.convert_list_zmq2py(reply)
            for msg_name in py_list:
                if not (msg_name in self.dataExchangeRecord):
                    self.dataExchangeRecord[msg_name] = CentralDataExchangeObject()
                self.dataExchangeRecord[msg_name].add_subscriber(nodeName)

    def initializeAllProcesses(self):
        """
            Request all nodes to initialize their simulation.
        """
        start_command = self.prepare_start_command()
        for node_name, socket in self.nodeReqSockets.iteritems():
            self.logger.info("Sending %s command to node %s." % (constants.START, node_name))
            socket.send_multipart([constants.START] + start_command)
        for node_name, socket in self.nodeReqSockets.iteritems():
            reply = socket.recv_multipart()
            self.parse_started_reply(reply, node_name)

    def stopAllProcesses(self):
        """
            Request internal result plots from all nodes.
        """
        for nodeName, socket in self.nodeReqSockets.items():
            socket.send(constants.FINISH)
        for nodeName, socket in self.nodeReqSockets.items():
            reply = socket.recv()
            self.logger.info("Received reply %s message" % reply)

    def restartChildProcess(self):
        self.logger.info('restartChildProcess(self)')

    def convert_list_zmq2py(self, zmq_list):
        zmq_list.pop(0)
        return zmq_list

    # def listen_for_nodes(self):


def add_arg_definitions(parser):
    parser.add_argument('--host_name', nargs='?', default="127.0.0.1", required=True,
                        help='IP address for controller')
    parser.add_argument('--sim_time', nargs='?', default="0.0", required=True,
                        help='Duration in seconds of the simulation')
    parser.add_argument('--sim_frame_time', nargs='?', default="0.1",
                        help='BL time step in seconds')
    parser.add_argument('--nodes', nargs="*", default="",
                        help='Order list of N number of BL nodes '
                             '[node_1_name node_1_address ... node_N_name node_N_address ]')
    parser.add_argument('--verbosity_level', nargs='?', default="",
                        help='Verbosity level of the Central Controller logger')
    parser.add_argument('--logging_path', nargs='?', default="",
                        help='Path to directory in to which BL logging files are written')


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description='EMM Simulation Standalone Test.')
    add_arg_definitions(arg_parser)
    parsed_args, unknown_args = arg_parser.parse_known_args()

    if unknown_args:
        warnings.warn("Unrecognised args parsed: %s" % unknown_args, RuntimeWarning)

    # controller requirements: host address & sim times
    host_name = "127.0.0.1"
    sim_time = 0.0
    sim_frame_time = 0.1
    verbosity_level = "DEBUG"
    if parsed_args.host_name:
        host_name = parsed_args.host_name
    if parsed_args.sim_time:
        sim_time = float(parsed_args.sim_time)
        print("controller sim_time = ", sim_time)
    if parsed_args.sim_frame_time:
        sim_frame_time = float(parsed_args.sim_frame_time)
        print("controller sim_frame_time = ", sim_frame_time)
    if parsed_args.verbosity_level:
        verbosity_level = parsed_args.verbosity_level
        print("controller logger verbosity_level = ", parsed_args.verbosity_level)

    controller = MasterController(host_name=host_name, verbosity_level=verbosity_level)
    controller.set_frames(sim_time, sim_frame_time)

    # controller additional options:
    if parsed_args.logging_path:
        logging_path = parsed_args.logging_path
        print("controller logging_path = ", logging_path)
        controller.set_log_path(logging_path)

    nodes = []
    if parsed_args.nodes:
        tmp = parsed_args.nodes
        # grab node name and address from string as two piece chunks
        # and put them in the nodes list as lists themselves
        pieces = tmp[0].split()
        if len(pieces) % 2 is not 0:
            raise RuntimeError("Incorrect number (%d) of node parameters."
                               " Node parameters come in pairs and should be an even number." % len(pieces))
        for i in range(0, len(pieces), 2):
            chunk = pieces[i:i + 2]
            nodes.append(chunk)

    for i in range(0, len(nodes)):
        node_name = nodes[i][0]
        bind_address = nodes[i][1]
        controller.create_node_request_socket(node_name, bind_address)

    print("Controller:STARTING")
    controller.run()
