import multiprocessing
import zmq
import time
import signal
from zmq.eventloop import ioloop, zmqstream

try:
    import constants, blLogging
except:
    from blackLion import constants, blLogging

ioloop.install()


class WorkerProcess(multiprocessing.Process):
    def __init__(self, name, proc_args, master_address, verbosity_level="DEBUG"):
        multiprocessing.Process.__init__(self, None, self.run, name, proc_args)

        self.context = None
        self.rep_socket = None
        self.stream_rep = None
        self.master_address = master_address
        self.master_protocol = None
        self.accum_sim_time = 0.0
        self.frame_time = 0.0
        self.frontend_address = ""
        self.backend_address = ""
        self.pub_socket = None
        self.sub_socket = None
        self.sub_msg_next_count = 0

        signal.signal(signal.SIGTERM, self.sig_handler)
        signal.signal(signal.SIGINT, self.sig_handler)
        self.logger_protocol = None
        self.logger = blLogging.createLogger(self._name, verbosity_level)

    def init_command_socket(self):
        self.rep_socket = self.context.socket(zmq.REP)
        self.rep_socket.connect(self.master_address)
        self.stream_rep = zmqstream.ZMQStream(self.rep_socket)
        self.stream_rep.on_recv(self.parse_command)

    def init_pub_socket(self):
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.connect(self.frontend_address)
        # let's piggy back logger publish here for now
        # because when a worker process is run as a python
        # multiprocessing.Process we must create the pub socket
        # within the zmq.Context() created after the process is forked
        # which is when the run() function is executed
        self.logger_protocol = blLogging.add_logger_pub(self._name, "tcp")

    def init_sub_socket(self):
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(self.backend_address)

    def parse_tick_message(self, msg):
        self.frame_time = float(msg[1])
        self.sub_msg_next_count = int(msg[2])
        #self.logger.info('Next sub msg count = %d' % self.sub_msg_next_count)

    def parse_start_message(self, msg):
        self.backend_address = msg[1]
        self.frontend_address = msg[2]
        self.init_sub_socket()
        self.init_pub_socket()

    def subscribe(self):
        # self.logger.info('Sub socket listening. Expecting %d messages.' % self.sub_msg_next_count)
        local_msg_count = 0
        while not (local_msg_count == self.sub_msg_next_count):
            subscription = self.sub_socket.recv_multipart()
            # self.logger.info('Process %s received subscribed message: %s' % (self.process_name, subscription))
            self.parse_incoming_packet(subscription)
            local_msg_count += 1
        # self.logger.info('Sub socket closing.')

    def handshake_frontend(self):
        self.pub_socket.send(constants.HANDSHAKE)
        self.logger.info('Node has published msg = %s' % constants.HANDSHAKE)
        return

    def subscribe_all_messages(self, sub_list):
        # self.sub_socket.setsockopt(zmq.SUBSCRIBE, constants.HANDSHAKE)
        for msg_name in sub_list:
            # self.logger.info('Process %s subscribing to msg = %s.' % (self.process_name, msg_name))
            self.sub_socket.setsockopt(zmq.SUBSCRIBE, msg_name)
        return

    def parse_incoming_packet(self, subscribed_zmq_msg):
        # self.logger.info('Receiving zmq_msg = %s' % subscribed_zmq_msg)
        msg_name = subscribed_zmq_msg[0]
        packet_type = subscribed_zmq_msg[1]
        payload_size = int(subscribed_zmq_msg[2])
        payload = subscribed_zmq_msg[3]
        self.route_message_in(msg_name, packet_type, payload_size, payload)
        if packet_type == constants.BSK:
            # Payload is hex string.
            # Do something, maybe.
            return
        elif (packet_type == constants.CCSDS or packet_type == '2'):
            # Payload is string of unsigned char.
            # Do something, maybe.
            return

        elif packet_type == constants.VIZ:
            # Payload is hex string.
            # Do something, maybe.
            return
        else:
            raise ValueError('Unrecognized message packet type.')

    def parse_command(self, msg):
        # self.logger.info("Received command: %s" % msg[0])
        if msg[0] == constants.TICK:
            self.parse_tick_message(msg)
            self.publish()
            self.subscribe()
            self.accum_sim_time += self.frame_time
            self.step_process()
            next_pub_list = self.next_pub_update()
            #self.logger.info('Next pub list = %s. ' % next_pub_list)
            self.stream_rep.send_multipart([constants.TOCK] + next_pub_list)

        elif msg[0] == constants.UNKNOWN_MSGS:
            sub_list = self.get_unpublished_msgs()
            #self.logger.info('sub_list = %s' % sub_list)
            self.subscribe_all_messages(sub_list)
            zmq_list = [msg[0]] + sub_list
            self.stream_rep.send_multipart(zmq_list)
            # self.stream_rep.send_multipart(self.convert_list_py2zmq(sub_list))

        elif msg[0] == constants.MATCH_MSGS:
            pub_list = self.match_published_msgs(msg[1:])
            zmq_list = [msg[0]] + pub_list
            # self.stream_rep.send_multipart(self.convert_list_py2zmq(pub_list))
            self.stream_rep.send_multipart(zmq_list)

        elif msg[0] == constants.START:
            self.parse_start_message(msg)
            self.initialize()
            self.stream_rep.send_multipart([constants.STARTED, self.logger_protocol.get_connect_address()])

        elif msg[0] == constants.FINISH:
            self.finish_process()
            self.stream_rep.send(constants.FINISHED)
            self.shutdown_process()

        elif msg[0] == constants.HANDSHAKE:
            self.stream_rep.send(constants.HANDSHAKEN)
            self.handshake_frontend()

        else:
            raise ValueError("Unrecognized command: %s" % msg[0])

    def shutdown_process(self):
        instance = ioloop.IOLoop.instance()
        terminateTime = time.time() + constants.MAX_WAIT_SECONDS_BEFORE_SHUTDOWN

        def terminate():
            now = time.time()
            if now < terminateTime:
                instance.add_timeout(now + 1, terminate)
            else:
                instance.stop()
                self.logger.info('Shutting down process %s' % self.name)

        # terminate() function is recursive until terminate time is reached.
        terminate()

    def sig_handler(self, sig, frame):
        self.logger.warning('Process: {} , caught signal: {}'.format(self.name, sig))
        ioloop.IOLoop.instance().add_callback(self.shutdown_process)

    def match_published_msgs(self, message_names):
        """
            Developer must override this method in their WorkerProcess derived subclass.
        """
        pass

    def get_unpublished_msgs(self):
        """
            Developer must override this method in their WorkerProcess derived subclass.
        """
        pass

    def next_pub_update(self):
        """
            Developer must override this method in their WorkerProcess derived subclass.
        """
        pass

    def publish(self):
        """
            Developer must override this method in their WorkerProcess derived subclass.
        """
        pass

    def route_message_in(self, msg_name, packet_type, payload_size, payload_byte_list):
        """
            Developer must override this method in their WorkerProcess derived subclass.
        """
        pass

    def route_incoming_message(self, subscribed_zmq_msg):
        """
            Developer must override this method in their WorkerProcess derived subclass.
        """
        pass

    def step_process(self):
        """
            Developer must override this method in their WorkerProcess derived subclass.
        """
        pass

    def finish_process(self):
        """
            Developer must override this method in their WorkerProcess derived subclass.
        """
        pass

    def initialize(self):
        """
            Developer must override this method in their WorkerProcess derived subclass.
        """
        pass

    def run(self):
        """
            Developer must override this method in their WorkerProcess derived subclass.
        """
        pass
