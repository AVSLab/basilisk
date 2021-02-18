import zmq
import sys, os
from zmq.eventloop import ioloop

from Basilisk.utilities import SimulationBaseClass

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(path + '/../')
from blackLion import workerProcess
from blackLion.message_processing import convert_str_list_into_bytes


class BskSim(workerProcess.WorkerProcess):
    def __init__(self, name, proc_args, master_address, verbosity_level):
        super(BskSim, self).__init__(name, proc_args, master_address, verbosity_level)
        self.scSim = proc_args[0]

    def step_process(self):
        self.scSim.RouterClass.router.currentDataExchangeOut = {}

        self.scSim.set_mode_request(int(self.accum_sim_time*1E9))
        self.scSim.ConfigureStopTime(int(self.accum_sim_time*1E9))
        self.scSim.ExecuteSimulation()

    def finish_process(self):
        plots_path = os.path.dirname(os.path.abspath(__file__)) + "/../plots"
        self.scSim.pull_outputs(plots_path)

    def run(self):
        self.context = zmq.Context()
        self.init_command_socket()
        ioloop.IOLoop.instance().start()

    def initialize(self):
        SimulationBaseClass.SimBaseClass.InitializeSimulation(self.scSim)
        self.scSim.set_mode_request(0)
        self.scSim.log_outputs()
        self.scSim.configure_initial_conditions()

    def match_published_msgs(self, message_names):
        localPublications = list(self.scSim.RouterClass.router.match_externalRouteMessages(message_names))
        return localPublications

    def get_unpublished_msgs(self):
        localSubscriptions = list(self.scSim.RouterClass.router.collect_localRouteMessages())
        return localSubscriptions

    def next_pub_update(self):
        return list(self.scSim.RouterClass.router.currentDataExchangeOut.keys())

    def publish(self):
        for msg_name, exchange_obj in self.scSim.RouterClass.router.currentDataExchangeOut.items():
            zmq_message = [msg_name, exchange_obj.packet_type, str(exchange_obj.payload_size)]
            zmq_message = convert_str_list_into_bytes(zmq_message)
            zmq_message.append(exchange_obj.payload)
            self.pub_socket.send_multipart(zmq_message)
            #print 'BSK node just published zmq_message = %s' % msg_name


    def route_message_in(self, msg_name, packet_type, payload_size, payload_byte_list):
        #print 'Routing message in: %s %s %s' % (msg_name, payload_size, payload_byte_list)
        self.scSim.RouterClass.router.routeMessage(self.accum_sim_time, msg_name, payload_size, payload_byte_list)