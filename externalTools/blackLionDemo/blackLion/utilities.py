from __future__ import print_function
from subprocess import Popen
from multiprocessing import Process, set_start_method, Pipe

try:
    set_start_method("spawn")
except RuntimeError:
    pass
from .controller import run_controller
import os
import sys
import io
import random
import socket
from .protocol import TcpProtocol, IpcProtocol

sys.path.append("..")
from bskNode.node_factory import NodeFactory

try:
    from Queue import Queue, Empty
except ImportError:
    from queue import Queue, Empty  # python 3.x


class UniqueDict(dict):
    def __setitem__(self, key, value):
        if key not in self:
            dict.__setitem__(self, key, value)
        else:
            raise KeyError("Key already exists")


class ApplicationSettings():
    def __init__(self):
        self.scenarioClass = None
        self.path = "/"


class BlacklionBash(object):
    def __init__(self, file_name, path, exec_type='bash'):
        self.exec_type = exec_type
        self.file_name = file_name
        self.path = path


class BlacklionNode(object):
    """
        A class to manage parameters associated with a node executable

        Parameters
        ----------
        exec_name : string
            name as used to run the executable
        arg_list : string
            list of arguments to pass the executable
        config_file : string
            a config file or script to pass the executable such as a python script
        path : string
    """

    def __init__(self, name, protocol):
        self.name = name
        self.exec_name = None
        self.config_file = None
        self.path = None
        self.protocol = protocol
        self.should_auto_execute = True
        self._argsOrd = dict()

    def add_argsOrd(self, key, value):
        self._argsOrd[key] = value

    def get_argsOrd(self):
        return self._argsOrd

    def get_port(self):
        return self.protocol.port


class TcpPortHandler(object):
    def __init__(self, max_num_nodes=20):
        self.population_size = max_num_nodes
        self.lower_bound = 49152
        self.upper_bound = 65535
        self.tcp_ports_avail = self.create_unique_ports()

    def create_unique_ports(self):
        try:  # Python 2
            tcp_ports_avail = random.sample(xrange(self.lower_bound, self.upper_bound), self.population_size)
        except:  # Python 3
            tcp_ports_avail = random.sample(range(self.lower_bound, self.upper_bound), self.population_size)
        return tcp_ports_avail

    def get_unique_port_string(self):
        port = self.tcp_ports_avail.pop()
        str_port = str(port)
        return str_port


class Launcher(object):
    def __init__(self):
        self.ON_POSIX = 'posix' in sys.builtin_module_names
        self.input_fd, self.output_fd = Pipe()  # create a pipe to get data
        self.file_address_record = {}
        self.exec_nodes = []
        self.controller_process = None
        self.processes = list()

    def launch_controller(self, controller_args):
        print("\nLAUNCHER: launch_controller")
        try:
            self.controller_process = Process(target=run_controller, args=(controller_args,))
            self.controller_process.start()
        except:
            print("Controller Failed to Launch")

    def execute_all_processes(self, nodes, bashes):
        print("\nLAUNCHER: execute_all_processes")
        # start several subprocesses


        for (name, node) in nodes.items():
            print("Spawning %s node" % node.name)
            process = NodeFactory.create(name, node, self.output_fd)
            process.start()
            print("%s is running as process %s " % (name, process.name))
            self.processes.append(process)

        for bash in bashes:
            print("Spawning new bash")
            params = [bash.exec_type, bash.path + bash.file_name]
            # print("bash_params = ", params)
            self.processes.append(
                Popen(params, stdout=self.output_fd, close_fds=self.ON_POSIX, cwd=bash.path))

        #os.close(self.output_fd)  # close unused end of the pipe
        # read output line by line as soon as it is available

        with io.open(self.input_fd.fileno(), 'r', buffering=1) as file:
            for line in file:
                print(line, end='')

    def wait_for_child_process(self):
        print("All the process are running , waiting for them to terminate")
        for process in self.processes:
            process.join()
        self.controller_process.join()


class Bootstrapper(object):
    def __init__(self, sim_time=20.0, frame_time=0.1, enable_logs=True, verbosity_level=None):
        self.nodes = UniqueDict()
        self.bashes = list()
        self.multi_launcher = Launcher()
        self.tcp_handler = TcpPortHandler()
        try:
            self.host_name = socket.gethostbyname(socket.gethostname())
        except:
            self.host_name = "127.0.0.1"
        self.sim_time = sim_time
        self.sim_frame_time = frame_time
        self.logs_path = None
        if enable_logs:
            path = os.path.dirname(os.path.abspath(__file__))
            log_dir = path + "/../logs/"
            if not os.path.exists(log_dir):
                print("Creating new '../logs' directory")
                os.makedirs(log_dir)
            self.logs_path = log_dir
        self.verbosity_level = verbosity_level

    def add_bash(self, file_name, path, exec_type='bash'):
        bash = BlacklionBash(file_name, path, exec_type)
        self.bashes.append(bash)

    def add_python_node(self, name, file_name, path, protocol_type="tcp", port=None, ip_address=None):
        self.add_executable_node("python", name, path, protocol_type, port, ip_address, file_name=file_name)

    def add_executable_node(self, exec_name, name, path, protocol_type="tcp", port=None, ip_address=None,
                            file_name=None):
        if protocol_type == "tcp":
            node = self.create_tcp_node(name)
            if ip_address is not None:
                node.protocol.ip_address = str(ip_address)

        elif protocol_type == "ipc":
            node = self.create_ipc_node(name, port)

        else:
            raise (TypeError("Unknown type %s" % protocol_type))

        node.exec_name = exec_name
        node.name = name
        if file_name is not None:
            node.config_file = path + file_name
        node.path = path
        self.nodes[name] = node

    def create_tcp_node(self, name):
        port = self.tcp_handler.get_unique_port_string()
        protocol = TcpProtocol(self.host_name, port)
        node = BlacklionNode(name, protocol)
        return node

    def create_ipc_node(self, name, port):
        ipc = IpcProtocol(port)
        node = BlacklionNode(name, ipc)
        return node

    def package_controller_args(self):
        controller_args = dict()

        controller_args['hostname'] = self.host_name
        controller_args['nodes'] = self.nodes
        controller_args['sim_time'] = self.sim_time
        controller_args['sim_frame_time'] = self.sim_frame_time

        nodes = list()
        for node_name, node in self.nodes.items():
            nodes.append([node_name, node.protocol.get_bind_address()])
        controller_args['nodes'] = nodes
        """
        node_list = ""
        for node_name, node in self.nodes.items():
            node_list += "%s %s " % (node_name, node.protocol.get_bind_address())
    
        controller_args.append("--nodes=%s" % node_list)
        controller_args.append("--host_name=%s" % self.host_name)
        controller_args.append("--sim_time=%s" % self.sim_time)
        controller_args.append("--sim_frame_time=%s" % self.sim_frame_time)
        """
        if self.logs_path:
            controller_args['logging_path'] = self.logs_path
            # controller_args.append("--logging_path=%s" % self.logs_path)
        if self.verbosity_level:
            controller_args['verbosity'] = self.verbosity_level
            # controller_args.append("--verbosity_level=%s" % self.verbosity_level)
        return controller_args

    def run(self):
        self.multi_launcher.launch_controller(self.package_controller_args())
        launcher_nodes = {}
        for node_name, node in self.nodes.items():
            if node.should_auto_execute is True:
                launcher_nodes[node_name] = node
        self.multi_launcher.execute_all_processes(launcher_nodes, self.bashes)
        self.multi_launcher.wait_for_child_process()
