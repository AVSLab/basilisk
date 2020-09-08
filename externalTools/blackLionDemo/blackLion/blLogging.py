from copy import copy
import logging
import logging.handlers
import zmq
from zmq.log.handlers import PUBHandler
import socket
try:
    from utilities import TcpProtocol, IpcProtocol
except:
    from blackLion.utilities import TcpProtocol, IpcProtocol


def add_logger_pub(name, transport_type, port=""):
    """
    method add a logger functionality
    """
    logger = logging.getLogger(name)
    protocol = None
    try:
        ctx = zmq.Context()
        pub_socket = ctx.socket(zmq.PUB)

        if transport_type == "tcp":
            try:
                host_name = socket.gethostbyname(socket.gethostname())
            except:
                host_name = 'localhost'

            protocol = TcpProtocol(host_name)
            if port == "":
                port = pub_socket.bind_to_random_port(protocol.get_bind_address())
                protocol.port = str(port)
            else:
                protocol.port = port

        elif transport_type == "ipc":
            protocol = IpcProtocol("tmp/" + name + "_logger_pub", absolute=True)
            pub_socket.bind(protocol.get_bind_address())

        # pub_socket.connect(protocol.get_connect_address())
        pub_handler = PUBHandler(pub_socket)
        pub_handler.root_topic = "LOGS"
        formatter = logging.Formatter("[%(levelname)s %(asctime)s.%(msecs)03d]  %(message)s")
        pub_handler.setFormatter(formatter)

        # override the default formatters in the PubHandler
        formatters = {
            logging.DEBUG: logging.Formatter(
                "%(levelname)s %(name)s %(asctime)s.%(msecs)03d]  %(message)s (%(filename)s:%(lineno)d)"),
            logging.INFO: logging.Formatter(
                "%(levelname)s %(name)s %(asctime)s.%(msecs)03d]  %(message)s (%(filename)s:%(lineno)d)"),
            logging.WARN: logging.Formatter(
                "%(levelname)s %(name)s %(asctime)s.%(msecs)03d]  %(message)s (%(filename)s:%(lineno)d)"),
            logging.ERROR: logging.Formatter(
                "%(levelname)s %(name)s %(asctime)s.%(msecs)03d]  %(message)s (%(filename)s:%(lineno)d) %(exc_info)s"),
            logging.CRITICAL: logging.Formatter(
                "%(levelname)s %(name)s %(asctime)s.%(msecs)03d]  %(message)s (%(filename)s:%(lineno)d)")}
        pub_handler.formatters = formatters

        memory_handler = logging.handlers.MemoryHandler(1024 * 10, flushLevel=logging.DEBUG, target=pub_handler)
        logger.addHandler(memory_handler)

    except zmq.error.ZMQError:
        pub_handler = None

    return protocol


def createLogger(name, verbosity_level="DEBUG", verbose=False):
    logger = logging.getLogger(name)
    try:
        level = eval("logging." + verbosity_level)
        print("%s verbosity_level is %s" % (name, verbosity_level))
    except:
        level = logging.DEBUG
        print("%s verbosity_level is DEBUG" % name)
    logger.setLevel(level)
    logger.propagate = False
    console_handler = logging.StreamHandler()

    if verbose:
        console_formatter = ColoredFormatter(
            "[%(levelname)s %(name)s %(asctime)s.%(msecs)03d]  %(message)s (%(filename)s:%(lineno)d)", "%I:%M:%S")
    else:
        console_formatter = ColoredFormatter(
            "[%(levelname)s %(name)s %(asctime)s.%(msecs)03d]  %(message)s", "%I:%M:%S")

    console_handler.setFormatter(console_formatter)

    logger.addHandler(console_handler)

    return logger


MAPPING = {
    'DEBUG': 37,  # white
    'INFO': 36,  # cyan
    'WARNING': 33,  # yellow
    'ERROR': 31,  # red
    'CRITICAL': 41,  # white on red bg
}

PREFIX = '\033['
SUFFIX = '\033[0m'


class ColoredFormatter(logging.Formatter):

    def __init__(self, *args):
        logging.Formatter.__init__(self, *args)

    def format(self, record):
        record = copy(record)
        level_name = record.levelname
        module = record.module
        name = record.name
        msg = record.msg

        seq = MAPPING.get(level_name, 37)  # default white

        colored_levelname = '{0}{1}m{2}{3}'.format(PREFIX, seq, level_name, SUFFIX)
        colored_module = '{0}{1}m{2}{3}'.format(PREFIX, seq, module, SUFFIX)
        colored_name = '{0}{1}m{2}{3}'.format(PREFIX, seq, name, SUFFIX)
        colored_msg = '{0}{1}m{2}{3}'.format(PREFIX, seq, msg, SUFFIX)

        record.levelname = colored_levelname
        record.module = colored_module
        record.name = colored_name
        record.msg = colored_msg

        return logging.Formatter.format(self, record)
