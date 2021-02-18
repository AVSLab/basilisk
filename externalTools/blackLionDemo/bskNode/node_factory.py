import sys
sys.path.append("..")
from listenerNode.listener import run_listener_node
from multiprocessing import Process
from .fswNode import run_fsw_node
from .dynNode import run_dyn_node


class NodeFactory:
    @staticmethod
    def create(name, node, out_pipe):
        if name == "py_listener":
            return Process(target=run_listener_node, args=(node.get_argsOrd(),))
        elif name == "bsk_fsw_node":
            return Process(target=run_fsw_node, args=(node.get_argsOrd(), out_pipe))
        elif name == "bsk_dyn_node":
            return Process(target=run_dyn_node, args=(node.get_argsOrd(),))
