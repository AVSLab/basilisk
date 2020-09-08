from blackLion.utilities import Bootstrapper
import os


class BSK_Bootstrapper(Bootstrapper):
    def __init__(self, sim_time=20.0, frame_time=0.1, enable_logs=True, verbosity_level=None):
        super(BSK_Bootstrapper, self).__init__(sim_time=sim_time, frame_time=frame_time, enable_logs=enable_logs,
                                               verbosity_level=verbosity_level)

    def add_external_node(boot_strap, node_name):
        node = boot_strap.create_tcp_node(node_name)
        node.should_auto_execute = False
        node.add_argsOrd(["--master_address=%s" % node.protocol.get_connect_address()])
        boot_strap.nodes[node_name] = node
        print('Launch %s. connect_address = %s' % (node.name, node.protocol.get_connect_address()))

    def add_local_node(boot_strap, node_name, file_name, path):
        boot_strap.add_python_node(name=node_name, file_name=file_name, path=path)
        node = boot_strap.nodes[node_name]
        node.add_argsOrd(["--node_name=%s" % node.name])
        node.add_argsOrd(["--master_address=%s" % node.protocol.get_connect_address()])
        node.add_argsOrd({"--verbosity_level": boot_strap.verbosity_level})


def launch_bsk_sims(boot_strap, local_path):
    """
    Launch the Basilisk simulation
    """
    #boot_strap.add_local_node(node_name="bsk_dyn_node", file_name="dynNode.py", path=local_path + "/bskNode/")
    boot_strap.add_local_node(node_name="bsk_fsw_node", file_name="fsWNode.py", path=local_path + "/bskNode/")
    boot_strap.add_local_node(node_name="py_listener", file_name="listener.py", path=local_path + "/listenerNode/")


if __name__ == "__main__":
    boot_strap = BSK_Bootstrapper(sim_time=600.0, frame_time=0.1, verbosity_level="DEBUG", enable_logs=False)
    local_path = os.path.dirname(os.path.abspath(__file__))
    launch_bsk_sims(boot_strap, local_path)
    boot_strap.run()
