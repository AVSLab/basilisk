import sys, os, inspect
import argparse, warnings
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../../fswAuto/fswExamples/')
from desktopFswSim import DesktopFSW
import bskRouter, bskWorkerProcess


class fswScenario:
    def __init__(self, theSimClass):
        self.theSim = theSimClass()
        # Create Router process on top of the Sim
        fswRoutingRate = int(5E8)
        self.theSim.RouterProcessName = "RouterProcess"
        self.theSim.routerProc = self.theSim.CreateNewPythonProcess(self.theSim.RouterProcessName, 10)
        fswTarget = bskRouter.TargetProcess_class(fswRoutingRate, self.theSim.fswProc.Name)
        self.theSim.RouterClass = bskRouter.BSK_router_class(self.theSim, fswTarget)


def add_arg_definitions(parser):
    parser.add_argument('--master_address', nargs='?', required=True,
                        help='Address string to connect to the controller')
    parser.add_argument('--node_name', nargs='?', required=False, default="",
                        help='Address string to connect to the controller')
    parser.add_argument('--verbosity_level', nargs='?', default="",
                        help='Verbosity level of the BSK sim logger')


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description='BSK FSW Standalone Test.')
    add_arg_definitions(arg_parser)
    parsed_args, unknown_args = arg_parser.parse_known_args()

    if unknown_args:
        warnings.warn("Unrecognised args parsed: %s" % unknown_args, RuntimeWarning)

    node_name = "BSK_FSWSim"
    if parsed_args.node_name:
        node_name = parsed_args.node_name

    verbosity_level = "DEBUG"
    if parsed_args.verbosity_level:
        verbosity_level = parsed_args.verbosity_level

    master_address = ""
    try:
        master_address = parsed_args.master_address
    except ValueError('Node %s needs to know the master address' % node_name):
        print("FAULT")

    # Execute
    scenario = fswScenario(DesktopFSW)
    fsw_process = bskWorkerProcess.BskSim(name=node_name, proc_args=[scenario],
                                          master_address=master_address, verbosity_level=verbosity_level)

    print("Node %s: STARTING " % node_name)
    fsw_process.run()

