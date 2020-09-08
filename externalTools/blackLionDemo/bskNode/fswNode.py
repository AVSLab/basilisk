import warnings
import argparse
from Basilisk.utilities import SimulationBaseClass
from bskWorkerProcess import BskSim
import fswModels
import bskRouter
import simPlotting as BSK_plt


class BSK_FSWSim(SimulationBaseClass.SimBaseClass):
    def __init__(self, plot_results=True, mode_request=None):
        self.plot_results = plot_results
        self.modeRequest = mode_request
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)
        # FSW process
        self.FSWProcessName = "FSWProcess"
        self.fswProc = self.CreateNewProcess(self.FSWProcessName)
        self.BSKFsw = fswModels.BSKFsw(self)
        # Router process
        self.RouterProcessName = "RouterProcess"
        self.routerProc = self.CreateNewPythonProcess(self.RouterProcessName)
        FSWTarget = bskRouter.TargetProcess_class(self.BSKFsw.processTasksTimeStep, self.FSWProcessName)
        self.RouterClass = bskRouter.BSK_router_class(self, FSWTarget)

    def set_mode_request(self, sim_time_nanos):
        pass

    def log_outputs(self):
        samplingTime = self.BSKFsw.processTasksTimeStep
        self.TotalSim.logThisMessage(self.BSKFsw.trackingErrorData.outputDataName, samplingTime)
        self.TotalSim.logThisMessage(self.BSKFsw.mrpFeedbackData.outputDataName, samplingTime)

    def pull_outputs(self, path):
        def print_local_outputs(sigma_RN, sigma_BR, Lr):
            print('sigma_RN = %s' % sigma_RN[-3:, 1:])
            print('sigma_BR = %s' % sigma_BR[-3:, 1:])
            print('Lr = %s' % Lr[-3:, 1:])
            print('t_sim_end = %s' % sigma_RN[-1:, 0])

        def plot_local_outputs(sigma_RN, sigma_BR, Lr):
            BSK_plt.plot_trackingError(sigma_BR, omega_BR_B)
            BSK_plt.save_plot_in_path(path, "fsw_track_error")
            BSK_plt.plot_attitudeGuidance(sigma_RN, omega_RN_N)
            BSK_plt.save_plot_in_path(path, "fsw_guidance")

        sigma_RN = self.pullMessageLogData(self.BSKFsw.trackingErrorData.inputRefName + ".sigma_RN", range(3))
        omega_RN_N = self.pullMessageLogData(self.BSKFsw.trackingErrorData.inputRefName + ".omega_RN_N", range(3))
        sigma_BR = self.pullMessageLogData(self.BSKFsw.trackingErrorData.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.pullMessageLogData(self.BSKFsw.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
        Lr = self.pullMessageLogData(self.BSKFsw.mrpFeedbackData.outputDataName + ".torqueRequestBody", range(3))

        print_local_outputs(sigma_RN, sigma_BR, Lr)
        if self.plot_results:
            plot_local_outputs(sigma_RN, sigma_BR, Lr)

    def configure_initial_conditions(self):
        self.modeRequest = 'inertial3D'


def add_arg_definitions(parser):
    parser.add_argument('--master_address', nargs='?', required=True,
                        help='Address string to connect to the controller')
    parser.add_argument('--node_name', nargs='?', required=False, default="",
                        help='Address string to connect to the controller')
    parser.add_argument('--verbosity_level', nargs='?', default="",
                        help='Verbosity level of the BSK sim logger')


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description='EMM Simulation Standalone Test.')
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
    BSKFsw_process = BskSim(name=node_name,
                            proc_args=[BSK_FSWSim(plot_results=True)],
                            master_address=master_address,
                            verbosity_level=verbosity_level)

    print("Node %s: STARTING " % node_name)
    BSKFsw_process.run()

