import warnings
import argparse
from Basilisk.utilities import SimulationBaseClass, orbitalMotion, macros, unitTestSupport
from bskWorkerProcess import BskSim
import dynModels
import bskRouter
import simPlotting as BSK_plt


class BSK_DynSim(SimulationBaseClass.SimBaseClass):
    def __init__(self, plot_results=True):
        self.plot_results = plot_results
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)
        # Dynamics process
        self.DynamicsProcessName = "DynamicsProcess"
        self.dynProc = self.CreateNewProcess(self.DynamicsProcessName)
        self.DynClass = dynModels.BSKDynamics(self)
        # Router process
        self.RouterProcessName = "RouterProcess"
        self.routerProc = self.CreateNewPythonProcess(self.RouterProcessName)
        DynTarget = bskRouter.TargetProcess_class(self.DynClass.processTasksTimeStep, self.DynamicsProcessName)
        self.RouterClass = bskRouter.BSK_router_class(self, DynTarget)

    def set_mode_request(self, sim_time_nanos):
        pass

    def log_outputs(self):
        samplingTime = self.DynClass.processTasksTimeStep
        self.TotalSim.logThisMessage(self.DynClass.scObject.scStateOutMsgName, samplingTime)
        self.TotalSim.logThisMessage(self.DynClass.simpleNavObject.outputAttName, samplingTime)

    def pull_outputs(self, path):
        def print_local_outputs(sigma_BN, omega_BN_B):
            print('sigma_BN = %s \n' % sigma_BN[-3:, 1:])
            print('omega_BN_B = %s \n' % omega_BN_B[-3:, 1:])
            print('t_sim_end = %s \n' % sigma_BN[-1:, 0])

        def plot_local_outputs(sigma_BN, omega_BN_B):
            print("Plotting results.")
            BSK_plt.plot_rotationalNav(sigma_BN, omega_BN_B)
            BSK_plt.save_plot_in_path(path, "sim_nav")

        sigma_BN = self.pullMessageLogData(self.DynClass.simpleNavObject.outputAttName + ".sigma_BN", range(3))
        omega_BN_B = self.pullMessageLogData(self.DynClass.simpleNavObject.outputAttName + ".omega_BN_B", range(3))

        print_local_outputs(sigma_BN, omega_BN_B)
        if self.plot_results:
            plot_local_outputs(sigma_BN, omega_BN_B)

    def configure_initial_conditions(self):
        oe = orbitalMotion.ClassicElements()
        oe.a = 10000.0 * 1000.0  # meters
        oe.e = 0.2
        oe.i = 0.0 * macros.D2R
        oe.Omega = 0.0 * macros.D2R
        oe.omega = 0.0 * macros.D2R
        oe.f = 280.0 * macros.D2R

        mu = self.DynClass.earthGravBody.mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        # Initialize Spacecraft States within the state manager.
        posRef = self.DynClass.scObject.dynManager.getStateObject("hubPosition")
        velRef = self.DynClass.scObject.dynManager.getStateObject("hubVelocity")
        sigmaRef = self.DynClass.scObject.dynManager.getStateObject("hubSigma")
        omegaRef = self.DynClass.scObject.dynManager.getStateObject("hubOmega")
        posRef.setState(unitTestSupport.np2EigenVectorXd(rN))  # r_BN_N [m]
        velRef.setState(unitTestSupport.np2EigenVectorXd(vN))  # v_BN_N [m]
        sigmaRef.setState([[0.1], [0.2], [-0.3]])  # sigma_BN
        omegaRef.setState([[0.001], [-0.01], [0.03]])  # omega_BN_B [rad/s]


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

    node_name = "BSK_DynSim"
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

    BSKSim_process = BskSim(name=node_name,
                            proc_args=[BSK_DynSim(plot_results=True)],
                            master_address=master_address,
                            verbosity_level=verbosity_level)

    print("Node %s: STARTING " % node_name)
    BSKSim_process.run()
