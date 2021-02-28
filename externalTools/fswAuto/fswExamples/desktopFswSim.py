import sys, os
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(path + '/../')
from Basilisk.utilities import (SimulationBaseClass, macros)
from desktopFswModels import FSWModels
from fswPlotter import FSWPlotter


class DesktopFSW(SimulationBaseClass.SimBaseClass):
    """
        Class defining the FSW scenario
    """
    def __init__(self):
        SimulationBaseClass.SimBaseClass.__init__(self)  # Create a sim module as an empty container
        self.modeRequest = 'None'
        self.fswProc = self.CreateNewProcess("FSWProcess", 100)
        self.fswModels = FSWModels(self)
        self.fsw_rate = int(5E8)
        self.populateFswTasks()
        self.create_FSW_events()
        self.fsw_plotter = None

    def populateFswTasks(self):
        self.fswProc.addTask(self.CreateNewTask("initOnlyTask", int(1E10)), 200000)
        self.AddModelToTask("initOnlyTask", self.fswModels.VehConfigDataWrap, self.fswModels.VehConfigData, 2)
        self.AddModelToTask("initOnlyTask", self.fswModels.rwConfigWrap, self.fswModels.rwConfigData, 1)

        self.fswProc.addTask(self.CreateNewTask("inertial3DPointTask",  self.fsw_rate), 120)
        self.AddModelToTask("inertial3DPointTask", self.fswModels.inertial3DWrap, self.fswModels.inertial3DData, 20)
        self.AddModelToTask("inertial3DPointTask", self.fswModels.attTrackingWrap_base, self.fswModels.attTrackingData_base, 19)

        self.fswProc.addTask(self.CreateNewTask("feedbackControlTask",  self.fsw_rate), 110)
        self.AddModelToTask("feedbackControlTask", self.fswModels.MRP_FeedbackRWAWrap, self.fswModels.MRP_FeedbackRWAData, 10)
        self.AddModelToTask("feedbackControlTask", self.fswModels.rwMotorTorqueWrap, self.fswModels.rwMotorTorqueData, 9)

        self.fswProc.disableAllTasks()

    def create_FSW_events(self):
        self.createNewEvent("initiateInertialPoint",  self.fsw_rate, True, ["self.modeRequest == 'inertialPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('feedbackControlTask')",
                                "self.setEventActivity('initiateInertialPoint', True)"])

    def log_outputs(self):
        self.vcRec = self.fswModels.VehConfigData.vecConfigOutMsg.recorder()
        self.rwParamRec = self.fswModels.rwConfigData.rwParamsOutMsg.recorder()
        self.in3dRec = self.fswModels.inertial3DData.attRefOutMsg.recorder()
        self.attErrRec = self.fswModels.attTrackingData_base.attGuidOutMsg.recorder()
        self.cmdTorRec = self.fswModels.MRP_FeedbackRWAData.cmdTorqueOutMsg.recorder()
        self.rwMotRec = self.fswModels.rwMotorTorqueData.rwMotorTorqueOutMsg.recorder()

        self.AddModelToTask("inertial3DPointTask", self.vcRec)
        self.AddModelToTask("inertial3DPointTask", self.rwParamRec)
        self.AddModelToTask("inertial3DPointTask", self.in3dRec)
        self.AddModelToTask("inertial3DPointTask", self.attErrRec)
        self.AddModelToTask("inertial3DPointTask", self.cmdTorRec)
        self.AddModelToTask("inertial3DPointTask", self.rwMotRec)

    def pull_outputs(self, plots_path=None):
        self.fsw_plotter = FSWPlotter(plots_path=plots_path, add_titles=False, name="fsw")

        ISCPntB_B = self.vcRec.ISCPntB_B
        CoM_B = self.vcRec.CoM_B
        print("ISCPntB_B = ", ISCPntB_B)
        print("CoM_B = ", CoM_B)

        GsMatrix_B = self.rwParamRec.GsMatrix_B[:, range(3 * self.fswModels.numRW)]
        JsList = self.rwParamRec.JsList[:, range(self.fswModels.numRW)]
        uMax = self.rwParamRec.uMax[:, range(self.fswModels.numRW)]
        print("GsMatrix_B = ", GsMatrix_B)
        print("JsList = ", JsList)
        print("uMax = ", uMax)

        sigma_RN = self.in3dRec.sigma_RN
        omega_RN_N = self.in3dRec.omega_RN_N
        print("sigma_RN = ", sigma_RN)
        print("omega_RN_N = ", omega_RN_N)
        self.fsw_plotter.plot_ref_attitude(sigma_RN, omega_RN_N)

        sigma_BR = self.attErrRec.sigma_BR
        omega_BR_B = self.attErrRec.omega_BR_B
        print( "sigma_BR = ", sigma_BR)
        print( "omega_BR_B = ", omega_BR_B)
        self.fsw_plotter.plot_track_error(sigma_BR, omega_BR_B)

        Lr = self.cmdTorRec.torqueRequestBody
        u_wheels = self.rwMotRec.motorTorque
        print( "Lr = ", Lr)
        print( "u_wheels = ", u_wheels)
        self.fsw_plotter.plot_rw_control(Lr, u_wheels)
        self.fsw_plotter.show_plots()


if __name__ == "__main__":
    theSim = DesktopFSW()
    theSim.InitializeSimulation()

    theSim.modeRequest = "inertialPoint"
    theSim.log_outputs()

    sim_time = macros.min2nano(4.0)
    theSim.ConfigureStopTime(int(sim_time))
    theSim.ExecuteSimulation()

    theSim.pull_outputs()
