import sys, os
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(path + '/../')
from Basilisk.utilities import (SimulationBaseClass, macros)
from desktopFSW_models import FSWModels
from fsw_plotter import FSWPlotter


class DesktopFSW(SimulationBaseClass.SimBaseClass):
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
        self.TotalSim.logThisMessage(self.fswModels.VehConfigData.outputPropsName, int(1E10))
        self.TotalSim.logThisMessage(self.fswModels.rwConfigData.rwParamsOutMsgName, int(1E10))
        self.TotalSim.logThisMessage(self.fswModels.inertial3DData.outputDataName, int(1E10))
        self.TotalSim.logThisMessage(self.fswModels.attTrackingData_base.outputDataName, int(1E10))
        self.TotalSim.logThisMessage(self.fswModels.MRP_FeedbackRWAData.outputDataName, int(1E10))
        self.TotalSim.logThisMessage(self.fswModels.rwMotorTorqueData.outputDataName, int(1E10))

    def pull_outputs(self, plots_path=None):
        self.fsw_plotter = FSWPlotter(plots_path=plots_path, add_titles=False, name="fsw")

        ISCPntB_B = self.pullMessageLogData(self.fswModels.VehConfigData.outputPropsName + ".ISCPntB_B", range(9))
        CoM_B = self.pullMessageLogData(self.fswModels.VehConfigData.outputPropsName + ".CoM_B", range(3))
        print("ISCPntB_B = ", ISCPntB_B[:, 1:])
        print( "CoM_B = ", CoM_B[:, 1:])

        GsMatrix_B = self.pullMessageLogData(self.fswModels.rwConfigData.rwParamsOutMsgName+".GsMatrix_B", range(3 * self.fswModels.numRW))
        JsList = self.pullMessageLogData(self.fswModels.rwConfigData.rwParamsOutMsgName+".JsList", range(self.fswModels.numRW))
        uMax = self.pullMessageLogData(self.fswModels.rwConfigData.rwParamsOutMsgName + ".uMax", range(self.fswModels.numRW))
        print( "GsMatrix_B = ", GsMatrix_B[:, 1:])
        print( "JsList = ", JsList[:, 1:])
        print( "uMax = ", uMax[:, 1:])

        sigma_RN = self.pullMessageLogData(self.fswModels.inertial3DData.outputDataName + ".sigma_RN", range(3))
        omega_RN_N = self.pullMessageLogData(self.fswModels.inertial3DData.outputDataName + ".omega_RN_N", range(3))
        print( "sigma_RN = ", sigma_RN[:, 1:])
        print( "omega_RN_N = ", omega_RN_N[:, 1:])
        self.fsw_plotter.plot_ref_attitude(sigma_RN, omega_RN_N)

        sigma_BR = self.pullMessageLogData(self.fswModels.attTrackingData_base.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.pullMessageLogData(self.fswModels.attTrackingData_base.outputDataName + ".omega_BR_B", range(3))
        print( "sigma_BR = ", sigma_BR[:, 1:])
        print( "omega_BR_B = ", omega_BR_B[:, 1:])
        self.fsw_plotter.plot_track_error(sigma_BR, omega_BR_B)

        Lr = self.pullMessageLogData(self.fswModels.MRP_FeedbackRWAData.outputDataName + ".torqueRequestBody", range(3))
        u_wheels = self.pullMessageLogData(self.fswModels.rwMotorTorqueData.outputDataName + ".motorTorque", range(self.fswModels.numRW))
        print( "Lr = ", Lr[:, 1:])
        print( "u_wheels = ", u_wheels[:, 1:])
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
