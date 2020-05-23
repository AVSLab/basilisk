from config_util import CreateRWAClass
import fsw_classes


class FSWModels(object):
        def __init__(self, masterSim):
            # Create a sim module as an empty container
            self.simBasePath = masterSim.simBasePath
            # Instantiate C classes
            fsw = fsw_classes.FSWClasses(masterSim)
            self.VehConfigData, self.VehConfigDataWrap = fsw.vehConfigDataClass()
            self.rwConfigData, self.rwConfigWrap = fsw.rwConfigDataClass()
            self.inertial3DData, self.inertial3DWrap = fsw.inertial3DClass()
            self.attTrackingData_base, self.attTrackingWrap_base = fsw.attTrackingError_baseClass()
            self.MRP_FeedbackRWAData, self.MRP_FeedbackRWAWrap = fsw.MRP_FeedbackRWAClass()
            self.rwMotorTorqueData, self.rwMotorTorqueWrap = fsw.rwMotorTorqueClass()
            # Initialize all objects
            self.InitAllFSWObjects(masterSim)

        def SetLocalConfigData(self, masterSim):
            # Set RW Config Data
            rwClass = CreateRWAClass()
            masterSim.TotalSim.CreateNewMessage("FSWProcess", "rwa_config_data", rwClass.getStructSize(), 2, "RWConstellation")
            masterSim.TotalSim.WriteMessageData("rwa_config_data", rwClass.getStructSize(), 0, rwClass)
            self.numRW = rwClass.numRW

        def SetVehConfigData(self):
            self.VehConfigData.ISCPntB_B = [700.0, 0.0, 0.0, 0.0, 700.0, 0.0, 0.0, 0.0, 800]  # kg * m^2
            self.VehConfigData.CoM_B = [0.0, 0.0, 1.0]
            self.VehConfigData.outputPropsName = "adcs_config_data"

        def SetRWConfigDataFSW(self):
            self.rwConfigData.rwConstellationInMsgName = "rwa_config_data"
            self.rwConfigData.vehConfigInMsgName = "adcs_config_data"
            self.rwConfigData.rwParamsOutMsgName = "rwa_config_data_parsed"

        def SetInertial3DPoint(self):
            self.inertial3DData.sigma_R0N = [0.2, -0.3, 0.4]
            self.inertial3DData.outputDataName = "att_ref_output_base"

        def setAttTrackingError_base(self):
            self.attTrackingData_base.inputRefName = "att_ref_output_base"
            self.attTrackingData_base.inputNavName = "simple_att_nav_output"
            self.attTrackingData_base.outputDataName = "nom_att_guid_out"
            self.attTrackingData_base.sigma_R0R = [0.0, 0.0, 1.0]

        def SetMRP_FeedbackRWA(self):
            self.MRP_FeedbackRWAData.K = 4.0#1.  # rad/sec
            self.MRP_FeedbackRWAData.P = 30.0#3.  # N*m*sec
            self.MRP_FeedbackRWAData.Ki = -1.0  # N*m - negative values turn off the integral feedback
            self.MRP_FeedbackRWAData.integralLimit = 0.0  # rad
            self.MRP_FeedbackRWAData.inputGuidName = "nom_att_guid_out"
            self.MRP_FeedbackRWAData.vehConfigInMsgName = "adcs_config_data"
            self.MRP_FeedbackRWAData.outputDataName = "controlTorqueRaw"
            self.MRP_FeedbackRWAData.rwParamsInMsgName = "rwa_config_data_parsed"
            # self.MRP_FeedbackRWAData.rwAvailInMsgName = "rw_availability"
            self.MRP_FeedbackRWAData.inputRWSpeedsName = "reactionwheel_output_states"

        def SetRWMotorTorque(self):
            controlAxes_B = [1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0]
            self.rwMotorTorqueData.controlAxes_B = controlAxes_B
            self.rwMotorTorqueData.inputVehControlName = "controlTorqueRaw"
            self.rwMotorTorqueData.outputDataName = "reactionwheel_torques"  # "reactionwheel_cmds_raw"
            self.rwMotorTorqueData.rwParamsInMsgName = "rwa_config_data_parsed"

        def InitAllFSWObjects(self, masterSim):
            self.SetLocalConfigData(masterSim)
            self.SetVehConfigData()
            self.SetRWConfigDataFSW()
            self.SetInertial3DPoint()
            self.setAttTrackingError_base()
            self.SetMRP_FeedbackRWA()
            self.SetRWMotorTorque()

