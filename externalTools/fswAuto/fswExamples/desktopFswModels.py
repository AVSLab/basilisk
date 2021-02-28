from configUtil import CreateRWAClass
import fswClasses
from Basilisk.architecture import messaging

class FSWModels(object):
    """
        Class instantiating the FSW modules and initializing them
    """
    def __init__(self, masterSim):
        # Create a sim module as an empty container
        self.simBasePath = masterSim.simBasePath
        # Instantiate C classes
        fsw = fswClasses.FSWClasses(masterSim)
        self.VehConfigData, self.VehConfigDataWrap = fsw.vehConfigDataClass()
        self.rwConfigData, self.rwConfigWrap = fsw.rwConfigDataClass()
        self.inertial3DData, self.inertial3DWrap = fsw.inertial3DClass()
        self.attTrackingData_base, self.attTrackingWrap_base = fsw.attTrackingError_baseClass()
        self.MRP_FeedbackRWAData, self.MRP_FeedbackRWAWrap = fsw.MRP_FeedbackRWAClass()
        self.rwMotorTorqueData, self.rwMotorTorqueWrap = fsw.rwMotorTorqueClass()

        sNavData = messaging.NavAttMsgPayload()
        self.sNavMsg = messaging.NavAttMsg().write(sNavData)
        rwSpeedData = messaging.RWSpeedMsgPayload()
        self.rwSpeedMsg = messaging.RWSpeedMsg().write(rwSpeedData)

        # Initialize all objects
        self.InitAllFSWObjects(masterSim)

    def SetLocalConfigData(self, masterSim):
        # Set RW Config Data
        rwClass = CreateRWAClass()
        self.fswRwConstMsg = messaging.RWConstellationMsg().write(rwClass)
        self.numRW = rwClass.numRW

    def SetVehConfigData(self):
        self.VehConfigData.ISCPntB_B = [700.0, 0.0, 0.0, 0.0, 700.0, 0.0, 0.0, 0.0, 800]  # kg * m^2
        self.VehConfigData.CoM_B = [0.0, 0.0, 1.0]

    def SetRWConfigDataFSW(self):
        self.rwConfigData.rwConstellationInMsg.subscribeTo(self.fswRwConstMsg)

    def SetInertial3DPoint(self):
        self.inertial3DData.sigma_R0N = [0.2, -0.3, 0.4]

    def setAttTrackingError_base(self):
        self.attTrackingData_base.attRefInMsg.subscribeTo(self.inertial3DData.attRefOutMsg)
        self.attTrackingData_base.attNavInMsg.subscribeTo(self.sNavMsg)
        self.attTrackingData_base.sigma_R0R = [0.0, 0.0, 1.0]

    def SetMRP_FeedbackRWA(self):
        self.MRP_FeedbackRWAData.K = 4.0#1.  # rad/sec
        self.MRP_FeedbackRWAData.P = 30.0#3.  # N*m*sec
        self.MRP_FeedbackRWAData.Ki = -1.0  # N*m - negative values turn off the integral feedback
        self.MRP_FeedbackRWAData.integralLimit = 0.0  # rad
        self.MRP_FeedbackRWAData.guidInMsg.subscribeTo(self.attTrackingData_base.attGuidOutMsg)
        self.MRP_FeedbackRWAData.vehConfigInMsg.subscribeTo(self.VehConfigData.vecConfigOutMsg)
        self.MRP_FeedbackRWAData.rwParamsInMsg.subscribeTo(self.rwConfigData.rwParamsOutMsg)
        self.MRP_FeedbackRWAData.rwSpeedsInMsg.subscribeTo(self.rwSpeedMsg)

    def SetRWMotorTorque(self):
        controlAxes_B = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]
        self.rwMotorTorqueData.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData.vehControlInMsg.subscribeTo(self.MRP_FeedbackRWAData.cmdTorqueOutMsg)
        self.rwMotorTorqueData.rwParamsInMsg.subscribeTo(self.rwConfigData.rwParamsOutMsg)

    def InitAllFSWObjects(self, masterSim):
        self.SetLocalConfigData(masterSim)
        self.SetVehConfigData()
        self.SetRWConfigDataFSW()
        self.SetInertial3DPoint()
        self.setAttTrackingError_base()
        self.SetMRP_FeedbackRWA()
        self.SetRWMotorTorque()

