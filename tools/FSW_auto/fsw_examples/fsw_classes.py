from Basilisk.fswAlgorithms import (vehicleConfigData, rwConfigData, MRP_Feedback, inertial3D,
                                    rwMotorTorque, attTrackingError)


class FSWClasses(object):
        def __init__(self, masterSim):
            self.masterSim = masterSim

        def vehConfigDataClass(self):
            VehConfigData = vehicleConfigData.VehConfigInputData()
            VehConfigDataWrap = self.masterSim.setModelDataWrap(VehConfigData)
            VehConfigDataWrap.ModelTag = "vehConfigData"
            return VehConfigData, VehConfigDataWrap

        def rwConfigDataClass(self):
            rwData = rwConfigData.rwConfigData_Config()
            rwConfigWrap = self.masterSim.setModelDataWrap(rwData)
            rwConfigWrap.ModelTag = "rwConfigData"
            return rwData, rwConfigWrap

        def inertial3DClass(self):
            inertial3DData = inertial3D.inertial3DConfig()
            inertial3DWrap = self.masterSim.setModelDataWrap(inertial3DData)
            inertial3DWrap.ModelTag = "inertial3D"
            return inertial3DData, inertial3DWrap

        def attTrackingError_baseClass(self):
            attTrackingData_base = attTrackingError.attTrackingErrorConfig()
            attTrackingWrap_base = self.masterSim.setModelDataWrap(attTrackingData_base)
            attTrackingWrap_base.ModelTag = "attTrackingError_base"
            return attTrackingData_base, attTrackingWrap_base

        def MRP_FeedbackRWAClass(self):
            MRP_FeedbackRWAData = MRP_Feedback.MRP_FeedbackConfig()
            MRP_FeedbackRWAWrap = self.masterSim.setModelDataWrap(MRP_FeedbackRWAData)
            MRP_FeedbackRWAWrap.ModelTag = "MRP_FeedbackRWA"
            return MRP_FeedbackRWAData, MRP_FeedbackRWAWrap

        def rwMotorTorqueClass(self):
            rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
            rwMotorTorqueWrap = self.masterSim.setModelDataWrap(rwMotorTorqueData)
            rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
            return rwMotorTorqueData, rwMotorTorqueWrap