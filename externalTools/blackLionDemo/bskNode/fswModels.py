''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''

from Basilisk.utilities import macros as mc
from Basilisk.fswAlgorithms import vehicleConfigData, hillPoint, inertial3D, attTrackingError, MRP_Feedback


class BSKFsw():
    def __init__(self, SimBase):
        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName
        self.processTasksTimeStep = mc.sec2nano(0.1)  # 0.5

        # Create tasks
        SimBase.fswProc.addTask(SimBase.CreateNewTask("initializationTask", int(1E10)), 1)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackTask", self.processTasksTimeStep), 10)

        # Create module data and module wraps
        self.vehicleData = vehicleConfigData.VehConfigInputData()
        self.vehicleWrap = SimBase.setModelDataWrap(self.vehicleData)
        self.vehicleWrap.ModelTag = "vehicleConfiguration"

        self.inertial3DData = inertial3D.inertial3DConfig()
        self.inertial3DWrap = SimBase.setModelDataWrap(self.inertial3DData)
        self.inertial3DWrap.ModelTag = "inertial3D"

        self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap = SimBase.setModelDataWrap(self.trackingErrorData)
        self.trackingErrorWrap.ModelTag = "trackingError"

        self.mrpFeedbackData = MRP_Feedback.MRP_FeedbackConfig()
        self.mrpFeedbackWrap = SimBase.setModelDataWrap(self.mrpFeedbackData)
        self.mrpFeedbackWrap.ModelTag = "mrpFeedback"

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Assign initialized modules to tasks

        SimBase.AddModelToTask("initializationTask", self.vehicleWrap, self.vehicleData, 10)
        SimBase.AddModelToTask("inertial3DPointTask", self.inertial3DWrap, self.inertial3DData, 10)
        SimBase.AddModelToTask("inertial3DPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)
        SimBase.AddModelToTask("mrpFeedbackTask", self.mrpFeedbackWrap, self.mrpFeedbackData, 10)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc.disableAllTasks()
        SimBase.createNewEvent("initiateAttitudeGuidance", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'inertial3D'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mrpFeedbackTask')"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetInertial3DPointGuidance(self):
        self.inertial3DData.sigma_R0N = [0.2, 0.4, 0.6]
        self.inertial3DData.outputDataName = "referenceOut"

    def SetAttitudeTrackingError(self, SimBase):
        # self.trackingErrorData.inputNavName = SimBase.DynClass.simpleNavObject.outputAttName
        self.trackingErrorData.inputNavName = "simple_att_nav_output"
        self.trackingErrorData.inputRefName = "referenceOut"
        self.trackingErrorData.outputDataName = "guidanceOut"

    def SetMRPFeedback(self, SimBase):
        self.mrpFeedbackData.inputGuidName = "guidanceOut"
        self.mrpFeedbackData.vehConfigInMsgName = "vehicleConfig"
        # self.mrpFeedbackData.outputDataName = SimBase.DynClass.extForceTorqueObject.cmdTorqueInMsgName
        self.mrpFeedbackData.outputDataName = "extTorquePntB_B_cmds"
        self.mrpFeedbackData.K = 3.5
        self.mrpFeedbackData.Ki = -1  # make value negative to turn off integral feedback
        self.mrpFeedbackData.P = 30.0
        self.mrpFeedbackData.integralLimit = 2. / self.mrpFeedbackData.Ki * 0.1

    def SetVehicleConfiguration(self, SimBase):
        # self.vehicleData.ISCPntB_B = SimBase.DynClass.I_sc
        self.vehicleData.ISCPntB_B = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
        self.vehicleData.CoM_B = [0.0, 0.0, 1.0]
        self.vehicleData.outputPropsName = "vehicleConfig"

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        self.SetInertial3DPointGuidance()
        self.SetAttitudeTrackingError(SimBase)
        self.SetMRPFeedback(SimBase)
        self.SetVehicleConfiguration(SimBase)
