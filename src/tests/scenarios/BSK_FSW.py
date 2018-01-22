''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
import sys, os, inspect

from Basilisk.utilities import macros as mc

from Basilisk.fswAlgorithms import vehicleConfigData
from Basilisk.fswAlgorithms import hillPoint
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import MRP_Feedback


class FSWClass():
    def __init__(self, SimBase):
        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName
        self.processTasksTimeStep = mc.sec2nano(0.5)

        # Create tasks
        SimBase.fswProc.addTask(SimBase.CreateNewTask("hillPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackTask", self.processTasksTimeStep), 10)

        # Create module data and module wraps
        self.hillPointData = hillPoint.hillPointConfig()
        self.hillPointWrap = SimBase.setModelDataWrap(self.hillPointData)
        self.hillPointWrap.ModelTag = "hillPoint"

        self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap = SimBase.setModelDataWrap(self.trackingErrorData)
        self.trackingErrorWrap.ModelTag = "trackingError"

        self.mrpFeedbackData = MRP_Feedback.MRP_FeedbackConfig()
        self.mrpFeedbackWrap = SimBase.setModelDataWrap(self.mrpFeedbackData)
        self.mrpFeedbackWrap.ModelTag = "mrpFeedback"

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("hillPointTask", self.hillPointWrap, self.hillPointData, 10)
        SimBase.AddModelToTask("hillPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)
        SimBase.AddModelToTask("mrpFeedbackTask", self.mrpFeedbackWrap, self.mrpFeedbackData, 10)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc.disableAllTasks()
        SimBase.createNewEvent("initiateHillPoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'hillPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('hillPointTask')",
                                "self.enableTask('mrpFeedbackTask')"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetHillPointGuidance(self, SimBase):
        self.hillPointData.inputNavDataName = SimBase.DynClass.simpleNavObject.outputTransName
        self.hillPointData.inputCelMessName = SimBase.DynClass.earthGravBody.outputMsgName
        self.hillPointData.outputDataName = "referenceOut"
        return

    def SetAttitudeTrackingError(self, SimBase):
        self.trackingErrorData.inputNavName = SimBase.DynClass.simpleNavObject.outputAttName
        self.trackingErrorData.inputRefName = "referenceOut"
        self.trackingErrorData.outputDataName = "guidanceOut"
        return

    def SetMRPFeedback(self, SimBase):
        self.mrpFeedbackData.inputGuidName = "guidanceOut"
        self.mrpFeedbackData.vehConfigInMsgName = "vehicleData"
        self.mrpFeedbackData.outputDataName = SimBase.DynClass.extForceTorqueObject.cmdTorqueInMsgName
        self.mrpFeedbackData.K = 3.5
        self.mrpFeedbackData.Ki = -1  # make value negative to turn off integral feedback
        self.mrpFeedbackData.P = 30.0
        self.mrpFeedbackData.integralLimit = 2. / self.mrpFeedbackData.Ki * 0.1
        return

    def SetVehicleData(self, SimBase):
        # Vehicle config FSW message
        self.vehicleData = vehicleConfigData.vehicleConfigData()
        self.vehicleData.ISCPntB_B = SimBase.DynClass.I_sc # Make sure you use the same inertia as in the Dyn sim class
        vehicleMessageSize = self.vehicleData.getStructSize()
        SimBase.TotalSim.CreateNewMessage(self.processName, "vehicleData", vehicleMessageSize, 2)
        SimBase.TotalSim.WriteMessageData("vehicleData", vehicleMessageSize, 0, self.vehicleData)
        return

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        self.SetHillPointGuidance(SimBase)
        self.SetAttitudeTrackingError(SimBase)
        self.SetMRPFeedback(SimBase)
        self.SetVehicleData(SimBase)
