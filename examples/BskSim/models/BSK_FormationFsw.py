#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import math
from Basilisk.utilities import macros as mc
from Basilisk.fswAlgorithms import (hillPoint, inertial3D, attTrackingError, MRP_Feedback,
                                    rwMotorTorque, fswMessages,
                                    velocityPoint, MRP_Steering, rateServoFullNonlinear,
                                    sunSafePoint, cssWlsEst, spacecraftPointing)
import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import unitTestSupport


class BSKFswModels():
    def __init__(self, SimBase, fswRate):
        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName
        self.processTasksTimeStep = mc.sec2nano(fswRate)  # 0.5

        # Create tasks
        SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask", self.processTasksTimeStep), 10)

        SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask2", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask2", self.processTasksTimeStep), 10)

        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("spacecraftPointingTask", self.processTasksTimeStep))

        # Create module data and module wraps
        self.inertial3DData = inertial3D.inertial3DConfig()
        self.inertial3DWrap = SimBase.setModelDataWrap(self.inertial3DData)
        self.inertial3DWrap.ModelTag = "inertial3D"

        self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap = SimBase.setModelDataWrap(self.trackingErrorData)
        self.trackingErrorWrap.ModelTag = "trackingError"

        self.mrpFeedbackRWsData = MRP_Feedback.MRP_FeedbackConfig()
        self.mrpFeedbackRWsWrap = SimBase.setModelDataWrap(self.mrpFeedbackRWsData)
        self.mrpFeedbackRWsWrap.ModelTag = "mrpFeedbackRWs"

        self.rwMotorTorqueData = rwMotorTorque.rwMotorTorqueConfig()
        self.rwMotorTorqueWrap = SimBase.setModelDataWrap(self.rwMotorTorqueData)
        self.rwMotorTorqueWrap.ModelTag = "rwMotorTorque"


        self.inertial3DData2 = inertial3D.inertial3DConfig()
        self.inertial3DWrap2 = SimBase.setModelDataWrap(self.inertial3DData2)
        self.inertial3DWrap2.ModelTag = "inertial3D2"

        self.trackingErrorData2 = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap2 = SimBase.setModelDataWrap(self.trackingErrorData2)
        self.trackingErrorWrap2.ModelTag = "trackingError2"

        self.mrpFeedbackRWsData2 = MRP_Feedback.MRP_FeedbackConfig()
        self.mrpFeedbackRWsWrap2 = SimBase.setModelDataWrap(self.mrpFeedbackRWsData2)
        self.mrpFeedbackRWsWrap2.ModelTag = "mrpFeedbackRWs2"

        self.rwMotorTorqueData2 = rwMotorTorque.rwMotorTorqueConfig()
        self.rwMotorTorqueWrap2 = SimBase.setModelDataWrap(self.rwMotorTorqueData2)
        self.rwMotorTorqueWrap2.ModelTag = "rwMotorTorque2"

        self.spacecraftPointing = spacecraftPointing.spacecraftPointingConfig()
        self.spacecraftPointingWrap = SimBase.setModelDataWrap(self.spacecraftPointing)
        self.spacecraftPointingWrap.ModelTag = "spacecraftPointing"

        self.mrpFeedbackControlData = MRP_Feedback.MRP_FeedbackConfig()
        self.mrpFeedbackControlWrap = SimBase.setModelDataWrap(self.mrpFeedbackControlData)
        self.mrpFeedbackControlWrap.ModelTag = "mrpFeedbackControl"

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("inertial3DPointTask", self.inertial3DWrap, self.inertial3DData, 10)
        SimBase.AddModelToTask("inertial3DPointTask", self.trackingErrorWrap, self.trackingErrorData, 9)

        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.mrpFeedbackRWsWrap, self.mrpFeedbackRWsData, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.rwMotorTorqueWrap, self.rwMotorTorqueData, 8)


        SimBase.AddModelToTask("inertial3DPointTask2", self.inertial3DWrap2, self.inertial3DData2, 10)
        SimBase.AddModelToTask("inertial3DPointTask2", self.trackingErrorWrap2, self.trackingErrorData2, 9)

        SimBase.AddModelToTask("mrpFeedbackRWsTask2", self.mrpFeedbackRWsWrap2, self.mrpFeedbackRWsData2, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask2", self.rwMotorTorqueWrap2, self.rwMotorTorqueData2, 8)

        SimBase.AddModelToTask("mrpFeedbackTask", self.mrpFeedbackControlWrap, self.mrpFeedbackControlData, 10) #used for external torque

        SimBase.AddModelToTask("spacecraftPointingTask", self.spacecraftPointingWrap, self.spacecraftPointing, 8)
        SimBase.AddModelToTask("spacecraftPointingTask", self.trackingErrorWrap2, self.trackingErrorData2, 7)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc.disableAllTasks()

        SimBase.createNewEvent("initiateStandby", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'standby'"],
                               ["self.fswProc.disableAllTasks()"])

        SimBase.createNewEvent("initiateAttitudeGuidance", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'inertial3D'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.enableTask('inertial3DPointTask2')",
                                "self.enableTask('mrpFeedbackRWsTask2')"])

        SimBase.createNewEvent("initiateSpacecraftPointing", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'spacecraftPointing'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('spacecraftPointingTask')",
                                "self.enableTask('mrpFeedbackTask')"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetInertial3DPointGuidance(self):
        self.inertial3DData.sigma_R0N = [0.2, 0.4, 0.6]
        self.inertial3DData.outputDataName = "att_reference"

        self.inertial3DData2.sigma_R0N = [0.2, 0.4, 0.6]
        self.inertial3DData2.outputDataName = "att_reference2"

    def SetAttitudeTrackingError(self, SimBase):
        self.trackingErrorData.inputNavName = SimBase.DynModels.simpleNavObject.outputAttName
        # Note: SimBase.DynModels.simpleNavObject.outputAttName = "simple_att_nav_output"
        self.trackingErrorData.inputRefName = "att_reference"
        self.trackingErrorData.outputDataName = "att_guidance"

        self.trackingErrorData2.inputNavName = SimBase.DynModels.simpleNavObject2.outputAttName
        # Note: SimBase.DynModels.simpleNavObject.outputAttName = "simple_att_nav_output"
        self.trackingErrorData2.inputRefName = "att_reference2"
        self.trackingErrorData2.outputDataName = "att_guidance2"

    def SetMRPFeedbackRWA(self):
        self.mrpFeedbackRWsData.K = 3.5
        self.mrpFeedbackRWsData.Ki = -1  # TURN OFF IN CASE OF RUNNING Inertial3D!!!
        self.mrpFeedbackRWsData.P = 30.0
        self.mrpFeedbackRWsData.integralLimit = 2. / self.mrpFeedbackRWsData.Ki * 0.1
        self.mrpFeedbackRWsData.vehConfigInMsgName = "adcs_config_data"
        self.mrpFeedbackRWsData.inputRWSpeedsName = "reactionwheel_output_states"
        self.mrpFeedbackRWsData.rwParamsInMsgName = "rwa_config_data"
        self.mrpFeedbackRWsData.inputGuidName = "att_guidance"
        self.mrpFeedbackRWsData.outputDataName = "controlTorqueRaw"

        self.mrpFeedbackRWsData2.K = 3.5
        self.mrpFeedbackRWsData2.Ki = -1  # TURN OFF IN CASE OF RUNNING Inertial3D!!!
        self.mrpFeedbackRWsData2.P = 30.0
        self.mrpFeedbackRWsData2.integralLimit = 2. / self.mrpFeedbackRWsData2.Ki * 0.1
        self.mrpFeedbackRWsData2.vehConfigInMsgName = "adcs_config_data2"
        self.mrpFeedbackRWsData2.inputRWSpeedsName = "reactionwheel_output_states2"
        self.mrpFeedbackRWsData2.rwParamsInMsgName = "rwa_config_data2"
        self.mrpFeedbackRWsData2.inputGuidName = "att_guidance2"
        self.mrpFeedbackRWsData2.outputDataName = "controlTorqueRaw2"

    def SetRWConfigMsg(self, SimBase):
        # Configure RW pyramid exactly as it is in the Dynamics (i.e. FSW with perfect knowledge)
        rwElAngle = np.array([40.0, 40.0, 40.0, 40.0]) * mc.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * mc.D2R
        wheelJs = 50.0 / (6000.0 * math.pi * 2.0 / 60)

        fswSetupRW.clearSetup()
        for elAngle, azAngle in zip(rwElAngle, rwAzimuthAngle):
            gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
            fswSetupRW.create(gsHat,  # spin axis
                              wheelJs,  # kg*m^2
                              0.2)  # Nm        uMax

        fswSetupRW.writeConfigMessage("rwa_config_data", SimBase.TotalSim, SimBase.FSWProcessName)
        fswSetupRW.writeConfigMessage("rwa_config_data2", SimBase.TotalSim, SimBase.FSWProcessName)

    def SetRWMotorTorque(self, SimBase):
        controlAxes_B = [
          1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0]

        self.rwMotorTorqueData.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData.inputVehControlName = "controlTorqueRaw"
        self.rwMotorTorqueData.outputDataName = "reactionwheel_cmds"
        self.rwMotorTorqueData.rwParamsInMsgName = "rwa_config_data"

        self.rwMotorTorqueData2.controlAxes_B = controlAxes_B
        self.rwMotorTorqueData2.inputVehControlName = "controlTorqueRaw2"
        self.rwMotorTorqueData2.outputDataName = "reactionwheel_cmds2"
        self.rwMotorTorqueData2.rwParamsInMsgName = "rwa_config_data2"

    def SetSpacecraftPointing(self):
        self.spacecraftPointing.chiefPositionInMsgName = "simple_trans_nav_output_chief"
        self.spacecraftPointing.deputyPositionInMsgName = "simple_trans_nav_output_deputy"
        self.spacecraftPointing.attReferenceOutMsgName = "att_reference2"
        self.spacecraftPointing.alignmentVector_B = [1.0, 2.0, 3.0]

    def SetVehicleConfiguration(self, SimBase):
        vehicleConfigOut = fswMessages.VehicleConfigFswMsg()
        # use the same inertia in the FSW algorithm as in the simulation
        vehicleConfigOut.ISCPntB_B = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
        unitTestSupport.setMessage(SimBase.TotalSim,
                                   SimBase.FSWProcessName,
                                    "adcs_config_data2",
                                    vehicleConfigOut)

    def SetMRPFeedbackControl(self, SimBase):
        self.mrpFeedbackControlData.inputGuidName = "att_guidance2"
        self.mrpFeedbackControlData.vehConfigInMsgName = "adcs_config_data2"
        self.mrpFeedbackControlData.outputDataName = SimBase.DynModels.extForceTorqueObject2.cmdTorqueInMsgName
        # Note: SimBase.DynModels.extForceTorqueObject.cmdTorqueInMsgName = "extTorquePntB_B_cmds"

        self.mrpFeedbackControlData.K = 10.0
        self.mrpFeedbackControlData.Ki = 0.0001 # Note: make value negative to turn off integral feedback
        self.mrpFeedbackControlData.P = 30.0
        self.mrpFeedbackControlData.integralLimit = 2. / self.mrpFeedbackControlData.Ki * 0.1

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        self.SetInertial3DPointGuidance()
        self.SetAttitudeTrackingError(SimBase)
        self.SetRWConfigMsg(SimBase)
        self.SetMRPFeedbackRWA()
        self.SetRWMotorTorque(SimBase)
        self.SetSpacecraftPointing()
        self.SetVehicleConfiguration(SimBase)
        self.SetMRPFeedbackControl(SimBase)
