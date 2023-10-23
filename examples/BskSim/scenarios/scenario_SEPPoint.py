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

# Get current file path
import inspect
import os
import sys

# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport
from Basilisk.utilities import RigidBodyKinematics as rbk
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../models')
sys.path.append(path + '/../plotting')
from BSK_SEPMasters import BSKSim, BSKScenario
import BSK_EnvironmentEarth, BSK_SEPDynamics, BSK_SEPFsw
# import BSK_SEPPrescribedDynamics, BSK_SEPPrescribedFsw

# Import plotting files for your scenario
import BSK_SEPPlotting as plt


# Create your own scenario child class
class scenario_SEPPoint(BSKSim, BSKScenario):
    def __init__(self, numberSpacecraft, prescribed):
        super(scenario_SEPPoint, self).__init__(numberSpacecraft, fswRate=1, platRefRate=3600, dynRate=0.1, envRate=0.1)
        self.name = 'scenario_SEPPoint'
        self.prescribed = prescribed

        self.set_EnvModel(BSK_EnvironmentEarth)
        if prescribed:
            self.set_DynModel([BSK_SEPPrescribedDynamics] * self.numberSpacecraft)
            self.set_FswModel([BSK_SEPPrescribedFsw] * self.numberSpacecraft)
        else:
            self.set_DynModel([BSK_SEPDynamics] * self.numberSpacecraft)
            self.set_FswModel([BSK_SEPFsw] * self.numberSpacecraft)

        # declare empty class variables
        self.samplingTime = []
        self.vehConfigLog = []
        self.fswVehConfigLog = []
        self.snTransLog = []
        self.snAttLog = []
        self.attErrorLog = []
        self.refLog = []
        self.rwMotorLog = []
        self.rwSpeedLog = []
        self.rwLogs = [[] for _ in range(self.numberSpacecraft)]
        self.inertialBoresightLog = []
        self.sensitiveBoresightLog = []
        self.sunBoresightLogs = []
        self.angleLogs = []
        self.refAngleLogs = []
        self.solarArrayTorqueLogs = []
        self.platformAngleLogs = []
        self.platformRefAngleLogs = []
        self.platformTorqueLogs = []
        self.platformTorqueJoinedLog = []
        self.platformLockJoinedLog = []
        self.prescribedMotionLog = []
        self.thrusterLog = []


        # declare empty containers for orbital elements
        self.oe = []

        self.configure_initial_conditions()
        self.log_outputs()

        # Create the effector lists and dictionaries for Vizard
        rwStateEffectorList = []
        scBodyList = []
        for i in range(self.numberSpacecraft):
            scBodyList.append(self.DynModels[i].scObject)
            rwStateEffectorList.append(self.DynModels[i].rwStateEffector)

            if prescribed:
                scBodyList.append([self.DynModels[i].platform1.ModelTag, self.DynModels[i].platform.prescribedMotionConfigLogOutMsg])
                for j in range(self.DynModels[i].numRSA):
                    scBodyList.append([self.DynModels[i].RSAList[j].ModelTag, self.DynModels[i].RSAList[j].prescribedMotionConfigLogOutMsg])
            else:
                scBodyList.append([self.DynModels[i].platform1.ModelTag, self.DynModels[i].platform1.spinningBodyConfigLogOutMsgs[1]])
                for j in range(self.DynModels[i].numRSA):
                    scBodyList.append([self.DynModels[i].RSAList[j].ModelTag, self.DynModels[i].RSAList[j].spinningBodyConfigLogOutMsg])                    

        # Enable Vizard
        viz = vizSupport.enableUnityVisualization(self, self.DynModels[0].taskName, scBodyList, saveFile=__file__)

        # Change how the spacecraft looks
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scBodyList[0].ModelTag]
                                     , modelPath=path + "/Textures/bus.obj"
                                     , rotation=[0, 0, -np.pi / 2]
                                     , offset=[0, 0, -1.5]
                                     )
        vizSupport.createCustomModel(viz
                                    , simBodiesToModify=[scBodyList[1][0]]
                                    , modelPath="CUBE"
                                    , scale=[0.75, 0.75, 0.2]
                                     )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scBodyList[2][0]]
                                     , modelPath=path + "/Textures/solar-panel2.obj"
                                     , rotation=[np.pi, 0, np.pi / 2]
                                     )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scBodyList[3][0]]
                                     , modelPath=path + "/Textures/solar-panel2.obj"
                                     , rotation=[np.pi, 0, np.pi / 2]
                                     )

        # Add heading lines to visually check pointing performance
        vizSupport.createPointLine(viz, fromBodyName=self.DynModels[0].RSAList[0].ModelTag, toBodyName='sun', lineColor='yellow')
        vizSupport.createPointLine(viz, fromBodyName=self.DynModels[0].RSAList[1].ModelTag, toBodyName='sun', lineColor='yellow')

        # Additional Vizard settings
        viz.settings.orbitLinesOn = -1
        viz.settings.spacecraftCSon = 1

    def configure_initial_conditions(self):
        EnvModel = self.get_EnvModel()
        DynModels = self.get_DynModel()

        # Configure Dynamics initial conditions
        self.oe.append(orbitalMotion.ClassicElements())
        self.oe[0].a = 100e9  # meters
        self.oe[0].e = 0.001
        self.oe[0].i = 0.0 * macros.D2R
        self.oe[0].Omega = 0.0 * macros.D2R
        self.oe[0].omega = 0.0 * macros.D2R
        self.oe[0].f = -135.0 * macros.D2R
        rN, vN = orbitalMotion.elem2rv(EnvModel.mu, self.oe[0])
        orbitalMotion.rv2elem(EnvModel.mu, rN, vN)
        DynModels[0].scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        DynModels[0].scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        DynModels[0].scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_BN_B
        DynModels[0].scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    def log_outputs(self):
        # Process outputs
        DynModels = self.get_DynModel()
        FswModels = self.get_FswModel()

        # Set the sampling time
        self.samplingTime = macros.sec2nano(1)

        # Loop through every spacecraft
        for sc in range(self.numberSpacecraft):

            thrusterFlag = FswModels[sc].thrusterFlag
            cmEstimation = FswModels[sc].cmEstimation

            # log the vehicle configuration message
            self.vehConfigLog.append(DynModels[sc].simpleMassPropsObject.vehicleConfigOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.vehConfigLog[sc])

            # log the fsw vehicle configuration message
            if cmEstimation:
                self.fswVehConfigLog.append(FswModels[sc].cmEstimationData.vehConfigOutMsg.recorder(self.samplingTime))
            else:
                self.fswVehConfigLog.append(FswModels[sc].fswVehConfigMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.fswVehConfigLog[sc])

            # log the navigation messages
            self.snTransLog.append(DynModels[sc].simpleNavObject.transOutMsg.recorder(self.samplingTime))
            self.snAttLog.append(DynModels[sc].simpleNavObject.attOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.snTransLog[sc])
            self.AddModelToTask(DynModels[sc].taskName, self.snAttLog[sc])

            # log the attitude error messages
            self.attErrorLog.append(FswModels[sc].attGuidMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.attErrorLog[sc])

            # log the reference messages
            self.refLog.append(FswModels[sc].attRefMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.refLog[sc])

            # log the RW torque messages
            self.rwMotorLog.append(FswModels[sc].rwMotorTorqueData.rwMotorTorqueOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.rwMotorLog[sc])

            # log the RW wheel speed information
            self.rwSpeedLog.append(DynModels[sc].rwStateEffector.rwSpeedOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.rwSpeedLog[sc])

            # log addition RW information (power, etc)
            for item in range(DynModels[sc].numRW):
                self.rwLogs[sc].append(DynModels[sc].rwStateEffector.rwOutMsgs[item].recorder(self.samplingTime))
                self.AddModelToTask(DynModels[sc].taskName, self.rwLogs[sc][item])

            # log the boresight output msg
            self.inertialBoresightLog.append(DynModels[sc].inertialBoresight1.angOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.inertialBoresightLog[sc])
            self.inertialBoresightLog.append(DynModels[sc].inertialBoresight2.angOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.inertialBoresightLog[sc+1])

            # log the thermal sensitive boresight output msg
            self.sensitiveBoresightLog.append(DynModels[sc].sensitiveBoresight.angOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.sensitiveBoresightLog[sc])

            # log solar panel angles , reference angles, array torques and boresight angles
            for item in range(DynModels[sc].numRSA):
                if self.prescribed:
                    self.angleLogs.append(FswModels[sc].solarArrayProfilerDataList[item].spinningBodyOutMsg.recorder(self.samplingTime))
                else:
                    self.angleLogs.append(DynModels[sc].RSAList[item].spinningBodyOutMsg.recorder(self.samplingTime))
                self.AddModelToTask(DynModels[sc].taskName, self.angleLogs[item])

                self.refAngleLogs.append(FswModels[sc].solarArrayReferenceDataList[item].hingedRigidBodyRefOutMsg.recorder(self.samplingTime))
                self.AddModelToTask(DynModels[sc].taskName, self.refAngleLogs[item])

                if not self.prescribed:
                    self.solarArrayTorqueLogs.append(FswModels[sc].solarArrayControllerDataList[item].motorTorqueOutMsg.recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.solarArrayTorqueLogs[item])

                self.sunBoresightLogs.append(DynModels[sc].sunBoresightList[item].angOutMsg.recorder(self.samplingTime))
                self.AddModelToTask(DynModels[sc].taskName, self.sunBoresightLogs[item])

            # log platform angles
            if self.prescribed:
                self.AddVariableForLogging(FswModels[sc].platformProfilerSEPWrap.ModelTag + ".phi", self.samplingTime, 0, 0, 'double')
                self.AddVariableForLogging(FswModels[sc].platformProfilerSEPWrap.ModelTag + ".phiRef", self.samplingTime, 0, 0, 'double')
                self.AddVariableForLogging(FswModels[sc].platformProfilerSEPWrap.ModelTag + ".phiAccum", self.samplingTime, 0, 0, 'double')
                self.prescribedMotionLog.append(FswModels[sc].prescribedMotionMsg.recorder(self.samplingTime))
                self.AddModelToTask(DynModels[sc].taskName, self.prescribedMotionLog[0])
            else:
                if thrusterFlag == 1:
                    self.platformAngleLogs.append(DynModels[sc].platform1.spinningBodyOutMsgs[0].recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformAngleLogs[0])
                    self.platformAngleLogs.append(DynModels[sc].platform1.spinningBodyOutMsgs[1].recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformAngleLogs[1])

                    # log platform reference angles
                    self.platformRefAngleLogs.append(FswModels[sc].platform1ReferenceData.hingedRigidBodyRef1OutMsg.recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformRefAngleLogs[0])
                    self.platformRefAngleLogs.append(FswModels[sc].platform1ReferenceData.hingedRigidBodyRef2OutMsg.recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformRefAngleLogs[1])
                else:
                    self.platformAngleLogs.append(DynModels[sc].platform2.spinningBodyOutMsgs[0].recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformAngleLogs[0])
                    self.platformAngleLogs.append(DynModels[sc].platform2.spinningBodyOutMsgs[1].recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformAngleLogs[1])

                    # log platform reference angles
                    self.platformRefAngleLogs.append(FswModels[sc].platform2ReferenceData.hingedRigidBodyRef1OutMsg.recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformRefAngleLogs[0])
                    self.platformRefAngleLogs.append(FswModels[sc].platform2ReferenceData.hingedRigidBodyRef2OutMsg.recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformRefAngleLogs[1])
            
            if thrusterFlag == 1:
                # log platform torques
                if not self.prescribed:
                    for item in range(len(FswModels[sc].platform1ControllerDataList)):
                        self.platformTorqueLogs.append(FswModels[sc].platform1ControllerDataList[item].motorTorqueOutMsg.recorder(self.samplingTime))
                        self.AddModelToTask(DynModels[sc].taskName, self.platformTorqueLogs[item])

                    # log platform torques combined
                    self.platformTorqueJoinedLog.append(FswModels[sc].platform1TorqueMsg.recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformTorqueJoinedLog[sc])

                    # log platform torques combined
                    self.platformLockJoinedLog.append(FswModels[sc].platform1LockMsg.recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformLockJoinedLog[sc])

                # log the thruster output msg
                self.thrusterLog.append(DynModels[sc].SEPThruster1.thrusterOutMsgs[0].recorder(self.samplingTime))
                self.AddModelToTask(DynModels[sc].taskName, self.thrusterLog[sc])
            else:
                # log platform torques
                if not self.prescribed:
                    for item in range(len(FswModels[sc].platform2ControllerDataList)):
                        self.platformTorqueLogs.append(FswModels[sc].platform2ControllerDataList[item].motorTorqueOutMsg.recorder(self.samplingTime))
                        self.AddModelToTask(DynModels[sc].taskName, self.platformTorqueLogs[item])

                    # log platform torques combined
                    self.platformTorqueJoinedLog.append(FswModels[sc].platform2TorqueMsg.recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformTorqueJoinedLog[sc])

                    # log platform torques combined
                    self.platformLockJoinedLog.append(FswModels[sc].platform2LockMsg.recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.platformLockJoinedLog[sc])

                # log the thruster output msg
                self.thrusterLog.append(DynModels[sc].SEPThruster2.thrusterOutMsgs[0].recorder(self.samplingTime))
                self.AddModelToTask(DynModels[sc].taskName, self.thrusterLog[sc])

            # log the SRP torque
            self.AddVariableForLogging(DynModels[sc].newSRP.ModelTag + ".torqueExternalPntB_B", self.samplingTime, 0, 2)


    def pull_outputs(self, show_plots, spacecraftIndex):
        # Dynamics process outputs
        DynModels = self.get_DynModel()
        FswModels = self.get_FswModel()

        thrusterFlag = FswModels[spacecraftIndex].thrusterFlag

        #
        #   Retrieve the logged data
        #

        dataCMLocation = self.vehConfigLog[spacecraftIndex].CoM_B
        dataFswCMLocation = self.fswVehConfigLog[spacecraftIndex].CoM_B
        dataUsReq = self.rwMotorLog[spacecraftIndex].motorTorque
        dataSigmaBR = self.attErrorLog[spacecraftIndex].sigma_BR
        dataOmegaBR = self.attErrorLog[spacecraftIndex].omega_BR_B
        dataSigmaBN = self.snAttLog[spacecraftIndex].sigma_BN
        dataOmegaBN_B = self.snAttLog[spacecraftIndex].omega_BN_B
        dataSigmaRN = self.refLog[spacecraftIndex].sigma_RN
        dataOmegaRN_N = self.refLog[spacecraftIndex].omega_RN_N
        dataOmegaRW = self.rwSpeedLog[spacecraftIndex].wheelSpeeds
        dataSensitiveBoreAngle = self.sensitiveBoresightLog[spacecraftIndex].missAngle
        if thrusterFlag == 1:
            dataInertialBoreAngle = self.inertialBoresightLog[spacecraftIndex].missAngle
        else:
            dataInertialBoreAngle = self.inertialBoresightLog[spacecraftIndex+1].missAngle

        dataOmegaBN_N = dataOmegaBN_B
        for i, omegaBN_B in enumerate(dataOmegaBN_B):
            BN = rbk.MRP2C(dataSigmaBN[i])
            dataOmegaBN_N[i] = np.matmul(BN.transpose(), omegaBN_B)

        dataRW = []
        for item in range(DynModels[spacecraftIndex].numRW):
            dataRW.append(self.rwLogs[spacecraftIndex][item].u_current)

        dataAngle = []
        dataAngleRate = []
        dataRefAngle = []
        dataArrayTorques = []
        dataSunBoreAngles = []
        for item in range(DynModels[spacecraftIndex].numRSA):
            dataAngle.append(self.angleLogs[item].theta)
            dataAngleRate.append(self.angleLogs[item].thetaDot)
            dataRefAngle.append(self.refAngleLogs[item].theta)
            dataSunBoreAngles.append(self.sunBoresightLogs[item].missAngle)

            if not self.prescribed:
                dataArrayTorques.append(self.solarArrayTorqueLogs[item].motorTorque)

        dataPlatformAngle = []
        dataPlatformRefAngle = []
        if self.prescribed:
            phi = self.GetLogVariableData(FswModels[spacecraftIndex].platformProfilerSEPWrap.ModelTag + ".phi")
            phiRef = self.GetLogVariableData(FswModels[spacecraftIndex].platformProfilerSEPWrap.ModelTag + ".phiRef")
            phiAccum = self.GetLogVariableData(FswModels[spacecraftIndex].platformProfilerSEPWrap.ModelTag + ".phiAccum")
            dataPlatformAngle.append(np.delete(phi, 0, axis=1))
            dataPlatformRefAngle.append(np.delete(phiRef, 0, axis=1))
            dataSigmaFM = self.prescribedMotionLog[spacecraftIndex].sigma_FM
        else:
            dataPlatformReqTorques = []
            for item in range(len(self.platformAngleLogs)):
                dataPlatformAngle.append(self.platformAngleLogs[item].theta)
                dataPlatformRefAngle.append(self.platformRefAngleLogs[item].theta)
                dataPlatformReqTorques.append(self.platformTorqueLogs[item].motorTorque)

            dataPlatTorque = self.platformTorqueJoinedLog[spacecraftIndex].motorTorque
            dataPlatLock = self.platformLockJoinedLog[spacecraftIndex].effectorLockFlag

        # very cumbersome routine to compute the offset between thruster and CM direction
        dataSigmaFB = []
        dataThrusterOffset = []
        if thrusterFlag == 1:
            MB = rbk.MRP2C(FswModels[spacecraftIndex].platform1ReferenceData.sigma_MB)
            r_BM_M = FswModels[spacecraftIndex].platform1ReferenceData.r_BM_M
            r_BM_B = np.matmul(MB.transpose(), r_BM_M)
            r_FM_F = FswModels[spacecraftIndex].platform1ReferenceData.r_FM_F
        else:
            MB = rbk.MRP2C(FswModels[spacecraftIndex].platform2ReferenceData.sigma_MB)
            r_BM_M = FswModels[spacecraftIndex].platform2ReferenceData.r_BM_M
            r_BM_B = np.matmul(MB.transpose(), r_BM_M)
            r_FM_F = FswModels[spacecraftIndex].platform2ReferenceData.r_FM_F
        r_TF_F = [0, 0, 0]
        T_F = [0, 0, 0.1]
        for i in range(len(dataCMLocation)):
            r_CM_B = r_BM_B + dataCMLocation[i]
            
            if not self.prescribed:
                theta1 = dataPlatformAngle[0][i]
                theta2 = dataPlatformAngle[1][i]
                FM = [[np.cos(theta2),  np.sin(theta1)*np.sin(theta2), -np.cos(theta1)*np.sin(theta2)],
                    [       0      ,          np.cos(theta1)       ,         np.sin(theta1)        ],
                    [np.sin(theta2), -np.sin(theta1)*np.cos(theta2),  np.cos(theta1)*np.cos(theta2)]]
            else:
                FM = rbk.MRP2C(dataSigmaFM[i])
            FB = np.matmul(FM, MB)
            dataSigmaFB.append(rbk.C2MRP(FB))
            r_TM_F = np.array(r_FM_F) + np.array(r_TF_F)
            r_CT_F = np.matmul(FB, r_CM_B) - r_TM_F
            dataThrusterOffset.append(np.arccos(min(max(np.dot(r_CT_F, T_F) / np.linalg.norm(r_CT_F) / np.linalg.norm(T_F),-1),1)))
        dataThrusterOffset = np.array(dataThrusterOffset)

        # SRP torque in B-frame components
        dataSRP = []
        L_SRP = self.GetLogVariableData(DynModels[spacecraftIndex].newSRP.ModelTag + ".torqueExternalPntB_B")
        dataSRP = np.delete(L_SRP, 0, axis=1)

        # Swirl torque
        dataSwirl = []
        dataThrust = []
        dataThrustTorqueError = []
        maxThrust = self.thrusterLog[spacecraftIndex].maxThrust
        thrustFactor = self.thrusterLog[spacecraftIndex].thrustFactor
        tHat_F = self.thrusterLog[spacecraftIndex].thrusterDirection
        for i in range(len(thrustFactor)):
            dataThrust.append(thrustFactor[i] * maxThrust[i] * np.matmul(tHat_F[i], rbk.MRP2C(dataSigmaFB[i])))
            dataSwirl.append(thrustFactor[i] * maxThrust[i] * 1.3e-3 * np.matmul(tHat_F[i], rbk.MRP2C(dataSigmaFB[i])))
            dataThrustTorqueError.append(np.cross(dataFswCMLocation[i]-dataCMLocation[i], dataThrust[i]))
        dataSwirl = np.array(dataSwirl)
        dataThrustTorqueError = np.array(dataThrustTorqueError)

        #
        # Plot results
        #

        plt.clear_all_plots()
        timeLineSetMin = self.snTransLog[spacecraftIndex].times() * macros.NANO2MIN
        timeLineSetSec = self.snTransLog[spacecraftIndex].times() * macros.NANO2SEC

        plt.plot_attitude(timeLineSetMin, dataSigmaBN, dataSigmaRN, 1)
        plt.plot_attitude_error(timeLineSetMin, dataSigmaBR, 2)
        plt.plot_rate(timeLineSetMin, dataOmegaBN_N, dataOmegaRN_N, 3)
        plt.plot_rate_error(timeLineSetMin, dataOmegaBR, 4)
        plt.plot_rw_motor_torque(timeLineSetMin, dataUsReq, dataRW, DynModels[spacecraftIndex].numRW, 5)
        plt.plot_rw_speeds(timeLineSetMin, dataOmegaRW, DynModels[spacecraftIndex].numRW, DynModels[spacecraftIndex].rwFactory.rwList['RW1'].Omega_max, 6)
        plt.plot_inertial_pointing(timeLineSetMin, dataInertialBoreAngle, 3, 7)
        # plt.plot_inertial_sun_boresight(timeLineSetMin, dataInertialBoreAngle, dataSensitiveBoreAngle, dataSunBoreAngles, 7)
        plt.plot_solar_array_angle(timeLineSetMin, dataAngle, dataRefAngle, 8)
        plt.plot_solar_array_angle_rate(timeLineSetMin, dataAngleRate, 9)
        plt.plot_platform_angle(timeLineSetMin, dataPlatformAngle, dataPlatformRefAngle, 10)
        plt.plot_thruster_offset(timeLineSetMin, dataThrusterOffset, 11)

        if not self.prescribed:
            plt.plot_platform_torques(timeLineSetMin, dataPlatTorque, dataPlatLock, dataPlatformReqTorques, 2, 12)
            # plt.plot_solar_array_torques(timeLineSetMin, dataArrayTorques, 13)

        plt.plot_external_torque(timeLineSetMin, dataSRP, 'SRP Torque', 14)
        plt.plot_external_torque(timeLineSetMin, dataSwirl, 'Swirl Torque', 15)
        plt.plot_external_torque(timeLineSetMin, dataThrustTorqueError, 'Thrust Torque Error', 16)

        if show_plots:
            plt.show_all_plots()

        # close the plots being saved off to avoid over-writing old and new figures
        plt.clear_all_plots()


def runScenario(scenario):
    # Configure FSW mode
    scenario.FSWModels[0].modeRequest = "sepPointing"

    # Initialize simulation
    scenario.InitializeSimulation()
    scenario.ShowExecutionOrder()

    # Configure run time and execute simulation
    simulationTime = macros.day2nano(7)
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()


def run(show_plots, numberSpacecraft, prescribed):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        numberSpacecraft (int): Number of spacecraft in the simulation

    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_SEPPoint(numberSpacecraft, prescribed)
    runScenario(TheScenario)
    TheScenario.pull_outputs(show_plots, 0)


if __name__ == "__main__":
    run(show_plots=True,
        numberSpacecraft=1,
        prescribed=False
        )
