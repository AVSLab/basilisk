#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import copy
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
sys.path.append(path + '/../modelsMAX')
sys.path.append(path + '/../plottingMAX')
from BSK_MAXMasters import BSKSim, BSKScenario
import BSK_EnvironmentEarth, BSK_MAXDynamics, BSK_MAXFsw
import BSK_MAXPrescribedDynamics, BSK_MAXPrescribedFsw

# Import plotting files for your scenario
import BSK_MAXPlotting as plt


# Create your own scenario child class
class scenario_EarthPoint(BSKSim, BSKScenario):
    def __init__(self, numberSpacecraft, prescribed):
        super(scenario_EarthPoint, self).__init__(numberSpacecraft, fswRate=0.1, dynRate=0.01, envRate=0.01)
        self.name = 'scenario_EarthPoint'
        self.prescribed = prescribed

        self.set_EnvModel(BSK_EnvironmentEarth)
        if prescribed:
            self.set_DynModel([BSK_MAXPrescribedDynamics] * self.numberSpacecraft)
            self.set_FswModel([BSK_MAXPrescribedFsw] * self.numberSpacecraft)
        else:
            self.set_DynModel([BSK_MAXDynamics] * self.numberSpacecraft)
            self.set_FswModel([BSK_MAXFsw] * self.numberSpacecraft)

        # declare empty class variables
        self.samplingTime = []
        self.snTransLog = []
        self.snAttLog = []
        self.attErrorLog = []
        self.refLog = []
        self.rwMotorLog = []
        self.rwSpeedLog = []
        self.rwLogs = [[] for _ in range(self.numberSpacecraft)]
        self.earthBoresightLog = []
        self.sunBoresightLogs = []
        self.sensitiveBoresightLog = []
        self.angleLogs = []
        self.refAngleLogs = []
        self.solarArrayTorqueLogs = []

        # declare empty containers for orbital elements
        self.oe = []

        self.configure_initial_conditions()
        self.log_outputs()

        # Create the effector lists and dictionaries for Vizard
        DynModelsList = []
        rwStateEffectorList = []
        scBodyList = []
        for i in range(self.numberSpacecraft):
            DynModelsList.append(self.DynModels[i].scObject)
            rwStateEffectorList.append(self.DynModels[i].rwStateEffector)

            bodyDict = {}
            if prescribed:
                bodyDict[self.DynModels[i].platform.ModelTag] = self.DynModels[i].platform.prescribedMotionConfigLogOutMsg
                for j in range(self.DynModels[i].numRSA):
                    bodyDict[self.DynModels[i].RSAList[j].ModelTag] = self.DynModels[i].RSAList[j].prescribedMotionConfigLogOutMsg
            else:
                bodyDict[self.DynModels[i].platform.ModelTag] = self.DynModels[i].platform.spinningBodyConfigLogOutMsgs[1]
                for j in range(self.DynModels[i].numRSA):
                    bodyDict[self.DynModels[i].RSAList[j].ModelTag] = self.DynModels[i].RSAList[j].spinningBodyConfigLogOutMsg
            scBodyList.append(bodyDict)

        # Enable Vizard
        viz = vizSupport.enableUnityVisualization(self, self.DynModels[0].taskName, DynModelsList
                                                  , saveFile=__file__
                                                  , rwEffectorList=rwStateEffectorList
                                                  , bodyList=scBodyList
                                                  )

        # Change how the spacecraft looks
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[self.DynModels[0].scObject.ModelTag]
                                     , modelPath=path+"/Textures/bus.obj"
                                     , rotation=[0, 0, -np.pi/2]
                                     , offset=[0, 0, -1.5]
                                     )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[self.DynModels[0].platform.ModelTag]
                                     , modelPath="CUBE"
                                     , scale=[0.75, 0.75, 0.2]
                                     )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[self.DynModels[0].RSAList[0].ModelTag]
                                     , modelPath=path+"/Textures/solar-panel2.obj"
                                     , rotation=[np.pi, 0, np.pi / 2]
                                     )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[self.DynModels[0].RSAList[1].ModelTag]
                                     , modelPath=path+"/Textures/solar-panel2.obj"
                                     , rotation=[np.pi, 0, np.pi / 2]
                                     )

        # Add heading lines to visually check pointing performance
        vizSupport.createPointLine(viz, fromBodyName=self.DynModels[0].scObject.ModelTag, toBodyName='earth', lineColor='cyan')
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

            # log the earth boresight output msg
            self.earthBoresightLog.append(DynModels[sc].earthBoresight.angOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[sc].taskName, self.earthBoresightLog[sc])

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

                self.refAngleLogs.append(FswModels[sc].solarArrayRotationDataList[item].spinningBodyRefOutMsg.recorder(self.samplingTime))
                self.AddModelToTask(DynModels[sc].taskName, self.refAngleLogs[item])

                if not self.prescribed:
                    self.solarArrayTorqueLogs.append(FswModels[sc].solarArrayControllerDataList[item].motorTorqueOutMsg.recorder(self.samplingTime))
                    self.AddModelToTask(DynModels[sc].taskName, self.solarArrayTorqueLogs[item])

                self.sunBoresightLogs.append(DynModels[sc].sunBoresightList[item].angOutMsg.recorder(self.samplingTime))
                self.AddModelToTask(DynModels[sc].taskName, self.sunBoresightLogs[item])

    def pull_outputs(self, show_plots, spacecraftIndex):
        # Dynamics process outputs
        DynModels = self.get_DynModel()

        #
        #   Retrieve the logged data
        #

        dataUsReq = self.rwMotorLog[spacecraftIndex].motorTorque
        dataSigmaBR = self.attErrorLog[spacecraftIndex].sigma_BR
        dataOmegaBR = self.attErrorLog[spacecraftIndex].omega_BR_B
        dataSigmaBN = self.snAttLog[spacecraftIndex].sigma_BN
        dataOmegaBN_B = self.snAttLog[spacecraftIndex].omega_BN_B
        dataSigmaRN = self.refLog[spacecraftIndex].sigma_RN
        dataOmegaRN_N = self.refLog[spacecraftIndex].omega_RN_N
        dataOmegaRW = self.rwSpeedLog[spacecraftIndex].wheelSpeeds
        dataEarthBoreAngle = self.earthBoresightLog[spacecraftIndex].missAngle
        dataSensitiveBoreAngle = self.sensitiveBoresightLog[spacecraftIndex].missAngle

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

        #
        # Plot results
        #

        plt.clear_all_plots()
        timeLineSetMin = self.snTransLog[spacecraftIndex].times() * macros.NANO2MIN
        timeLineSetSec = self.snTransLog[spacecraftIndex].times() * macros.NANO2SEC

        plt.plot_attitude(timeLineSetMin, dataSigmaBN, dataSigmaRN, 1)
        plt.plot_rate(timeLineSetMin, dataOmegaBN_N, dataOmegaRN_N, 2)
        plt.plot_attitude_error(timeLineSetMin, dataSigmaBR, 3)
        plt.plot_rate_error(timeLineSetMin, dataOmegaBR, 4)
        plt.plot_rw_motor_torque(timeLineSetMin, dataUsReq, dataRW, DynModels[spacecraftIndex].numRW, 5)
        plt.plot_rw_speeds(timeLineSetMin, dataOmegaRW, DynModels[spacecraftIndex].numRW, 6)
        plt.plot_earth_sun_boresight(timeLineSetMin, dataEarthBoreAngle, dataSensitiveBoreAngle, dataSunBoreAngles, 7)
        plt.plot_solar_array_angle(timeLineSetMin, dataAngle, dataRefAngle, 8)
        plt.plot_solar_array_angle_rate(timeLineSetMin, dataAngleRate, 9)
        if not self.prescribed:
            plt.plot_solar_array_torques(timeLineSetMin, dataArrayTorques, 10)

        if show_plots:
            plt.show_all_plots()

        # close the plots being saved off to avoid over-writing old and new figures
        plt.clear_all_plots()


def runScenario(scenario):
    # Configure FSW mode
    scenario.FSWModels[0].modeRequest = "earthPointing"

    # Initialize simulation
    scenario.InitializeSimulation()
    # scenario.ShowExecutionOrder()

    # Configure run time and execute simulation
    simulationTime = macros.hour2nano(1/6)
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()


def run(show_plots, numberSpacecraft, prescribed):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        numberSpacecraft (int): Number of spacecraft in the simulation
        prescribed (bool): Determines which dynamics and fsw classes to use

    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_EarthPoint(numberSpacecraft, prescribed)
    runScenario(TheScenario)
    TheScenario.pull_outputs(show_plots, 0)


if __name__ == "__main__":
    run(show_plots=True,
        numberSpacecraft=1,
        prescribed=True
        )
