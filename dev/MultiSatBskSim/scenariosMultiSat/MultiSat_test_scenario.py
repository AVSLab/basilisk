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

r"""
Overview
--------

This script sets up three 6-DOF spacecraft orbiting the Earth in formation. The goal of this scenario is to

#. introduce formation flying control with desired orbital element differences,
#. evidence how the module can conciliate attitude requests with thruster requirements, and
#. show how one can choose whether the chief of the formation is a spacecraft or the formation's barycenter.

The script is found in the folder ``basilisk/examples/MultiSatBskSim/scenariosMultiSat`` and is executed by using::

      python3 scenario_StationKeepingMultiSat.py

This simulation is based on the :ref:`scenario_AttGuidMultiSat` with the addition of station keeping control. Attitude
mode requests are processed in the same way as before, but now there is the added complexity of introducing formation
control, which can influence attitude requests.

The user can choose whether to use the zeroth index spacecraft or the formation's barycenter as the formation's chief.
On that note, some geometries are not possible when using the barycenter as a reference point for the formation. This is
because the barycenter is influenced by the spacecraft reorienting themselves, and so only some geometries are feasible.
Failure to take this into account results in the spacecraft continuously correcting their orbits without ever
converging.

For simplicity, the script plots only the information related to one of the spacecraft, despite logging the necessary
information for all spacecraft in the simulation.

Custom Dynamics Configurations Instructions
-------------------------------------------

The dynamics modules required for this scenario are the same used in :ref:`scenario_BasicOrbitMultiSat` and
:ref:`scenario_AttGuidMultiSat`. However, this example takes full advantage of all the features of the dynamics class,
which includes thrusters for orbit corrections.

Custom FSW Configurations Instructions
--------------------------------------

As stated in the previous section, the :ref:`BSK_MultiSatFsw` class used in this example is the same as the one used in
:ref:`scenario_AttGuidMultiSat`. The main difference is that the station keeping module is now used, which allows for
relative orbit geometry control.

If no station keeping is desired, then the FSW stack works exactly as in :ref:`scenario_AttGuidMultiSat`. However, if
station keeping is set properly, the FSW events work as follows. First, the attitude reference is set given the pointing
requirements. Then, the station keeping module computes the information regarding the necessary corrective burns, such
as point in orbit, duration, thrust, attitude requirements, etc. With this information, the module then chooses whether
the spacecraft is in a point in orbit where a burn is required. If it is, the attitude reference from the pointing
requirement is overruled in favor of the necessary attitude to complete the current burn. If it is not, the reference
attitude passes through unchanged.

The control law used to drive the formation to its intended orbital element differences is guaranteed to converge if the
chief has Keplerian motion. This might not be the case when the chief is the formation's barycenter, as its orbital
elements change in accordance to how each spacecraft is maneuvering. One way to help with convergence is to make sure
that the barycenter has invariant orbital elements. This can be achieved by guaranteeing that the following equation
holds:

.. math::
    \sum_i m_i\Delta oe_i = 0

Activation of the station keeping mode is done through the ``stationKeeping`` flag. If set to ``True``, formation
control will be activated.

Due to the fact that the ``spacecraftReconfig`` module only accepts messages of the type :ref:`attRefMsgPayload`, the
``locationPointing`` module always outputs a reference message and the ``attTrackingError`` module is always called,
unlike how it happens in :ref:`scenario_AttGuidMultiSat`.

Illustration of Simulation Results
----------------------------------

Since three spacecraft are simulated, and to prevent excessively busy plots, only the information pertaining to one
spacecraft is plotted per simulation.

::

    show_plots = True, numberSpacecraft=3, relativeNavigation=False

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_attitude.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_rate.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_attitudeTrackingError.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_trackingErrorRate.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_attitudeReference.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_rateReference.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_rwMotorTorque.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_rwSpeeds.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_orbits.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_relativeOrbits.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_oeDifferences.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_power.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_fuel.svg
   :align: center

.. image:: /_images/Scenarios/scenario_StationKeepingMultiSat_thrustPercentage.svg
   :align: center

"""

# Get current file path
import inspect, math, os, sys, copy, numpy as np
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../..') # For importing scConfig
import scConfig

from Basilisk.architecture import messaging
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport, RigidBodyKinematics as rbk
from Basilisk.simulation import simSynch

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../modelsMultiSat')
sys.path.append(path + '/../plottingMultiSat')
from BSK_MultiSatMasters import BSKSim, BSKScenario
import BSK_EnvironmentEarth, BSK_MultiSatDynamics, BSK_MultiSatFsw
# import BSK_MultiSatDynamicsOld as BSK_MultiSatDynamics # Temp. solution to make it work
# import BSK_MultiSatFswOld as BSK_MultiSatFsw # Temp. solution to make it work

# Import plotting files for your scenario
import BSK_MultiSatPlotting as plt

# Create your own scenario child class

# class shall Inherits BSK_MultiSatMasters.BSKSim & .BSKScenario
class MultiSat_test_scenario(BSKSim, BSKScenario): 
    def __init__(self, initConfigPath, simRate, dataSamplingTimeSec, relativeNavigation, runRealtime=False):
    # def __init__(self, targetOE, initConfigs, simRate, dataSamplingTimeSec, relativeNavigation, initConfigPath):
        # This below is initializing the scenario itself using the class structure defined in 
        super(MultiSat_test_scenario, self).__init__(
            initConfigPath, relativeNavigation=relativeNavigation, fswRate=simRate, dynRate=simRate, envRate=simRate, relNavRate=simRate)
        self.name = 'MultiSat_test_scenario'
        self.initConfigName = scConfig.getFileNameFromInitConfigJSON(initConfigPath)
        
        # Connect the environment, dynamics and FSW classes. It is crucial that these are set in the order specified, as
        # some connects made imply that some modules already exist
        self.set_EnvModel(BSK_EnvironmentEarth)
        self.set_DynModel([BSK_MultiSatDynamics] * self.numberSpacecraft)
        self.set_FswModel([BSK_MultiSatFsw] * self.numberSpacecraft)

        # declare empty class variables
        self.samplingTime = []
        self.snTransLog = []
        self.snAttLog = []
        self.attErrorLog = []
        self.attRefLog = []
        self.rwMotorLog = []
        self.rwSpeedLog = []
        self.spLog = []
        self.psLog = []
        self.pmLog = []
        self.rwLogs = [[] for _ in range(self.numberSpacecraft)]
        self.rwPowerLogs = [[] for _ in range(self.numberSpacecraft)]
        self.fuelLog = []
        self.thrLogs = [[] for _ in range(self.numberSpacecraft)]
        self.cmdThrLog = [] # Cmd force log from transController module's `cmdForceOutMsg`.
        self.chiefTransLog = None

        # declare empty containers for orbital elements
        self.oe = []
        
        # transGuidMsg logs:
        self.transGuidLog = []

        # Set initial conditions and record the relevant messages
        self.configure_initial_conditions()
        self.log_outputs(relativeNavigation, dataSamplingTimeSec)

        if vizSupport.vizFound:
            # if this scenario is to interface with the BSK Viz, uncomment the following line
            DynModelsList = []
            rwStateEffectorList = []
            thDynamicEffectorList = []
            for i in range(self.numberSpacecraft):
                DynModelsList.append(self.DynModels[i].scObject)
                rwStateEffectorList.append(self.DynModels[i].rwStateEffector)
                thDynamicEffectorList.append([self.DynModels[i].thrusterStateEffector])

            # Below is only for Battery/Fuel Tank, we do not need these for now but keep for later use.
            # gsList = []
            # for i in range(self.numberSpacecraft):
            #     batteryPanel = vizSupport.vizInterface.GenericStorage()
            #     batteryPanel.label = "Battery"
            #     batteryPanel.units = "Ws"
            #     batteryPanel.color = vizSupport.vizInterface.IntVector(vizSupport.toRGBA255("red") + vizSupport.toRGBA255("lightgreen"))
            #     batteryPanel.thresholds = vizSupport.vizInterface.IntVector([20])
            #     batteryInMsg = messaging.PowerStorageStatusMsgReader()
            #     batteryInMsg.subscribeTo(self.DynModels[i].powerMonitor.batPowerOutMsg)
            #     batteryPanel.batteryStateInMsg = batteryInMsg
            #     batteryPanel.this.disown()

            #     tankPanel = vizSupport.vizInterface.GenericStorage()
            #     tankPanel.label = "Tank"
            #     tankPanel.units = "kg"
            #     tankPanel.color = vizSupport.vizInterface.IntVector(vizSupport.toRGBA255("cyan"))
            #     tankInMsg = messaging.FuelTankMsgReader()
            #     tankInMsg.subscribeTo(self.DynModels[i].fuelTankStateEffector.fuelTankOutMsg)
            #     tankPanel.fuelTankStateInMsg = tankInMsg
            #     tankPanel.this.disown()

            #     gsList.append([batteryPanel, tankPanel])

            ## Update 20241110:
            ## runRealtime setting: Enslave simulation to run in Realtime by the simSync.ClockSync module function:
            if runRealtime: # Hardcoded False now...
                clockSync = simSynch.ClockSynch()
                clockSync.accelFactor = 1.0
                # clockSync.accelFactor = 50.0
                self.AddModelToTask(self.DynModels[0].taskName, clockSync) # Check if this task name is valid later...

            # viz = vizSupport.enableUnityVisualization(self, self.DynModels[0].taskName, DynModelsList
            viz = vizSupport.enableUnityVisualization(self, self.DynModels[self.numberSpacecraft - 1].taskName, DynModelsList
                                                      , saveFile=self.initConfigName
                                                    #   , saveFile=__file__
                                                    #   , liveStream=runRealtime
                                                      , rwEffectorList=rwStateEffectorList
                                                      , thrEffectorList=thDynamicEffectorList
                                                    #   , genericStorageList=gsList
                                                      )
            viz.settings.showSpacecraftLabels = True
            viz.settings.orbitLinesOn = 2  # show osculating relative orbit trajectories
            viz.settings.mainCameraTarget = "sat-0"
            viz.liveSettings.relativeOrbitChief = "sat-0"  # set the chief for relative orbit trajectory
            for i in range(self.numberSpacecraft):
                vizSupport.setInstrumentGuiSetting(viz, spacecraftName=self.DynModels[i].scObject.ModelTag,
                                                   showGenericStoragePanel=True)

    ## Orbital elements initiailization:
    def configure_initial_conditions(self): # Set Orbital Elements here!
        EnvModel = self.get_EnvModel()
        DynModels = self.get_DynModel()

        oe_list = scConfig.setInitialCondition(EnvModel, DynModels, self.targetOE, self.initConfigs)
        # targetOE, oe_list = scConfig.setInitialCondition(EnvModel, DynModels, self.initConfigPath)
        # self.targetOE = targetOE
        self.oe = oe_list
        return

    def log_outputs(self, relativeNavigation, dataSamplingTimeSec):
        # Process outputs
        DynModels = self.get_DynModel()
        FswModels = self.get_FswModel()

        # Set the sampling time
        # self.samplingTime = macros.sec2nano(10)
        self.samplingTime = macros.sec2nano(dataSamplingTimeSec)

        # Log the barycentre's position and velocity
        if relativeNavigation:
            self.chiefTransLog = self.relativeNavigationModule.transOutMsg.recorder(self.samplingTime)
            self.AddModelToTask(self.relativeNavigationTaskName, self.chiefTransLog)

        # Loop through every spacecraft
        for spacecraft in range(self.numberSpacecraft):

            # log the navigation messages
            self.snTransLog.append(DynModels[spacecraft].simpleNavObject.transOutMsg.recorder(self.samplingTime))
            self.snAttLog.append(DynModels[spacecraft].simpleNavObject.attOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.snTransLog[spacecraft])
            self.AddModelToTask(DynModels[spacecraft].taskName, self.snAttLog[spacecraft])

            # log the reference messages
            self.attRefLog.append(FswModels[spacecraft].attRefMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.attRefLog[spacecraft])

            # log the attitude error messages
            self.attErrorLog.append(FswModels[spacecraft].attGuidMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.attErrorLog[spacecraft])

            # log transGuidOutMsg -> mind FSW self.transGuidOutMsg (WRONG) v.s. self.transError.transGuidOutMsg (CORRECT)
            # self.transGuidLog.append(FswModels[spacecraft].transGuidOutMsg.recorder(self.samplingTime))
            self.transGuidLog.append(FswModels[spacecraft].transError.transGuidOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.transGuidLog[spacecraft])

            # log the RW torque messages
            self.rwMotorLog.append(
                FswModels[spacecraft].rwMotorTorque.rwMotorTorqueOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.rwMotorLog[spacecraft])

            # # log the RW wheel speed information
            # self.rwSpeedLog.append(DynModels[spacecraft].rwStateEffector.rwSpeedOutMsg.recorder(self.samplingTime))
            # self.AddModelToTask(DynModels[spacecraft].taskName, self.rwSpeedLog[spacecraft])

            # log addition RW information (power, etc)
            for item in range(DynModels[spacecraft].numRW):
                self.rwLogs[spacecraft].append(DynModels[spacecraft].rwStateEffector.rwOutMsgs[item].recorder(self.samplingTime))
                self.AddModelToTask(DynModels[spacecraft].taskName, self.rwLogs[spacecraft][item])
                # self.rwPowerLogs[spacecraft].append(DynModels[spacecraft].rwPowerList[item].nodePowerOutMsg.recorder(self.samplingTime))
                # self.AddModelToTask(DynModels[spacecraft].taskName, self.rwPowerLogs[spacecraft][item])

            # # log the remaining power modules
            # self.spLog.append(DynModels[spacecraft].solarPanel.nodePowerOutMsg.recorder(self.samplingTime))
            # self.psLog.append(DynModels[spacecraft].powerSink.nodePowerOutMsg.recorder(self.samplingTime))
            # self.pmLog.append(DynModels[spacecraft].powerMonitor.batPowerOutMsg.recorder(self.samplingTime))
            # self.AddModelToTask(DynModels[spacecraft].taskName, self.spLog[spacecraft])
            # self.AddModelToTask(DynModels[spacecraft].taskName, self.psLog[spacecraft])
            # self.AddModelToTask(DynModels[spacecraft].taskName, self.pmLog[spacecraft])

            # # log fuel information
            # self.fuelLog.append(DynModels[spacecraft].fuelTankStateEffector.fuelTankOutMsg.recorder(self.samplingTime))
            # self.AddModelToTask(DynModels[spacecraft].taskName, self.fuelLog[spacecraft])

            # log thruster information
            for item in range(DynModels[spacecraft].numThr):
                # self.thrLogs[spacecraft].append(DynModels[spacecraft].thrusterDynamicEffector.thrusterOutMsgs[item].recorder(self.samplingTime))
                self.thrLogs[spacecraft].append(FswModels[spacecraft].thrForceMapping.thrForceCmdOutMsg.recorder(self.samplingTime))
                self.AddModelToTask(DynModels[spacecraft].taskName, self.thrLogs[spacecraft][item])
                
            # log cmd force from transController:
            self.cmdThrLog.append(
                FswModels[spacecraft].transController.cmdForceOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.cmdThrLog[spacecraft])

    def pull_outputs(self, showPlots, relativeNavigation, spacecraftIndex):
        print("SC Index: ", spacecraftIndex)
        # Process outputs
        DynModels = self.get_DynModel()
        EnvModel = self.get_EnvModel()
        FswModels = self.get_FswModel()
        targetSCIndex = FswModels[spacecraftIndex].targetSCIndex
        #
        #   Retrieve the logged data
        #
        dataUsReq = self.rwMotorLog[spacecraftIndex].motorTorque # Cmd torque
        dataSigmaBR = self.attErrorLog[spacecraftIndex].sigma_BR # MRP, Body from Reference frame (Hill frame initiailly)
        dataOmegaBR = self.attErrorLog[spacecraftIndex].omega_BR_B # Angular velocity, Body to Reference frame w.r.t. Body
        dataSigmaBN = self.snAttLog[spacecraftIndex].sigma_BN # MRP, Body from Inertial
        dataOmegaBN_B = self.snAttLog[spacecraftIndex].omega_BN_B # Angular velocity, Body from Inertial
        # dataOmegaRW = self.rwSpeedLog[spacecraftIndex].wheelSpeeds # RW Speed log
        dataSigmaRN = self.attRefLog[spacecraftIndex].sigma_RN # MRP, Reference frmae to Inertial
        dataOmegaRN_N = self.attRefLog[spacecraftIndex].omega_RN_N # Angular velocity, Reference frmae to Inertial
        # dataFuelMass = self.fuelLog[spacecraftIndex].fuelMass # Fuel mass

        dataTransGuid = self.transGuidLog[spacecraftIndex].r_BR_B # Guidance navigation Relative Position, Body to Reference frame (Hill frame initially)
        dataTransGuid_Velocity = self.transGuidLog[spacecraftIndex].v_BR_B # Guidance navigation Relative Velocity (linear), Body to Reference frame (Hill frame initially)

        # Save RW information
        dataRW = [] # RW Actuated Torque log (mapped by mapping module!)
        # dataRWPower = []
        for item in range(DynModels[spacecraftIndex].numRW):
            dataRW.append(self.rwLogs[spacecraftIndex][item].u_current)
        #     dataRWPower.append(self.rwPowerLogs[spacecraftIndex][item].netPower)

        # Save thrusters information
        dataThrust = [] # Thrusters Actuated Forces log (mapped by mapping module!)
        dataThrustPercentage = [] # Thrusted percentage w.r.t. Max. Thrust per thrusters.
        for item in range(DynModels[spacecraftIndex].numThr):
            # print((self.thrLogs[spacecraftIndex][item].thrForce).shape)
            # print((self.thrLogs[spacecraftIndex][item].thrForce[:,:6]).shape)
            # print((self.thrLogs[spacecraftIndex][item].thrForce[:DynModels[spacecraftIndex].numThr][:]).shape)
            dataThrust.append(self.thrLogs[spacecraftIndex][item].thrForce[:,item])
            # dataThrust.append(self.thrLogs[spacecraftIndex][item].thrustForce_B)
            # dataThrustPercentage.append(self.thrLogs[spacecraftIndex][item].thrustFactor)

        # Save Cmd force data
        dataCmdForce = self.cmdThrLog[spacecraftIndex].forceRequestBody # Cmd force
        
        # # Save power info
        # supplyData = self.spLog[spacecraftIndex].netPower
        # sinkData = self.psLog[spacecraftIndex].netPower
        # storageData = self.pmLog[spacecraftIndex].storageLevel
        # netData = self.pmLog[spacecraftIndex].currentNetPower

        # Retrieve the time info
        timeLineSetMin = self.snTransLog[spacecraftIndex].times() * macros.NANO2MIN # Time vector in MINUTES
        timeLineSetSec = self.snTransLog[spacecraftIndex].times() * macros.NANO2SEC # Time vector in SECONDS

        # Compute the number of time steps of the simulation
        simLength = len(timeLineSetMin) # Size/Time Length of Simulation

        # Convert the reference attitude rate into body frame components
        dataOmegaRN_B = [] 
        for i in range(simLength):
            dcmBN = rbk.MRP2C(dataSigmaBN[i, :])
            dataOmegaRN_B.append(dcmBN.dot(dataOmegaRN_N[i, :]))
        dataOmegaRN_B = np.array(dataOmegaRN_B)

        # Extract position and velocity information for all spacecraft
        r_BN_N = [] # S/C Inertial Postion 
        v_BN_N = [] # S/C Inertial Velocity
        for i in range(self.numberSpacecraft):
            r_BN_N.append(self.snTransLog[i].r_BN_N)
            v_BN_N.append(self.snTransLog[i].v_BN_N)

        # Extract position and velocity information of the chief
        if relativeNavigation:
            dataChiefPosition = self.chiefTransLog.r_BN_N
            dataChiefVelocity = self.chiefTransLog.v_BN_N
        else:
            dataChiefPosition = r_BN_N[0]
            dataChiefVelocity = v_BN_N[0]

        # Compute the relative position in the Hill frame
        dr = [] # Relative position (Hill frame) of Chasers from Target spacecraft
        if relativeNavigation:
            for i in range(self.numberSpacecraft):
                rd = np.array([orbitalMotion.rv2hill(dataChiefPosition[item], dataChiefVelocity[item], r_BN_N[i][item],
                                                     v_BN_N[i][item])[0] for item in range(simLength)])
                dr.append(rd)
        else:
            for i in range(self.numberSpacecraft):
            # for i in range(1, self.numberSpacecraft):
                rd = np.array([orbitalMotion.rv2hill(dataChiefPosition[item], dataChiefVelocity[item], r_BN_N[i][item],
                                                     v_BN_N[i][item])[0] for item in range(simLength)])
                dr.append(rd)
                
        # print('Size of dr:', np.shape(dr))

        # Compute the orbital element differences between the spacecraft and the chief
        oed = np.empty((simLength, 6)) # Orbital Element Differences
        for i in range(simLength):
            oe_tmp = orbitalMotion.rv2elem(EnvModel.mu, dataChiefPosition[i], dataChiefVelocity[i])
            oe2_tmp = orbitalMotion.rv2elem(EnvModel.mu, r_BN_N[spacecraftIndex][i], v_BN_N[spacecraftIndex][i])
            oed[i, 0] = (oe2_tmp.a - oe_tmp.a) / oe_tmp.a
            oed[i, 1] = oe2_tmp.e - oe_tmp.e
            oed[i, 2] = oe2_tmp.i - oe_tmp.i
            oed[i, 3] = oe2_tmp.Omega - oe_tmp.Omega
            oed[i, 4] = oe2_tmp.omega - oe_tmp.omega
            E_tmp = orbitalMotion.f2E(oe_tmp.f, oe_tmp.e)
            E2_tmp = orbitalMotion.f2E(oe2_tmp.f, oe2_tmp.e)
            oed[i, 5] = orbitalMotion.E2M(E2_tmp, oe2_tmp.e) - orbitalMotion.E2M(E_tmp, oe_tmp.e)
            for j in range(3, 6):
                if oed[i, j] > math.pi:
                    oed[i, j] = oed[i, j] - 2 * math.pi
                if oed[i, j] < -math.pi:
                    oed[i, j] = oed[i, j] + 2 * math.pi

        # Compute the orbit period - Kepler's 3rd Law
        T = 2*math.pi*math.sqrt(self.oe[spacecraftIndex].a ** 3 / EnvModel.mu)
        
        #
        # Plot results
        #
        plt.clear_all_plots()

        # plt.plot_attitude(timeLineSetMin, dataSigmaBN, 1)
        # plt.plot_rate(timeLineSetMin, dataOmegaBN_B, 2)
        # plt.plot_attitude_error(timeLineSetMin, dataSigmaBR, 3)
        # plt.plot_rate_error(timeLineSetMin, dataOmegaBR, 4)
        # plt.plot_attitude_reference(timeLineSetMin, dataSigmaRN, 5)
        # plt.plot_rate_reference(timeLineSetMin, dataOmegaRN_B, 6)
        plt.plot_rw_motor_torque(timeLineSetMin, dataUsReq, dataRW, DynModels[spacecraftIndex].numRW)
        # plt.plot_rw_speeds(timeLineSetMin, dataOmegaRW, DynModels[spacecraftIndex].numRW, 8)        
        plt.plot_orbits(r_BN_N, self.numberSpacecraft)
        
        # Added animated plot to relative orbits, keep `_` there!
        # _ = plt.plot_relative_orbits(dr, len(dr), 10)
        
        # print(dr, len(dr))
        # print(range(0, self.numberSpacecraft - 1))
        # for scIndex in range(self.numberSpacecraft):
        #     # print("add", scIndex)
        #     if scIndex != targetSCIndex:
        #         if scIndex != 0:
        #             plt.orbit_xyz_time_series(timeLineSetMin, dr, scIndex - 1) # Subtract by 1 
        #         elif scIndex == 0:
        #             plt.orbit_xyz_time_series(timeLineSetMin, dr, 0) # Subtract by 1 
                    
        for i in range(self.numberSpacecraft):
            plt.orbit_xyz_time_series(timeLineSetMin, dr, i) # Subtract by 1 
            
        # plt.plot_orbital_element_differences(timeLineSetSec / T, oed, 13)
        
        # plt.plot_power(timeLineSetMin, netData, supplyData, sinkData, 14)
        # plt.plot_fuel(timeLineSetMin, dataFuelMass, 15)
        plt.plot_cmd_force(timeLineSetMin, dataCmdForce)
        plt.plot_thrust(timeLineSetMin, dataThrust, DynModels[spacecraftIndex].numThr)
        # plt.plot_thrust_percentage(timeLineSetMin, dataThrustPercentage, DynModels[spacecraftIndex].numThr, 31)

        
        figureList = {}
        if showPlots:
            plt.show_all_plots()
            data_dict = None
        # else:
            # To comment out
            # fileName = os.path.basename(os.path.splitext(__file__)[0])
            # figureNames = ["attitude", "rate", "attitudeTrackingError", "trackingErrorRate", "attitudeReference",
            #                "rateReference", "rwMotorTorque", "rwSpeeds", "orbits", "relativeOrbits", "oeDifferences",
            #                "power", "fuel", "thrustPercentage"]
            # figureList = plt.save_all_plots(fileName, figureNames)
            # To comment out
        else:
            # Save plot data to .mat:
            data_filename = scConfig.getFileNameFromInitConfigJSON(self.initConfigPath)
            
            # Manual Output Data as .mat format for plots in MATLAB:
            data_dict = plt.matrices_to_dict(
                simLength=simLength,
                timeLineSetMin=timeLineSetMin,
                dataUsReq=dataUsReq,
                dataSigmaBN=dataSigmaBN,
                dataSigmaBR=dataSigmaBR,
                dataOmegaRN_N=dataOmegaRN_N,
                dataOmegaBR=dataOmegaBR,
                dataRW=dataRW,
                dataThrust=dataThrust,
                dataCmdForce=dataCmdForce,
                dataOmegaRN_B=dataOmegaRN_B,
                dr_index=dr[spacecraftIndex], # Special case, as dr holds all the data for index! We export per index of S/C, so we only get the dr data for particular index!
                oed=oed,
            )
            
            dataPath = "dev/MultiSatBskSim/ResultData/"
            plt.export_data_mat(dataPath, data_filename, data_dict, index_string="index_"+str(spacecraftIndex))
            
            # Save plots:
            figurePath = "dev/MultiSatBskSim/ResultPlots/"
            plt.save_plots_to_path(figurePath, data_filename, index_folder_string="index_"+str(spacecraftIndex))
            
        # close the plots being saved off to avoid over-writing old and new figures
        plt.clear_all_plots()

        return data_dict, figureList


def runScenario(scenario, relativeNavigation, simulationTimeHours, turnOnController=1):
    # Get the environment model
    EnvModel = scenario.get_EnvModel()

    # Configure initial FSW attitude modes, later iterate through a for-loop?
    # scenario.FSWModels[0].modeRequest = "inertialPointing"
    # scenario.FSWModels[1].modeRequest = "inertialPointing" # We need to turn on all S/C tasks!
    # scenario.FSWModels[2].modeRequest = "inertialPointing" # We need to turn on all S/C tasks!    
    
    # scenario.FSWModels[0].modeRequest = "hillPointing"
    # scenario.FSWModels[1].modeRequest = "hillPointing" 
    # scenario.FSWModels[2].modeRequest = "hillPointing"
     
    # Turn on Hill pointing tasks defined in FSW, for all S/C modules:
    # This is done before turning on all the TransControllers for the Chaser spacecraft to follow the Target:
    for i in range(scenario.numberSpacecraft): # Indexxing range starts from 0.
        scenario.FSWModels[i].modeRequest = "hillPointing"
    
    # scenario.FSWModels[1].modeRequest = "sunPointing"
    # scenario.FSWModels[2].modeRequest = "locationPointing"

    # Initialize simulation
    scenario.InitializeSimulation()

    # Configure run time and execute simulation
    simulationTime = macros.hour2nano(simulationTimeHours)
    # Hill pointing time: ~1.5 mins sufficient
    hillPointSimTime = macros.min2nano(5)
    
    scenario.ConfigureStopTime(hillPointSimTime)
    scenario.ExecuteSimulation()
    # scenario.TotalSim.SingleStepProcesses() # Single Step of simulation, keep here in case of debugging.
    
    # Reconfigure FSW attitude modes
    # scenario.FSWModels[0].modeRequest = "inertialPointing"
    # scenario.FSWModels[1].modeRequest = "startTransController"
    # scenario.FSWModels[2].modeRequest = "startTransController" 
    # scenario.FSWModels[3].modeRequest = "startTransController" 
    
    # Start FSW TransController tasks to turn on Relative Position Controllers after sustaining sufficient time for Hill Pointing:
    if turnOnController:
        for i in range(1, scenario.numberSpacecraft): # Ignore the index 0 target: 
            scenario.FSWModels[i].modeRequest = "startTransController"
    
    # Execute the simulation
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()

def run(showPlots, relativeNavigation = False,  
        initConfigPath = "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/init_config.json",
        simulationTimeHours = 1.0,
        turnOnController = 1, # Turn ON: 1, Turn OFF: 0 (base cases validation)
        simRate = 0.1,
        dataSamplingTimeSec = 1.0, # Define simulation rate (for dynamics, FSW & environment models) & data sampling rate.
        runRealtime = False
        ):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots
        (REMOVED) numberSpacecraft (int): Number of spacecraft in the simulation
        relativeNavigation (bool): Determines if the formation's chief is the barycenter or the zeroth index spacecraft

    """ 
    # Command Line (CLI) execution of this python file, pass CLI arguments in NO positional order:
    # Add on CLI arguments are all optional with help strings:
    import argparse
    parser = argparse.ArgumentParser()
    # Specify 5 parameters with arguments for `MultiSat_test_scenario.py`
    parser.add_argument("-p", "--path", type=str, 
                        help="[string] input path to a scenario configuration file")
    parser.add_argument("-st", "--simTime", type=float, 
                        help="[float] simulation time in hours")
    parser.add_argument("-c", "--control", type=int, 
                        help="[int] turn ON/OFF relative position controllers in 1 (ON,default) or 0 (OFF)",
                        choices=[0, 1])
    parser.add_argument("-sr", "--simRate", type=float, 
                        help="[float] simulation rate in seconds")
    parser.add_argument("-sp", "--sampleTime", type=float, 
                        help="[float] data sampling rate in seconds")
    
    args = parser.parse_args()
    initConfigPath = args.path if args.path is not None else  initConfigPath
    simulationTimeHours = args.simTime if args.simTime is not None else  simulationTimeHours
    turnOnController = args.control if args.control is not None else turnOnController
    simRate = args.simRate if args.simRate is not None else  simRate
    dataSamplingTimeSec = args.sampleTime if args.sampleTime is not None else  dataSamplingTimeSec
    
    # Print arguments for verification
    print(f"Scenario Configuration Path: {initConfigPath}")
    print(f"Simulation Time (hours): {simulationTimeHours}")
    print(f"Controller ON/OFF: {turnOnController}")
    print(f"Simulation Rate (seconds): {simRate}")
    print(f"Data Sampling Time (seconds): {dataSamplingTimeSec}")
    
    TheScenario = MultiSat_test_scenario(initConfigPath, simRate, dataSamplingTimeSec, relativeNavigation, runRealtime=runRealtime)
    runScenario(TheScenario, relativeNavigation, simulationTimeHours, turnOnController)
    
    data_dicts = []
    for i in range(TheScenario.numberSpacecraft):
        data_dict, _ = TheScenario.pull_outputs(showPlots, relativeNavigation, i)
        data_dicts.append(data_dict)
        # data_dict, figureList = TheScenario.pull_outputs(showPlots, relativeNavigation, i, initConfigPath)
    if not showPlots:
        data_filename = scConfig.getFileNameFromInitConfigJSON(TheScenario.initConfigPath)
        dataPath = "dev/MultiSatBskSim/ResultData/"
        plt.export_dicts_to_mat(data_dicts, dataPath, data_filename)
    return


if __name__ == "__main__":
    run(showPlots=False, # Set showPlots=False to save plots!
        relativeNavigation=False,
        initConfigPath = "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/control_case_8_xyzr_dot.json",
        # initConfigPath = "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/init_config.json",
        simulationTimeHours = 0.3,
        turnOnController = 1,
        simRate = 0.1,
        dataSamplingTimeSec = 0.1,
        # runRealtime = True
        )