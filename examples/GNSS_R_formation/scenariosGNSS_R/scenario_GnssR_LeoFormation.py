#
#  ISC License
#
#  Copyright (c) 2024, Norwegian University of Science and Technology (NTNU) and
#                      Autonomous Vehicle Systems Lab, University of Colorado
#                      at Boulder [Oliver Hasler] based on:
#                      -> scenario_StationKeepingMultiSat.py
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

As stated in the previous section, the :ref:`BSK_GnssrSatFsw` class used in this example is the same as the one used in
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

import copy, inspect, math, os, sys

from mypy.util import T
import numpy as np
from Basilisk.architecture import messaging
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport, tleHandling, RigidBodyKinematics as rbk

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(os.path.join(path, '..', 'modelsGNSS_R'))
sys.path.append(os.path.join(path, '..', 'plottingGNSS_R'))
from BSK_GnssrMaster import BSKSim, BSKScenario
import BSK_GnssrEnvironmentEarth
import BSK_GnssrSatDynamics, BSK_GpsSatDynamics
import BSK_GnssrSatFsw

# Import plotting files for your scenario
import BSK_MultiSatPlotting as plt

# Create your own scenario child class
class scenario_StatKeepingAttPointGnssrFormaton(BSKSim, BSKScenario):
    def __init__(self, numberSpacecraft, relativeNavigation, txConstTleData):
        super(scenario_StatKeepingAttPointGnssrFormaton, self).__init__(
            numberSpacecraft, relativeNavigation=relativeNavigation, fswRate=1, dynRate=1, envRate=1, relNavRate=1)
        self.name = 'scenario_StatKeepingAttPointGnssrFormaton'

        # Connect the environment, dynamics and FSW classes. It is crucial that these are set in the order specified, as
        # some connects made imply that some modules already exist
        self.set_EnvModel(BSK_GnssrEnvironmentEarth)
        self.set_DynModel([BSK_GnssrSatDynamics] * numberSpacecraft)
        self.set_FswModel([BSK_GnssrSatFsw] * numberSpacecraft)

        # Configure the GNSS constellations
#        for constIdx in range(len(txConstTleData)):
#            # Set GNSS transmitter Dynamics Model
#            self.set_DynModel(BSK_GpsSatDynamics, modelIndex=constIdx, modelCount=len(txConstTleData))

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
        self.chiefTransLog = None

        # declare empty containers for orbital elements
        self.oe = []

        # Set initial conditions and record the relevant messages
        self.configure_initial_conditions()
        self.log_outputs(relativeNavigation)

        if vizSupport.vizFound:
            # if this scenario is to interface with the BSK Viz, uncomment the following line
            DynModelsList = []
            rwStateEffectorList = []
            thDynamicEffectorList = []
            for i in range(self.numberSpacecraft):
                DynModelsList.append(self.DynModels[i].scObject)
                rwStateEffectorList.append(self.DynModels[i].rwStateEffector)
                thDynamicEffectorList.append([self.DynModels[i].thrusterDynamicEffector])

            gsList = []
            # Initialize the vizPanels list before the loop
            self.vizPanels = []
            for i in range(self.numberSpacecraft):
                batteryPanel = vizSupport.vizInterface.GenericStorage()
                batteryPanel.label = "Battery"
                batteryPanel.units = "Ws"
                batteryPanel.color = vizSupport.vizInterface.IntVector(vizSupport.toRGBA255("red") + vizSupport.toRGBA255("lightgreen"))
                batteryPanel.thresholds = vizSupport.vizInterface.IntVector([20])
                batteryInMsg = messaging.PowerStorageStatusMsgReader()
                batteryInMsg.subscribeTo(self.DynModels[i].powerMonitor.batPowerOutMsg)
                batteryPanel.batteryStateInMsg = batteryInMsg

                tankPanel = vizSupport.vizInterface.GenericStorage()
                tankPanel.label = "Tank"
                tankPanel.units = "kg"
                tankPanel.color = vizSupport.vizInterface.IntVector(vizSupport.toRGBA255("cyan"))
                tankInMsg = messaging.FuelTankMsgReader()
                tankInMsg.subscribeTo(self.DynModels[i].fuelTankStateEffector.fuelTankOutMsg)
                tankPanel.fuelTankStateInMsg = tankInMsg

                # Append panels to the class-level list
                self.vizPanels.append(batteryPanel)
                self.vizPanels.append(tankPanel)
                gsList.append([batteryPanel, tankPanel])

            lastTaskName = self.DynModels[-1].taskName  # Load Vizard with the last spacecraft's dynamics task

            # Add barycenter modules to dynamics task for proper Vizard sync
            if relativeNavigation:
                self.AddModelToTask(lastTaskName, self.relativeNavigationModule, 5)
                self.AddModelToTask(lastTaskName, self.barycenterPoint.getConverter(), 4)

            viz = vizSupport.enableUnityVisualization(self, lastTaskName, DynModelsList
                                                      , saveFile=__file__
                                                      , rwEffectorList=rwStateEffectorList
                                                      , thrEffectorList=thDynamicEffectorList
                                                      , genericStorageList=gsList
                                                      )
            if relativeNavigation:
                # Create VizSpacecraftData for the barycenter
                self.barycenterVizData = vizSupport.vizInterface.VizSpacecraftData()
                self.barycenterVizData.spacecraftName = self.barycenterPoint.ModelTag
                self.barycenterVizData.scStateInMsg.subscribeTo(self.barycenterPoint.scStateOutMsg)

                # Explicitly set empty thruster configuration to prevent memory leak
                self.barycenterVizData.thrInMsgs = messaging.THROutputMsgInMsgsVector([])
                self.barycenterVizData.thrInfo = vizSupport.vizInterface.ThrClusterVector([])

                # Add to viz
                viz.scData.push_back(self.barycenterVizData)
                # Hide the barycenter model using zero scale
                vizSupport.createCustomModel(viz,
                    modelPath="SPHERE",
                    simBodiesToModify=["barycenter"],
                    scale=[0.1, 0.1, 0.1])

            viz.settings.showSpacecraftLabels = True
            viz.settings.orbitLinesOn = 2  # show osculating relative orbit trajectories
            viz.settings.mainCameraTarget = "sat-1"
            viz.liveSettings.relativeOrbitChief = "barycenter" if relativeNavigation else "sat-0" # set the chief for relative orbit trajectory
            for i in range(self.numberSpacecraft):
                vizSupport.setInstrumentGuiSetting(viz, spacecraftName=self.DynModels[i].scObject.ModelTag,
                                                   showGenericStoragePanel=True)

    def configure_initial_conditions(self):
        EnvModel = self.get_EnvModel()
        DynModels = self.get_DynModel()

        # SSO parameters for 550 km altitude
        alt = 550e3  # m
        a_sso = EnvModel.planetRadius + alt
        i_sso = 97.6 * macros.D2R  # SSO inclination for 550 km

        # Separation in along-track
        separation = 50  # meters
        delta_f = separation / a_sso  # ≈ 7.2e-6 rad

        # Base true anomaly (this will be the barycenter position)
        f_base = 85.3 * macros.D2R

        # SC0: AHEAD of barycenter (+50m)
        self.oe.append(orbitalMotion.ClassicElements())
        self.oe[0].a = a_sso
        self.oe[0].e = 0.0001
        self.oe[0].i = i_sso
        self.oe[0].Omega = 48.2 * macros.D2R
        self.oe[0].omega = 0.0
        self.oe[0].f = f_base + delta_f  # AHEAD
        rN0, vN0 = orbitalMotion.elem2rv(EnvModel.mu, self.oe[0])
        DynModels[0].scObject.hub.r_CN_NInit = rN0
        DynModels[0].scObject.hub.v_CN_NInit = vN0
        DynModels[0].scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
        DynModels[0].scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

        # SC1: AT barycenter (0m)
        self.oe.append(orbitalMotion.ClassicElements())
        self.oe[1].a = a_sso
        self.oe[1].e = 0.0001
        self.oe[1].i = i_sso
        self.oe[1].Omega = 48.2 * macros.D2R
        self.oe[1].omega = 0.0
        self.oe[1].f = f_base  # AT BARYCENTER
        rN1, vN1 = orbitalMotion.elem2rv(EnvModel.mu, self.oe[1])
        DynModels[1].scObject.hub.r_CN_NInit = rN1
        DynModels[1].scObject.hub.v_CN_NInit = vN1
        DynModels[1].scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
        DynModels[1].scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

        # SC2: BEHIND barycenter (-50m)
        self.oe.append(orbitalMotion.ClassicElements())
        self.oe[2].a = a_sso
        self.oe[2].e = 0.0001
        self.oe[2].i = i_sso
        self.oe[2].Omega = 48.2 * macros.D2R
        self.oe[2].omega = 0.0
        self.oe[2].f = f_base - delta_f  # BEHIND
        rN2, vN2 = orbitalMotion.elem2rv(EnvModel.mu, self.oe[2])
        DynModels[2].scObject.hub.r_CN_NInit = rN2
        DynModels[2].scObject.hub.v_CN_NInit = vN2
        DynModels[2].scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
        DynModels[2].scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    def log_outputs(self, relativeNavigation):
        # Process outputs
        DynModels = self.get_DynModel()
        FswModels = self.get_FswModel()

        # Set the sampling time
        self.samplingTime = macros.sec2nano(10)

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

            # log the RW torque messages
            self.rwMotorLog.append(
                FswModels[spacecraft].rwMotorTorque.rwMotorTorqueOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.rwMotorLog[spacecraft])

            # log the RW wheel speed information
            self.rwSpeedLog.append(DynModels[spacecraft].rwStateEffector.rwSpeedOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.rwSpeedLog[spacecraft])

            # log addition RW information (power, etc)
            for item in range(DynModels[spacecraft].numRW):
                self.rwLogs[spacecraft].append(DynModels[spacecraft].rwStateEffector.rwOutMsgs[item].recorder(self.samplingTime))
                self.rwPowerLogs[spacecraft].append(DynModels[spacecraft].rwPowerList[item].nodePowerOutMsg.recorder(self.samplingTime))
                self.AddModelToTask(DynModels[spacecraft].taskName, self.rwLogs[spacecraft][item])
                self.AddModelToTask(DynModels[spacecraft].taskName, self.rwPowerLogs[spacecraft][item])

            # log the remaining power modules
            self.spLog.append(DynModels[spacecraft].solarPanel.nodePowerOutMsg.recorder(self.samplingTime))
            self.psLog.append(DynModels[spacecraft].powerSink.nodePowerOutMsg.recorder(self.samplingTime))
            self.pmLog.append(DynModels[spacecraft].powerMonitor.batPowerOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.spLog[spacecraft])
            self.AddModelToTask(DynModels[spacecraft].taskName, self.psLog[spacecraft])
            self.AddModelToTask(DynModels[spacecraft].taskName, self.pmLog[spacecraft])

            # log fuel information
            self.fuelLog.append(DynModels[spacecraft].fuelTankStateEffector.fuelTankOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.fuelLog[spacecraft])

            # log thruster information
            for item in range(DynModels[spacecraft].numThr):
                self.thrLogs[spacecraft].append(DynModels[spacecraft].thrusterDynamicEffector.thrusterOutMsgs[item].recorder(self.samplingTime))
                self.AddModelToTask(DynModels[spacecraft].taskName, self.thrLogs[spacecraft][item])

    def pull_outputs(self, showPlots, relativeNavigation, spacecraftIndex):
        # Process outputs
        DynModels = self.get_DynModel()
        EnvModel = self.get_EnvModel()

        #
        #   Retrieve the logged data
        #
        dataUsReq = self.rwMotorLog[spacecraftIndex].motorTorque
        dataSigmaBR = self.attErrorLog[spacecraftIndex].sigma_BR
        dataOmegaBR = self.attErrorLog[spacecraftIndex].omega_BR_B
        dataSigmaBN = self.snAttLog[spacecraftIndex].sigma_BN
        dataOmegaBN_B = self.snAttLog[spacecraftIndex].omega_BN_B
        dataOmegaRW = self.rwSpeedLog[spacecraftIndex].wheelSpeeds
        dataSigmaRN = self.attRefLog[spacecraftIndex].sigma_RN
        dataOmegaRN_N = self.attRefLog[spacecraftIndex].omega_RN_N
        dataFuelMass = self.fuelLog[spacecraftIndex].fuelMass

        # Save RW information
        dataRW = []
        dataRWPower = []
        for item in range(DynModels[spacecraftIndex].numRW):
            dataRW.append(self.rwLogs[spacecraftIndex][item].u_current)
            dataRWPower.append(self.rwPowerLogs[spacecraftIndex][item].netPower)

        # Save thrusters information
        dataThrust = []
        dataThrustPercentage = []
        for item in range(DynModels[spacecraftIndex].numThr):
            dataThrust.append(self.thrLogs[spacecraftIndex][item].thrustForce_B)
            dataThrustPercentage.append(self.thrLogs[spacecraftIndex][item].thrustFactor)

        # Save power info
        supplyData = self.spLog[spacecraftIndex].netPower
        sinkData = self.psLog[spacecraftIndex].netPower
        storageData = self.pmLog[spacecraftIndex].storageLevel
        netData = self.pmLog[spacecraftIndex].currentNetPower

        # Retrieve the time info
        timeLineSetMin = self.snTransLog[spacecraftIndex].times() * macros.NANO2MIN
        timeLineSetSec = self.snTransLog[spacecraftIndex].times() * macros.NANO2SEC

        # Compute the number of time steps of the simulation
        simLength = len(timeLineSetMin)

        # Convert the reference attitude rate into body frame components
        dataOmegaRN_B = []
        for i in range(simLength):
            dcmBN = rbk.MRP2C(dataSigmaBN[i, :])
            dataOmegaRN_B.append(dcmBN.dot(dataOmegaRN_N[i, :]))
        dataOmegaRN_B = np.array(dataOmegaRN_B)

        # Extract position and velocity information for all spacecraft
        r_BN_N = []
        v_BN_N = []
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
        dr = []
        if relativeNavigation:
            for i in range(self.numberSpacecraft):
                rd = np.array([orbitalMotion.rv2hill(dataChiefPosition[item], dataChiefVelocity[item], r_BN_N[i][item],
                                                     v_BN_N[i][item])[0] for item in range(simLength)])
                dr.append(rd)
        else:
            for i in range(1, self.numberSpacecraft):
                rd = np.array([orbitalMotion.rv2hill(dataChiefPosition[item], dataChiefVelocity[item], r_BN_N[i][item],
                                                     v_BN_N[i][item])[0] for item in range(simLength)])
                dr.append(rd)

        # Compute the orbital element differences between the spacecraft and the chief
        oed = np.empty((simLength, 6))
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

        # Compute the orbit period
        T = 2*math.pi*math.sqrt(self.oe[spacecraftIndex].a ** 3 / EnvModel.mu)

        #
        # Plot results
        #
        plt.clear_all_plots()

        plt.plot_attitude(timeLineSetMin, dataSigmaBN, 1)
        plt.plot_rate(timeLineSetMin, dataOmegaBN_B, 2)
        plt.plot_attitude_error(timeLineSetMin, dataSigmaBR, 3)
        plt.plot_rate_error(timeLineSetMin, dataOmegaBR, 4)
        plt.plot_attitude_reference(timeLineSetMin, dataSigmaRN, 5)
        plt.plot_rate_reference(timeLineSetMin, dataOmegaRN_B, 6)
        plt.plot_rw_motor_torque(timeLineSetMin, dataUsReq, dataRW, DynModels[spacecraftIndex].numRW, 7)
        plt.plot_rw_speeds(timeLineSetMin, dataOmegaRW, DynModels[spacecraftIndex].numRW, 8)
        plt.plot_orbits(r_BN_N, self.numberSpacecraft, 9)
        plt.plot_relative_orbits(dr, len(dr), 10)
        plt.plot_orbital_element_differences(timeLineSetSec / T, oed, 11)
        plt.plot_power(timeLineSetMin, netData, supplyData, sinkData, 12)
        plt.plot_fuel(timeLineSetMin, dataFuelMass, 13)
        plt.plot_thrust_percentage(timeLineSetMin, dataThrustPercentage, DynModels[spacecraftIndex].numThr, 14)

        figureList = {}
        if showPlots:
            plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitude", "rate", "attitudeTrackingError", "trackingErrorRate", "attitudeReference",
                           "rateReference", "rwMotorTorque", "rwSpeeds", "orbits", "relativeOrbits", "oeDifferences",
                           "power", "fuel", "thrustPercentage"]
            figureList = plt.save_all_plots(fileName, figureNames)

        # close the plots being saved off to avoid over-writing old and new figures
        plt.clear_all_plots()

        return figureList


def runScenario(scenario, relativeNavigation):
    # Get the environment model
    EnvModel = scenario.get_EnvModel()

    # Phase 0: Initial attitude mode -> Inertial pointing
    scenario.FSWModels[0].modeRequest = "inertialPointing"
    scenario.FSWModels[1].modeRequest = "inertialPointing"
    scenario.FSWModels[2].modeRequest = "inertialPointing"

    # ===================================
    # Configure station keeping
    # ===================================
    # Configure station keeping module
    for spacecraft in range(scenario.numberSpacecraft): #| TODO understand this
        if relativeNavigation:
            # Barycenter = chief
            scenario.relativeNavigationModule.addSpacecraftToModel(
                scenario.DynModels[spacecraft].simpleNavObject.transOutMsg,
                scenario.DynModels[spacecraft].simpleMassPropsObject.vehicleConfigOutMsg)
            scenario.FSWModels[spacecraft].spacecraftReconfig.chiefTransInMsg.subscribeTo(
                scenario.relativeNavigationModule.transOutMsg)
        else:
            # Spacecraft 0 = chief
            scenario.FSWModels[spacecraft].spacecraftReconfig.chiefTransInMsg.subscribeTo(
                scenario.DynModels[0].simpleNavObject.transOutMsg)

    # Configure the relative navigation module
    if relativeNavigation:
        scenario.relativeNavigationModule.useOrbitalElements = False
        scenario.relativeNavigationModule.mu = EnvModel.mu #| TODO why mu set only in relative navigation case?

    # Set up the station keeping requirements
    if relativeNavigation:
        delta_e = 1.5e-5  # eccentricity difference for spacecraft 0
        scenario.FSWModels[0].stationKeeping = "ON"
        scenario.FSWModels[0].spacecraftReconfig.targetClassicOED = [0.0,  delta_e, 0.0, 0.0, 0.0, 0.0] #| Δa/a, Δe, Δi, ΔΩ, Δω, ΔM
    scenario.FSWModels[1].stationKeeping = "ON"
    scenario.FSWModels[1].spacecraftReconfig.targetClassicOED = [0.0, -0.5*delta_e, 0.0, 0.0, +np.sqrt(3)/2*delta_e, 0.0]  #| Δa/a, Δe, Δi, ΔΩ, Δω, ΔM
    scenario.FSWModels[2].stationKeeping = "ON"
    scenario.FSWModels[2].spacecraftReconfig.targetClassicOED = [0.0, -0.5*delta_e, 0.0, 0.0, -np.sqrt(3)/2*delta_e, 0.0]  #| Δa/a, Δe, Δi, ΔΩ, Δω, ΔM

    # ===================================
    # Initialize simulation
    # ===================================
    scenario.InitializeSimulation()

    # ===================================
    # Run simulation phases
    # ===================================
    simulationTime0 = macros.min2nano(5.) # 5 minutes
    scenario.ConfigureStopTime(simulationTime0)
    scenario.ExecuteSimulation()

    # Phase 1: Sun pointing (charging batteries)
    scenario.FSWModels[0].modeRequest = "solarCharging"
    scenario.FSWModels[1].modeRequest = "solarCharging"
    scenario.FSWModels[2].modeRequest = "solarCharging"

    simulationTime1 = macros.hour2nano(0.5) # 30 minutes (35 minutes)
    scenario.ConfigureStopTime(simulationTime0 + simulationTime1)
    scenario.ExecuteSimulation()

    # Phase 2: Reconfigure formation (station keeping -> ON)

    scenario.FSWModels[0].modeRequest = "initiateStationKeeping_"
    scenario.FSWModels[1].modeRequest = "initiateStationKeeping_"
    scenario.FSWModels[2].modeRequest = "initiateStationKeeping_"

    simulationTime2 = macros.hour2nano(24.0) # 24 minutes (24 hours 35 minutes)
    scenario.ConfigureStopTime(simulationTime0 + simulationTime1 + simulationTime2)
    scenario.ExecuteSimulation()

    # Phase 3: location pointing (GNSS-R operations)
    scenario.FSWModels[0].modeRequest = "nadirPoint"  # TODO seems to be not working?
    scenario.FSWModels[1].modeRequest = "nadirPoint"
    scenario.FSWModels[2].modeRequest = "nadirPoint"

    simulationTime3 = macros.hour2nano(2.0) # 2 hours
    scenario.ConfigureStopTime(simulationTime0 + simulationTime1 + simulationTime2 + simulationTime3)
    scenario.ExecuteSimulation()

    # Phase 4: Downlinking (nadirPoint pointing), station keeping OFF
    simulationTime4 = macros.hour2nano(5.0) # 5 hours
    scenario.FSWModels[0].stationKeeping = "OFF"
    scenario.FSWModels[1].stationKeeping = "OFF"
    scenario.FSWModels[2].stationKeeping = "OFF"
    scenario.ConfigureStopTime(simulationTime0 + simulationTime1 + simulationTime2 + simulationTime3 + simulationTime4)
    scenario.ExecuteSimulation()

def run(showPlots, numberSpacecraft, relativeNavigation, txConstTleData):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots
        numberSpacecraft (int): Number of spacecraft in the simulation
        relativeNavigation (bool): Determines if the formation's chief is the barycenter or the zeroth index spacecraft

    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_StatKeepingAttPointGnssrFormaton(numberSpacecraft, relativeNavigation, txConstTleData)
    runScenario(TheScenario, relativeNavigation)
    figureList = TheScenario.pull_outputs(showPlots, relativeNavigation, 1)

    return figureList

if __name__ == "__main__":
    # show current path
    gpsTleData = tleHandling.satTle2elem(os.path.join(path, "TLE", "GPS_operational.tle"))# Options are "Galileo.tle", "GLONAS_operational.tle", "GPS_operational.tle", "BeiDou.tle", "oneWeb.tle"

    run(showPlots=True,
        numberSpacecraft=3,
        relativeNavigation=True,    # If False, the chief is spacecraft 0; if True, the chief is the barycenter
        txConstTleData=[gpsTleData] # can be any of ['GPS', 'Galileo', 'Beidou', 'Glonass']
        )
