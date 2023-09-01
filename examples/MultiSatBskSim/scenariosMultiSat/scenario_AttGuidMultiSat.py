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

This script sets up three 6-DOF spacecraft orbiting the Earth. The goal of this scenario is to

#. introduce the flight software class,
#. show how one can change the active flight mode on the run, and
#. discuss how this script takes advantage of the new BSK Sim structure.

The script is found in the folder ``basilisk/examples/MultiSatBskSim/scenariosMultiSat`` and is executed by using::

      python3 scenario_AttGuidMultiSat.py

This simulation is based on the :ref:`scenario_BasicOrbitMultiSat` with the addition of flight software modules. It also
takes some cues from :ref:`scenario_AttGuidance`, but with several spacecraft and more possible flight modes.

For simplicity, the script plots only the information related to one of the spacecraft, despite logging the necessary
information for all spacecraft in the simulation.

Custom Dynamics Configurations Instructions
-------------------------------------------

The dynamics modules required for this scenario are identical to those used in :ref:`scenario_BasicOrbitMultiSat`.


Custom FSW Configurations Instructions
--------------------------------------

In this example, all spacecraft inherit the same flight software class defined in :ref:`BSK_MultiSatFsw`. Four flight
modes are implemented through the use of events and are described below:

#. ``standby``: the spacecraft has no attitude requirements.
#. ``inertialPointing``: the spacecraft points at some inertially fixed location.
#. ``sunPointing``: the spacecraft points at the Sun.
#. ``locationPointing``: the spacecraft aims at a ground location on Earth.

The attitude is controlled using a set of four reaction wheels that are set up in :ref:`BSK_MultiSatDynamics`. The
``mrpFeedback`` is used for the control law and ``rwMotorTorque`` interfaces with the reaction wheels. The
``attTrackingError`` module is used with all modes to convert from a reference message to a guidance one.

The events can be set using the ``modeRequest`` flag inside the FSW class. It is crucial that all events call the
``setAllButCurrentEventActivity`` method. This function is called in a way such that all events' activity is made active
except for the current one. Without this command, every event could only be made active once. The method also makes
sure it only affects the events specific to each spacecraft. For more information, see :ref:`SimulationBaseClass`.

No formation flying control is done in this scenario. To see a more complete example which includes formation geometry
control, see :ref:`scenario_StationKeepingMultiSat`.

In this scenario, it is shown how the flight software events are set up, and how to change them on-the-fly.

Illustration of Simulation Results
----------------------------------

Since three spacecraft are simulated, and to prevent excessively busy plots, only the information pertaining to one
spacecraft is plotted per simulation.

::

    show_plots = True, numberSpacecraft=3

.. image:: /_images/Scenarios/scenario_AttGuidMultiSat_attitude.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttGuidMultiSat_rate.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttGuidMultiSat_attitude_tracking_error.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttGuidMultiSat_tracking_error_rate.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttGuidMultiSat_rw_motor_torque.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttGuidMultiSat_rw_speeds.svg
   :align: center

"""

import copy
# Get current file path
import inspect
import os
import sys

# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../modelsMultiSat')
sys.path.append(path + '/../plottingMultiSat')
from BSK_MultiSatMasters import BSKSim, BSKScenario
import BSK_EnvironmentEarth, BSK_MultiSatDynamics, BSK_MultiSatFsw

# Import plotting files for your scenario
import BSK_MultiSatPlotting as plt

# Create your own scenario child class
class scenario_AttGuidFormationFlying(BSKSim, BSKScenario):
    def __init__(self, numberSpacecraft):
        super(scenario_AttGuidFormationFlying, self).__init__(numberSpacecraft, fswRate=10, dynRate=10, envRate=10)
        self.name = 'scenario_AttGuidFormationFlying'

        self.set_EnvModel(BSK_EnvironmentEarth)
        self.set_DynModel([BSK_MultiSatDynamics]*self.numberSpacecraft)
        self.set_FswModel([BSK_MultiSatFsw]*self.numberSpacecraft)

        # declare empty class variables
        self.samplingTime = []
        self.snTransLog = []
        self.snAttLog = []
        self.attErrorLog = []
        self.rwMotorLog = []
        self.rwSpeedLog = []
        self.rwLogs = [[] for _ in range(self.numberSpacecraft)]

        # declare empty containers for orbital elements
        self.oe = []

        self.configure_initial_conditions()
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the saveFile line
        DynModelsList = []
        rwStateEffectorList = []
        for i in range(self.numberSpacecraft):
            DynModelsList.append(self.DynModels[i].scObject)
            rwStateEffectorList.append(self.DynModels[i].rwStateEffector)

        vizSupport.enableUnityVisualization(self, self.DynModels[0].taskName, DynModelsList
                                            # , saveFile=__file__
                                            , rwEffectorList=rwStateEffectorList
                                            )

    def configure_initial_conditions(self):
        EnvModel = self.get_EnvModel()
        DynModels = self.get_DynModel()

        # Configure Dynamics initial conditions
        self.oe.append(orbitalMotion.ClassicElements())
        self.oe[0].a = 1.1*EnvModel.planetRadius  # meters
        self.oe[0].e = 0.01
        self.oe[0].i = 45.0 * macros.D2R
        self.oe[0].Omega = 48.2 * macros.D2R
        self.oe[0].omega = 347.8 * macros.D2R
        self.oe[0].f = 85.3 * macros.D2R
        rN, vN = orbitalMotion.elem2rv(EnvModel.mu, self.oe[0])
        orbitalMotion.rv2elem(EnvModel.mu, rN, vN)
        DynModels[0].scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        DynModels[0].scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        DynModels[0].scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        DynModels[0].scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

        # Configure Dynamics initial conditions
        self.oe.append(copy.deepcopy(self.oe[0]))
        self.oe[1].a = 1.2*EnvModel.planetRadius
        self.oe[1].e = 0.1
        self.oe[1].i = 30.0 * macros.D2R
        rN2, vN2 = orbitalMotion.elem2rv(EnvModel.mu, self.oe[1])
        orbitalMotion.rv2elem(EnvModel.mu, rN2, vN2)
        DynModels[1].scObject.hub.r_CN_NInit = rN2  # m   - r_CN_N
        DynModels[1].scObject.hub.v_CN_NInit = vN2  # m/s - v_CN_N
        DynModels[1].scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        DynModels[1].scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

        # Configure Dynamics initial conditions
        self.oe.append(copy.deepcopy(self.oe[0]))
        self.oe[2].a = 1.3 * EnvModel.planetRadius
        self.oe[2].e = 0.05
        self.oe[2].i = 60.0 * macros.D2R
        rN3, vN3 = orbitalMotion.elem2rv(EnvModel.mu, self.oe[2])
        orbitalMotion.rv2elem(EnvModel.mu, rN3, vN3)
        DynModels[2].scObject.hub.r_CN_NInit = rN3  # m   - r_CN_N
        DynModels[2].scObject.hub.v_CN_NInit = vN3  # m/s - v_CN_N
        DynModels[2].scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        DynModels[2].scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    def log_outputs(self):
        # Process outputs
        DynModels = self.get_DynModel()
        FswModels = self.get_FswModel()

        # Set the sampling time
        self.samplingTime = macros.sec2nano(1)

        # Loop through every spacecraft
        for spacecraft in range(self.numberSpacecraft):

            # log the navigation messages
            self.snTransLog.append(DynModels[spacecraft].simpleNavObject.transOutMsg.recorder(self.samplingTime))
            self.snAttLog.append(DynModels[spacecraft].simpleNavObject.attOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.snTransLog[spacecraft])
            self.AddModelToTask(DynModels[spacecraft].taskName, self.snAttLog[spacecraft])

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
                self.rwLogs[spacecraft].append(
                    DynModels[spacecraft].rwStateEffector.rwOutMsgs[item].recorder(self.samplingTime))
                self.AddModelToTask(DynModels[spacecraft].taskName, self.rwLogs[spacecraft][item])

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
        dataOmegaRW = self.rwSpeedLog[spacecraftIndex].wheelSpeeds

        dataRW = []
        for item in range(DynModels[spacecraftIndex].numRW):
            dataRW.append(self.rwLogs[spacecraftIndex][item].u_current)

        #
        # Plot results
        #

        plt.clear_all_plots()
        timeLineSetMin = self.snTransLog[spacecraftIndex].times() * macros.NANO2MIN
        timeLineSetSec = self.snTransLog[spacecraftIndex].times() * macros.NANO2SEC

        plt.plot_attitude(timeLineSetMin, dataSigmaBN, 1)
        plt.plot_rate(timeLineSetMin, dataOmegaBN_B, 2)
        plt.plot_attitude_error(timeLineSetMin, dataSigmaBR, 3)
        plt.plot_rate_error(timeLineSetMin, dataOmegaBR, 4)
        plt.plot_rw_motor_torque(timeLineSetMin, dataUsReq, dataRW, DynModels[spacecraftIndex].numRW, 5)
        plt.plot_rw_speeds(timeLineSetMin, dataOmegaRW, DynModels[spacecraftIndex].numRW, 6)

        figureList = {}
        if show_plots:
            plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitude", "rate", "attitude_tracking_error", "tracking_error_rate",
                           "rw_motor_torque", "rw_speeds"]
            figureList = plt.save_all_plots(fileName, figureNames)

        # close the plots being saved off to avoid over-writing old and new figures
        plt.clear_all_plots()

        return figureList

def runScenario(scenario):
    # Configure FSW mode
    scenario.FSWModels[0].modeRequest = "sunPointing"
    scenario.FSWModels[1].modeRequest = "standby"
    scenario.FSWModels[2].modeRequest = "locationPointing"

    # Initialize simulation
    scenario.InitializeSimulation()

    # Configure run time and execute simulation
    simulationTime = macros.hour2nano(1.)
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()

    # change flight mode on selected spacecraft
    scenario.FSWModels[1].modeRequest = "sunPointing"
    scenario.FSWModels[2].modeRequest = "inertialPointing"
    scenario.ConfigureStopTime(2*simulationTime)
    scenario.ExecuteSimulation()

def run(show_plots, numberSpacecraft):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        numberSpacecraft (int): Number of spacecraft in the simulation

    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_AttGuidFormationFlying(numberSpacecraft)
    runScenario(TheScenario)
    figureList = TheScenario.pull_outputs(show_plots, 2)

    return figureList


if __name__ == "__main__":
    run(show_plots=True,
        numberSpacecraft=3
        )
