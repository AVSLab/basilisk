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

This script sets up three spacecraft orbiting a planet (Earth or Mercury). The goal of the scenario is to

#. highlight the refactored BSK_Sim structure that allows for easy addition of several spacecraft,
#. demonstrate how to create a formation flying scenario with any number of spacecraft, and
#. show how to customize the environment, dynamics and flight software files.

The script is found in the folder ``basilisk/examples/MultiSatBskSim/scenariosMultiSat`` and is executed by using::

      python3 scenario_BasicOrbitMultiSat.py

The simulation mimics the basic simulation in the earlier tutorials in :ref:`scenario_BasicOrbit` and
:ref:`scenario_BasicOrbitFormation`. :ref:`scenario_BasicOrbit` introduces the use of the BSK_Sim structure and
its advantages, and :ref:`scenario_BasicOrbitFormation` simulates a basic scenario with two spacecraft using the BSK_Sim
structure. However, this last scenario hard codes both spacecraft into the dynamics and flight software (FSW) classes.
While this works for a small number of spacecraft, it is not easily scalable. This example aims to show a more pipelined
way of adding spacecraft that can be either homogeneous or heterogeneous (different dynamics and flight software modules
from one another). Another advantage of these changes is that the simulation now has a separate process for the
environment, dynamics and FSW. When Basilisk supports multi-threading the process evaluation, this will
greatly reduce the time it takes to run the simulation with a large number of spacecraft.

In line with these changes, the environment is not embedded into the dynamics class, as each spacecraft has its own
dynamics class. Therefore, a new environment class has also been created, where any changes can be made to accommodate
different planets, ground locations, etc.

No flight software module is added in this scenario, only the environment and dynamics modules. However, for
organization purposes the flight software module customization will also addressed. A more thorough review of the FSW
class can be read in :ref:`scenario_AttGuidMultiSat`.

Configuring the scenario file
-----------------------------
The simulation layout is very similar to the one used for the :ref:`scenario_BasicOrbitFormation` file. As stated
before, several simulation processes are created: one for the environment and two for each of the spacecraft, each
representing the dynamics and flight software modules. It is absolutely crucial that the environment model be created
first, then the dynamics model for each spacecraft and then the FSW model for each spacecraft. In this case, the
Environment files used are :ref:`BSK_EnvironmentEarth` and :ref:`BSK_EnvironmentMercury`, whereas the Dynamics
file used is the :ref:`BSK_MultiSatDynamics`. The environment used in the simulation can be changed with either the
``Earth`` or ``Mercury`` flag on the ``run`` method. As stated before, the FSW file is not added in this scenario
to focus first on setting up the dynamics of a number of spacecraft. All these files have been
created for this specific formation flying implementation into Basilisk but can be changed to accommodate any changes to
the intended simulation.

After initializing the interfaces and making sure that the :ref:`scenario_BasicOrbitMultiSat`
class inherits from the modified BSKSim class, the initial conditions are configured using the
``configure_initial_conditions`` method. This method cannot take advantage of the new BSK structure, and therefore the
initial conditions for each spacecraft must be hard coded. Three sets of orbital elements are created, each
corresponding to one spacecraft and can adapt to the planet that the spacecraft is orbiting. After that, the orbital
elements are converted to position and velocity and the initial conditions are set for each spacecraft.

After that the function that logs the outputs can be observed. Again, this looks very similar to
the ``log_outputs`` method in the :ref:`scenario_BasicOrbitFormation` file. However, there is one difference: since multiple
spacecraft are simulated, we must loop through each dynamics process and record each module. This makes the logging
variables be lists of arrays instead of simply arrays. The same is true for the FSW objects.

In the ``pull_outputs`` method, the logging variables (position of each spacecraft) are pulled and used to compute a 3D
plot with all the spacecraft's orbits.

BSK_EnvironmentEarth and BSK_EnvironmentMercury files description
-----------------------------------------------------------------
Both the :ref:`BSK_EnvironmentEarth` and :ref:`BSK_EnvironmentMercury` share the same structure, with a difference in the
gravity bodies used: the first uses the Sun, Earth and the Moon, while the second one only uses the Sun and Mercury.

The gravity bodies are created using :ref:`simIncludeGravBody` and their information is overridden by
the SPICE library. An eclipse module is set up using the gravitational bodies used in the simulation. A ground location
(representing Boulder's location on Earth) is also set to be used in flight software. All modules are added to the
environment process.

BSK_MultiSatDynamics file description
-------------------------------------
Looking at the :ref:`BSK_MultiSatDynamics` file, it can be observed that the dynamics process for each spacecraft
consists of one tasks named ``DynamicsTaskX`` where ``X`` represents that spacecraft's index. This task are added to the
corresponding dynamics process and an instance of a specific object is added.

The dynamics class creates a :ref:`spacecraft`, :ref:`simpleNav`, :ref:`reactionWheelStateEffector` and
:ref:`thrusterDynamicEffector` objects. It also creates a :ref:`fuelTank` module that uses truster information to show
the status of the onboard fuel tank. Although no attitude guidance and control is implemented in this example, this
class will be used in other scenarios that make use of those control surfaces (see :ref:`scenario_AttGuidMultiSat` and
:ref:`scenario_StationKeepingMultiSat`).

The dynamics script also sets up a number of power-related modules such as :ref:`simpleSolarPanel`,
:ref:`simplePowerSink`, :ref:`simpleBattery` and :ref:`ReactionWheelPower`.

The necessary connections between the environment and dynamics classes are also done in this file, such as adding the
gravity bodies from the environment into the spacecraft object.

After that, each object is added to the corresponding task. As before, the message names are very important. The
standardization of these names is the only way to properly connect the environment, dynamics and FSW messages properly.

BSK_MultiSatFsw file description
--------------------------------
This file can be added in a similar way to the dynamics file. By adding this file, each spacecraft will have its own FSW
process with specific guidance modes.

Illustration of Simulation Results
----------------------------------

::

    showPlots = True, numberSpacecraft=3, environment = 'Earth'

.. image:: /_images/Scenarios/scenario_BasicOrbitMultiSat_spacecraft_orbits.svg
   :align: center


"""

import copy
# Get current file path
import inspect
import os
import sys

from Basilisk.architecture import messaging
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../modelsMultiSat')
sys.path.append(path + '/../plottingMultiSat')
from BSK_MultiSatMasters import BSKSim, BSKScenario
import BSK_EnvironmentEarth, BSK_EnvironmentMercury, BSK_MultiSatDynamics

# Import plotting files for your scenario
import BSK_MultiSatPlotting as plt

# Create your own scenario child class
class scenario_BasicOrbitFormationFlying(BSKSim, BSKScenario):
    def __init__(self, numberSpacecraft, environment):
        super(scenario_BasicOrbitFormationFlying, self).__init__(numberSpacecraft, fswRate=10, dynRate=10, envRate=10)
        self.name = 'scenario_BasicOrbitFormationFlying'

        if environment == "Earth":
            self.set_EnvModel(BSK_EnvironmentEarth)
        elif environment == "Mercury":
            self.set_EnvModel(BSK_EnvironmentMercury)

        # Here we are setting the list of spacecraft dynamics to be a homogenous formation.
        # To use a heterogeneous spacecraft formation, this list can contain different classes
        # of type BSKDynamicModels
        self.set_DynModel([BSK_MultiSatDynamics]*numberSpacecraft)

        # declare empty class variables
        self.samplingTime = []
        self.snTransLog = []
        self.snAttLog = []
        self.rwSpeedLog = []
        self.rwLogs = [[] for _ in range(self.numberSpacecraft)]

        # declare empty containers for orbital elements
        self.oe = []

        self.configure_initial_conditions()
        self.log_outputs()

        if vizSupport.vizFound:
            # if this scenario is to interface with the BSK Viz, uncomment the saveFile line
            DynModelsList = []
            rwStateEffectorList = []
            for i in range(self.numberSpacecraft):
                DynModelsList.append(self.DynModels[i].scObject)
                rwStateEffectorList.append(self.DynModels[i].rwStateEffector)

            batteryPanel = vizSupport.vizInterface.GenericStorage()
            batteryPanel.label = "Battery"
            batteryPanel.units = "Ws"
            batteryPanel.color = vizSupport.vizInterface.IntVector(vizSupport.toRGBA255("red") + vizSupport.toRGBA255("green"))
            batteryPanel.thresholds = vizSupport.vizInterface.IntVector([20])
            batteryInMsg = messaging.PowerStorageStatusMsgReader()
            batteryInMsg.subscribeTo(self.DynModels[0].powerMonitor.batPowerOutMsg)
            batteryPanel.batteryStateInMsg = batteryInMsg
            batteryPanel.this.disown()

            tankPanel = vizSupport.vizInterface.GenericStorage()
            tankPanel.label = "Tank"
            tankPanel.units = "kg"
            tankPanel.color = vizSupport.vizInterface.IntVector(vizSupport.toRGBA255("cyan"))
            tankInMsg = messaging.FuelTankMsgReader()
            tankInMsg.subscribeTo(self.DynModels[0].fuelTankStateEffector.fuelTankOutMsg)
            tankPanel.fuelTankStateInMsg = tankInMsg
            tankPanel.this.disown()

            storageList = [None]*self.numberSpacecraft
            storageList[0] = [batteryPanel, tankPanel]

            viz = vizSupport.enableUnityVisualization(self, self.DynModels[0].taskName, DynModelsList
                                                      # , saveFile=__file__
                                                      , rwEffectorList=rwStateEffectorList
                                                      , genericStorageList=storageList
                                                      )
            vizSupport.setInstrumentGuiSetting(viz, showGenericStoragePanel=True)

    def configure_initial_conditions(self):
        EnvModel = self.get_EnvModel()
        DynModels = self.get_DynModel()

        # Configure Dynamics initial conditions
        self.oe.append(orbitalMotion.ClassicElements())
        self.oe[0].a = 1.1 * EnvModel.planetRadius  # meters
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
        self.oe[1].a = 1.2 * EnvModel.planetRadius
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

        # Set the sampling time
        self.samplingTime = macros.sec2nano(1)

        # Loop through every spacecraft
        for spacecraft in range(self.numberSpacecraft):

            # log the navigation messages
            self.snTransLog.append(DynModels[spacecraft].simpleNavObject.transOutMsg.recorder(self.samplingTime))
            self.snAttLog.append(DynModels[spacecraft].simpleNavObject.attOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.snTransLog[spacecraft])
            self.AddModelToTask(DynModels[spacecraft].taskName, self.snAttLog[spacecraft])

            # log the RW wheel speed information
            self.rwSpeedLog.append(DynModels[spacecraft].rwStateEffector.rwSpeedOutMsg.recorder(self.samplingTime))
            self.AddModelToTask(DynModels[spacecraft].taskName, self.rwSpeedLog[spacecraft])

            # log addition RW information (power, etc)
            for item in range(DynModels[spacecraft].numRW):
                self.rwLogs[spacecraft].append(
                    DynModels[spacecraft].rwStateEffector.rwOutMsgs[item].recorder(self.samplingTime))
                self.AddModelToTask(DynModels[spacecraft].taskName, self.rwLogs[spacecraft][item])

    def pull_outputs(self, show_plots):
        #
        #   Retrieve the logged data
        #

        r_BN_N = []
        for i in range(self.numberSpacecraft):
            r_BN_N.append(self.snTransLog[i].r_BN_N)

        #
        # Plot results
        #

        plt.clear_all_plots()

        plt.plot_orbits(r_BN_N, self.numberSpacecraft, 1)

        figureList = {}
        if show_plots:
            plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["spacecraft_orbits"]
            figureList = plt.save_all_plots(fileName, figureNames)

        # close the plots being saved off to avoid over-writing old and new figures
        plt.clear_all_plots()

        return figureList


def runScenario(scenario):
    # Initialize simulation
    scenario.InitializeSimulation()

    # Configure run time and execute simulation
    simulationTime = macros.hour2nano(5)
    scenario.ConfigureStopTime(simulationTime)

    scenario.ExecuteSimulation()


def run(show_plots, numberSpacecraft, environment):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        numberSpacecraft (int): Number of spacecraft in the simulation
        environment (string): Chooses which environment to set the simulation in.  Options are "Earth" or "Mercury"

    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_BasicOrbitFormationFlying(numberSpacecraft, environment)
    runScenario(TheScenario)
    figureList = TheScenario.pull_outputs(show_plots)

    return figureList


if __name__ == "__main__":
    run(show_plots=True,
        numberSpacecraft=3,
        environment="Earth"  # Earth or Mercury
        )
