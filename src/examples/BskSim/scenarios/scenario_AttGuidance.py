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

r"""
Overview
--------

This script sets up a 6-DOF spacecraft orbiting Earth. The goal of the scenario is to
make use of the hill pointing module with
the :ref:`MRP_Feedback` module and a reaction wheel pyramid
to control the attitude all within the new BSK_Sim architecture.

The script is found in the folder ``src/examples/BskSim/scenarios`` and executed by using::

      python3 scenario_AttGuidance.py

The simulation mimics the basic simulation simulation in the earlier tutorial in
:ref:`scenarioAttitudeGuidance`.


The simulation layout is shown in the following illustration.

.. image:: /_images/static/test_scenario_AttGuidance.svg
   :align: center

The initial setup for the simulation closely models that of :ref:`scenario_FeedbackRW`.


Custom Dynamics Configurations Instructions
-------------------------------------------

The modules required for this scenario are identical to those used in :ref:`scenario_FeedbackRW`.


Custom FSW Configurations Instructions
--------------------------------------

Three of the four modules required to configure the :ref:`hillPoint` FSW mode have already been included
within the :ref:`BSK_FSW` framework
(``mrpFeedbackRWConfig()``, ``attTrackingErrorConfig()``, ``rwMotorTorqueConfig()``). The only remaining
module is the hill pointing module itself which is set within ``__init__()``.

These modules provide the initial setup for an attitude guidance system that makes use of an hill
pointing model, a module
that tracks the error of the spacecraft's MRP parameters against the vector pointing towards the central, planetary
body, and uses a module that takes that information to provide a torque to correct for the error.

This event is triggered when a user calls `self.masterSim.modeRequest = 'hillPoint'` in any
current or future :ref:`Folder_BskSim` file.

Illustration of Simulation Results
----------------------------------

::

    showPlots = True

.. image:: /_images/Scenarios/scenario_AttGuidance_attitudeErrorNorm.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttGuidance_rwMotorTorque.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttGuidance_rateError.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttGuidance_orientation.svg
   :align: center

"""



# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport

# Get current file path
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_masters import BSKSim, BSKScenario
import BSK_Dynamics, BSK_Fsw

# Import plotting files for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

# Create your own scenario child class
class scenario_HillPointing(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_HillPointing, self).__init__()
        self.name = 'scenario_AttGuidance'

        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)
        self.initInterfaces()

        self.configure_initial_conditions()
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the following line
        # vizSupport.enableUnityVisualization(self, self.DynModels.taskName, self.DynamicsProcessName,
        #                                     gravBodies=self.DynModels.gravFactory,
        #                                     saveFile=__file__)

    def configure_initial_conditions(self):
        print('%s: configure_initial_conditions' % self.name)
        # Configure FSW mode
        self.modeRequest = 'hillPoint'

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 10000000.0  # meters
        oe.e = 0.1
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R
        mu = self.get_DynModel().gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.get_DynModel().scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        self.get_DynModel().scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        self.get_DynModel().scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B


    def log_outputs(self):
        print('%s: log_outputs' % self.name)
        # Dynamics process outputs
        samplingTime = self.get_DynModel().processTasksTimeStep
        self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject.outputAttName, samplingTime)
        self.TotalSim.logThisMessage(self.get_DynModel().simpleNavObject.outputTransName, samplingTime)

        # FSW process outputs
        samplingTime = self.get_FswModel().processTasksTimeStep
        self.TotalSim.logThisMessage(self.get_FswModel().hillPointData.outputDataName, samplingTime)
        self.TotalSim.logThisMessage(self.get_FswModel().trackingErrorData.outputDataName, samplingTime)
        self.TotalSim.logThisMessage(self.get_FswModel().mrpFeedbackRWsData.outputDataName, samplingTime)

    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)
        # Dynamics process outputs
        sigma_BN = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputAttName + ".sigma_BN", list(range(3)))
        r_BN_N = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N", list(range(3)))
        v_BN_N = self.pullMessageLogData(self.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N", list(range(3)))

        # FSW process outputs
        sigma_RN = self.pullMessageLogData(self.get_FswModel().trackingErrorData.inputRefName + ".sigma_RN", list(range(3)))
        omega_RN_N = self.pullMessageLogData(self.get_FswModel().trackingErrorData.inputRefName + ".omega_RN_N", list(range(3)))
        sigma_BR = self.pullMessageLogData(self.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", list(range(3)))
        omega_BR_B = self.pullMessageLogData(self.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", list(range(3)))
        Lr = self.pullMessageLogData(self.get_FswModel().mrpFeedbackRWsData.outputDataName + ".torqueRequestBody", list(range(3)))

        # Plot results
        BSK_plt.clear_all_plots()
        timeLineSet = sigma_BR[:, 0] * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeLineSet, sigma_BR)
        BSK_plt.plot_control_torque(timeLineSet, Lr)
        BSK_plt.plot_rate_error(timeLineSet, omega_BR_B)
        BSK_plt.plot_orientation(timeLineSet, r_BN_N, v_BN_N, sigma_BN)
        BSK_plt.plot_attitudeGuidance(sigma_RN, omega_RN_N)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "orientation", "attitudeGuidance"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList

def runScenario(TheScenario):


    # Initialize simulation
    TheScenario.InitializeSimulationAndDiscover()

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(10.)
    TheScenario.ConfigureStopTime(simulationTime)

    TheScenario.ExecuteSimulation()


def run(showPlots):
    """
        The scenarios can be run with the followings setups parameters:

        Args:
            showPlots (bool): Determines if the script should display plots

    """
    # Instantiate base simulation
    scenario = scenario_HillPointing()
    runScenario(scenario)
    figureList = scenario.pull_outputs(showPlots)
    return figureList


if __name__ == "__main__":
    run(True)
