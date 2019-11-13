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

"""
Overview
--------

This script sets up a 6-DOF spacecraft orbiting Earth. The goal of the scenario is to

#. add reaction wheels to :ref:`BSK_Dynamics`, and
#. establish a inertial pointing FSW mode in :ref:`BSK_FSW`.

The script is found in the folder ``src/examples/BskSim/scenarios`` and executed by using::

      python3 scenario_FeedbackRW.py


The simulation mimics the basic simulation simulation in the earlier tutorial in
:ref:`scenarioAttitudeFeedbackRW`.
The simulation layout is shown in the following illustration.

.. image:: /_images/static/test_scenario_FeedbackRW.svg
   :align: center

Two simulation processes are created: one
which contains dynamics modules, and one that contains the FSW
modules. The initial setup for the simulation closely models that of :ref:`scenario_BasicOrbit`.



Custom Dynamics Configurations Instructions
-------------------------------------------

In addition to the modules used in :ref:`scenario_BasicOrbit`, the user must configure the RW module
in :ref:`BSK_Dynamics`
to stabilize the tumbling. This is accomplished by first creating the RW state effector.
The RW object is then configured through ``InitAllDynObjects(SimBase)`` which
includes the ``SetReactionWheelDynEffector()``
function which configures the RW pyramid's properties and messages.


Custom FSW Configurations Instructions
--------------------------------------

To configure the desired :ref:`inertial3D` FSW mode the user must declare the following modules
within the ``__init__()`` function in :ref:`BSK_FSW`.
These provide the initial setup for an attitude guidance system that makes use of an inertial pointing model, a module
that tracks the error of the spacecraft's MRP parameters against the pointing model, and a module that takes that
information to provide a torque to correct for the error.

Following the initial declaration of these configuration modules, :ref:`BSK_FSW`
calls a ``InitAllFSWObjects()`` command,
which, like :ref:`BSK_Dynamics`'s ``InitAllDynObjects()``, calls additional setter functions that configure each
of the FSW modules with the appropriate information and message names.

In addition to the modules used for attitude guidance, there are also two setter functions that send vehicle and RW
configuration messages that are linked into the attitude guidance modules called ``SetVehicleConfiguration()`` and
``SetRWConfigMsg()``.

After each configuration module has been properly initialized with various message names, FSW tasks are generated.
The two tasks required for the :ref:`inertial3D` mode are ``inertial3DPointTask`` and ``mrpFeedbackRWsTask``.
Note how the tasks are divided between the pointing model and control loop. These modular tasks allow
for simple FSW reconfigurations should the user want to use a different pointing model, but to use the same feedback
control loop. This will be seen and discussed in later scenarios.


Finally, the :ref:`inertial3D` mode call in :ref:`scenario_FeedbackRW` needs to be triggered by::

     SimBase.createNewEvent("initiateInertial3D", self.processTasksTimeStep, True,
                   ["self.modeRequest == 'inertial3D'"],
                   ["self.fswProc.disableAllTasks()",
                    "self.enableTask('inertial3DPointTask')",
                    "self.enableTask('mrpFeedbackRWsTask')"])

which disables any existing tasks and enables the inertial pointing task and RW feedback task.
This concludes how to construct a preconfigured FSW mode that will be available for any future scenario
that uses the BSK_Sim architecture.

Illustration of Simulation Results
----------------------------------

::

    showPlots = True

.. image:: /_images/Scenarios/scenario_FeedbackRW_attitudeErrorNorm.svg
   :align: center

.. image:: /_images/Scenarios/scenario_FeedbackRW_rwMotorTorque.svg
   :align: center

.. image:: /_images/Scenarios/scenario_FeedbackRW_rateError.svg
   :align: center

.. image:: /_images/Scenarios/scenario_FeedbackRW_rwSpeed.svg
   :align: center

"""


# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport, vizSupport


# Get current file path
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_masters import BSKSim, BSKScenario
import BSK_Dynamics, BSK_Fsw

# Import plotting file for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

# The scenario must inherit from the BSK_master class using:
class scenario_AttitudeFeedbackRW(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_AttitudeFeedbackRW, self).__init__(masterSim)
        self.name = 'scenario_AttitudeFeedbackRW'
        self.masterSim = masterSim

    def configure_initial_conditions(self):
        print('%s: configure_initial_conditions' % self.name)
        # Within configure_initial_conditions(), the user needs to first define the spacecraft
        # FSW mode for the simulation through:
        self.masterSim.modeRequest = 'inertial3D'
        # which triggers the `initiateInertial3D` event within the BSK_FSW.py script.

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 10000000.0  # meters
        oe.e = 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R

        mu = self.masterSim.get_DynModel().gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.masterSim.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        # The initial conditions for the scenario are the same as found within scenario_BasicOrbit.py except the tumble of the
        # spacecraft must be simulated by adding:
        self.masterSim.get_DynModel().scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.masterSim.get_DynModel().scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    def log_outputs(self):
        print('%s: log_outputs' % self.name)

        # Dynamics process outputs: log messages below if desired.

        # FSW process outputs
        samplingTime = self.masterSim.get_FswModel().processTasksTimeStep
        # Within BSK_Scenario.py log_outputs(), the user must log additional messages to observe how the spacecraft corrected
        # for its initial tumbling through:
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().mrpFeedbackRWsData.inputRWSpeedsName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.get_FswModel().trackingErrorData.outputDataName, samplingTime)
        return

    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)
        num_RW = 4 # number of wheels used in the scenario

        # Dynamics process outputs: pull log messages below if any

        # FSW process outputs
        dataUsReq = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().rwMotorTorqueData.outputDataName + ".motorTorque", list(range(num_RW)))
        sigma_BR = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", list(range(3)))
        omega_BR_B = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", list(range(3)))
        RW_speeds = self.masterSim.pullMessageLogData(
            self.masterSim.get_FswModel().mrpFeedbackRWsData.inputRWSpeedsName + ".wheelSpeeds", list(range(num_RW)))

        # Plot results
        BSK_plt.clear_all_plots()
        timeData = dataUsReq[:, 0] * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeData, sigma_BR)
        BSK_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
        BSK_plt.plot_rate_error(timeData, omega_BR_B)
        BSK_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)
        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "rwSpeed"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def run(showPlots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots

    """
    # Instantiate base simulation
    TheBSKSim = BSKSim()
    TheBSKSim.set_DynModel(BSK_Dynamics)
    TheBSKSim.set_FswModel(BSK_Fsw)
    TheBSKSim.initInterfaces()

    # Configure a scenario in the base simulation
    TheScenario = scenario_AttitudeFeedbackRW(TheBSKSim)
    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    # vizSupport.enableUnityVisualization(TheBSKSim, TheBSKSim.DynModels.taskName, TheBSKSim.DynamicsProcessName,
    #                                     gravBodies=TheBSKSim.DynModels.gravFactory,
    #                                     saveFile=filename,
    #                                     numRW=TheBSKSim.DynModels.rwFactory.getNumOfDevices())

    # Initialize simulation
    TheBSKSim.InitializeSimulationAndDiscover()

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(10.)
    TheBSKSim.ConfigureStopTime(simulationTime)
    print('Starting Execution')
    TheBSKSim.ExecuteSimulation()
    print('Finished Execution. Post-processing results')

    # Pull the results of the base simulation running the chosen scenario
    figureList = TheScenario.pull_outputs(showPlots)

    return figureList


if __name__ == "__main__":
    run(True)
