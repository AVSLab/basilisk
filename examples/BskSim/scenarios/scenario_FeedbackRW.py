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

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/34JXysCB77g" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

This script sets up a 6-DOF spacecraft orbiting Earth. The goal of the scenario is to

#. add reaction wheels to :ref:`BSK_Dynamics`, and
#. establish a inertial pointing FSW mode in :ref:`BSK_FSW`.

The script is found in the folder ``basilisk/examples/BskSim/scenarios`` and executed by using::

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

Because the FSW guidance and control are broken up into separate tasks to be enabled, they must share a common
stand-along (gateway) message to connect these tasks.  These are setup with the method ``setupGatewayMsgs()``.
Note that if a C FSW module re-directs its message writing to such a stand-alone message, when recording
the message this stand-alone message should be recorded.  The output message payload within the module itself
remain zero in such a case.

Finally, the :ref:`inertial3D` mode call in :ref:`scenario_FeedbackRW` needs to be triggered by::

     SimBase.createNewEvent("initiateInertial3D", self.processTasksTimeStep, True,
                   ["self.modeRequest == 'inertial3D'"],
                   ["self.fswProc.disableAllTasks()",
                    "self.FSWModels.zeroGateWayMsgs()",
                    "self.enableTask('inertial3DPointTask')",
                    "self.enableTask('mrpFeedbackRWsTask')"])

which disables any existing tasks, zero's all the stand-alone gateway messages
and enables the inertial pointing task and RW feedback task.
This concludes how to construct a preconfigured FSW mode that will be available for any future scenario
that uses the BSK_Sim architecture.

This simulation runs for 10 minutes and then switches the FSW mode to ``directInertial3D``.  The difference here
is that the requested command torque from the ``mrpFeedback`` module in this mode is directly sent to the
spacecraft object as an external torque.  Also, not that while in ``inertial3D`` mode the ``mrpFeedback`` module
receives the RW speeds to feed-forward compensate for them, in the ``directInertial3D`` mode this is not the case.

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


# Get current file path
import inspect
import os
import sys

import numpy as np
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../models')
sys.path.append(path + '/../plotting')
from BSK_masters import BSKSim, BSKScenario
import BSK_Dynamics, BSK_Fsw
import BSK_Plotting as BSK_plt


# Create your own scenario child class
class scenario_AttitudeFeedbackRW(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_AttitudeFeedbackRW, self).__init__()
        self.name = 'scenario_AttitudeFeedbackRW'

        # declare additional class variables
        self.rwSpeedRec = None
        self.rwMotorRec = None
        self.attErrRec = None
        self.rwLogs = []

        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)

        self.configure_initial_conditions()
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the following line
        DynModels = self.get_DynModel()
        vizSupport.enableUnityVisualization(self, DynModels.taskName, DynModels.scObject
                                            # , saveFile=__file__
                                            , rwEffectorList=DynModels.rwStateEffector
                                            )

    def configure_initial_conditions(self):
        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 10000000.0  # meters
        oe.e = 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R

        DynModels = self.get_DynModel()
        mu = DynModels.gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        DynModels.scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        DynModels.scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        DynModels.scObject.hub.omega_BN_BInit = [[0.1], [-0.01], [0.03]]  # rad/s - omega_BN_B

    def log_outputs(self):
        FswModel = self.get_FswModel()
        DynModel = self.get_DynModel()
        samplingTime = FswModel.processTasksTimeStep
        self.rwSpeedRec = DynModel.rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
        self.rwMotorRec = FswModel.cmdRwMotorMsg.recorder(samplingTime)
        self.attErrRec = FswModel.attGuidMsg.recorder(samplingTime)

        self.AddModelToTask(DynModel.taskName, self.rwSpeedRec)
        self.AddModelToTask(DynModel.taskName, self.rwMotorRec)
        self.AddModelToTask(DynModel.taskName, self.attErrRec)

        self.rwLogs = []
        for item in range(4):
            self.rwLogs.append(DynModel.rwStateEffector.rwOutMsgs[item].recorder(samplingTime))
            self.AddModelToTask(DynModel.taskName, self.rwLogs[item])

        return

    def pull_outputs(self, showPlots):
        num_RW = 4  # number of wheels used in the scenario

        # FSW process outputs, remove first data point as it is before FSW is called
        dataUsReq = np.delete(self.rwMotorRec.motorTorque[:, range(num_RW)], 0, 0)
        sigma_BR = np.delete(self.attErrRec.sigma_BR, 0, 0)
        omega_BR_B = np.delete(self.attErrRec.omega_BR_B, 0, 0)
        RW_speeds = np.delete(self.rwSpeedRec.wheelSpeeds[:, range(num_RW)], 0, 0)
        dataRW = []
        for i in range(num_RW):
            dataRW.append(np.delete(self.rwLogs[i].u_current, 0, 0))

        # Plot results
        BSK_plt.clear_all_plots()
        timeData = np.delete(self.rwMotorRec.times(), 0, 0) * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeData, sigma_BR)
        BSK_plt.plot_rw_cmd_actual_torque(timeData, dataUsReq, dataRW, num_RW)
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


def runScenario(scenario):
    # Initialize simulation
    scenario.InitializeSimulation()

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(10.)
    scenario.modeRequest = 'inertial3D'
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()

    simulationTime = macros.min2nano(30.)
    scenario.modeRequest = 'directInertial3D'
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()


def run(showPlots):
    """
        The scenarios can be run with the followings setups parameters:

        Args:
            showPlots (bool): Determines if the script should display plots

    """
    TheScenario = scenario_AttitudeFeedbackRW()
    runScenario(TheScenario)
    figureList = TheScenario.pull_outputs(showPlots)

    return figureList


if __name__ == "__main__":
    run(True)
