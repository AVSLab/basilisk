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

This script demonstrates how to use event handlers to add reaction wheel faults. The script is found in the folder ``basilisk/examples/BskSim/scenarios`` and executed by using::

      python3 scenario_AddRWFault.py
      
Using event handlers
--------------------

The event handler class, defined in ``SimulationBaseClass``, allows the user to define events. These events can be used for a variety of reasons, ranging from changing flight modes to creating faults. The events used in this example are defined in ``BSK_Dynamics``.

When creating an event handler, the following syntax is used::

    SimBase.createNewEvent("eventName", eventRate, eventActive,
        conditionList=[], actionList = [])

Within the condition list should be the conditions for executing the event, specified as strings. Likewise, the action list enumerates strings that are executed when the event happens. Because the code parses these strings, rather than having complex conditions or actions in the string literals, it can be convenient to create methods (see the examples below).

By default, after the event has happened once, the eventActive flag is turned to false. For repeated events, this behavior can be overriden by calling the function ``setEventActivity``. Likewise, the eventActive flags of other events can be set using ``setAllButCurrentEventActivity``. See the associated documentation for more details.

Setting up the faults
---------------------

This script uses two event handlers. The first one defines an event that injects a large, one-time static friction fault::

        SimBase.createNewEvent("addOneTimeRWFault", self.processTasksTimeStep, True,
            ["self.TotalSim.CurrentNanos>=self.oneTimeFaultTime and self.oneTimeRWFaultFlag==1"],
            ["self.DynModels.AddRWFault('friction',0.05,1, self.TotalSim.CurrentNanos)", "self.oneTimeRWFaultFlag=0"])

For this event, the conditions are that the time for the fault has passed, and that the corresponding fault flag is active. The fault time is specified in the scenario script. The ``oneTimeRWFaultFlag`` and the ``repeatRWFaultFlag``, also set in the scenario script, ensures that the faults are added only for the fault scenario.

The second event handler defines an event that is always active, and adds a smaller static friction fault with small probability::

        SimBase.createNewEvent("addRepeatedRWFault", self.processTasksTimeStep, True,
            ["self.repeatRWFaultFlag==1"],
            ["self.DynModels.PeriodicRWFault(1./3000,'friction',0.005,1, self.TotalSim.CurrentNanos)", "self.setEventActivity('addRepeatedRWFault',True)"])

Note the command ``"self.setEventActivity('addRepeatedRWFault',True)"``, keeping the eventActive flag turned on for this event handler. For both event handlers, the particular methods that change the reaction wheel friction parameters are defined in ``BSK_Dynamics``.

Illustration of Simulation Results
----------------------------------

::

    showPlots = True

.. image:: /_images/Scenarios/scenario_AddRWFault_rateError.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AddRWFault_attitudeErrorNorm.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AddRWFault_RWSpeeds.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AddRWFault_RWFriction.svg
   :align: center

"""

# Get current file path
import inspect
import os
import sys

import numpy as np
from Basilisk.utilities import orbitalMotion, macros

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
class scenario_AddRWFault(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_AddRWFault, self).__init__()
        self.name = 'scenario_AddRWFault'

        # declare additional class variables
        self.msgRecList = {}
        self.sNavTransName = "sNavTransMsg"
        self.attGuidName = "attGuidMsg"

        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)

        self.configure_initial_conditions()
        self.log_outputs()
        
        self.oneTimeRWFaultFlag = 1
        self.repeatRWFaultFlag = 1
        self.oneTimeFaultTime = macros.min2nano(10.)
        
        DynModels = self.get_DynModel()
        self.DynModels.RWFaultLog = []

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
        DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    def log_outputs(self):
        FswModel = self.get_FswModel()
        DynModel = self.get_DynModel()
        samplingTime = FswModel.processTasksTimeStep

        self.rwSpeedRec = DynModel.rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.rwSpeedRec)

        self.msgRecList[self.attGuidName] = FswModel.attGuidMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList[self.attGuidName])

        self.msgRecList[self.sNavTransName] = DynModel.simpleNavObject.transOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList[self.sNavTransName])

        self.rwLogs = []
        for item in range(4):
            self.rwLogs.append(DynModel.rwStateEffector.rwOutMsgs[item].recorder(samplingTime))
            self.AddModelToTask(DynModel.taskName, self.rwLogs[item])

        return

    def pull_outputs(self, showPlots):
        # FSW process outputs, remove first data point as it is before FSW is called
        attErrRec = self.msgRecList[self.attGuidName]

        sigma_BR = np.delete(attErrRec.sigma_BR, 0, 0)
        omega_BR_B = np.delete(attErrRec.omega_BR_B, 0, 0)
        
        num_RW = 4
        RW_speeds = np.delete(self.rwSpeedRec.wheelSpeeds[:, range(num_RW)], 0, 0)
        RW_friction = []
        for i in range(num_RW):
            RW_friction.append(np.delete(self.rwLogs[i].u_f, 0, 0))

        # Plot results
        BSK_plt.clear_all_plots()
        timeData = np.delete(attErrRec.times(), 0, 0) * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeData, sigma_BR)
        BSK_plt.plot_rate_error(timeData, omega_BR_B)
        BSK_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)
        BSK_plt.plot_rw_friction(timeData, RW_friction, num_RW, self.DynModels.RWFaultLog)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rateError", "RWSpeeds", "RWFriction"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def runScenario(scenario):
    """method to initialize and execute the scenario"""

    simulationTime = macros.min2nano(30.)
    scenario.modeRequest = "hillPoint"
        
    # Run the simulation
    scenario.InitializeSimulation()
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()

    return


def run(showPlots):
    """
        The scenarios can be run with the following parameters:

        Args:
            showPlots (bool): Determines if the script should display plots

    """
    scenario = scenario_AddRWFault()
    runScenario(scenario)
    figureList = scenario.pull_outputs(showPlots)
    return figureList

if __name__ == "__main__":
    run(True)
