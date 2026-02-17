#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario demonstrates fault scheduling using a fault list. The script lives in
``basilisk/examples/BskSim/scenarios`` and can be executed with::

      python3 scenario_FaultList.py

Scenario Purpose
----------------

This file shows how to:

1. Build a mixed ``faultList`` containing different fault classes from
   ``BSK_Faults``.
2. Register each fault as a simulation event through
   ``fault.addFaultToSimulation(...)``.
3. Run a long-duration simulation with both one-time and recurring faults.
4. Plot attitude/rate performance together with magnetometer output and latitude.

Default Fault Configuration
---------------------------

By default this scenario injects:

- One reaction wheel power-limit fault at 5 minutes.
- A latitude-dependent magnetometer noise update every 5 minutes
  (``MagPolarNoise`` with ``faultType="NOISE"``) across the full simulation.

Notes
-----

The orbital setup is a near-polar, Sun-synchronous-like orbit to emphasize
high-latitude behavior in the magnetometer fault model.

Illustration of Simulation Results
----------------------------------

::

    showPlots = True

.. image:: /_images/Scenarios/scenario_FaultList_attitudeErrorNorm.svg
   :align: center

.. image:: /_images/Scenarios/scenario_FaultList_rateError.svg
   :align: center

.. image:: /_images/Scenarios/scenario_FaultList_RWSpeeds.svg
   :align: center

.. image:: /_images/Scenarios/scenario_FaultList_Magnetometer.svg
   :align: center

.. image:: /_images/Scenarios/scenario_FaultList_Latitude.svg
   :align: center

"""

# Get current file path
import inspect
import os
import sys

import numpy as np
from Basilisk.utilities import macros, orbitalMotion

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../models')
sys.path.append(path + '/../plotting')
import BSK_Dynamics
import BSK_Fsw
import BSK_Plotting as BSK_plt
from BSK_masters import BSKScenario, BSKSim
import BSK_Faults


# Create your own scenario child class
class scenario_FaultList(BSKSim, BSKScenario):
    def __init__(self, fswRate=0.1, dynRate=0.1):
        super(scenario_FaultList, self).__init__(fswRate=fswRate, dynRate=dynRate)
        self.name = "scenario_FaultList"

        # declare additional class variables
        self.msgRecList = {}
        self.sNavTransName = "sNavTransMsg"
        self.attGuidName = "attGuidMsg"

        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)

        self.configure_initial_conditions()
        self.log_outputs()

        self.DynModels.RWFaultLog = []

    def configure_initial_conditions(self):
        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 6835525.292
        oe.e = 0.00063
        oe.i = 98.0 * macros.D2R  # SUN-SYNCHRONOUS POLAR
        oe.Omega = 85.47 * macros.D2R
        oe.omega = 0.0 * macros.D2R
        oe.f = 30.0 * macros.D2R

        DynModels = self.get_DynModel()
        mu = DynModels.gravFactory.gravBodies["earth"].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        DynModels.scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        DynModels.scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    def configure_faults(self, faultList):
        self.faultList = faultList
        # Add each unique fault event to the sim using the event handler.
        for i, fault in enumerate(self.faultList):
            fault.addFaultToSimulation(self, i)

    def log_outputs(self):
        FswModel = self.get_FswModel()
        DynModel = self.get_DynModel()
        samplingTime = FswModel.processTasksTimeStep

        self.rwSpeedRec = DynModel.rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.rwSpeedRec)

        self.msgRecList[self.attGuidName] = FswModel.attGuidMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList[self.attGuidName])

        self.msgRecList[self.sNavTransName] = DynModel.simpleNavObject.transOutMsg.recorder(
            samplingTime
        )
        self.AddModelToTask(DynModel.taskName, self.msgRecList[self.sNavTransName])

        self.rwLogs = []
        for item in range(4):
            self.rwLogs.append(
                DynModel.rwStateEffector.rwOutMsgs[item].recorder(samplingTime)
            )
            self.AddModelToTask(DynModel.taskName, self.rwLogs[item])

        self.magLog = DynModel.magModule.envOutMsgs[0].recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.magLog)
        self.tamLog = DynModel.TAM.tamDataOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.tamLog)

        self.scStatesLog = DynModel.scObject.scStateOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.scStatesLog)

        self.earthStateLog = (
            DynModel.gravFactory.spiceObject.planetStateOutMsgs[DynModel.earth].recorder(
                samplingTime
            )
        )
        self.AddModelToTask(DynModel.taskName, self.earthStateLog)

    def pull_outputs(self, showPlots):
        # FSW process outputs, remove first data point as it is before FSW is called
        attErrRec = self.msgRecList[self.attGuidName]

        sigma_BR = np.delete(attErrRec.sigma_BR, 0, 0)
        omega_BR_B = np.delete(attErrRec.omega_BR_B, 0, 0)

        num_RW = 4
        RW_speeds = np.delete(self.rwSpeedRec.wheelSpeeds[:, range(num_RW)], 0, 0)

        tamData = np.delete(self.tamLog.tam_S, 0, 0)

        scLocData = np.delete(self.msgRecList[self.sNavTransName].r_BN_N, 0, 0)
        J20002Pfix = np.delete(self.earthStateLog.J20002Pfix, 0, 0)

        # Plot results
        BSK_plt.clear_all_plots()
        timeData = np.delete(attErrRec.times(), 0, 0) * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeData, sigma_BR)
        BSK_plt.plot_rate_error(timeData, omega_BR_B)
        BSK_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)
        BSK_plt.plot_data_tam(timeData, tamData)
        BSK_plt.plot_data_lat(timeData, scLocData, J20002Pfix)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = [
                "attitudeErrorNorm",
                "rateError",
                "RWSpeeds",
                "Magnetometer",
                "Latitude",
            ]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def runScenario(scenario, faultList, simulationTime_min=100):
    """Initialize and execute the scenario."""

    simulationTime = macros.min2nano(simulationTime_min)
    scenario.modeRequest = "hillPoint"
    scenario.configure_faults(faultList)

    # Run the simulation
    scenario.InitializeSimulation()
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()


def run(showPlots):
    """
    Run the fault-list scenario.

    Args:
        showPlots (bool): Determines if the script should display plots.
    """

    fswRate = 1.0  # seconds
    dynRate = 1.0  # seconds
    scenario = scenario_FaultList(fswRate=fswRate, dynRate=dynRate)

    rwPowerFault = BSK_Faults.RwPowerFault(
        time=macros.min2nano(5.0),
        reducedLimit=0.01,
        wheelIdx=1,  # 0-based index: 1 -> second wheel (RW2)
    )

    faultRate = 1.0 # minutes
    faultPeriod = macros.min2nano(faultRate)
    simulationTime_min = 50 # minutes
    tStop = macros.min2nano(simulationTime_min)
    numFaults = int(tStop // faultPeriod) + 1

    latitudeDependentMagNoiseList = [
        BSK_Faults.MagPolarNoise(
            time=i * faultPeriod,
            faultType="NOISE",
        )
        for i in range(numFaults)
    ]

    faultList = [
        rwPowerFault,
        *latitudeDependentMagNoiseList,
    ]
    runScenario(scenario, faultList, simulationTime_min=simulationTime_min)
    figureList = scenario.pull_outputs(showPlots)
    return figureList


if __name__ == "__main__":
    run(True)
