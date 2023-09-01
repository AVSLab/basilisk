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

This bskSim script demonstrates how to use sun safe pointing in conjunction with the Eclipse,
RW, CSS Weighted Least Squares Estimator, and
CSS modules to provide attitude guidance as the spacecraft passes through an eclipse while orbiting the Earth.

This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The goal of the scenario is to
illustrate

#. how to add the eclipse module to simulate shadows being cast over a CSS constellation,
#. how to use these added modules to make use of sun safe pointing as a flight software algorithm to control RWs, and
#. configure a custom timestep for the dynamics and FSW processes.

The script is found in the folder ``basilisk/examples/BskSim/scenarios`` and executed by using::

      python3 scenario_AttitudeEclipse.py

The simulation layout is shown in the following illustration.  Two simulation processes are created: one
which contains dynamics modules, and one that contains the Flight Software (FSW) algorithm
modules.

.. image:: /_images/static/test_scenario_AttEclipseUpdated.svg
   :align: center

Given the complexity of the simulation, the standard dynamics and FSW time step of 0.1 seconds leads to excessively long
computational time. The user can change the standard time step for either or both processes by
changing the ``fswRate`` and ``dynRate``::

  TheBSKSim = BSKSim(1.0, 1.0)

The first argument is the FSW time step and the second is the dynamics time step (both units of seconds).
The user is cautioned when setting a changing the standard time step
as too large a time step can lead to propagated inaccuracy.

When the simulation completes several plots are shown for the eclipse shadow factor, the sun direction vector,
attitude error, RW motor torque, and RW speed.

Custom Dynamics Configurations Instructions
-------------------------------------------
The fundamental simulation setup is a combination of the setups used in
:ref:`scenarioAttitudeFeedback` and :ref:`scenarioCSS`.
The dynamics simulation is setup using a :ref:`spacecraft` module to which an Earth gravity
effector is attached. In addition a CSS constellation and RW pyramid are attached.

The new element is adding the eclipse module ``self.eclipseObject`` to the simulation environment.

The module requires spice data regarding the location of the sun, the planets, and the spacecraft
to simulate shadow-casting effects. In combination these inputs can produce an output that is attached to the
CSS constellation which simulates a shadow. The eclipse object output is sent to the individual CSS sensors.

Custom FSW Configurations Instructions
--------------------------------------

The general flight algorithm setup is different than the earlier simulation scripts. Here we
use the :ref:`sunSafePoint` guidance module, the :ref:`CSSWlsEst` module to evaluate the
sun pointing vector, and the :ref:`mrpFeedback` module to provide the desired :math:`{\mathbf L}_r`
control torque vector.

The :ref:`sunSafePoint` guidance module is used to steer the spacecraft to point towards the sun direction vector.
This is used for functionality like safe mode, or a power generation mode. The inputs of the module are the
sun direction vector (as provided by the :ref:`CSSWlsEst` module), as well as the body rate
information (as provided by the simpleNav module).

The ``sHatBdyCmd`` defines the desired body pointing vector that will align with the sun direction vector.
The sun direction vector itself is calculated through the use of a CSS constellation and the :ref:`CSSWlsEst`
module. The
setup for the CSS constellation can be found in the :ref:`scenarioCSS` scenario. The :ref:`CSSWlsEst` module
is a weighted least-squares minimum-norm algorithm used to estimate the body-relative sun heading using a cluster of
coarse sun sensors. The algorithm requires a minimum of three CSS to operate correctly.

Illustration of Simulation Results
----------------------------------

::

    showPlots = True

This plot illustrates the shadow fraction calculated by the CSS as the spacecraft orbits Earth and passes through
the Earth's shadow. 0.0 corresponds with total eclipse and 1.0 corresponds with direct sunlight.

.. image:: /_images/Scenarios/scenario_AttEclipse_shadowFraction.svg
   :align: center

The :ref:`CSSWlsEst` module calculates the position of the sun based on input from the CSS.
The corresponding vector's three
components are plotted. When the spacecraft passes through the eclipse, it sets the sun direction vector to
[0.0,0.0,0.0].

.. image:: /_images/Scenarios/scenario_AttEclipse_sunDirectionVector.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttEclipse_attitudeErrorNorm.svg
   :align: center

The spacecraft does not change attitude if the sun direction vector is not detectable.
Once the CSS rediscovers the sun upon
exiting the eclipse, the spacecraft corrects and realigns with the sun direction vector.

.. image:: /_images/Scenarios/scenario_AttEclipse_rwMotorTorque.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttEclipse_rwSpeed.svg
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
sys.path.append(path + '/..')
from BSK_masters import BSKSim, BSKScenario
import BSK_Dynamics, BSK_Fsw

# Import plotting file for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

sys.path.append(path + '/../../scenarios')

# To begin, one must first create a class that will
# inherent from the masterSim class and provide a name to the sim.
# This is accomplished through:
# Create your own scenario child class
class scenario_AttitudeEclipse(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_AttitudeEclipse, self).__init__(fswRate=1.0, dynRate=1.0)
        self.name = 'scenario_AttitudeEclipse'

        self.shadowRec = None
        self.rwSpeedRec = None
        self.rwMotorRec = None
        self.sunSafeRec = None
        self.cssEstRec = None

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
        oe.a = 7000000.0  # meters
        oe.e = 0.0
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
        samplingTime = self.get_FswModel().processTasksTimeStep

        # Dynamics process outputs: log messages below if desired.
        self.shadowRec = self.get_DynModel().eclipseObject.eclipseOutMsgs[0].recorder(samplingTime)
        self.rwSpeedRec = self.get_DynModel().rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)

        # FSW process outputs
        self.rwMotorRec = self.get_FswModel().cmdRwMotorMsg.recorder(samplingTime)
        self.sunSafeRec = self.get_FswModel().attGuidMsg.recorder(samplingTime)
        self.cssEstRec = self.get_FswModel().cssWlsEst.navStateOutMsg.recorder(samplingTime)

        self.AddModelToTask(self.get_DynModel().taskName, self.shadowRec)
        self.AddModelToTask(self.get_DynModel().taskName, self.rwSpeedRec)
        self.AddModelToTask(self.get_DynModel().taskName, self.rwMotorRec)
        self.AddModelToTask(self.get_DynModel().taskName, self.sunSafeRec)
        self.AddModelToTask(self.get_DynModel().taskName, self.cssEstRec)

        return

    def pull_outputs(self, showPlots):
        num_RW = 4  # number of wheels used in the scenario

        # Dynamics process outputs: pull log messages below if any
        shadowFactor = np.delete(self.shadowRec.shadowFactor, 0, 0)

        # FSW process outputs
        dataUsReq = np.delete(self.rwMotorRec.motorTorque[:, range(num_RW)], 0, 0)
        sigma_BR = np.delete(self.sunSafeRec.sigma_BR, 0, 0)
        omega_BR_B = np.delete(self.sunSafeRec.omega_BR_B, 0, 0)

        RW_speeds = np.delete(self.rwSpeedRec.wheelSpeeds[:, range(num_RW)], 0, 0)
        sunPoint = np.delete(self.cssEstRec.vehSunPntBdy, 0, 0)

        # Plot results
        BSK_plt.clear_all_plots()
        timeData = np.delete(self.cssEstRec.times(), 0, 0) * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeData, sigma_BR)
        BSK_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
        BSK_plt.plot_rate_error(timeData, omega_BR_B)
        BSK_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)

        BSK_plt.plot_shadow_fraction(timeData, shadowFactor)
        BSK_plt.plot_sun_point(timeData, sunPoint)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "rwSpeed", "shadowFraction", "sunDirectionVector"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def runScenario(TheScenario):
    # Initialize simulation
    TheScenario.InitializeSimulation()
    # Configure FSW mode
    TheScenario.modeRequest = 'sunSafePoint'

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(60.0)
    TheScenario.ConfigureStopTime(simulationTime)
    TheScenario.ExecuteSimulation()


def run(showPlots):
    """
        The scenarios can be run with the followings setups parameters:

        Args:
            showPlots (bool): Determines if the script should display plots

    """

    scenario = scenario_AttitudeEclipse()
    runScenario(scenario)
    # Pull the results of the base simulation running the chosen scenario
    figureList = scenario.pull_outputs(showPlots)

    return figureList


if __name__ == "__main__":
    run(True)
