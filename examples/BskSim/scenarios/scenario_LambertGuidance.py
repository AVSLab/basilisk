#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This BSK Sim scenario demonstrates how to use the Lambert solver module package, consisting of
``lambertPlanner()``, ``lambertSolver()`` and ``lambertValidator()``, as well as the modules that can be used for the
second maneuver of the Lambert transfer arc, ``lambertSurfaceRelativeVelocity()`` and ``lambertSecondDV()``. In this
scenario, the goal is to reach a target position and velocity at final time tf (equal to time of the second
maneuver tm2) by performing two maneuvers.

The first maneuver at time tm1 is done by solving Lambert's problem. The
Lambert problem is set up using :ref:`lambertPlanner`, which provides the information in the form of
:ref:`lambertProblemMsgPayload` to :ref:`lambertSolver`. Lambert's problem is solved within
:ref:`lambertSolver`, which writes the :ref:`lambertSolutionMsgPayload` and :ref:`lambertPerformanceMsgPayload` output
messages. Finally, :ref:`lambertValidator` processes the content of those messages, computes the required Delta-V, and
only writes a non-zero Delta-V message within :ref:`dvBurnCmdMsgPayload` if no constraints are violated
(minimum orbit radius and final distance from targeted location) and the Delta-V solution has converged.

The second maneuver at time tm2 is performed at the end of the Lambert transfer arc to match a desired velocity at that
point. In the case of this scenario, the desired velocity is obtained from the ``lambertSurfaceRelativeVelocity()``
module to such that the relative velocity to the central body surface is zero.

The script is found in the folder ``basilisk/examples/BskSim/scenarios`` and executed by using::

      python3 scenario_LambertGuidance.py

The simulation is a more complex simulation than the earlier tutorial for the Lambert solver modules in
:ref:`scenarioLambertSolver`.


Custom Dynamics Configurations Instructions
-------------------------------------------

The modules required for this scenario are identical to those used in :ref:`scenario_BasicOrbit`.


Custom FSW Configurations Instructions
--------------------------------------

The five Lambert modules were added to the :ref:`BSK_FSW` framework.

The first maneuver event is triggered when a user calls `self.masterSim.modeRequest = 'lambertFirstDV'` in any
current or future :ref:`Folder_BskSim` file, and the second maneuver using
`self.masterSim.modeRequest = 'lambertSecondDV'`

Illustration of Simulation Results
----------------------------------

::

    showPlots = True

.. image:: /_images/Scenarios/scenario_LambertGuidance_orbit.svg
   :align: center

.. image:: /_images/Scenarios/scenario_LambertGuidance_position.svg
   :align: center

.. image:: /_images/Scenarios/scenario_LambertGuidance_velocity.svg
   :align: center

.. image:: /_images/Scenarios/scenario_LambertGuidance_surfaceRelativeVelocity.svg
   :align: center

"""

# Get current file path
import inspect
import os
import sys

import numpy as np
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport, unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../models')
sys.path.append(path + '/../plotting')
from BSK_masters import BSKSim, BSKScenario
import BSK_Dynamics
import BSK_Fsw

# Import plotting files for your scenario
import BSK_Plotting as BSK_plt


# Create your own scenario child class
class scenario_LambertGuidance(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_LambertGuidance, self).__init__(fswRate=60., dynRate=10.)
        self.name = 'scenario_LambertGuidance'

        # declare empty class variables
        self.sNavAttRec = None
        self.sNavTransRec = None

        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)

        # set module parameters
        DynModels = self.get_DynModel()
        FswModels = self.get_FswModel()
        rEarth = 6378 * 1e3
        self.r_TN_N = np.array([(42164 * 1e3), 0., 0.])  # targeted position
        pos_sigma_sc = 1.
        vel_sigma_sc = 0.1
        self.tm1 = 2460.
        self.tm2 = 4980.
        vRelativeDesired_S = np.array([0., 0., 0.])
        p_matrix_sc = np.diag([pos_sigma_sc, pos_sigma_sc, pos_sigma_sc,
                               vel_sigma_sc, vel_sigma_sc, vel_sigma_sc,
                               0., 0., 0.,
                               0., 0., 0.,
                               0., 0., 0.,
                               0., 0., 0.])
        walk_bounds_sc = [[10.], [10.], [10.],
                      [1], [1], [1],
                      [0.], [0.], [0.],
                      [0.], [0.], [0.],
                      [0.], [0.], [0.],
                      [0.], [0.], [0.]]

        DynModels.simpleNavObject.PMatrix = p_matrix_sc
        DynModels.simpleNavObject.walkBounds = walk_bounds_sc

        FswModels.lambertPlannerObject.r_TN_N = self.r_TN_N
        FswModels.lambertPlannerObject.finalTime = self.tm2
        FswModels.lambertPlannerObject.maneuverTime = self.tm1
        FswModels.lambertPlannerObject.mu = DynModels.gravFactory.gravBodies['earth'].mu
        FswModels.lambertPlannerObject.numRevolutions = 0

        FswModels.lambertValidatorObject.finalTime = self.tm2
        FswModels.lambertValidatorObject.maneuverTime = self.tm1
        FswModels.lambertValidatorObject.maxDistanceTarget = 500.
        FswModels.lambertValidatorObject.minOrbitRadius = rEarth
        FswModels.lambertValidatorObject.uncertaintyStates = np.diag([pos_sigma_sc, pos_sigma_sc, pos_sigma_sc,
                                                                      vel_sigma_sc, vel_sigma_sc, vel_sigma_sc])
        FswModels.lambertValidatorObject.uncertaintyDV = 0.1
        FswModels.lambertValidatorObject.dvConvergenceTolerance = 1.

        FswModels.lambertSurfaceRelativeVelocityObject.vRelativeDesired_S = vRelativeDesired_S
        FswModels.lambertSurfaceRelativeVelocityObject.time = self.tm2

        self.configure_initial_conditions()
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the following line
        vizSupport.enableUnityVisualization(self, self.DynModels.taskName, self.DynModels.scObject
                                            # , saveFile=__file__
                                            , rwEffectorList=self.DynModels.rwStateEffector
                                            )

    def configure_initial_conditions(self):
        DynModels = self.get_DynModel()

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 40000. * 1e3  # meters
        oe.e = 0.001
        oe.i = 1. * macros.D2R
        oe.Omega = 1. * macros.D2R
        oe.omega = 1. * macros.D2R
        oe.f = -30. * macros.D2R
        mu = DynModels.gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        DynModels.scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        DynModels.scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    def log_outputs(self):
        DynModels = self.get_DynModel()
        FswModels = self.get_FswModel()
        self.sc_truth_recorder = DynModels.scObject.scStateOutMsg.recorder()
        self.sc_meas_recorder = DynModels.simpleNavObject.transOutMsg.recorder()
        self.dv1Cmd_recorder = FswModels.lambertValidatorObject.dvBurnCmdOutMsg.recorder()
        self.dv2Cmd_recorder = FswModels.lambertSecondDvObject.dvBurnCmdOutMsg.recorder()
        self.ephem_recorder = DynModels.EarthEphemObject.ephemOutMsgs[0].recorder()
        self.AddModelToTask(DynModels.taskName, self.sc_truth_recorder)
        self.AddModelToTask(DynModels.taskName, self.sc_meas_recorder)
        self.AddModelToTask(DynModels.taskName, self.dv1Cmd_recorder)
        self.AddModelToTask(DynModels.taskName, self.dv2Cmd_recorder)
        self.AddModelToTask(DynModels.taskName, self.ephem_recorder)

    def pull_outputs(self, showPlots):
        # retrieve logged data
        time = self.sc_truth_recorder.times() * macros.NANO2MIN
        r_BN_N_truth = self.sc_truth_recorder.r_BN_N
        r_BN_N_meas = self.sc_meas_recorder.r_BN_N
        v_BN_N_truth = self.sc_truth_recorder.v_BN_N
        v_BN_N_meas = self.sc_meas_recorder.v_BN_N
        sigma_PN = self.ephem_recorder.sigma_BN
        omega_PN_N = self.ephem_recorder.omega_BN_B

        # Plot results
        BSK_plt.clear_all_plots()
        BSK_plt.plot_orbit(r_BN_N_truth)
        BSK_plt.plot_position(time, np.array(r_BN_N_truth), np.array(r_BN_N_meas), self.tm2 / 60., self.r_TN_N)
        BSK_plt.plot_velocity(time, np.array(v_BN_N_truth), np.array(v_BN_N_meas))
        BSK_plt.plot_surface_rel_velocity(time, r_BN_N_truth, v_BN_N_truth, sigma_PN, omega_PN_N)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["orbit", "position", "velocity", "surfaceRelativeVelocity"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def runScenario(scenario):

    # Initialize simulation
    scenario.InitializeSimulation()
    velRef = scenario.DynModels.scObject.dynManager.getStateObject("hubVelocity")

    # Configure FSW mode for first Lambert maneuver
    scenario.modeRequest = 'lambertFirstDV'

    # Configure run time and execute simulation
    simulationTime = macros.sec2nano(scenario.tm1)
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()

    # Next, the state manager objects are called to retrieve the latest inertial position and
    # velocity vector components:
    vm_N = unitTestSupport.EigenVector3d2np(velRef.getState())

    dv_N = scenario.dv1Cmd_recorder.dvInrtlCmd[-1, :]

    # After reading the Delta-V command, the state managers velocity is updated through
    velRef.setState(unitTestSupport.np2EigenVectorXd(vm_N + dv_N))
    # Configure FSW mode for second Lambert maneuver
    scenario.modeRequest = 'lambertSecondDV'

    # To start up the simulation again, note that the total simulation time must be provided,
    # not just the next incremental simulation time.
    simulationTime = macros.sec2nano(scenario.tm2)
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()

    # Next, the state manager objects are called to retrieve the latest inertial position and
    # velocity vector components:
    vm_N = unitTestSupport.EigenVector3d2np(velRef.getState())

    dv_N = scenario.dv2Cmd_recorder.dvInrtlCmd[-1, :]

    # After reading the Delta-V command, the state managers velocity is updated through
    velRef.setState(unitTestSupport.np2EigenVectorXd(vm_N + dv_N))
    # disable flight software after maneuver
    scenario.modeRequest = 'standby'

    # To start up the simulation again, note that the total simulation time must be provided,
    # not just the next incremental simulation time.
    simulationTime = macros.sec2nano(6000.)
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()


def run(showPlots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots

    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_LambertGuidance()
    runScenario(TheScenario)
    figureList = TheScenario.pull_outputs(showPlots)

    return figureList


if __name__ == "__main__":
    run(True)
