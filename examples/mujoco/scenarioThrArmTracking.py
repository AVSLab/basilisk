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

"""
It's recommended to review the following scenario(s) first (and any
recommended scenario(s) that they may have):

#. ``examples/mujoco/scenarioReactionWheel.py``
#. ``examples/BskSim/scenario_BasicOrbit.py``

Overview
--------

This script demonstrates tracking control of a 6-DOF spacecraft with two
attached 4-DOF thruster arms using the MuJoCo physics engine within the BSK_Sim
architecture.  Scenario settings, including the spacecraft model, trajectory,
controller gains, and task rates, are provided by ``TrackingConfig``.

The multi-body system is created from the MuJoCo XML file
``sat_w_thruster_arms_tracking.xml``.  It consists of a central hub and two
arms, each with four hinged joints, and an end-mounted
thruster.  MuJoCo single-input actuators command the hinged joints and
thrusters, while separate actuators can apply torque directly to the hub.

The script is found in the folder ``basilisk/examples/mujoco`` and executed by using::

      python3 scenarioThrArmTracking.py

.. note::

   The full 8-hour tracking simulation can take several minutes to run,
   depending on the machine.

Two simulation processes are created: one which contains dynamics modules, and
one which contains the Flight Software (FSW) modules.  The Dynamics process
contains a single task with the modules:

#. ``MJScene``: the MuJoCo dynamics module that simulates the multi-body system defined in the XML file
#. ``MJSystemCoM``: extracts the center of mass position and velocity of the multi-body system from the MuJoCo scene
#. ``MJSystemMassMatrix``: extracts the mass matrix of the multi-body system from the MuJoCo scene
#. ``MJJointReactionForces``: extracts the internal reaction forces and torques
   of the multi-body system from the MuJoCo scene

The FSW process contains four tasks: an outer-loop control task, a joint-motion
task, a thruster-firing task, and an always-enabled recorder task.  The recorder
task has no modules by default; scenarios can add recorders that must execute at
every FSW time step.  The outer-loop control task includes:

#. ``stateMerge`` and ``simpleNav``: provide navigation states for the controller
#. ``arcLineHoldRef``: generates the fixed 45-degree out-of-plane translation reference
#. ``locationPointing``: generates the target-pointing attitude reference
#. ``mrpFeedback``: computes the attitude control torque
#. ``inertialCartFeedback``: computes the position tracking control force
#. ``jointThrAllocation``: determines the joint angles and thruster forces
   needed to achieve the desired control torque and force
#. ``thrFiringRound``: determines quantized thruster on-times from the allocated thruster forces

The modules for the joint-motion task include:

#. ``hingedJointArrayMotor``: determines the motor commands for the hinged joints of the thruster arms
#. ``jointMotionCompensator``: optionally determines hub compensation torques that counter arm-motion reaction torques
#. ``jointArrayRefProfiler``: optionally profiles allocated joint references before they are sent to the joint motors

The modules for the thruster-firing task include:

#. ``thrOnTimeToForce``: converts on time to thruster force
#. ``thrJointCompensation``: determines the motor commands to hold hinged joints static during thruster firing

The joint-motion and thruster-firing tasks are enabled and disabled using three
events: joint motion, thruster firing, and coasting.  Each new allocation from
the outer loop begins the event sequence.  The recorder task remains enabled
through these transitions.

- Joint Motion Event:

    - Condition: A new allocated joint-angle and thruster-on-time command is received.
    - Action: Enables joint motion for a nonzero requested pulse.  A zero-pulse
      command does not begin a new thrust sequence.

- Thruster Firing Event:

    - Condition: The joints reach the allocated angle and rate references.
    - Action: Enables the thruster-firing task and applies the cached on-time command.

- Coasting Event:

    - Condition: The requested firing completes, or the outer loop requests no firing.
    - Action: Zeros actuator commands and allows the spacecraft to coast until the next outer-loop update.

Control-Period Timing and Pulse Truncation
------------------------------------------

The outer control loop updates every 10 seconds (0.1 Hz).  At each update, it
allocates a force to each thruster for the coming period.  Since a thruster
fires at a fixed force rather than continuously varying its output,
``thrFiringRound`` converts the allocated force into a requested on-time:
``on-time = 10 s * allocated force / maximum thruster force``.  A request for
half the maximum force therefore becomes a 5-second pulse.  The module clips
the requested on-time to the 10-second period, rounds it to the configured
pulse resolution, and applies the configured minimum on-time.

This calculation uses the *full* outer-loop period; it does not account for
joint motion.  For a nonzero pulse request, the arms first move to their
commanded angles and settle within the position and rate tolerances.  Only
then does firing begin.  The time available to fire is therefore roughly
``max(0, 10 s - joint-motion time)``, and the delivered pulse cannot exceed that
remaining time.  At the next outer-loop update, any unfinished pulse is
stopped and discarded rather than carried into the next period.

For example, if joint motion takes 8 seconds and the requested pulse is
5 seconds, the thruster fires for only about 2 seconds; the remaining
3 seconds are not delivered.  If thrusters request different pulse lengths,
this cutoff can also change the ratio of their delivered impulses.  This is
an intentional saturation of the available firing window, not a reduction
of the on-time request itself.
The controller does not make up the missing impulse within that period.
At the next update, however, it computes a new command from the current
spacecraft state, so the resulting control error can affect later commands.
Because both the requested on-time and the available firing window depend on
the 10-second period, changing the outer-loop period alone does not preserve
the same impulse behavior.

With ``useJointProfiler=True``, the commanded arm movement follows a smooth
9.5-second cubic profile.  When a full move takes nearly that long, the
thrusters have about 0.5 seconds (or less, if settling takes longer) before
the next update.  A request for several seconds of firing can therefore
deliver only a fraction of that time.  The exact start of firing depends on
when the joints meet both the commanded angle and rate tolerances.


Illustration of Simulation Results
----------------------------------

::

    showPlots = True, runTime = 8.0*3600.0, useJointProfiler = False, useMotionPenalty = False

.. image:: ../../../../examples/mujoco/thrArmControlSupport/trackingControl_position.svg
   :align: center

.. image:: ../../../../examples/mujoco/thrArmControlSupport/trackingControl_attitude.svg
   :align: center
"""

import argparse
import inspect
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import macros as mc
from Basilisk.utilities import simHelpers, vizSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
fileNameForImages = os.path.basename(os.path.splitext(__file__)[0])
DEFAULT_RUN_TIME = 8.0 * 3600.0  # [s]
SECONDS_PER_HOUR = 3600.0  # [s/hr]
CUBIC_PROFILE_DURATION = 9.5  # [s]
MOTION_PENALTY_WEIGHT = 1e-5  # [-]

sys.path.append(path + "/mujocoModels")
sys.path.append(path + "/thrArmControlSupport")
import BSK_MujocoDynamics
import BSK_MujocoFSW
from BSK_mujocoMasters import BSKScenario, BSKSim
from trackingConfig import TrackingConfig


class ScenarioThrArmTracking(BSKSim, BSKScenario):
    """Configure and run the thruster-arm tracking simulation."""

    def __init__(self, useJointProfiler: bool = False,
                 useMotionPenalty: bool = False):
        self.config = TrackingConfig()
        if useJointProfiler:
            self.config.useJointProfiler = True
            self.config.jointProfileType = "cubic"
            self.config.jointProfileDuration = CUBIC_PROFILE_DURATION
        if useMotionPenalty:
            self.config.useThetaAllocPenalty = True
            self.config.thetaWeight = MOTION_PENALTY_WEIGHT
        super().__init__(
            fswRate=self.config.fswTimeStep,
            dynRate=self.config.dynTimeStep,
        )
        self.name = "scenarioThrArmTracking"

        self.posRec = None
        self.desPosRec = None
        self.attErrorRec = None

        self.set_DynModel(BSK_MujocoDynamics, self.config)
        self.set_FswModel(BSK_MujocoFSW, self.config)
        self.log_outputs()

    def configure_initial_conditions(self):
        """Set the initial hub and joint states used for the tracking case."""
        dynamics = self.get_DynModel()
        fsw = self.get_FswModel()
        hub = dynamics.scene.getBody(fsw.hubName)
        hub.setPosition([0.5, 19.65, -0.25])  # [m]
        hub.setVelocity([2.5e-3, -4.0e-4, 1.75e-3])  # [m/s]
        hub.setAttitude([0.16, 0.05, -0.54])  # [-]
        hub.setAttitudeRate([1.0e-3, -0.25e-3, -0.95e-3])  # [rad/s]
        for bodyName, jointName in fsw.jointRefs:
            joint = dynamics.scene.getBody(bodyName).getScalarJoint(jointName)
            joint.setPosition(0.0)  # [rad]
            joint.setVelocity(0.0)  # [rad/s]

    def log_outputs(self):
        """Record position and attitude tracking states."""
        dynamics = self.get_DynModel()
        fsw = self.get_FswModel()
        self.posRec = dynamics.systemCoM.comStatesOutMsg.recorder()
        self.desPosRec = fsw.transRef.transRefOutMsg.recorder()
        self.attErrorRec = fsw.attRef.attGuidOutMsg.recorder()
        self.AddModelToTask(fsw.outerLoopTaskName, self.posRec)
        self.AddModelToTask(fsw.outerLoopTaskName, self.desPosRec)
        self.AddModelToTask(fsw.outerLoopTaskName, self.attErrorRec)

    def pull_outputs(self, savePlots: bool = False):
        """Create position and attitude tracking figures."""
        times = self.posRec.times() * mc.NANO2SEC  # [s]
        positionError = (
            np.asarray(self.posRec.r_CN_N) - np.asarray(self.desPosRec.r_RN_N)
        )  # [m]
        sigmaError = np.asarray(self.attErrorRec.sigma_BR)  # [-]

        colorX = simHelpers.getLineColor(0, 3)
        colorY = simHelpers.getLineColor(1, 3)
        colorZ = simHelpers.getLineColor(2, 3)
        figures = {}

        positionFigure = plt.figure()
        plt.plot(times / SECONDS_PER_HOUR, positionError[:, 0], color=colorX, label="x")
        plt.plot(times / SECONDS_PER_HOUR, positionError[:, 1], color=colorY, label="y")
        plt.plot(times / SECONDS_PER_HOUR, positionError[:, 2], color=colorZ, label="z")
        plt.xlabel("Time [hr]")
        plt.ylabel("Position Tracking Error [m]")
        plt.legend()
        figures[fileNameForImages + "_position"] = positionFigure

        attitudeFigure = plt.figure()
        plt.plot(times / SECONDS_PER_HOUR, sigmaError[:, 0], color=colorX, label=r"$\sigma_1$")
        plt.plot(times / SECONDS_PER_HOUR, sigmaError[:, 1], color=colorY, label=r"$\sigma_2$")
        plt.plot(times / SECONDS_PER_HOUR, sigmaError[:, 2], color=colorZ, label=r"$\sigma_3$")
        plt.xlabel("Time [hr]")
        plt.ylabel(r"Attitude Tracking Error [$\boldsymbol{\sigma}_{B/R}$]")
        plt.legend()
        figures[fileNameForImages + "_attitude"] = attitudeFigure

        if savePlots:
            supportPath = os.path.join(path, "thrArmControlSupport")
            positionFigure.savefig(
                os.path.join(supportPath, "trackingControl_position.svg")
            )
            attitudeFigure.savefig(
                os.path.join(supportPath, "trackingControl_attitude.svg")
            )

        return figures


def runScenario(scenario, runTime: float, showProgressBar: bool = True):
    """Initialize and execute a configured tracking scenario."""
    if vizSupport.vizFound:
        dynamics = scenario.get_DynModel()
        vizSupport.enableUnityVisualization(
            scenario,
            dynamics.taskName,
            dynamics.scene,
            # saveFile=__file__,
        )

    scenario.InitializeSimulation()
    scenario.configure_initial_conditions()
    scenario.ConfigureStopTime(mc.sec2nano(runTime))
    scenario.SetProgressBar(showProgressBar)
    scenario.ExecuteSimulation()


def run(showPlots: bool = False, runTime: float = DEFAULT_RUN_TIME,
        showProgressBar: bool = True, savePlots: bool = False,
        useJointProfiler: bool = False, useMotionPenalty: bool = False):
    """
    The scenario can be run with the following setup parameters:

    Args:
        showPlots (bool): Determines if the script displays plots.
        runTime (float): Simulation duration in seconds.
        showProgressBar (bool): Determines if Basilisk displays execution progress.
        savePlots (bool): Determines if the position and attitude figures are
            saved as SVG files.
        useJointProfiler (bool): Determines if a 9.5-second cubic joint
            reference profile is used.  This can leave little firing time
            in the fixed 10-second control period; unfinished pulses are
            discarded at the next outer-loop update.
        useMotionPenalty (bool): Determines if a theta allocation penalty
            with weight ``1e-5`` is used.

    """
    scenario = ScenarioThrArmTracking(useJointProfiler, useMotionPenalty)
    runScenario(scenario, runTime, showProgressBar)
    figures = scenario.pull_outputs(savePlots)
    if showPlots:
        plt.show()
    return figures


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run scenarioThrArmTracking")
    parser.add_argument(
        "--showPlots",
        action="store_true",
        help="Display matplotlib figures.",
    )
    parser.add_argument(
        "--savePlots",
        action="store_true",
        default=False,
        help="Save the position and attitude figures as SVG files.",
    )
    parser.add_argument(
        "--runTime",
        type=float,
        default=DEFAULT_RUN_TIME,
        help="Simulation run time [s].",
    )
    parser.add_argument(
        "--noProgressBar",
        action="store_false",
        dest="showProgressBar",
        default=True,
        help="Disable the Basilisk simulation progress bar.",
    )
    parser.add_argument(
        "--useJointProfiler",
        action="store_true",
        default=False,
        help=(
            "Use a 9.5-second cubic joint profile; unfinished pulses are "
            "discarded at the next 10-second update."
        ),
    )
    parser.add_argument(
        "--useMotionPenalty",
        action="store_true",
        default=False,
        help="Use a theta allocation penalty with weight 1e-5.",
    )
    args = parser.parse_args()
    run(
        showPlots=args.showPlots,
        savePlots=args.savePlots,
        runTime=args.runTime,
        showProgressBar=args.showProgressBar,
        useJointProfiler=args.useJointProfiler,
        useMotionPenalty=args.useMotionPenalty,
    )
