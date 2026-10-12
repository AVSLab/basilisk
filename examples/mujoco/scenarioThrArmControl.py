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

This script demonstrates regulation control of a 6-DOF spacecraft with two
attached 4-DOF thruster arms using the MuJoCo physics engine within the BSK_Sim
architecture.  Scenario settings, including the spacecraft model, controller
gains, and task rates, are provided by ``RegulationConfig``.

The multi-body system is created from the MuJoCo XML file
``sat_w_thruster_arms_regulation.xml``.  It consists of a central hub and two
arms, each with four hinged joints, and an end-mounted
thruster.  MuJoCo single-input actuators command the hinged joints and
thrusters, while separate actuators can apply torque directly to the hub.

The script is found in the folder ``basilisk/examples/mujoco`` and executed by using::

      python3 scenarioThrArmControl.py

Two simulation processes are created: one
which contains dynamics modules, and one that contains the Flight Software (FSW)
modules. The Dynamics process contains a single task with the modules:

#. ``MJScene``: the MuJoCo dynamics module that simulates the multi-body system defined in the XML file
#. ``MJSystemCoM``: extracts the center of mass position and velocity of the multi-body system from the MuJoCo scene
#. ``MJSystemMassMatrix``: extracts the mass matrix of the multi-body system from the MuJoCo scene
#. ``MJJointReactionForces``: extracts the internal reaction forces and torques of the multi-body system from the MuJoCo scene

The FSW process contains four tasks: an outer-loop control task, a joint-motion
task, a thruster-firing task, and an always-enabled recorder task.  The recorder
task has no modules by default; scenarios can add recorders that must execute at
every FSW time step.  The outer-loop control task includes:

#. ``inertial3D``: generates an inertial reference attitude
#. ``attError``: computes the attitude error between the current and reference attitude
#. ``mrpFeedback``: computes the attitude control torque
#. ``inertialCartFeedback``: computes the positional control force
#. ``jointThrAllocation``: determines the joint angles and thruster forces needed to achieve the desired control torque and force
#. ``thrFiringRound``: determines quantized thruster on-times from the allocated thruster forces

The modules for the joint motion task include:

#. ``hingedJointArrayMotor``: determines the motor commands for the hinged joints of the thruster arms
#. ``jointMotionCompensator``: optionally determines hub compensation torques that counter arm-motion reaction torques
#. ``jointArrayRefProfiler``: optionally profiles allocated joint references before they are sent to the joint motors

The modules for the thruster firing task include:

#. ``thrOnTimeToForce``: converts on time to thruster force
#. ``thrJointCompensation``: determines the motor commands to hold hinged joints static during thruster firing

The joint-motion and thruster-firing tasks are enabled and disabled using three
events: joint motion, thruster firing, and coasting.  Each new allocation from
the outer loop begins the event sequence.  The recorder task remains enabled
through these transitions.

- Joint Motion Event:

    - Condition: A new allocated joint-angle and thruster-on-time command is received.
    - Action: Enables joint motion for a nonzero requested pulse.  A zero-pulse command does not begin a new thrust sequence.

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


Illustration of Simulation Results
----------------------------------

::

    showPlots = True, runTime = 500.0

.. image:: ../../../../examples/mujoco/thrArmControlSupport/regulationControl_position.svg
   :align: center

.. image:: ../../../../examples/mujoco/thrArmControlSupport/regulationControl_attitude.svg
   :align: center


"""
# Get current file path
import inspect
import os
import sys
import argparse
import matplotlib.pyplot as plt

from Basilisk.utilities import (
    macros as mc,
    vizSupport,
)
from Basilisk.utilities import simHelpers

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
fileNameForImages = os.path.basename(os.path.splitext(__file__)[0])

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/mujocoModels')
sys.path.append(path + '/thrArmControlSupport')
import BSK_MujocoDynamics
import BSK_MujocoFSW
from BSK_mujocoMasters import BSKSim, BSKScenario
from regulationConfig import RegulationConfig

# Create scenario child class
class scenarioThrArmControl(BSKSim, BSKScenario):
    def __init__(self):
        self.config = RegulationConfig()
        super(scenarioThrArmControl, self).__init__(
            fswRate=self.config.fswTimeStep,
            dynRate=self.config.dynTimeStep,
        )
        self.name = "scenarioThrArmControl"

        # Declare empty class variables
        self.posRec = None
        self.attRec = None

        self.set_DynModel(BSK_MujocoDynamics, self.config)
        self.set_FswModel(BSK_MujocoFSW, self.config)
        self.log_outputs()

    def configure_initial_conditions(self):
        # Set initial conditions for the MuJoCo simulation here
        DynModel = self.get_DynModel()
        FSWModel = self.get_FswModel()
        hubName = FSWModel.hubName
        jointRefs = FSWModel.jointRefs
        DynModel.scene.getBody(hubName).setPosition([1.5,-1.0,0.75])              #[m] Initial position of deputy's hub in inertial frame
        DynModel.scene.getBody(hubName).setVelocity([5e-3,-5e-3,5e-3])          #[m/s] Initial velocity of deputy's hub in inertial frame
        DynModel.scene.getBody(hubName).setAttitude([0.15,-0.1,0.1])            #[-] Initial attitude between the deputy's body frame and the inertial frame expressed in MRP
        DynModel.scene.getBody(hubName).setAttitudeRate([0.01,-0.01,0.005])     #[rad/s] Initial angular velocity of the deputy body frame relative to the inertial frame, expressed in the body frame
        for _, (bodyName, jointName) in enumerate(jointRefs):
            joint = DynModel.scene.getBody(bodyName).getScalarJoint(jointName)
            # Setting the joint position to zero causes the spacecraft hub location to be collocated with the
            # system center of mass position.
            joint.setPosition(0.0)  #[rad] Initial joint angle
            # Setting the joint velocity to zero ensures that the spacecraft hub velocity is the same as the
            # system center of mass velocity.
            joint.setVelocity(0.0)  #[rad/s] Initial joint angular velocity

    def log_outputs(self):
        DynModel = self.get_DynModel()
        self.posRec = DynModel.systemCoM.comStatesOutMsg.recorder()
        self.attRec = DynModel.scene.getBody("hub").getOrigin().stateOutMsg.recorder()
        self.AddModelToTask(DynModel.taskName, self.posRec)
        self.AddModelToTask(DynModel.taskName, self.attRec)

    def pull_outputs(self, showPlots: bool = False, savePlots: bool = False):
        times = self.posRec.times() * mc.NANO2SEC
        r_CN_N = self.posRec.r_CN_N
        sigma_BN = self.attRec.sigma_BN

        color_x = simHelpers.getLineColor(0, 3)
        color_y = simHelpers.getLineColor(1, 3)
        color_z = simHelpers.getLineColor(2, 3)

        figureList = {}

        # Plot the spacecraft position
        fig1 = plt.figure()
        plt.plot(times/60, r_CN_N[:, 0], color=color_x, label='x')
        plt.plot(times/60, r_CN_N[:, 1], color=color_y, label='y')
        plt.plot(times/60, r_CN_N[:, 2], color=color_z, label='z')
        plt.xlabel('Time [min]')
        plt.ylabel('Position Error [m]')
        plt.legend()
        figureList[fileNameForImages + "_position"] = fig1

        # Plot the spacecraft attitude
        fig2 = plt.figure()
        plt.plot(times/60, sigma_BN[:, 0], color=color_x, label=r'$\sigma_1$')
        plt.plot(times/60, sigma_BN[:, 1], color=color_y, label=r'$\sigma_2$')
        plt.plot(times/60, sigma_BN[:, 2], color=color_z, label=r'$\sigma_3$')
        plt.xlabel('Time [min]')
        plt.ylabel(r'Attitude Error [$\boldsymbol{\sigma}$]')
        plt.legend()
        figureList[fileNameForImages + "_attitude"] = fig2

        if savePlots:
            supportPath = os.path.join(path, "thrArmControlSupport")
            fig1.savefig(os.path.join(
                supportPath, "regulationControl_position.svg"
            ))
            fig2.savefig(os.path.join(
                supportPath, "regulationControl_attitude.svg"
            ))

        return figureList

def runScenario(scenario, runTime: float = 320.0, showProgressBar: bool = True):
    if vizSupport.vizFound:
        DynModel = scenario.get_DynModel()
        vizSupport.enableUnityVisualization(
            scenario,
            DynModel.taskName,
            DynModel.scene,
            # saveFile=__file__,
        )

    # initialize the scenario
    scenario.InitializeSimulation()

    # set the initial conditions for MuJoCo
    scenario.configure_initial_conditions()

    # configure the run time and execute the simulation
    simulationTime = mc.sec2nano(runTime)
    scenario.ConfigureStopTime(simulationTime)
    scenario.SetProgressBar(showProgressBar)
    scenario.ExecuteSimulation()

def run(showPlots: bool = False, runTime: float = 500.0,
        showProgressBar: bool = True, savePlots: bool = False):
    """
    The scenario can be run with the following setup parameters:

    Args:
        showPlots (bool): Determines if the script displays plots.
        runTime (float): Simulation duration in seconds.
        showProgressBar (bool): Determines if Basilisk displays execution progress.
        savePlots (bool): Determines if the position and attitude figures are
            saved as SVG files.

    """
    scenario = scenarioThrArmControl()
    runScenario(scenario, runTime, showProgressBar)
    figureList = scenario.pull_outputs(showPlots, savePlots)

    if showPlots:
        plt.show()

    return figureList


if __name__ == "__main__":
        parser = argparse.ArgumentParser(description="Run scenarioThrArmControl")
        parser.add_argument(
        "--showPlots",
        action="store_true",
        help="Display matplotlib figures."
        )
        parser.add_argument(
        "--savePlots",
        action="store_true",
        default=False,
        help="Save the position and attitude figures as SVG files."
        )
        parser.add_argument(
        "--runTime",
        type=float,
        default=500.0,
        help="Simulation run time [s]."
        )
        parser.add_argument(
        "--noProgressBar",
        action="store_false",
        dest="showProgressBar",
        default=True,
        help="Disable the Basilisk simulation progress bar."
        )
        args = parser.parse_args()
        run(showPlots=args.showPlots, savePlots=args.savePlots,
            runTime=args.runTime, showProgressBar=args.showProgressBar)
