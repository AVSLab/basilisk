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

This script demonstrates how to simulate the control of a 6-DOF spacecraft with
two attached 4-DOF thruster arms using the MuJoCo physics engine within the BSK_Sim architecture.

The multi-body systems is created by importing the MuJoCo XML file ``sat_w_thruster_arms.xml``
which defines the spacecraft hub and two attached 4-DOF thruster arms. It consists of a central hub
with two arms each consisting of 4 hinged joints, 2 rigid arm segments, and a thruster at the end.
MuJoCo single-input actuators are used for each of the spacecraft's thrusters hinged joints, and to apply
an additional torque capability direct to the spacecraft hub.

The script is found in the folder ``basilisk/examples/mujoco`` and executed by using::

      python3 scenarioThrArmControl.py

Two simulation processes are created: one
which contains dynamics modules, and one that contains the Flight Software (FSW)
modules. The Dynamics process contains a single task with the modules:

#. ``MJScene``: the MuJoCo dynamics module that simulates the multi-body system defined in the XML file
#. ``MJSystemCoM``: extracts the center of mass position and velocity of the multi-body system from the MuJoCo scene
#. ``MJSystemMassMatrix``: extracts the mass matrix of the multi-body system from the MuJoCo scene
#. ``MJJointReactionForces``: extracts the internal reaction forces and torques of the multi-body system from the MuJoCo scene

The FSW process contains three tasks: the outer loop control task, the joint motion task, and the thruster firing task.
The modules for the outer loop control task include:

#. ``inertial3D``: generates and inertial reference attitude
#. ``attError``: computes the attitude error between the current and reference attitude
#. ``mrpFeedback``: computes the attitude control torque
#. ``inertialCartFeedback``: computes the positional control force
#. ``jointThrAllocation``: determines the joint angles and thruster forces needed to achieve the desired control torque and force
#. ``thrFiringRound``: determines thruster firing time based on the thruster forces and a specified minimum impulse bit

The modules for the joint motion task include:

#. ``hingedJointArrayMotor``: determines the motor commands for the hinged joints of the thruster arms
#. ``jointMotionCompensator``: determines the compensation torques to apply at the spacecraft hub to counteract the reaction torques from moving the thruster arms

The modules for the thruster firing task include:

#. ``thrOnTimeToForce``: converts on time to thruster force
#. ``thrJointCompensation``: determines the motor commands to hold hinged joints static during thruster firing

The different tasks are enabled and disabled using three events: joint motion, thruster firing, and coasting. The
condition to trigger each event and action it performs are as follows:

- Joint Motion Event:

    - Condition: New desired joint angles are received from the outer control loop
    - Action: enables the joint motion task and move the arms to their desired positions

- Thruster Firing Event:

    - Condition: New desired thruster forces are received from the outer control loop and
      the joints have reached their desired positions
    - Action: enables the thruster firing task and fire the thrusters to achieve the desired forces

- Coasting Event:

    - Condition: The thrusters have finished firing and there is time before the next outer loop update
    - Action: set commands for all actuators to zero and allow the spacecraft to coast until the next control update


Illustration of Simulation Results
----------------------------------

::

    showPlots = True

.. image:: /_images/Scenarios/scenarioThrArmControl_position.svg
   :align: center

.. image:: /_images/Scenarios/scenarioThrArmControl_attitude.svg
   :align: center


"""
# Get current file path
import inspect
import os
import sys
import argparse
import matplotlib.pyplot as plt

from Basilisk.utilities import macros as mc, unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
fileNameForImages = os.path.basename(os.path.splitext(__file__)[0])

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/mujocoModels')
import BSK_MujocoDynamics
import BSK_MujocoFSW
from BSK_mujocoMasters import BSKSim, BSKScenario

# Create scenario child class
class scenarioThrArmControl(BSKSim, BSKScenario):
    def __init__(self, fswRate=0.01, dynRate=0.01):
        super(scenarioThrArmControl, self).__init__(fswRate=fswRate, dynRate=dynRate)
        self.name = "scenarioThrArmControl"

        # Declare empty class variables
        self.posRec = None
        self.attRec = None

        self.set_DynModel(BSK_MujocoDynamics)
        self.set_FswModel(BSK_MujocoFSW)
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

    def pull_outputs(self, showPlots: bool = False):
        times = self.posRec.times() * mc.NANO2SEC
        r_CN_N = self.posRec.r_CN_N
        sigma_BN = self.attRec.sigma_BN

        color_x = unitTestSupport.getLineColor(0, 3)
        color_y = unitTestSupport.getLineColor(1, 3)
        color_z = unitTestSupport.getLineColor(2, 3)

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

        return figureList

def runScenario(scenario, runTime: float = 320.0):
    # initialize the scenario
    scenario.InitializeSimulation()

    # set the initial conditions for MuJoCo
    scenario.configure_initial_conditions()

    # configure the run time and execute the simulation
    simulationTime = mc.sec2nano(runTime)
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()

def run(showPlots: bool = False, timeStep: float = 0.01, runTime: float = 320.0):
   """
    The scenarios can be run with the following setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots

   """
   scenario = scenarioThrArmControl(fswRate=timeStep, dynRate=timeStep)
   runScenario(scenario, runTime)
   figureList = scenario.pull_outputs(showPlots)

   if showPlots:
        plt.show()

   return figureList


if __name__ == "__main__":
        parser = argparse.ArgumentParser(description="Run scenarioThrArmControl")
        parser.add_argument(
        "--showPlots",
        action="store_true",
        default=True,
        help="Display matplotlib figures."
        )
        parser.add_argument(
        "--timeStep",
        type=float,
        default=0.01,
        help="FSW and dynamics update period [s]."
        )
        parser.add_argument(
        "--runTime",
        type=float,
        default=320.0,
        help="Simulation run time [s]."
        )

        args = parser.parse_args()
        run(showPlots=args.showPlots, timeStep=args.timeStep, runTime=args.runTime)
