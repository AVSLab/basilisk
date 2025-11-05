#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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
#

import numpy as np
import pytest

from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.architecture import messaging
try:
    from Basilisk.simulation import MJJointReactionForces, mujoco

    couldImportMujoco = True
except:
    couldImportMujoco = False

# Constants used throughout tests
MASS1 = 1.0
MASS2 = 2.0
LINK1_LENGTH = 0.5
LINK2_COM_OFFSET = 0.3
DAMPING = 2.5
OFFSET = 0.15

xmlFixedBase2r = f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="base">  <!-- FIXED base: NO <freejoint/> -->
      <geom type="box" size="0.2 0.2 0.2" mass="5" contype="0" conaffinity="0"/>

      <body name="link1" pos="0 0 0">
        <!-- Joint 1 about +Z -->
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <!-- Give link1 some mass; keep COM near joint to reduce extra coupling -->
        <geom type="sphere" pos="0.05 0 0" size="0.01" mass="{MASS1}" contype="0" conaffinity="0"/>
        <!-- Visual-only capsule to show the link -->
        <geom type="capsule" fromto="0 0 0  {LINK1_LENGTH} 0 0" size="0.03" mass="0.0" contype="0" conaffinity="0"/>

        <body name="link2" pos="{LINK1_LENGTH} 0 0">
          <!-- Joint 2 about +Z -->
          <joint name="j2" type="hinge" axis="0 0 1"/>
          <!-- Put all link2 mass at an offset COM (r2,0,0) from joint 2 -->
          <geom type="sphere" pos="{LINK2_COM_OFFSET} 0 0" size="0.01" mass="{MASS2}" contype="0" conaffinity="0"/>
          <!-- Visual-only stub -->
          <geom type="capsule" fromto="0 0 0  0.2 0 0" size="0.02" mass="0.0" contype="0" conaffinity="0"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>"""

xmlFreeBaseDampedHinge = f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <geom type="box" size="0.5 0.3 0.2" mass="50"/>
      <body name="link1" pos="0 0 0">
        <!-- Hinge about +Z; COM at joint; principal axis -->
        <joint name="j1" type="hinge" axis="0 0 1" damping="{DAMPING}"/>
        <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
        <geom type="capsule" fromto="0 0 0  0.3 0 0" size="0.05" mass="0.0"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

xmlDoubleFreeBaseDampedHinge = f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>

    <!-- Spacecraft 1 -->
    <body name="hub1" pos="0 0 0">
      <freejoint/>
      <!-- Visual hub box; collisions off -->
      <geom type="box" size="0.5 0.3 0.2" mass="50" contype="0" conaffinity="0"/>
      <body name="link1" pos="0 0 0">
        <joint name="j1" type="hinge" axis="0 0 1" damping="{DAMPING}"/>
        <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
        <!-- Visual link; collisions off -->
        <geom type="capsule" fromto="0 0 0  0.3 0 0" size="0.05" mass="0.0" contype="0" conaffinity="0"/>
      </body>
    </body>

    <!-- Spacecraft 2 (moved away to avoid overlap) -->
    <body name="hub2" pos="2 0 0">
      <freejoint/>
      <!-- Visual hub box; collisions off -->
      <geom type="box" size="0.5 0.3 0.2" mass="50" contype="0" conaffinity="0"/>
      <body name="link2" pos="0 0 0">
        <joint name="j2" type="hinge" axis="0 0 1" damping="{DAMPING}"/>
        <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
        <!-- Visual link; collisions off -->
        <geom type="capsule" fromto="0 0 0  0.3 0 0" size="0.05" mass="0.0" contype="0" conaffinity="0"/>
      </body>
    </body>

  </worldbody>
</mujoco>"""

xmlThruster =  f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <geom type="box" size="0.5 0.3 0.2" mass="50"/>
      <site name="thruster1" pos="0 {OFFSET} 0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="F_thruster1" site="thruster1" gear="-1 0 0 0 0 0"/>
  </actuator>
</mujoco>"""

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("modelName, xml, jointTreeIdx, jointParentBodyIdx, jointTypes, jointDOFStart", [
    ("Fixed Base", xmlFixedBase2r, [0, 0], [2,3], [3,3], [0,1]),
    ("One SC", xmlFreeBaseDampedHinge, [0, 0], [1,2], [0,3], [0,6]),
    ("Two SC", xmlDoubleFreeBaseDampedHinge, [0, 0, 1, 1], [1,2,3,4], [0,3,0,3], [0,6,7,13]),
    ("Thruster", xmlThruster, [0], [1], [0], [0])
])


def test_MJJointReactionForces(modelName, xml, jointTreeIdx, jointParentBodyIdx, jointTypes, jointDOFStart):
    r"""
    **Validation Test Description**

    This test sets up a MuJoCo simulation to extract the reaction forces and torques acting on the joints

    **Test Parameters**

    The system geometry is varied between tests.

    Args:
        modelName (str): the model of the spacecraft for this parameterized unit test
        xml (str): the XML name for the spacecraft model
        jointTreeIdx (list): the expected kinematic tree indices for each joint
        jointParentBodyIdx (list): the expected parent body indices for each joint
        jointTypes (list): the expected joint types in the model
        jointDOFStart (list): the expected starting degrees of freedom for each joint

    **Description of Variables Being Tested**

    In this test we are checking the system properties and the reaction force and torque values to ensure they are accurate
    """
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.01)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # create the Mujoco scene
    scene = mujoco.MJScene(xml)
    scene.extraEoMCall = True
    unitTestSim.AddModelToTask(unitTaskName, scene)

    # setup module to be tested
    module = MJJointReactionForces.MJJointReactionForces()
    module.ModelTag = "MJJointReactionForcesTag"
    module.scene = scene
    unitTestSim.AddModelToTask(unitTaskName, module)

    if modelName == "Fixed Base":
        angle1 = 0.0
        angle2 = np.pi / 4
        rate1 = 1.0
        rate2 = -0.5
    elif modelName == "One SC" or modelName == "Two SC":
        jointRate = 1.2
    elif modelName == "Thruster":
        thrusterForce = 1.0
        thrForceMsg = messaging.SingleActuatorMsg()
        thrForceMsg.write(messaging.SingleActuatorMsgPayload(input=thrusterForce))
        scene.getSingleActuator("F_thruster1").actuatorInMsg.subscribeTo(thrForceMsg)

    # setup output message recorder objects
    reactionForcesMsgRec = module.reactionForcesOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, reactionForcesMsgRec)

    unitTestSim.InitializeSimulation()

    if modelName == "Fixed Base":
        # set initial condition on the joint
        scene.getBody("link1").getScalarJoint("j1").setPosition(angle1)
        scene.getBody("link1").getScalarJoint("j1").setVelocity(rate1)
        scene.getBody("link2").getScalarJoint("j2").setPosition(angle2)
        scene.getBody("link2").getScalarJoint("j2").setVelocity(rate2)
    elif modelName == "One SC":
        # set initial condition on the joint
        scene.getBody("link1").getScalarJoint("j1").setVelocity(jointRate)
    elif modelName == "Two SC":
        # set initial condition on the joints
        scene.getBody("link1").getScalarJoint("j1").setVelocity(jointRate)
        scene.getBody("link2").getScalarJoint("j2").setVelocity(jointRate)

    unitTestSim.ConfigureStopTime(macros.sec2nano(0.01))
    unitTestSim.ExecuteSimulation()

    # pull module data
    treeIdx    = reactionForcesMsgRec.jointTreeIdx[0,:]
    parentBodyIdx = reactionForcesMsgRec.jointParentBodyIdx[0,:]
    jointTypesModule = reactionForcesMsgRec.jointTypes[0,:]
    dofStart   = reactionForcesMsgRec.jointDOFStart[0,:]
    biasForces = reactionForcesMsgRec.biasForces[0,:]
    passiveForces = reactionForcesMsgRec.passiveForces[0,:]
    constraintForces = reactionForcesMsgRec.constraintForces[0,:]
    appliedForces = reactionForcesMsgRec.appliedForces[0,:]
    actuatorForces = reactionForcesMsgRec.actuatorForces[0,:]

    # Build the expected reaction forces and torques
    if modelName == "Fixed Base":
        passiveTruth, constraintTruth, appliedTruth, actuatorTruth = np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2)
        h =  MASS2 * LINK1_LENGTH * LINK2_COM_OFFSET * np.sin(angle2)
        tau1 = -h * (2.0 * rate1 * rate2 + rate2 ** 2)
        tau2 = h * (rate1 ** 2)
        biasTruth = np.array([tau1, tau2])
    elif modelName == "One SC":
        biasTruth, passiveTruth, constraintTruth, appliedTruth, actuatorTruth = np.zeros(7), np.zeros(7), np.zeros(7), np.zeros(7), np.zeros(7)
        passiveTruth[6] = -DAMPING * jointRate
    elif modelName == "Two SC":
        biasTruth, passiveTruth, constraintTruth, appliedTruth, actuatorTruth = np.zeros(14), np.zeros(14), np.zeros(14), np.zeros(14), np.zeros(14)
        passiveTruth[6] = -DAMPING * jointRate
        passiveTruth[13] = -DAMPING * jointRate
    elif modelName == "Thruster":
        biasTruth, passiveTruth, constraintTruth, appliedTruth = np.zeros(6), np.zeros(6), np.zeros(6), np.zeros(6)
        actuatorTruth = np.array([-thrusterForce, 0, 0, 0 , 0, OFFSET * thrusterForce])

    # Assert the jointTreeIdx array is correct
    np.testing.assert_array_equal(treeIdx, np.array(jointTreeIdx),
                              err_msg=f"{modelName} joint tree indices mismatch")

    # Assert the parent body indices are correct
    np.testing.assert_array_equal(parentBodyIdx, np.array(jointParentBodyIdx),
                              err_msg=f"{modelName} parent body indices mismatch")

    # Assert the joint types are correct
    np.testing.assert_array_equal(jointTypesModule, np.array(jointTypes),
                              err_msg=f"{modelName} joint types mismatch")

    # Assert the joint DOF start indices are correct
    np.testing.assert_array_equal(dofStart, np.array(jointDOFStart),
                              err_msg=f"{modelName} joint DOF start indices mismatch")

    # Assert the reaction forces are correct
    np.testing.assert_allclose(biasForces, biasTruth, rtol=1e-8,
                           err_msg=f"{modelName} bias reaction forces {biasForces} not close to expected {biasTruth}")
    np.testing.assert_allclose(passiveForces, passiveTruth, rtol=1e-8,
                           err_msg=f"{modelName} passive reaction forces {passiveForces} not close to expected {passiveTruth}")
    np.testing.assert_allclose(constraintForces, constraintTruth, rtol=1e-8,
                           err_msg=f"{modelName} constraint reaction forces {constraintForces} not close to expected {constraintTruth}")
    np.testing.assert_allclose(appliedForces, appliedTruth, rtol=1e-8,
                           err_msg=f"{modelName} applied reaction forces {appliedForces} not close to expected {appliedTruth}")
    np.testing.assert_allclose(actuatorForces, actuatorTruth, rtol=1e-8,
                           err_msg=f"{modelName} actuator reaction forces {actuatorForces} not close to expected {actuatorTruth}")


if __name__ == "__main__":
    test_MJJointReactionForces("Fixed Base", xmlFixedBase2r, [0, 0], [3,3], [0,1], [0,1])
