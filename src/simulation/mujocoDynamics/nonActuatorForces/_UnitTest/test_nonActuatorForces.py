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

import os
import numpy as np
from numpy.testing import assert_allclose
import pytest
import tempfile

from Basilisk.utilities import SimulationBaseClass, macros

try:
    from Basilisk.simulation import mujoco, nonActuatorForces

    couldImportMujoco = True
except:
    couldImportMujoco = False

def _tmp_xml(xml_text: str, base: str) -> str:
    td = tempfile.mkdtemp(prefix="mjnact_")
    p = os.path.join(td, f"{base}.xml")
    with open(p, "w") as f:
        f.write(xml_text)
    return p

def xml_fixedbase_planar_2r(L1=0.5, r2=0.3, m1=1.0, m2=2.0):
    """
    Fixed base, planar 2R arm (hinges about +Z), no gravity.
    Link1 length L1 (visual), Link2 COM offset r2 (real mass via tiny sphere).
    """
    return f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="base">  <!-- FIXED base: NO <freejoint/> -->
      <geom type="box" size="0.2 0.2 0.2" mass="5" contype="0" conaffinity="0"/>

      <body name="link1" pos="0 0 0">
        <!-- Joint 1 about +Z -->
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <!-- Give link1 some mass; keep COM near joint to reduce extra coupling -->
        <geom type="sphere" pos="0.05 0 0" size="0.01" mass="{m1}" contype="0" conaffinity="0"/>
        <!-- Visual-only capsule to show the link -->
        <geom type="capsule" fromto="0 0 0  {L1} 0 0" size="0.03" mass="0.0" contype="0" conaffinity="0"/>

        <body name="link2" pos="{L1} 0 0">
          <!-- Joint 2 about +Z -->
          <joint name="j2" type="hinge" axis="0 0 1"/>
          <!-- Put all link2 mass at an offset COM (r2,0,0) from joint 2 -->
          <geom type="sphere" pos="{r2} 0 0" size="0.01" mass="{m2}" contype="0" conaffinity="0"/>
          <!-- Visual-only stub -->
          <geom type="capsule" fromto="0 0 0  0.2 0 0" size="0.02" mass="0.0" contype="0" conaffinity="0"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>"""

def xml_freebase_onehinge_damped(damping=2.5):
    """Free base, one hinge at COM, principal axis; gravity=0; passive damping only."""
    return f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <geom type="box" size="0.5 0.3 0.2" mass="50"/>
      <body name="link1" pos="0 0 0">
        <!-- Hinge about +Z; COM at joint; principal axis -->
        <joint name="j1" type="hinge" axis="0 0 1" damping="{damping}"/>
        <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
        <geom type="capsule" fromto="0 0 0  0.3 0 0" size="0.05" mass="0.0"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

def _first_base(rec):
    F = np.array([rec.baseTransForces[0,0], rec.baseTransForces[0,1], rec.baseTransForces[0,2]])
    T = np.array([rec.baseRotForces[0,0],   rec.baseRotForces[0,1],   rec.baseRotForces[0,2]])
    return F, T

def _first_joint(rec, idx=0):
    return float(rec.jointForces[0, idx])

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("scenario", [
     "planar2r_bias", "damped_hinge"
])

def test_nonActuatorForces(scenario):
    """Module Unit Test"""
    [testResults, testMessage] = nonActuatorForcesTestFunction(scenario)
    assert testResults < 1, testMessage

def nonActuatorForcesTestFunction(scenario):
    r"""
    **Validation Test Description**

    This unit test simulates a mujoco spacecraft to determine the non-actuator forces acting on the system.

    **Test Parameters**

    The spacecraft geometry and associated forces are varied between tests.

    Args:
        scenario (str): the model and forces used for the test

    **Description of Variables Being Tested**

    In this test we are checking that the non-actuator forces extracted from the mujoco simulation match the expected values.
    """
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    if scenario == "planar2r_bias":
        angle1 = 0.0
        angle2 = np.pi / 4
        rate1 = 1.0
        rate2 = -0.5
        L1 = 0.5
        L2 = 0.3
        m1 = 1.0
        m2 = 2.0
        xml = xml_fixedbase_planar_2r(L1, L2, m1, m2)
        xml_path = _tmp_xml(xml, "planar2r")
        numJoints = 2
    elif scenario == "damped_hinge":
        damping = 2.5
        jointRate = 1.2
        xml = xml_freebase_onehinge_damped(damping=damping)
        xml_path = _tmp_xml(xml, "damped_hinge")
        numJoints = 1


    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.01)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # create the Mujoco scene
    scene = mujoco.MJScene.fromFile(xml_path)
    scene.extraEoMCall = True
    unitTestSim.AddModelToTask(unitTaskName, scene)

    # setup module to be tested
    module = nonActuatorForces.NonActuatorForces()
    module.ModelTag = "nonActuatorForcesTag"
    module.scene = scene
    unitTestSim.AddModelToTask(unitTaskName, module)

    # setup output message recorder objects
    forcesOutMsgRec = module.forcesOutMsg.recorder()
    forcesOutMsgRecC = module.forcesOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, forcesOutMsgRec)
    unitTestSim.AddModelToTask(unitTaskName, forcesOutMsgRecC)

    unitTestSim.InitializeSimulation()

    if scenario == "planar2r_bias":
        # set initial condition on the joint
        scene.getBody("link1").getScalarJoint("j1").setPosition(angle1)
        scene.getBody("link1").getScalarJoint("j1").setVelocity(rate1)
        scene.getBody("link2").getScalarJoint("j2").setPosition(angle2)
        scene.getBody("link2").getScalarJoint("j2").setVelocity(rate2)
    elif scenario == "damped_hinge":
        # set initial condition on the joint
        scene.getBody("link1").getScalarJoint("j1").setVelocity(jointRate)

    unitTestSim.ConfigureStopTime(macros.sec2nano(0.01))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    F_mod, T_mod = _first_base(forcesOutMsgRec)
    tau_mod = np.zeros(numJoints)
    for i in range(numJoints):
        tau_mod[i] = _first_joint(forcesOutMsgRec, i)

    f_modC, t_modC = _first_base(forcesOutMsgRecC)
    tau_modC = np.zeros(numJoints)
    for i in range(numJoints):
        tau_modC[i] = _first_joint(forcesOutMsgRecC, i)

    # Calculate truth data and compare to module data
    F_truth = np.zeros(3)
    T_truth = np.zeros(3)
    if scenario == "planar2r_bias":
        h = -1 * m2 * L1 * L2 * np.sin(angle2)
        tau1 = -h * (2.0 * rate1 * rate2 + rate2 ** 2)
        tau2 = h * (rate1 ** 2)
        tau_truth = np.array([tau1, tau2])
        try:
            assert_allclose(F_mod, F_truth, atol=1e-8)
        except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Base Force (planar2r_bias) not ~0: {err}\n")
        try:
            assert_allclose(T_mod, T_truth, atol=1e-8)
        except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Base Torque (planar2r_bias) not ~0: {err}\n")
        try:
            assert_allclose(tau_mod, tau_truth, atol=1e-8)
        except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Joint Torque (planar2r_bias): {err}\n")
    elif scenario == "damped_hinge":
        tau_truth = np.array([-damping * jointRate])
        try:
            assert_allclose(F_mod, F_truth, atol=1e-8)
        except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Base Force (damped_hinge) not ~0: {err}\n")
        try:
            assert_allclose(T_mod, T_truth, atol=1e-8)
        except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Base Torque (damped_hinge) not ~0: {err}\n")
        try:
            print(tau_mod, tau_truth)
            assert_allclose(tau_mod, tau_truth, atol=1e-8)
        except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Joint Torque (damped_hinge): {err}\n")

    try:
        assert_allclose(F_mod, f_modC, atol=1e-14)
    except AssertionError as err:
        testFailCount += 1
        testMessages.append(f"Base Force C/C++ msg not equal: {err}\n")
    try:
        assert_allclose(T_mod, t_modC, atol=1e-14)
    except AssertionError as err:
        testFailCount += 1
        testMessages.append(f"Base Torque C/C++ msg not equal: {err}\n")
    try:
        assert_allclose(tau_mod, tau_modC, atol=1e-14)
    except AssertionError as err:
        testFailCount += 1
        testMessages.append(f"Joint Torque C/C++ msg not equal: {err}\n")

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_nonActuatorForces("planar2r_bias")
