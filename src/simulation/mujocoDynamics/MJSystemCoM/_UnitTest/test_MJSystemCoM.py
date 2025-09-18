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
import tempfile
import pytest

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJSystemCoM

    couldImportMujoco = True
except:
    couldImportMujoco = False

# Common parameters used in both XML and truth calc
HUB_MASS = 50.0
ARM_MASS = 10.0
SX, SY, SZ = 0.5, 0.3, 0.2   # hub half-sizes (m) → face centers at ±SX
L = 2.0                      # arm length (m)
THETA_DEG = 30.0             # arm yaw angle for "angled" case

def _xml_single():
    return f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="1 1 1"/>
      <geom type="box" size="{SX} {SY} {SZ}" rgba="0.8 0.2 0.2 0.4"/>
    </body>
  </worldbody>
</mujoco>
"""

def _xml_straight():
    # Arms go straight ±X from the face centers.
    # Place each arm body at the face center; set its inertial COM at L/2 along arm direction.
    # Welded to hub (no joint).
    return f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="10 10 10"/>
      <geom type="box" size="{SX} {SY} {SZ}" rgba="0.8 0.2 0.2 0.4"/>

      <!-- +X arm -->
      <body name="arm_plus" pos="{SX} 0 0">
        <!-- arm COM is L/2 out along +X -->
        <inertial pos="{L/2} 0 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="5 5 5"/>
        <geom type="capsule" fromto="0 0 0 {L} 0 0" size="0.05" rgba="0.2 0.6 1 1"/>
      </body>

      <!-- -X arm -->
      <body name="arm_minus" pos="-{SX} 0 0">
        <!-- arm COM is L/2 out along -X -->
        <inertial pos="-{L/2} 0 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="5 5 5"/>
        <geom type="capsule" fromto="0 0 0 -{L} 0 0" size="0.05" rgba="0.2 0.6 1 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

def _xml_angled():
    # Arms leave ±X faces at yaw = ±θ outward (both tilt toward +Y).
    # Right arm direction u_plus = [cosθ, sinθ, 0].
    # Left arm direction u_minus = [-cosθ, sinθ, 0] (still outward from -X face).
    c = np.cos(np.deg2rad(THETA_DEG))
    s = np.sin(np.deg2rad(THETA_DEG))
    return f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="10 10 10"/>
      <geom type="box" size="{SX} {SY} {SZ}" rgba="0.8 0.2 0.2 0.4"/>

      <!-- +X angled arm -->
      <body name="arm_plus" pos="{SX} 0 0">
        <inertial pos="{(L/2)*c} {(L/2)*s} 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="5 5 5"/>
        <geom type="capsule" fromto="0 0 0 {L*c} {L*s} 0" size="0.05" rgba="0.2 1 0.4 1"/>
      </body>

      <!-- -X angled arm -->
      <body name="arm_minus" pos="-{SX} 0 0">
        <inertial pos="{- (L/2)*c} {(L/2)*s} 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="5 5 5"/>
        <geom type="capsule" fromto="0 0 0 {-L*c} {L*s} 0" size="0.05" rgba="1 0.8 0.2 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

def _write_temp_xml(xml_text: str, basename: str) -> str:
    tmpdir = tempfile.mkdtemp(prefix="mjcom_")
    path = os.path.join(tmpdir, f"{basename}.xml")
    with open(path, "w") as f:
        f.write(xml_text)
    return path

def _expected_com(model: str, r_root, v_root):
    r_root = np.asarray(r_root, dtype=float)
    v_root = np.asarray(v_root, dtype=float)

    if model == "single":
        # Only hub
        rC = r_root
        vC = v_root

    elif model == "straight":
        # Symmetric equal arms along ±X → CoM at hub center
        rC = r_root
        vC = v_root

    elif model == "angled":
        # Compute mass-weighted average of hub @ 0 and two arms at their COMs.
        c = np.cos(np.deg2rad(THETA_DEG))
        s = np.sin(np.deg2rad(THETA_DEG))

        # COM locations of the welded arm bodies in the hub frame:
        # right arm COM = face center + (L/2)[c, s, 0]
        r_plus = np.array([ SX, 0.0, 0.0]) + np.array([(L/2)*c, (L/2)*s, 0.0])
        # left arm COM  = (-SX,0,0) + (L/2)[-c, s, 0]
        r_minus= np.array([-SX, 0.0, 0.0]) + np.array([-(L/2)*c, (L/2)*s, 0.0])

        M  = HUB_MASS + 2*ARM_MASS
        r_off = (HUB_MASS*np.zeros(3) + ARM_MASS*r_plus + ARM_MASS*r_minus) / M

        rC = r_root + r_off
        vC = v_root

    else:
        raise ValueError("unknown model")

    return rC, vC

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("model", ["single", "straight", "angled"])
@pytest.mark.parametrize("moving", [True, False])
@pytest.mark.parametrize("displaced", [True, False])

def test_MJSystemCoM(show_plots, model, moving, displaced):
    """Module Unit Test"""
    [testResults, testMessage] = MJSystemCoMTest(show_plots, model, moving, displaced)
    assert testResults < 1, testMessage

def MJSystemCoMTest(show_plots, model, moving, displaced):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft to extract its center of mass.

    **Test Parameters**

    The spacecraft geometry, velocity, and base position are varied between tests

    Args:
        model (str): the model of the spacecraft for this parameterized unit test
        moving (bool): whether the spacecraft is moving or not for this parameterized unit test
        displaced (bool): whether the spacecraft base is displaced or not for this parameterized unit test

    **Description of Variables Being Tested**

    In this test we are checking the location and velocity of the spacecraft's center of mass
    compared to the expected values
    """
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    if model == "single":
        xml_path = _write_temp_xml(_xml_single(), "single")
        root_name = "hub"
    elif model == "straight":
        xml_path = _write_temp_xml(_xml_straight(), "straight")
        root_name = "hub"
    elif model == "angled":
        xml_path = _write_temp_xml(_xml_angled(), "angled")
        root_name = "hub"

    # Root initial pose
    r0 = np.array([0.0, 0.0, 0.0]) if not displaced else np.array([1234.5, -678.9, 42.0])  # m
    v0 = np.array([0.0, 0.0, 0.0]) if not moving else np.array([0.12, -0.07, 0.03])  # m/s

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # create the Mujoco scene
    scene = mujoco.MJScene.fromFile(xml_path)
    scene.extraEoMCall = True
    unitTestSim.AddModelToTask(unitTaskName, scene)

    # setup module to be tested
    module = MJSystemCoM.MJSystemCoM()
    module.ModelTag = "MJSystemCoMTag"
    module.scene = scene
    scene.AddModelToDynamicsTask(module)

    # setup output message recorder objects
    comStatesOutMsgRec = module.comStatesOutMsg.recorder()
    comStatesOutMsgCRec = module.comStatesOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, comStatesOutMsgRec)
    unitTestSim.AddModelToTask(unitTaskName, comStatesOutMsgCRec)

    unitTestSim.InitializeSimulation()

    # set initial conditions
    scene.getBody("hub").setPosition(r0)
    scene.getBody("hub").setVelocity(v0)

    unitTestSim.ConfigureStopTime(macros.sec2nano(0.2))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    r_CN_N_module = comStatesOutMsgRec.r_CN_N[-1,:]
    v_CN_N_module = comStatesOutMsgRec.v_CN_N[-1,:]
    r_CN_N_module_c = comStatesOutMsgCRec.r_CN_N[-1,:]
    v_CN_N_module_c = comStatesOutMsgCRec.v_CN_N[-1,:]

    # compute the truth data
    r_CN_N_truth0, v_CN_N_truth0 = _expected_com(model, r0, v0)
    r_CN_N_truth = r_CN_N_truth0 + v_CN_N_truth0 * 0.2
    v_CN_N_truth = v_CN_N_truth0

    # Compare
    acc = 1e-12
    testFailCount, testMessages = unitTestSupport.compareArrayND(
        [r_CN_N_truth], [r_CN_N_module], acc, "CoM_position", 3, testFailCount, testMessages
    )
    testFailCount, testMessages = unitTestSupport.compareArrayND(
        [v_CN_N_truth], [v_CN_N_module], acc, "CoM_velocity", 3, testFailCount, testMessages
    )
    testFailCount, testMessages = unitTestSupport.compareArrayND(
        [r_CN_N_module], [r_CN_N_module_c], acc, "cMsgPosition", 3, testFailCount, testMessages
    )
    testFailCount, testMessages = unitTestSupport.compareArrayND(
        [v_CN_N_module], [v_CN_N_module_c], acc, "cMsgVelocity", 3, testFailCount, testMessages
    )

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_MJSystemCoM(False, "angled", True, False)
