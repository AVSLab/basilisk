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
try:
    from Basilisk.simulation import MJSystemMassMatrix, mujoco

    couldImportMujoco = True
except:
    couldImportMujoco = False

# Constants used throughout tests
HUB_MASS  = 50.0
HUB_I_COM = np.diag([10.0, 11.0, 12.0])   # hub inertia @ COM
ARM_MASS  = 10.0
ROD_I_COM = np.diag([ 5.0,  6.0,  7.0])   # child inertia @ COM
SX, SY, SZ = 0.5, 0.3, 0.2

xmlSingle =  f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="{HUB_I_COM[0,0]} {HUB_I_COM[1,1]} {HUB_I_COM[2,2]}"/>
      <geom type="box" size="{SX} {SY} {SZ}"/>
    </body>
  </worldbody>
</mujoco>"""

xmlOneHinge = f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="{HUB_I_COM[0,0]} {HUB_I_COM[1,1]} {HUB_I_COM[2,2]}"/>
      <geom type="box" size="{SX} {SY} {SZ}"/>
      <body name="link_h" pos="0.6 0 0">
        <joint name="j_h" type="hinge" axis="0 0 1"/>
        <inertial pos="0 0 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="{ROD_I_COM[0,0]} {ROD_I_COM[1,1]} {ROD_I_COM[2,2]}"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

xmlOneSlider = f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="{HUB_I_COM[0,0]} {HUB_I_COM[1,1]} {HUB_I_COM[2,2]}"/>
      <geom type="box" size="{SX} {SY} {SZ}"/>
      <body name="link_s" pos="-0.6 0 0">
        <joint name="j_s" type="slide" axis="1 0 0"/>
        <inertial pos="0 0 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="{ROD_I_COM[0,0]} {ROD_I_COM[1,1]} {ROD_I_COM[2,2]}"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

xmlDouble =  f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub1">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="{HUB_I_COM[0,0]} {HUB_I_COM[1,1]} {HUB_I_COM[2,2]}"/>
      <geom type="box" size="{SX} {SY} {SZ}"/>
    </body>
    <body name="hub2">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="{HUB_I_COM[0,0]} {HUB_I_COM[1,1]} {HUB_I_COM[2,2]}"/>
      <geom type="box" size="{SX} {SY} {SZ}"/>
    </body>
  </worldbody>
</mujoco>"""

xmlTranslation = f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <!-- 3-DOF translational base (no rotation) -->
      <joint name="tx" type="slide" axis="1 0 0" limited="false"/>
      <joint name="ty" type="slide" axis="0 1 0" limited="false"/>
      <joint name="tz" type="slide" axis="0 0 1" limited="false"/>

      <inertial pos="0 0 0" mass="{HUB_MASS}" diaginertia="{HUB_I_COM[0,0]} {HUB_I_COM[1,1]} {HUB_I_COM[2,2]}"/>
      <geom type="box" size="{SX} {SY} {SZ}"/>
    </body>
  </worldbody>
</mujoco>"""

def hingedMassMatrix(hubMass, hubInertia, armMass, armInertia, armLength=0.6):
    """
    Helper function to compute the expected mass matrix for a hub with a hinged arm when the hinge angle is zero.
    """
    M = np.zeros((7,7), float)

    # Hub translational mass
    M[0:3,0:3] = hubMass * np.eye(3)

    # Hub rotational inertia
    M[3:6,3:6] = hubInertia

    # Arm contribution
    M[0:3,0:3] += armMass * np.eye(3)
    M[3:6,3:6] += armInertia
    M[4,4] += armMass * armLength**2
    M[5,5] += armMass * armLength**2
    M[2,4] = M[4,2] = -armMass * armLength
    M[1,5] = M[5,1] = armMass * armLength
    M[6,6] =M[6,5] = M[5,6] = armInertia[2,2]


    return M

def sliderMassMatrix(hubMass, hubInertia, armMass, armInertia, armLength=0.6):
    """
    Helper function to compute the expected mass matrix for a hub with a sliding arm when the slide position is zero.
    """
    M = np.zeros((7,7), float)

    # Hub translational mass
    M[0:3,0:3] = hubMass * np.eye(3)

    # Hub rotational inertia
    M[3:6,3:6] = hubInertia

    # Arm contribution
    M[0:3,0:3] += armMass * np.eye(3)
    M[3:6,3:6] += armInertia
    M[4,4] += armMass * armLength**2
    M[5,5] += armMass * armLength**2
    M[2,4] = M[4,2] = armMass * armLength
    M[1,5] = M[5,1] = -armMass * armLength
    M[6,6] = M[0,6] = M[6,0] = armMass


    return M


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("modelName, xml, nSC, scStartIdx, jointTypes", [
    ("One SC",              xmlSingle,              1,  [0], [0]),
    ("One Hinge",           xmlOneHinge,            1,  [0], [0,3]),
    ("One Slider",          xmlOneSlider,           1,  [0], [0,2]),
    ("Two SC",              xmlDouble,              2,  [0,1], [0,0]),
    ("Translation SC Only", xmlTranslation,         1,  [0], [2,2,2])
])

def test_MJSystemMassMatrix(modelName, xml, nSC, scStartIdx, jointTypes):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft to determine its system mass matrix.

    **Test Parameters**

    The spacecraft geometry is varied between tests

    Args:
        modelName (str): the model of the spacecraft for this parameterized unit test
        xml (str): the XML name for the spacecraft model
        nSC (int): the expected number of spacecraft in the model
        scStartIdx (list): the expected starting joint index for each spacecraft
        jointTypes (list): the expected joint types in the model

    **Description of Variables Being Tested**

    In this test we are checking the system mass matrix values and properties to ensure it is accurate
    """
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # create the Mujoco scene
    scene = mujoco.MJScene(xml)
    scene.extraEoMCall = True
    unitTestSim.AddModelToTask(unitTaskName, scene)

    # setup module to be tested
    module = MJSystemMassMatrix.MJSystemMassMatrix()
    module.ModelTag = "MJSystemMassMatrixTag"
    module.scene = scene
    unitTestSim.AddModelToTask(unitTaskName, module)

    # setup output message recorder objects
    massMatrixOutMsgRec = module.massMatrixOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, massMatrixOutMsgRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.1))
    unitTestSim.ExecuteSimulation()

    # Read from recorder
    nSCOutMsgData    = massMatrixOutMsgRec.nSC[0]
    scStartIdxOutMsgData = massMatrixOutMsgRec.scStartIdx[0,:]
    jointTypesOutMsgData = massMatrixOutMsgRec.jointTypes[0,:]
    massMatrixOutMsgData = massMatrixOutMsgRec.massMatrix[0,:]
    nDOF = int(np.sqrt(len(massMatrixOutMsgData)))
    massMatrixOutMsgData = massMatrixOutMsgData.reshape((nDOF, nDOF), order='C')

    print("massMatrixOutMsgData = ", massMatrixOutMsgData)

    # Build the expected mass matrix
    if modelName == "One SC":
        massMatrixTruth = np.zeros((6,6), float)
        massMatrixTruth[:3,:3] = HUB_MASS * np.eye(3)
        massMatrixTruth[3:,3:] = HUB_I_COM
    elif modelName == "One Hinge":
        massMatrixTruth = hingedMassMatrix(HUB_MASS, HUB_I_COM, ARM_MASS, ROD_I_COM)
    elif modelName == "One Slider":
        massMatrixTruth = sliderMassMatrix(HUB_MASS, HUB_I_COM, ARM_MASS, ROD_I_COM)
    elif modelName == "Two SC":
        massMatrixTruth = np.zeros((12,12), float)
        massMatrixTruth[:3,:3] = HUB_MASS * np.eye(3)
        massMatrixTruth[3:6,3:6] = HUB_I_COM
        massMatrixTruth[6:9,6:9] = HUB_MASS * np.eye(3)
        massMatrixTruth[9:,9:] = HUB_I_COM
    elif modelName == "Translation SC Only":
        massMatrixTruth = HUB_MASS * np.eye(3)

    # Assert the number of spacecraft is correct
    assert nSCOutMsgData == nSC, f"{modelName} number of spacecraft {nSCOutMsgData} does not match expected {nSC}"

    # Assert the starting joint indices are correct
    np.testing.assert_array_equal(scStartIdxOutMsgData, np.array(scStartIdx),
                              err_msg=f"{modelName} starting joint indices mismatch")

    # Assert the joint types are correct
    np.testing.assert_array_equal(jointTypesOutMsgData, np.array(jointTypes),
                              err_msg=f"{modelName} joint types mismatch")

    # Assert the mass matrix is correct
    assert massMatrixOutMsgData == pytest.approx(massMatrixTruth, rel=1e-8), \
        f"{modelName} mass matrix {massMatrixOutMsgData} not close to expected {massMatrixTruth}"


if __name__ == "__main__":
    # Run one case by hand
    test_MJSystemMassMatrix( "One Hinge", xmlOneHinge, 1, [0], [0,3])
