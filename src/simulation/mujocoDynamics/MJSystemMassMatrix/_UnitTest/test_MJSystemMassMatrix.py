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
from numpy.testing import assert_allclose, assert_equal, assert_array_less
import pytest
import tempfile

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

def _write_tmp(xml_text: str, base: str) -> str:
    td = tempfile.mkdtemp(prefix="mjmass_")
    p = os.path.join(td, f"{base}.xml")
    with open(p, "w") as f:
        f.write(xml_text)
    return p

def xml_single():
    return f"""<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="{HUB_I_COM[0,0]} {HUB_I_COM[1,1]} {HUB_I_COM[2,2]}"/>
      <geom type="box" size="{SX} {SY} {SZ}"/>
    </body>
  </worldbody>
</mujoco>"""

def xml_one_hinge():
    # Hinge about z at child origin; child COM at joint → M22 ≈ [Izz]
    return f"""<mujoco>
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

def xml_one_slider():
    # Slider along x at child origin; child COM at joint → M22 ≈ [m]
    return f"""<mujoco>
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

def xml_two_joints_separate():
    # Two independent branches: hinge child and slider child → M22 ≈ diag([Izz, m])
    return f"""<mujoco>
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
      <body name="link_s" pos="-0.6 0 0">
        <joint name="j_s" type="slide" axis="1 0 0"/>
        <inertial pos="0 0 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="{ROD_I_COM[0,0]} {ROD_I_COM[1,1]} {ROD_I_COM[2,2]}"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

def _skew(v):
    x, y, z = v
    return np.array([[0, -z,  y],
                     [z,  0, -x],
                     [-y, x,  0]], float)

def _composite_spatial_inertia(bodies):
    """
    bodies: [{'m':float,'r':(3,), 'Icom':(3,3)}, ...]
    Return (M6, rC).
    """
    m_tot = sum(b['m'] for b in bodies)
    rC    = sum(b['m']*np.asarray(b['r'], float) for b in bodies) / m_tot

    I_O = np.zeros((3,3))
    for b in bodies:
        r = np.asarray(b['r'], float)
        Icom = np.asarray(b['Icom'], float)
        I_O += Icom + b['m'] * ((r@r)*np.eye(3) - np.outer(r, r))

    M6 = np.zeros((6,6))
    M6[:3,:3] = m_tot*np.eye(3)
    M6[:3,3:] = -m_tot*_skew(rC)
    M6[3:,:3] =  m_tot*_skew(rC)
    M6[3:,3:] = I_O
    return M6, rC

def expected_base_block(bodies):
    M6, _ = _composite_spatial_inertia(bodies)
    return M6

def expected_joint_block(model_name):
    if model_name == "one_hinge":
        return np.array([[ROD_I_COM[2,2]]], float)    # hinge about z → Izz
    if model_name == "one_slider":
        return np.array([[ARM_MASS]], float)          # slider along x → mass
    if model_name == "two_joints":
        return np.diag([ROD_I_COM[2,2], ARM_MASS]).astype(float)

def add_allclose(testFailCount, testMessages, name, A, B, rtol=0.0, atol=1e-10):
    try:
        assert_allclose(A, B, rtol=rtol, atol=atol)
    except AssertionError as e:
        testFailCount += 1
        testMessages.append(f"{name}: {str(e)}\n")
    return testFailCount, testMessages

def add_equal(testFailCount, testMessages, name, a, b):
    try:
        assert_equal(a, b)
    except AssertionError as e:
        testFailCount += 1
        testMessages.append(f"{name}: {str(e)}\n")
    return testFailCount, testMessages

def add_symmetry(testFailCount, testMessages, name, M, tol=1e-12):
    return add_allclose(testFailCount, testMessages, name, M, M.T, rtol=0.0, atol=tol)

def add_psd(testFailCount, testMessages, name, M, tol=-1e-12):
    try:
        w = np.linalg.eigvalsh(0.5*(M+M.T))
        # require every eigenvalue >= tol (allow tiny negative for roundoff)
        assert_array_less(np.full_like(w, tol), w)
    except AssertionError as e:
        testFailCount += 1
        testMessages.append(f"{name}: not PSD; {str(e)}; eigvals={w}\n")
    return testFailCount, testMessages

def add_padding_zero(testFailCount, testMessages, name, M, used, atol=1e-14):
    try:
        if used < M.shape[0]:
            assert_allclose(M[used:, :], 0.0, rtol=0.0, atol=atol)
            assert_allclose(M[:, used:], 0.0, rtol=0.0, atol=atol)
    except AssertionError as e:
        testFailCount += 1
        testMessages.append(f"{name}: {str(e)}\n")
    return testFailCount, testMessages

def read_matrix_from_recorder(rec):
    """Infer DIM at runtime; return (Mmsg, used, nbase, nj)."""
    nbase = int(rec.nbase[-1])
    nj    = int(rec.nj[-1])
    used  = nbase + nj
    Mraw  = np.array(rec.MassMatrix[-1,:,:])
    if Mraw.ndim == 1:
        dim = int(round(np.sqrt(Mraw.size)))
        Mmsg = Mraw.reshape(dim, dim)
    else:
        dim = Mraw.shape[0]
        Mmsg = Mraw
    return Mmsg, used, nbase, nj

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("model_name,xml_fn, nj_expected", [
    ("single",      xml_single,              0),
    ("one_hinge",   xml_one_hinge,           1),
    ("one_slider",  xml_one_slider,          1),
    ("two_joints",  xml_two_joints_separate, 2),
])

def test_MJSystemMassMatrix(show_plots, model_name, xml_fn, nj_expected):
    """Module Unit Test"""
    [testResults, testMessage] = MJSystemMassMatrixTestFunction(show_plots, model_name, xml_fn, nj_expected)
    assert testResults < 1, testMessage


def MJSystemMassMatrixTestFunction(show_plots, model_name, xml_fn, nj_expected):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft to determine its system mass matrix.

    **Test Parameters**

    The spacecraft geometry is varied between tests

    Args:
        model_name (str): the model of the spacecraft for this parameterized unit test
        xml_fn (str): the XML file name for the spacecraft model
        nj_expected (int): the expected number of joints in the spacecraft model

    **Description of Variables Being Tested**

    In this test we are checking the system mass matrix values and properties to ensure it is accurate
    """
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    xml_path = _write_tmp(xml_fn(), model_name)

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # create the Mujoco scene
    scene = mujoco.MJScene.fromFile(xml_path)
    scene.extraEoMCall = True
    unitTestSim.AddModelToTask(unitTaskName, scene)

    # setup module to be tested
    module = MJSystemMassMatrix.MJSystemMassMatrix()
    module.ModelTag = "MJSystemMassMatrixTag"
    module.scene = scene
    unitTestSim.AddModelToTask(unitTaskName, module)

    # setup output message recorder objects
    massMatrixOutMsgRec = module.massMatrixOutMsg.recorder()
    massMatrixOutMsgCRec = module.massMatrixOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, massMatrixOutMsgRec)
    unitTestSim.AddModelToTask(unitTaskName, massMatrixOutMsgCRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.1))
    unitTestSim.ExecuteSimulation()

    # Read from recorder
    Mmsg, used, nbase, nj = read_matrix_from_recorder(massMatrixOutMsgRec)
    MmsgC, usedC, nbaseC, njC = read_matrix_from_recorder(massMatrixOutMsgCRec)

    # check sizing
    testFailCount, testMessages = add_equal(testFailCount, testMessages, "nbase", nbase, 6)
    testFailCount, testMessages = add_equal(testFailCount, testMessages, "nj",    nj,    nj_expected)

    # build truth dictionary
    bodies = [dict(m=HUB_MASS, r=np.zeros(3), Icom=HUB_I_COM)]
    if model_name == "one_hinge":
        bodies.append(dict(m=ARM_MASS, r=np.array([ 0.6, 0.0, 0.0]), Icom=ROD_I_COM))
    elif model_name == "one_slider":
        bodies.append(dict(m=ARM_MASS, r=np.array([-0.6, 0.0, 0.0]), Icom=ROD_I_COM))
    elif model_name == "two_joints":
        bodies.append(dict(m=ARM_MASS, r=np.array([ 0.6, 0.0, 0.0]), Icom=ROD_I_COM))
        bodies.append(dict(m=ARM_MASS, r=np.array([-0.6, 0.0, 0.0]), Icom=ROD_I_COM))

    M6_truth = expected_base_block(bodies)
    M6       = Mmsg[:6, :6]
    testFailCount, testMessages = add_allclose(testFailCount, testMessages, "Base 6x6", M6, M6_truth, atol=1e-10, rtol=0.0)

    # Joint block truth
    if nj_expected > 0:
        M22       = Mmsg[6:6+nj_expected, 6:6+nj_expected]
        M22_truth = expected_joint_block(model_name)
        testFailCount, testMessages = add_allclose(testFailCount, testMessages, "M22", M22, M22_truth, atol=1e-10, rtol=0.0)

    # Global properties on full populated block
    M_used = Mmsg[:used, :used]
    testFailCount, testMessages = add_symmetry(testFailCount, testMessages, "Symmetry(full used block)", M_used, tol=1e-12)
    testFailCount, testMessages = add_psd(testFailCount, testMessages,      "PSD(full used block)",      M_used, tol=-1e-12)

    # Zero padding outside populated block
    testFailCount, testMessages = add_padding_zero(testFailCount, testMessages, "Padding", Mmsg, used, atol=1e-14)

    # Check the two output messages agree
    testFailCount, testMessages = add_allclose(testFailCount, testMessages, "C vs C++ full matrix", Mmsg, MmsgC, rtol=0.0, atol=1e-14)
    testFailCount, testMessages = add_equal(testFailCount, testMessages, "C vs C++ nbase", nbase, nbaseC)
    testFailCount, testMessages = add_equal(testFailCount, testMessages, "C vs C++ nj",    nj,    njC)

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    # Run one case by hand
    MJSystemMassMatrixTestFunction(False, "single", xml_single, 0)
