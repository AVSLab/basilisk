"""
Tests for PayloadType.__dtype__ - numpy structured dtype mirroring the C struct layout.

Each test:
  1. Creates a payload object.
  2. Gets a writable numpy view of the underlying C memory via sim_model.getObjectAddress.
  3. Writes values through numpy and asserts the Python wrapper sees them (numpy → Python).
  4. Writes values through the Python wrapper and asserts numpy sees them (Python → numpy).

Coverage:
  - Primitive scalars with implicit padding   (EpochMsgPayload)
  - 1-D float arrays                          (AttStateMsgPayload)
  - 2-D float array                           (SCMassPropsMsgPayload)
  - Enum field                                (DeviceStatusMsgPayload)
  - Char array + 2-D array + scalar           (SpicePlanetStateMsgPayload)
  - Array of nested structs                   (AccDataMsgPayload)
  - Pointer field → uint64 in dtype           (CameraImageMsgPayload)
  - Incomplete struct (C++ vectors) → None    (JointArrayStateMsgPayload)
"""

import ctypes

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture import sim_model


def _writable_view(payload, dtype):
    """Return a writable 1-element structured numpy array backed by the payload's C memory."""
    addr = sim_model.getObjectAddress(payload)
    buf  = (ctypes.c_byte * dtype.itemsize).from_address(addr)
    return np.ndarray(1, dtype=dtype, buffer=buf)


# ---------------------------------------------------------------------------
# 1. Primitive scalars + implicit padding  (EpochMsgPayload)
#    struct: int year, month, day, hours, minutes; double seconds;
#    layout: 5×int32 (20 bytes) + 4-byte pad + float64 = 32 bytes total
# ---------------------------------------------------------------------------

def test_epoch_dtype_layout():
    dt = messaging.EpochMsgPayload.__dtype__
    assert dt is not None
    assert dt.itemsize == 32
    assert dt.fields['year'][1]    == 0
    assert dt.fields['month'][1]   == 4
    assert dt.fields['day'][1]     == 8
    assert dt.fields['hours'][1]   == 12
    assert dt.fields['minutes'][1] == 16
    assert dt.fields['seconds'][1] == 24   # 4-byte padding gap at bytes 20-23


def test_epoch_dtype_numpy_to_python():
    p   = messaging.EpochMsgPayload()
    arr = _writable_view(p, messaging.EpochMsgPayload.__dtype__)

    arr[0]['year']    = 2024
    arr[0]['month']   = 3
    arr[0]['day']     = 15
    arr[0]['hours']   = 12
    arr[0]['minutes'] = 30
    arr[0]['seconds'] = 45.5

    assert p.year    == 2024
    assert p.month   == 3
    assert p.day     == 15
    assert p.hours   == 12
    assert p.minutes == 30
    assert p.seconds == pytest.approx(45.5)


def test_epoch_dtype_python_to_numpy():
    p   = messaging.EpochMsgPayload()
    arr = _writable_view(p, messaging.EpochMsgPayload.__dtype__)

    p.year    = 1999
    p.seconds = 0.125

    assert arr[0]['year']    == 1999
    assert arr[0]['seconds'] == pytest.approx(0.125)


# ---------------------------------------------------------------------------
# 2. 1-D float arrays  (AttStateMsgPayload)
#    struct: double state[3], rate[3];  - 48 bytes, no padding
# ---------------------------------------------------------------------------

def test_att_state_dtype_layout():
    dt = messaging.AttStateMsgPayload.__dtype__
    assert dt is not None
    assert dt.itemsize == 48
    assert dt.fields['state'][1] == 0
    assert dt.fields['rate'][1]  == 24
    assert dt['state'].shape == (3,)
    assert dt['rate'].shape  == (3,)


def test_att_state_dtype_numpy_to_python():
    p   = messaging.AttStateMsgPayload()
    arr = _writable_view(p, messaging.AttStateMsgPayload.__dtype__)

    arr[0]['state'] = [0.1, 0.2, 0.3]
    arr[0]['rate']  = [1.0, 2.0, 3.0]

    np.testing.assert_allclose(p.state, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(p.rate,  [1.0, 2.0, 3.0])


def test_att_state_dtype_python_to_numpy():
    p   = messaging.AttStateMsgPayload()
    arr = _writable_view(p, messaging.AttStateMsgPayload.__dtype__)

    p.state = [9.0, 8.0, 7.0]
    p.rate  = [-1.0, -2.0, -3.0]

    np.testing.assert_allclose(arr[0]['state'], [9.0, 8.0, 7.0])
    np.testing.assert_allclose(arr[0]['rate'],  [-1.0, -2.0, -3.0])


# ---------------------------------------------------------------------------
# 3. 2-D float array  (SCMassPropsMsgPayload)
#    struct: double massSC; double c_B[3]; double ISC_PntB_B[3][3];  - 104 bytes
# ---------------------------------------------------------------------------

def test_sc_mass_props_dtype_layout():
    dt = messaging.SCMassPropsMsgPayload.__dtype__
    assert dt is not None
    assert dt.itemsize == 104
    assert dt.fields['massSC'][1]     == 0
    assert dt.fields['c_B'][1]        == 8
    assert dt.fields['ISC_PntB_B'][1] == 32
    assert dt['ISC_PntB_B'].shape == (3, 3)


def test_sc_mass_props_dtype_numpy_to_python():
    p   = messaging.SCMassPropsMsgPayload()
    arr = _writable_view(p, messaging.SCMassPropsMsgPayload.__dtype__)

    inertia = np.array([[1., 0., 0.], [0., 2., 0.], [0., 0., 3.]])
    arr[0]['massSC']     = 500.0
    arr[0]['c_B']        = [0.1, 0.2, 0.3]
    arr[0]['ISC_PntB_B'] = inertia

    assert p.massSC == pytest.approx(500.0)
    np.testing.assert_allclose(list(p.c_B), [0.1, 0.2, 0.3])
    np.testing.assert_allclose(list(p.ISC_PntB_B), inertia.tolist())


def test_sc_mass_props_dtype_python_to_numpy():
    p   = messaging.SCMassPropsMsgPayload()
    arr = _writable_view(p, messaging.SCMassPropsMsgPayload.__dtype__)

    p.massSC = 750.0

    assert arr[0]['massSC'] == pytest.approx(750.0)


# ---------------------------------------------------------------------------
# 4. Enum field  (DeviceStatusMsgPayload)
#    struct: deviceState deviceStatus;  - 4 bytes
# ---------------------------------------------------------------------------

def test_device_status_dtype_layout():
    dt = messaging.DeviceStatusMsgPayload.__dtype__
    assert dt is not None
    assert dt.itemsize == 4
    assert dt.fields['deviceStatus'][0] == np.dtype('int32')
    assert dt.fields['deviceStatus'][1] == 0


def test_device_status_dtype_numpy_to_python():
    p   = messaging.DeviceStatusMsgPayload()
    arr = _writable_view(p, messaging.DeviceStatusMsgPayload.__dtype__)

    arr[0]['deviceStatus'] = 1   # On
    assert p.deviceStatus == 1


def test_device_status_dtype_python_to_numpy():
    p   = messaging.DeviceStatusMsgPayload()
    arr = _writable_view(p, messaging.DeviceStatusMsgPayload.__dtype__)

    p.deviceStatus = 0           # Off
    assert arr[0]['deviceStatus'] == 0


# ---------------------------------------------------------------------------
# 5. Char array + 2-D array + scalar  (SpicePlanetStateMsgPayload)
#    struct: double J2000Current; double PositionVector[3]; double VelocityVector[3];
#            double J20002Pfix[3][3]; double J20002Pfix_dot[3][3];
#            int computeOrient; char PlanetName[64];  - 272 bytes
# ---------------------------------------------------------------------------

def test_spice_planet_dtype_layout():
    dt = messaging.SpicePlanetStateMsgPayload.__dtype__
    assert dt is not None
    assert dt.itemsize == 272
    assert dt['PlanetName']            == np.dtype('S64')
    assert dt.fields['J2000Current'][1]  == 0
    assert dt.fields['PlanetName'][1]    == 204


def test_spice_planet_dtype_numpy_to_python():
    p   = messaging.SpicePlanetStateMsgPayload()
    arr = _writable_view(p, messaging.SpicePlanetStateMsgPayload.__dtype__)

    arr[0]['J2000Current']   = 1234.5
    arr[0]['PositionVector'] = [1e6, 2e6, 3e6]
    arr[0]['PlanetName']     = b'Earth'

    assert p.J2000Current == pytest.approx(1234.5)
    np.testing.assert_allclose(list(p.PositionVector), [1e6, 2e6, 3e6])
    # SWIG exposes char[] as a Python string
    assert p.PlanetName[:5] == 'Earth'


def test_spice_planet_dtype_python_to_numpy():
    p   = messaging.SpicePlanetStateMsgPayload()
    arr = _writable_view(p, messaging.SpicePlanetStateMsgPayload.__dtype__)

    p.J2000Current = 9999.0
    assert arr[0]['J2000Current'] == pytest.approx(9999.0)


# ---------------------------------------------------------------------------
# 6. Array of nested structs  (AccDataMsgPayload)
#    struct: AccPktDataMsgPayload accPkts[120];
#    each AccPktDataMsgPayload: uint64 measTime + double gyro_B[3] + double accel_B[3] = 56 bytes
#    total: 6720 bytes
# ---------------------------------------------------------------------------

def test_acc_data_dtype_layout():
    dt = messaging.AccDataMsgPayload.__dtype__
    assert dt is not None
    assert dt.itemsize == 6720
    # accPkts field: array of 120 nested structs
    pkt_dt = dt['accPkts'].base
    assert pkt_dt.itemsize == 56
    assert pkt_dt.fields['measTime'][1] == 0
    assert pkt_dt.fields['gyro_B'][1]   == 8
    assert pkt_dt.fields['accel_B'][1]  == 32


def test_acc_data_dtype_numpy_to_python():
    p   = messaging.AccDataMsgPayload()
    arr = _writable_view(p, messaging.AccDataMsgPayload.__dtype__)

    arr[0]['accPkts'][0]['measTime'] = 999
    arr[0]['accPkts'][0]['gyro_B']   = [0.1, 0.2, 0.3]

    assert p.accPkts[0].measTime == 999
    np.testing.assert_allclose(list(p.accPkts[0].gyro_B), [0.1, 0.2, 0.3])


def test_acc_data_dtype_python_to_numpy():
    p   = messaging.AccDataMsgPayload()
    arr = _writable_view(p, messaging.AccDataMsgPayload.__dtype__)

    p.accPkts[5].measTime = 42
    assert arr[0]['accPkts'][5]['measTime'] == 42


# ---------------------------------------------------------------------------
# 7. Struct with pointer field → pointer is uint64 in dtype (CameraImageMsgPayload)
#    struct: uint64 timeTag; int valid; int64 cameraID; void* imagePointer;
#            int32 imageBufferLength; int8 imageType;  - 40 bytes
# ---------------------------------------------------------------------------

def test_camera_image_dtype_layout():
    dt = messaging.CameraImageMsgPayload.__dtype__
    assert dt is not None
    assert dt.itemsize == 40
    assert dt['imagePointer']              == np.dtype('uint64')
    assert dt.fields['timeTag'][1]         == 0
    assert dt.fields['imagePointer'][1]    == 24
    assert dt.fields['imageBufferLength'][1] == 32


def test_camera_image_dtype_numpy_to_python():
    p   = messaging.CameraImageMsgPayload()
    arr = _writable_view(p, messaging.CameraImageMsgPayload.__dtype__)

    arr[0]['timeTag']           = 123456789
    arr[0]['valid']             = 1
    arr[0]['imageBufferLength'] = 512

    assert p.timeTag           == 123456789
    assert p.valid             == 1
    assert p.imageBufferLength == 512


def test_camera_image_dtype_python_to_numpy():
    p   = messaging.CameraImageMsgPayload()
    arr = _writable_view(p, messaging.CameraImageMsgPayload.__dtype__)

    p.timeTag = 987654321
    assert arr[0]['timeTag'] == 987654321


# ---------------------------------------------------------------------------
# 8. Incomplete struct (C++ std::vector fields) → __dtype__ is None
# ---------------------------------------------------------------------------

def test_joint_array_state_dtype_is_none():
    assert messaging.JointArrayStateMsgPayload.__dtype__ is None
