#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

import math

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import groundMapping
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

import pytest

@pytest.mark.parametrize("maxRange", [1e-12, 0.001, -1.0])

def test_groundMapping(maxRange):
    r"""
    This test checks two points to determine if they are accessible for mapping or not. One point should be mapped,
    and one point should not be mapped.

    The inertial, planet-fixed planet-centered, and spacecraft body frames are all aligned.
    The spacecraft is in the -y direction of the inertial frame. The first point is along the line from the spacecraft
    to the origin. The second point is along the z-axis. The first point should be accessible because a.) the spacecraft
    is within the point's visibility cone and the point is within the spacecraft's visibility cone. The second point is
    not accessible because the spacecraft is not within the point's visibility cone and the point is not within the
    spacecraft's visibility cone.
    """
    [testResults, testMessage] = groundMappingTestFunction(maxRange)
    assert testResults < 1, testMessage


def groundMappingTestFunction(maxRange):
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Configure blank module input messages
    planetInMsgData = messaging.SpicePlanetStateMsgPayload()
    planetInMsgData.J20002Pfix = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    planetInMsgData.PositionVector = [0., 0., 0.]
    planetInMsg = messaging.SpicePlanetStateMsg().write(planetInMsgData)

    scStateInMsgData = messaging.SCStatesMsgPayload()
    scStateInMsgData.r_BN_N = [0., -1., 0.]
    scStateInMsgData.sigma_BN = [0., 0., 0.]
    scStateInMsg = messaging.SCStatesMsg().write(scStateInMsgData)

    # Create the initial imaging target
    groundMap = groundMapping.GroundMapping()
    groundMap.ModelTag = "groundMapping"
    groundMap.addPointToModel([0., -0.1, 0.])
    groundMap.addPointToModel([0., 0., math.tan(np.radians(22.5))+0.1])
    groundMap.minimumElevation = np.radians(45.)
    if maxRange > 0.0:
        groundMap.maximumRange = maxRange
    groundMap.cameraPos_B = [0, 0, 0]
    groundMap.nHat_B = [0, 1, 0]
    groundMap.halfFieldOfView = np.radians(22.5)
    groundMap.scStateInMsg.subscribeTo(scStateInMsg)
    groundMap.planetInMsg.subscribeTo(planetInMsg)
    unitTestSim.AddModelToTask(unitTaskName, groundMap)

    # Setup the logging for the mapping locations
    mapLog = []
    for idx in range(0, 2):
        mapLog.append(groundMap.accessOutMsgs[idx].recorder())
        unitTestSim.AddModelToTask(unitTaskName, mapLog[idx])

    # subscribe input messages to module
    groundMap.planetInMsg.subscribeTo(planetInMsg)
    groundMap.scStateInMsg.subscribeTo(scStateInMsg)

    # setup output message recorder objects
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    map_access = np.zeros(2, dtype=bool)
    for idx in range(0, 2):
        access = mapLog[idx].hasAccess
        if sum(access):
            map_access[idx] = 1

    # If the first target is not mapped, failure
    if not map_access[0] and (maxRange > 1.0 or maxRange < 0.0) :
        testFailCount += 1

    # If the second target is mapped, failure
    if map_access[1]:
        testFailCount += 1

    if testFailCount == 0:
        print("PASSED: " + groundMap.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


def _skew(v):
    """Return the skew-symmetric cross-product matrix of a 3-vector."""
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _dcm_LP(lat, lon):
    """Return the planet-fixed to SEZ rotation matrix for a latitude and longitude [rad]."""
    return np.array([
        [np.sin(lat) * np.cos(lon), np.sin(lat) * np.sin(lon), -np.cos(lat)],
        [-np.sin(lon), np.cos(lon), 0.0],
        [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)]])


def _azEl(r_L):
    """Return azimuth and elevation [rad] of an SEZ position, using the module's azimuth convention."""
    return np.arctan2(r_L[1], -r_L[0]), np.arctan2(r_L[2], np.hypot(r_L[0], r_L[1]))


def _runMappingPoints(tilt_deg, v_PN_N, r_PN_N, v_rel_P, latLon_deg):
    """
    Run groundMapping for one step and return the access message records, the expected SEZ position and velocity
    of the spacecraft for each point, together with the planet-fixed spacecraft position.

    :param tilt_deg: [deg] angle between the planet spin axis and the inertial 3-axis
    :param v_PN_N: [m/s] planet velocity relative to the inertial frame origin, in N components
    :param r_PN_N: [m] planet position relative to the inertial frame origin, in N components
    :param v_rel_P: [m/s] spacecraft velocity relative to the planet-fixed frame, in P components
    :param latLon_deg: list of [lat, lon] pairs [deg] of the mapping points
    :return: access message records, expected r_BL_L list, expected v_BL_L list, ground state message records,
             expected (rHat_LP_N, r_LP_N, r_LN_N) list
    """
    radius = 6378136.3  # [m] planet radius
    omega = 7.2921159e-5  # [rad/s] planet spin rate
    tilt = np.radians(tilt_deg)  # [rad] angle between the planet spin axis and the inertial 3-axis

    c, s = np.cos(tilt), np.sin(tilt)
    dcm_PN = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])  # [-] N to P, the P 3-axis is the spin axis
    w_PN_P = np.array([0.0, 0.0, omega])  # [rad/s] angular velocity of P relative to N, in P components
    w_PN_N = dcm_PN.T @ w_PN_P  # [rad/s] same vector in N components
    dcm_PN_dot = -_skew(w_PN_P) @ dcm_PN  # [1/s] time derivative of the N to P DCM

    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.PositionVector = list(r_PN_N)
    planetPayload.VelocityVector = list(v_PN_N)
    planetPayload.J20002Pfix = dcm_PN.tolist()
    planetPayload.J20002Pfix_dot = dcm_PN_dot.tolist()
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload)

    r_BP_P = np.array([radius + 600e3, 3.0e5, 1.0e6])  # [m] spacecraft position relative to the planet, in P
    r_BP_N = dcm_PN.T @ r_BP_P  # [m]
    scPayload = messaging.SCStatesMsgPayload()
    scPayload.r_BN_N = (np.array(r_PN_N) + r_BP_N).tolist()
    # velocity in N: planet translation + rotation of the point + velocity relative to P rotated into N
    v_BN_N = np.array(v_PN_N) + np.cross(w_PN_N, r_BP_N) + dcm_PN.T @ np.array(v_rel_P)  # [m/s]
    scPayload.v_BN_N = v_BN_N.tolist()
    scMsg = messaging.SCStatesMsg().write(scPayload)

    scSim = SimulationBaseClass.SimBaseClass()
    scSim.CreateNewProcess("simProcess").addTask(scSim.CreateNewTask("simTask", macros.sec2nano(1.0)))
    groundMap = groundMapping.GroundMapping()
    groundMap.ModelTag = "groundMapping"
    groundMap.minimumElevation = -np.pi / 2
    groundMap.nHat_B = [0, 1, 0]
    groundMap.halfFieldOfView = np.radians(22.5)
    groundMap.planetInMsg.subscribeTo(planetMsg)
    groundMap.scStateInMsg.subscribeTo(scMsg)

    expected_r = []
    expected_v = []
    logs = []
    stateLogs = []
    for lat_deg, lon_deg in latLon_deg:
        lat, lon = np.radians(lat_deg), np.radians(lon_deg)  # [rad]
        r_LP_P = radius * np.array([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)])  # [m]
        groundMap.addPointToModel(r_LP_P.tolist())
        dcm_LP = _dcm_LP(lat, lon)
        expected_r.append(dcm_LP @ (r_BP_P - r_LP_P))
        # evaluate the rotation correction entirely in P, independent of the module's N-frame route
        v_BL_P = dcm_PN @ (v_BN_N - np.array(v_PN_N)) - np.cross(w_PN_P, r_BP_P)  # [m/s]
        expected_v.append(dcm_LP @ v_BL_P)
    scSim.AddModelToTask("simTask", groundMap)
    for idx in range(len(latLon_deg)):
        logs.append(groundMap.accessOutMsgs[idx].recorder())
        scSim.AddModelToTask("simTask", logs[idx])
        stateLogs.append(groundMap.currentGroundStateOutMsgs[idx].recorder())
        scSim.AddModelToTask("simTask", stateLogs[idx])

    scSim.InitializeSimulation()
    scSim.TotalSim.SingleStepProcesses()
    # expected ground state: position, surface normal in N components
    expected_state = []
    for lat_deg, lon_deg in latLon_deg:
        lat, lon = np.radians(lat_deg), np.radians(lon_deg)  # [rad]
        rHat_P = np.array([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)])  # [-]
        r_LP_N = dcm_PN.T @ (radius * rHat_P)  # [m]
        expected_state.append((dcm_PN.T @ rHat_P, r_LP_N, np.array(r_PN_N) + r_LP_N))
    return logs, expected_r, expected_v, stateLogs, expected_state


POINTS_LAT_LON = [[10.0, 20.0], [-35.0, 140.0], [60.0, -75.0]]  # [deg] mapping point latitude and longitude


@pytest.mark.parametrize("tilt_deg", [0.0, 30.0, 60.0])
@pytest.mark.parametrize("v_PN_N", [[0.0, 0.0, 0.0], [3.0e4, -1.0e3, 2.0e3]])  # [m/s]
def test_planetFixedSpacecraftZeroRates(tilt_deg, v_PN_N):
    """
    Tests that a spacecraft fixed in the planet frame has zero relative velocity and zero range, azimuth and
    elevation rates for several mapping points at different latitudes and longitudes, with a tilted planet spin axis,
    a moving planet and a planet displaced from the inertial origin.

    :param tilt_deg: [deg] angle between the planet spin axis and the inertial 3-axis
    :param v_PN_N: [m/s] planet velocity relative to the inertial frame origin, in N components
    :return:
    """
    logs, expected_r, _, stateLogs, expected_state = _runMappingPoints(tilt_deg, v_PN_N, [1.5e11, -2.0e10, 5.0e9], [0., 0., 0.],
                                               POINTS_LAT_LON)
    for log, r_exp in zip(logs, expected_r):
        np.testing.assert_allclose(log.r_BL_L[0], r_exp, rtol=1e-9, atol=1e-3)  # [m]
        np.testing.assert_allclose(log.v_BL_L[0], np.zeros(3), atol=1e-6)  # [m/s]
        assert log.range_dot[0] == pytest.approx(0.0, abs=1e-6)  # [m/s]
        assert log.az_dot[0] == pytest.approx(0.0, abs=1e-11)  # [rad/s]
        assert log.el_dot[0] == pytest.approx(0.0, abs=1e-11)  # [rad/s]
    for stateLog, (rHat_exp, r_LP_exp, r_LN_exp) in zip(stateLogs, expected_state):
        np.testing.assert_allclose(stateLog.rHat_LP_N[0], rHat_exp, atol=1e-12)  # [-]
        np.testing.assert_allclose(stateLog.r_LP_N[0], r_LP_exp, rtol=1e-12)  # [m]
        np.testing.assert_allclose(stateLog.r_LN_N[0], r_LN_exp, rtol=1e-12)  # [m]


@pytest.mark.parametrize("tilt_deg", [0.0, 30.0, 60.0])
@pytest.mark.parametrize("v_PN_N", [[0.0, 0.0, 0.0], [3.0e4, -1.0e3, 2.0e3]])  # [m/s]
def test_nonzeroRelativeVelocity(tilt_deg, v_PN_N):
    """
    Tests the SEZ position, SEZ velocity and the range, azimuth and elevation rates of a spacecraft moving relative
    to the planet for several mapping points. The expected velocity is evaluated entirely in the P frame and the
    angular rates are obtained by central finite differences of the SEZ geometry.

    :param tilt_deg: [deg] angle between the planet spin axis and the inertial 3-axis
    :param v_PN_N: [m/s] planet velocity relative to the inertial frame origin, in N components
    :return:
    """
    v_rel_P = np.array([-1.2e3, 6.5e3, 3.0e3])  # [m/s] spacecraft velocity relative to P, in P components
    logs, expected_r, expected_v, stateLogs, expected_state = _runMappingPoints(
        tilt_deg, v_PN_N, [1.5e11, -2.0e10, 5.0e9], v_rel_P, POINTS_LAT_LON)
    dt = 1e-3  # [s] finite difference step
    for log, r_exp, v_exp in zip(logs, expected_r, expected_v):
        np.testing.assert_allclose(log.r_BL_L[0], r_exp, rtol=1e-9, atol=1e-3)  # [m]
        np.testing.assert_allclose(log.v_BL_L[0], v_exp, rtol=1e-9, atol=1e-6)  # [m/s]
        assert np.linalg.norm(v_exp) > 1e3  # the case must exercise a nonzero relative velocity

        r_plus, r_minus = r_exp + v_exp * dt, r_exp - v_exp * dt  # [m]
        az_p, el_p = _azEl(r_plus)
        az_m, el_m = _azEl(r_minus)
        assert log.range_dot[0] == pytest.approx((np.linalg.norm(r_plus) - np.linalg.norm(r_minus)) / (2 * dt),
                                                 rel=1e-6)  # [m/s]
        assert log.az_dot[0] == pytest.approx((az_p - az_m) / (2 * dt), rel=1e-5, abs=1e-10)  # [rad/s]
        assert log.el_dot[0] == pytest.approx((el_p - el_m) / (2 * dt), rel=1e-5, abs=1e-10)  # [rad/s]


if __name__ == "__main__":
    test_groundMapping(1e9)
