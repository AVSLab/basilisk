# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Shared regression tests for the ground access modules' overhead convention."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import groundLocation, groundMapping, stripLocation


@pytest.mark.parametrize("module_name", ["groundLocation", "groundMapping", "stripLocation"])
@pytest.mark.parametrize("origin_distance", [0.0, 1.5e11])  # [m]
@pytest.mark.parametrize("horizontal_offset", [0.0, 0.01])  # [m]
@pytest.mark.parametrize("lat_deg, lon_deg, tilt_deg", [
    (0.0, 0.0, 0.0), (35.0, -70.0, 30.0), (90.0, 20.0, 60.0), (-90.0, -140.0, 20.0),
])  # [deg]
def test_overheadResolution(module_name, origin_distance, horizontal_offset, lat_deg, lon_deg, tilt_deg):
    """Preserve resolved offsets while guarding exact overhead geometry in all three modules.

    The planet and target are stationary. A moving spacecraft is either exactly overhead
    or displaced 1 cm to the east. Translating the inertial origin must not suppress the
    nonzero azimuth and angular rates of the displaced case. Tilted planet frames and
    polar sites exercise round-off in the transformations of exactly overhead positions.

    :param module_name: ground access module to exercise
    :param origin_distance: [m] scale of the planet displacement from the inertial origin
    :param horizontal_offset: [m] spacecraft displacement east of the target
    :param lat_deg: [deg] target latitude
    :param lon_deg: [deg] target longitude
    :param tilt_deg: [deg] planet frame tilt about the inertial second axis
    """
    radius = 6378136.3  # [m]
    altitude = 600e3  # [m]
    lat, lon, tilt = np.radians([lat_deg, lon_deg, tilt_deg])  # [rad]
    dcm_LP = np.array([
        [np.sin(lat) * np.cos(lon), np.sin(lat) * np.sin(lon), -np.cos(lat)],
        [-np.sin(lon), np.cos(lon), 0.0],
        [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)],
    ])
    dcm_PN = np.array([[np.cos(tilt), 0.0, -np.sin(tilt)], [0.0, 1.0, 0.0],
                       [np.sin(tilt), 0.0, np.cos(tilt)]])
    r_PN_N = origin_distance * np.array([1.0, -0.13, 0.03])  # [m]
    r_LP_P = radius * dcm_LP[2]  # [m]
    expected_r_BL_L = np.array([0.0, horizontal_offset, altitude])  # [m]
    expected_v_BL_L = np.array([-10.0, 2.0, 1.0])  # [m/s]

    planet_payload = messaging.SpicePlanetStateMsgPayload()
    planet_payload.PositionVector = r_PN_N.tolist()
    planet_payload.J20002Pfix = dcm_PN.tolist()
    planet_message = messaging.SpicePlanetStateMsg().write(planet_payload)
    spacecraft_payload = messaging.SCStatesMsgPayload()
    spacecraft_payload.r_BN_N = (r_PN_N + dcm_PN.T @ (r_LP_P + dcm_LP.T @ expected_r_BL_L)).tolist()
    spacecraft_payload.v_BN_N = (dcm_PN.T @ dcm_LP.T @ expected_v_BL_L).tolist()
    spacecraft_message = messaging.SCStatesMsg().write(spacecraft_payload)

    if module_name == "groundLocation":
        model = groundLocation.GroundLocation()
        model.planetRadius = radius
        model.specifyLocation(lat, lon, 0.0)  # [rad, rad, m]
        model.addSpacecraftToModel(spacecraft_message)
    elif module_name == "groundMapping":
        model = groundMapping.GroundMapping()
        model.nHat_B = [1.0, 0.0, 0.0]
        model.addPointToModel(r_LP_P.tolist())
        model.scStateInMsg.subscribeTo(spacecraft_message)
    else:
        model = stripLocation.StripLocation()
        model.planetRadius = radius
        model.acquisitionSpeed = 0.0  # [m/ns]
        model.specifyLocationStart(lat, lon, 0.0)  # [rad, rad, m]
        model.specifyLocationEnd(lat, lon, 0.0)  # [rad, rad, m]
        model.addSpacecraftToModel(spacecraft_message)
    model.planetInMsg.subscribeTo(planet_message)
    model.Reset(0)  # [ns]
    model.UpdateState(0)  # [ns]
    access = model.accessOutMsgs[0].read()

    # Allow position quantization at a distant origin, but require accurate local velocity.
    np.testing.assert_allclose(access.r_BL_L, expected_r_BL_L, rtol=0.0, atol=1e-4)  # [m]
    np.testing.assert_allclose(access.v_BL_L, expected_v_BL_L, rtol=0.0, atol=1e-10)  # [m/s]
    if horizontal_offset == 0.0:
        assert access.azimuth == 0.0  # [rad]
        assert access.az_dot == 0.0  # [rad/s]
        assert access.el_dot == 0.0  # [rad/s]
    else:
        # Analytic rates for a spacecraft due east of the target; tolerances cover input position quantization.
        expected_az_dot = expected_v_BL_L[0] / horizontal_offset  # [rad/s]
        expected_el_dot = (horizontal_offset * expected_v_BL_L[2] - altitude * expected_v_BL_L[1]) / (
            horizontal_offset**2 + altitude**2)  # [rad/s]
        assert access.azimuth == pytest.approx(np.pi / 2, abs=1e-2)  # [rad]
        assert access.az_dot == pytest.approx(expected_az_dot, rel=1e-2)  # [rad/s]
        assert access.el_dot == pytest.approx(expected_el_dot, rel=1e-2)  # [rad/s]


if __name__ == "__main__":
    pytest.main([__file__])
