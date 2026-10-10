# ISC License
#
# Copyright (c) 2026, Suhas Beemineni
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

"""Validate message-driven moving camera platforms without a Vizard renderer."""

import sys
from pathlib import Path

import numpy as np
import pytest

from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import vizSupport

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples"))
import scenarioMovingCamera  # noqa: E402


@pytest.fixture(scope="module", params=[False, True], ids=["core", "visualization"])
def camera_run(request):
    """Execute the same native scenario with and without visualization wiring."""
    if request.param and not vizSupport.vizFound:
        pytest.skip("Basilisk built without vizInterface")
    return request.param, scenarioMovingCamera.run(False, enable_viz=request.param)


def test_camera_platform_commands(camera_run):
    """Check independent platform rotations and translations after both commands.

    Validation Test Description
    ---------------------------
    Run native profilers and spacecraft dynamics, then compare each settled
    platform's inertial pose against the commanded angles and displacements.

    Test Parameter Discussion
    -------------------------
    The fixture enables or disables the optional visualization interface.

    Description of Variables Being Tested
    -------------------------------------
    Platform attitudes compose P/M, M/B, and B/N rotations. Positions include
    the inertial host position and body-frame mount offsets. Absolute position
    tolerance is 10 nm; dimensionless DCM tolerance is 2e-8. Relative tolerance
    is zero so a large inertial position cannot conceal a mounting error.
    """
    _, data = camera_run
    sample_indexes = [np.searchsorted(data["time_s"], data["switch_time_s"]), -1]
    for stage, sample in enumerate(sample_indexes):
        dcm_BN = rbk.MRP2C(data["hub_sigma"][sample])
        for camera in range(2):
            dcm_PM = rbk.PRV2C(data["axes"][camera]*data["commands_rad"][stage, camera])
            dcm_MB = rbk.MRP2C(data["mount_sigmas"][camera])
            expected_dcm = dcm_PM @ dcm_MB @ dcm_BN
            actual_dcm = rbk.MRP2C(data["platform_sigma"][camera][sample])
            np.testing.assert_allclose(actual_dcm, expected_dcm, rtol=0, atol=2e-8)
            displacement_M = np.zeros(3)  # [m]
            if camera == 0:
                displacement_M[0] = data["translations_m"][stage]
            position_N = data["hub_position"][sample] + dcm_BN.T @ (
                data["offsets"][camera] + dcm_MB.T @ displacement_M
            )
            position_tolerance_m = 1e-8  # [m]
            np.testing.assert_allclose(
                data["platform_position"][camera][sample], position_N,
                rtol=0, atol=position_tolerance_m,
            )


def test_camera_host_remains_fixed(camera_run):
    """Verify camera motion and retargeting leave the prescribed host pose fixed."""
    _, data = camera_run
    np.testing.assert_allclose(
        data["hub_sigma"], np.tile([0.1, -0.2, 0.05], (len(data["time_s"]), 1)),
        rtol=0, atol=1e-14,
    )
    position_tolerance_m = 1e-10  # [m]
    np.testing.assert_allclose(
        data["hub_position"], np.tile([100.0, -200.0, 300.0], (len(data["time_s"]), 1)),
        rtol=0, atol=position_tolerance_m,
    )
    for attitudes in data["platform_sigma"]:
        # Both platform boresights must change while the host stays fixed.
        start = rbk.MRP2C(attitudes[0]).T[:, 2]
        end = rbk.MRP2C(attitudes[-1]).T[:, 2]
        assert np.linalg.norm(end - start) > 0.1  # [-]


def test_camera_visualization_parent_wiring(camera_run):
    """Check native visualization buffers receive the final platform poses."""
    enabled, data = camera_run
    if not enabled:
        assert data["camera_parents"] == []
        assert data["viz_parents"] == []
        return
    expected_parents = ["cameraPlatform1", "cameraPlatform2"]
    assert data["camera_parents"] == expected_parents
    assert data["viz_camera_parents"] == expected_parents
    assert data["viz_parents"] == ["", "cameraHost", "cameraHost"]
    for camera in range(2):
        np.testing.assert_allclose(
            data["viz_platform_sigma"][camera], data["platform_sigma"][camera][-1],
            rtol=0, atol=1e-14,
        )
