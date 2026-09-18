# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Verify the public naming example against message outputs and legacy dynamics."""

from pathlib import Path
import sys

import numpy as np
import pytest

from Basilisk.simulation import hingedRigidBodyStateEffector

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples"))
import scenarioEffectorNaming


@pytest.mark.scenarioTest
@pytest.mark.parametrize("custom_names", [False, True])
def test_scenarioEffectorNaming(custom_names):
    """Names ignore unrelated constructions while callback and message histories agree."""
    histories = []
    # Keep an unrelated effector alive so constructor counters cannot mimic local allocation.
    unrelated = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    for local in (False, True):
        names, angles, recorded_angles, figures = scenarioEffectorNaming.run(
            False, useManagerLocalEffectorNames=local, customNames=custom_names)
        assert len(set(names)) == 2
        if custom_names:
            assert names[0] == "leftPanelAngle"
        if local:
            assert names[1] == "hingedRigidBodyTheta2"
            if not custom_names:
                assert names[0] == "hingedRigidBodyTheta1"
        np.testing.assert_allclose(angles, recorded_angles, rtol=0.0, atol=1e-14)
        assert np.all(np.isfinite(angles))
        assert abs(angles[-1] - angles[0]) > 1e-4  # [rad]
        assert figures
        histories.append(angles)
    np.testing.assert_allclose(histories[0], histories[1], rtol=0.0, atol=1e-13)
