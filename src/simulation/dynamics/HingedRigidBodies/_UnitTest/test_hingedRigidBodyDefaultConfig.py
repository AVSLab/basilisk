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

#
#   Unit Test Script
#   Module Name:        hingedRigidBodyStateEffector
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      July 4, 2026
#

"""
Regression test for issue #469: the constructor used to call
``this->IPntS_S.Identity()`` and ``this->dcm_HB.Identity()`` as statements.
``Eigen::Matrix3d::Identity()`` is a static factory whose return value was
discarded, so the members were left uninitialized instead of defaulting to
identity. This test asserts a freshly constructed effector exposes identity
defaults for both members.
"""

import numpy as np
from Basilisk.simulation import hingedRigidBodyStateEffector


def test_defaultDcmAndInertiaAreIdentity():
    """A fresh effector must default dcm_HB and IPntS_S to identity."""
    effector = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    np.testing.assert_allclose(np.asarray(effector.dcm_HB, dtype=float), np.eye(3),
                               err_msg="default dcm_HB is not identity")
    np.testing.assert_allclose(np.asarray(effector.IPntS_S, dtype=float), np.eye(3),
                               err_msg="default IPntS_S is not identity")


if __name__ == "__main__":
    test_defaultDcmAndInertiaAreIdentity()
