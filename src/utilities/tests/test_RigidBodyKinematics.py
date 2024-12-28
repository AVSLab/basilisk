# ISC License
#
# Copyright (c) 2024, AVS Laboratory, University of Colorado at Boulder
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

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk


def test_subMRP(show_plots):
    mrp_1 = np.array([1.5, 0.5, 0.5])
    mrp_2 = np.array([-0.5, 0.25, 0.15])
    ans = [-0.005376344086021518,0.04301075268817203,-0.4408602150537635]
    np.testing.assert_allclose(rbk.subMRP(mrp_1, mrp_2), ans, atol=1e-10, err_msg="subtraction failed")

    # should get the same answer if one MRP is switched to shadoe set
    mrp_2 = rbk.MRPswitch(mrp_2, 0.0)
    np.testing.assert_allclose(rbk.subMRP(mrp_1, mrp_2), ans, atol=1e-10, err_msg="subtraction after q2 MRP switch failed")

    mrp_1 = np.array([0.0, 0.0, 1.0])
    mrp_2 = np.array([0.0, 0.0, -1.0])
    ans = [0.0, 0.0, 0.0]
    np.testing.assert_allclose(rbk.subMRP(mrp_1, mrp_2), ans, atol=1e-10, err_msg="180 deg case failed")


if __name__ == "__main__":
    test_subMRP(True)
