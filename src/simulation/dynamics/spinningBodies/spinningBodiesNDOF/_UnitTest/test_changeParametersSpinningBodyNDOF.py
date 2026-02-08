# ISC License
#
# Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        spinningBodiesNDOF
#   Author:             João Vaz Carneiro
#   Creation Date:      October 10, 2025
#

import numpy as np
from Basilisk.simulation import spinningBodyNDOFStateEffector


def test_changeParametersSpinningBody(show_plots):
    r"""
    **Validation Test Description**

    This unit test checks that we can retrieve a body of the N-DOF module, change its parameters and retrieve them while
    keeping all data in memory.

    **Description of Variables Being Tested**

    All setter and getter methods are exercised and compared to the truth values.
    """
    changeParameters()


def changeParameters():
    mass = 20
    ISPntSc_S = [[767, 0.0, 0.0],
                 [0.0, 912, 0.0],
                 [0.0, 0.0, 360]]
    dcm_S0P = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    r_ScS_S = [[0.412],
               [0.623],
               [0.456]]
    r_SP_P = [[0.199],
              [0.624],
              [0.771]]
    sHat_S = [[1], [0], [0]]
    k = 152
    c = 432

    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    spinningBodyEffector.addSpinningBody(spinningBodyNDOFStateEffector.SpinningBody())

    spinningBody = spinningBodyEffector.getSpinningBody(0)
    spinningBody.setMass(mass)
    spinningBody.setISPntSc_S(ISPntSc_S)
    spinningBody.setDCM_S0P(dcm_S0P)
    spinningBody.setR_ScS_S(r_ScS_S)
    spinningBody.setR_SP_P(r_SP_P)
    spinningBody.setSHat_S(sHat_S)
    spinningBody.setK(k)
    spinningBody.setC(c)

    spinningBodyTest = spinningBodyEffector.getSpinningBody(0)
    np.testing.assert_equal(spinningBodyTest.getMass(), mass)
    np.testing.assert_equal(spinningBodyTest.getISPntSc_S(), ISPntSc_S)
    np.testing.assert_equal(spinningBodyTest.getDCM_S0P(), dcm_S0P)
    np.testing.assert_equal(spinningBodyTest.getR_ScS_S(), r_ScS_S)
    np.testing.assert_equal(spinningBodyTest.getR_SP_P(), r_SP_P)
    np.testing.assert_equal(spinningBodyTest.getSHat_S(), sHat_S)
    np.testing.assert_equal(spinningBodyTest.getK(), k)
    np.testing.assert_equal(spinningBodyTest.getC(), c)


if __name__ == "__main__":
    changeParameters()
