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
#   Module Name:        linearTranslatingBodiesNDOF
#   Author:             João Vaz Carneiro
#   Creation Date:      October 10, 2025
#

import numpy as np
from Basilisk.simulation import linearTranslationNDOFStateEffector


def test_changeParametersTranslatingBody(show_plots):
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
    IPntFc_F = [[767, 0.0, 0.0],
                 [0.0, 912, 0.0],
                 [0.0, 0.0, 360]]
    dcm_FP = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    r_FcF_F = [[0.412],
               [0.623],
               [0.456]]
    r_F0P_P = [[0.199],
              [0.624],
              [0.771]]
    fHat_P = [[1], [0], [0]]
    k = 152
    c = 432

    translatingBodyEffector = linearTranslationNDOFStateEffector.LinearTranslationNDOFStateEffector()
    translatingBodyEffector.addTranslatingBody(linearTranslationNDOFStateEffector.TranslatingBody())

    translatingBody = translatingBodyEffector.getTranslatingBody(0)
    translatingBody.setMass(mass)
    translatingBody.setIPntFc_F(IPntFc_F)
    translatingBody.setDCM_FP(dcm_FP)
    translatingBody.setR_FcF_F(r_FcF_F)
    translatingBody.setR_F0P_P(r_F0P_P)
    translatingBody.setFHat_P(fHat_P)
    translatingBody.setK(k)
    translatingBody.setC(c)

    translatingBodyTest = translatingBodyEffector.getTranslatingBody(0)
    np.testing.assert_equal(translatingBodyTest.getMass(), mass)
    np.testing.assert_equal(translatingBodyTest.getIPntFc_F(), IPntFc_F)
    np.testing.assert_equal(translatingBodyTest.getDCM_FP(), dcm_FP)
    np.testing.assert_equal(translatingBodyTest.getR_FcF_F(), r_FcF_F)
    np.testing.assert_equal(translatingBodyTest.getR_F0P_P(), r_F0P_P)
    np.testing.assert_equal(translatingBodyTest.getFHat_P(), fHat_P)
    np.testing.assert_equal(translatingBodyTest.getK(), k)
    np.testing.assert_equal(translatingBodyTest.getC(), c)


if __name__ == "__main__":
    changeParameters()
