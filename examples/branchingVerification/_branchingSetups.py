#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


r"""Shared host configurations for the effector-branching verification scripts.

Both :ref:`scenarioBranchingConvergence` and :ref:`scenarioBranchingConservation` exercise the same
family of field-free configurations: a hub, a moving-platform host state effector, and an
``extForceTorque`` dynamic effector branched onto the host's tip segment. The host builders live
here so the two scripts cannot drift apart in the configuration they verify.

Each builder returns the configured state effector and a :class:`HostProps` describing the tip
segment, which is all the convergence study needs, plus the mass properties and configuration log
name the conservation study needs to rebuild the external impulse in post-processing.
"""

import numpy as np

from Basilisk.simulation import (
    extForceTorque,
    linearTranslationOneDOFStateEffector,
    spinningBodyNDOFStateEffector,
    spinningBodyTwoDOFStateEffector,
)
from Basilisk.utilities import macros


class HostProps:
    """Information needed for the post-processing impulse computation."""

    def __init__(self, total_mass, r_PcP_P, log_attr, segment):
        self.total_mass = float(total_mass)
        self.r_PcP_P = np.array(r_PcP_P, dtype=np.float64).flatten()
        self.log_attr = log_attr
        self.segment = int(segment)


def setup_extFT():
    extFT = extForceTorque.ExtForceTorque()
    extFT.ModelTag = "extFT"
    extFT.extForce_B = [[10.0], [-5.0], [3.0]]
    extFT.extTorquePntB_B = [[2.0], [-1.0], [4.0]]
    return extFT


def setup_spinningBodyNDOF():
    """3-segment, 2-DOF-per-segment spinning body. Tip = body 6."""
    sbe = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    sbe.ModelTag = "spinningBodyNDOF"

    numberOfSegments = 3
    massSubPanel = 100.0 / numberOfSegments
    lengthSubPanel = 18.0 / numberOfSegments
    widthSubPanel = 3.0
    thicknessSubPanel = 0.3

    for idx in range(numberOfSegments):
        sb = spinningBodyNDOFStateEffector.SpinningBody()
        sb.setMass(0.0)
        sb.setISPntSc_S([[0.0, 0.0, 0.0],
                         [0.0, 0.0, 0.0],
                         [0.0, 0.0, 0.0]])
        sb.setDCM_S0P([[1.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0],
                       [0.0, 0.0, 1.0]])
        sb.setR_ScS_S([[0.0], [lengthSubPanel / 2], [0.0]])
        if idx == 0:
            sb.setR_SP_P([[0.0], [1.5], [1.5 - thicknessSubPanel / 2]])
        else:
            sb.setR_SP_P([[0.0], [lengthSubPanel], [0.0]])
        sb.setSHat_S([[1], [0], [0]])
        sb.setThetaInit(2.0 * macros.D2R)
        sb.setThetaDotInit(-0.5 * macros.D2R)
        sb.setK(10.0)
        sb.setC(8.0)
        sbe.addSpinningBody(sb)

        sb = spinningBodyNDOFStateEffector.SpinningBody()
        sb.setMass(massSubPanel)
        sb.setISPntSc_S([[massSubPanel / 12 * (lengthSubPanel ** 2 + thicknessSubPanel ** 2), 0.0, 0.0],
                         [0.0, massSubPanel / 12 * (widthSubPanel ** 2 + thicknessSubPanel ** 2), 0.0],
                         [0.0, 0.0, massSubPanel / 12 * (widthSubPanel ** 2 + lengthSubPanel ** 2)]])
        sb.setDCM_S0P([[1.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0],
                       [0.0, 0.0, 1.0]])
        sb.setR_ScS_S([[0.0], [lengthSubPanel / 2], [0.0]])
        sb.setR_SP_P([[0.0], [0.0], [0.0]])
        sb.setSHat_S([[0], [1], [0]])
        sb.setThetaInit(2.0 * macros.D2R)
        sb.setThetaDotInit(-0.5 * macros.D2R)
        sb.setK(1.0)
        sb.setC(0.8)
        sbe.addSpinningBody(sb)

    props = HostProps(
        total_mass=massSubPanel * numberOfSegments,
        r_PcP_P=[0.0, lengthSubPanel / 2, 0.0],
        log_attr="spinningBodyConfigLogOutMsgs",
        segment=6,
    )
    return sbe, props


def setup_spinningBodyTwoDOF():
    sb = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    sb.ModelTag = "spinningBodyTwoDOF"
    sb.mass1 = 100.0
    sb.mass2 = 50.0
    sb.IS1PntSc1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    sb.IS2PntSc2_S2 = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    sb.dcm_S10B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    sb.dcm_S20S1 = [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
    sb.r_Sc1S1_S1 = [[1.0], [-0.5], [0.0]]
    sb.r_Sc2S2_S2 = [[1.0], [0.0], [-1.0]]
    sb.r_S1B_B = [[-1.0], [0.5], [-1.0]]
    sb.r_S2S1_S1 = [[0.5], [-0.5], [-0.5]]
    sb.s1Hat_S1 = [[0], [0], [1]]
    sb.s2Hat_S2 = [[0], [-1], [0]]
    sb.theta1DotInit = 1.0 * macros.D2R
    sb.theta2DotInit = 1.0 * macros.D2R
    sb.k1 = 1000.0
    sb.k2 = 500.0
    sb.c1 = 500.0
    sb.c2 = 200.0

    props = HostProps(
        total_mass=150.0,
        r_PcP_P=[1.0, 0.0, -1.0],          # r_Sc2S2_S2
        log_attr="spinningBodyConfigLogOutMsgs",
        segment=2,
    )
    return sb, props


def setup_linearTranslationOneDOF():
    tr = linearTranslationOneDOFStateEffector.LinearTranslationOneDOFStateEffector()
    tr.ModelTag = "linearTranslationOneDOF"
    tr.setMass(20.0)
    tr.setK(100.0)
    tr.setC(70.0)            # zeta ~ 0.78, matching spinningBodyTwoDOF damping
    tr.setRhoInit(0.0)       # start at spring equilibrium so the prefactor of
    tr.setRhoDotInit(0.0)    # the leading dt^4 error matches the spinning bodies
    tr.setFHat_B([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    tr.setR_FcF_F([[-1.0], [1.0], [0.0]])
    tr.setR_F0B_B([[-1.0], [1.0], [0.0]])
    tr.setIPntFc_F([[50.0, 0.0, 0.0],
                    [0.0, 80.0, 0.0],
                    [0.0, 0.0, 60.0]])
    tr.setDCM_FB([[0.0, -1.0, 0.0],
                  [0.0, 0.0, -1.0],
                  [1.0, 0.0, 0.0]])

    props = HostProps(
        total_mass=20.0,
        r_PcP_P=[-1.0, 1.0, 0.0],          # r_FcF_F
        log_attr="translatingBodyConfigLogOutMsg",
        segment=1,
    )
    return tr, props
