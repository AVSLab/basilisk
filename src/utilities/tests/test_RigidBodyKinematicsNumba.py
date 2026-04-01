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

"""
Tests for RigidBodyKinematicsNumba (RBKN).

Two orthogonal concerns are checked:

  1. Side-effect isolation - importing RBKN must not alter RigidBodyKinematics
     (RBK).  All RBK functions must remain plain Python callables that accept
     the same inputs and return the same outputs as before RBKN was imported.

  2. Numerical correctness - every RBKN function must return results that
     match the RBK reference to floating-point tolerance.  The test matrix
     covers:
       - direct functions (no internal calls)
       - functions that call other functions (cross-call chains)
       - edge cases (zero rotation, shadow-set switches, 180° rotations)
"""

import inspect

import numpy as np
import pytest

from Basilisk.utilities import RigidBodyKinematics as RBK
# Import RBKN *after* RBK to exercise the side-effect isolation guarantee.
from Basilisk.utilities import RigidBodyKinematicsNumba as RBKN

ATOL = 1e-12

# ---------------------------------------------------------------------------
# Fixtures - shared attitude parameters
# ---------------------------------------------------------------------------

@pytest.fixture
def mrp():
    return np.array([0.1, 0.2, 0.3])

@pytest.fixture
def mrp2():
    return np.array([0.05, 0.1, 0.15])

@pytest.fixture
def ep():
    q = np.array([0.9, 0.1, 0.2, 0.3])
    return q / np.linalg.norm(q)

@pytest.fixture
def ep2():
    q = np.array([0.8, 0.2, 0.3, 0.1])
    return q / np.linalg.norm(q)

@pytest.fixture
def gibbs():
    return np.array([0.1, 0.2, 0.15])

@pytest.fixture
def prv():
    return np.array([0.3, 0.5, 0.2])

@pytest.fixture
def euler():
    return np.array([0.1, 0.2, 0.3])

@pytest.fixture
def euler2():
    return np.array([0.05, 0.1, 0.15])

@pytest.fixture
def dcm(mrp):
    return RBK.MRP2C(mrp)

@pytest.fixture
def omega():
    return np.array([0.01, 0.02, 0.03])

# ---------------------------------------------------------------------------
# 1. Side-effect isolation
# ---------------------------------------------------------------------------

class TestSideEffectIsolation:
    """Importing RBKN must not change the behaviour of RBK in any way."""

    def testRbkFunctionsRemainPlainPython(self):
        """All RBK functions must still be plain Python callables."""
        for name, obj in inspect.getmembers(RBK, inspect.isfunction):
            assert type(obj).__name__ == 'function', (
                f"RBK.{name} was replaced by a {type(obj).__name__} "
                f"(expected plain Python function)"
            )

    def testRbkMrp2cOutputType(self, mrp):
        C = RBK.MRP2C(mrp)
        assert isinstance(C, np.ndarray)
        assert C.shape == (3, 3)

    def testRbkC2epOutputType(self, dcm):
        ep = RBK.C2EP(dcm)
        assert isinstance(ep, np.ndarray)
        assert ep.shape == (4,)

    def testRbkAddmrpOutputType(self, mrp, mrp2):
        result = RBK.addMRP(mrp, mrp2)
        assert isinstance(result, np.ndarray)
        assert result.shape == (3,)

    def testRbkSubmrpUnchanged(self, mrp, mrp2):
        """Verify the original RBK.subMRP test still passes after RBKN import."""
        mrp_1 = np.array([1.5, 0.5, 0.5])
        mrp_2 = np.array([-0.5, 0.25, 0.15])
        ans = [-0.005376344086021518, 0.04301075268817203, -0.4408602150537635]
        np.testing.assert_allclose(
            RBK.subMRP(mrp_1, mrp_2), ans, atol=1e-10
        )
        mrp_2_shadow = RBK.MRPswitch(mrp_2, 0.0)
        np.testing.assert_allclose(
            RBK.subMRP(mrp_1, mrp_2_shadow), ans, atol=1e-10
        )

    def testRbkResultMatchesPreImportReference(self, mrp):
        """RBK.MRP2C must return the same matrix regardless of RBKN being imported."""
        expected = np.array([
            [ 0.19975377,  0.91720529, -0.34472145],
            [-0.67097568,  0.38442598,  0.63404124],
            [ 0.71406587,  0.10464758,  0.69221299],
        ])
        np.testing.assert_allclose(RBK.MRP2C(mrp), expected, atol=1e-7)

# ---------------------------------------------------------------------------
# 2. Numerical correctness - RBKN results must match RBK
# ---------------------------------------------------------------------------

class TestNumericalCorrectness:

    # --- Direct functions (no internal calls) ---

    def test_MRP2C(self, mrp):
        np.testing.assert_allclose(RBKN.MRP2C(mrp), RBK.MRP2C(mrp), atol=ATOL)

    def test_EP2C(self, ep):
        np.testing.assert_allclose(RBKN.EP2C(ep), RBK.EP2C(ep), atol=ATOL)

    def test_PRV2C(self, prv):
        np.testing.assert_allclose(RBKN.PRV2C(prv), RBK.PRV2C(prv), atol=ATOL)

    def test_PRV2C_zero_angle(self):
        """Edge case: zero PRV must return identity."""
        np.testing.assert_allclose(RBKN.PRV2C(np.zeros(3)), np.eye(3), atol=1e-10)

    def test_MRPswitch(self, mrp):
        np.testing.assert_allclose(
            RBKN.MRPswitch(mrp, 1.0), RBK.MRPswitch(mrp, 1.0), atol=ATOL
        )

    def test_Picheck(self):
        for angle in [-4.0, -np.pi, 0.0, np.pi, 4.0]:
            assert RBKN.Picheck(angle) == pytest.approx(RBK.Picheck(angle))

    def testV3tilde(self, mrp):
        np.testing.assert_allclose(RBKN.v3Tilde(mrp), RBK.v3Tilde(mrp), atol=ATOL)

    def test_Mi(self):
        for axis in [1, 2, 3]:
            np.testing.assert_allclose(
                RBKN.Mi(0.3, axis), RBK.Mi(0.3, axis), atol=ATOL
            )

    # --- Cross-call chains ---

    def test_C2MRP_calls_C2EP_EP2MRP(self, dcm):
        """C2MRP -> C2EP -> EP2MRP: two-level cross-call."""
        np.testing.assert_allclose(RBKN.C2MRP(dcm), RBK.C2MRP(dcm), atol=ATOL)

    def test_C2EP(self, dcm):
        np.testing.assert_allclose(RBKN.C2EP(dcm), RBK.C2EP(dcm), atol=ATOL)

    def testAddmrpCallsMrpswitch(self, mrp, mrp2):
        """addMRP internally calls MRPswitch for shadow-set check."""
        np.testing.assert_allclose(
            RBKN.addMRP(mrp, mrp2), RBK.addMRP(mrp, mrp2), atol=ATOL
        )

    def testSubmrpCallsAddmrpMrpswitch(self, mrp, mrp2):
        """subMRP -> addMRP -> MRPswitch: three-level cross-call chain."""
        np.testing.assert_allclose(
            RBKN.subMRP(mrp, mrp2), RBK.subMRP(mrp, mrp2), atol=ATOL
        )

    def testSubmrpShadowSet(self):
        """subMRP with a near-180° difference triggers shadow-set switch."""
        s1 = np.array([1.5, 0.5, 0.5])
        s2 = np.array([-0.5, 0.25, 0.15])
        np.testing.assert_allclose(
            RBKN.subMRP(s1, s2), RBK.subMRP(s1, s2), atol=ATOL
        )

    def testSubmrp180deg(self):
        """Exact 180° case must yield zero MRP."""
        s1 = np.array([0.0, 0.0, 1.0])
        s2 = np.array([0.0, 0.0, -1.0])
        np.testing.assert_allclose(
            RBKN.subMRP(s1, s2), np.zeros(3), atol=1e-10
        )

    def test_EP2MRP_calls_MRPswitch(self, ep):
        """EP2MRP calls MRPswitch to choose the shorter MRP set."""
        np.testing.assert_allclose(
            RBKN.EP2MRP(ep), RBK.EP2MRP(ep), atol=ATOL
        )

    def testAddepCallsEp2cC2ep(self, ep, ep2):
        """addEP -> EP2C -> C2EP: DCM-mediated composition."""
        np.testing.assert_allclose(
            RBKN.addEP(ep, ep2), RBK.addEP(ep, ep2), atol=ATOL
        )

    def testDmrp2omegaCallsBinvmrp(self, mrp, omega):
        """dMRP2Omega uses np.dot(BinvMRP(q), dq) (was np.matmul)."""
        np.testing.assert_allclose(
            RBKN.dMRP2Omega(mrp, omega), RBK.dMRP2Omega(mrp, omega), atol=ATOL
        )

    def testAddeuler123CallsMi(self, euler, euler2):
        """addEuler123 builds DCMs via Mi() internally."""
        np.testing.assert_allclose(
            RBKN.addEuler123(euler, euler2), RBK.addEuler123(euler, euler2),
            atol=ATOL,
        )

    def testAddeuler321CallsMi(self, euler, euler2):
        np.testing.assert_allclose(
            RBKN.addEuler321(euler, euler2), RBK.addEuler321(euler, euler2),
            atol=ATOL,
        )

    def testGibbs2c(self, gibbs):
        np.testing.assert_allclose(
            RBKN.gibbs2C(gibbs), RBK.gibbs2C(gibbs), atol=ATOL
        )

    def testGibbs2mrp(self, gibbs):
        np.testing.assert_allclose(
            RBKN.gibbs2MRP(gibbs), RBK.gibbs2MRP(gibbs), atol=ATOL
        )

    def test_MRP2EP_round_trip(self, mrp):
        """MRP -> EP -> MRP round-trip must recover original."""
        ep = RBKN.MRP2EP(mrp)
        recovered = RBKN.EP2MRP(ep)
        np.testing.assert_allclose(recovered, mrp, atol=ATOL)

    def test_PRV2EP_round_trip(self, prv):
        """PRV -> EP -> PRV round-trip."""
        ep = RBKN.PRV2EP(prv)
        recovered = RBKN.EP2PRV(ep)
        np.testing.assert_allclose(recovered, prv, atol=ATOL)

    def test_BmatMRP(self, mrp):
        np.testing.assert_allclose(
            RBKN.BmatMRP(mrp), RBK.BmatMRP(mrp), atol=ATOL
        )

    def test_BinvMRP(self, mrp):
        np.testing.assert_allclose(
            RBKN.BinvMRP(mrp), RBK.BinvMRP(mrp), atol=ATOL
        )

    def test_BinvEP(self, ep):
        np.testing.assert_allclose(
            RBKN.BinvEP(ep), RBK.BinvEP(ep), atol=ATOL
        )

    def test_BmatEP(self, ep):
        np.testing.assert_allclose(
            RBKN.BmatEP(ep), RBK.BmatEP(ep), atol=ATOL
        )

    def testDep(self, ep, omega):
        np.testing.assert_allclose(
            RBKN.dEP(ep, omega), RBK.dEP(ep, omega), atol=ATOL
        )

    def testDmrp(self, mrp, omega):
        np.testing.assert_allclose(
            RBKN.dMRP(mrp, omega), RBK.dMRP(mrp, omega), atol=ATOL
        )

    def testDdmrp(self, mrp, omega):
        domega = np.array([0.001, -0.002, 0.003])
        np.testing.assert_allclose(
            RBKN.ddMRP(mrp, omega, omega, domega),
            RBK.ddMRP(mrp, omega, omega, domega),
            atol=ATOL,
        )

    def test_C2PRV(self, dcm):
        np.testing.assert_allclose(
            RBKN.C2PRV(dcm), RBK.C2PRV(dcm), atol=ATOL
        )

    def test_C2Gibbs(self, dcm):
        np.testing.assert_allclose(
            RBKN.C2Gibbs(dcm), RBK.C2Gibbs(dcm), atol=ATOL
        )

    def test_MRP2Gibbs(self, mrp):
        np.testing.assert_allclose(
            RBKN.MRP2Gibbs(mrp), RBK.MRP2Gibbs(mrp), atol=ATOL
        )

    # --- Euler-angle families (spot-check each sequence) ---

    @pytest.mark.parametrize("seq", [
        "121", "123", "131", "132",
        "212", "213", "231", "232",
        "312", "313", "321", "323",
    ])
    def test_C2Euler(self, dcm, seq):
        fn_rbk  = getattr(RBK,  f"C2Euler{seq}")
        fn_rbkn = getattr(RBKN, f"C2Euler{seq}")
        np.testing.assert_allclose(fn_rbkn(dcm), fn_rbk(dcm), atol=ATOL)

    @pytest.mark.parametrize("seq", [
        "121", "123", "131", "132",
        "212", "213", "231", "232",
        "312", "313", "321", "323",
    ])
    def testAddeuler(self, euler, euler2, seq):
        fn_rbk  = getattr(RBK,  f"addEuler{seq}")
        fn_rbkn = getattr(RBKN, f"addEuler{seq}")
        np.testing.assert_allclose(
            fn_rbkn(euler, euler2), fn_rbk(euler, euler2), atol=ATOL
        )

    @pytest.mark.parametrize("seq", [
        "121", "123", "131", "132",
        "212", "213", "231", "232",
        "312", "313", "321", "323",
    ])
    def test_BinvEuler(self, euler, seq):
        fn_rbk  = getattr(RBK,  f"BinvEuler{seq}")
        fn_rbkn = getattr(RBKN, f"BinvEuler{seq}")
        np.testing.assert_allclose(fn_rbkn(euler), fn_rbk(euler), atol=ATOL)
