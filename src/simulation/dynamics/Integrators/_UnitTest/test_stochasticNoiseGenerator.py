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
"""
Focused unit tests for the pluggable Gaussian noise generators used by the native
stochastic integrators (``stochasticNoiseGenerator.h``). These exercise the public
behaviors directly, rather than only through an integrator's happy path:

- ``PrescribedGaussianNoiseGenerator``: FIFO replay, ``remaining()``/``clear()``,
  omitted-``dZ`` zero-filling, and the error paths (queue exhaustion, mismatched ``dW``
  length, undersized explicit ``dZ``).
- ``RandomGaussianNoiseGenerator``: seed repeatability and the ``sqrt(h)`` scaling.
"""
import numpy as np
import numpy.testing as npt
import pytest

from Basilisk.simulation import svIntegrators


def _flat(vec):
    """svIntegrators returns Eigen column vectors as a list of 1-element lists."""
    return np.asarray(vec, dtype=float).reshape(-1)


def test_prescribed_replaysInFifoOrderWithRemaining():
    gen = svIntegrators.PrescribedGaussianNoiseGenerator()
    assert gen.remaining() == 0
    gen.pushStep([0.1, 0.2], [0.3, 0.4])
    gen.pushStep([1.1, 1.2], [1.3, 1.4])
    assert gen.remaining() == 2

    s0 = gen.generate(2, 0.01)
    npt.assert_allclose(_flat(s0.dW), [0.1, 0.2])
    npt.assert_allclose(_flat(s0.dZ), [0.3, 0.4])
    assert gen.remaining() == 1

    s1 = gen.generate(2, 0.01)
    npt.assert_allclose(_flat(s1.dW), [1.1, 1.2])
    npt.assert_allclose(_flat(s1.dZ), [1.3, 1.4])
    assert gen.remaining() == 0


def test_prescribed_omittedDzZeroFills():
    gen = svIntegrators.PrescribedGaussianNoiseGenerator()
    gen.pushStep([1.0, 2.0])  # dZ omitted
    s = gen.generate(2, 0.01)
    npt.assert_allclose(_flat(s.dW), [1.0, 2.0])
    npt.assert_allclose(_flat(s.dZ), [0.0, 0.0])  # zero-filled to m


def test_prescribed_clear():
    gen = svIntegrators.PrescribedGaussianNoiseGenerator()
    gen.pushStep([1.0])
    gen.pushStep([2.0])
    assert gen.remaining() == 2
    gen.clear()
    assert gen.remaining() == 0


def test_prescribed_queueExhaustionRaises():
    gen = svIntegrators.PrescribedGaussianNoiseGenerator()
    gen.pushStep([1.0])
    gen.generate(1, 0.01)
    # Popping past the end raises rather than crashing the interpreter.
    with pytest.raises(RuntimeError):
        gen.generate(1, 0.01)


def test_prescribed_mismatchedDwLengthRaises():
    gen = svIntegrators.PrescribedGaussianNoiseGenerator()
    gen.pushStep([1.0, 2.0])  # 2 sources queued
    with pytest.raises(RuntimeError):
        gen.generate(1, 0.01)  # integrator asks for 1


def test_prescribed_undersizedDzRaises():
    gen = svIntegrators.PrescribedGaussianNoiseGenerator()
    gen.pushStep([1.0, 2.0], [0.5])  # explicit dZ shorter than m = 2
    with pytest.raises(RuntimeError):
        gen.generate(2, 0.01)


def test_random_seedRepeatable():
    a = svIntegrators.RandomGaussianNoiseGenerator()
    b = svIntegrators.RandomGaussianNoiseGenerator()
    a.setSeed(12345)
    b.setSeed(12345)
    sa = a.generate(3, 0.04)
    sb = b.generate(3, 0.04)
    # Same seed => identical dW and dZ streams.
    npt.assert_array_equal(_flat(sa.dW), _flat(sb.dW))
    npt.assert_array_equal(_flat(sa.dZ), _flat(sb.dZ))
    # Different seed => different draw (overwhelmingly likely).
    b.setSeed(54321)
    sc = b.generate(3, 0.04)
    assert not np.allclose(_flat(sa.dW), _flat(sc.dW))


def test_random_incrementsScaleWithSqrtH():
    """Each increment is drawn as sqrt(h)*N(0,1); with a fixed seed, halving h scales the
    draw by sqrt(1/2) exactly (same underlying standard-normal sequence)."""
    g = svIntegrators.RandomGaussianNoiseGenerator()
    g.setSeed(7)
    big = _flat(g.generate(4, 1.0).dW)
    g.setSeed(7)
    small = _flat(g.generate(4, 0.25).dW)  # sqrt(0.25) = 0.5
    npt.assert_allclose(small, 0.5 * big, rtol=1e-12)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
