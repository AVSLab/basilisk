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
Shared discrete random-variable transforms for the paper-faithful reference generators
(generate_dri1_reference.py, generate_rs_reference.py, generate_w2ito_reference.py).

These mirror the C++ ``stochasticWeakRV::threePoint`` / ``twoPoint`` (see
stochasticWeakRandomVariables.h) so a generator and the integrator it validates share a
single definition of the transforms and the 1/6-quantile threshold. Both functions are
elementwise and accept either a scalar or a NumPy array.
"""
import numpy as np

# (Negative) 1/6 quantile of the standard normal, the three-point threshold. Must match
# stochasticWeakRV::NORMAL_ONESIX_QUANTILE in the C++.
NORMAL_ONESIX_QUANTILE = -0.9674215661017014


def three_point(dW, h):
    """Three-point transform: {-sqrt(3h), 0, +sqrt(3h)} with probs {1/6, 2/3, 1/6},
    thresholding dW/sqrt(h) against the 1/6 quantile. Elementwise; scalar or array."""
    s = dW / np.sqrt(h)
    sq3h = np.sqrt(3.0 * h)
    return np.where(np.abs(s) > -NORMAL_ONESIX_QUANTILE,
                    np.where(s < NORMAL_ONESIX_QUANTILE, -sq3h, sq3h), 0.0)


def two_point(dZ, scale=1.0):
    """Two-point transform: +scale if dZ > 0 else -scale (strictly-positive convention).
    Elementwise; scalar or array. Pass scale=sqrt(h) for the sqrt(h)-scaled variant."""
    return np.where(dZ > 0.0, scale, -scale)
