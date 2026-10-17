/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef stochasticWeakRandomVariables_h
#define stochasticWeakRandomVariables_h

#include <cmath>

/**
 * Helpers that convert the underlying Gaussian Wiener increments into the discrete
 * random variables used by the weak-order stochastic Runge-Kutta methods (DRI1,
 * DRI1NM, W2Ito1, ...).
 *
 * Using these transforms (rather than storing the discrete variables directly) means
 * the integrators can be driven either by a random Gaussian generator in production
 * or by a prescribed Gaussian generator in tests, and in both cases the resulting
 * discrete variables match the reference implementation exactly.
 *
 * @warning Stochastic integration is in beta.
 */
namespace stochasticWeakRV {

/** The (negative) 1/6 quantile of the standard normal distribution, used as the
 * threshold for the three-point transform. */
constexpr double NORMAL_ONESIX_QUANTILE = -0.9674215661017014;

/** Three-point distributed random variable.
 *
 * Given a Gaussian increment ``dW`` (~ N(0,h)) and the step ``h``, returns a variable
 * taking values in \f$\{-\sqrt{3h},\,0,\,+\sqrt{3h}\}\f$ with probabilities
 * \f$\{1/6,\,2/3,\,1/6\}\f$. The scaled increment ``dW/sqrt(h)`` is thresholded
 * against the 1/6 quantile.
 */
inline double threePoint(double dW, double h)
{
    const double sq3h = std::sqrt(3.0 * h);
    const double dWscaled = dW / std::sqrt(h);
    if (std::abs(dWscaled) > -NORMAL_ONESIX_QUANTILE) {
        return (dWscaled < NORMAL_ONESIX_QUANTILE) ? -sq3h : sq3h;
    }
    return 0.0;
}

/** Two-point distributed random variable.
 *
 * Given a Gaussian increment ``dZ`` and a scale ``s``, returns \f$+s\f$ if
 * ``dZ > 0`` else \f$-s\f$. Uses a strictly-positive sign convention: a strictly
 * positive increment maps to ``+s`` and zero/negative maps to ``-s``.
 */
inline double twoPoint(double dZ, double scale)
{
    return (dZ > 0.0) ? scale : -scale;
}

} // namespace stochasticWeakRV

#endif /* stochasticWeakRandomVariables_h */
