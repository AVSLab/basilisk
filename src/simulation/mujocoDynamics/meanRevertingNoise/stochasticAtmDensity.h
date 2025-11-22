/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#ifndef STOCHASTIC_ATM_DENSITY_H
#define STOCHASTIC_ATM_DENSITY_H

#include "meanRevertingNoise.h"

#include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"

/**
 * @class StochasticAtmDensity
 * @brief Applies a mean-reverting (Ornstein–Uhlenbeck) correction to atmospheric density.
 *
 * This class derives from MeanRevertingNoise, which provides the scalar
 * OU state \f$x\f$ evolving as:
 * \f[
 *   dx = -\frac{1}{\tau}\, x \, dt + \sqrt{\frac{2}{\tau}}\, \sigma_{\text{st}}\, dW
 * \f]
 *
 * The corrected density is:
 * \f[
 *   \rho_\text{out} = \rho_\text{in}\,(1 + x)
 * \f]
 *
 * The state \f$x\f$ is stored and propagated by the base class.
 * This class only specifies how the state modifies the atmosphere message.
 */
class StochasticAtmDensity : public MeanRevertingNoise {
public:

    /** @name I/O Messages @{ */

    /**
     * @brief Input atmospheric properties message.
     *
     * This message is read each step to obtain the unperturbed atmospheric density
     * and temperature before applying the stochastic correction.
     */
    ReadFunctor<AtmoPropsMsgPayload> atmoDensInMsg;

    /**
     * @brief Output atmospheric properties message.
     *
     * This message is written each step with the corrected neutral density value.
     */
    Message<AtmoPropsMsgPayload> atmoDensOutMsg;

    /** @} */

protected:
    /**
     * @brief Apply the OU correction factor and write output.
     *
     * Called automatically by MeanRevertingNoise::UpdateState(), after
     * the stochastic state has been updated and before the simulation advances.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     * @param x Current value of the scalar mean-reverting correction factor.
     */
    void writeOutput(uint64_t CurrentSimNanos, double x) override;
};


#endif
