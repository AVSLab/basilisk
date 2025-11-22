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
#ifndef STOCHASTIC_DRAG_COEFF_H
#define STOCHASTIC_DRAG_COEFF_H

#include "meanRevertingNoise.h"
#include "architecture/msgPayloadDefC/DragGeometryMsgPayload.h"

/**
 * @class StochasticDragCoeff
 * @brief Applies a mean-reverting Ornstein–Uhlenbeck correction to the drag coefficient.
 *
 * The scalar state \( x \) evolves as
 * \( dx = -(1/\tau)\,x\,dt + \sqrt{2/\tau}\,\sigma_{\text{st}}\,dW \).
 * The outgoing coefficient is modified as
 * \( C_{D,\text{out}} = C_{D,\text{in}}(1 + x) \).
 *
 * The projected area and \( r_{CP,S} \) are passed through unchanged.
 */
class StochasticDragCoeff : public MeanRevertingNoise {
public:
    /** @name I/O Messages @{ */

    /**
     * @brief Input drag geometry message.
     *
     * Read each step to obtain the nominal geometry and coefficient.
     */
    ReadFunctor<DragGeometryMsgPayload> dragGeomInMsg;

    /**
     * @brief Output drag geometry message with corrected drag coefficient.
     */
    Message<DragGeometryMsgPayload> dragGeomOutMsg;

    /** @} */

protected:
    /**
     * @brief Apply the OU correction to \( C_D \) and write the result.
     *
     * Called by MeanRevertingNoise::UpdateState after drift and diffusion are set.
     *
     * @param CurrentSimNanos Simulation time in nanoseconds.
     * @param x Current scalar correction factor.
     */
    void writeOutput(uint64_t CurrentSimNanos, double x) override;
};

#endif
