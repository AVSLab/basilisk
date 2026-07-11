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
#ifndef IGBM_ATM_DENSITY_H
#define IGBM_ATM_DENSITY_H

#include "simulation/mujocoDynamics/_GeneralModuleFiles/inhomogeneousGeometricBrownianMotion.h"

#include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"

/**
 * @class IgbmAtmDensity
 * @brief Applies an inhomogeneous geometric Brownian motion (IGBM) correction to
 *        atmospheric density.
 *
 * Derives from InhomogeneousGeometricBrownianMotion. Scales the incoming neutral density
 * by the IGBM factor \f$x\f$ maintained by the base class (mean-reverting to the level
 * \f$\mu\f$); setting \f$\mu = 1\f$ gives a mean-preserving multiplicative perturbation.
 * The exact IGBM process is positive, but an explicit integrator can occasionally drive
 * \f$x\f$ non-positive for a large step/increment, so the corrected density is clamped to
 * be non-negative (a negative density is unphysical).
 */
class IgbmAtmDensity : public InhomogeneousGeometricBrownianMotion {
public:

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

protected:
    /**
     * @brief Apply the IGBM correction factor and write output.
     *
     * Called automatically by InhomogeneousGeometricBrownianMotion::UpdateState(), after
     * the stochastic state has been updated and before the simulation advances.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     * @param x Current value of the scalar multiplicative correction factor.
     */
    void writeOutput(uint64_t CurrentSimNanos, double x) override;
};


#endif
