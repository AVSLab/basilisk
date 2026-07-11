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

#ifndef svStochasticIntegratorRS_h
#define svStochasticIntegratorRS_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "../_GeneralModuleFiles/extendedStateVector.h"

#include <memory>
#include <vector>

/** Coefficients for the Roessler-Stratonovich weak-order-2 methods RS1 / RS2. Names
 * follow the RS1 / RS2 tableau. */
struct RSCoefficients {
    double a021, /*!< drift-stage A-matrix coefficient */ a031, /*!< drift-stage A-matrix coefficient */ a032, /*!< drift-stage A-matrix coefficient */ a131, /*!< drift-stage A-matrix coefficient */ a141;  //!< drift-stage A-matrix coefficient
    double b031, /*!< first diffusion-stage B-matrix coefficient */ b032, /*!< first diffusion-stage B-matrix coefficient */ b121, /*!< first diffusion-stage B-matrix coefficient */ b131, /*!< first diffusion-stage B-matrix coefficient */ b132, /*!< first diffusion-stage B-matrix coefficient */ b141, /*!< first diffusion-stage B-matrix coefficient */ b142, /*!< first diffusion-stage B-matrix coefficient */ b143;  //!< first diffusion-stage B-matrix coefficient
    double b221, /*!< second diffusion-stage B-matrix coefficient */ b231, /*!< second diffusion-stage B-matrix coefficient */ b331, /*!< second diffusion-stage B-matrix coefficient */ b332, /*!< second diffusion-stage B-matrix coefficient */ b341, /*!< second diffusion-stage B-matrix coefficient */ b342;  //!< second diffusion-stage B-matrix coefficient
    double alpha1, /*!< drift weight vector */ alpha2, /*!< drift weight vector */ alpha3, /*!< drift weight vector */ alpha4;  //!< drift weight vector
    double c02, /*!< stage time node */ c03, /*!< stage time node */ c13, /*!< stage time node */ c14;  //!< stage time node
    double beta11, /*!< diffusion weight vector */ beta12, /*!< diffusion weight vector */ beta13, /*!< diffusion weight vector */ beta14, /*!< diffusion weight vector */ beta22, /*!< diffusion weight vector */ beta23;  //!< diffusion weight vector
};

/**
 * The svStochasticIntegratorRS class implements the Roessler weak second-order
 * Stratonovich methods RS1 and RS2 for SDEs in the Stratonovich interpretation.
 *
 * Implementation of the RS1 / RS2 methods of Roessler (2007). It is the
 * weak-order-2 counterpart, for the Stratonovich interpretation, of the Ito weak
 * methods (DRI1, W2Ito1) - giving weak-order capability parity with the Stratonovich
 * strong method (Euler-Heun). Like DRI1 it handles non-commutative noise, using the
 * mixed iterated integral of Roessler eq. (5.2),
 * \f$\hat I_{(k,l)} = \hat I_{(k)}\tilde I_{(l)}\f$ for \f$l<k\f$ and
 * \f$-\hat I_{(l)}\tilde I_{(k)}\f$ for \f$k<l\f$, with three-point \f$\hat I\f$ and
 * two-point \f$\tilde I\f$ (only \f$2m-1\f$ independent random variables per step).
 *
 * The step is fully explicit and derivative-free (four drift/diffusion stages plus
 * cross-noise stages). See the .cpp for the exact recurrence.
 *
 * @warning Stochastic integration is in beta.
 *
 * @note This integrator implements the method exactly as published in Roessler (2007)
 * and is verified for any number m of noise sources - including multiple, commutative
 * and non-commutative, noise - against a paper-faithful reference that reproduces
 * Roessler's own published weak-error tables (see paperReference/generate_rs_reference.py
 * and test_stochasticIntegratorsPaper.py). It uses vector-valued diffusions b^l and
 * is faithful to the published method; it deliberately does NOT reproduce some in-place
 * reference implementations whose multi-noise cross-terms build their stage states by
 * aliasing (not copying) the working state, and so deviate from the published method for
 * m > 1; only the single-noise case, where the cross-terms vanish, agrees with those.
 */
class svStochasticIntegratorRS : public StochasticRKIntegratorBase {
public:
    /** Constructor taking the dynamic object and the RS coefficient tableau. */
    svStochasticIntegratorRS(DynamicObject* dynIn, const RSCoefficients& coefficients);

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    virtual void integrate(double currentTime, double timeStep) override;

protected:
    /** RS coefficients. */
    const RSCoefficients coefficients;
};

/** @brief RS1: Roessler-Stratonovich weak-order-2 method (deterministic order 2). */
class svStochasticIntegratorRS1 : public svStochasticIntegratorRS {
public:
    /** Constructor. */
    svStochasticIntegratorRS1(DynamicObject* dyn);
private:
    /** Returns the RS1 coefficient tableau. */
    static RSCoefficients getCoefficients();
};

/** @brief RS2: Roessler-Stratonovich weak-order-2 method (deterministic order 3). */
class svStochasticIntegratorRS2 : public svStochasticIntegratorRS {
public:
    /** Constructor. */
    svStochasticIntegratorRS2(DynamicObject* dyn);
private:
    /** Returns the RS2 coefficient tableau. */
    static RSCoefficients getCoefficients();
};

#endif /* svStochasticIntegratorRS_h */
