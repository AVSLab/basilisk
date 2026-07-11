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

#ifndef stochasticRKIntegratorBase_h
#define stochasticRKIntegratorBase_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/stateVecStochasticIntegrator.h"
#include "../_GeneralModuleFiles/extendedStateVector.h"
#include "../_GeneralModuleFiles/stochasticNoiseGenerator.h"

#include <memory>
#include <vector>

/**
 * Shared base for the native stochastic Runge-Kutta integrators (Euler-Maruyama, the
 * SRI/SRA strong methods, the weak W2Ito/DRI1/RI/RS/SIESME families, RDI1WM and
 * Euler-Heun/RKMil).
 *
 * It factors out the machinery every one of these integrators needs, so each concrete
 * method only has to implement its own ``integrate()`` step recurrence:
 *
 *  - a pluggable ``GaussianNoiseGenerator`` (defaulting to a random Mersenne-Twister
 *    source, replaceable by a prescribed-replay generator for tests), plus the
 *    ``setRNGSeed`` / ``setNoiseGenerator`` accessors;
 *  - the pointwise stage-evaluation helpers ``computeDerivatives`` / ``computeDiffusion``
 *    / ``computeDiffusions`` (set the stage state, call the dynamic objects'
 *    ``equationsOfMotion`` / ``equationsOfMotionDiffusion``, and gather the result);
 *  - a cached view of ``getStateIdToNoiseIndexMaps()`` (``noiseIndexMaps()``), whose
 *    state/noise topology is invariant during a run, so it is built once instead of
 *    every step.
 *
 * Every native stochastic integrator derives from this base. Methods that need only the
 * Wiener increment (Euler-Maruyama, Euler-Heun, RKMil) simply ignore the second increment
 * ``dZ`` the generator also draws.
 *
 * @warning Stochastic integration is in beta.
 */
class StochasticRKIntegratorBase : public StateVecStochasticIntegrator {
public:
    using StateVecStochasticIntegrator::StateVecStochasticIntegrator;

    /** Sets the seed for the (default) Random Number Generator used by this integrator.
     *
     * As a stochastic integrator, random numbers are drawn during each time step. By
     * default a randomly generated seed is used. Setting the seed makes the integrator
     * draw the same sequence each run. Has no effect if a custom noise generator that
     * does not honour the seed was installed via ``setNoiseGenerator``. */
    inline void setRNGSeed(size_t seed) { this->rvGenerator->setSeed(seed); }

    /** Replaces the noise generator used by this integrator. This is primarily useful for
     * testing, where a ``PrescribedGaussianNoiseGenerator`` can be installed so the
     * integrator replays a known sequence of Wiener increments. */
    inline void setNoiseGenerator(std::shared_ptr<GaussianNoiseGenerator> generator)
    {
        this->rvGenerator = std::move(generator);
    }

public:
    /** Random Number Generator for the integrator (supplies dW and dZ per noise source). */
    std::shared_ptr<GaussianNoiseGenerator> rvGenerator =
        std::make_shared<RandomGaussianNoiseGenerator>();

protected:
    /** Returns the (cached) state-id -> noise-index maps. The topology is fixed for the
     * run, so it is computed once on first use and reused thereafter. */
    const std::vector<StateIdToIndexMap>& noiseIndexMaps();

    /** Computes f at the current state/time (sets states, calls equationsOfMotion). */
    ExtendedStateVector computeDerivatives(double time, double timeStep);

    /** Computes g for a single noise source at the current state/time. */
    ExtendedStateVector computeDiffusion(double time, double timeStep,
                                         const StateIdToIndexMap& stateIdToNoiseIndexMap);

    /** Computes g for every noise source at the current state/time. */
    std::vector<ExtendedStateVector>
    computeDiffusions(double time, double timeStep,
                      const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps);

private:
    /** Cached noise-index maps (empty until first noiseIndexMaps() call). */
    std::vector<StateIdToIndexMap> cachedNoiseIndexMaps;
    bool noiseIndexMapsCached = false;
};

#endif /* stochasticRKIntegratorBase_h */
