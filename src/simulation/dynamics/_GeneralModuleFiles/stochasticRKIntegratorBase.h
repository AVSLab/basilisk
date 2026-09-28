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
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "../_GeneralModuleFiles/stateVecStochasticIntegrator.h"
#include "../_GeneralModuleFiles/extendedStateVector.h"
#include "../_GeneralModuleFiles/stochasticNoiseGenerator.h"

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <unordered_map>
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
 *    / ``computeDiffusions`` (call the dynamic objects'
 *    ``equationsOfMotion`` / ``equationsOfMotionDiffusion``, and gather the result);
 *  - cached noise-index maps and per-state increment buffers, refreshed by
 *    ``noiseIndexMaps()`` at the start of each nonzero integration step and reused
 *    by ``propagateStateWithCachedNoise()`` at each stage.
 *
 * State registration, object membership, noise counts, and shared-noise mappings
 * may change between integration calls but must remain stable during a call.
 * Routing buffers allocate on first use and after topology changes. Strong methods
 * also reuse owned state, drift, and sparse diffusion stage buffers. Matrices resize
 * on assignment after dimension changes; their dimensions need not match each other.
 * Propagation uses the virtual StateData interface. The by-value propagation
 * argument, noise generator, and model callbacks can still allocate on each call.
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
    /** Owned matrices in state order, or in the sparse order for one noise source. */
    using StateBuffer = std::vector<Eigen::MatrixXd>;
    /** One sparse state buffer per global noise source. */
    using DiffusionBuffer = std::vector<StateBuffer>;

    /** @brief Prepare reusable stage storage after refreshing the noise routing.
     * @param derivativeCount Number of derivative snapshots needed by the method.
     * @param diffusionCount Number of diffusion snapshots needed by the method.
     * @param snapshotCount Number of state snapshots needed by the method.
     * @param noiseVectorCount Number of per-source scratch vectors needed by the method.
     * @return Number of global noise sources.
     * @note Values are captured separately. Matrices resize on assignment, so state,
     * derivative, and diffusion dimensions may differ and may change between calls.
     */
    size_t prepareStageBuffers(size_t derivativeCount, size_t diffusionCount,
                               size_t snapshotCount, size_t noiseVectorCount);

    /** @brief Capture the current states in an owned snapshot.
     * @param snapshot Index of the prepared state snapshot.
     */
    void captureStates(size_t snapshot);
    /** @brief Restore every state before applying derivatives or diffusions.
     * @param snapshot Index of the state snapshot to restore.
     */
    void restoreStates(size_t snapshot);
    /** @brief Evaluate all drift callbacks and capture the derivatives.
     * @param time Evaluation time in seconds.
     * @param timeStep Integration step in seconds.
     * @param stage Index of the derivative snapshot to fill.
     */
    void evaluateStageDerivatives(double time, double timeStep, size_t stage);
    /** @brief Evaluate all diffusion callbacks and capture every source.
     * @param time Evaluation time in seconds.
     * @param timeStep Integration step in seconds.
     * @param stage Index of the diffusion snapshot to fill.
     */
    void evaluateStageDiffusions(double time, double timeStep, size_t stage);
    /** @brief Evaluate all diffusion callbacks and capture only one source.
     * @param time Evaluation time in seconds.
     * @param timeStep Integration step in seconds.
     * @param source Global noise-source index.
     * @param stage Index of the diffusion snapshot to fill.
     */
    void evaluateStageDiffusion(double time, double timeStep, size_t source, size_t stage);
    /** @brief Publish a derivative snapshot through the virtual state setters.
     * @param stage Index of the derivative snapshot to publish.
     */
    void applyStageDerivatives(size_t stage);
    /** @brief Publish a diffusion snapshot for one source.
     * @param source Global noise-source index.
     * @param stage Index of the diffusion snapshot to publish.
     */
    void applyStageDiffusion(size_t source, size_t stage);
    /** @brief Form and publish a weighted derivative sum in reusable storage.
     * @param weights Array containing at least length coefficients.
     * @param length Number of stages to sum, greater than zero.
     * @note Matches scaledSum(): multiply the first term even when its weight is
     * zero; skip subsequent zero weights and accumulate the other terms in order.
     */
    void applyDerivativeSum(const double* weights, size_t length);
    /** @brief Form and publish a weighted diffusion sum for one source.
     * @param source Global noise-source index.
     * @param weights Array containing at least length coefficients.
     * @param length Number of stages to sum, greater than zero.
     * @note Uses the same arithmetic ordering and zero-weight behavior as applyDerivativeSum().
     */
    void applyDiffusionSum(size_t source, const double* weights, size_t length);

    std::vector<StateBuffer> stateSnapshots;      //!< Owned state snapshots for stage restoration.
    std::vector<StateBuffer> derivativeStages;    //!< Owned drift snapshots, indexed by stage.
    std::vector<DiffusionBuffer> diffusionStages; //!< Owned sparse diffusion snapshots, indexed by stage.
    std::vector<Eigen::VectorXd> noiseBuffers;    //!< Reused per-source pseudo-steps and iterated integrals.

    /** @brief Refresh routing at the integration boundary and return the source maps.
     * @return Source maps, valid until the next routing rebuild or integrator destruction.
     * @note Call once before evaluating a nonzero integration step. State/noise
     * topology must remain stable until the step finishes.
     */
    const std::vector<StateIdToIndexMap>& noiseIndexMaps();

    /** @brief Propagate using the routing prepared by noiseIndexMaps().
     * @param timeStep Drift time step in seconds; zero still applies noise increments.
     * @param pseudoTimeSteps Increment for each global source, in source-map order.
     * @note Preserves state order and virtual propagation. The public map-taking
     * StateVecStochasticIntegrator::propagateState() remains available for arbitrary maps.
     */
    void propagateStateWithCachedNoise(double timeStep, const Eigen::VectorXd& pseudoTimeSteps);

    /** @brief Evaluate Euler drift, retaining virtual setter writeback for derived states.
     * @param time Evaluation time in seconds.
     * @param timeStep Integration step in seconds.
     * @note Requires noiseIndexMaps() to have prepared the current state layout.
     */
    void computeEulerDerivatives(double time, double timeStep);

    /** Computes f at the current state/time by calling equationsOfMotion(). */
    ExtendedStateVector computeDerivatives(double time, double timeStep);

    /** Computes g for a single noise source at the current state/time. */
    ExtendedStateVector computeDiffusion(double time, double timeStep,
                                         const StateIdToIndexMap& stateIdToNoiseIndexMap);

    /** Computes g for every noise source at the current state/time. */
    std::vector<ExtendedStateVector>
    computeDiffusions(double time, double timeStep,
                      const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps);

private:
    /** @brief One state/local-channel pair affected by a global noise source. */
    struct DiffusionTarget {
        StateData* state; //!< Owner of the diffusion matrix.
        size_t localIndex; //!< Local noise channel.
    };
    /** @brief Copy the current diffusion of a source into its stage snapshot.
     * @param source Global noise-source index.
     * @param stage Index of the snapshot to fill.
     */
    void captureStageDiffusion(size_t source, size_t stage);

    std::vector<std::vector<DiffusionTarget>> diffusionTargets; //!< Sparse source/state associations.
    StateBuffer derivativeSum;  //!< Weighted drift accumulator.
    StateBuffer derivativeTerm; //!< Separate weighted term to preserve multiply-then-add ordering.
    DiffusionBuffer diffusionSum;  //!< Weighted diffusion accumulators.
    DiffusionBuffer diffusionTerm; //!< Separate weighted diffusion terms.
    bool stageBuffersValid = false; //!< Stage associations match the current noise routing.

    /** @brief Routing and reusable increments for one state, in propagation order. */
    struct StateNoiseRouting {
        StateData* state;                  //!< State owner of the local noise channels.
        std::string name;                  //!< Registration key used to detect renaming.
        std::vector<size_t> sourceIndices; //!< Global source for each local channel.
        std::vector<double> increments;    //!< Reused local pseudo-time steps.
    };

    /** @brief Object metadata used to detect changes without dereferencing old state pointers. */
    struct ObjectNoiseLayout {
        DynamicObject* object;                        //!< Object at this position in dynPtrs.
        size_t stateCount;                            //!< Number of registered states.
        std::unordered_map<std::pair<std::string, size_t>, size_t> sharedNoiseMap; //!< Snapshot of source sharing.
    };

    /** @brief Check the live registration against the prepared routing. */
    bool noiseLayoutMatches() const;

    /** @brief Rebuild source maps, state routing, and local increment buffers together. */
    void rebuildNoiseRouting();

    /** Cached noise-index maps (empty until first noiseIndexMaps() call). */
    std::vector<StateIdToIndexMap> cachedNoiseIndexMaps;
    std::vector<StateNoiseRouting> stateNoiseRouting; //!< Ordered routing for all states.
    std::vector<ObjectNoiseLayout> objectNoiseLayouts; //!< Object and sharing snapshots.
    bool noiseIndexMapsCached = false;
};

#endif /* stochasticRKIntegratorBase_h */
