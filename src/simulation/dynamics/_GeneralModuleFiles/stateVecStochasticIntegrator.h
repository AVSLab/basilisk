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


/** @file stateVecStochasticIntegrator.h
 * @brief Shared noise topology and stochastic candidate construction.
 */

#ifndef stateVecStochasticIntegrator_h
#define stateVecStochasticIntegrator_h

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "../_GeneralModuleFiles/flatStateBinding.h"
#include "../_GeneralModuleFiles/stateVecIntegrator.h"

/** @brief Stochastic methods reuse the same live-buffer metadata as deterministic RK. */
using StochasticObjectDescriptor = FlatObjectDescriptor;
/** @brief Stochastic methods reuse state shapes and offsets from the common binding. */
using StochasticStateDescriptor = FlatStateDescriptor;

/** One state-local diffusion tangent assigned to a global noise slot. */
struct StochasticNoiseBinding
{
    size_t boundStateIndex = 0; ///< State descriptor affected by this endpoint.
    size_t packedOffset = 0;    ///< First tangent scalar in packed global-slot storage.
    size_t scalarCount = 0;     ///< Number of tangent scalars for this endpoint.
    const double* liveData = nullptr; ///< Borrowed tangent pointer in the finalized registry.
};

/** A finalized global noise source and its packed diffusion range. */
struct StochasticNoiseSlot
{
    size_t bindingBegin = 0; ///< First endpoint in the noise-binding vector.
    size_t bindingCount = 0; ///< Endpoints sharing this global process.
    size_t packedBegin = 0;  ///< First scalar of this slot in a packed diffusion buffer.
    size_t packedCount = 0;  ///< Sum of tangent sizes for all endpoints in this slot.
};

/**
 * @brief Flat topology and candidate operations shared by stochastic numerical methods.
 *
 * State layout follows registration order. Global noise slots are assigned by the
 * binding's canonical traversal and group shared endpoints. Packed diffusion buffers
 * follow global-slot order, but updates to each state follow its local source order;
 * special policies may have noncommuting increments.
 *
 * This base owns reusable candidate/rollback storage and mappings between those orders.
 * StochasticRKIntegratorBase adds generator output; concrete methods supply recurrences
 * and stage storage. The protected API serves those methods. Dynamics modules only
 * register states and supply drift/diffusion callbacks. Rollback restores state values,
 * not RNG position or external callback effects.
 */
class StateVecStochasticIntegrator : public StateVecIntegrator
{

public:
    using StateVecIntegrator::StateVecIntegrator;

  protected:
    /** Reject invalid stochastic step sizes before state or RNG mutation. */
    static void validateStochasticTimeStep(double timeStep)
    {
        if (!std::isfinite(timeStep) || timeStep < 0.0) {
            throw std::invalid_argument("Stochastic integration requires a finite, nonnegative time step.");
        }
    }

    /** Lazily builds immutable flat stochastic topology and scratch. */
    void bindStochasticTopology();

    /** Discards an incomplete topology binding after method-scratch failure. */
    void resetStochasticTopologyBinding() noexcept;

    /** Validates that dynamic-object and manager topology remains bound. */
    void validateStochasticTopology() const;

    /** Returns the number of global noise sources in the bound topology. */
    size_t globalNoiseCount() const noexcept { return this->noiseSlots.size(); }

    /** @brief Borrow state metadata in flat registration order. */
    const std::vector<StochasticStateDescriptor>& stochasticStateDescriptors() const noexcept
    {
        return this->flatBinding.states();
    }

    /** @brief Borrow live object buffer descriptors. */
    const std::vector<StochasticObjectDescriptor>& stochasticObjectDescriptors() const noexcept
    {
        return this->flatBinding.objects();
    }

    /** @brief Borrow state-local endpoints grouped by global source. */
    const std::vector<StochasticNoiseBinding>& stochasticNoiseBindings() const noexcept { return this->noiseBindings; }

    /** @brief Borrow independent global sources and their packed ranges. */
    const std::vector<StochasticNoiseSlot>& stochasticNoiseSlots() const noexcept { return this->noiseSlots; }

    /** @brief Borrow local endpoint to global source mappings. */
    const std::vector<size_t>& stochasticLocalNoiseSlots() const noexcept { return this->localNoiseToGlobalSlot; }

    /** @brief Borrow local endpoint to packed tangent mappings. */
    const std::vector<size_t>& stochasticLocalNoisePackedOffsets() const noexcept
    {
        return this->localNoiseToPackedOffset;
    }

    /** Captures the accepted state for rollback. */
    void gatherStochasticStates()
    {
        this->gatherStochasticStates(this->rollbackState);
        this->workingStateIsRollback = true;
    }

    /** Copies live object buffer segments into caller-owned flat storage. */
    void gatherStochasticStates(Eigen::VectorXd& output) const { this->flatBinding.gatherStates(output); }

    /** Copies caller-owned flat storage into live object buffer segments. */
    void scatterStochasticStates(const Eigen::VectorXd& input) const { this->flatBinding.scatterStates(input); }

    /** Restores the accepted state after a failed attempted step. */
    void restoreStochasticStates() const;

    /** Builds and scatters a candidate in descriptor-local noise order. */
    void buildStochasticCandidateInPlace(const Eigen::VectorXd& base,
                                         const Eigen::VectorXd& drift,
                                         double timeStep,
                                         const Eigen::VectorXd& diffusions,
                                         const Eigen::VectorXd& globalPseudoSteps);

    /** Builds and scatters a candidate using only a global noise-slot range. */
    void buildStochasticCandidateInPlace(const Eigen::VectorXd& base,
                                         const Eigen::VectorXd& drift,
                                         double timeStep,
                                         const Eigen::VectorXd& diffusions,
                                         const Eigen::VectorXd& globalPseudoSteps,
                                         size_t slotBegin,
                                         size_t slotEnd);

    /** Applies packed diffusion increments in descriptor-local noise order. */
    void applyStochasticDiffusionUpdateInPlace(const Eigen::VectorXd& diffusions,
                                               const Eigen::VectorXd& globalPseudoSteps);

    /** Applies one packed global noise slot to the live state. */
    void applyStochasticNoiseSlotInPlace(const Eigen::VectorXd& diffusions, size_t slotIndex, double pseudoStep);

    /** Captures the current drift values in flat buffer order. */
    void gatherStochasticDerivatives() { this->gatherStochasticDerivatives(this->packedDerivatives); }

    /** Copies live derivative buffer segments into caller-owned flat storage. */
    void gatherStochasticDerivatives(Eigen::VectorXd& output) const { this->flatBinding.gatherDerivatives(output); }

    /** Copies live drift values into caller-owned contiguous storage. */
    void gatherStochasticDerivatives(double* output, Eigen::Index outputSize) const
    {
        this->flatBinding.gatherDerivatives(output, outputSize);
    }

    /** Captures all diffusion tangents in packed global-slot storage. */
    void gatherStochasticDiffusions() { this->gatherStochasticDiffusions(this->packedDiffusions); }

    /** Copies live diffusion tangents into caller-owned packed storage. */
    void gatherStochasticDiffusions(Eigen::VectorXd& output) const
    {
        if (output.size() != this->packedDiffusions.size()) {
            throw std::invalid_argument("Packed diffusion buffer does not match the bound topology.");
        }
        for (const auto& binding : this->noiseBindings) {
            std::copy_n(
              binding.liveData, binding.scalarCount, output.data() + static_cast<Eigen::Index>(binding.packedOffset));
        }
    }

    /** Applies one flat Euler-Maruyama candidate in local state-noise order. */
    void buildEulerMaruyamaCandidate(double timeStep, const Eigen::VectorXd& globalPseudoSteps);

    /** Updates Euclidean states from captured drift and live diffusion, without packing diffusion or scattering.
     * @note Only valid when every state uses ordinary vector addition. The captured drift remains
     * independent of writes made by the diffusion callback.
     */
    void advanceEuclideanEulerMaruyama(double timeStep, const Eigen::VectorXd& globalPseudoSteps);

    /** Builds and scatters a candidate directly from packed work buffers. */
    void buildStochasticCandidate(const Eigen::VectorXd& base,
                                  const Eigen::VectorXd& drift,
                                  double timeStep,
                                  const Eigen::VectorXd& diffusions,
                                  const Eigen::VectorXd& globalPseudoSteps,
                                  size_t slotBegin,
                                  size_t slotEnd);

    /** Builds a candidate using every global noise slot. */
    void buildStochasticCandidate(const Eigen::VectorXd& base,
                                  const Eigen::VectorXd& drift,
                                  double timeStep,
                                  const Eigen::VectorXd& diffusions,
                                  const Eigen::VectorXd& globalPseudoSteps)
    {
        this->buildStochasticCandidate(
          base, drift, timeStep, diffusions, globalPseudoSteps, 0, this->globalNoiseCount());
    }

    /** Builds and scatters the drift baseline for sparse per-slot candidates. */
    void beginStochasticSparseCandidates(const Eigen::VectorXd& base, const Eigen::VectorXd& drift, double timeStep);

    /** Restores the sparse baseline and scatters one noise-slot candidate. */
    void buildStochasticNoiseSlotCandidate(const Eigen::VectorXd& diffusions, double pseudoStep, size_t slotIndex);

    /** Makes the last packed candidate the base for a subsequent update. */
    void acceptStochasticCandidate();

    /** Whether every bound state uses ordinary vector addition. */
    bool stochasticUpdatesAreAllEuclidean() const noexcept { return this->flatBinding.updatesAreAllEuclidean(); }

    /** Starts an unscattered final candidate for an all-Euclidean topology. */
    void beginAllEuclideanFinalCandidate(const Eigen::VectorXd& base,
                                         const Eigen::VectorXd& drift,
                                         double timeStep,
                                         const Eigen::VectorXd& diffusions,
                                         const Eigen::VectorXd& globalPseudoSteps);

    /** Appends one ordered term to an all-Euclidean final candidate. */
    void appendAllEuclideanFinalCandidate(const Eigen::VectorXd& drift,
                                          double timeStep,
                                          const Eigen::VectorXd& diffusions,
                                          const Eigen::VectorXd& globalPseudoSteps);

    /** Scatters the completed all-Euclidean final candidate once. */
    void commitAllEuclideanFinalCandidate();

    /** @brief Borrow the method's current base; step-entry rollback remains separate. */
    const Eigen::VectorXd& stochasticAcceptedState() const noexcept
    {
        return this->workingStateIsRollback ? this->rollbackState : this->workingState;
    }

    /** @brief Borrow the most recently captured flat drift. */
    const Eigen::VectorXd& stochasticPackedDerivatives() const noexcept { return this->packedDerivatives; }

    /** @brief Borrow the most recently captured packed tangents. */
    const Eigen::VectorXd& stochasticPackedDiffusions() const noexcept { return this->packedDiffusions; }

    /** Builds one policy-aware drift candidate. */
    void buildStochasticDriftCandidate(const StochasticStateDescriptor& descriptor,
                                       ConstMatrixView base,
                                       ConstMatrixView drift,
                                       double timeStep,
                                       MutableMatrixView output);

    /** Applies one policy-aware diffusion increment. */
    void applyStochasticNoiseIncrement(const StochasticStateDescriptor& descriptor,
                                       MutableMatrixView state,
                                       ConstMatrixView diffusion,
                                       double pseudoStep);

    /** Applies selected global sources in the descriptor's local noise order. */
    void applyStochasticNoiseInLocalOrder(const StochasticStateDescriptor& descriptor,
                                          MutableMatrixView state,
                                          const Eigen::VectorXd& diffusions,
                                          const Eigen::VectorXd& globalPseudoSteps,
                                          size_t slotBegin,
                                          size_t slotEnd);

  private:
    /** @brief Which candidate operations are valid for the reusable scratch. */
    enum class CandidateAssemblyPhase
    {
        Idle,          ///< No complete candidate is available for acceptance.
        Ready,         ///< A full candidate can become the next working base.
        Sparse,        ///< Per-slot trials share sparseBaselineState.
        EuclideanFinal ///< Ordered terms are accumulating before one final scatter.
    };

    /** @brief Fill candidateState without scattering, preserving each state's local noise order. */
    void writeStochasticCandidate(const Eigen::VectorXd& base,
                                  const Eigen::VectorXd& drift,
                                  double timeStep,
                                  const Eigen::VectorXd& diffusions,
                                  const Eigen::VectorXd& globalPseudoSteps,
                                  size_t slotBegin,
                                  size_t slotEnd);

    /** @brief Check packed tangent and pseudo-step sizes against the binding. */
    bool diffusionInputsMatchTopology(const Eigen::VectorXd& diffusions,
                                      const Eigen::VectorXd& globalPseudoSteps) const;

    FlatStateBinding flatBinding; ///< Owns descriptors and borrows finalized buffers/policies.
    std::vector<StochasticNoiseBinding> noiseBindings; ///< Endpoints grouped by global source.
    std::vector<StochasticNoiseSlot> noiseSlots; ///< One packed range per independent global process.
    std::vector<size_t> localNoiseToGlobalSlot; ///< Local endpoint -> shared/global process index.
    std::vector<size_t> localNoiseToPackedOffset; ///< Local endpoint -> packed tangent offset.
    std::vector<const double*> localNoiseLiveData; ///< Local endpoint -> live tangent for direct updates.
    Eigen::VectorXd rollbackState; ///< State at attempted-step entry, preserved until the step completes.
    Eigen::VectorXd workingState; ///< Accepted intermediate candidate when a method advances its own base.
    Eigen::VectorXd candidateState; ///< Reusable destination for complete, sparse, or final assembly.
    Eigen::VectorXd sparseBaselineState; ///< Drift-only baseline reused by per-source trials.
    Eigen::VectorXd packedDerivatives; ///< Captured drift in flat registration order.
    Eigen::VectorXd packedDiffusions; ///< Captured tangents in global-slot order.
    bool workingStateIsRollback = true; ///< Selects rollbackState until an intermediate candidate is accepted.
    size_t activeSparseSlot = static_cast<size_t>(-1); ///< Previous perturbed scratch slot; size_t(-1) means none.
    CandidateAssemblyPhase candidatePhase = CandidateAssemblyPhase::Idle; ///< Validity of candidateState.
};

#endif /* stateVecStochasticIntegrator_h */
