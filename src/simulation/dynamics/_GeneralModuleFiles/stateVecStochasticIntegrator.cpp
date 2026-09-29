#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include "dynParamManager.h"
#include "dynamicObject.h"
#include "stateData.h"
#include "stateVecStochasticIntegrator.h"
#include "stateRegistry.h"

namespace {
struct PendingNoiseBinding
{
    size_t boundStateIndex;
    size_t localNoiseIndex;
    size_t scalarCount;
    const double* liveData;
};

size_t
checkedPackedAdd(size_t left, size_t right)
{
    const size_t eigenLimit = static_cast<size_t>(std::numeric_limits<Eigen::Index>::max());
    if (right > eigenLimit || left > eigenLimit - right) {
        throw std::overflow_error("Packed stochastic diffusion size exceeds Eigen::Index.");
    }
    return left + right;
}
}

void
StateVecStochasticIntegrator::bindStochasticTopology()
{
    if (this->flatBinding.isBound() && this->flatBinding.objects().size() == this->dynamics().size()) {
        this->validateStochasticTopology();
        return;
    }

    FlatStateBinding newBinding;
    newBinding.bind(this->dynamics(), FlatStateBinding::Mode::Stochastic);
    const auto& newObjects = newBinding.objects();
    const auto& newStates = newBinding.states();
    std::vector<std::vector<size_t>> sharedGroupsByState;
    sharedGroupsByState.resize(newStates.size());

    for (const auto& object : newObjects) {
        StateRegistry& registry = *object.registry;
        const auto& layouts = registry.getStateLayouts();
        const size_t noSharedGroup = std::numeric_limits<size_t>::max();
        for (size_t registrationIndex = 0; registrationIndex < layouts.size(); ++registrationIndex) {
            sharedGroupsByState[object.firstStateIndex + registrationIndex].assign(
              layouts[registrationIndex].noiseCount, noSharedGroup);
        }
        const auto& sharedTopology = registry.getSharedNoiseTopology();
        for (size_t groupIndex = 0; groupIndex < sharedTopology.size(); ++groupIndex) {
            for (const auto& endpoint : sharedTopology[groupIndex]) {
                sharedGroupsByState.at(object.firstStateIndex + endpoint.first).at(endpoint.second) = groupIndex;
            }
        }
    }

    std::vector<std::vector<PendingNoiseBinding>> pendingSlots;
    std::map<std::pair<size_t, size_t>, size_t> sharedSlots;
    const size_t localNoiseCount = newBinding.localNoiseCount();
    std::vector<size_t> newLocalToGlobal(localNoiseCount);

    for (size_t stateIndex : newBinding.noiseTraversalOrder()) {
        const auto& descriptor = newStates[stateIndex];
        const auto& stateSharedGroups = sharedGroupsByState.at(stateIndex);
        const size_t noSharedGroup = std::numeric_limits<size_t>::max();

        for (size_t localNoiseIndex = 0; localNoiseIndex < descriptor.noiseCount; ++localNoiseIndex) {
            const size_t sharedGroup = stateSharedGroups.at(localNoiseIndex);
            size_t globalSlot = 0;

            if (sharedGroup == noSharedGroup) {
                globalSlot = pendingSlots.size();
                pendingSlots.emplace_back();
            } else {
                const auto sharedKey = std::make_pair(descriptor.dynamicObjectIndex, sharedGroup);
                const auto existingSlot = sharedSlots.find(sharedKey);
                if (existingSlot == sharedSlots.end()) {
                    globalSlot = pendingSlots.size();
                    pendingSlots.emplace_back();
                    sharedSlots.emplace(sharedKey, globalSlot);
                } else {
                    globalSlot = existingSlot->second;
                }
            }

            auto& slotBindings = pendingSlots.at(globalSlot);
            for (const auto& binding : slotBindings) {
                if (binding.boundStateIndex == stateIndex) {
                    throw std::logic_error("A shared-noise group maps two local noise indices of state '" +
                                           descriptor.stateName + "' to one global source.");
                }
            }

            slotBindings.push_back({ stateIndex,
                                     localNoiseIndex,
                                     descriptor.diffusionCount,
                                     descriptor.state->diffusionData(localNoiseIndex) });
            newLocalToGlobal.at(descriptor.localNoiseOffset + localNoiseIndex) = globalSlot;
        }
    }

    std::vector<StochasticNoiseBinding> newBindings;
    std::vector<StochasticNoiseSlot> newSlots;
    std::vector<size_t> newLocalToPacked(localNoiseCount);
    std::vector<const double*> newLocalLiveData(localNoiseCount);
    newBindings.reserve(localNoiseCount);
    newSlots.reserve(pendingSlots.size());
    size_t packedDiffusionCount = 0;
    for (const auto& pendingSlot : pendingSlots) {
        StochasticNoiseSlot slot;
        slot.bindingBegin = newBindings.size();
        slot.bindingCount = pendingSlot.size();
        slot.packedBegin = packedDiffusionCount;
        for (const auto& pending : pendingSlot) {
            StochasticNoiseBinding binding;
            binding.boundStateIndex = pending.boundStateIndex;
            binding.packedOffset = packedDiffusionCount;
            binding.scalarCount = pending.scalarCount;
            binding.liveData = pending.liveData;
            newLocalToPacked.at(newStates[pending.boundStateIndex].localNoiseOffset + pending.localNoiseIndex) =
              packedDiffusionCount;
            newLocalLiveData.at(newStates[pending.boundStateIndex].localNoiseOffset + pending.localNoiseIndex) =
              pending.liveData;
            packedDiffusionCount = checkedPackedAdd(packedDiffusionCount, pending.scalarCount);
            newBindings.push_back(binding);
        }
        slot.packedCount = packedDiffusionCount - slot.packedBegin;
        newSlots.push_back(slot);
    }

    const size_t stateCount = newBinding.stateScalarCount();
    const size_t derivativeCount = newBinding.derivativeScalarCount();
    Eigen::VectorXd newRollbackState(static_cast<Eigen::Index>(stateCount));
    Eigen::VectorXd newWorkingState(static_cast<Eigen::Index>(stateCount));
    Eigen::VectorXd newCandidateState(static_cast<Eigen::Index>(stateCount));
    Eigen::VectorXd newSparseBaselineState(static_cast<Eigen::Index>(stateCount));
    Eigen::VectorXd newPackedDerivatives(static_cast<Eigen::Index>(derivativeCount));
    Eigen::VectorXd newPackedDiffusions(static_cast<Eigen::Index>(packedDiffusionCount));

    this->flatBinding = std::move(newBinding);
    this->noiseBindings = std::move(newBindings);
    this->noiseSlots = std::move(newSlots);
    this->localNoiseToGlobalSlot = std::move(newLocalToGlobal);
    this->localNoiseToPackedOffset = std::move(newLocalToPacked);
    this->localNoiseLiveData = std::move(newLocalLiveData);
    this->rollbackState.swap(newRollbackState);
    this->workingState.swap(newWorkingState);
    this->candidateState.swap(newCandidateState);
    this->sparseBaselineState.swap(newSparseBaselineState);
    this->packedDerivatives.swap(newPackedDerivatives);
    this->packedDiffusions.swap(newPackedDiffusions);
    this->workingStateIsRollback = true;
    this->activeSparseSlot = static_cast<size_t>(-1);
    this->candidatePhase = CandidateAssemblyPhase::Idle;
}

void
StateVecStochasticIntegrator::resetStochasticTopologyBinding() noexcept
{
    this->flatBinding.reset();
    std::vector<StochasticNoiseBinding>().swap(this->noiseBindings);
    std::vector<StochasticNoiseSlot>().swap(this->noiseSlots);
    std::vector<size_t>().swap(this->localNoiseToGlobalSlot);
    std::vector<size_t>().swap(this->localNoiseToPackedOffset);
    std::vector<const double*>().swap(this->localNoiseLiveData);
    Eigen::VectorXd().swap(this->rollbackState);
    Eigen::VectorXd().swap(this->workingState);
    Eigen::VectorXd().swap(this->candidateState);
    Eigen::VectorXd().swap(this->sparseBaselineState);
    Eigen::VectorXd().swap(this->packedDerivatives);
    Eigen::VectorXd().swap(this->packedDiffusions);
    this->workingStateIsRollback = true;
    this->activeSparseSlot = static_cast<size_t>(-1);
    this->candidatePhase = CandidateAssemblyPhase::Idle;
}

void
StateVecStochasticIntegrator::validateStochasticTopology() const
{
    this->flatBinding.validate(this->dynamics());
}

void
StateVecStochasticIntegrator::restoreStochasticStates() const
{
    this->scatterStochasticStates(this->rollbackState);
}

void
StateVecStochasticIntegrator::buildStochasticCandidateInPlace(const Eigen::VectorXd& base,
                                                              const Eigen::VectorXd& drift,
                                                              double timeStep,
                                                              const Eigen::VectorXd& diffusions,
                                                              const Eigen::VectorXd& globalPseudoSteps)
{
    this->buildStochasticCandidateInPlace(
      base, drift, timeStep, diffusions, globalPseudoSteps, 0, this->globalNoiseCount());
}

void
StateVecStochasticIntegrator::buildStochasticCandidateInPlace(const Eigen::VectorXd& base,
                                                              const Eigen::VectorXd& drift,
                                                              double timeStep,
                                                              const Eigen::VectorXd& diffusions,
                                                              const Eigen::VectorXd& globalPseudoSteps,
                                                              size_t slotBegin,
                                                              size_t slotEnd)
{
    if (base.size() != this->rollbackState.size() || drift.size() != this->packedDerivatives.size() ||
        !this->diffusionInputsMatchTopology(diffusions, globalPseudoSteps) || slotBegin > slotEnd ||
        slotEnd > this->noiseSlots.size()) {
        throw std::invalid_argument("In-place stochastic candidate does not match the bound topology.");
    }

    const bool singleSparseSlot = slotEnd == slotBegin + 1 && (slotBegin != 0 || slotEnd != this->noiseSlots.size());
    for (const auto& descriptor : this->flatBinding.states()) {
        auto state = descriptor.state->stateView();
        this->buildStochasticDriftCandidate(
          descriptor, descriptor.stateView(base), descriptor.derivativeView(drift), timeStep, state);

        if (singleSparseSlot) {
            continue;
        }
        this->applyStochasticNoiseInLocalOrder(descriptor, state, diffusions, globalPseudoSteps, slotBegin, slotEnd);
    }

    if (singleSparseSlot) {
        this->applyStochasticNoiseSlotInPlace(
          diffusions, slotBegin, globalPseudoSteps(static_cast<Eigen::Index>(slotBegin)));
    }
}

void
StateVecStochasticIntegrator::applyStochasticDiffusionUpdateInPlace(const Eigen::VectorXd& diffusions,
                                                                    const Eigen::VectorXd& globalPseudoSteps)
{
    if (!this->diffusionInputsMatchTopology(diffusions, globalPseudoSteps)) {
        throw std::invalid_argument("In-place stochastic diffusion update does not match the bound topology.");
    }

    for (const auto& descriptor : this->flatBinding.states()) {
        auto state = descriptor.state->stateView();
        this->applyStochasticNoiseInLocalOrder(
          descriptor, state, diffusions, globalPseudoSteps, 0, this->noiseSlots.size());
    }
}

void
StateVecStochasticIntegrator::applyStochasticNoiseSlotInPlace(const Eigen::VectorXd& diffusions,
                                                              size_t slotIndex,
                                                              double pseudoStep)
{
    if (diffusions.size() != this->packedDiffusions.size() || slotIndex >= this->noiseSlots.size()) {
        throw std::invalid_argument("In-place stochastic noise slot does not match the bound topology.");
    }

    const auto& slot = this->noiseSlots[slotIndex];
    const size_t bindingEnd = slot.bindingBegin + slot.bindingCount;
    for (size_t bindingIndex = slot.bindingBegin; bindingIndex < bindingEnd; ++bindingIndex) {
        const auto& binding = this->noiseBindings[bindingIndex];
        const auto& descriptor = this->flatBinding.states()[binding.boundStateIndex];
        auto state = descriptor.state->stateView();
        this->applyStochasticNoiseIncrement(
          descriptor, state, descriptor.diffusionView(diffusions, binding.packedOffset), pseudoStep);
    }
}

void
StateVecStochasticIntegrator::buildEulerMaruyamaCandidate(double timeStep, const Eigen::VectorXd& globalPseudoSteps)
{
    this->buildStochasticCandidate(
      this->stochasticAcceptedState(), this->packedDerivatives, timeStep, this->packedDiffusions, globalPseudoSteps);
}

void
StateVecStochasticIntegrator::advanceEuclideanEulerMaruyama(double timeStep, const Eigen::VectorXd& globalPseudoSteps)
{
    if (!this->stochasticUpdatesAreAllEuclidean() ||
        globalPseudoSteps.size() != static_cast<Eigen::Index>(this->globalNoiseCount())) {
        throw std::invalid_argument("Euler-Maruyama update requires a matching Euclidean topology.");
    }
    const auto& base = this->stochasticAcceptedState();
    for (const auto& descriptor : this->flatBinding.states()) {
        const auto count = static_cast<Eigen::Index>(descriptor.stateCount);
        Eigen::Map<Eigen::VectorXd> state(descriptor.state->stateView().data(), count);
        const auto initial = base.segment(static_cast<Eigen::Index>(descriptor.stateOffset), count);
        const auto drift =
          this->packedDerivatives.segment(static_cast<Eigen::Index>(descriptor.derivativeOffset), count);
        const auto diffusion = [&](size_t localIndex) {
            const size_t offset = descriptor.localNoiseOffset + localIndex;
            return Eigen::Map<const Eigen::VectorXd>(this->localNoiseLiveData[offset], count);
        };
        const auto increment = [&](size_t localIndex) {
            return globalPseudoSteps(
              static_cast<Eigen::Index>(this->localNoiseToGlobalSlot[descriptor.localNoiseOffset + localIndex]));
        };
        // Fuse the common one- and two-source updates into one vectorized pass,
        // preserving the same parenthesized additions in local source order.
        if (descriptor.noiseCount == 1) {
            state = (initial + drift * timeStep) + diffusion(0) * increment(0);
        } else if (descriptor.noiseCount == 2) {
            state = ((initial + drift * timeStep) + diffusion(0) * increment(0)) + diffusion(1) * increment(1);
        } else {
            state = initial + drift * timeStep;
            for (size_t localIndex = 0; localIndex < descriptor.noiseCount; ++localIndex) {
                state += diffusion(localIndex) * increment(localIndex);
            }
        }
    }
}

void
StateVecStochasticIntegrator::buildStochasticCandidate(const Eigen::VectorXd& base,
                                                       const Eigen::VectorXd& drift,
                                                       double timeStep,
                                                       const Eigen::VectorXd& diffusions,
                                                       const Eigen::VectorXd& globalPseudoSteps,
                                                       size_t slotBegin,
                                                       size_t slotEnd)
{
    this->writeStochasticCandidate(base, drift, timeStep, diffusions, globalPseudoSteps, slotBegin, slotEnd);
    this->scatterStochasticStates(this->candidateState);
    this->candidatePhase = CandidateAssemblyPhase::Ready;
}

void
StateVecStochasticIntegrator::writeStochasticCandidate(const Eigen::VectorXd& base,
                                                       const Eigen::VectorXd& drift,
                                                       double timeStep,
                                                       const Eigen::VectorXd& diffusions,
                                                       const Eigen::VectorXd& globalPseudoSteps,
                                                       size_t slotBegin,
                                                       size_t slotEnd)
{
    if (base.size() != this->candidateState.size() || drift.size() != this->packedDerivatives.size() ||
        !this->diffusionInputsMatchTopology(diffusions, globalPseudoSteps) || slotBegin > slotEnd ||
        slotEnd > this->noiseSlots.size()) {
        throw std::invalid_argument("Packed stochastic candidate does not match the bound topology.");
    }
    this->flatBinding.writeDriftCandidate(
      base, drift, timeStep, this->candidateState);

    for (const auto& descriptor : this->flatBinding.states()) {
        this->applyStochasticNoiseInLocalOrder(
          descriptor, descriptor.stateView(this->candidateState), diffusions, globalPseudoSteps, slotBegin, slotEnd);
    }
}

void
StateVecStochasticIntegrator::beginStochasticSparseCandidates(const Eigen::VectorXd& base,
                                                              const Eigen::VectorXd& drift,
                                                              double timeStep)
{
    if (base.size() != this->sparseBaselineState.size() || drift.size() != this->packedDerivatives.size()) {
        throw std::invalid_argument("Sparse stochastic baseline does not match the bound topology.");
    }
    this->flatBinding.writeDriftCandidate(
      base, drift, timeStep, this->sparseBaselineState);
    this->candidateState = this->sparseBaselineState;
    this->scatterStochasticStates(this->sparseBaselineState);
    this->activeSparseSlot = static_cast<size_t>(-1);
    this->candidatePhase = CandidateAssemblyPhase::Sparse;
}

void
StateVecStochasticIntegrator::buildStochasticNoiseSlotCandidate(const Eigen::VectorXd& diffusions,
                                                                double pseudoStep,
                                                                size_t slotIndex)
{
    if (this->candidatePhase != CandidateAssemblyPhase::Sparse) {
        throw std::logic_error("Sparse stochastic candidates must be initialized before use.");
    }
    if (diffusions.size() != this->packedDiffusions.size() || slotIndex >= this->noiseSlots.size()) {
        throw std::invalid_argument("Sparse stochastic noise candidate does not match the bound topology.");
    }

    // Diffusion callbacks may mutate arbitrary state. A block comparison keeps
    // the common no-mutation path cheap while preserving that legacy behavior.
    for (const auto& object : this->flatBinding.objects()) {
        if (object.stateCount == 0) {
            continue;
        }
        const double* baseline = this->sparseBaselineState.data() + static_cast<Eigen::Index>(object.stateOffset);
        const size_t byteCount = object.stateCount * sizeof(double);
        if (std::memcmp(object.stateData, baseline, byteCount) != 0) {
            std::memcpy(object.stateData, baseline, byteCount);
        }
    }

    if (this->activeSparseSlot != static_cast<size_t>(-1)) {
        const auto& previous = this->noiseSlots[this->activeSparseSlot];
        const size_t previousEnd = previous.bindingBegin + previous.bindingCount;
        for (size_t bindingIndex = previous.bindingBegin; bindingIndex < previousEnd; ++bindingIndex) {
            const auto& binding = this->noiseBindings[bindingIndex];
            const auto& descriptor = this->flatBinding.states()[binding.boundStateIndex];
            std::copy_n(this->sparseBaselineState.data() + static_cast<Eigen::Index>(descriptor.stateOffset),
                        descriptor.stateCount,
                        this->candidateState.data() + static_cast<Eigen::Index>(descriptor.stateOffset));
        }
    }

    const auto& slot = this->noiseSlots[slotIndex];
    const size_t bindingEnd = slot.bindingBegin + slot.bindingCount;
    try {
        for (size_t bindingIndex = slot.bindingBegin; bindingIndex < bindingEnd; ++bindingIndex) {
            const auto& binding = this->noiseBindings[bindingIndex];
            const auto& descriptor = this->flatBinding.states()[binding.boundStateIndex];
            this->applyStochasticNoiseIncrement(descriptor,
                                                descriptor.stateView(this->candidateState),
                                                descriptor.diffusionView(diffusions, binding.packedOffset),
                                                pseudoStep);
            std::copy_n(this->candidateState.data() + static_cast<Eigen::Index>(descriptor.stateOffset),
                        descriptor.stateCount,
                        descriptor.state->stateView().data());
        }
    } catch (...) {
        for (size_t bindingIndex = slot.bindingBegin; bindingIndex < bindingEnd; ++bindingIndex) {
            const auto& descriptor = this->flatBinding.states()[this->noiseBindings[bindingIndex].boundStateIndex];
            const double* baseline =
              this->sparseBaselineState.data() + static_cast<Eigen::Index>(descriptor.stateOffset);
            std::copy_n(baseline,
                        descriptor.stateCount,
                        this->candidateState.data() + static_cast<Eigen::Index>(descriptor.stateOffset));
            std::copy_n(baseline, descriptor.stateCount, descriptor.state->stateView().data());
        }
        throw;
    }
    this->activeSparseSlot = slotIndex;
}

void
StateVecStochasticIntegrator::acceptStochasticCandidate()
{
    if (this->candidatePhase != CandidateAssemblyPhase::Ready) {
        throw std::logic_error("A stochastic candidate must be built before it can be accepted.");
    }
    this->workingState.swap(this->candidateState);
    this->workingStateIsRollback = false;
    this->candidatePhase = CandidateAssemblyPhase::Idle;
}

void
StateVecStochasticIntegrator::beginAllEuclideanFinalCandidate(const Eigen::VectorXd& base,
                                                              const Eigen::VectorXd& drift,
                                                              double timeStep,
                                                              const Eigen::VectorXd& diffusions,
                                                              const Eigen::VectorXd& globalPseudoSteps)
{
    if (!this->flatBinding.updatesAreAllEuclidean()) {
        throw std::logic_error("All-Euclidean final candidate requested for special state topology.");
    }
    this->writeStochasticCandidate(base, drift, timeStep, diffusions, globalPseudoSteps, 0, this->globalNoiseCount());
    this->candidatePhase = CandidateAssemblyPhase::EuclideanFinal;
}

void
StateVecStochasticIntegrator::appendAllEuclideanFinalCandidate(const Eigen::VectorXd& drift,
                                                               double timeStep,
                                                               const Eigen::VectorXd& diffusions,
                                                               const Eigen::VectorXd& globalPseudoSteps)
{
    if (this->candidatePhase != CandidateAssemblyPhase::EuclideanFinal) {
        throw std::logic_error("Euclidean final candidate must be initialized before appending.");
    }
    if (!this->flatBinding.updatesAreAllEuclidean() || drift.size() != this->packedDerivatives.size() ||
        !this->diffusionInputsMatchTopology(diffusions, globalPseudoSteps)) {
        throw std::invalid_argument("All-Euclidean final candidate does not match the bound topology.");
    }

    for (const auto& run : this->flatBinding.updateRuns()) {
        const auto stateOffset = static_cast<Eigen::Index>(run.stateOffset);
        const auto derivativeOffset = static_cast<Eigen::Index>(run.derivativeOffset);
        const auto count = static_cast<Eigen::Index>(run.stateCount);
        Eigen::Map<Eigen::VectorXd> candidate(this->candidateState.data() + stateOffset, count);
        const Eigen::Map<const Eigen::VectorXd> derivative(drift.data() + derivativeOffset, count);
        candidate += derivative * timeStep;
    }

    for (const auto& descriptor : this->flatBinding.states()) {
        this->applyStochasticNoiseInLocalOrder(descriptor,
                                               descriptor.stateView(this->candidateState),
                                               diffusions,
                                               globalPseudoSteps,
                                               0,
                                               this->noiseSlots.size());
    }
}

void
StateVecStochasticIntegrator::commitAllEuclideanFinalCandidate()
{
    if (this->candidatePhase != CandidateAssemblyPhase::EuclideanFinal) {
        throw std::logic_error("Euclidean final candidate must be initialized before commit.");
    }
    if (!this->flatBinding.updatesAreAllEuclidean()) {
        throw std::logic_error("All-Euclidean final candidate requested for special state topology.");
    }
    this->scatterStochasticStates(this->candidateState);
    this->candidatePhase = CandidateAssemblyPhase::Idle;
}

void
StateVecStochasticIntegrator::buildStochasticDriftCandidate(const StochasticStateDescriptor& descriptor,
                                                            ConstMatrixView base,
                                                            ConstMatrixView drift,
                                                            double timeStep,
                                                            MutableMatrixView output)
{
    if (descriptor.usesEuclideanUpdate()) {
        output = base + drift * timeStep;
    } else {
        descriptor.specialUpdate->buildDriftCandidate(base, drift, timeStep, output);
    }
}

void
StateVecStochasticIntegrator::applyStochasticNoiseIncrement(const StochasticStateDescriptor& descriptor,
                                                            MutableMatrixView state,
                                                            ConstMatrixView diffusion,
                                                            double pseudoStep)
{
    if (descriptor.usesEuclideanUpdate()) {
        state += diffusion * pseudoStep;
    } else {
        descriptor.specialUpdate->applyNoiseIncrement(state, diffusion, pseudoStep);
    }
}

void
StateVecStochasticIntegrator::applyStochasticNoiseInLocalOrder(const StochasticStateDescriptor& descriptor,
                                                               MutableMatrixView state,
                                                               const Eigen::VectorXd& diffusions,
                                                               const Eigen::VectorXd& globalPseudoSteps,
                                                               size_t slotBegin,
                                                               size_t slotEnd)
{
    // Shared-source discovery can reverse global slots relative to local indices.
    // Local order matters for special policies and Euclidean rounding alike.
    for (size_t localNoiseIndex = 0; localNoiseIndex < descriptor.noiseCount; ++localNoiseIndex) {
        const size_t localIndex = descriptor.localNoiseOffset + localNoiseIndex;
        const size_t globalSlot = this->localNoiseToGlobalSlot[localIndex];
        if (globalSlot < slotBegin || globalSlot >= slotEnd) {
            continue;
        }
        this->applyStochasticNoiseIncrement(
          descriptor,
          state,
          descriptor.diffusionView(diffusions, this->localNoiseToPackedOffset[localIndex]),
          globalPseudoSteps(static_cast<Eigen::Index>(globalSlot)));
    }
}

bool
StateVecStochasticIntegrator::diffusionInputsMatchTopology(const Eigen::VectorXd& diffusions,
                                                           const Eigen::VectorXd& globalPseudoSteps) const
{
    return diffusions.size() == this->packedDiffusions.size() &&
           globalPseudoSteps.size() == static_cast<Eigen::Index>(this->noiseSlots.size());
}
