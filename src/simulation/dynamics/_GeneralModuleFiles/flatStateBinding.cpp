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

#include "flatStateBinding.h"
#include "stateRegistry.h"

#include "dynParamManager.h"
#include "dynamicObject.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {
const char*
finalizedTopologyError(FlatStateBinding::Mode mode)
{
    return mode == FlatStateBinding::Mode::DeterministicRungeKutta
             ? "Runge-Kutta binding requires every state manager to be finalized."
             : "Stochastic integrator binding requires finalized state topology.";
}

const char*
inconsistentLayoutError(FlatStateBinding::Mode mode)
{
    return mode == FlatStateBinding::Mode::DeterministicRungeKutta
             ? "A finalized state manager has inconsistent state-layout metadata."
             : "A finalized stochastic state manager has inconsistent state-layout metadata.";
}

const char*
synchronizedDynamicsError(FlatStateBinding::Mode mode)
{
    return mode == FlatStateBinding::Mode::DeterministicRungeKutta
             ? "Runge-Kutta synchronized dynamics changed after integrator binding."
             : "Stochastic integrator synchronized dynamics changed after binding.";
}

const char*
unfinalizedStorageError(FlatStateBinding::Mode mode)
{
    return mode == FlatStateBinding::Mode::DeterministicRungeKutta
             ? "A bound state manager has no finalized storage."
             : "A bound stochastic state manager has no finalized storage.";
}

size_t
checkedBindingAdd(size_t left, size_t right, const char* quantity)
{
    const size_t eigenLimit = static_cast<size_t>(std::numeric_limits<Eigen::Index>::max());
    if (right > eigenLimit || left > eigenLimit - right) {
        throw std::overflow_error(std::string("Flat integrator ") + quantity + " exceeds Eigen::Index.");
    }
    return left + right;
}
}

void
FlatStateBinding::bind(const std::vector<DynamicObject*>& dynamics, Mode mode)
{
    if (this->bound) {
        if (this->bindingMode != mode) {
            throw std::logic_error("Flat state binding mode cannot change after binding.");
        }
        this->validate(dynamics);
        return;
    }

    std::vector<FlatObjectDescriptor> newObjects;
    std::vector<FlatStateDescriptor> newStates;
    std::vector<size_t> newNoiseTraversalOrder;
    size_t newStateCount = 0;
    size_t newDerivativeCount = 0;
    size_t newNoiseCount = 0;
    bool newAllEuclideanUpdates = true;

    newObjects.reserve(dynamics.size());
    for (size_t dynamicObjectIndex = 0; dynamicObjectIndex < dynamics.size(); ++dynamicObjectIndex) {
        DynamicObject* object = dynamics.at(dynamicObjectIndex);
        if (object == nullptr) {
            throw std::logic_error(mode == Mode::Stochastic
                                     ? "Stochastic integrator cannot bind a null DynamicObject."
                                     : "Runge-Kutta integrator cannot bind a null DynamicObject.");
        }

        StateRegistry& registry = object->dynManager.getStateRegistry();
        if (!registry.statesAreFinalized()) {
            throw std::logic_error(finalizedTopologyError(mode));
        }

        const auto& states = registry.getStateRegistrationOrder();
        const auto& layouts = registry.getStateLayouts();
        if (states.size() != layouts.size()) {
            throw std::logic_error(inconsistentLayoutError(mode));
        }

        const size_t firstStateIndex = newStates.size();
        const size_t objectStateOffset = newStateCount;
        const size_t objectDerivativeOffset = newDerivativeCount;
        for (size_t stateIndex = 0; stateIndex < states.size(); ++stateIndex) {
            StateData* state = states[stateIndex];
            const StateLayout& layout = layouts[stateIndex];
            if (mode == Mode::DeterministicRungeKutta && layout.noiseCount != 0) {
                throw std::runtime_error("A deterministic Runge-Kutta integrator cannot bind state '" +
                                         state->getName() + "' because it has stochastic noise sources.");
            }

            const bool euclideanUpdate = layout.updateKind == StateUpdateKind::Euclidean;
            newStates.push_back({ state,
                                  state->getName(),
                                  dynamicObjectIndex,
                                  checkedBindingAdd(objectStateOffset, layout.stateOffset, "state offset"),
                                  layout.stateCount,
                                  static_cast<Eigen::Index>(layout.stateRows),
                                  static_cast<Eigen::Index>(layout.stateCols),
                                  checkedBindingAdd(objectDerivativeOffset, layout.derivOffset, "derivative offset"),
                                  layout.derivCount,
                                  static_cast<Eigen::Index>(layout.derivRows),
                                  static_cast<Eigen::Index>(layout.derivCols),
                                  layout.diffusionCountPerSource,
                                  static_cast<Eigen::Index>(layout.diffusionRows),
                                  static_cast<Eigen::Index>(layout.diffusionCols),
                                  layout.noiseCount,
                                  0,
                                  layout.updateKind,
                                  layout.errorControl,
                                  layout.specialUpdate });
            newStateCount = checkedBindingAdd(newStateCount, layout.stateCount, "state size");
            newDerivativeCount = checkedBindingAdd(newDerivativeCount, layout.derivCount, "derivative size");
            newAllEuclideanUpdates = newAllEuclideanUpdates && euclideanUpdate;
        }

        std::vector<size_t> lexicalStateIndices(states.size());
        for (size_t index = 0; index < lexicalStateIndices.size(); ++index) {
            lexicalStateIndices[index] = firstStateIndex + index;
        }
        std::sort(lexicalStateIndices.begin(), lexicalStateIndices.end(), [&newStates](size_t left, size_t right) {
            return newStates[left].stateName < newStates[right].stateName;
        });
        for (size_t stateIndex : lexicalStateIndices) {
            newStates[stateIndex].localNoiseOffset = newNoiseCount;
            newNoiseCount = checkedBindingAdd(newNoiseCount, newStates[stateIndex].noiseCount, "noise-source count");
            newNoiseTraversalOrder.push_back(stateIndex);
        }

        double* stateData = nullptr;
        double* derivativeData = nullptr;
        if (!states.empty()) {
            const StateBufferSegment stateSegment = registry.getStateSegment(0, newStateCount - objectStateOffset);
            const StateBufferSegment derivativeSegment =
              registry.getDerivativeSegment(0, newDerivativeCount - objectDerivativeOffset);
            stateData = registry.stateSegmentData(stateSegment);
            derivativeData = registry.derivativeSegmentData(derivativeSegment);
        }

        newObjects.push_back({ object,
                               &registry,
                               firstStateIndex,
                               objectStateOffset,
                               newStateCount - objectStateOffset,
                               objectDerivativeOffset,
                               newDerivativeCount - objectDerivativeOffset,
                               stateData,
                               derivativeData });
    }

    std::vector<FlatUpdateRun> newUpdateRuns;
    newUpdateRuns.reserve(newStates.size());
    for (size_t descriptorIndex = 0; descriptorIndex < newStates.size(); ++descriptorIndex) {
        const auto& descriptor = newStates[descriptorIndex];
        if (descriptor.updateKind == StateUpdateKind::Euclidean && !newUpdateRuns.empty()) {
            auto& previous = newUpdateRuns.back();
            const bool stateIsContiguous = previous.stateOffset + previous.stateCount == descriptor.stateOffset;
            const bool derivativeIsContiguous =
              previous.derivativeOffset + previous.derivativeCount == descriptor.derivativeOffset;
            if (previous.updateKind == StateUpdateKind::Euclidean && stateIsContiguous && derivativeIsContiguous) {
                previous.stateCount += descriptor.stateCount;
                previous.derivativeCount += descriptor.derivativeCount;
                continue;
            }
        }
        newUpdateRuns.push_back({ descriptorIndex,
                                  descriptor.stateOffset,
                                  descriptor.stateCount,
                                  descriptor.derivativeOffset,
                                  descriptor.derivativeCount,
                                  descriptor.updateKind });
    }

    this->objectDescriptors = std::move(newObjects);
    this->stateDescriptors = std::move(newStates);
    this->stateUpdateRuns = std::move(newUpdateRuns);
    this->canonicalNoiseTraversalOrder = std::move(newNoiseTraversalOrder);
    this->stateCount = newStateCount;
    this->derivativeCount = newDerivativeCount;
    this->noiseCount = newNoiseCount;
    this->allEuclideanUpdates = newAllEuclideanUpdates;
    this->bindingMode = mode;
    this->bound = true;
}

void
FlatStateBinding::reset() noexcept
{
    this->bound = false;
    this->stateCount = 0;
    this->derivativeCount = 0;
    this->noiseCount = 0;
    this->allEuclideanUpdates = true;
    std::vector<FlatObjectDescriptor>().swap(this->objectDescriptors);
    std::vector<FlatStateDescriptor>().swap(this->stateDescriptors);
    std::vector<FlatUpdateRun>().swap(this->stateUpdateRuns);
    std::vector<size_t>().swap(this->canonicalNoiseTraversalOrder);
}

void
FlatStateBinding::validate(const std::vector<DynamicObject*>& dynamics) const
{
    if (!this->bound || dynamics.size() != this->objectDescriptors.size()) {
        throw std::runtime_error(synchronizedDynamicsError(this->bindingMode));
    }
    for (size_t dynamicObjectIndex = 0; dynamicObjectIndex < this->objectDescriptors.size(); ++dynamicObjectIndex) {
        const auto& descriptor = this->objectDescriptors[dynamicObjectIndex];
        if (dynamics[dynamicObjectIndex] != descriptor.object || descriptor.object == nullptr ||
            &descriptor.object->dynManager.getStateRegistry() != descriptor.registry) {
            throw std::runtime_error(synchronizedDynamicsError(this->bindingMode));
        }
        if (!descriptor.registry->statesAreFinalized()) {
            throw std::logic_error(unfinalizedStorageError(this->bindingMode));
        }
    }
}

void
FlatStateBinding::writeDriftCandidate(const Eigen::VectorXd& base,
                                      const Eigen::VectorXd& drift,
                                      double timeStep,
                                      Eigen::Ref<Eigen::VectorXd> output) const
{
    if (base.size() != static_cast<Eigen::Index>(this->stateCount) ||
        output.size() != static_cast<Eigen::Index>(this->stateCount) ||
        drift.size() != static_cast<Eigen::Index>(this->derivativeCount)) {
        throw std::invalid_argument("Flat drift candidate does not match the bound topology.");
    }

    for (const auto& run : this->stateUpdateRuns) {
        const auto stateOffset = static_cast<Eigen::Index>(run.stateOffset);
        const auto derivativeOffset = static_cast<Eigen::Index>(run.derivativeOffset);
        if (run.updateKind == StateUpdateKind::Special) {
            const auto& descriptor = this->stateDescriptors.at(run.firstDescriptorIndex);
            descriptor.specialUpdate->buildDriftCandidate(
              descriptor.stateView(base),
              descriptor.derivativeView(drift),
              timeStep,
              MutableMatrixView(output.data() + descriptor.stateOffset, descriptor.stateRows, descriptor.stateColumns));
            continue;
        }

        const auto count = static_cast<Eigen::Index>(run.stateCount);
        const Eigen::Map<const Eigen::VectorXd> baseState(base.data() + stateOffset, count);
        const Eigen::Map<const Eigen::VectorXd> derivative(drift.data() + derivativeOffset, count);
        Eigen::Map<Eigen::VectorXd> candidate(output.data() + stateOffset, count);
        candidate = baseState + derivative * timeStep;
    }
}
