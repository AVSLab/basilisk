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
#include "stochasticRKIntegratorBase.h"

#include <stdexcept>
#include <typeinfo>
#include <unordered_map>

const std::vector<StateIdToIndexMap>& StochasticRKIntegratorBase::noiseIndexMaps()
{
    if (!this->noiseLayoutMatches()) {
        this->rebuildNoiseRouting();
    }
    return this->cachedNoiseIndexMaps;
}

bool StochasticRKIntegratorBase::noiseLayoutMatches() const
{
    if (!this->noiseIndexMapsCached || this->dynPtrs.size() != this->objectNoiseLayouts.size()) {
        return false;
    }
    size_t stateIndex = 0;
    for (size_t objectIndex = 0; objectIndex < this->dynPtrs.size(); ++objectIndex) {
        const auto* object = this->dynPtrs[objectIndex];
        const auto& layout = this->objectNoiseLayouts[objectIndex];
        const auto& manager = object->dynManager;
        if (object != layout.object || manager.stateContainer.stateMap.size() != layout.stateCount ||
            manager.sharedNoiseMap != layout.sharedNoiseMap) {
            return false;
        }
        for (const auto& [name, state] : manager.stateContainer.stateMap) {
            const auto& routing = this->stateNoiseRouting[stateIndex++];
            if (state.get() != routing.state || name != routing.name ||
                state->getNumNoiseSources() != routing.sourceIndices.size()) {
                return false;
            }
        }
    }
    return true;
}

void StochasticRKIntegratorBase::rebuildNoiseRouting()
{
    // Prepare a complete replacement before publishing it, including after a failed rebuild.
    auto maps = this->getStateIdToNoiseIndexMaps();
    std::vector<StateNoiseRouting> routing;
    std::vector<ObjectNoiseLayout> layouts;
    std::unordered_map<ExtendedStateId, size_t> stateIndices;
    for (size_t objectIndex = 0; objectIndex < this->dynPtrs.size(); ++objectIndex) {
        auto* object = this->dynPtrs[objectIndex];
        const auto& manager = object->dynManager;
        layouts.push_back({object, manager.stateContainer.stateMap.size(), manager.sharedNoiseMap});
        for (const auto& [name, state] : manager.stateContainer.stateMap) {
            stateIndices.emplace(ExtendedStateId{objectIndex, name}, routing.size());
            const size_t noiseCount = state->getNumNoiseSources();
            // The map representation can omit a local channel (for example, when
            // two channels of one state share a source). Keep such increments zero,
            // matching the public map-taking propagation method.
            routing.push_back({state.get(), name, std::vector<size_t>(noiseCount, maps.size()),
                               std::vector<double>(noiseCount)});
        }
    }
    for (size_t sourceIndex = 0; sourceIndex < maps.size(); ++sourceIndex) {
        for (const auto& [stateId, localIndex] : maps[sourceIndex]) {
            routing.at(stateIndices.at(stateId)).sourceIndices.at(localIndex) = sourceIndex;
        }
    }
    this->cachedNoiseIndexMaps = std::move(maps);
    this->stateNoiseRouting = std::move(routing);
    this->objectNoiseLayouts = std::move(layouts);
    this->noiseIndexMapsCached = true;
}

void StochasticRKIntegratorBase::propagateStateWithCachedNoise(
    double timeStep, const Eigen::VectorXd& pseudoTimeSteps)
{
    if (!this->noiseIndexMapsCached ||
        static_cast<size_t>(pseudoTimeSteps.size()) != this->cachedNoiseIndexMaps.size()) {
        throw std::invalid_argument("Stochastic propagation requires prepared routing and one increment per source");
    }
    for (auto& routing : this->stateNoiseRouting) {
        for (size_t localIndex = 0; localIndex < routing.sourceIndices.size(); ++localIndex) {
            const size_t sourceIndex = routing.sourceIndices[localIndex];
            routing.increments[localIndex] = sourceIndex < this->cachedNoiseIndexMaps.size()
                ? pseudoTimeSteps(static_cast<Eigen::Index>(sourceIndex)) : 0.0;
        }
        routing.state->propagateState(timeStep, routing.increments);
    }
}

void StochasticRKIntegratorBase::computeEulerDerivatives(double time, double timeStep)
{
    for (const auto& routing : this->stateNoiseRouting) {
        if (typeid(*routing.state) != typeid(StateData)) {
            // A custom virtual setter may transform the derivative. Preserve the legacy
            // snapshot and writeback, including its ordering, for these states.
            this->computeDerivatives(time, timeStep).setDerivatives(this->dynPtrs);
            return;
        }
    }
    for (auto* object : this->dynPtrs) {
        object->equationsOfMotion(time, timeStep);
    }
}

ExtendedStateVector StochasticRKIntegratorBase::computeDerivatives(double time, double timeStep)
{
    for (auto dynPtr : this->dynPtrs) {
        dynPtr->equationsOfMotion(time, timeStep);
    }
    return ExtendedStateVector::fromStateDerivs(this->dynPtrs);
}

ExtendedStateVector StochasticRKIntegratorBase::computeDiffusion(
    double time, double timeStep, const StateIdToIndexMap& stateIdToNoiseIndexMap)
{
    for (auto dynPtr : this->dynPtrs) {
        dynPtr->equationsOfMotionDiffusion(time, timeStep);
    }
    return ExtendedStateVector::fromStateDiffusions(this->dynPtrs, stateIdToNoiseIndexMap);
}

std::vector<ExtendedStateVector> StochasticRKIntegratorBase::computeDiffusions(
    double time, double timeStep, const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps)
{
    for (auto dynPtr : this->dynPtrs) {
        dynPtr->equationsOfMotionDiffusion(time, timeStep);
    }
    return ExtendedStateVector::fromStateDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps);
}
