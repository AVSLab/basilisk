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
    this->stageBuffersValid = false;
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

size_t StochasticRKIntegratorBase::prepareStageBuffers(
    size_t derivativeCount, size_t diffusionCount, size_t snapshotCount, size_t noiseVectorCount,
    size_t diffusionStagesPerSource)
{
    const auto& maps = this->noiseIndexMaps();
    if (maps.size() > 1) diffusionCount += diffusionStagesPerSource * maps.size();
    if (!this->stageBuffersValid || this->derivativeStages.size() != derivativeCount ||
        this->diffusionStages.size() != diffusionCount || this->stateSnapshots.size() != snapshotCount ||
        this->noiseBuffers.size() != noiseVectorCount) {
        // Keep the validity flag false if allocation fails during preparation.
        this->stageBuffersValid = false;
        this->diffusionTargets.clear();
        this->diffusionTargets.resize(maps.size());
        for (size_t source = 0; source < maps.size(); ++source) {
            auto& targets = this->diffusionTargets[source];
            targets.reserve(maps[source].size());
            for (const auto& [id, localIndex] : maps[source]) {
                targets.push_back({this->dynPtrs.at(id.first)->dynManager.stateContainer.stateMap.at(id.second).get(),
                                   localIndex});
            }
        }
        const size_t stateCount = this->stateNoiseRouting.size();
        this->stateSnapshots.resize(snapshotCount);
        for (auto& snapshot : this->stateSnapshots) snapshot.resize(stateCount);
        this->derivativeStages.resize(derivativeCount);
        for (auto& stage : this->derivativeStages) stage.resize(stateCount);
        this->derivativeSum.resize(stateCount);
        this->derivativeTerm.resize(stateCount);
        this->diffusionStages.resize(diffusionCount);
        for (auto& stage : this->diffusionStages) {
            stage.resize(maps.size());
            for (size_t source = 0; source < maps.size(); ++source) stage[source].resize(maps[source].size());
        }
        this->diffusionSum.resize(maps.size());
        this->diffusionTerm.resize(maps.size());
        for (size_t source = 0; source < maps.size(); ++source) {
            this->diffusionSum[source].resize(maps[source].size());
            this->diffusionTerm[source].resize(maps[source].size());
        }
        this->noiseBuffers.resize(noiseVectorCount);
        for (auto& buffer : this->noiseBuffers) buffer.resize(static_cast<Eigen::Index>(maps.size()));
        this->stageBuffersValid = true;
    }
    return maps.size();
}

void StochasticRKIntegratorBase::captureStates(size_t snapshot)
{
    auto& buffer = this->stateSnapshots.at(snapshot);
    for (size_t index = 0; index < this->stateNoiseRouting.size(); ++index)
        buffer[index] = this->stateNoiseRouting[index].state->getStateReference();
}

void StochasticRKIntegratorBase::restoreStates(size_t snapshot)
{
    const auto& buffer = this->stateSnapshots.at(snapshot);
    for (size_t index = 0; index < this->stateNoiseRouting.size(); ++index)
        this->stateNoiseRouting[index].state->setState(buffer[index]);
}

void StochasticRKIntegratorBase::evaluateStageDerivatives(double time, double timeStep, size_t stage)
{
    for (auto* object : this->dynPtrs) object->equationsOfMotion(time, timeStep);
    auto& buffer = this->derivativeStages.at(stage);
    for (size_t index = 0; index < this->stateNoiseRouting.size(); ++index)
        buffer[index] = this->stateNoiseRouting[index].state->getStateDerivReference();
}

void StochasticRKIntegratorBase::captureStageDiffusion(size_t source, size_t stage)
{
    const auto& targets = this->diffusionTargets.at(source);
    auto& buffer = this->diffusionStages.at(stage).at(source);
    for (size_t index = 0; index < targets.size(); ++index)
        buffer[index] = targets[index].state->getStateDiffusionReference(targets[index].localIndex);
}

void StochasticRKIntegratorBase::evaluateStageDiffusions(double time, double timeStep, size_t stage)
{
    for (auto* object : this->dynPtrs) object->equationsOfMotionDiffusion(time, timeStep);
    for (size_t source = 0; source < this->diffusionTargets.size(); ++source)
        this->captureStageDiffusion(source, stage);
}

void StochasticRKIntegratorBase::evaluateStageDiffusion(double time, double timeStep, size_t source, size_t stage)
{
    for (auto* object : this->dynPtrs) object->equationsOfMotionDiffusion(time, timeStep);
    this->captureStageDiffusion(source, stage);
}

void StochasticRKIntegratorBase::applyStageDerivatives(size_t stage)
{
    const auto& buffer = this->derivativeStages.at(stage);
    for (size_t index = 0; index < this->stateNoiseRouting.size(); ++index)
        this->stateNoiseRouting[index].state->setDerivative(buffer[index]);
}

void StochasticRKIntegratorBase::applyStageDiffusion(size_t source, size_t stage)
{
    const auto& buffer = this->diffusionStages.at(stage).at(source);
    const auto& targets = this->diffusionTargets.at(source);
    for (size_t index = 0; index < targets.size(); ++index)
        targets[index].state->setDiffusion(buffer[index], targets[index].localIndex);
}

void StochasticRKIntegratorBase::applyDerivativeSum(const double* weights, size_t length, bool skipZeroWeights)
{
    if (length == 0 || length > this->derivativeStages.size())
        throw std::invalid_argument("Derivative sum requires at least one prepared stage");
    for (size_t index = 0; index < this->stateNoiseRouting.size(); ++index) {
        auto& sum = this->derivativeSum[index];
        auto& term = this->derivativeTerm[index];
        sum = this->derivativeStages[0][index] * weights[0];
        for (size_t stage = 1; stage < length; ++stage) {
            if (skipZeroWeights && weights[stage] == 0.0) continue;
            // Retain the original separately evaluated product before addition.
            term = this->derivativeStages[stage][index] * weights[stage];
            sum += term;
        }
    }
    for (size_t index = 0; index < this->stateNoiseRouting.size(); ++index)
        this->stateNoiseRouting[index].state->setDerivative(this->derivativeSum[index]);
}

void StochasticRKIntegratorBase::applyDiffusionSum(
    size_t source, const double* weights, size_t length, size_t firstStage, bool skipZeroWeights)
{
    if (length == 0 || firstStage >= this->diffusionStages.size() ||
        length > this->diffusionStages.size() - firstStage)
        throw std::invalid_argument("Diffusion sum requires at least one prepared stage");
    const auto& targets = this->diffusionTargets.at(source);
    for (size_t index = 0; index < targets.size(); ++index) {
        auto& sum = this->diffusionSum[source][index];
        auto& term = this->diffusionTerm[source][index];
        sum = this->diffusionStages[firstStage][source][index] * weights[0];
        for (size_t stage = 1; stage < length; ++stage) {
            if (skipZeroWeights && weights[stage] == 0.0) continue;
            term = this->diffusionStages[firstStage + stage][source][index] * weights[stage];
            sum += term;
        }
    }
    for (size_t index = 0; index < targets.size(); ++index)
        targets[index].state->setDiffusion(this->diffusionSum[source][index], targets[index].localIndex);
}
