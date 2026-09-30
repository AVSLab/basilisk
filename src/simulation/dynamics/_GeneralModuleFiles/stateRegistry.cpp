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

/**
 * @file stateRegistry.cpp
 * @brief Named state declarations and one-time contiguous buffer allocation.
 * Prepare potentially throwing work before committing topology or rebinding handles.
 */

#include "stateRegistry.h"

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace {
// Validate each matrix shape before allocating seed matrices or computing offsets.
size_t
checkedProduct(const MatrixShape& shape, const std::string& stateName, const char* field)
{
    if (shape.rows == 0 || shape.cols == 0) {
        throw std::invalid_argument("State '" + stateName + "' " + field + " shape must be nonzero.");
    }
    const size_t rows = shape.rows;
    const size_t cols = shape.cols;
    if (rows > std::numeric_limits<size_t>::max() / cols) {
        throw std::overflow_error("State '" + stateName + "' " + field + " element count overflows.");
    }
    const size_t count = rows * cols;
    if (count > static_cast<size_t>(std::numeric_limits<Eigen::Index>::max())) {
        throw std::overflow_error("State '" + stateName + "' " + field + " element count exceeds Eigen::Index.");
    }
    return count;
}

// Buffer offsets must fit both size_t arithmetic and Eigen indexing.
size_t
checkedAdd(size_t left, size_t right, const char* bufferName)
{
    if (left > std::numeric_limits<size_t>::max() - right) {
        throw std::overflow_error(std::string("The ") + bufferName + " buffer size overflows.");
    }
    const size_t result = left + right;
    if (result > static_cast<size_t>(std::numeric_limits<Eigen::Index>::max())) {
        throw std::overflow_error(std::string("The ") + bufferName + " buffer exceeds Eigen::Index.");
    }
    return result;
}

// Return a complete record's scalar offset and size in the selected buffer.
std::pair<size_t, size_t>
recordRange(const StateLayout& layout, StateBufferKind kind)
{
    switch (kind) {
        case StateBufferKind::State:
            return { layout.stateOffset, layout.stateCount };
        case StateBufferKind::Derivative:
            return { layout.derivOffset, layout.derivCount };
        case StateBufferKind::Diffusion:
            return { layout.diffusionOffset, layout.diffusionCountPerSource * layout.noiseCount };
    }
    throw std::invalid_argument("Unknown state buffer kind.");
}

// Typed accessors supply their buffer directly so resolving a cached segment
// does not need a buffer-selection branch.
const double*
resolveSegment(const StateRegistry* registry,
               const StateBufferSegment& segment,
               StateBufferKind expectedKind,
               const Eigen::VectorXd& buffer)
{
    if (segment.registry != registry || segment.bufferKind != expectedKind ||
        segment.offset > static_cast<size_t>(buffer.size()) ||
        segment.count > static_cast<size_t>(buffer.size()) - segment.offset) {
        throw std::logic_error("State buffer segment has the wrong owner, kind, or bounds.");
    }
    return segment.offset == 0 ? buffer.data() : buffer.data() + segment.offset;
}
}

void
StateRegistry::finalizeStates()
{
    if (this->finalized) {
        return;
    }
    this->allocateBuffers();
    for (size_t index = 0; index < this->stateLayoutView.size(); ++index) {
        const StateLayout& layout = this->stateLayoutView[index];
        this->stateRecords[index].handle->bindStateViews(this->liveBuffers.states.data() + layout.stateOffset,
                                                       this->liveBuffers.derivatives.data() + layout.derivOffset);
        this->stateRecords[index].seed.reset();
    }
    this->finalized = true;
}

StateData*
StateRegistry::registerState(std::string stateName, const StateSpec& spec)
{
    return this->registerManagedState(std::move(stateName), spec, nullptr);
}

StateData*
StateRegistry::registerState(std::string stateName, const StateSpec& spec, std::unique_ptr<StateUpdatePolicy> policy)
{
    return this->registerManagedState(std::move(stateName), spec, std::move(policy));
}

StateData*
StateRegistry::registerManagedState(std::string stateName,
                                    const StateSpec& spec,
                                    std::unique_ptr<StateUpdatePolicy> policy)
{
    this->validateSpec(stateName, spec, policy.get());

    const auto existing = this->stateSlotByName.find(stateName);
    if (existing != this->stateSlotByName.end()) {
        const StateRecord& record = this->stateRecords[existing->second];
        const StateUpdatePolicy* previousPolicy = record.updatePolicy.get();
        if (record.spec != spec || (previousPolicy == nullptr) != (policy == nullptr) ||
            (previousPolicy != nullptr && !previousPolicy->topologyEquals(*policy))) {
            throw std::logic_error("Topology mismatch for state '" + stateName + "'.");
        }
        return record.handle.get();
    }
    if (this->finalized) {
        throw std::logic_error("Cannot add state '" + stateName + "' after finalization.");
    }

    const size_t slot = this->stateRecords.size();
    auto state = std::unique_ptr<StateData>(new StateData(this, slot, spec));
    auto seed = std::make_unique<StateSeed>();
    seed->state = Eigen::MatrixXd::Zero(spec.state.rows, spec.state.cols);
    seed->derivative = Eigen::MatrixXd::Zero(spec.derivative.rows, spec.derivative.cols);
    seed->diffusions.resize(spec.noiseCount);
    for (auto& diffusion : seed->diffusions) {
        diffusion = Eigen::MatrixXd::Zero(spec.diffusionTangent.rows, spec.diffusionTangent.cols);
    }
    StateData* result = state.get();
    StateRecord record;
    record.handle = std::move(state);
    record.name = stateName;
    record.spec = spec;
    record.seed = std::move(seed);
    record.updatePolicy = std::move(policy);

    const auto inserted = this->stateSlotByName.emplace(std::move(stateName), slot);
    try {
        this->stateRecords.push_back(std::move(record));
        this->registrationOrderView.push_back(result);
    } catch (...) {
        if (this->stateRecords.size() != slot) {
            this->stateRecords.pop_back();
        }
        this->stateSlotByName.erase(inserted.first);
        throw;
    }
    result->bindStateViews(this->stateRecords.back().seed->state.data(),
                           this->stateRecords.back().seed->derivative.data());
    return result;
}

StateData*
StateRegistry::registerState(uint32_t nRow, uint32_t nCol, std::string stateName)
{
    StateSpec spec;
    spec.state = { nRow, nCol };
    spec.derivative = spec.state;
    spec.diffusionTangent = spec.state;
    const auto existing = this->stateSlotByName.find(stateName);
    if (existing != this->stateSlotByName.end()) {
        const StateRecord& established = this->stateRecords[existing->second];
        if (established.spec.updateKind == StateUpdateKind::Euclidean &&
            established.spec.state == spec.state && established.spec.derivative == spec.derivative &&
            established.spec.diffusionTangent == spec.diffusionTangent) {
            spec = established.spec;
        }
    }
    return this->registerState(std::move(stateName), spec);
}

void
StateRegistry::validateSpec(const std::string& stateName, const StateSpec& spec, const StateUpdatePolicy* policy) const
{
    if (stateName.empty()) {
        throw std::invalid_argument("A state name cannot be empty.");
    }
    const size_t stateCount = checkedProduct(spec.state, stateName, "state");
    const size_t derivativeCount = checkedProduct(spec.derivative, stateName, "derivative");
    const size_t diffusionCount = checkedProduct(spec.diffusionTangent, stateName, "diffusion");
    (void)stateCount;
    (void)derivativeCount;
    if (spec.noiseCount > 0 && diffusionCount > std::numeric_limits<size_t>::max() / spec.noiseCount) {
        throw std::overflow_error("State '" + stateName + "' diffusion storage size overflows.");
    }

    if (spec.updateKind == StateUpdateKind::Euclidean) {
        if (policy != nullptr) {
            throw std::invalid_argument("Euclidean state '" + stateName + "' cannot supply a special update policy.");
        }
        if (spec.state != spec.derivative || spec.state != spec.diffusionTangent) {
            throw std::invalid_argument("Euclidean state '" + stateName +
                                        "' requires equal state, derivative, and diffusion shapes.");
        }
        return;
    }

    if (policy == nullptr) {
        throw std::invalid_argument("Special state '" + stateName + "' requires an update policy.");
    }
    policy->validate(spec);
}

void
StateRegistry::allocateBuffers()
{
    NoiseTopology noiseTopology = this->canonicalNoiseTopology();
    std::vector<StateLayout> layouts;
    layouts.reserve(this->stateRecords.size());
    size_t stateTotal = 0;
    size_t derivativeTotal = 0;
    size_t diffusionTotal = 0;

    for (size_t index = 0; index < this->stateRecords.size(); ++index) {
        const StateRecord& record = this->stateRecords[index];
        const StateSpec& spec = record.spec;
        StateLayout layout;
        layout.stateRows = spec.state.rows;
        layout.stateCols = spec.state.cols;
        layout.stateOffset = stateTotal;
        layout.stateCount = checkedProduct(spec.state, record.name, "state");
        stateTotal = checkedAdd(stateTotal, layout.stateCount, "state");

        layout.derivRows = spec.derivative.rows;
        layout.derivCols = spec.derivative.cols;
        layout.derivOffset = derivativeTotal;
        layout.derivCount = checkedProduct(spec.derivative, record.name, "derivative");
        derivativeTotal = checkedAdd(derivativeTotal, layout.derivCount, "derivative");

        layout.diffusionRows = spec.diffusionTangent.rows;
        layout.diffusionCols = spec.diffusionTangent.cols;
        layout.diffusionOffset = diffusionTotal;
        layout.diffusionCountPerSource = checkedProduct(spec.diffusionTangent, record.name, "diffusion");
        layout.noiseCount = spec.noiseCount;
        const size_t thisDiffusionCount = layout.diffusionCountPerSource * layout.noiseCount;
        diffusionTotal = checkedAdd(diffusionTotal, thisDiffusionCount, "diffusion");
        layout.errorControl = spec.errorControl;
        layout.updateKind = spec.updateKind;
        layout.specialUpdate = record.updatePolicy.get();
        layouts.push_back(layout);
    }

    StateBuffers newLive;
    newLive.states.resize(static_cast<Eigen::Index>(stateTotal));
    newLive.derivatives.resize(static_cast<Eigen::Index>(derivativeTotal));
    newLive.diffusions.resize(static_cast<Eigen::Index>(diffusionTotal));

    for (size_t index = 0; index < layouts.size(); ++index) {
        const StateLayout& layout = layouts[index];
        const StateSeed& seed = *this->stateRecords[index].seed;
        newLive.states.segment(static_cast<Eigen::Index>(layout.stateOffset), layout.stateCount) =
          Eigen::Map<const Eigen::VectorXd>(seed.state.data(), static_cast<Eigen::Index>(layout.stateCount));
        newLive.derivatives.segment(static_cast<Eigen::Index>(layout.derivOffset), layout.derivCount) =
          Eigen::Map<const Eigen::VectorXd>(seed.derivative.data(), static_cast<Eigen::Index>(layout.derivCount));
        for (size_t noiseIndex = 0; noiseIndex < layout.noiseCount; ++noiseIndex) {
            newLive.diffusions.segment(
              static_cast<Eigen::Index>(layout.diffusionOffset + noiseIndex * layout.diffusionCountPerSource),
              layout.diffusionCountPerSource) =
              Eigen::Map<const Eigen::VectorXd>(seed.diffusions[noiseIndex].data(),
                                                static_cast<Eigen::Index>(layout.diffusionCountPerSource));
        }
    }

    this->stateLayoutView.swap(layouts);
    this->liveBuffers.states.swap(newLive.states);
    this->liveBuffers.derivatives.swap(newLive.derivatives);
    this->liveBuffers.diffusions.swap(newLive.diffusions);
    this->finalizedNoiseTopology.swap(noiseTopology);

    this->pendingNoiseGroups.clear();
}

StateRegistry::NoiseTopology
StateRegistry::canonicalNoiseTopology() const
{
    NoiseTopology result = this->pendingNoiseGroups;
    std::set<NoiseEndpoint> memberships;
    for (NoiseGroup& group : result) {
        std::sort(group.begin(), group.end());
        if (std::adjacent_find(group.begin(), group.end()) != group.end()) {
            throw std::logic_error("A shared-noise group contains the same local source more than once.");
        }
        if (std::adjacent_find(group.begin(), group.end(), [](const NoiseEndpoint& left, const NoiseEndpoint& right) {
                return left.first == right.first;
            }) != group.end()) {
            throw std::logic_error("A shared-noise group cannot contain multiple sources from "
                                   "the same state.");
        }
        for (const NoiseEndpoint& endpoint : group) {
            if (!memberships.emplace(endpoint).second) {
                throw std::logic_error("A local noise source belongs to more than one shared group.");
            }
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

StateData*
StateRegistry::getStateObject(const std::string& stateName) const
{
    const auto state = this->stateSlotByName.find(stateName);
    if (state != this->stateSlotByName.end()) {
        return this->stateRecords.at(state->second).handle.get();
    }
    return nullptr;
}

const std::vector<StateData*>&
StateRegistry::getStateRegistrationOrder() const
{
    this->requireFinalized("getStateRegistrationOrder()");
    return this->registrationOrderView;
}

const std::vector<StateLayout>&
StateRegistry::getStateLayouts() const
{
    this->requireFinalized("getStateLayouts()");
    return this->stateLayoutView;
}

StateBufferSegment
StateRegistry::getSegment(StateBufferKind kind, size_t firstSlot, size_t elementCount) const
{
    this->requireFinalized("getSegment()");
    if (firstSlot >= this->stateRecords.size()) {
        throw std::out_of_range("Buffer segment first slot is out of range.");
    }
    const size_t offset = recordRange(this->stateLayoutView[firstSlot], kind).first;
    size_t count = 0;
    for (size_t slot = firstSlot; slot < this->stateRecords.size() && count < elementCount; ++slot) {
        count += recordRange(this->stateLayoutView[slot], kind).second;
    }
    if (count != elementCount) {
        throw std::logic_error("Buffer segment does not end on a complete record boundary.");
    }
    return { this, kind, offset, count };
}

MutableMatrixView
StateRegistry::segmentView(const StateBufferSegment& segment)
{
    const auto view = static_cast<const StateRegistry&>(*this).segmentView(segment);
    return MutableMatrixView(const_cast<double*>(view.data()), view.rows(), view.cols());
}

ConstMatrixView
StateRegistry::segmentView(const StateBufferSegment& segment) const
{
    const double* data;
    switch (segment.bufferKind) {
        case StateBufferKind::State:
            data = this->stateSegmentData(segment);
            break;
        case StateBufferKind::Derivative:
            data = this->derivativeSegmentData(segment);
            break;
        case StateBufferKind::Diffusion:
            data = this->diffusionSegmentData(segment);
            break;
        default:
            throw std::logic_error("Unknown state buffer kind.");
    }
    return ConstMatrixView(data, static_cast<Eigen::Index>(segment.count), 1);
}

StateBufferSegment
StateRegistry::getStateSegment(size_t firstSlot, size_t elementCount) const
{
    return this->getSegment(StateBufferKind::State, firstSlot, elementCount);
}

StateBufferSegment
StateRegistry::getDerivativeSegment(size_t firstSlot, size_t elementCount) const
{
    return this->getSegment(StateBufferKind::Derivative, firstSlot, elementCount);
}

StateBufferSegment
StateRegistry::getDiffusionSegment(size_t firstSlot, size_t elementCount) const
{
    return this->getSegment(StateBufferKind::Diffusion, firstSlot, elementCount);
}

double*
StateRegistry::stateSegmentData(const StateBufferSegment& segment)
{
    return const_cast<double*>(static_cast<const StateRegistry&>(*this).stateSegmentData(segment));
}

const double*
StateRegistry::stateSegmentData(const StateBufferSegment& segment) const
{
    this->requireFinalized("stateSegmentData()");
    return resolveSegment(this, segment, StateBufferKind::State, this->liveBuffers.states);
}

double*
StateRegistry::derivativeSegmentData(const StateBufferSegment& segment)
{
    return const_cast<double*>(static_cast<const StateRegistry&>(*this).derivativeSegmentData(segment));
}

const double*
StateRegistry::derivativeSegmentData(const StateBufferSegment& segment) const
{
    this->requireFinalized("derivativeSegmentData()");
    return resolveSegment(this, segment, StateBufferKind::Derivative, this->liveBuffers.derivatives);
}

double*
StateRegistry::diffusionSegmentData(const StateBufferSegment& segment)
{
    return const_cast<double*>(static_cast<const StateRegistry&>(*this).diffusionSegmentData(segment));
}

const double*
StateRegistry::diffusionSegmentData(const StateBufferSegment& segment) const
{
    this->requireFinalized("diffusionSegmentData()");
    return resolveSegment(this, segment, StateBufferKind::Diffusion, this->liveBuffers.diffusions);
}

const StateRegistry::NoiseTopology&
StateRegistry::getSharedNoiseTopology() const
{
    this->requireFinalized("getSharedNoiseTopology()");
    return this->finalizedNoiseTopology;
}

void
StateRegistry::registerSharedNoiseSource(std::vector<std::pair<const StateData&, size_t>> sharedNoises)
{
    NoiseGroup group;
    group.reserve(sharedNoises.size());
    for (const auto& [state, noiseIndex] : sharedNoises) {
        if (state.owner != this || state.slot >= this->stateRecords.size() ||
            this->stateRecords[state.slot].handle.get() != &state) {
            throw std::invalid_argument("A shared-noise state does not belong to this DynParamManager.");
        }
        const StateRecord& record = this->stateRecords[state.slot];
        if (noiseIndex >= record.spec.noiseCount) {
            throw std::out_of_range("State '" + record.name + "' has only " + std::to_string(record.spec.noiseCount) +
                                    " noise sources; cannot share index " + std::to_string(noiseIndex) + ".");
        }
        group.emplace_back(state.slot, noiseIndex);
    }
    std::sort(group.begin(), group.end());
    const auto& groups = this->finalized ? this->finalizedNoiseTopology : this->pendingNoiseGroups;
    if (std::find(groups.begin(), groups.end(), group) != groups.end()) {
        return;
    }
    if (this->finalized) {
        throw std::logic_error("Shared-noise topology cannot change after finalization.");
    }
    this->pendingNoiseGroups.push_back(std::move(group));
}

void
StateRegistry::requireFinalized(const char* operation) const
{
    if (!this->finalized) {
        throw std::logic_error(std::string(operation) + " requires finalized state topology.");
    }
}

const std::string&
StateRegistry::handleName(size_t slot) const
{
    return this->stateRecords.at(slot).name;
}

void
StateRegistry::setHandleNoiseCount(size_t slot, size_t numSources)
{
    StateRecord& record = this->stateRecords.at(slot);
    if (this->finalized) {
        if (numSources != record.spec.noiseCount) {
            throw std::logic_error("State '" + record.name +
                                   "' has an established noise-source "
                                   "count of " +
                                   std::to_string(record.spec.noiseCount) +
                                   "; setNumNoiseSources() cannot change finalized topology.");
        }
        return;
    }
    if (numSources == record.spec.noiseCount) {
        return;
    }
    for (const NoiseGroup& group : this->pendingNoiseGroups) {
        for (const NoiseEndpoint& endpoint : group) {
            if (endpoint.first == slot && endpoint.second >= numSources) {
                throw std::logic_error("State '" + record.name +
                                       "' cannot reduce its noise-source "
                                       "count below an already-shared source index.");
            }
        }
    }

    StateSpec updatedSpec = record.spec;
    updatedSpec.noiseCount = numSources;
    this->validateSpec(record.name, updatedSpec, record.updatePolicy.get());

    std::vector<Eigen::MatrixXd> updatedDiffusions = record.seed->diffusions;
    const size_t previousCount = updatedDiffusions.size();
    updatedDiffusions.resize(numSources);
    for (size_t index = previousCount; index < numSources; ++index) {
        updatedDiffusions[index] =
          Eigen::MatrixXd::Zero(updatedSpec.diffusionTangent.rows, updatedSpec.diffusionTangent.cols);
    }
    record.seed->diffusions = std::move(updatedDiffusions);
    record.spec = updatedSpec;
}

double*
StateRegistry::activeDiffusionData(size_t slot, size_t localNoiseIndex)
{
    return const_cast<double*>(static_cast<const StateRegistry&>(*this).activeDiffusionData(slot, localNoiseIndex));
}

const double*
StateRegistry::activeDiffusionData(size_t slot, size_t localNoiseIndex) const
{
    const StateRecord& record = this->stateRecords.at(slot);
    const StateSpec& spec = record.spec;
    if (localNoiseIndex >= spec.noiseCount) {
        throw std::out_of_range("State diffusion index is out of range.");
    }
    if (!this->finalized) {
        return record.seed->diffusions.at(localNoiseIndex).data();
    }
    const StateLayout& layout = this->stateLayoutView.at(slot);
    return this->liveBuffers.diffusions.data() + layout.diffusionOffset + localNoiseIndex * layout.diffusionCountPerSource;
}

void
StateRegistry::requireRawAccess(const char* field) const
{
    if (!this->finalized) {
        throw std::logic_error(std::string("Raw ") + field + " access requires finalized state storage.");
    }
}

double*
StateRegistry::liveStateData(size_t slot)
{
    this->requireRawAccess("state");
    return this->liveBuffers.states.data() + this->stateLayoutView.at(slot).stateOffset;
}

double*
StateRegistry::liveDerivativeData(size_t slot)
{
    this->requireRawAccess("derivative");
    return this->liveBuffers.derivatives.data() + this->stateLayoutView.at(slot).derivOffset;
}

double*
StateRegistry::liveDiffusionData(size_t slot, size_t localNoiseIndex)
{
    this->requireRawAccess("diffusion");
    const StateLayout& layout = this->stateLayoutView.at(slot);
    if (localNoiseIndex >= layout.noiseCount) {
        throw std::out_of_range("State diffusion index is out of range.");
    }
    return this->liveBuffers.diffusions.data() + layout.diffusionOffset +
           localNoiseIndex * layout.diffusionCountPerSource;
}
