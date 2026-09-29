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

/** @file flatStateBinding.h
 * @brief Native descriptors that connect finalized registries to numerical buffers.
 */

#ifndef flatStateBinding_h
#define flatStateBinding_h

#include "stateData.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

class DynamicObject;
class StateRegistry;

/** @brief One finalized object's buffers within the integrator's concatenated buffers.
 * Counts and offsets are in doubles. Live pointers borrow the registry's committed
 * buffers; binding validation must succeed before any pointer is used.
 */
struct FlatObjectDescriptor
{
    DynamicObject* object = nullptr; ///< Borrowed dynamics callback target.
    StateRegistry* registry = nullptr; ///< Borrowed registry owning this object's buffers.
    size_t firstStateIndex = 0; ///< First entry in the binding's state-descriptor vector.
    size_t stateOffset = 0; ///< First scalar in a combined state buffer.
    size_t stateCount = 0; ///< Scalars in this object's live state buffer.
    size_t derivativeOffset = 0; ///< First scalar in a combined derivative buffer.
    size_t derivativeCount = 0; ///< Scalars in this object's derivative buffer.
    double* stateData = nullptr; ///< Borrowed start of this object's live state buffer.
    double* derivativeData = nullptr; ///< Borrowed start of this object's derivative buffer.
};

/** @brief One state with shapes and integrator-global offsets resolved at binding time.
 * State, derivative, and diffusion counts may differ for special policies. Noise
 * offsets address the state-local mapping; packed global-slot offsets live in the
 * stochastic binding. No state-name lookup is needed during numerical updates.
 */
struct FlatStateDescriptor
{
    StateData* state = nullptr; ///< Borrowed stable handle.
    std::string stateName; ///< Registration name, used for diagnostics and tolerance resolution.
    size_t dynamicObjectIndex = 0; ///< Owning entry in the object-descriptor vector.
    size_t stateOffset = 0; ///< First scalar in a combined state buffer.
    size_t stateCount = 0; ///< Stored state scalars.
    Eigen::Index stateRows = 0; ///< Rows in the stored state.
    Eigen::Index stateColumns = 0; ///< Columns in the stored state.
    size_t derivativeOffset = 0; ///< First scalar in a combined drift buffer.
    size_t derivativeCount = 0; ///< Drift scalars.
    Eigen::Index derivativeRows = 0; ///< Rows in drift views.
    Eigen::Index derivativeColumns = 0; ///< Columns in drift views.
    size_t diffusionCount = 0; ///< Scalars per local diffusion tangent.
    Eigen::Index diffusionRows = 0; ///< Rows in tangent views.
    Eigen::Index diffusionColumns = 0; ///< Columns in tangent views.
    size_t noiseCount = 0; ///< Local noise endpoints for this state.
    size_t localNoiseOffset = 0; ///< First entry in concatenated local-noise mappings.
    StateUpdateKind updateKind = StateUpdateKind::Euclidean; ///< Cached arithmetic dispatch choice.
    ErrorControlMode errorControl = ErrorControlMode::WholeState; ///< Cached adaptive scaling convention.
    const StateUpdatePolicy* specialUpdate = nullptr; ///< Borrowed immutable policy; null for Euclidean states.

    /** @brief Select ordinary vector arithmetic without virtual policy dispatch. */
    bool usesEuclideanUpdate() const noexcept { return this->updateKind == StateUpdateKind::Euclidean; }

    /** @brief Select scalar adaptive thresholds instead of one matrix-norm threshold. */
    bool usesPerComponentErrorControl() const noexcept { return this->errorControl == ErrorControlMode::PerComponent; }

    /** Maps this state's values in an already validated flat buffer. */
    ConstMatrixView stateView(const Eigen::VectorXd& buffer) const
    {
        return ConstMatrixView(buffer.data() + this->stateOffset, this->stateRows, this->stateColumns);
    }

    /** Maps this state's writable values in an already validated flat buffer. */
    MutableMatrixView stateView(Eigen::VectorXd& buffer) const
    {
        return MutableMatrixView(buffer.data() + this->stateOffset, this->stateRows, this->stateColumns);
    }

    /** Maps this state's drift in an already validated flat buffer. */
    ConstMatrixView derivativeView(const Eigen::VectorXd& buffer) const
    {
        return ConstMatrixView(buffer.data() + this->derivativeOffset, this->derivativeRows, this->derivativeColumns);
    }

    /** Maps a diffusion tangent at its validated packed offset. */
    ConstMatrixView diffusionView(const Eigen::VectorXd& buffer, size_t packedOffset) const
    {
        return ConstMatrixView(buffer.data() + packedOffset, this->diffusionRows, this->diffusionColumns);
    }
};

/** One contiguous Euclidean update span or one special-policy state. */
struct FlatUpdateRun
{
    size_t firstDescriptorIndex = 0; ///< First state in this run; the only state for a special update.
    size_t stateOffset = 0; ///< First scalar in a combined state buffer.
    size_t stateCount = 0; ///< State scalars covered by the run.
    size_t derivativeOffset = 0; ///< First scalar in a combined derivative buffer.
    size_t derivativeCount = 0; ///< Drift scalars covered by the run.
    StateUpdateKind updateKind = StateUpdateKind::Euclidean; ///< A vector-addition run or one policy-dispatched state.
};

/**
 * @brief Native bridge between named state registration and flat integrator arithmetic.
 *
 * bind() walks finalized registries once, validates shapes and policies, and builds
 * object/state descriptors plus contiguous Euclidean update runs. Descriptors borrow
 * buffers and policies; this class owns only topology metadata, not numerical stages.
 * validate() checks object identity and fixed layout before reuse.
 *
 * Gather/scatter operations copy complete object buffers. Candidate construction uses
 * vector addition for Euclidean runs and policy dispatch for special states. These
 * operations require correctly sized caller-owned buffers and do not resize them.
 */
class FlatStateBinding
{
  public:
    /** @brief Select validation and diagnostics for the consuming numerical family. */
    enum class Mode
    {
        DeterministicRungeKutta, ///< Drift-only RK candidate construction.
        Stochastic ///< Drift and diffusion candidate construction.
    };

    FlatStateBinding() = default;
    FlatStateBinding(const FlatStateBinding&) = delete;
    FlatStateBinding& operator=(const FlatStateBinding&) = delete;
    /** @brief Transfer prepared descriptors when a binding transaction commits. */
    FlatStateBinding(FlatStateBinding&&) noexcept = default;
    /** @brief Replace descriptors with a successfully prepared binding. */
    FlatStateBinding& operator=(FlatStateBinding&&) noexcept = default;

    /** Builds a binding from finalized managers. */
    void bind(const std::vector<DynamicObject*>& dynamics, Mode mode);

    /** Discards a binding assembled as part of an aborted transaction. */
    void reset() noexcept;

    /** Validates object/registry identity, finalized status, and fixed layout. */
    void validate(const std::vector<DynamicObject*>& dynamics) const;

    /** @brief Report whether descriptor construction committed. */
    bool isBound() const noexcept { return this->bound; }
    /** @brief Return combined state-buffer size in doubles. */
    size_t stateScalarCount() const noexcept { return this->stateCount; }
    /** @brief Return combined drift-buffer size in doubles. */
    size_t derivativeScalarCount() const noexcept { return this->derivativeCount; }
    /** @brief Return total state-local noise endpoints before shared-source grouping. */
    size_t localNoiseCount() const noexcept { return this->noiseCount; }
    /** @brief Report whether every state supports ordinary vector addition. */
    bool updatesAreAllEuclidean() const noexcept { return this->allEuclideanUpdates; }

    /** @brief Borrow object descriptors in synchronized integration order. */
    const std::vector<FlatObjectDescriptor>& objects() const noexcept { return this->objectDescriptors; }

    /** @brief Borrow state descriptors in object and registration order. */
    const std::vector<FlatStateDescriptor>& states() const noexcept { return this->stateDescriptors; }

    /** @brief Borrow coalesced Euclidean spans and individual special-state runs. */
    const std::vector<FlatUpdateRun>& updateRuns() const noexcept { return this->stateUpdateRuns; }

    /** @brief Borrow state indices in object order, then lexical state-name order. */
    const std::vector<size_t>& noiseTraversalOrder() const noexcept { return this->canonicalNoiseTraversalOrder; }

    /** Copies live state buffers into flat storage. */
    void gatherStates(Eigen::VectorXd& output) const
    {
        if (output.size() != static_cast<Eigen::Index>(this->stateCount)) {
            throw std::invalid_argument("Flat state buffer does not match the bound topology.");
        }
        for (const auto& object : this->objectDescriptors) {
            if (object.stateCount != 0) {
                std::copy_n(object.stateData, object.stateCount, output.data() + object.stateOffset);
            }
        }
    }

    /** Copies flat state storage into live buffers. */
    void scatterStates(const Eigen::VectorXd& input) const
    {
        if (input.size() != static_cast<Eigen::Index>(this->stateCount)) {
            throw std::invalid_argument("Flat state buffer does not match the bound topology.");
        }
        for (const auto& object : this->objectDescriptors) {
            if (object.stateCount != 0) {
                std::copy_n(input.data() + object.stateOffset, object.stateCount, object.stateData);
            }
        }
    }

    /** Copies live derivative buffers into flat storage. */
    void gatherDerivatives(Eigen::VectorXd& output) const { this->gatherDerivatives(output.data(), output.size()); }

    /** Copies live derivative buffers into caller-owned contiguous storage. */
    void gatherDerivatives(double* output, Eigen::Index outputSize) const
    {
        if (outputSize != static_cast<Eigen::Index>(this->derivativeCount)) {
            throw std::invalid_argument("Flat derivative buffer does not match the bound topology.");
        }
        for (const auto& object : this->objectDescriptors) {
            if (object.derivativeCount != 0) {
                std::copy_n(object.derivativeData, object.derivativeCount, output + object.derivativeOffset);
            }
        }
    }

    /** Builds one drift-only candidate in descriptor order. */
    void writeDriftCandidate(const Eigen::VectorXd& base,
                             const Eigen::VectorXd& drift,
                             double timeStep,
                             Eigen::Ref<Eigen::VectorXd> output) const;

  private:
    bool bound = false; ///< True after complete descriptor construction.
    Mode bindingMode = Mode::DeterministicRungeKutta; ///< Consumer family used for validation diagnostics.
    size_t stateCount = 0; ///< Total stored state scalars.
    size_t derivativeCount = 0; ///< Total drift scalars.
    size_t noiseCount = 0; ///< Total local noise endpoints.
    bool allEuclideanUpdates = true; ///< Enables whole-buffer arithmetic when no policy dispatch is needed.
    std::vector<FlatObjectDescriptor> objectDescriptors; ///< Borrowed buffer metadata in integration order.
    std::vector<FlatStateDescriptor> stateDescriptors; ///< Per-state metadata in registration order.
    std::vector<FlatUpdateRun> stateUpdateRuns; ///< Adjacent compatible Euclidean states coalesced for vector arithmetic.
    std::vector<size_t> canonicalNoiseTraversalOrder; ///< State indices sorted by object and name for stable noise assignment.
};

#endif /* flatStateBinding_h */
