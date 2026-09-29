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
 * @file stateRegistry.h
 * @brief Native state-registration lifecycle, immutable layouts, and buffer access.
 */

#ifndef STATE_REGISTRY_H
#define STATE_REGISTRY_H

#include "stateData.h"
#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class DynParamManager;
class StateRegistry;

/** @brief Dimensions and scalar offsets of one state in the committed buffers.
 *
 * Offsets count doubles, not bytes. Matrices use Eigen's column-major ordering.
 * State, derivative, and diffusion shapes may differ for a special update policy.
 * Layouts are immutable after the first successful registration.
 */
struct StateLayout
{
    uint32_t stateRows = 0; ///< Rows in the state matrix.
    uint32_t stateCols = 0; ///< Columns in the state matrix.
    size_t stateOffset = 0; ///< First state scalar in StateBuffers::states.
    size_t stateCount = 0;  ///< Number of state scalars, equal to rows times columns.

    uint32_t derivRows = 0; ///< Rows in the time-derivative matrix.
    uint32_t derivCols = 0; ///< Columns in the time-derivative matrix.
    size_t derivOffset = 0; ///< First derivative scalar in StateBuffers::derivatives.
    size_t derivCount = 0;  ///< Number of derivative scalars.

    uint32_t diffusionRows = 0;         ///< Rows in each local noise tangent.
    uint32_t diffusionCols = 0;         ///< Columns in each local noise tangent.
    size_t diffusionOffset = 0;         ///< First tangent scalar in StateBuffers::diffusions.
    size_t diffusionCountPerSource = 0; ///< Scalars per tangent; local sources occupy consecutive blocks.
    size_t noiseCount = 0;              ///< Number of local noise sources declared for this state.

    ErrorControlMode errorControl = ErrorControlMode::WholeState; ///< Adaptive error scaling for this record.
    StateUpdateKind updateKind = StateUpdateKind::Euclidean;      ///< Ordinary addition or a special update rule.
    const StateUpdatePolicy* specialUpdate = nullptr; ///< Borrowed immutable policy, or nullptr for Euclidean states.
};

/** @brief Contiguous state, derivative, and diffusion buffers allocated at finalization.
 * Their addresses remain stable for the registry's lifetime.
 */
struct StateBuffers
{
    Eigen::VectorXd states;      ///< State values in registration order.
    Eigen::VectorXd derivatives; ///< Time derivatives in registration order.
    Eigen::VectorXd diffusions;  ///< Noise tangents grouped by state, then by local source.
};

/** @brief Select the buffer addressed by a StateBufferSegment. */
enum class StateBufferKind
{
    State,      ///< State values.
    Derivative, ///< Time derivatives.
    Diffusion   ///< All local noise tangents, grouped by state and then by source.
};

/** @brief Consecutive complete records in one registry's buffer.
 * The registry must outlive the segment and every pointer resolved from it.
 */
struct StateBufferSegment
{
    const StateRegistry* registry = nullptr; ///< Owner of the referenced storage.
    StateBufferKind bufferKind = StateBufferKind::State; ///< Buffer containing the scalar range.
    size_t offset = 0; ///< First scalar in the buffer.
    size_t count = 0; ///< Number of consecutive scalars.
};

/**
 * @brief State declarations and contiguous storage owned by DynParamManager.
 *
 * Registration creates named handles with temporary matrices. finalizeStates()
 * computes offsets, allocates the buffers, and redirects those handles to the
 * contiguous storage. This transition happens once. Later declarations look up
 * the existing state by name and must match its dimensions and update policy.
 * Values can be reset directly without moving storage or rebuilding integrators.
 *
 * Setup is not transactional. An exception may leave partially initialized values;
 * callers must complete setup successfully before running the simulation.
 */
class StateRegistry
{
  public:
    using NoiseEndpoint = std::pair<size_t, size_t>; ///< Registration slot and local noise-source index.
    using NoiseGroup = std::vector<NoiseEndpoint>;   ///< Endpoints driven by one shared process.
    using NoiseTopology = std::vector<NoiseGroup>;   ///< Canonically sorted shared-noise groups.

    /** @brief Release states, their update policies, and the buffers. */
    ~StateRegistry() = default;
    StateRegistry(const StateRegistry&) = delete;
    StateRegistry& operator=(const StateRegistry&) = delete;
    StateRegistry(StateRegistry&&) = delete;
    StateRegistry& operator=(StateRegistry&&) = delete;

    /** @brief Report whether the fixed layout and contiguous buffers are available. */
    bool statesAreFinalized() const noexcept { return this->finalized; }
    /** @brief Return the number of named states. */
    size_t getStateCount() const noexcept { return this->stateRecords.size(); }

    /** @brief Borrow handles in registration order; requires finalized registration. */
    const std::vector<StateData*>& getStateRegistrationOrder() const;
    /** @brief Borrow immutable layouts in registration order; requires finalized registration. */
    const std::vector<StateLayout>& getStateLayouts() const;
    /** @brief Borrow canonical shared-noise groups; requires finalized registration. */
    const NoiseTopology& getSharedNoiseTopology() const;

    /**
     * @brief Issue a segment of any buffer spanning complete state records.
     * @param kind State, derivative, or diffusion storage.
     * @param firstSlot Zero-based registration slot at the start of the span.
     * @param elementCount Number of scalars, ending at a complete record boundary.
     * A diffusion record includes every local noise tangent of one state;
     * states without noise contribute no scalars.
     * @return Segment referring to this registry's fixed storage.
     * @throws std::exception If storage is not finalized, kind is invalid, or the span is invalid.
     * @note Diffusion segments retain state/local-source order. They do not group shared sources.
     */
    StateBufferSegment getSegment(StateBufferKind kind, size_t firstSlot, size_t elementCount) const;

    /**
     * @brief Borrow a segment of any buffer as a writable column matrix.
     * @param segment Segment previously issued by this registry.
     * @return View with segment.count rows and one column; no values are copied.
     * The view remains valid until the registry is destroyed.
     * @throws std::logic_error If storage is not finalized or the segment is invalid.
     */
    MutableMatrixView segmentView(const StateBufferSegment& segment);
    /**
     * @brief Borrow a segment of any buffer as a read-only column matrix.
     * @param segment Segment previously issued by this registry.
     * @return View with segment.count rows and one column, valid while the registry exists.
     * @throws std::logic_error If storage is not finalized or the segment is invalid.
     */
    ConstMatrixView segmentView(const StateBufferSegment& segment) const;

    /**
     * @brief Issue a segment spanning complete state records.
     * @param firstSlot Zero-based registration slot at the start of the span.
     * @param elementCount Total number of scalars, ending at a record boundary.
     * @return Segment referring to this registry's fixed storage.
     * @throws std::exception If registration is not finalized or the span is invalid.
     */
    StateBufferSegment getStateSegment(size_t firstSlot, size_t elementCount) const;
    /**
     * @brief Issue a segment spanning complete derivative records.
     * @param firstSlot Zero-based registration slot at the start of the span.
     * @param elementCount Total derivative scalars, ending at a record boundary.
     * @return Derivative segment owned by this registry.
     * @throws std::exception If registration is not finalized or the span is invalid.
     */
    StateBufferSegment getDerivativeSegment(size_t firstSlot, size_t elementCount) const;
    /**
     * @brief Issue a segment spanning complete diffusion records.
     * @param firstSlot Zero-based registration slot at the start of the span.
     * @param elementCount Total scalars across all local tangents of the selected states.
     * @return Diffusion segment in state/local-source order. @see getSegment()
     */
    StateBufferSegment getDiffusionSegment(size_t firstSlot, size_t elementCount) const;

    /**
     * @brief Resolve a live state segment without allocating or copying.
     * @param segment Segment previously issued by this registry.
     * @return Borrowed pointer into the committed state buffer.
     * @throws std::logic_error If storage is not finalized or the segment has the
     * wrong owner, kind, or bounds.
     */
    double* stateSegmentData(const StateBufferSegment& segment);
    /** @brief Resolve a state segment for read-only access. @see stateSegmentData() */
    const double* stateSegmentData(const StateBufferSegment& segment) const;
    /**
     * @brief Resolve a live derivative segment without allocating or copying.
     * @param segment Derivative segment previously issued by this registry.
     * @return Borrowed pointer into the committed derivative buffer.
     * @throws std::logic_error If storage is not finalized or the segment is invalid.
     */
    double* derivativeSegmentData(const StateBufferSegment& segment);
    /** @brief Resolve a derivative segment for read-only access. @see derivativeSegmentData() */
    const double* derivativeSegmentData(const StateBufferSegment& segment) const;
    /**
     * @brief Resolve a live diffusion segment without allocating or copying.
     * @param segment Diffusion segment previously issued by this registry.
     * @return Borrowed pointer into diffusion storage; may be null for an empty buffer.
     * @throws std::logic_error If storage is not finalized or the segment is invalid.
     */
    double* diffusionSegmentData(const StateBufferSegment& segment);
    /**
     * @brief Resolve a diffusion segment for read-only access.
     * @param segment Diffusion segment previously issued by this registry.
     * @return Borrowed pointer into diffusion storage; may be null for an empty buffer.
     * @throws std::logic_error If storage is not finalized or the segment is invalid.
     */
    const double* diffusionSegmentData(const StateBufferSegment& segment) const;

  private:
    friend class DynParamManager;
    friend class StateData;
    /** @brief Construct a registry that accepts new state declarations. */
    StateRegistry() = default;

    /** @brief Temporary matrices used only before buffer dimensions are established. */
    struct StateSeed
    {
        Eigen::MatrixXd state;                   ///< Initial state values, writable through the new handle.
        Eigen::MatrixXd derivative;              ///< Initial derivative values.
        std::vector<Eigen::MatrixXd> diffusions; ///< Initial tangent for each local noise source.
    };

    /** @brief Own one state's identity, immutable declaration, and optional update rule. */
    struct StateRecord
    {
        std::unique_ptr<StateData> handle; ///< Stable module-facing access object.
        std::string name;                  ///< Unique lookup key and repeated-registration identity.
        StateSpec spec;                    ///< Frozen shape, noise count, and integration choices.
        std::unique_ptr<StateSeed> seed;   ///< Present during first collection; released after finalization.
        std::unique_ptr<StateUpdatePolicy> updatePolicy; ///< Owned immutable special policy, or nullptr.
    };

    /** @brief Allocate fixed storage once; subsequent calls have no effect. */
    void finalizeStates();
    /** @brief Declare a state with Euclidean updates and the supplied dimensions and noise count. */
    StateData* registerState(std::string stateName, const StateSpec& spec);
    /** @brief Declare a special-policy state, taking ownership of the policy. */
    StateData* registerState(std::string stateName, const StateSpec& spec, std::unique_ptr<StateUpdatePolicy> policy);
    /** @brief Preserve the legacy equal-shape declaration and its repeated-registration defaults. */
    StateData* registerState(uint32_t nRow, uint32_t nCol, std::string stateName);
    /** @brief Find a handle without logging; the manager reports a missing name. */
    StateData* getStateObject(const std::string& stateName) const;
    /** @brief Stage shared-noise endpoints after checking ownership and local source bounds. */
    void registerSharedNoiseSource(std::vector<std::pair<const StateData&, size_t>> sharedNoises);

    /** @brief Create a named state or return its existing handle after validating the declaration. */
    StateData* registerManagedState(std::string stateName,
                                    const StateSpec& spec,
                                    std::unique_ptr<StateUpdatePolicy> policy);
    /** @brief Check nonzero shapes, safe storage sizes, and compatibility with the update policy. */
    void validateSpec(const std::string& stateName, const StateSpec& spec, const StateUpdatePolicy* policy) const;
    /** @brief Sort shared-noise groups and reject ambiguous memberships. */
    NoiseTopology canonicalNoiseTopology() const;
    /** @brief Compute offsets and allocate buffers from the initial declarations. */
    void allocateBuffers();
    /** @brief Throw with the operation name unless live buffers are available for integration. */
    void requireFinalized(const char* operation) const;

    /** @brief Look up the immutable name of a valid handle's registration slot. */
    const std::string& handleName(size_t slot) const;
    /** @brief Borrow the declaration associated with a valid handle's slot. */
    const StateSpec& handleSpec(size_t slot) const { return this->stateRecords.at(slot).spec; }
    /** @brief Support registration-time noise-count changes; established topology accepts only its existing count. */
    void setHandleNoiseCount(size_t slot, size_t numSources);
    /** @brief Resolve one diffusion tangent in temporary matrices or the finalized buffer. */
    double* activeDiffusionData(size_t slot, size_t localNoiseIndex);
    /** @brief Resolve one diffusion tangent for read-only access. */
    const double* activeDiffusionData(size_t slot, size_t localNoiseIndex) const;
    /** @brief Resolve one record in finalized live state storage. */
    double* liveStateData(size_t slot);
    /** @brief Resolve one record in finalized live derivative storage. */
    double* liveDerivativeData(size_t slot);
    /** @brief Resolve one local source in finalized live diffusion storage. */
    double* liveDiffusionData(size_t slot, size_t localNoiseIndex);
    /** @brief Reject buffer access before storage has been allocated. */
    void requireRawAccess(const char* field) const;

    bool finalized = false; ///< True after the one-time layout and buffer allocation.
    std::unordered_map<std::string, size_t> stateSlotByName; ///< Name to stable registration slot.
    std::vector<StateRecord> stateRecords; ///< Owned handles, declarations, initial matrices, and policies.
    std::vector<StateData*> registrationOrderView; ///< Handles in first-declaration order.
    std::vector<StateLayout> stateLayoutView; ///< Fixed shapes and scalar offsets.
    StateBuffers liveBuffers; ///< Values used by modules and integrators; never resized after finalization.
    NoiseTopology finalizedNoiseTopology; ///< Canonical shared-noise groups fixed at finalization.
    NoiseTopology pendingNoiseGroups; ///< Shared-noise declarations collected before finalization.
};

#endif /* STATE_REGISTRY_H */
