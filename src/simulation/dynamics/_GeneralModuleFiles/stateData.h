/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

/** @file stateData.h
 * @brief State declarations, update-policy contract, and borrowed matrix access.
 */

#ifndef STATE_DATA_H
#define STATE_DATA_H

#include <Eigen/Dense>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/** @brief Shape of a matrix stored by the dynamics state registry. */
struct MatrixShape
{
    uint32_t rows = 0; ///< Number of matrix rows.
    uint32_t cols = 0; ///< Number of matrix columns.

    /** @brief Compare two matrix shapes. */
    bool operator==(const MatrixShape& other) const noexcept
    {
        return this->rows == other.rows && this->cols == other.cols;
    }

    /** @brief Compare two matrix shapes. */
    bool operator!=(const MatrixShape& other) const noexcept { return !(*this == other); }
};

/** @brief Adaptive-integrator error scaling associated with a state. */
enum class ErrorControlMode
{
    WholeState,   ///< Compare the norm of the state error against one state-level threshold.
    PerComponent ///< Compare each scalar error against its own scaled threshold.
};

/** @brief Kind of update applied when a state is propagated. */
enum class StateUpdateKind
{
    Euclidean, ///< State, drift, and diffusion share a shape and use ordinary addition.
    Special    ///< An immutable StateUpdatePolicy defines drift and noise updates.
};

/** @brief Writable column-major matrix alias; does not own or extend storage lifetime. */
using MutableMatrixView = Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned>;
/** @brief Read-only column-major matrix alias; does not own or extend storage lifetime. */
using ConstMatrixView = Eigen::Map<const Eigen::MatrixXd, Eigen::Unaligned>;

/** @brief Immutable topology supplied when a state is registered. */
struct StateSpec
{
    MatrixShape state;            ///< Shape of the stored physical state.
    MatrixShape derivative;       ///< Shape of drift returned by the equations of motion.
    MatrixShape diffusionTangent; ///< Shape of one local noise tangent, even when no noise is registered.
    size_t noiseCount = 0;        ///< Number of local sources; shared declarations may map them to global sources.
    ErrorControlMode errorControl = ErrorControlMode::WholeState; ///< Adaptive error-scaling convention.
    StateUpdateKind updateKind = StateUpdateKind::Euclidean;      ///< Selects ordinary addition or policy dispatch.

    /** @brief Compare two state specifications. */
    bool operator==(const StateSpec& other) const noexcept
    {
        return this->state == other.state && this->derivative == other.derivative &&
               this->diffusionTangent == other.diffusionTangent && this->noiseCount == other.noiseCount &&
               this->errorControl == other.errorControl && this->updateKind == other.updateKind;
    }

    /** @brief Compare two state specifications. */
    bool operator!=(const StateSpec& other) const noexcept { return !(*this == other); }
};

/**
 * @brief Defines propagation when a state's representation requires more than vector addition.
 *
 * The registry owns the policy; handles and integrators borrow it. Configuration is
 * immutable after registration, and topologyEquals() compares configuration by value
 * when a repeated Reset presents a replacement policy. A policy has no stage storage:
 * the integrator combines derivatives before calling it.
 *
 * Implementations must fully write drift output and apply noise increments in place.
 * Noise calls arrive in state-local source order, which matters for noncommuting updates.
 * Use the supplied views without resizing, retaining them, or changing registration.
 */
class StateUpdatePolicy
{
public:
  virtual ~StateUpdatePolicy() = default;

  /** @brief Compare immutable policy topology by value. */
  virtual bool topologyEquals(const StateUpdatePolicy& other) const = 0;

  /** @brief Validate that a state specification is supported by this policy. */
  virtual void validate(const StateSpec& spec) const = 0;

  /** @brief Construct one drift candidate from a base state and combined drift.
   * @param base State at the start of this candidate update.
   * @param combinedDrift Weighted drift in the declared derivative shape.
   * @param timeStep Drift multiplier in seconds.
   * @param output Destination in the declared state shape; must be fully written.
   */
  virtual void buildDriftCandidate(ConstMatrixView base,
                                   ConstMatrixView combinedDrift,
                                   double timeStep,
                                   MutableMatrixView output) const = 0;

  /** @brief Apply one diffusion-tangent increment to a candidate state.
   * @param state Candidate to update in place.
   * @param diffusionTangent One combined tangent in the declared diffusion shape.
   * @param pseudoStep Method-provided multiplier for this tangent, often a Wiener increment.
   */
  virtual void applyNoiseIncrement(MutableMatrixView state,
                                   ConstMatrixView diffusionTangent,
                                   double pseudoStep) const = 0;
};

class StateRegistry;

/**
 * @brief Stable borrowed handle to one registry-owned state and its drift and diffusion.
 *
 * Obtain handles from DynParamManager during registration and retain them for model
 * callbacks. Values begin in temporary matrices; the first finalization redirects
 * handles into contiguous buffers. Reacquire views after that allocation. Later resets
 * update live values in place and preserve both handle and buffer addresses.
 *
 * The manager must outlive native handles and views. Python wrappers retain the
 * owning manager. Copying getters return independent snapshots of current values.
 */
class StateData
{
  public:
    StateData(const StateData&) = delete;
    StateData& operator=(const StateData&) = delete;
    StateData(StateData&&) = delete;
    StateData& operator=(StateData&&) = delete;

    /** @brief Return the number of local noise sources, before shared-source grouping. */
    size_t getNumNoiseSources() const;

    /** @brief Set the registration-time number of independent noise sources.
     *
     * This deprecated compatibility API may establish the count during the
     * initial registration. Once topology is established, only the
     * already-established value is accepted.
     */
    void setNumNoiseSources(size_t numSources);

    /** @brief Set the state value, requiring an exact shape match. */
    void setState(Eigen::Ref<const Eigen::MatrixXd> newState);

    /** @brief Set the derivative value, requiring an exact shape match. */
    void setDerivative(Eigen::Ref<const Eigen::MatrixXd> newDeriv);

    /** @brief Return a mutable borrowed view of the active state storage.
     *
     * The view aliases the manager buffer active when it is acquired. Do not
     * retain it across the first finalizeStates(); reacquire it after storage
     * allocation. Later resets preserve its address.
     */
    MutableMatrixView stateView()
    {
        return MutableMatrixView(this->activeState, this->viewStateShape.rows, this->viewStateShape.cols);
    }

    /** @brief Return a constant borrowed view of the active state storage.
     *
     * The view follows the same storage-lifetime restrictions as the mutable
     * stateView().
     */
    ConstMatrixView stateView() const
    {
        return ConstMatrixView(this->activeState, this->viewStateShape.rows, this->viewStateShape.cols);
    }

    /** @brief Compatibility alias for stateView() on a constant state handle.
     * @deprecated Will be removed after September 30, 2027. Use stateView().
     */
    [[deprecated("Will be removed after 2027-09-30; use stateView().")]]
    ConstMatrixView getStateReference() const
    {
        return this->stateView();
    }

    /** @brief Return a mutable borrowed view of active derivative storage.
     *
     * The view aliases the manager buffer active when it is acquired and must
     * be reacquired after the first finalization.
     */
    MutableMatrixView derivativeView()
    {
        return MutableMatrixView(
          this->activeDerivative, this->viewDerivativeShape.rows, this->viewDerivativeShape.cols);
    }

    /** @brief Return a constant borrowed view of active derivative storage.
     *
     * The view follows the same storage-lifetime restrictions as the mutable
     * derivativeView().
     */
    ConstMatrixView derivativeView() const
    {
        return ConstMatrixView(this->activeDerivative, this->viewDerivativeShape.rows, this->viewDerivativeShape.cols);
    }

    /** @brief Compatibility alias for derivativeView() on a constant state handle.
     * @deprecated Will be removed after September 30, 2027. Use derivativeView().
     */
    [[deprecated("Will be removed after 2027-09-30; use derivativeView().")]]
    ConstMatrixView getStateDerivReference() const
    {
        return this->derivativeView();
    }

    /** @brief Return a mutable borrowed view of one diffusion tangent.
     *
     * The view aliases the manager buffer active when it is acquired and must
     * be reacquired after the first finalization.
     */
    MutableMatrixView diffusionView(size_t localNoiseIndex);

    /** @brief Return a constant borrowed view of one diffusion tangent.
     *
     * The view follows the same storage-lifetime restrictions as the mutable
     * diffusionView().
     */
    ConstMatrixView diffusionView(size_t localNoiseIndex) const;

    /** @brief Return the live state-buffer pointer.
     *
     * Raw access is valid only while the owning manager is finalized.
     */
    double* stateData();

    /** @brief Return the live derivative-buffer pointer. */
    double* derivativeData();

    /** @brief Return one live diffusion-buffer pointer. */
    double* diffusionData(size_t localNoiseIndex);

    /** @brief Set one diffusion tangent, requiring an exact shape match.
     *
     * @param newDiffusion New diffusion tangent.
     * @param index Local noise-source index.
     */
    void setDiffusion(Eigen::Ref<const Eigen::MatrixXd> newDiffusion, size_t index);

    /** @brief Return a copy of the active state value. */
    Eigen::MatrixXd getState() const;

    /** @brief Return a copy of the active derivative value. */
    Eigen::MatrixXd getStateDeriv() const;

    /** @brief Return a copy of one active diffusion tangent. */
    Eigen::MatrixXd getStateDiffusion(size_t index) const;

    /** @brief Return the state name. */
    std::string getName() const;

    /** @brief Return whether adaptive error is measured per scalar component. */
    bool usesPerComponentErrorControl() const;

    /** @brief Return the state shape. */
    MatrixShape stateShape() const;

    /** @brief Return the derivative shape. */
    MatrixShape derivativeShape() const;

    /** @brief Return the diffusion tangent shape. */
    MatrixShape diffusionShape() const;

    /** @brief Return the state row count. */
    uint32_t getRowSize() const { return this->stateShape().rows; }

    /** @brief Return the state column count. */
    uint32_t getColumnSize() const { return this->stateShape().cols; }

    /** @brief Return the derivative row count. */
    uint32_t getDerivativeRowSize() const { return this->derivativeShape().rows; }

    /** @brief Return the derivative column count. */
    uint32_t getDerivativeColumnSize() const { return this->derivativeShape().cols; }

    /** @brief Return the diffusion tangent row count. */
    uint32_t getDiffusionRowSize() const { return this->diffusionShape().rows; }

    /** @brief Return the diffusion tangent column count. */
    uint32_t getDiffusionColumnSize() const { return this->diffusionShape().cols; }

  private:
    friend class StateRegistry;
    friend struct std::default_delete<StateData>;

    /** @brief Create a stable handle for a registry slot and cache its immutable view shapes. */
    StateData(StateRegistry* owner, size_t slot, const StateSpec& spec);
    ~StateData() = default;

    /** @brief Rebind borrowed views when the manager changes the active buffer. */
    void bindStateViews(double* state, double* derivative) noexcept
    {
        this->activeState = state;
        this->activeDerivative = derivative;
    }

    StateRegistry* owner = nullptr; //!< Registry owning this handle's metadata and storage
    size_t slot = 0;                  //!< Stable registration slot
    double* activeState = nullptr;    //!< Active seed or buffer state storage
    double* activeDerivative = nullptr; //!< Active seed or buffer derivative storage
    const MatrixShape viewStateShape;      ///< Immutable shape cached for inline state maps.
    const MatrixShape viewDerivativeShape; ///< Immutable shape cached for inline derivative maps.
};

#endif /* STATE_DATA_H */
