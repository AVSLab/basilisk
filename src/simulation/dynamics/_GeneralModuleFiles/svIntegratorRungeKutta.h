/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

/** @file svIntegratorRungeKutta.h
 * @brief Explicit Runge-Kutta stages over finalized flat state storage.
 */

#ifndef svIntegratorRungeKutta_h
#define svIntegratorRungeKutta_h

#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/flatStateBinding.h"
#include "../_GeneralModuleFiles/stateVecIntegrator.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <stdint.h>
#include <unordered_map>
#include <vector>

#include <iostream>

/**
 * Stores the coefficients necessary to use the Runge-Kutta methods.
 *
 * The Butcher table looks like:
 *
 *   c | A
 *     -----
 *       b
 */
template <size_t numberStages> struct RKCoefficients {

    /** Array with size = numberStages */
    using StageSizedArray = std::array<double, numberStages>;

    /** Square matrix with size = numberStages */
    using StageSizedMatrix = std::array<StageSizedArray, numberStages>;

    // = {} performs zero-initialization
    StageSizedMatrix aMatrix = {}; /**< "a" coefficient matrix of the RK method */
    StageSizedArray bArray = {};   /**< "b" coefficient array of the RK method */
    StageSizedArray cArray = {};   /**< "c" coefficient array of the RK method */
};

/**
 * The svIntegratorRungeKutta class implements a state integrator based on the
 * family of explicit Runge-Kutta numerical integrators.
 *
 * A Runge-Kutta method is defined by its stage number and its coefficients.
 * The stage number drives the computational cost of the method: an RK method
 * of stage 4 requires 4 dynamics evaluations (FSAL optimizations are not done).
 *
 * The integrator binds finalized registries through FlatStateBinding and owns one
 * contiguous derivative column per stage. Adjacent Euclidean states share vector
 * arithmetic; special states use their immutable StateUpdatePolicy. A single
 * all-Euclidean object can expose its live buffer as the candidate buffer.
 *
 * baseState preserves step-entry values for candidate construction and exception
 * rollback. Derived adaptive methods reuse the stage machinery but manage accepted
 * internal steps separately.
 */
template <size_t numberStages> class svIntegratorRungeKutta : public StateVecIntegrator {
  public:
    static_assert(numberStages > 0, "One cannot declare Runge Kutta integrators of stage 0");

    /** Creates an explicit RK integrator for the given DynamicObject using the passed
     * coefficients.*/
    svIntegratorRungeKutta(DynamicObject* dynIn, const RKCoefficients<numberStages>& coefficients);

  protected:
    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep
     */
    void integrateImpl(double currentTime, double timeStep) override;

    /** @brief Validate topology and allocate stage storage before first use. */
    void prepareIntegrationBinding() override { this->bindFlatStorage(); }
    /** @brief Check that descriptors still address the same finalized buffers. */
    void validateIntegrationBinding() const override { this->validateFlatBinding(); }
    /** @brief Build descriptors and buffers transactionally, or validate an existing binding. */
    void bindFlatStorage();
    /** @brief Discard descriptors and workspace after failed preparation. */
    void resetFlatStorage() noexcept;
    /** @brief Validate object identity and fixed layout through FlatStateBinding. */
    void validateFlatBinding() const;
    /** @brief Copy live state buffers into an already sized combined buffer. */
    void gatherStates(Eigen::VectorXd& output) const;
    /** @brief Make an already sized combined candidate visible to dynamics callbacks. */
    void scatterStates(const Eigen::VectorXd& input) const;
    /** @brief Capture live derivative buffers in the selected stage column. */
    void gatherDerivatives(size_t stageIndex);
    /** @brief Build a policy-aware candidate from baseState and captured stages.
     * @param timeStep Drift multiplier in seconds.
     * @param stageCoefficients Weights for the captured derivative columns.
     * @param maxStage Number of initialized columns available to this combination.
     * @param output Already sized destination; may alias a live Euclidean buffer.
     */
    void buildFlatCandidate(double timeStep,
                            const std::array<double, numberStages>& stageCoefficients,
                            size_t maxStage,
                            Eigen::Ref<Eigen::VectorXd> output);
    /** @brief Borrow state shapes and offsets, also used by adaptive tolerance resolution. */
    const std::vector<FlatStateDescriptor>& flatStateDescriptors() const noexcept { return this->flatBinding.states(); }
    /** @brief Borrow contiguous Euclidean spans and individual special-state runs. */
    const std::vector<FlatUpdateRun>& flatStateUpdateRuns() const noexcept { return this->flatBinding.updateRuns(); }

  protected:
    /** Coefficients to be used in the method */
    const RKCoefficients<numberStages> coefficients;

  protected:
    FlatStateBinding flatBinding; ///< Owns descriptors and borrows finalized registry storage.
    Eigen::VectorXd baseState; ///< Accepted base for stage construction and fixed-step rollback.
    Eigen::VectorXd candidateState; ///< Candidate scratch when live-buffer construction is unavailable.
    Eigen::VectorXd combinedDerivative; ///< Weighted drift used by policy-aware candidate construction.
    Eigen::MatrixXd kStorage; ///< Derivative scalars by RK stage, with each stage contiguous.
};

template<size_t numberStages>
svIntegratorRungeKutta<numberStages>::svIntegratorRungeKutta(DynamicObject* dynIn,
                                                             const RKCoefficients<numberStages>& coefficients)
  : StateVecIntegrator(dynIn)
  , coefficients(coefficients)
{
    const auto requireFinite = [](double value) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("Runge-Kutta coefficients must all be finite.");
        }
    };
    for (size_t rowIndex = 0; rowIndex < numberStages; ++rowIndex) {
        const auto& row = this->coefficients.aMatrix[rowIndex];
        std::for_each(row.cbegin(), row.cend(), requireFinite);
        for (size_t columnIndex = rowIndex; columnIndex < numberStages; ++columnIndex) {
            if (row[columnIndex] != 0.0) {
                throw std::invalid_argument("Explicit Runge-Kutta A coefficients must be strictly "
                                            "lower triangular.");
            }
        }
    }
    std::for_each(this->coefficients.bArray.cbegin(), this->coefficients.bArray.cend(), requireFinite);
    std::for_each(this->coefficients.cArray.cbegin(), this->coefficients.cArray.cend(), requireFinite);
}

template<size_t numberStages>
void
svIntegratorRungeKutta<numberStages>::integrateImpl(double currentTime, double timeStep)
{
    this->gatherStates(this->baseState);

    try {
        if constexpr (numberStages == 1) {
            if (this->flatBinding.updatesAreAllEuclidean()) {
                this->evaluateDerivatives(currentTime + this->coefficients.cArray[0] * timeStep, timeStep);
                if (this->coefficients.bArray[0] == 0.0) {
                    this->scatterStates(this->baseState);
                } else {
                    for (const auto& object : this->flatBinding.objects()) {
                        if (object.stateCount == 0) {
                            continue;
                        }
                        const auto count = static_cast<Eigen::Index>(object.stateCount);
                        Eigen::Map<Eigen::VectorXd> state(object.stateData, count);
                        const Eigen::Map<const Eigen::VectorXd> drift(object.derivativeData, count);
                        const Eigen::Map<const Eigen::VectorXd> base(this->baseState.data() + object.stateOffset,
                                                                     count);
                        state = base + drift * (timeStep * this->coefficients.bArray[0]);
                    }
                }
                return;
            }
        }
        // A single Euclidean buffer is already a contiguous candidate buffer.
        const bool candidateIsLive =
          this->flatBinding.updatesAreAllEuclidean() && this->flatBinding.objects().size() == 1;
        Eigen::Map<Eigen::VectorXd> candidate(candidateIsLive ? this->flatBinding.objects().front().stateData
                                                              : this->candidateState.data(),
                                              this->baseState.size());
        for (size_t stageIndex = 0; stageIndex < numberStages; ++stageIndex) {
            const auto& stageCoefficients = this->coefficients.aMatrix.at(stageIndex);
            const bool reuseGatheredBase =
              stageIndex == 0 && std::none_of(stageCoefficients.cbegin(),
                                              stageCoefficients.cend(),
                                              [](double coefficient) { return coefficient != 0.0; });
            if (!reuseGatheredBase) {
                this->buildFlatCandidate(timeStep, stageCoefficients, stageIndex, candidate);
                if (!candidateIsLive) {
                    this->scatterStates(this->candidateState);
                }
            }

            const double stageTime = currentTime + this->coefficients.cArray.at(stageIndex) * timeStep;
            this->evaluateDerivatives(stageTime, timeStep);
            this->gatherDerivatives(stageIndex);
        }

        this->buildFlatCandidate(timeStep, this->coefficients.bArray, numberStages, candidate);
        if (!candidateIsLive) {
            this->scatterStates(this->candidateState);
        }
    } catch (...) {
        this->scatterStates(this->baseState);
        throw;
    }
}

template<size_t numberStages>
void
svIntegratorRungeKutta<numberStages>::bindFlatStorage()
{
    if (this->flatBinding.isBound() && this->flatBinding.objects().size() == this->dynamics().size()) {
        this->validateFlatBinding();
        return;
    }

    FlatStateBinding newBinding;
    newBinding.bind(this->dynamics(), FlatStateBinding::Mode::DeterministicRungeKutta);
    const size_t stateCount = newBinding.stateScalarCount();
    const size_t derivativeCount = newBinding.derivativeScalarCount();
    Eigen::VectorXd newBaseState(static_cast<Eigen::Index>(stateCount));
    Eigen::VectorXd newCandidateState(static_cast<Eigen::Index>(stateCount));
    Eigen::VectorXd newCombinedDerivative(static_cast<Eigen::Index>(derivativeCount));
    Eigen::MatrixXd newKStorage(static_cast<Eigen::Index>(derivativeCount), static_cast<Eigen::Index>(numberStages));

    this->flatBinding = std::move(newBinding);
    this->baseState.swap(newBaseState);
    this->candidateState.swap(newCandidateState);
    this->combinedDerivative.swap(newCombinedDerivative);
    this->kStorage.swap(newKStorage);
}

template<size_t numberStages>
void
svIntegratorRungeKutta<numberStages>::validateFlatBinding() const
{
    this->flatBinding.validate(this->dynamics());
}

template<size_t numberStages>
void
svIntegratorRungeKutta<numberStages>::resetFlatStorage() noexcept
{
    this->flatBinding.reset();
    Eigen::VectorXd().swap(this->baseState);
    Eigen::VectorXd().swap(this->candidateState);
    Eigen::VectorXd().swap(this->combinedDerivative);
    Eigen::MatrixXd().swap(this->kStorage);
}

template<size_t numberStages>
void
svIntegratorRungeKutta<numberStages>::gatherStates(Eigen::VectorXd& output) const
{
    if (output.size() != this->baseState.size()) {
        throw std::invalid_argument("Flat Runge-Kutta state buffer has an invalid size.");
    }
    this->flatBinding.gatherStates(output);
}

template<size_t numberStages>
void
svIntegratorRungeKutta<numberStages>::scatterStates(const Eigen::VectorXd& input) const
{
    if (input.size() != this->baseState.size()) {
        throw std::invalid_argument("Flat Runge-Kutta state buffer has an invalid size.");
    }
    this->flatBinding.scatterStates(input);
}

template<size_t numberStages>
void
svIntegratorRungeKutta<numberStages>::gatherDerivatives(size_t stageIndex)
{
    auto stage = this->kStorage.col(static_cast<Eigen::Index>(stageIndex));
    this->flatBinding.gatherDerivatives(stage.data(), stage.size());
}

template<size_t numberStages>
void
svIntegratorRungeKutta<numberStages>::buildFlatCandidate(double timeStep,
                                                         const std::array<double, numberStages>& stageCoefficients,
                                                         size_t maxStage,
                                                         Eigen::Ref<Eigen::VectorXd> output)
{
    if (output.size() != this->baseState.size()) {
        throw std::invalid_argument("Flat Runge-Kutta candidate buffer has an invalid size.");
    }

    bool haveTerm = false;
    for (size_t stageIndex = 0; stageIndex < maxStage; ++stageIndex) {
        const double coefficient = stageCoefficients.at(stageIndex);
        if (coefficient == 0.0) {
            continue;
        }

        const auto stageDerivative = this->kStorage.col(static_cast<Eigen::Index>(stageIndex));
        if (!haveTerm) {
            this->combinedDerivative = stageDerivative * coefficient;
        } else {
            this->combinedDerivative += stageDerivative * coefficient;
        }
        haveTerm = true;
    }

    if (!haveTerm) {
        output = this->baseState;
        return;
    }
    if (this->flatBinding.updatesAreAllEuclidean()) {
        output = this->baseState + this->combinedDerivative * timeStep;
        return;
    }
    this->flatBinding.writeDriftCandidate(this->baseState,
                                          this->combinedDerivative,
                                          timeStep,
                                          output);
}

#endif /* svIntegratorRungeKutta_h */
