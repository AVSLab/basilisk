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

/** @file svIntegratorAdaptiveRungeKutta.h
 * @brief Embedded RK error control and cached state tolerance configuration.
 */

#ifndef svIntegratorAdaptiveRungeKutta_h
#define svIntegratorAdaptiveRungeKutta_h

#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/svIntegratorRungeKutta.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <stdint.h>
#include <string>
#include <unordered_map>

/**
 * @brief Interface for integrators that support state-specific adaptive tolerances.
 *
 * This non-templated interface lets callers configure per-state tolerances
 * without needing to know the concrete Runge-Kutta stage count.
 */
class StateVecAdaptiveIntegrator {
  public:
    virtual ~StateVecAdaptiveIntegrator() = default;

    /** Sets the relative tolerance for the state identified by the given name. */
    virtual void setRelativeTolerance(std::string stateName, double relTol) = 0;

    /** Sets the absolute tolerance for the state identified by the given name. */
    virtual void setAbsoluteTolerance(std::string stateName, double absTol) = 0;
};

/**
 * Extends RKCoefficients with weights for the higher-order member of an embedded pair.
 * The inherited bArray contains the lower-order weights.
 *
 * The extended Butcher table looks like:
 *
 *   c | A
 *     -----
 *       b
 *     -----
 *       b*
 */
template <size_t numberStages> struct RKAdaptiveCoefficients : public RKCoefficients<numberStages> {

    /** "b*" Coefficients of the Adaptive RK method ("b" coefficients of the higher order method) */
    typename RKCoefficients<numberStages>::StageSizedArray bStarArray = {};
};

/**
 * @brief Explicit embedded RK integration with internal error-controlled substeps.
 *
 * integrateImpl() covers the entire requested interval, accepting the higher-order
 * candidate or reducing the internal step when its error ratio exceeds one. The
 * inherited baseState tracks accepted substeps; entryState preserves the start of the
 * requested interval for exception rollback.
 *
 * Tolerances resolve independently from object-and-state overrides, state-name
 * defaults, then global defaults. Cached descriptor-aligned spans avoid map lookups
 * during error evaluation. ErrorControlMode selects a whole-state norm or individual
 * scalar thresholds, including for states with special update policies.
 */
template <size_t numberStages>
class svIntegratorAdaptiveRungeKutta : public svIntegratorRungeKutta<numberStages>,
                                       public StateVecAdaptiveIntegrator {
  public:
    /**
     * Constructs the integrator for the given dynamic object and specified Adaptive RK
     * coefficients.
     *
     * methodLargestOrder is the higher order of the two
     * orders used in adaptive RK methods. For the RKF45 method,
     * for example, methodLargestOrder should be 5.
     */
    svIntegratorAdaptiveRungeKutta(DynamicObject* dynIn,
                                   const RKAdaptiveCoefficients<numberStages>& coefficients,
                                   const double methodLargestOrder);

  protected:
    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep
     */
    void integrateImpl(double currentTime, double timeStep) override;

    /** @brief Prepare RK stages, interval rollback, embedded candidates, and tolerance spans. */
    void prepareIntegrationBinding() override;

  public:
    /**
     * Sets the relative tolerance for every DynamicObject
     * and every state.
     *
     * This is akin to changing svIntegratorAdaptiveRungeKutta::relTol
     * directly
     */
    void setRelativeTolerance(double relTol);

    /**
     * Returns the relative tolerance for every DynamicObject
     * and every state.
     *
     * This is akin to reading svIntegratorAdaptiveRungeKutta::relTol
     * directly
     */
    double getRelativeTolerance();

    /**
     * Sets the absolute tolerance for every DynamicObject
     * and every state.
     *
     * This is akin to changing svIntegratorAdaptiveRungeKutta::absTol
     * directly
     */
    void setAbsoluteTolerance(double absTol);

    /**
     * Returns the absolute tolerance for every DynamicObject
     * and every state.
     *
     * This is akin to reading svIntegratorAdaptiveRungeKutta::absTol
     * directly
     */
    double getAbsoluteTolerance();

    /**
     * Sets the relative tolerance for every DynamicObject
     * and the state identified by the given name.
     */
    void setRelativeTolerance(std::string stateName, double relTol) override;

    /**
     * Returns the relative tolerance for every DynamicObject
     * and the state identified by the given name.
     *
     * If the relative tolerance for this state was not set, then
     * returns an empty optional.
     */
    std::optional<double> getRelativeTolerance(std::string stateName);

    /**
     * Sets the absolute tolerance for every DynamicObject
     * and the state identified by the given name.
     */
    void setAbsoluteTolerance(std::string stateName, double absTol) override;

    /**
     * Returns the absolute tolerance for every DynamicObject
     * and the state identified by the given name.
     *
     * If the absolute tolerance for this state was not set, then
     * returns an empty optional.
     */
    std::optional<double> getAbsoluteTolerance(std::string stateName);

    /**
     * Sets the relative tolerance for the given DynamicObject
     * and the state identified by the given name.
     */
    void
    setRelativeTolerance(const DynamicObject& dynamicObject, std::string stateName, double relTol);

    /**
     * Returns the relative tolerance for the given DynamicObject
     * and the state identified by the given name.
     *
     * If the relative tolerance for this state and DynamicObject was not set, then
     * returns an empty optional.
     */
    std::optional<double> getRelativeTolerance(const DynamicObject& dynamicObject,
                                               std::string stateName);

    /**
     * Sets the absolute tolerance for the given DynamicObject
     * and the state identified by the given name.
     */
    void
    setAbsoluteTolerance(const DynamicObject& dynamicObject, std::string stateName, double absTol);

    /**
     * Returns the absolute tolerance for the given DynamicObject
     * and the state identified by the given name.
     *
     * If the absolute tolerance for this state and DynamicObject was not set, then
     * returns an empty optional.
     */
    std::optional<double> getAbsoluteTolerance(const DynamicObject& dynamicObject,
                                               std::string stateName);

  public:
    /** Maximum relative truncation error allowed.
     *
     * The relative truncation error is the absolute error of the state divided by the magnitude of
     * the state.
     */
    double relTol = 1e-4;

    /** Maximum absolute truncation error allowed. */
    double absTol = 1e-8;

    /** When a new step size is computed, it is multiplied by this factor.
     *
     * A factor below one reduces the predicted step size to leave room for error-estimate
     * uncertainty. This can reduce rejected trials but does not guarantee acceptance.
     */
    double safetyFactorForNextStepSize = 0.9;

    /** When a new step size is computed, it can only be maximumFactorIncreaseForNextStepSize times
     * the old step size.
     */
    double maximumFactorIncreaseForNextStepSize = 4.0;

    /** When a new step size is computed, it must be above minimumFactorDecreaseForNextStepSize
     * times the old step size.
     */
    double minimumFactorDecreaseForNextStepSize = 0.1;

  protected:
    /** Validates that a DynamicObject is propagated by this integrator. */
    const DynamicObject* requireDynamicObject(const DynamicObject& dynamicObject) const;

    /** @brief Compute the largest error-to-tolerance ratio from embedded candidates.
     * Uses high-minus-low error scaled by absolute + relative * magnitude(high).
     * Whole-state scaling uses matrix norms; per-component scaling uses scalar magnitudes.
     * @return Maximum ratio; a trial is acceptable when this is at most one.
     */
    double computeFlatMaxRelativeError();

    /** @brief Refresh descriptor-aligned tolerances after configuration or binding changes.
     * @param force Rebuild even with unchanged tolerances when state descriptors have changed.
     */
    void resolveToleranceSpans(bool force = false);

    /** Rejects invalid tolerance values before they enter error scaling. */
    static void validateTolerance(double tolerance, const char* toleranceName);

    /** @brief Resolved tolerance and shape for one flat state, with no name lookup at use. */
    struct ToleranceSpan
    {
        size_t globalStateOffset; ///< First scalar in a combined state buffer.
        Eigen::Index rows; ///< Rows in this state's matrix.
        Eigen::Index columns; ///< Columns in this state's matrix.
        double relative; ///< Dimensionless resolved relative tolerance.
        double absolute; ///< Resolved absolute tolerance, in the state's units.
        ErrorControlMode mode; ///< Whole-state norm or per-component scaling.
    };

    /** @brief Independently optional overrides; absent values inherit the next default. */
    struct ToleranceOverride
    {
        std::optional<double> relative; ///< Optional dimensionless relative tolerance.
        std::optional<double> absolute; ///< Optional absolute tolerance in the state's units.
    };

    /** @brief Configuration indexed by registration name; resolved outside stage loops. */
    using StateToleranceOverrides = std::unordered_map<std::string, ToleranceOverride>;

    /** The higher order of the two orders used in adaptive RK methods.
     *
     * For the RKF45 method, for example, methodLargestOrder should be 5.
     */
    const double methodLargestOrder;
    const typename RKCoefficients<numberStages>::StageSizedArray bStarArray; ///< Higher-order candidate weights.

    /** State-name defaults shared by every synchronized dynamic object. */
    StateToleranceOverrides stateToleranceOverrides;

    /** Dynamic-object-specific state tolerance overrides. */
    std::unordered_map<const DynamicObject*, StateToleranceOverrides> objectToleranceOverrides;

    Eigen::VectorXd entryState; ///< Full-interval rollback, independent of accepted internal substeps.
    Eigen::VectorXd lowOrderState; ///< Embedded lower-order trial candidate.
    Eigen::VectorXd highOrderState; ///< Embedded higher-order trial, swapped into baseState on acceptance.
    std::vector<ToleranceSpan> toleranceSpans; ///< Resolved metadata in flat state order.
    uint64_t toleranceConfigurationGeneration = 0; ///< Advances when a tolerance setter changes configuration.
    uint64_t resolvedToleranceConfigurationGeneration = std::numeric_limits<uint64_t>::max(); ///< Cached revision.
    double resolvedRelativeTolerance = std::numeric_limits<double>::quiet_NaN(); ///< Detects direct relTol writes.
    double resolvedAbsoluteTolerance = std::numeric_limits<double>::quiet_NaN(); ///< Detects direct absTol writes.
};

template<size_t numberStages>
svIntegratorAdaptiveRungeKutta<numberStages>::svIntegratorAdaptiveRungeKutta(
  DynamicObject* dynIn,
  const RKAdaptiveCoefficients<numberStages>& coefficients,
  const double methodLargestOrder)
  : svIntegratorRungeKutta<numberStages>::svIntegratorRungeKutta(dynIn, coefficients)
  , methodLargestOrder(methodLargestOrder)
  , bStarArray(coefficients.bStarArray)
{
    if (!std::isfinite(this->methodLargestOrder) || this->methodLargestOrder <= 0.0) {
        throw std::invalid_argument("Adaptive Runge-Kutta order must be finite and positive.");
    }
    if (!std::all_of(
          this->bStarArray.cbegin(), this->bStarArray.cend(), [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Adaptive Runge-Kutta coefficients must all be finite.");
    }
}

template<size_t numberStages>
void
svIntegratorAdaptiveRungeKutta<numberStages>::prepareIntegrationBinding()
{
    try {
        svIntegratorRungeKutta<numberStages>::prepareIntegrationBinding();
        Eigen::VectorXd newEntryState(this->baseState.size());
        Eigen::VectorXd newLowOrderState(this->baseState.size());
        Eigen::VectorXd newHighOrderState(this->baseState.size());
        this->entryState.swap(newEntryState);
        this->lowOrderState.swap(newLowOrderState);
        this->highOrderState.swap(newHighOrderState);
        // A rebuilt binding can change state offsets and shapes without changing tolerance settings.
        this->resolveToleranceSpans(true);
    } catch (...) {
        this->entryState.resize(0);
        this->lowOrderState.resize(0);
        this->highOrderState.resize(0);
        this->toleranceSpans.clear();
        this->resolvedToleranceConfigurationGeneration = std::numeric_limits<uint64_t>::max();
        this->resolvedRelativeTolerance = std::numeric_limits<double>::quiet_NaN();
        this->resolvedAbsoluteTolerance = std::numeric_limits<double>::quiet_NaN();
        this->resetFlatStorage();
        throw;
    }
}

template<size_t numberStages>
void
svIntegratorAdaptiveRungeKutta<numberStages>::integrateImpl(double startingTime, double desiredTimeStep)
{
    if (!std::isfinite(startingTime) || !std::isfinite(desiredTimeStep) || desiredTimeStep < 0.0) {
        throw std::invalid_argument("Adaptive Runge-Kutta integration requires finite time and a "
                                    "nonnegative time step.");
    }
    const double endTime = startingTime + desiredTimeStep;
    if (!std::isfinite(endTime)) {
        throw std::invalid_argument("Adaptive Runge-Kutta integration end time is not finite.");
    }

    double elapsedTime = 0.0;
    double timeStep = desiredTimeStep;
    this->gatherStates(this->baseState);
    this->entryState = this->baseState;
    this->resolveToleranceSpans();
    bool liveStateIsBase = true;
    try {
        while (elapsedTime < desiredTimeStep) {
            if (!std::isfinite(timeStep) || timeStep <= 0.0 || elapsedTime + timeStep <= elapsedTime) {
                throw std::runtime_error("Adaptive Runge-Kutta integration cannot make representable "
                                         "forward progress.");
            }

            for (size_t stageIndex = 0; stageIndex < numberStages; ++stageIndex) {
                const auto& stageCoefficients = this->coefficients.aMatrix.at(stageIndex);
                const bool stageUsesBase =
                  stageIndex == 0 && std::none_of(stageCoefficients.cbegin(),
                                                  stageCoefficients.cend(),
                                                  [](double coefficient) { return coefficient != 0.0; });
                if (stageUsesBase) {
                    if (!liveStateIsBase) {
                        this->scatterStates(this->baseState);
                        liveStateIsBase = true;
                    }
                } else {
                    this->buildFlatCandidate(timeStep, stageCoefficients, stageIndex, this->candidateState);
                    this->scatterStates(this->candidateState);
                    liveStateIsBase = false;
                }

                const double stageTime =
                  startingTime + elapsedTime + this->coefficients.cArray.at(stageIndex) * timeStep;
                this->evaluateDerivatives(stageTime, timeStep);
                // User equations may mutate live state while producing derivatives.
                liveStateIsBase = false;
                this->gatherDerivatives(stageIndex);
            }

            this->buildFlatCandidate(timeStep, this->coefficients.bArray, numberStages, this->lowOrderState);
            this->buildFlatCandidate(timeStep, this->bStarArray, numberStages, this->highOrderState);

            const double maxRelError = this->computeFlatMaxRelativeError();
            if (maxRelError <= 1.0) {
                elapsedTime += timeStep;
                this->baseState.swap(this->highOrderState);
            }

            double newTimeStep = this->safetyFactorForNextStepSize * timeStep *
                                 std::pow(1.0 / maxRelError, 1.0 / this->methodLargestOrder);
            newTimeStep = std::min(newTimeStep, timeStep * this->maximumFactorIncreaseForNextStepSize);
            newTimeStep = std::max(newTimeStep, timeStep * this->minimumFactorDecreaseForNextStepSize);
            newTimeStep = std::min(newTimeStep, desiredTimeStep - elapsedTime);
            const double currentTime = startingTime + elapsedTime;
            if (elapsedTime < desiredTimeStep &&
                (!std::isfinite(newTimeStep) || newTimeStep <= 0.0 || elapsedTime + newTimeStep <= elapsedTime ||
                 (maxRelError > 1.0 && currentTime + newTimeStep <= currentTime))) {
                throw std::runtime_error("Adaptive Runge-Kutta error control reduced the time step "
                                         "below representable forward progress.");
            }
            timeStep = newTimeStep;
        }

        this->scatterStates(this->baseState);
    } catch (...) {
        this->scatterStates(this->entryState);
        throw;
    }
}

template<size_t numberStages>
double
svIntegratorAdaptiveRungeKutta<numberStages>::computeFlatMaxRelativeError()
{
    double maxRelativeError = 0.0;
    for (const auto& tolerance : this->toleranceSpans) {
        const auto offset = static_cast<Eigen::Index>(tolerance.globalStateOffset);
        Eigen::Map<Eigen::MatrixXd> truncationError(
          this->candidateState.data() + offset, tolerance.rows, tolerance.columns);
        const Eigen::Map<const Eigen::MatrixXd> lowOrderState(
          this->lowOrderState.data() + offset, tolerance.rows, tolerance.columns);
        const Eigen::Map<const Eigen::MatrixXd> highOrderState(
          this->highOrderState.data() + offset, tolerance.rows, tolerance.columns);

        truncationError = highOrderState - lowOrderState;

        if (tolerance.mode == ErrorControlMode::PerComponent) {
            for (Eigen::Index index = 0; index < highOrderState.size(); ++index) {
                if (!std::isfinite(highOrderState(index)) || !std::isfinite(lowOrderState(index)) ||
                    !std::isfinite(truncationError(index))) {
                    throw std::runtime_error("Adaptive Runge-Kutta produced a nonfinite state or "
                                             "truncation error.");
                }
                const double threshold = std::abs(highOrderState(index)) * tolerance.relative + tolerance.absolute;
                if (!std::isfinite(threshold) || threshold < 0.0) {
                    throw std::runtime_error("Adaptive Runge-Kutta produced an invalid error "
                                             "threshold.");
                }
                const double absoluteError = std::abs(truncationError(index));
                const double relativeError = threshold == 0.0
                                               ? (absoluteError == 0.0 ? 0.0 : std::numeric_limits<double>::infinity())
                                               : absoluteError / threshold;
                maxRelativeError = std::max(maxRelativeError, relativeError);
            }
        } else {
            const double errorNorm = truncationError.norm();
            const double highOrderNorm = highOrderState.norm();
            if (!std::isfinite(errorNorm) || !std::isfinite(highOrderNorm)) {
                throw std::runtime_error("Adaptive Runge-Kutta produced a nonfinite state or "
                                         "truncation error.");
            }
            const double threshold = highOrderNorm * tolerance.relative + tolerance.absolute;
            if (!std::isfinite(threshold) || threshold < 0.0) {
                throw std::runtime_error("Adaptive Runge-Kutta produced an invalid error threshold.");
            }
            const double relativeError = threshold == 0.0
                                           ? (errorNorm == 0.0 ? 0.0 : std::numeric_limits<double>::infinity())
                                           : errorNorm / threshold;
            maxRelativeError = std::max(maxRelativeError, relativeError);
        }
    }
    return maxRelativeError;
}

template<size_t numberStages>
void
svIntegratorAdaptiveRungeKutta<numberStages>::resolveToleranceSpans(bool force)
{
    const bool defaultTolerancesChanged =
      this->relTol != this->resolvedRelativeTolerance || this->absTol != this->resolvedAbsoluteTolerance;
    if (!force && this->resolvedToleranceConfigurationGeneration == this->toleranceConfigurationGeneration &&
        !defaultTolerancesChanged) {
        return;
    }
    this->validateTolerance(this->relTol, "relative tolerance");
    this->validateTolerance(this->absTol, "absolute tolerance");
    const auto& states = this->flatStateDescriptors();
    if (this->toleranceSpans.size() != states.size()) {
        this->toleranceSpans.resize(states.size());
    }

    for (size_t index = 0; index < states.size(); ++index) {
        const auto& descriptor = states[index];
        const std::string& stateName = descriptor.stateName;
        double relative = this->relTol;
        double absolute = this->absTol;

        const auto applyOverride = [&stateName, &relative, &absolute](const StateToleranceOverrides& overrides) {
            const auto stateOverride = overrides.find(stateName);
            if (stateOverride == overrides.cend()) {
                return;
            }
            if (stateOverride->second.relative.has_value()) {
                relative = *stateOverride->second.relative;
            }
            if (stateOverride->second.absolute.has_value()) {
                absolute = *stateOverride->second.absolute;
            }
        };
        applyOverride(this->stateToleranceOverrides);

        const auto objectOverride =
          this->objectToleranceOverrides.find(this->dynamics().at(descriptor.dynamicObjectIndex));
        if (objectOverride != this->objectToleranceOverrides.cend()) {
            applyOverride(objectOverride->second);
        }
        this->validateTolerance(relative, "relative tolerance");
        this->validateTolerance(absolute, "absolute tolerance");

        this->toleranceSpans[index] = { descriptor.stateOffset,
                                        descriptor.stateRows,
                                        descriptor.stateColumns,
                                        relative,
                                        absolute,
                                        descriptor.usesPerComponentErrorControl() ? ErrorControlMode::PerComponent
                                                                                  : ErrorControlMode::WholeState };
    }

    this->resolvedToleranceConfigurationGeneration = this->toleranceConfigurationGeneration;
    this->resolvedRelativeTolerance = this->relTol;
    this->resolvedAbsoluteTolerance = this->absTol;
}

template<size_t numberStages>
void
svIntegratorAdaptiveRungeKutta<numberStages>::validateTolerance(double tolerance, const char* toleranceName)
{
    if (!std::isfinite(tolerance) || tolerance < 0.0) {
        throw std::invalid_argument(std::string("Adaptive Runge-Kutta ") + toleranceName +
                                    " must be finite and nonnegative.");
    }
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setRelativeTolerance(double relTol)
{
    this->validateTolerance(relTol, "relative tolerance");
    this->relTol = relTol;
    ++this->toleranceConfigurationGeneration;
}

template <size_t numberStages>
double svIntegratorAdaptiveRungeKutta<numberStages>::getRelativeTolerance()
{
    return this->relTol;
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setAbsoluteTolerance(double absTol)
{
    this->validateTolerance(absTol, "absolute tolerance");
    this->absTol = absTol;
    ++this->toleranceConfigurationGeneration;
}

template <size_t numberStages>
double svIntegratorAdaptiveRungeKutta<numberStages>::getAbsoluteTolerance()
{
    return this->absTol;
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setRelativeTolerance(std::string stateName,
                                                                        double relTol)
{
    this->validateTolerance(relTol, "relative tolerance");
    this->stateToleranceOverrides[stateName].relative = relTol;
    ++this->toleranceConfigurationGeneration;
}

template <size_t numberStages>
std::optional<double>
svIntegratorAdaptiveRungeKutta<numberStages>::getRelativeTolerance(std::string stateName)
{
    const auto tolerance = this->stateToleranceOverrides.find(stateName);
    if (tolerance != this->stateToleranceOverrides.cend()) {
        return tolerance->second.relative;
    }
    return std::nullopt;
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setAbsoluteTolerance(std::string stateName,
                                                                        double absTol)
{
    this->validateTolerance(absTol, "absolute tolerance");
    this->stateToleranceOverrides[stateName].absolute = absTol;
    ++this->toleranceConfigurationGeneration;
}

template <size_t numberStages>
std::optional<double>
svIntegratorAdaptiveRungeKutta<numberStages>::getAbsoluteTolerance(std::string stateName)
{
    const auto tolerance = this->stateToleranceOverrides.find(stateName);
    if (tolerance != this->stateToleranceOverrides.cend()) {
        return tolerance->second.absolute;
    }
    return std::nullopt;
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setRelativeTolerance(
    const DynamicObject& dynamicObject,
    std::string stateName,
    double relTol)
{
    this->validateTolerance(relTol, "relative tolerance");
    const DynamicObject* dynamicObjectPointer = this->requireDynamicObject(dynamicObject);
    this->objectToleranceOverrides[dynamicObjectPointer][stateName].relative = relTol;
    ++this->toleranceConfigurationGeneration;
}

template <size_t numberStages>
inline std::optional<double> svIntegratorAdaptiveRungeKutta<numberStages>::getRelativeTolerance(
    const DynamicObject& dynamicObject,
    std::string stateName)
{
    const DynamicObject* dynamicObjectPointer = this->requireDynamicObject(dynamicObject);
    const auto objectTolerances = this->objectToleranceOverrides.find(dynamicObjectPointer);
    if (objectTolerances != this->objectToleranceOverrides.cend()) {
        const auto stateTolerance = objectTolerances->second.find(stateName);
        if (stateTolerance != objectTolerances->second.cend()) {
            return stateTolerance->second.relative;
        }
    }
    return std::nullopt;
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setAbsoluteTolerance(
    const DynamicObject& dynamicObject,
    std::string stateName,
    double absTol)
{
    this->validateTolerance(absTol, "absolute tolerance");
    const DynamicObject* dynamicObjectPointer = this->requireDynamicObject(dynamicObject);
    this->objectToleranceOverrides[dynamicObjectPointer][stateName].absolute = absTol;
    ++this->toleranceConfigurationGeneration;
}

template <size_t numberStages>
inline std::optional<double> svIntegratorAdaptiveRungeKutta<numberStages>::getAbsoluteTolerance(
    const DynamicObject& dynamicObject,
    std::string stateName)
{
    const DynamicObject* dynamicObjectPointer = this->requireDynamicObject(dynamicObject);
    const auto objectTolerances = this->objectToleranceOverrides.find(dynamicObjectPointer);
    if (objectTolerances != this->objectToleranceOverrides.cend()) {
        const auto stateTolerance = objectTolerances->second.find(stateName);
        if (stateTolerance != objectTolerances->second.cend()) {
            return stateTolerance->second.absolute;
        }
    }
    return std::nullopt;
}

template<size_t numberStages>
const DynamicObject*
svIntegratorAdaptiveRungeKutta<numberStages>::requireDynamicObject(const DynamicObject& dynamicObject) const
{
    auto it = std::find(this->dynamics().cbegin(), this->dynamics().cend(), &dynamicObject);
    if (it == this->dynamics().end()) {
        throw std::invalid_argument(
            "Given DynamicObject is not integrated by this integrator object");
    }
    return *it;
}

#endif /* svIntegratorAdaptiveRungeKutta_h */
