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

#ifndef svIntegratorAdaptiveRungeKutta_h
#define svIntegratorAdaptiveRungeKutta_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/svIntegratorRungeKutta.h"
#include <cmath>
#include <memory>
#include <optional>
#include <stdint.h>

/**
 * Extends RKCoefficients with "b" coefficients used for the lower order method.
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
 * The svIntegratorRungeKutta class implements a state integrator based on the
 * family of explicit Runge-Kutta numerical integrators with variable time step.
 *
 * When 'svIntegratorAdaptiveRungeKutta::integrate' is called, this integrator will
 * try to integrate with the given time step. It will then evaluate the error commited
 * and, if it's too large, internally use smaller time steps until the error tolerances
 * are met.
 */
template <size_t numberStages>
class svIntegratorAdaptiveRungeKutta : public svIntegratorRungeKutta<numberStages> {
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

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep
     */
    virtual void integrate(double currentTime, double timeStep) override;

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
    void setRelativeTolerance(std::string stateName, double relTol);

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
    void setAbsoluteTolerance(std::string stateName, double absTol);

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
     * Returns the absolute tolerance for he given DynamicObject
     * and the state identified by the given name.
     *
     * If the absolute tolerance for this state and DynamicObject was not set, then
     * returns an empty optional.
     */
    std::optional<double> getAbsoluteTolerance(const DynamicObject& dynamicObject,
                                               std::string stateName);

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
     * New step sizes are initially computed so that the error made using the new step matches
     * exactly the tolerance limits. By providing a safetyFactor < 0.9, we obtain a step size
     * the produces a lower error than the tolerance, thus almost guaranteen that we don't repeat
     * a step twice.
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
    /**
     * Computes the absolute error of every state
     * using the lower and higher order RK methods.
     *
     * Then, the norm of each state error is compared to the
     * error tolerance for that state. Greatest relation
     * between error and tolerance is returned,
     * which is the relation that defines the minimum acceptable
     * time step.
     */
    double computeMaxRelativeError(
        double timeStep,
        const ExtendedStateVector& lowOrderNextState,
        const ExtendedStateVector& highOrderNextState) const;

    /** Finds index of dynamicObject in dynPtrs (vector of pointers to DynamicObject) */
    size_t findDynamicObjectIndex(const DynamicObject& dynamicObject) const;

    /** Combines the relative and absolute tolerances into a single tolerance
     *
     * This checks for the general, state-specific, and
     * dynamicObject-state-specific tolerances; only the most specific
     * tolerance is used.
     */
    double
    getTolerance(size_t dynamicObjectIndex, const std::string& stateName, double stateNorm) const;

    /** The higher order of the two orders used in adaptive RK methods.
     *
     * For the RKF45 method, for example, methodLargestOrder should be 5.
     */
    const double methodLargestOrder;

    /** Holds the maximum relative truncation error allowed for specific states */
    std::unordered_map<std::string, double> stateSpecificRelTol;

    /** Holds the maximum absolute truncation error allowed for specific states */
    std::unordered_map<std::string, double> stateSpecificAbsTol;

    /** Holds the maximum relative truncation error allowed for specific states of specific dynamic
     * objects*/
    std::unordered_map<ExtendedStateId, double, ExtendedStateIdHash> dynObjectStateSpecificRelTol;

    /** Holds the maximum absolute truncation error allowed for specific states of specific dynamic
     * objects*/
    std::unordered_map<ExtendedStateId, double, ExtendedStateIdHash> dynObjectStateSpecificAbsTol;
};

template <size_t numberStages>
svIntegratorAdaptiveRungeKutta<numberStages>::svIntegratorAdaptiveRungeKutta(
    DynamicObject* dynIn,
    const RKAdaptiveCoefficients<numberStages>& coefficients,
    const double methodLargestOrder)
    : svIntegratorRungeKutta<numberStages>::svIntegratorRungeKutta(
          dynIn,
          (std::unique_ptr<RKCoefficients<numberStages>>)std::move(
              std::make_unique<RKAdaptiveCoefficients<numberStages>>(coefficients))),
      methodLargestOrder(methodLargestOrder)
{
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::integrate(double startingTime,
                                                             double desiredTimeStep)
{
    double time = startingTime;
    double timeStep = desiredTimeStep;
    ExtendedStateVector state = ExtendedStateVector::fromStates(this->dynPtrs);
    typename svIntegratorRungeKutta<numberStages>::KCoefficientsValues kValues;

    auto castCoefficients =
        static_cast<RKAdaptiveCoefficients<numberStages>*>(this->coefficients.get());

    // Continue until we are done with the desired time step
    while (time < startingTime + desiredTimeStep) {
        // Much like regular Runge Kutta, we compute the
        // "k" coefficients
        kValues = this->computeKCoefficients(time, timeStep, state);

        // Now we generate two solutions, one of low order and one of
        // high order by using either the b or b* coefficients
        ExtendedStateVector lowOrderNextStep =
            this->propagateStateWithKVectors(timeStep,
                                             state,
                                             kValues,
                                             castCoefficients->bArray,
                                             numberStages);
        ExtendedStateVector highOrderNextStep =
            this->propagateStateWithKVectors(timeStep,
                                             state,
                                             kValues,
                                             castCoefficients->bStarArray,
                                             numberStages);

        // For the adaptive RK, we also compute the maximum
        // relationship between error and tolerance
        double maxRelError = this->computeMaxRelativeError(timeStep, lowOrderNextStep, highOrderNextStep);

        // If maxRelError > 1, then we need a smaller time step,
        // so we should reject the current time step.
        // Otherwise, we can afford a greater time step for the next
        // integration, and this step was valid.
        if (maxRelError <= 1.) // Accept integration step
        {
            // Advance time and set new state to the computed state
            time += timeStep;
            state = std::move(highOrderNextStep);
        }

        // Regardless of accepting or not the step, we compute a new time step
        double newTimeStep = this->safetyFactorForNextStepSize * timeStep *
                             std::pow(1.0 / maxRelError, 1.0 / this->methodLargestOrder);
        newTimeStep = std::min(newTimeStep, timeStep * this->maximumFactorIncreaseForNextStepSize);
        newTimeStep = std::max(newTimeStep, timeStep * this->minimumFactorDecreaseForNextStepSize);
        newTimeStep =
            std::min(newTimeStep, startingTime + desiredTimeStep - time); // Avoid over-stepping
        timeStep = newTimeStep;
    }

    // Update the dynamic objects with the final state obtained
    state.setStates(this->dynPtrs);
}

template <size_t numberStages>
double svIntegratorAdaptiveRungeKutta<numberStages>::computeMaxRelativeError(
    double timeStep,
    const ExtendedStateVector& lowOrderNextStep,
    const ExtendedStateVector& highOrderNextStep) const
{
    // Compute the absolute truncation error for every state
    ExtendedStateVector truncationError = highOrderNextStep - lowOrderNextStep;

    // Compute the maximum relative error being committed
    // Each state has a truncationError (thisTruncationError),
    // and an acceptable error tolerance (a combination of relTol and absTol)
    // We care only about the largest relationship between
    // truncation error and tolerance.
    double maxRelativeError = 0;
    auto maxRelativeErrorRef = std::ref(maxRelativeError);
    highOrderNextStep.apply([this, &maxRelativeErrorRef, &truncationError](
                                 const size_t& dynObjIndex,
                                 const std::string& stateName,
                                 const Eigen::MatrixXd& thisState) {
        double thisTruncationError = truncationError.at({dynObjIndex, stateName}).norm();
        double thisErrorTolerance = this->getTolerance(dynObjIndex, stateName, thisState.norm());
        maxRelativeErrorRef.get() =
            std::max(maxRelativeErrorRef.get(), thisTruncationError / thisErrorTolerance);
    });

    return maxRelativeError;
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setRelativeTolerance(double relTol)
{
    this->relTol = relTol;
}

template <size_t numberStages>
double svIntegratorAdaptiveRungeKutta<numberStages>::getRelativeTolerance()
{
    return this->relTol;
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setAbsoluteTolerance(double absTol)
{
    this->absTol = absTol;
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
    this->stateSpecificRelTol.at(stateName) = relTol;
}

template <size_t numberStages>
std::optional<double>
svIntegratorAdaptiveRungeKutta<numberStages>::getRelativeTolerance(std::string stateName)
{
    if (this->stateSpecificRelTol.count(stateName) > 0) {
        return std::optional<double>(this->stateSpecificRelTol.at(stateName));
    }
    return std::optional<double>();
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setAbsoluteTolerance(std::string stateName,
                                                                        double absTol)
{
    this->stateSpecificAbsTol.at(stateName) = absTol;
}

template <size_t numberStages>
std::optional<double>
svIntegratorAdaptiveRungeKutta<numberStages>::getAbsoluteTolerance(std::string stateName)
{
    if (this->stateSpecificAbsTol.count(stateName) > 0) {
        return std::optional<double>(this->stateSpecificAbsTol.at(stateName));
    }
    return std::optional<double>();
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setRelativeTolerance(
    const DynamicObject& dynamicObject,
    std::string stateName,
    double relTol)
{
    this->dynObjectStateSpecificRelTol.at(
        {this->findDynamicObjectIndex(dynamicObject), stateName}) = relTol;
}

template <size_t numberStages>
inline std::optional<double> svIntegratorAdaptiveRungeKutta<numberStages>::getRelativeTolerance(
    const DynamicObject& dynamicObject,
    std::string stateName)
{
    const ExtendedStateId key = {this->findDynamicObjectIndex(dynamicObject), stateName};
    if (this->dynObjectStateSpecificRelTol.count(key) > 0) {
        return std::optional<double>(this->dynObjectStateSpecificRelTol.at(key));
    }
    return std::optional<double>();
}

template <size_t numberStages>
void svIntegratorAdaptiveRungeKutta<numberStages>::setAbsoluteTolerance(
    const DynamicObject& dynamicObject,
    std::string stateName,
    double absTol)
{
    this->dynObjectStateSpecificAbsTol.at(
        {this->findDynamicObjectIndex(dynamicObject), stateName}) = absTol;
}

template <size_t numberStages>
inline std::optional<double> svIntegratorAdaptiveRungeKutta<numberStages>::getAbsoluteTolerance(
    const DynamicObject& dynamicObject,
    std::string stateName)
{
    const ExtendedStateId key = {this->findDynamicObjectIndex(dynamicObject), stateName};
    if (this->dynObjectStateSpecificAbsTol.count(key) > 0) {
        return std::optional<double>(this->dynObjectStateSpecificAbsTol.at(key));
    }
    return std::optional<double>();
}

template <size_t numberStages>
size_t svIntegratorAdaptiveRungeKutta<numberStages>::findDynamicObjectIndex(
    const DynamicObject& dynamicObject) const
{
    auto it = std::find(this->dynPtrs.cbegin(), this->dynPtrs.cend(), &dynamicObject);
    if (it == this->dynPtrs.end()) {
        throw std::invalid_argument(
            "Given DynamicObject is not integrated by this integrator object");
    }
    return std::distance(this->dynPtrs.begin(), it);
}

template <size_t numberStages>
double svIntegratorAdaptiveRungeKutta<numberStages>::getTolerance(size_t dynamicObjectIndex,
                                                                  const std::string& stateName,
                                                                  double stateNorm) const
{
    const ExtendedStateId id{dynamicObjectIndex, stateName};

    double relTol{this->relTol};
    double absTol{this->absTol};

    if (this->dynObjectStateSpecificRelTol.count(id) > 0) {
        relTol = this->dynObjectStateSpecificRelTol.at(id);
    }
    else if (this->stateSpecificRelTol.count(stateName) > 0) {
        relTol = this->stateSpecificRelTol.at(stateName);
    }

    if (this->dynObjectStateSpecificAbsTol.count(id) > 0) {
        absTol = this->dynObjectStateSpecificAbsTol.at(id);
    }
    else if (this->stateSpecificAbsTol.count(stateName) > 0) {
        absTol = this->stateSpecificAbsTol.at(stateName);
    }

    return stateNorm * relTol + absTol;
}

#endif /* svIntegratorAdaptiveRungeKutta_h */
