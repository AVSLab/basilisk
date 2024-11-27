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

#ifndef svIntegratorRungeKutta_h
#define svIntegratorRungeKutta_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stateVecIntegrator.h"
#include "extendedStateVector.h"
#include <array>
#include <functional>
#include <memory>
#include <stdint.h>
#include <unordered_map>

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
 * Note that the order of the integrator is lower or equal to the stage number.
 * A RK method of order 5, for example, requires 7 stages.
 */
template <size_t numberStages> class svIntegratorRungeKutta : public StateVecIntegrator {
  public:
    static_assert(numberStages > 0, "One cannot declare Runge Kutta integrators of stage 0");

    /** Creates an explicit RK integrator for the given DynamicObject using the passed
     * coefficients.*/
    svIntegratorRungeKutta(DynamicObject* dynIn, const RKCoefficients<numberStages>& coefficients);

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep
     */
    virtual void integrate(double currentTime, double timeStep) override;

  protected:
    /**
     * Can be used by subclasses to support passing coefficients
     * that are subclasses of RKCoefficients
     */
    svIntegratorRungeKutta(DynamicObject* dynIn,
                           std::unique_ptr<RKCoefficients<numberStages>>&& coefficients);

    /**
     * For an s-stage RK method, s number of "k" coefficients are
     * needed, where each "k" coefficient has the same size as the state.
     * This type allows us to store these "k" coefficients.
     */
    using KCoefficientsValues = std::array<ExtendedStateVector, numberStages>;

    /**
     * Computes the derivatives of every state given a time and current states.
     *
     * Internally, this sets the states on the dynamic objects and
     * calls the equationsOfMotion methods.
     */
    ExtendedStateVector
    computeDerivatives(double time, double timeStep, const ExtendedStateVector& states);

    /**
     * Computes the "k" coefficients of the Runge-Kutta method
     * for a time and state.
     */
    KCoefficientsValues computeKCoefficients(double currentTime,
                                             double timeStep,
                                             const ExtendedStateVector& currentStates);

    /**
     * Computes:
     *
     *      y = y_0 + dt*( coeff_0*k_0 + coeff_1*k_1 + ... + coeff_maxStage*k_maxStage )
     *
     * where y_0 is currentStates, dt is timeStep, [coeff_0, coeff_1, ...] is coefficients
     * and [k_0, k_1, ...] is kVectors.
     */
    ExtendedStateVector
    propagateStateWithKVectors(double timeStep,
                               const ExtendedStateVector& currentStates,
                               const KCoefficientsValues& kVectors,
                               const std::array<double, numberStages>& coefficients,
                               size_t maxStage);

  protected:
    // coefficients is stored as a pointer to support polymorphism
    /** Coefficients to be used in the method */
    const std::unique_ptr<RKCoefficients<numberStages>> coefficients;
};

template <size_t numberStages>
svIntegratorRungeKutta<numberStages>::svIntegratorRungeKutta(
    DynamicObject* dynIn,
    const RKCoefficients<numberStages>& coefficients)
    : StateVecIntegrator(dynIn),
      coefficients(std::make_unique<RKCoefficients<numberStages>>(coefficients))
{
}

template <size_t numberStages>
svIntegratorRungeKutta<numberStages>::svIntegratorRungeKutta(
    DynamicObject* dynIn,
    std::unique_ptr<RKCoefficients<numberStages>>&& coefficients)
    : StateVecIntegrator(dynIn), coefficients(std::move(coefficients))
{
}

template <size_t numberStages>
void svIntegratorRungeKutta<numberStages>::integrate(double currentTime, double timeStep)
{
    ExtendedStateVector currentState = ExtendedStateVector::fromStates(this->dynPtrs);
    KCoefficientsValues kValues = this->computeKCoefficients(currentTime, timeStep, currentState);
    ExtendedStateVector nextState = this->propagateStateWithKVectors(timeStep,
                                                                     currentState,
                                                                     kValues,
                                                                     this->coefficients->bArray,
                                                                     numberStages);
    nextState.setStates(this->dynPtrs);
}

template <size_t numberStages>
ExtendedStateVector
svIntegratorRungeKutta<numberStages>::computeDerivatives(double time,
                                                         double timeStep,
                                                         const ExtendedStateVector& states)
{
    states.setStates(this->dynPtrs);

    for (auto dynPtr : this->dynPtrs) {
        dynPtr->equationsOfMotion(time, timeStep);
    }

    return ExtendedStateVector::fromStateDerivs(this->dynPtrs);
}

template <size_t numberStages>
auto svIntegratorRungeKutta<numberStages>::computeKCoefficients(
    double currentTime,
    double timeStep,
    const ExtendedStateVector& currentStates) -> KCoefficientsValues
{
    KCoefficientsValues kVectors;

    for (size_t stageIndex = 0; stageIndex < numberStages; stageIndex++) {
        double timeToComputeK = currentTime + this->coefficients->cArray.at(stageIndex) * timeStep;

        auto stateToComputeTheDerivatesAt =
            this->propagateStateWithKVectors(timeStep,
                                             currentStates,
                                             kVectors,
                                             this->coefficients->aMatrix.at(stageIndex),
                                             stageIndex);
        kVectors.at(stageIndex) =
            this->computeDerivatives(timeToComputeK, timeStep, stateToComputeTheDerivatesAt);
    }

    return kVectors;
}

template <size_t numberStages>
ExtendedStateVector svIntegratorRungeKutta<numberStages>::propagateStateWithKVectors(
    double timeStep,
    const ExtendedStateVector& currentStates,
    const KCoefficientsValues& kVectors,
    const std::array<double, numberStages>& coefficients,
    size_t maxStage)
{
    ExtendedStateVector derivative;
    for (size_t stageIndex = 0; stageIndex < maxStage; stageIndex++) {
        if (coefficients.at(stageIndex) == 0) continue;

        auto scaledKVector = kVectors.at(stageIndex) * coefficients.at(stageIndex);
        if (derivative.empty()) {
            derivative = scaledKVector;
        }
        else {
            derivative += scaledKVector;
        }
    }

    if (derivative.empty()) {
        return currentStates;
    }
    else {
        currentStates.setStates(this->dynPtrs);
        derivative.setDerivatives(this->dynPtrs);

        for (auto dynPtr : this->dynPtrs) {
            dynPtr->dynManager.propagateStateVector(timeStep);
        }

        return ExtendedStateVector::fromStates(this->dynPtrs);
    }
}

#endif /* svIntegratorRungeKutta_h */
