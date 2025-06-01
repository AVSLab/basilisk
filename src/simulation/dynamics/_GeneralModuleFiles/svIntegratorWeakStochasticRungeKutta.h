/*
 ISC License

Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef svIntegratorWeakStochasticRungeKutta_h
#define svIntegratorWeakStochasticRungeKutta_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stateVecStochasticIntegrator.h"
#include "extendedStateVector.h"

#include <array>
#include <functional>
#include <memory>
#include <stdint.h>
#include <unordered_map>
#include <random>
#include <iostream>

/**
 * Stores the coefficients necessary to use the Stochastic Runge-Kutta method.
 *
 * The Butcher table looks like:
 *
 *   c0 |  A0   |  B0   |
 *   c1 |  A1   |  B1   |  B2
 *  ----|---------------------
 *      | alpha | beta0 | beta1
 *
 * See more in the description of svIntegratorWeakStochasticRungeKutta.
 */
template <size_t numberStages> struct SRKCoefficients {

    /** Array with size = numberStages */
    using StageSizedArray = std::array<double, numberStages>;

    /** Square matrix with size = numberStages */
    using StageSizedMatrix = std::array<StageSizedArray, numberStages>;

    // = {} performs zero-initialization
    StageSizedArray   alpha  = {}; /**< "alpha" coefficient array of the SRK method */
    StageSizedArray   beta0  = {}; /**< "beta0" coefficient array of the SRK method */
    StageSizedArray   beta1  = {}; /**< "beta1" coefficient array of the SRK method */
    StageSizedMatrix  A0     = {}; /**< "A0" coefficient matrix of the SRK method */
    StageSizedMatrix  B0     = {}; /**< "B0" coefficient matrix of the SRK method */
    StageSizedMatrix  A1     = {}; /**< "A1" coefficient matrix of the SRK method */
    StageSizedMatrix  B1     = {}; /**< "B1" coefficient matrix of the SRK method */
    StageSizedMatrix  B2     = {}; /**< "B2" coefficient matrix of the SRK method */

    /** "c0" coefficient array of the SRK method
     *
     * c0 is the row sum of the A(0) matrix:
     *
     *     c0 = sum_j ​a(0)_ij
     */
    StageSizedArray   c0     = {};

    /** "c1" coefficient array of the SRK method
     *
     * c1 is the row sum of the A(1) matrix:
     *
     *     c1 = sum_j ​a(1)_ij
     */
    StageSizedArray   c1     = {};
};

/** Random variables used in the Weak Stochastic Integrator */
struct SRKRandomVariables
{
    Eigen::VectorXd Ik; /**< See Eq 3.2 in Tang & Xiao */
    Eigen::MatrixXd Ikl; /**< See Eq 3.3 in Tang & Xiao */
    Eigen::VectorXd Ikk; /**< Same as Ikl(k,k) */
    double xi; /**< See Eq 3.2 in Tang & Xiao */
};

/** Random Generator for the integrator described in:
 *
 *     Tang, X., Xiao, A. Efficient weak second-order stochastic Runge–Kutta methods
 *     for Itô stochastic differential equations. Bit Numer Math 57, 241–260 (2017).
 *     https://doi.org/10.1007/s10543-016-0618-9
 */
class SRKRandomVariableGenerator
{
public:

    /** Sets the seed to the RNG */
    inline void setSeed(size_t seed) {rng.seed(seed);}

    /** Generates tha random values necessary for one step
     * of the integrator.abs_y_is_huge
     *
     * @param m number of noise sources
     * @param h time step, in seconds
     */
    inline SRKRandomVariables generate(size_t m, double h)
    {
        // ensure there's no hidden state on the RVs so setting the seed
        // is always consistent
        uniform_rv.reset();
        bernoulli_rv.reset();

        SRKRandomVariables result;
        result.Ik.resize(m);
        result.Ikl.resize(m, m);
        result.Ikk.resize(m);

        // 1) sample I_k per (3.2)
        for(size_t k=0; k<m; ++k)
        {
            double u = uniform_rv(rng);

            if(u < 1.0/6.0)      result.Ik(k) =  std::sqrt(3*h);
            else if(u < 2.0/6.0) result.Ik(k) = -std::sqrt(3*h);
            else                 result.Ik(k) =  0.0;
        }
        // xi = eta1 * sqrt(h)
        result.xi = (bernoulli_rv(rng) ? 1.0 : -1.0) * std::sqrt(h);
        // eta2 for cross-integrals
        int eta2 = bernoulli_rv(rng) ? +1 : -1;

        // 2) build I_{k,l} and I_{k,k} per (3.3)
        for(size_t k=0; k<m; ++k)
        {
            for(size_t l=0; l<m; ++l)
            {
                if(k<l)      result.Ikl(k,l) = 0.5*(result.Ik(l) - eta2 * result.Ik(k));
                else if(k>l) result.Ikl(k,l) = 0.5*(result.Ik(l) + eta2 * result.Ik(k));
                else         result.Ikl(k,l) = 0.5*(result.Ik(k)*result.Ik(k) - h);
            }

            result.Ikk(k) = result.Ikl(k,k);
        }

        return result;
    }

protected:
    /** Random Number Generator */
    std::mt19937 rng{std::random_device{}()};

    /** Uniform random variable, from 0 to 1 */
    std::uniform_real_distribution<double> uniform_rv;

    /** Bernoulli random variable (either 0 or 1, with equal probability) */
    std::bernoulli_distribution bernoulli_rv;
};

/**
 * The svIntegratorWeakStochasticRungeKutta class implements a state integrator
 * that provides weak solutions to problems with stochastic dynamics (SDEs).
 *
 * The method is described in:
 *
 *     Tang, X., Xiao, A. Efficient weak second-order stochastic Runge–Kutta methods
 *     for Itô stochastic differential equations. Bit Numer Math 57, 241–260 (2017).
 *     https://doi.org/10.1007/s10543-016-0618-9
 *
 * The method described in said paper is designed for autonomous systems (where
 * f and g do not depend on time), so we need to modify it for non-autonomous systems.
 * This is simple: we just need to treat the time as a state (with f=1 and g=0).
 *
 * The resulting pseudocode for the (explicit) method is (note that the order of the
 * sums and some elements has moved around to reflect the code):
 *
 *  --- (3.1*) stage definitions ---
 *  for i = 1..s:
 *      // drift‐stage
 *      H0[i] = y_n
 *          + h * sum_{j=1..i-1}( a0[i][j] * f( t_n + c0[j]*h,   H0[j] ) )
 *          + sum_{k=1..m}( Ihat[k] * sum_{j=1..i-1}( b0[i][j] * g[k]( t_n + c1[j]*h,   Hk[j] ) ) )
 *
 *      // diffusion‐stages, one per noise source k
 *      for k = 1..m:
 *          Hk[i] = y_n
 *              + h  * sum_{j=1..i-1}( a1[i][j] * f(    t_n + c0[j]*h, H0[j] ) )
 *              + xi * sum_{j=1..i-1}( b1[i][j] * g[k]( t_n + c1[j]*h, Hk[j] ) )
 *              + sum_{l=1..m, l!=k}( Ihat_kl[k][l] * sum_{j=1..i-1}( b2[i][j] * g[l]( t_n + c1[i]*h, Hl[i] ) ) );
 *
 *  where
 *    c0[j] = sum_{p=1..s}( a0[j][p] )
 *    c1[j] = sum_{p=1..s}( a1[j][p] )
 *
 *  --- (3.2) driving‐variable distributions ---
 *  For each k:
 *      P( Ihat[k] = +sqrt(3*h) ) = 1/6
 *      P( Ihat[k] = -sqrt(3*h) ) = 1/6
 *      P( Ihat[k] =  0        ) = 2/3
 *
 *  xi = eta1 * sqrt(h)    // eta1 is {+1,−1} with equal prob.
 *
 *  --- (3.3) mixed‐integral approximations ---
 *  for each pair (k,l):
 *      if   k < l:
 *          Ihat_kl[k][l] = 0.5 * ( Ihat[l] - eta2 * Ihat[l] )
 *      else if k > l:
 *          Ihat_kl[k][l] = 0.5 * ( Ihat[l] + eta2 * Ihat[l] )
 *      else // k == l:
 *          Ihat_kl[k][k] = 0.5 * ( Ihat[k]*Ihat[k] * xi - xi )
 *
 *  --- (3.1*) state evolution ---
 *  y_n+1 = y_n
 *          + h * sum_{i=1..s}( alpha[i] * f( t_n + c0[i]*h, H0[i] ) )
 *          + sum_{k=1..m}( Ihat[k]       * sum_{i=1..s}( beta0[i] * g[k]( t_n + c1[i]*h, Hk[i] ) ) )
 *          + sum_{k=1..m}( Ihat_kl[k][k] * sum_{i=1..s}( beta1[i] * g[k]( t_n + c1[i]*h, Hk[i] ) ) )
 */
template <size_t numberStages> class svIntegratorWeakStochasticRungeKutta : public StateVecStochasticIntegrator {
public:
    static_assert(numberStages > 0, "One cannot declare Runge Kutta integrators of stage 0");

    /** Creates an explicit RK integrator for the given DynamicObject using the passed
     * coefficients.*/
    svIntegratorWeakStochasticRungeKutta(DynamicObject* dynIn, const SRKCoefficients<numberStages>& coefficients);

    /** Sets the seed for the Random Number Generator used by this integrator.
     *
     * As a stochastic integrator, random numbers are drawn during each time step.
     * By default, a randomly generated seed is used each time.
     *
     * If the seed is set, the integrator will always draw the same numbers
     * during time-stepping.
     */
    inline void setRNGSeed(size_t seed) {rvGenerator.setSeed(seed);}

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    virtual void integrate(double currentTime, double timeStep) override;

public:
    /** Random Number Generator for the integrator */
    SRKRandomVariableGenerator rvGenerator;

protected:
    /**
     * Can be used by subclasses to support passing coefficients
     * that are subclasses of SRKCoefficients
     */
    svIntegratorWeakStochasticRungeKutta(DynamicObject* dynIn,
                        std::unique_ptr<SRKCoefficients<numberStages>>&& coefficients);

    /**
     * Computes the derivatives of every state given a time and current states.
     *
     * Internally, this sets the states on the dynamic objects and
     * calls the equationsOfMotion methods.
     */
    ExtendedStateVector
    computeDerivatives(double time, double timeStep);

    /**
     * Computes the diffusions of every state given a time and current states.
     *
     * Internally, this sets the states on the dynamic objects and
     * calls the equationsOfMotionDiffusion methods.
     */
    std::vector<ExtendedStateVector>
    computeDiffusions(double time, double timeStep, const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps);

    /**
     * Computes the diffusions for the states and noise index
     * in ``stateIdToNoiseIndexMap`` given a time and current states.
     *
     * Internally, this sets the states on the dynamic objects and
     * calls the equationsOfMotionDiffusion methods.
     */
    ExtendedStateVector computeDiffusions(double time,
                                          double timeStep,
                                          const StateIdToIndexMap& stateIdToNoiseIndexMap);

    /** Utility function, computes:
     *
     * result = sum_{i=0..length-1} factors[i]*vectors[i]
     */
    ExtendedStateVector scaledSum(const std::array<double, numberStages>& factors, const std::array<ExtendedStateVector, numberStages>& vectors, size_t length);

protected:
    // coefficients is stored as a pointer to support polymorphism
    /** Coefficients to be used in the method */
    const std::unique_ptr<SRKCoefficients<numberStages>> coefficients;
};

template <size_t numberStages>
svIntegratorWeakStochasticRungeKutta<numberStages>::svIntegratorWeakStochasticRungeKutta(
    DynamicObject* dynIn,
    const SRKCoefficients<numberStages>& coefficients)
    : StateVecStochasticIntegrator(dynIn),
    coefficients(std::make_unique<SRKCoefficients<numberStages>>(coefficients))
{
}

template <size_t numberStages>
svIntegratorWeakStochasticRungeKutta<numberStages>::svIntegratorWeakStochasticRungeKutta(
    DynamicObject* dynIn,
    std::unique_ptr<SRKCoefficients<numberStages>>&& coefficients)
    : StateVecStochasticIntegrator(dynIn), coefficients(std::move(coefficients))
{
}

template <size_t numberStages>
void svIntegratorWeakStochasticRungeKutta<numberStages>::integrate(double currentTime, double timeStep)
{
    ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);

    // this is a map (ExtendedStateId -> noise index) for each of the noise sources
    // (so the length of this vector should be m)
    std::vector<StateIdToIndexMap> stateIdToNoiseIndexMaps = getStateIdToNoiseIndexMaps();

    // Sample the random variables
    auto rv = rvGenerator.generate(stateIdToNoiseIndexMaps.size(), timeStep);

    std::array<ExtendedStateVector, numberStages> f_H0; // f(H_i^(0))   for i=0..s-1
    std::vector<std::array<ExtendedStateVector, numberStages>> g_Hk; // g^k(H_i^(k))   for i=0..s-1; k=0..m-1

    // i = 0 (first stage) we do here becase we can optimize it
    // Note that current state == H_0^(0) == H_0^(k)
    f_H0.at(0) = computeDerivatives(
        currentTime,
        timeStep
    );

    std::vector<ExtendedStateVector> diffs = computeDiffusions(
        currentTime,
        timeStep,
        stateIdToNoiseIndexMaps
    );
    for (auto&& diff : std::move(diffs))
    {
        g_Hk.emplace_back().at(0) = std::move(diff);
    }

    for (size_t i = 1; i < numberStages; i++)
    {
        /* // drift‐stage
        *      H0[i] = y_n
        *          + h * sum_{j=1..i-1}( a0[i][j] * f( t_n + c0[j]*h,   H0[j] ) )
        *          + sum_{k=1..m}( Ihat[k] * sum_{j=1..i-1}( b0[i][j] * g[k]( t_n + c1[j]*h,   Hk[j] ) ) )
        */

        // y_n
        //    and store the state in the dynPtrs
        currentState.setStates(dynPtrs);

        // sum_{j=1..i-1}( a0[i][j] * f( t_n + c0[j]*h,   H0[j] ) )
        //    and store the derivative in the dynPtrs
        scaledSum(coefficients->A0.at(i), f_H0, i).setDerivatives(dynPtrs);

        // sum_{k=1..m}( Ihat[k] * sum_{j=1..i-1}( b0[i][j] * g[k]( t_n + c1[j]*h,   Hk[j] ) ) )
        for (size_t k = 0; k < stateIdToNoiseIndexMaps.size(); k++)
        {
            // sum_{j=1..i-1}( b0[i][j] * g[k]( t_n + c1[j]*h,   Hk[j] ) )
            //     and store the diffusion for the noise source k in the dynPtrs
            scaledSum(coefficients->B0.at(i), g_Hk.at(k), i).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
        }

        // This sets the current state to H_i^(0)
        propagateState(timeStep, rv.Ik, stateIdToNoiseIndexMaps);

        // This computes f(t_n + c0[i]*h, H0[i]) for the next steps
        f_H0.at(i) = computeDerivatives(currentTime + coefficients->c0.at(i)*timeStep, timeStep);


        /* // diffusion‐stages, one per noise source k
        *      for k = 1..m:
        *          Hk[i] = y_n
        *              + h  * sum_{j=1..i-1}( a1[i][j] * f(    t_n + c0[j]*h, H0[j] ) )
        *              + xi * sum_{j=1..i-1}( b1[i][j] * g[k]( t_n + c1[j]*h, Hk[j] ) )
        *              + sum_{l=1..m, l!=k}( Ihat_kl[k][l] * sum_{j=1..i-1}( b2[i][j] * g[l]( t_n + c1[i]*h, Hl[i] ) ) );
        */
        for (size_t k = 0; k < stateIdToNoiseIndexMaps.size(); k++)
        {
            // y_n
            //    and store the state in the dynPtrs
            currentState.setStates(dynPtrs);

            // sum_{j=1..i-1}( a0[i][j] * f( t_n + c0[j]*h,   H0[j] ) )
            //    and store the derivative in the dynPtrs
            scaledSum(coefficients->A1.at(i), f_H0, i).setDerivatives(dynPtrs);

            // sum_{j=1..i-1}( b1[i][j] * g[k]( t_n + c1[j]*h, Hk[j] ) )
            //    and store the diffusion for the noise source k in the dynPtrs
            scaledSum(coefficients->B1.at(i), g_Hk.at(k), i).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));

            for (size_t l = 0; l < stateIdToNoiseIndexMaps.size(); l++)
            {
                if (l==k) continue;

                // sum_{j=1..i-1}( b2[i][j] * g[l]( t_n + c1[i]*h, Hl[i] ) )
                //    and store the diffusion for the noise source k in the dynPtrs
                scaledSum(coefficients->B2.at(i), g_Hk.at(l), i).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(l));
            }

            // The kth element uses pseudo timestep xi, the rest Ikl[k,l]
            Eigen::VectorXd pseudoTimeStep(stateIdToNoiseIndexMaps.size());
            for (size_t l = 0; l < stateIdToNoiseIndexMaps.size(); l++)
            {
                if (l==k) pseudoTimeStep(l) = rv.xi;
                else      pseudoTimeStep(l) = rv.Ikl(k,l);
            }

            // This sets the current state to H_i^(k)
            propagateState(timeStep, pseudoTimeStep, stateIdToNoiseIndexMaps);

            // This computes g^(k)(t_n + c1[i]*h, Hk[i]) for the next steps
            g_Hk.at(k).at(i) = computeDiffusions(currentTime + coefficients->c1.at(i)*timeStep, timeStep, stateIdToNoiseIndexMaps.at(k));
        }
    }



    /*  y_n+1 = y_n
    *          + h * sum_{i=1..s}( alpha[i] * f( t_n + c0[i]*h, H0[i] ) )
    *          + sum_{k=1..m}( Ihat[k]       * sum_{i=1..s}( beta0[i] * g[k]( t_n + c1[i]*h, Hk[i] ) ) )
    *          + sum_{k=1..m}( Ihat_kl[k][k] * sum_{i=1..s}( beta1[i] * g[k]( t_n + c1[i]*h, Hk[i] ) ) )
    */

    // y_n
    //    and store the state in the dynPtrs
    currentState.setStates(dynPtrs);

    // sum_{i=1..s}( alpha[i] * f( t_n + c0[i]*h, H0[i] ) )
    //    and store the derivative in the dynPtrs
    scaledSum(coefficients->alpha, f_H0, numberStages).setDerivatives(dynPtrs);

    // sum_{k=1..m}( Ihat[k] * sum_{i=1..s}( beta0[i] * g[k]( t_n + c1[i]*h, Hk[i] ) ) )
    for (size_t k = 0; k < stateIdToNoiseIndexMaps.size(); k++)
    {
        // sum_{i=1..s}( beta0[i] * g[k]( t_n + c1[i]*h, Hk[i] ) )
        //     and store the diffusion for the noise source k in the dynPtrs
        scaledSum(coefficients->beta0, g_Hk.at(k), numberStages).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }

    /*
    * This computes:
    *
    *          y_n
    *          + h * sum_{i=1..s}( alpha[i] * f( t_n + c0[i]*h, H0[i] ) )
    *          + sum_{k=1..m}( Ihat[k]       * sum_{i=1..s}( beta0[i] * g[k]( t_n + c1[i]*h, Hk[i] ) ) )
    *
    * BUT! We're missing the 'last line' of the full y_n+1
    * because we need to set the diffusions to different values
    */
    propagateState(timeStep, rv.Ik, stateIdToNoiseIndexMaps);

    // sum_{k=1..m}( Ihat_kl[k] * sum_{i=1..s}( beta1[i] * g[k]( t_n + c1[i]*h, Hk[i] ) ) )
    for (size_t k = 0; k < stateIdToNoiseIndexMaps.size(); k++)
    {
        // sum_{i=1..s}( beta1[i] * g[k]( t_n + c1[i]*h, Hk[i] ) )
        //     and store the diffusion for the noise source k in the dynPtrs
        scaledSum(coefficients->beta1, g_Hk.at(k), numberStages).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }

    /*
    * This computes:
    *
    *  y_n+1 = y_n
    *          + h * sum_{i=1..s}( alpha[i] * f( t_n + c0[i]*h, H0[i] ) )
    *          + sum_{k=1..m}( Ihat[k]       * sum_{i=1..s}( beta0[i] * g[k]( t_n + c1[i]*h, Hk[i] ) ) )
    *          + sum_{k=1..m}( Ihat_kl[k][k] * sum_{i=1..s}( beta1[i] * g[k]( t_n + c1[i]*h, Hk[i] ) ) )
    *
    * By propagating the previously computed state (see previous call
    * to propagateState) with only the last row. To do so, we replaced the
    * diffusions with those of the last row and set the timeStep to zero
    * so that the contribution of the derivative would not be double counted.
    */
    propagateState(0, rv.Ikk, stateIdToNoiseIndexMaps);

    // now the dynPtrs have y_n+1 in them!!
}

template <size_t numberStages>
ExtendedStateVector
svIntegratorWeakStochasticRungeKutta<numberStages>::computeDerivatives(double time, double timeStep)
{
    for (auto dynPtr : this->dynPtrs) {
        dynPtr->equationsOfMotion(time, timeStep);
    }

    return ExtendedStateVector::fromStateDerivs(this->dynPtrs);
}

template <size_t numberStages>
std::vector<ExtendedStateVector>
svIntegratorWeakStochasticRungeKutta<numberStages>::computeDiffusions(double time,
                                                        double timeStep,
                                                        const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps
                                                    )
{
    for (auto dynPtr : this->dynPtrs) {
        dynPtr->equationsOfMotionDiffusion(time, timeStep);
    }

    return ExtendedStateVector::fromStateDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps);
}

template <size_t numberStages>
ExtendedStateVector
svIntegratorWeakStochasticRungeKutta<numberStages>::computeDiffusions(double time,
                                                        double timeStep,
                                                        const StateIdToIndexMap& stateIdToNoiseIndexMap
                                                    )
{
    for (auto dynPtr : this->dynPtrs) {
        dynPtr->equationsOfMotionDiffusion(time, timeStep);
    }

    return ExtendedStateVector::fromStateDiffusions(this->dynPtrs, stateIdToNoiseIndexMap);
}

template<size_t numberStages>
inline ExtendedStateVector
svIntegratorWeakStochasticRungeKutta<numberStages>::scaledSum(const std::array<double, numberStages>& factors,
                                                              const std::array<ExtendedStateVector, numberStages>& vectors,
                                                              size_t length)
{
    ExtendedStateVector result = vectors.at(0) * factors.at(0);
    for (size_t i = 1; i < length; i++) {
        if (factors.at(i) == 0) continue;
        result += vectors.at(i) * factors.at(i);
    }
    return result;
}

#endif /* svIntegratorWeakStochasticRungeKutta_h */
