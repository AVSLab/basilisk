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

#ifndef svStochIntegratorMayurama_h
#define svStochIntegratorMayurama_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stateVecStochasticIntegrator.h"
#include "../_GeneralModuleFiles/extendedStateVector.h"

#include <random>


/** Random Number Generator for the Euler-Mayurama integrator */
class EulerMayuramaRandomVariableGenerator
{
public:
    /** Sets the seed for the RNG */
    inline void setSeed(size_t seed) {rng.seed(seed);}

    /** Generates m normally distributed RV samples, with mean
     * zero and standard deviation equal to sqrt(h)
     */
    Eigen::VectorXd generate(size_t m, double h);

protected:
    /** Random Number Generator */
    std::mt19937 rng{std::random_device{}()};

    /** Standard normally distributed random variable */
    std::normal_distribution<double> normal_rv{0., 1.};
};

/** The 1-weak 1-strong order Euler-Mayurama stochastic integrator.
 *
 * For an SDE of the form:
 *
 *   dx = f(t,x)*dt + sum_i g_i(t,x)*dW
 *
 * The integrator, with timestep h, computes:
 *
 *   x_{n+1} = x_n + f(t,x)*h + sum_i g_i(t,x)*sqrt(h)*N_i
 *
 * where N_i are independent and identically distributed standard normal
 * random variables.
 */
class svStochIntegratorMayurama : public StateVecStochasticIntegrator {
public:

    /** Uses same constructor as StateVecStochasticIntegrator */
    using StateVecStochasticIntegrator::StateVecStochasticIntegrator;

    /** Sets the seed for the Random Number Generator used by this integrator.
     *
     * As a stochastic integrator, random numbers are drawn during each time step.
     * By default, a randomly generated seed is used each time.
     *
     * If the seed is set, the integrator will always draw the same numbers
     * during time-stepping.
     */
    inline void setRNGSeed(size_t seed) {rvGenerator.setSeed(seed);};

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    virtual void integrate(double currentTime, double timeStep) override;

public:
    /** Random Number Generator for the integrator */
    EulerMayuramaRandomVariableGenerator rvGenerator;
};

#endif // svStochIntegratorMayurama_h
