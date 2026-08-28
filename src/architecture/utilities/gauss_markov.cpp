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

#include <math.h>
#include <limits>
#include "gauss_markov.h"
#include "linearAlgebra.h"

/*! The constructor initializes the random number generator used for the walks. */
GaussMarkov::GaussMarkov()
{
    this->RNGSeed = 0x1badcad1;
    this->numStates = 0;
    initializeRNG();
}

GaussMarkov::GaussMarkov(uint64_t size, uint64_t newSeed)
{
    if (size > static_cast<uint64_t>(std::numeric_limits<Eigen::Index>::max())) {
        this->bskLogger.bskError("Gauss Markov state count exceeds the Eigen index range");
    }
    const Eigen::Index stateCount = static_cast<Eigen::Index>(size);
    this->RNGSeed = newSeed;
    this->numStates = size;
    initializeRNG();
    this->propMatrix.resize(stateCount, stateCount);
    this->propMatrix.fill(0.0);
    this->currentState.resize(stateCount);
    this->currentState.fill(0.0);
    this->noiseMatrix.resize(stateCount, stateCount);
    this->noiseMatrix.fill(0.0);
    this->stateBounds.resize(stateCount);
    this->stateBounds.fill(DEFAULT_BOUND);
    this->randomValues.resize(stateCount);
    this->stateNoise.resize(stateCount);
    this->propagatedState.resize(stateCount);
}

void GaussMarkov::initializeRNG() {
    //! - Set up standard normal distribution N(0,1) parameters for random number generation
    std::normal_distribution<double>::param_type updatePair(0.0, 1.0);
    this->rGen.seed(static_cast<std::minstd_rand::result_type>(this->RNGSeed));
    this->rNum.reset();
    this->rNum.param(updatePair);
}

uint64_t
GaussMarkov::deriveSecondarySeed(uint64_t baseSeed)
{
    constexpr uint64_t secondaryStreamDiscriminator = 0x9E3779B97F4A7C15ULL;
    const uint64_t candidateSeed = baseSeed ^ secondaryStreamDiscriminator;
    std::minstd_rand primaryGenerator(static_cast<std::minstd_rand::result_type>(baseSeed));
    std::minstd_rand secondaryGenerator(static_cast<std::minstd_rand::result_type>(candidateSeed));
    if (primaryGenerator == secondaryGenerator) {
        return static_cast<uint64_t>(primaryGenerator());
    }
    return candidateSeed;
}

/*! The destructor is a placeholder for one that might do something*/
GaussMarkov::~GaussMarkov()
{
}

/*! This method performs almost all of the work for the Gauss Markov random
    walk.  It uses the current random walk configuration, propagates the current
    state, and then applies appropriate errors to the states to set the current
    error level.
*/
void GaussMarkov::computeNextState()
{
    //! - Check for consistent sizes
    if((this->propMatrix.size() != this->noiseMatrix.size()) ||
       ((uint64_t) this->propMatrix.size() != this->numStates*this->numStates))
    {
        bskLogger.bskError("Matrix size mismatch in Gauss Markov model");
    }
    if((uint64_t) this->stateBounds.size() != this->numStates)
    {
        bskLogger.bskError("State bounds size mismatch in Gauss Markov model");
    }

    //! - Generate base random numbers
    for(Eigen::Index i = 0; i < this->randomValues.size(); i++) {
        this->randomValues[i] = this->rNum(rGen);
    }

    //! - Apply noise first
    this->stateNoise.noalias() = this->noiseMatrix * this->randomValues;

    //! - Then propagate previous state
    this->propagatedState.noalias() = this->propMatrix * this->currentState;

    //! - Add noise to propagated state
    this->currentState = this->propagatedState + this->stateNoise;

    //! - Apply bounds if needed
    for(Eigen::Index i = 0; i < this->stateBounds.size(); i++) {
        if(this->stateBounds[i] > 0.0) {
            if(fabs(this->currentState[i]) > this->stateBounds[i]) {
                this->currentState[i] = copysign(this->stateBounds[i], this->currentState[i]);
            }
        }
    }
}
