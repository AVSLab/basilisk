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
#include "gauss_markov.h"
#include "linearAlgebra.h"

/*! The constructor initialies the random number generator used for the walks*/
GaussMarkov::GaussMarkov()
{
    this->RNGSeed = 0x1badcad1;
    this->numStates = 0;
    initializeRNG();
}

GaussMarkov::GaussMarkov(uint64_t size, uint64_t newSeed)
{
    this->RNGSeed = newSeed;
    this->numStates = size;
    initializeRNG();
    this->propMatrix.resize(size,size);
    this->propMatrix.fill(0.0);
    this->currentState.resize((int64_t) size);
    this->currentState.fill(0.0);
    this->noiseMatrix.resize((int64_t) size, (int64_t) size);
    this->noiseMatrix.fill(0.0);
    this->stateBounds.resize((int64_t) size);
    this->stateBounds.fill(DEFAULT_BOUND);
}

void GaussMarkov::initializeRNG() {
    //! - Set up standard normal distribution N(0,1) parameters for random number generation
    std::normal_distribution<double>::param_type updatePair(0.0, 1.0);
    this->rGen.seed((unsigned int)this->RNGSeed);
    this->rNum.param(updatePair);
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
    Eigen::VectorXd errorVector;
    Eigen::VectorXd ranNums;

    //! - Check for consistent sizes
    if((this->propMatrix.size() != this->noiseMatrix.size()) ||
       ((uint64_t) this->propMatrix.size() != this->numStates*this->numStates))
    {
        bskLogger.bskLog(BSK_ERROR, "Matrix size mismatch in Gauss Markov model");
        return;
    }
    if((uint64_t) this->stateBounds.size() != this->numStates)
    {
        bskLogger.bskLog(BSK_ERROR, "State bounds size mismatch in Gauss Markov model");
        return;
    }

    //! - Generate base random numbers
    ranNums.resize((int64_t) this->numStates);
    for(size_t i = 0; i < this->numStates; i++) {
        ranNums[i] = this->rNum(rGen);
    }

    //! - Apply noise first
    errorVector = this->noiseMatrix * ranNums;

    //! - Then propagate previous state
    this->currentState = this->propMatrix * this->currentState;

    //! - Add noise to propagated state
    this->currentState += errorVector;

    //! - Apply bounds if needed
    for(size_t i = 0; i < this->numStates; i++) {
        if(this->stateBounds[i] > 0.0) {
            if(fabs(this->currentState[i]) > this->stateBounds[i]) {
                this->currentState[i] = copysign(this->stateBounds[i], this->currentState[i]);
            }
        }
    }
}
