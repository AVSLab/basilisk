/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "gauss_markov.h"
#include "linearAlgebra.h"
#include <iostream>
#include <math.h>

/*! The constructor initialies the random number generator used for the walks*/
GaussMarkov::GaussMarkov()
{
    std::normal_distribution<double>::param_type
    UpdatePair(0.0, 1.0/3.0);
    rGen.seed((unsigned int)RNGSeed);
    rNum.param(UpdatePair);
}

/*! The destructor is a placeholder for one that might do something*/
GaussMarkov::~GaussMarkov()
{
}

/*! This method performs almost all of the work for the Gauss Markov random 
    walk.  It uses the current random walk configuration, propagates the current 
    state, and then applies appropriate errors to the states to set the current 
    error level.
    @return void
*/
void GaussMarkov::computeNextState()
{
    std::vector<double> errorVector;
    std::vector<double> ranNums;
    std::vector<double>::iterator it;
    std::vector<double>::iterator boundIt;
    std::vector<double>::iterator stateIt;
    
    //! Begin method steps
    //! - Check for consistent sizes on all of the user-settable matrices.  Quit if they don't match.
    if((propMatrix.size() != noiseMatrix.size()) ||
       propMatrix.size() != stateBounds.size()*stateBounds.size())
    {
        std::cerr << "For the Gauss Markov model, you HAVE, and I mean HAVE, ";
        std::cerr << "to have your propagate and noise matrices be same size";
        std::cerr << std::endl << "I quit.";
        return;
    }
    //! - Get the number of states to walk on and pad the currentState if necessary.
    uint64_t numStates = stateBounds.size();
    if(currentState.size() < stateBounds.size())
    {
        currentState.insert(currentState.begin(),
                            stateBounds.size() - currentState.size(), 0.0);
    }
    //! - Propagate the state forward in time using the propMatrix and the currentState
    errorVector = currentState;
    mMultM(propMatrix.data(), numStates, numStates, errorVector.data(),
           numStates, 1, currentState.data());
    
    //! - Compute the random numbers used for each state.  Note that the same generator is used for all
    ranNums.insert(ranNums.begin(), stateBounds.size(), 0.0);
    for(it=ranNums.begin(), boundIt = stateBounds.begin(),
        stateIt = currentState.begin(); it!=ranNums.end();
        it++, stateIt++, boundIt++)
    {
        *it = rNum(rGen);
        //! - For cases where we want to bound the walk, find how close it is to the bounds
        if(*boundIt > 0.0)
        {
            double stateCalc = fabs(*stateIt) > *boundIt*1E-10 ?
            fabs(*stateIt) : *boundIt*1E-10;
            /*! - Ideally we should find the statistical likelihood of violating a bound and use that.
                  However, that would require an error function, for now (PDR), just use an exponential 
                  to pull states down that are getting "close" to the bound.*/
            double boundCheck = ((*boundIt)*2.0 - stateCalc)/stateCalc;
            boundCheck = boundCheck > *boundIt*1E-10 ?
            boundCheck :*boundIt*1E-10;
            boundCheck = 1.0/exp(boundCheck*boundCheck*boundCheck);
            boundCheck *= copysign(boundCheck, -(*stateIt));
            /*! - As we get close to the bound apply values whose absolute value is < 1 with the
                appropriate sign to the computed random number ensuring that the walk starts heading 
                away from the bound*/
            *it += boundCheck;
        }
    }
    
    //! - Apply the noise matrix to the random numbers to get error values
    mMultM(noiseMatrix.data(), numStates, numStates, ranNums.data(), numStates,
           1, errorVector.data());
    //! - Add the new errors to the currentState to get a good currentState
    vAdd(currentState.data(), numStates, errorVector.data(),
         currentState.data());
}

