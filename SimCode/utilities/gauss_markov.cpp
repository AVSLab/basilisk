
#include "utilities/gauss_markov.h"
#include "utilities/linearAlgebra.h"
#include <iostream>
#include <math.h>

GaussMarkov::GaussMarkov()
{
    std::normal_distribution<double>::param_type
    UpdatePair(0.0, 1.0/3.0);
    rGen.seed(RNGSeed);
    rNum.param(UpdatePair);
}

GaussMarkov::~GaussMarkov()
{
}

void GaussMarkov::computeNextState()
{
    std::vector<double> errorVector;
    std::vector<double> ranNums;
    std::vector<double>::iterator it;
    std::vector<double>::iterator boundIt;
    std::vector<double>::iterator stateIt;
    if((propMatrix.size() != noiseMatrix.size()) ||
       propMatrix.size() != stateBounds.size()*stateBounds.size())
    {
        std::cerr << "For the Gauss Markov model, you HAVE, and I mean HAVE, ";
        std::cerr << "to have your propagate and noise matrices be same size";
        std::cerr << std::endl << "I quit.";
        return;
    }
    uint64_t numStates = stateBounds.size();
    if(currentState.size() < stateBounds.size())
    {
        currentState.insert(currentState.begin(),
                            stateBounds.size() - currentState.size(), 0.0);
    }
    errorVector = currentState;
    mMultM(propMatrix.data(), numStates, numStates, errorVector.data(),
           numStates, 1, currentState.data());
    ranNums.insert(ranNums.begin(), stateBounds.size(), 0.0);
    for(it=ranNums.begin(), boundIt = stateBounds.begin(),
        stateIt = currentState.begin(); it!=ranNums.end();
        it++, stateIt++, boundIt++)
    {
        *it = rNum(rGen);
        if(*boundIt > 0.0)
        {
            double stateCalc = fabs(*stateIt) > *boundIt*1E-10 ?
            fabs(*stateIt) : *boundIt*1E-10;
            double boundCheck = ((*boundIt)*2.0 - stateCalc)/stateCalc;
            boundCheck = boundCheck > *boundIt*1E-10 ?
            boundCheck :*boundIt*1E-10;
            boundCheck = 1.0/exp(boundCheck*boundCheck*boundCheck);
            boundCheck *= copysign(boundCheck, -(*stateIt));
            *it += boundCheck;
        }
    }
    mMultM(noiseMatrix.data(), numStates, numStates, ranNums.data(), numStates,
           1, errorVector.data());
    vAdd(currentState.data(), numStates, errorVector.data(),
         currentState.data());
    
}

