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


#include "svIntegratorRK4.h"
#include "dynamicObject.h"
#include <stdio.h>

svIntegratorRK4::svIntegratorRK4(DynamicObject* dyn) : StateVecIntegrator(dyn)
{
    
    return;
}

svIntegratorRK4::~svIntegratorRK4()
{
    return;
}

void svIntegratorRK4::integrate(double currentTime, double timeStep)
{
    std::vector<StateVector> stateOut;
    std::vector<StateVector> stateInit;
    std::map<std::string, StateData>::iterator it;
    std::map<std::string, StateData>::iterator itOut;
    std::map<std::string, StateData>::iterator itInit;
    int i;              // dynamic object loop counter

    for (i=0; i<dynPtrs.size(); i++) {
        stateOut.push_back(dynPtrs[i]->dynManager.getStateVector());
        stateInit.push_back(dynPtrs[i]->dynManager.getStateVector());
        dynPtrs[i]->equationsOfMotion(currentTime, timeStep);
        for (it = dynPtrs[i]->dynManager.stateContainer.stateMap.begin(), itOut = stateOut[i].stateMap.begin(), itInit = stateInit[i].stateMap.begin(); it != dynPtrs[i]->dynManager.stateContainer.stateMap.end(); it++, itOut++, itInit++)
        {
            itOut->second.setDerivative(it->second.getStateDeriv());
            itOut->second.propagateState(timeStep / 6.0);
            it->second.state = itInit->second.state + 0.5*timeStep*it->second.stateDeriv;
        }
    }

    for (i=0; i<dynPtrs.size(); i++) {
        dynPtrs[i]->equationsOfMotion(currentTime + timeStep * 0.5, timeStep);
        for (it = dynPtrs[i]->dynManager.stateContainer.stateMap.begin(), itOut = stateOut[i].stateMap.begin(), itInit = stateInit[i].stateMap.begin(); it != dynPtrs[i]->dynManager.stateContainer.stateMap.end(); it++, itOut++, itInit++)
        {
            itOut->second.setDerivative(it->second.getStateDeriv());
            itOut->second.propagateState(2.0*timeStep / 6.0);
            it->second.state = itInit->second.state + 0.5*timeStep*it->second.stateDeriv;
        }
    }

    for (i=0; i<dynPtrs.size(); i++) {
        dynPtrs[i]->equationsOfMotion(currentTime + timeStep * 0.5, timeStep);
        for (it = dynPtrs[i]->dynManager.stateContainer.stateMap.begin(), itOut = stateOut[i].stateMap.begin(), itInit = stateInit[i].stateMap.begin(); it != dynPtrs[i]->dynManager.stateContainer.stateMap.end(); it++, itOut++, itInit++)
        {
            itOut->second.setDerivative(it->second.getStateDeriv());
            itOut->second.propagateState(2.0*timeStep / 6.0);
            it->second.state = itInit->second.state + timeStep*it->second.stateDeriv;

        }
    }

    for (i=0; i<dynPtrs.size(); i++) {
        dynPtrs[i]->equationsOfMotion(currentTime + timeStep, timeStep);
        for (it = dynPtrs[i]->dynManager.stateContainer.stateMap.begin(), itOut = stateOut[i].stateMap.begin(), itInit = stateInit[i].stateMap.begin(); it != dynPtrs[i]->dynManager.stateContainer.stateMap.end(); it++, itOut++, itInit++)
        {
            itOut->second.setDerivative(it->second.getStateDeriv());
            itOut->second.propagateState(timeStep / 6.0);

        }
    }

    for (i=0; i<dynPtrs.size(); i++) {
        dynPtrs[i]->dynManager.updateStateVector(stateOut[i]);
    }

    return;
}
