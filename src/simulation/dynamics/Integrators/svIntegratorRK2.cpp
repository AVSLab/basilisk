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


#include "svIntegratorRK2.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include <stdio.h>

svIntegratorRK2::svIntegratorRK2(DynamicObject* dyn) : StateVecIntegrator(dyn)
{
}

svIntegratorRK2::~svIntegratorRK2()
{
}

/*<!
 Implements a 2nd order Runge Kutta integration method called Heun's method
 see [Wiki Page on Heun's Method](https://en.wikipedia.org/wiki/Heun%27s_method#Runge.E2.80.93Kutta_method)
 */
void svIntegratorRK2::integrate(double currentTime, double timeStep)
{
    std::vector<StateVector> stateOut;
    std::vector<StateVector> stateInit;
    std::map<std::string, StateData>::iterator it;
    std::map<std::string, StateData>::iterator itOut;
    std::map<std::string, StateData>::iterator itInit;

    for (const auto& dynPtr : dynPtrs) {
        stateOut.push_back(dynPtr->dynManager.getStateVector());
        stateInit.push_back(dynPtr->dynManager.getStateVector());
        dynPtr->equationsOfMotion(currentTime, timeStep);
    }
    for (size_t i=0; i<dynPtrs.size(); i++) {
        for (it = dynPtrs.at(i)->dynManager.stateContainer.stateMap.begin(),
             itOut = stateOut.at(i).stateMap.begin(),
             itInit = stateInit.at(i).stateMap.begin();
             it != dynPtrs.at(i)->dynManager.stateContainer.stateMap.end();
             it++,
             itOut++,
             itInit++)
        {
            itOut->second.setDerivative(it->second.getStateDeriv());
            itOut->second.propagateState(timeStep / 2.0);
            it->second.state = itInit->second.state + timeStep*it->second.stateDeriv;
        }
    }
    
    for (const auto& dynPtr : dynPtrs) {
        dynPtr->equationsOfMotion(currentTime + timeStep, timeStep);
    }
    for (size_t i=0; i<dynPtrs.size(); i++) {
        for (it = dynPtrs.at(i)->dynManager.stateContainer.stateMap.begin(),
             itOut = stateOut.at(i).stateMap.begin(),
             itInit = stateInit.at(i).stateMap.begin();
             it != dynPtrs.at(i)->dynManager.stateContainer.stateMap.end();
             it++,
             itOut++,
             itInit++)
        {
            itOut->second.setDerivative(it->second.getStateDeriv());
            itOut->second.propagateState(timeStep / 2.0);
        }
    }

    for (size_t i=0; i<dynPtrs.size(); i++) {
        dynPtrs.at(i)->dynManager.updateStateVector(stateOut.at(i));
    }
}
