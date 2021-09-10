/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#include "svIntegratorRKF45.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include <stdio.h>

svIntegratorRKF45::svIntegratorRKF45(DynamicObject* dyn) : StateVecIntegrator(dyn)
{
    memset(aMatrix, 0x0, sizeof(aMatrix));
    memset(bMatrix, 0x0, sizeof(bMatrix));
    memset(cMatrix, 0x0, sizeof(cMatrix));
    memset(dMatrix, 0x0, sizeof(dMatrix));

    aMatrix[2] = 1.0 / 4.0;
    aMatrix[3] = 3.0 / 8.0;
    aMatrix[4] = 12.0 / 13.0;
    aMatrix[5] = 1.0;
    aMatrix[6] = 1.0 / 2.0;

    bMatrix[2][1] = 1.0 / 4.0;
    bMatrix[3][1] = 3.0 / 32.0;
    bMatrix[3][2] = 9.0 / 32.0;
    bMatrix[4][1] = 1932.0 / 2197.0;
    bMatrix[4][2] = -7200.0 / 2197.0;
    bMatrix[4][3] = 7296.0 / 2197.0;
    bMatrix[5][1] = 439.0 / 216.0;
    bMatrix[5][2] = -8.0;
    bMatrix[5][3] = 3680.0 / 513.0;
    bMatrix[5][4] = -845.0 / 4104.0;
    bMatrix[6][1] = -8.0 / 27.0;
    bMatrix[6][2] = 2.0;
    bMatrix[6][3] = -3544.0 / 2565.0;
    bMatrix[6][4] = 1859.0 / 4104.0;
    bMatrix[6][5] = -11.0 / 40.0;

    cMatrix[1] = 25.0 / 216.0;
    cMatrix[3] = 1408.0 / 2565.0;
    cMatrix[4] = 2197.0 / 4104.0;
    cMatrix[5] = -1.0 / 5.0;

    dMatrix[1] = 1.0 / 360.0;
    dMatrix[3] = -128.0 / 4275.0;
    dMatrix[4] = -2197.0 / 75240.0;
    dMatrix[5] = -1.0 / 50.0;
    dMatrix[6] = 2.0 / 55.0;

    
    return;
}

svIntegratorRKF45::~svIntegratorRKF45()
{
    return;
}

/*<!
 Implements a 4th order Runge Kutta Fehlberg variable time step integration method
 see [Wiki Page on Runge-Kutta-Fehlberg's Method](https://en.wikipedia.org/wiki/Runge–Kutta–Fehlberg_method)
 */
void svIntegratorRKF45::integrate(double currentTime, double timeStep)
{
	StateVector stateOut;
	StateVector stateInit;
	std::map<std::string, StateData>::iterator it;
	std::map<std::string, StateData>::iterator itOut;
	std::map<std::string, StateData>::iterator itInit;
	stateOut = dynPtr->dynManager.getStateVector();
	stateInit = dynPtr->dynManager.getStateVector();
    dynPtr->equationsOfMotion(currentTime);
    for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itOut = stateOut.stateMap.begin(), itInit = stateInit.stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itOut++, itInit++)
    {
        itOut->second.setDerivative(it->second.getStateDeriv());
        itOut->second.propagateState(timeStep / 2.0);
        it->second.state = itInit->second.state + timeStep*it->second.stateDeriv;
    }

    dynPtr->equationsOfMotion(currentTime + timeStep);
    for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itOut = stateOut.stateMap.begin(), itInit = stateInit.stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itOut++, itInit++)
    {
        itOut->second.setDerivative(it->second.getStateDeriv());
        itOut->second.propagateState(timeStep / 2.0);
    }

	dynPtr->dynManager.updateStateVector(stateOut);	

    return;
}


