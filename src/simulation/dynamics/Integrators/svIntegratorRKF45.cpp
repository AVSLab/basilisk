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

    aMatrix[1] = 1.0 / 4.0;
    aMatrix[2] = 3.0 / 8.0;
    aMatrix[3] = 12.0 / 13.0;
    aMatrix[4] = 1.0;
    aMatrix[5] = 1.0 / 2.0;

    bMatrix[1][0] = 1.0 / 4.0;
    bMatrix[2][0] = 3.0 / 32.0;
    bMatrix[2][1] = 9.0 / 32.0;
    bMatrix[3][0] = 1932.0 / 2197.0;
    bMatrix[3][1] = -7200.0 / 2197.0;
    bMatrix[3][2] = 7296.0 / 2197.0;
    bMatrix[4][0] = 439.0 / 216.0;
    bMatrix[4][1] = -8.0;
    bMatrix[4][2] = 3680.0 / 513.0;
    bMatrix[4][3] = -845.0 / 4104.0;
    bMatrix[5][0] = -8.0 / 27.0;
    bMatrix[5][1] = 2.0;
    bMatrix[5][2] = -3544.0 / 2565.0;
    bMatrix[5][3] = 1859.0 / 4104.0;
    bMatrix[5][4] = -11.0 / 40.0;

    cMatrix[0] = 25.0 / 216.0;
    cMatrix[2] = 1408.0 / 2565.0;
    cMatrix[3] = 2197.0 / 4104.0;
    cMatrix[4] = -1.0 / 5.0;

    dMatrix[0] = 1.0 / 360.0;
    dMatrix[2] = -128.0 / 4275.0;
    dMatrix[3] = -2197.0 / 75240.0;
    dMatrix[4] = -1.0 / 50.0;
    dMatrix[5] = 2.0 / 55.0;

    
    return;
}

svIntegratorRKF45::~svIntegratorRKF45()
{
    return;
}

/*<!
 Implements a 4th order Runge Kutta Fehlberg variable time step integration method
 see [Wiki Page on Runge-Kutta-Fehlberg's Method](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta%E2%80%93Fehlberg_method)
 */
void svIntegratorRKF45::integrate(double currentTime, double timeStep)
{
	StateVector stateOut;
	StateVector stateInit;
    std::vector<StateVector> kMatrix;
	std::map<std::string, StateData>::iterator it;
	std::map<std::string, StateData>::iterator itOut;
	std::map<std::string, StateData>::iterator itInit;
	stateOut = dynPtr->dynManager.getStateVector();
	stateInit = dynPtr->dynManager.getStateVector();
    kMatrix.clear(); //Clearing out the matrix and beginning to populate

    // Compute the equations of motion for t0
    dynPtr->equationsOfMotion(currentTime);

    // Loop through all 6 coefficients (k1 through k6)
    for (uint64_t i = 0; i < 6; i++)
    {
        // Initialize the state iterators
        for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itInit = stateInit.stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itInit++)
        {
            it->second.state = itInit->second.state;
        }

        // Loop through the B matrix coefficients that defien the point of integration
        for (uint64_t j = 0; j < i; j++)
        {
            for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itOut = kMatrix[j].stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itOut++)
            {
                it->second.state = it->second.state + timeStep * bMatrix[i][j] * itOut->second.stateDeriv;

            }

        }

        // Integrate with the appropriate time step using the A matrix
        dynPtr->equationsOfMotion(currentTime + timeStep * aMatrix[i]);

        // Save the current coefficient
        kMatrix.push_back(dynPtr->dynManager.getStateVector());

        // Update the intermediate result
        for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itOut = stateOut.stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itOut++)
        {
            itOut->second.state = itOut->second.state + timeStep * cMatrix[i] * it->second.stateDeriv;
        }
    }

    // Update the entire state vector after integration
    dynPtr->dynManager.updateStateVector(stateOut);

    return;
}


