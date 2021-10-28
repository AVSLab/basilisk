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
#include <iostream>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

svIntegratorRKF45::svIntegratorRKF45(DynamicObject* dyn) : StateVecIntegrator(dyn)
{
    // Initialize the matrices to 0
    memset(alphaMatrix, 0x0, sizeof(alphaMatrix));
    memset(betaMatrix, 0x0, sizeof(betaMatrix));
    memset(chMatrix, 0x0, sizeof(chMatrix));
    memset(ctMatrix, 0x0, sizeof(ctMatrix));

    // Populate the coefficient matrices 
    alphaMatrix[1] = 1.0 / 4.0;
    alphaMatrix[2] = 3.0 / 8.0;
    alphaMatrix[3] = 12.0 / 13.0;
    alphaMatrix[4] = 1.0;
    alphaMatrix[5] = 1.0 / 2.0;

    betaMatrix[1][0] = 1.0 / 4.0;
    betaMatrix[2][0] = 3.0 / 32.0;
    betaMatrix[2][1] = 9.0 / 32.0;
    betaMatrix[3][0] = 1932.0 / 2197.0;
    betaMatrix[3][1] = -7200.0 / 2197.0;
    betaMatrix[3][2] = 7296.0 / 2197.0;
    betaMatrix[4][0] = 439.0 / 216.0;
    betaMatrix[4][1] = -8.0;
    betaMatrix[4][2] = 3680.0 / 513.0;
    betaMatrix[4][3] = -845.0 / 4104.0;
    betaMatrix[5][0] = -8.0 / 27.0;
    betaMatrix[5][1] = 2.0;
    betaMatrix[5][2] = -3544.0 / 2565.0;
    betaMatrix[5][3] = 1859.0 / 4104.0;
    betaMatrix[5][4] = -11.0 / 40.0;

    chMatrix[0] = 25.0 / 216.0;
    chMatrix[2] = 1408.0 / 2565.0;
    chMatrix[3] = 2197.0 / 4104.0;
    chMatrix[4] = -1.0 / 5.0;

    ctMatrix[0] = 1.0 / 360.0;
    ctMatrix[2] = -128.0 / 4275.0;
    ctMatrix[3] = -2197.0 / 75240.0;
    ctMatrix[4] = -1.0 / 50.0;
    ctMatrix[5] = 2.0 / 55.0;

    // Set the default values for absolute and relative tolerance
    this->absTol = 1e-8;
    this->relTol = 1e-4;
    
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
	StateVector stateOut;  // output state vector
	StateVector stateInit;  // initial state vector
    StateVector errorMatrix;  // error state vector
    std::vector<StateVector> kMatrix;  // matrix of k coefficients
	std::map<std::string, StateData>::iterator it;
	std::map<std::string, StateData>::iterator itOut;
	std::map<std::string, StateData>::iterator itInit;
    std::map<std::string, StateData>::iterator itkMatrix;
    std::map<std::string, StateData>::iterator itError;
	stateOut = dynPtr->dynManager.getStateVector();  // copy current state variables
	stateInit = dynPtr->dynManager.getStateVector();  // copy current state variables
    errorMatrix = dynPtr->dynManager.getStateVector();  // copy current state variables
    double h = timeStep;  // updated variable time step that depends on the relative error and the relative tolerance
    double t = currentTime;  // integration time
    double hInt;  // time step used for the current integration loop
    double relError;  // relative error for the current state variable
    double maxRelError;  // largest relative error of all the state variables
    double scaleFactor = 0.9;  // scale factor used for robustness. If the error and the tolerance are very close, this scale factor decreases the time step to improve performance

    while (abs(t - currentTime - timeStep) > 1e-12) {

        // Enter the loop
        maxRelError = 10 * relTol;

        // Time step refinement loop
        while (maxRelError > relTol) {

            // Reset the maximum relative error
            maxRelError = 0;

            // Reset the time step for integration
            hInt = h;

            // Reset the k matrix
            kMatrix.clear();

            // Compute the equations of motion for t
            dynPtr->equationsOfMotion(t, hInt);

            // Reset the ouput and error vectors
            for (itOut = stateOut.stateMap.begin(), itInit = stateInit.stateMap.begin(), itError = errorMatrix.stateMap.begin(); itOut != stateOut.stateMap.end(); itOut++, itInit++, itError++)
            {
                itOut->second.state = itInit->second.state;
                itError->second.state.setZero();
            }

            // Loop through all 6 coefficients (k1 through k6)
            for (uint64_t i = 0; i < 6; i++)
            {
                // Initialize the state iterators. The state variables are defined in a dictionary and are pulled and populated one by one
                for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itInit = stateInit.stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itInit++)
                {
                    it->second.state = itInit->second.state;
                }

                // Loop through the B matrix coefficients that define the point of integration
                for (uint64_t j = 0; j < i; j++)
                {
                    for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itOut = kMatrix[j].stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itOut++)
                    {
                        it->second.state = it->second.state + hInt * betaMatrix[i][j] * itOut->second.stateDeriv;
                    }
                }

                // Integrate with the appropriate time step using the A matrix coefficients
                dynPtr->equationsOfMotion(t + hInt * alphaMatrix[i], hInt);

                // Save the current k coefficient
                kMatrix.push_back(dynPtr->dynManager.getStateVector());

                // Update the state at the end of the current integration step
                for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itOut = stateOut.stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itOut++)
                {
                    itOut->second.state += hInt * chMatrix[i] * it->second.stateDeriv;
                }

                // Update the current error vector
                for (itkMatrix = kMatrix[i].stateMap.begin(), itError = errorMatrix.stateMap.begin(); itkMatrix != kMatrix[i].stateMap.end(); itkMatrix++, itError++)
                {
                    // Update the error vector with the appropriate coefficients
                    itError->second.state += hInt * ctMatrix[i] * itkMatrix->second.stateDeriv;
                }
            }

            // Calculate the relative error. The error is calculated using the norm of each state variable
            for (it = stateOut.stateMap.begin(), itError = errorMatrix.stateMap.begin(); it != stateOut.stateMap.end(); it++, itError++)
            {
                // Check if the norm is smaller than the absolute tolerance. If it is, calculate the error relative to the absolute tolerance instead
                if (it->second.state.norm() < this->absTol)
                    relError = itError->second.state.norm() / this->absTol;
                else {
                    relError = itError->second.state.norm() / it->second.state.norm();
                }
                
                // Save the maximum relative error to use on the time step refinement
                if (maxRelError < relError) {
                    maxRelError = relError;
                }
            }

            // Recalculate the time step. If the relative error is larger than the relative tolerance, then decrease the time step and vice-versa.
            h *= scaleFactor * pow(this->relTol / maxRelError, 0.2);
        }

        // Update the entire state vector after integration
        dynPtr->dynManager.updateStateVector(stateOut);

        // Update the initial state
        stateInit = dynPtr->dynManager.getStateVector();

        // Update the time
        t += hInt;

        // Check for overpassing time
        if (t + h > currentTime + timeStep) {
            h = currentTime + timeStep - t;
        }
    }
    
    return;
}
