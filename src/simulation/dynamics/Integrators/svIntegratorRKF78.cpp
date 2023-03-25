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


#include "svIntegratorRKF78.h"
#include "../_GeneralModuleFiles/dynamicObject.h"


svIntegratorRKF78::svIntegratorRKF78(DynamicObject* dyn) : StateVecIntegrator(dyn)
{
    memset(alphaMatrix, 0x0, sizeof(alphaMatrix));
    memset(betaMatrix, 0x0, sizeof(betaMatrix));
    memset(chMatrix, 0x0, sizeof(chMatrix));
    memset(ctMatrix, 0x0, sizeof(ctMatrix));
    
    chMatrix[5] = 34.0 / 105;
    chMatrix[6]= 9.0 / 35;
    chMatrix[7] = chMatrix[6];
    chMatrix[8]= 9.0 / 280;
    chMatrix[9]= chMatrix[8];
    chMatrix[11] = 41.0 / 840;
    chMatrix[12]= chMatrix[11];

    ctMatrix[0] = -41.0 / 840.0;
    ctMatrix[10] = ctMatrix[0];
    ctMatrix[11] = 41.0 / 840.0;
    ctMatrix[12] = ctMatrix[11];
    
    alphaMatrix[1] = 2.0/27.0;
    alphaMatrix[2] = 1.0/9.0;
    alphaMatrix[3] = 1.0/6.0;
    alphaMatrix[4] = 5.0/12.0;
    alphaMatrix[5] = 1.0/2.0;
    alphaMatrix[6] = 5.0/6.0;
    alphaMatrix[7] = 1.0/6.0;
    alphaMatrix[8] = 2.0/3.0;
    alphaMatrix[9] = 1.0/3.0;
    alphaMatrix[10] = 1.0;
    alphaMatrix[12] = 1.0;
    
    betaMatrix[1][0] = 2.0 / 27;
    betaMatrix[2][0] = 1.0 / 36;
    betaMatrix[3][0] = 1.0 / 24;
    betaMatrix[4][0] = 5.0 / 12;
    betaMatrix[5][0] = 0.05;
    betaMatrix[6][0] = -25.0 / 108;
    betaMatrix[7][0] = 31.0 / 300;
    betaMatrix[8][0] = 2.0;
    betaMatrix[9][0] = -91.0 / 108;
    betaMatrix[10][0] = 2383.0 / 4100;
    betaMatrix[11][0] = 3.0 / 205;
    betaMatrix[12][0] = -1777.0 / 4100;
    betaMatrix[2][1] = 1.0 / 12;
    betaMatrix[3][2] = 1.0 / 8;
    betaMatrix[4][2] = -25.0 / 16;
    betaMatrix[4][3] = -betaMatrix[4][2];
    betaMatrix[5][3] = 0.25;
    betaMatrix[6][3] = 125.0 / 108;
    betaMatrix[8][3] = -53.0 / 6;
    betaMatrix[9][3] = 23.0 / 108;
    betaMatrix[10][3] = -341.0 / 164;
    betaMatrix[12][3] = betaMatrix[10][3];
    betaMatrix[5][4] = 0.2;
    betaMatrix[6][4] = -65.0 / 27;
    betaMatrix[7][4] = 61.0 / 225;
    betaMatrix[8][4] = 704.0 / 45;
    betaMatrix[9][4] = -976.0 / 135;
    betaMatrix[10][4] = 4496.0 / 1025;
    betaMatrix[12][4] = betaMatrix[10][4];
    betaMatrix[6][5] = 125.0 / 54;
    betaMatrix[7][5] = -2.0 / 9;
    betaMatrix[8][5] = -107.0 / 9;
    betaMatrix[9][5] = 311.0 / 54;
    betaMatrix[10][5] = -301.0 / 82;
    betaMatrix[11][5] = -6.0 / 41;
    betaMatrix[12][5] = -289.0 / 82;
    betaMatrix[7][6] = 13.0 / 900;
    betaMatrix[8][6] = 67.0 / 90;
    betaMatrix[9][6] = -19.0 / 60;
    betaMatrix[10][6] = 2133.0 / 4100;
    betaMatrix[11][6] = -3.0 / 205;
    betaMatrix[12][6] = 2193.0 / 4100;
    betaMatrix[8][7] = 3.0;
    betaMatrix[9][7] = 17.0 / 6;
    betaMatrix[10][7] = 45.0 / 82;
    betaMatrix[11][7] = -3.0 / 41;
    betaMatrix[12][7] = 51.0 / 82;
    betaMatrix[9][8] = -1.0 / 12;
    betaMatrix[10][8] = 45.0 / 164;
    betaMatrix[11][8] = 3.0 / 41;
    betaMatrix[12][8] = 33.0 / 164;
    betaMatrix[10][9] = 18.0 / 41;
    betaMatrix[11][9] = 6.0 / 41;
    betaMatrix[12][9] = 12.0 / 41;
    betaMatrix[12][11] = 1.0;

    // Set the default values for absolute and relative tolerance
    this->absTol = 1e-8;
    this->relTol = 1e-4;
}

svIntegratorRKF78::~svIntegratorRKF78()
{
}


/*<!
 Implements a 7th order Runge Kutta Fehlberg variable time step integration method
 */
void svIntegratorRKF78::integrate(double currentTime, double timeStep)
{
    std::vector<StateVector> stateOut;
    std::vector<StateVector> stateInit;
    std::vector<StateVector> errorMatrix;  // error state vector
    std::vector<std::vector<StateVector>> kMatrix;  // matrix of k coefficients
    std::map<std::string, StateData>::iterator it;
    std::map<std::string, StateData>::iterator itOut;
    std::map<std::string, StateData>::iterator itInit;
    std::map<std::string, StateData>::iterator itkMatrix;
    std::map<std::string, StateData>::iterator itError;

    for (const auto& dynPtr : this->dynPtrs) {
        stateOut.push_back(dynPtr->dynManager.getStateVector());  // copy current state variables
        stateInit.push_back(dynPtr->dynManager.getStateVector());  // copy current state variables
        errorMatrix.push_back(dynPtr->dynManager.getStateVector());  // copy current state variables
    }
    kMatrix.resize(this->dynPtrs.size());
    double h = timeStep;  // updated variable time step that depends on the relative error and the relative tolerance
    double t = currentTime;  // integration time
    double hInt = timeStep;  // time step used for the current integration loop
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

            // loop over all the dynamic object elements
            for (int c = 0; c < this->dynPtrs.size(); c++) {
                // Reset the k matrix
                kMatrix.at(c).clear();

                // Compute the equations of motion for t
                dynPtrs.at(c)->equationsOfMotion(t, hInt);
            }
            for (int c = 0; c < this->dynPtrs.size(); c++) {
                // Reset the ouput and error vectors
                for (itOut = stateOut.at(c).stateMap.begin(),
                     itInit = stateInit.at(c).stateMap.begin(),
                     itError = errorMatrix.at(c).stateMap.begin();
                     itOut != stateOut.at(c).stateMap.end();
                     itOut++,
                     itInit++,
                     itError++)
                {
                    itOut->second.state = itInit->second.state;
                    itError->second.state.setZero();
                }
            }

            // Loop through all 13 coefficients (k1 through k13)
            for (uint64_t i = 0; i < 13; i++)
            {
                for (int c = 0; c < this->dynPtrs.size(); c++) {
                    // Initialize the state iterators. The state variables are defined in a dictionary and are pulled and populated one by one
                    for (it = dynPtrs.at(c)->dynManager.stateContainer.stateMap.begin(), itInit = stateInit.at(c).stateMap.begin(); it != dynPtrs.at(c)->dynManager.stateContainer.stateMap.end(); it++, itInit++)
                    {
                        it->second.state = itInit->second.state;
                    }

                    // Loop through the B matrix coefficients that define the point of integration
                    for (uint64_t j = 0; j < i; j++)
                    {
                        for (it = dynPtrs.at(c)->dynManager.stateContainer.stateMap.begin(), itOut = kMatrix.at(c)[j].stateMap.begin(); it != dynPtrs.at(c)->dynManager.stateContainer.stateMap.end(); it++, itOut++)
                        {
                            it->second.state = it->second.state + hInt * betaMatrix[i][j] * itOut->second.stateDeriv;
                        }
                    }
                }
                for (int c = 0; c < this->dynPtrs.size(); c++) {

                    // Integrate with the appropriate time step using the A matrix coefficients
                    dynPtrs.at(c)->equationsOfMotion(t + hInt * alphaMatrix[i], hInt);

                    // Save the current k coefficient
                    kMatrix.at(c).push_back(dynPtrs.at(c)->dynManager.getStateVector());
                }
                for (int c = 0; c < this->dynPtrs.size(); c++) {
                    // Update the state at the end of the current integration step
                    for (it = dynPtrs.at(c)->dynManager.stateContainer.stateMap.begin(), itOut = stateOut.at(c).stateMap.begin(); it != dynPtrs.at(c)->dynManager.stateContainer.stateMap.end(); it++, itOut++)
                    {
                        itOut->second.state += hInt * chMatrix[i] * it->second.stateDeriv;
                    }

                    // Update the current error vector
                    for (itkMatrix = kMatrix.at(c)[i].stateMap.begin(), itError = errorMatrix.at(c).stateMap.begin(); itkMatrix != kMatrix.at(c)[i].stateMap.end(); itkMatrix++, itError++)
                    {
                        // Update the error vector with the appropriate coefficients
                        itError->second.state += hInt * ctMatrix[i] * itkMatrix->second.stateDeriv;
                    }
                }
            }

            for (int c = 0; c < this->dynPtrs.size(); c++) {
                // Calculate the relative error. The error is calculated using the norm of each state variable
                for (it = stateOut.at(c).stateMap.begin(), itError = errorMatrix.at(c).stateMap.begin(); it != stateOut.at(c).stateMap.end(); it++, itError++)
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
            }

            // Recalculate the time step. If the relative error is larger than the relative tolerance, then decrease the time step and vice-versa.
            h *= scaleFactor * pow(this->relTol / maxRelError, 0.2);
        }

        for (int c = 0; c < this->dynPtrs.size(); c++) {
            // Update the entire state vector after integration
            dynPtrs.at(c)->dynManager.updateStateVector(stateOut.at(c));

            // Update the initial state
            stateInit.at(c) = dynPtrs.at(c)->dynManager.getStateVector();
        }
        // Update the time
        t += hInt;

        // Check for overpassing time
        if (t + h > currentTime + timeStep) {
            h = currentTime + timeStep - t;
        }
    }
}
