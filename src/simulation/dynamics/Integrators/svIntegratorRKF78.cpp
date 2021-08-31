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


#include "svIntegratorRKF78.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include <stdio.h>

svIntegratorRKF78::svIntegratorRKF78(DynamicObject* dyn) : StateVecIntegrator(dyn)
{
    memset(alphaMatrix, 0x0, sizeof(alphaMatrix));
    memset(betaMatrix, 0x0, sizeof(betaMatrix));
    memset(chMatrix, 0x0, sizeof(chMatrix));
    
    chMatrix[5] = 34.0 / 105;
    chMatrix[6]= 9.0 / 35;
    chMatrix[7] = chMatrix[6];
    chMatrix[8]= 9.0 / 280;
    chMatrix[9]= chMatrix[8];
    chMatrix[11] = 41.0 / 840;
    chMatrix[12]= chMatrix[11];
    
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
    
    return;
}

svIntegratorRKF78::~svIntegratorRKF78()
{
    return;
}

void svIntegratorRKF78::integrate(double currentTime, double timeStep)
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
    
    for(uint64_t i=0; i<13; i++)
    {
        for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itInit = stateInit.stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itInit++)
        {
            it->second.state = itInit->second.state;
        }
        for(uint64_t j=0; j<i; j++)
        {
            for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itOut=kMatrix[j].stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itOut++)
            {
                it->second.state = it->second.state + timeStep*betaMatrix[i][j]*itOut->second.stateDeriv;
                
            }
            
        }
        dynPtr->equationsOfMotion(currentTime+timeStep*alphaMatrix[i]);
        kMatrix.push_back(dynPtr->dynManager.getStateVector());
        for (it = dynPtr->dynManager.stateContainer.stateMap.begin(), itOut = stateOut.stateMap.begin(); it != dynPtr->dynManager.stateContainer.stateMap.end(); it++, itOut++)
        {
            itOut->second.state  = itOut->second.state + timeStep*chMatrix[i]*it->second.stateDeriv;
        }
    }
    
    dynPtr->dynManager.updateStateVector(stateOut);	

    return;
}
