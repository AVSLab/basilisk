/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#include "simulation/environment/dentonFluxModel/dentonFluxModel.h"
#include "architecture/utilities/linearAlgebra.h"
#include <iostream>
#include <cstring>
#include <fstream>
#include <cmath> // trig

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */

 // Final Desired Constructor
DentonFluxModel::DentonFluxModel(int kp, int numEn)
{
    choose_kp = kp;
    numEnergies = numEn;
}

/*! Module Destructor */
DentonFluxModel::~DentonFluxModel()
{
    
}

/*! This method is used to reset the module and checks that required input messages are connect.
    @return void
*/
void DentonFluxModel::Reset(uint64_t CurrentSimNanos)
{
    // Check that required input messages are connected
    if (!this->satStateInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_ERROR, "DentonFluxModel.scStateInMsg was not linked.");
    }
    
    // Check the disired array size is not larger than the maximum value
    if (numEnergies > MAX_SIZE)
    {
        bskLogger.bskLog(BSK_ERROR, "Maximum array size exceeded.");
    }
}


/*! This method takes the computed shadow factors and outputs them to the
 messaging system.
 @param CurrentClock The current simulation time (used for time stamping)
 @return void
 */

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @return void
*/
void DentonFluxModel::UpdateState(uint64_t CurrentSimNanos)
{
    // Make local copies of messages
    SCStatesMsgPayload scStateInMsgBuffer;  //!< local copy of message buffer
    FluxMsgPayload fluxOutMsgBuffer;
    SpicePlanetStateMsgPayload sunSpiceInMsgBuffer;
    SpicePlanetStateMsgPayload earthSpiceInMsgBuffer;

    // Always zero the output message buffers before assigning values
    fluxOutMsgBuffer = this->fluxOutMsg.zeroMsgPayload;

    // Read in the input messages
    scStateInMsgBuffer = this->satStateInMsg();  //!< populating local copy
    sunSpiceInMsgBuffer = this->sunStateInputMsg();
    earthSpiceInMsgBuffer = this->earthStateInputMsg();
    
    // Set parameters
    int numKps = 28;
    int numEnergies = 40;
    int numLocalTimes = 24;
    
    // Define Energy Array
    double inputEnergies[numEnergies];
    double step = (40000 - 1)/numEnergies;
 
    inputEnergies[0] = 1;
    for (int i = 1; i < numEnergies; i++)
    {
        inputEnergies[i] = inputEnergies[i-1] + step;
    }
    
    // Electron: All F10.7
    double mean_e_all[numKps][numEnergies][numLocalTimes];
    
    // Ion: All F10.7
    double mean_i_all[numKps][numEnergies][numLocalTimes];
    
    // Input file stream object
    std::ifstream inputFile1;
    
    // Read data from file 1: electron all F10.7
    inputFile1.open("/Basilisk/src/simulation/environment/dentonFluxModel/data/model_e_array_all.txt");
    
    // Read information into arrays: MEAN
    for (int i = 0; i < numKps; i++)
    {   for (int j = 0; j < numEnergies; j++)
        {   for (int k = 0; k < numLocalTimes; k++)
            {   inputFile1 >> mean_e_all[i][j][k];
                //cout << mean_e_all[i][j][k];
            }
        }
    }
    
    // Close file
    inputFile1.close();
    
    // Input file stream object
    std::ifstream inputFile2;
    
    // Read data from file 2: ion all F10.7
    inputFile2.open("/Basilisk/src/simulation/environment/dentonFluxModel/data/model_i_array_all.txt");
    
    // Read information into arrays: MEAN
    for (int i = 0; i < numKps; i++)
    {   for (int j = 0; j < numEnergies; j++)
        {   for (int k = 0; k < numLocalTimes; k++)
            {   inputFile2 >> mean_i_all[i][j][k];
                //cout << mean_i_all[i][j][k];
            }
        }
    }
    
    // Close file
    inputFile2.close();
    
    // Fill average centre energies, normalized by satellite
    double enElec[40] = {1.034126,     1.346516,     1.817463,     2.399564,
    3.161048,     4.153217,     5.539430,     7.464148,
    9.836741,    12.543499,    16.062061,    20.876962,
    27.183572,    35.843437,    47.179073,    61.424732,
    80.120170,   104.563461,   136.914871,   179.740982,
    235.406829,   309.020721,   405.806213,   532.664123,
    699.243896,   917.146484,  1205.174438,  1582.510986,
    2069.619628,  2703.301269,  3540.124511,  4639.775390,
    6069.347656,  7957.457519, 10436.841796, 13677.195312,
    17923.560546, 23488.560546, 30782.000000, 40326.937500};
    
    double enProt[40] = { 1.816424,     2.284231,     2.904752,     3.639589,
    4.483188,     5.671049,     7.343667,     9.450922,
    11.934194,    15.105951,    19.372854,    24.943658,
    32.053474,    41.142940,    53.239536,    68.940170,
    89.082473,   115.585487,   150.529022,   196.249755,
    256.610107,   335.709136,   439.549621,   574.766357,
    749.907531,   982.261108,  1278.967041,  1662.856079,
    2170.886474,  2829.989013,  3691.509765,  4822.499023,
    6300.260742,  8217.569335, 10726.390625, 14001.280273,
    18276.244140, 23856.085937, 31140.962890, 40649.562500};
    
    // Define output (array if we open more than 2 files)
    double finalElecAll;
    double finalIonAll;
    //double finalElecAll[7]; //!< Desired end result for all F10.7
    //double finalIonAll[7];

    //  Calculate both Sun snd spacecraft position vectors from Earth in ECI frame
    double sat_r_EN_N[3] = {scStateInMsgBuffer.r_BN_N[0] - earthSpiceInMsgBuffer.PositionVector[0], scStateInMsgBuffer.r_BN_N[1] - earthSpiceInMsgBuffer.PositionVector[1], scStateInMsgBuffer.r_BN_N[2] - earthSpiceInMsgBuffer.PositionVector[2]};
    
    double sun_r_EN_N[3] = {sunSpiceInMsgBuffer.PositionVector[0] - earthSpiceInMsgBuffer.PositionVector[0], sunSpiceInMsgBuffer.PositionVector[1] - earthSpiceInMsgBuffer.PositionVector[1], sunSpiceInMsgBuffer.PositionVector[2] - earthSpiceInMsgBuffer.PositionVector[2]};
    
    // Find local lime from spacecraft and Earth state messages
    localTime = calcLocalTime(sat_r_EN_N, sun_r_EN_N);
    
    // For loop to calculate each element of output flux vectors
    for (int i = 0; i < numEnergies; i++)
    {
        choose_energy = inputEnergies[i];
            
        // Convert energies to log10
        double choose_energy_log = log(choose_energy);
        double logEnElec[numEnergies];
        double logEnProt[numEnergies];
        
        for (int k = 0; k < numEnergies; k++)
        {
            logEnElec[k] = log(enElec[k]);
            logEnProt[k] = log(enProt[k]);
        }
        
        // ELECTRONS: Find nearest neighbors in energy
        double eHigher = 0.0;
        double eLower = 0.0;
        int eHigherIndex = 0;
        int eLowerIndex = 0;
        
        for (int j = 0; j < numEnergies; j++)
        {
            if (logEnElec[j] > choose_energy_log)
            {
                eHigher = logEnElec[j];
                eLower = logEnElec[j-1];
                eHigherIndex = j;
                eLowerIndex = j-1;
                break;
            }
            else
            {
            }
        }
        
        // IONS: Find nearest neighbors in energy
        double iHigher = 0.0;
        double iLower = 0.0;
        int iHigherIndex = 0;
        int iLowerIndex = 0;
        
        for (int m = 0; m < numEnergies; m++)
        {
            if (logEnProt[m] > choose_energy_log)
            {
                iHigher = logEnProt[m];
                iLower = logEnProt[m-1];
                iHigherIndex = m;
                iLowerIndex = m-1;
                break;
            }
            else
            {
            }
        }
        
        int localTimeFloor = floor(localTime + 1);
        int localTimeCeil = ceil(localTime + 1);
        
        // Initialize flux variables
        double flux11 = 0.0;
        double flux12 = 0.0;
        double flux13 = 0.0;
        double flux14 = 0.0;
        
        // ELECTRON: Gather four nearest *MEAN* flux values for *ALL F10.7*
        flux11 = mean_e_all[choose_kp][eLowerIndex][localTimeFloor];
        flux12 = mean_e_all[choose_kp][eHigherIndex][localTimeFloor];
        flux13 = mean_e_all[choose_kp][eLowerIndex][localTimeCeil];
        flux14 = mean_e_all[choose_kp][eHigherIndex][localTimeCeil];
        
        // ELECTRON: Find flux
        //finalElecAll[0] = bilinear((localTimeFloor - 1), (localTimeCeil-1), logEnElec[eLowerIndex], logEnElec[eHigherIndex], localTime, choose_energy_log, flux11, flux12, flux13, flux14); // Future
        finalElecAll = bilinear((localTimeFloor - 1), (localTimeCeil-1), logEnElec[eLowerIndex], logEnElec[eHigherIndex], localTime, choose_energy_log, flux11, flux12, flux13, flux14);
        
        // ION: Gather four nearest *MEAN* flux values for *ALL F10.7*
        flux11 = mean_i_all[choose_kp][iLowerIndex][localTimeFloor];
        flux12 = mean_i_all[choose_kp][iHigherIndex][localTimeFloor];
        flux13 = mean_i_all[choose_kp][iLowerIndex][localTimeCeil];
        flux14 = mean_i_all[choose_kp][iHigherIndex][localTimeCeil];
        
        // ION: Find flux
        //finalIonAll[0] = bilinear(localTimeFloor, localTimeCeil, logEnProt[iHigherIndex], logEnProt[iLowerIndex], localTime, choose_energy_log, flux11, flux12, flux13, flux14); // Future
        finalIonAll = bilinear(localTimeFloor, localTimeCeil, logEnProt[iHigherIndex], logEnProt[iLowerIndex], localTime, choose_energy_log, flux11, flux12, flux13, flux14);
        
        // Store the output message
        fluxOutMsgBuffer.meanElectronFlux[i] = finalElecAll;
        fluxOutMsgBuffer.meanIonFlux[i] = finalIonAll;
        fluxOutMsgBuffer.energies[i] = choose_energy;
    }
    
    // Write to the output message
    this->fluxOutMsg.write(&fluxOutMsgBuffer, this->moduleID, CurrentSimNanos);
}

// Calcualte local time method
double DentonFluxModel::calcLocalTime(double sunIPosVec[3], double scIPosVec[3])
{
    double localTime;
    
    // Calculate 2D position vector magnitudes
    double sc2DIPosMag = sqrt(scIPosVec[0]*scIPosVec[0] + scIPosVec[1]*scIPosVec[1]);
    double sun2DIPosMag = sqrt(sunIPosVec[0]*sunIPosVec[0] + sunIPosVec[1]*sunIPosVec[1]);
    
    // Convert positions to 2D unit vectors
    double sc2DIPos_hat[2] = {scIPosVec[0]/sc2DIPosMag, scIPosVec[1]/sc2DIPosMag};
    double sun2DIPos_hat[2] = {sunIPosVec[0]/sun2DIPosMag, sunIPosVec[1]/sun2DIPosMag};
    
    // Determine Local Time: Using atan2()
    double x = v2Dot(sc2DIPos_hat, sun2DIPos_hat);
    double y = sun2DIPos_hat[0]*sc2DIPos_hat[1] - sun2DIPos_hat[1]*sc2DIPos_hat[0];
    
    double theta = atan2(y,x)*(3.141592654/180);

    if (y == 1 || y == -1)
    {
        localTime = 0.0;    //!<  Data files are from 0-23 LT
    }
    else
    {
        localTime = 12.00 + (theta / 360)*24;
    }
    
    return localTime;
    
}

// Bilinear interpolation method
double DentonFluxModel::bilinear(int x1, int x2, double y1, double y2, double x, double y, double f11, double f12, double f13, double f14)
{
  // Define variables
  double R1, R2, bilinear = 0.0;
  if (x1 != x2)
  {
      R1 = ( (x2 - x) / (x2 - x1) ) * f11 + ( (x - x1) / (x2 - x1) ) * f13;
      R2 = ( (x2 - x) / (x2 - x1) ) * f12 + ( (x - x1) / (x2 - x1) ) * f14;
      bilinear = ( (y2 - y ) / (y2 - y1) ) * R1 + ( (y - y1) / (y2 - y1) ) * R2;
      return bilinear;
  }
  else
  {
      bilinear = ( (y2 - y ) / (y2 - y1) ) * f11 + ( (y - y1) / (y2 - y1) ) * f13;
      return bilinear;
  }

}
