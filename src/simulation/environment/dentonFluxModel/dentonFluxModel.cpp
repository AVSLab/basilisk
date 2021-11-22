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
#include <cmath>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
DentonFluxModel::DentonFluxModel(int kp, double energy)
{
    choose_kp = kp;
    choose_energy = energy;
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
    /*
    // Check that required input messages are connected
    if (!this->satStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "DentonFluxModel.scStateInMsg was not linked.");
    }
     */

}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @return void
*/
void DentonFluxModel::UpdateState(uint64_t CurrentSimNanos)
{
    // Make local copies of messages
    SCStatesMsgPayload scStateInMsgBuffer;  //!< local copy of message buffer
    FluxMsgPayload fluxOutMsgBuffer;  //!< local copy of message buffer
    //SpicePlanetStateMsgPayload sunSpiceInMsgBuffer;

    // Always zero the output message buffers before assigning values
    fluxOutMsgBuffer = this->fluxOutMsg.zeroMsgPayload;

    // Read in the input messages
    scStateInMsgBuffer = this->satStateInMsg();
    //sunSpiceInMsgBuffer = this->sunStateInputMsg();
    
    // Set parameters
    int numKps = 28;
    int numEnergies = 40;
    int numLocalTimes = 24;
    
    // Electron: All F10.7
    long double mean_e_all[numKps][numEnergies][numLocalTimes];
    
    // Ion: All F10.7
    long double mean_i_all[numKps][numEnergies][numLocalTimes];
    
    // Input file stream object
    std::ifstream inputFile1;
    
    // Read data from file 1: electron all F10.7
    inputFile1.open("/Users/leahkiner/Repositories/flux_model_v1.0/data/model_e_array_all.txt");
    
    // Read information into arrays: MEAN
    for (int i = 0; i < numKps; i++)
    {
        for (int j = 0; j < numEnergies; j++)
        {
            for (int k = 0; k < numLocalTimes; k++)
            {
                inputFile1 >> mean_e_all[i][j][k];
                //cout << mean_e_all[i][j][k];
            }
        }
    }
    
    // Close file
    inputFile1.close();
    
    // Input file stream object
    std::ifstream inputFile2;
    
    // Read data from file 2: ion all F10.7
    inputFile2.open("/Users/leahkiner/Repositories/flux_model_v1.0/data/model_i_array_all.txt");
    
    // Read information into arrays: MEAN
    for (int i = 0; i < numKps; i++)
    {
        for (int j = 0; j < numEnergies; j++)
        {
            for (int k = 0; k < numLocalTimes; k++)
            {
                inputFile2 >> mean_i_all[i][j][k];
                //cout << mean_i_all[i][j][k];
            }
        }
    }
    
    // Close file
    inputFile2.close();
    
    // Fill average centre energies, normalized by satellite
    long double enElec[40] = {1.034126,     1.346516,     1.817463,     2.399564,
    3.161048,     4.153217,     5.539430,     7.464148,
    9.836741,    12.543499,    16.062061,    20.876962,
    27.183572,    35.843437,    47.179073,    61.424732,
    80.120170,   104.563461,   136.914871,   179.740982,
    235.406829,   309.020721,   405.806213,   532.664123,
    699.243896,   917.146484,  1205.174438,  1582.510986,
    2069.619628,  2703.301269,  3540.124511,  4639.775390,
    6069.347656,  7957.457519, 10436.841796, 13677.195312,
    17923.560546, 23488.560546, 30782.000000, 40326.937500};
    
    long double enProt[40] = { 1.816424,     2.284231,     2.904752,     3.639589,
    4.483188,     5.671049,     7.343667,     9.450922,
    11.934194,    15.105951,    19.372854,    24.943658,
    32.053474,    41.142940,    53.239536,    68.940170,
    89.082473,   115.585487,   150.529022,   196.249755,
    256.610107,   335.709136,   439.549621,   574.766357,
    749.907531,   982.261108,  1278.967041,  1662.856079,
    2170.886474,  2829.989013,  3691.509765,  4822.499023,
    6300.260742,  8217.569335, 10726.390625, 14001.280273,
    18276.244140, 23856.085937, 31140.962890, 40649.562500};
    
    // Define output array
    //long double finalElecAll[7];
    //long double finalIonAll[7];
    long double finalElecAll;
    long double finalIonAll;
    
    // Convert energies to log10
    long double choose_energy_log = log(choose_energy);
    long double logEnElec[numEnergies];
    long double logEnProt[numEnergies];
    
    for (int i = 0; i < numEnergies; i++)
    {
        logEnElec[i] = log(enElec[i]);
        logEnProt[i] = log(enProt[i]);
    }
    
    // ELECTRONS: Find nearest neighbors in energy
    long double eHigher = 0.0;
    long double eLower = 0.0;
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
    long double iHigher = 0.0;
    long double iLower = 0.0;
    int iHigherIndex = 0;
    int iLowerIndex = 0;
    
    for (int k = 0; k < numEnergies; k++)
    {
        if (logEnProt[k] > choose_energy_log)
        {
            iHigher = logEnProt[k];
            iLower = logEnProt[k-1];
            iHigherIndex = k;
            iLowerIndex = k-1;
            break;
        }
        else
        {
        }
    }
    
    // Find local lime from spacecraft state message
    //choose_local_time = calcLocalTime(sunSpiceInMsgBuffer.PositionVector, scStateInMsgBuffer.r_BN_N);
    double PositionVector[3] = {1, 0, 0};
    choose_local_time = calcLocalTime(PositionVector, scStateInMsgBuffer.r_BN_N);

    //choose_local_time = 12.07;
    
    int localTimeFloor = floor(choose_local_time + 1);
    int localTimeCeil = ceil(choose_local_time + 1);
    
    // Initialize flux variables
    long double flux11 = 0.0;
    long double flux12 = 0.0;
    long double flux13 = 0.0;
    long double flux14 = 0.0;
    
    // ELECTRON: Gather four nearest *MEAN* flux values for *ALL F10.7*
    flux11 = mean_e_all[choose_kp][eLowerIndex][localTimeFloor];
    flux12 = mean_e_all[choose_kp][eHigherIndex][localTimeFloor];
    flux13 = mean_e_all[choose_kp][eLowerIndex][localTimeCeil];
    flux14 = mean_e_all[choose_kp][eHigherIndex][localTimeCeil];
    
    // ELECTRON: Find flux
    //finalElecAll[0] = bilinear((localTimeFloor - 1), (localTimeCeil-1), logEnElec[eLowerIndex], logEnElec[eHigherIndex], choose_local_time, choose_energy_log, flux11, flux12, flux13, flux14); // Future
    finalElecAll = bilinear((localTimeFloor - 1), (localTimeCeil-1), logEnElec[eLowerIndex], logEnElec[eHigherIndex], choose_local_time, choose_energy_log, flux11, flux12, flux13, flux14);
    
    // ION: Gather four nearest *MEAN* flux values for *ALL F10.7*
    flux11 = mean_i_all[choose_kp][iLowerIndex][localTimeFloor];
    flux12 = mean_i_all[choose_kp][iHigherIndex][localTimeFloor];
    flux13 = mean_i_all[choose_kp][iLowerIndex][localTimeCeil];
    flux14 = mean_i_all[choose_kp][iHigherIndex][localTimeCeil];
    
    // ION: Find flux
    //finalIonAll[0] = bilinear(localTimeFloor, localTimeCeil, logEnProt[iHigherIndex], logEnProt[iLowerIndex], choose_local_time, choose_energy_log, flux11, flux12, flux13, flux14); // Future
    finalIonAll = bilinear(localTimeFloor, localTimeCeil, logEnProt[iHigherIndex], logEnProt[iLowerIndex], choose_local_time, choose_energy_log, flux11, flux12, flux13, flux14);
    
    /*! - store the output message */
    fluxOutMsgBuffer.MeanElectronFlux = finalElecAll;
    fluxOutMsgBuffer.MeanIonFlux = finalElecAll;
    
    // Write to the output messages
    this->fluxOutMsg.write(&fluxOutMsgBuffer, this->moduleID, CurrentSimNanos);
}


// Calcualte local time method
double DentonFluxModel::calcLocalTime(double sunIPosVec[3], double scIPosVec[3])
{
    double choose_local_time;
    
    // Calculate 2D position vector magnitudes
    double sc2DIPosMag = sqrt(scIPosVec[0]*scIPosVec[0] + scIPosVec[1]*scIPosVec[1]);
    double sun2DIPosMag = sqrt(sunIPosVec[0]*sunIPosVec[0] + sunIPosVec[1]*sunIPosVec[1]);
    
    // Convert positions to 2D unit vectors
    double sc2DIPos_hat[2] = {scIPosVec[0]/sc2DIPosMag, scIPosVec[1]/sc2DIPosMag};
    double sun2DIPos_hat[2] = {sunIPosVec[0]/sun2DIPosMag, sunIPosVec[1]/sun2DIPosMag};

   // Find (right-handed) perpendicular unit vector to the Sun unit vector relative to the ECI frame
    double I = sun2DIPos_hat[0];
    double J = sun2DIPos_hat[1];
    
    double sun2DIPos_perp_hat[2];
    
    if (I > 0)
    {
        if (J < 0)
        {
            //sun2DIPos_perp_hat = {I, -J};
            sun2DIPos_perp_hat[0] = I;
            sun2DIPos_perp_hat[1] = -J;
        }
        else // J > 0
        {
            //sun2DIPos_perp_hat = {-I, J};
            sun2DIPos_perp_hat[0] = -I;
            sun2DIPos_perp_hat[1] = J;
            
        }
    }
    else // I < 0
    {
        if (J < 0)
        {
            //sun2DIPos_perp_hat = {-I, -J};
            sun2DIPos_perp_hat[0] = -I;
            sun2DIPos_perp_hat[1] = -J;
        }
        else // J > 0
        {
            //sun2DIPos_perp_hat = {I, -J};
            sun2DIPos_perp_hat[0] = I;
            sun2DIPos_perp_hat[1] = -J;
        }
    }

    // Dot spacecraft position vector with the Earth-Sun radial unit vector (Only equatorial components)
    double dot_Sc_Sun = (sc2DIPos_hat[0]*sun2DIPos_hat[0] + sc2DIPos_hat[1]*sun2DIPos_hat[1]) / (sc2DIPosMag*sun2DIPosMag);

    // Dot spacecraft position vector with the vectory perpendicular to the Earth-Sun radial unit vector (Only equatorial components)
    double dot_Sc_Sun_perp = (sc2DIPos_hat[0]*sun2DIPos_perp_hat[0] + sc2DIPos_hat[1]*sun2DIPos_perp_hat[1]) / (sc2DIPosMag*sun2DIPosMag);
    
    // Find theta, the angle from the Earth-Sun radial unit vector (degrees)
    double theta = (acos(dot_Sc_Sun)*180)/(3.14);

    // Determine local time
    if (dot_Sc_Sun > 0) // 06:00 < LT < 18:00
    {
        if (dot_Sc_Sun_perp < 0)
        {
            choose_local_time = ((theta*24) / 360) + 6.00;              // CASE A
        }
        else
        {
            choose_local_time = ((theta*24) / 360) + 12.00;             // CASE B
        }
    }
    else if (dot_Sc_Sun < 0) // 18:00 < LT < 06:00
    {
        if (dot_Sc_Sun_perp < 0)
        {
            choose_local_time = (((180 - theta)*24) / 360);             // CASE C
        }
        else
        {
            choose_local_time = (((180 - theta)*24) / 360) + 18.00;     // CASE D
        }
    }
    else if (dot_Sc_Sun == 0)
    {
        // Either 18:00 or 6:00
        if (dot_Sc_Sun_perp == 1)
        {
            choose_local_time = 18.00;
        }
        else if(dot_Sc_Sun_perp == -1)
        {
            choose_local_time = 6.00;
        }
        else
        {
            // SOMETHING IS WRONG......
            choose_local_time = 99.99;
        }
    }
    else if (dot_Sc_Sun == 1)
    {
        choose_local_time = 12.00;
    }
    else if (dot_Sc_Sun == -1)
    {
        choose_local_time = 00.00;
    }
    else
    {
        // SOMETHING IS WRONG......
        choose_local_time = 99.99;
    }
    
    return choose_local_time;

}

// Bilinear interpolation method
long double DentonFluxModel::bilinear(int x1, int x2, long double y1, long double y2, double x, long double y, long double f11, long double f12, long double f13, long double f14)
{
  // Define variables
  long double R1, R2, Bilinear = 0.0;
  if (x1 != x2)
  {
      R1 = ( (x2 - x) / (x2 - x1) ) * f11 + ( (x - x1) / (x2 - x1) ) * f13;
      R2 = ( (x2 - x) / (x2 - x1) ) * f12 + ( (x - x1) / (x2 - x1) ) * f14;
      Bilinear = ( (y2 - y ) / (y2 - y1) ) * R1 + ( (y - y1) / (y2 - y1) ) * R2;
      return Bilinear;
  }
  else
  {
      Bilinear = ( (y2 - y ) / (y2 - y1) ) * f11 + ( (y - y1) / (y2 - y1) ) * f13;
      return Bilinear;
  }

}















/*
// Calcualte local time method
double DentonFluxModel::calcLocalTime(double pnMatrix[3][3], double scIPosVec[3])
{
    double choose_local_time;
    
    // Gather spacecraft position vector, Earth-Sun radial unit vector, and unit vector perpendicular to the Earth-Sun radial unit vector
    double u_hat[3] = {pnMatrix[0][0], pnMatrix[0][1], pnMatrix[0][2]};    // Earth-Sun Radial Unit Vector
    double u_hat_perp[3] = {pnMatrix[1][0], pnMatrix[1][1], pnMatrix[1][2]};
    
    // Gather magnitudes of equatorial projections of spacecraft position and Earth-Sun radial unit vector
    double u_hat_eq_mag = sqrt(u_hat[0]*u_hat[0] + u_hat[1]*u_hat[1]);
    double u_hat_perp_eq_mag = sqrt(u_hat_perp[0]*u_hat_perp[0] + u_hat_perp[1]*u_hat_perp[1]);

    double r_BN_N_eq_mag = sqrt(scIPosVec[0]*scIPosVec[0] + scIPosVec[1]*scIPosVec[1]);
    
    // Linear Algebra Toolbox
    // Dot spacecraft position vector with the Earth-Sun radial unit vector (Only equatorial components)
    double dot_uHat_r_BN_N = (scIPosVec[0]*u_hat[0] + scIPosVec[1]*u_hat[1]) / (u_hat_eq_mag*r_BN_N_eq_mag);
    
    // Dot spacecraft position vector with the vectory perpendicular to the Earth-Sun radial unit vector (Only equatorial components)
    double dot_uHatPerp_r_BN_N = (scIPosVec[0]*u_hat_perp[0] + scIPosVec[1]*u_hat_perp[1]) / (u_hat_perp_eq_mag*r_BN_N_eq_mag);
    
    // Find theta, the angle from the Earth-Sun radial unit vector (degrees)
    double theta = (acos(dot_uHat_r_BN_N)*180)/(3.14);

    // Determine local time
    if (dot_uHat_r_BN_N > 0)
    {
        // 06:00 < LT < 18:00
        if (dot_uHatPerp_r_BN_N < 0)
        {
            choose_local_time = ((theta*24) / 360) + 6.00;              // CASE A
        }
        else
        {
            choose_local_time = ((theta*24) / 360) + 12.00;             // CASE B
        }
    }
    else if (dot_uHat_r_BN_N < 0)
    {
        // 18:00 < LT < 06:00
        if (dot_uHatPerp_r_BN_N < 0)
        {
            choose_local_time = (((180 - theta)*24) / 360);             // CASE C
        }
        else
        {
            choose_local_time = (((180 - theta)*24) / 360) + 18.00;     // CASE D
        }
    }
    else if (dot_uHat_r_BN_N == 0)
    {
        // Either 18:00 or 6:00
        if (dot_uHatPerp_r_BN_N == 1)
        {
            choose_local_time = 18.00;
        }
        else if(dot_uHatPerp_r_BN_N == -1)
        {
            choose_local_time = 6.00;
        }
        else
        {
            // SOMETHING IS WRONG......
            choose_local_time = 99.99;
        }
    }
    else if (dot_uHat_r_BN_N == 1)
    {
        choose_local_time = 12.00;
    }
    else if (dot_uHat_r_BN_N == -1)
    {
        choose_local_time = 00.00;
    }
    else
    {
        // SOMETHING IS WRONG......
        choose_local_time = 99.99;
    }
    
    return choose_local_time;

}
*/
