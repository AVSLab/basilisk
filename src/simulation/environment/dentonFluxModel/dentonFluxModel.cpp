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
#include "architecture/utilities/astroConstants.h"

#include <iostream>
#include <cstring>
#include <fstream>
#include <cmath> // trig

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */

 // Final Desired Constructor
DentonFluxModel::DentonFluxModel()
{
}

/*! Module Destructor */
DentonFluxModel::~DentonFluxModel()
{
    
}

/*! This method is used to reset the module and checks that required input messages are connect.
    @param CurrentSimNanos current simulation time in nano-seconds
    @return void
*/
void DentonFluxModel::Reset(uint64_t CurrentSimNanos)
{
    // Check that required input messages are connected
    if (!this->scStateInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_ERROR, "DentonFluxModel.scStateInMsg was not linked.");
    }

    if (!this->earthStateInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_ERROR, "DentonFluxModel.earthStateInMsg was not linked.");
    }

    if (!this->sunStateInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_ERROR, "DentonFluxModel.sunStateInMsg was not linked.");
    }
    
    if (this->kpIndex < 0) {
        bskLogger.bskLog(BSK_ERROR, "DentonFluxModel.kpIndex was not set to a proper value.");
    }

    // Check the desired array size is not larger than the maximum value
    if (this->numOutputEnergies > MAX_PLASMA_FLUX_SIZE)
    {
        bskLogger.bskLog(BSK_ERROR, "DentonFluxModel: Maximum denton space weather array size exceeded.");
    }
    // Check Kp is not larger than 9 (corresponding to maximum index 27)
    if (this->kpIndex > MAX_NUM_KPS - 1)
    {
        bskLogger.bskLog(BSK_ERROR, "DentonFluxModel: Maximum Kp index exceeded.");
    }
    if (this->numOutputEnergies < 0)
    {
        bskLogger.bskLog(BSK_ERROR, "DentonFluxModel.numEnergies was not set.");
    }
    if (this->dataPath == "") {
        bskLogger.bskLog(BSK_ERROR, "DentonFluxModel.dataPath was not set.");
    }
    
    // convert energies to log10 values
    for (int k = 0; k < MAX_NUM_ENERGIES; k++)
    {
        this->logEnElec[k] = log(this->enElec[k]);
        this->logEnProt[k] = log(this->enProt[k]);
    }

    // Define Energy Array
    double step = (40000 - 1)/this->numOutputEnergies;
 
    this->inputEnergies[0] = 1;
    for (int i = 1; i < numOutputEnergies; i++)
    {
        this->inputEnergies[i] = this->inputEnergies[i-1] + step;
    }

    // Read in Denton data files
    readDentonDataFile(this->eDataFileName, this->mean_e_flux);
    readDentonDataFile(this->iDataFileName, this->mean_i_flux);

}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @param CurrentSimNanos current simulation time in nano-seconds
    @return void
*/
void DentonFluxModel::UpdateState(uint64_t CurrentSimNanos)
{
    // Make local copies of messages
    SCStatesMsgPayload scStateInMsgBuffer;  //!< local copy of spacecraft states
    PlasmaFluxMsgPayload fluxOutMsgBuffer; //!< local copy of the plasma flux output message content
    SpicePlanetStateMsgPayload sunSpiceInMsgBuffer;  //!< local copy of the sun state input message payload
    SpicePlanetStateMsgPayload earthSpiceInMsgBuffer;  //!< local copy of the earth state input message payload

    // Always zero the output message buffers before assigning values
    fluxOutMsgBuffer = this->fluxOutMsg.zeroMsgPayload;

    // Read in the input messages
    scStateInMsgBuffer = this->scStateInMsg();
    sunSpiceInMsgBuffer = this->sunStateInMsg();
    earthSpiceInMsgBuffer = this->earthStateInMsg();
    
    // Define output (array if we open more than 2 files)
    double finalElec;
    double finalIon;

    //  Calculate both Sun (S) and spacecraft (B) position vectors from Earth (E) in ECI frame
    double r_BE_N[3];       /* satellite position relative to Earth in N frame components */
    double r_SE_N[3];       /* sun position relative to Earth in N frame components */
    v3Subtract(scStateInMsgBuffer.r_BN_N, earthSpiceInMsgBuffer.PositionVector, r_BE_N);
    v3Subtract(sunSpiceInMsgBuffer.PositionVector, earthSpiceInMsgBuffer.PositionVector, r_SE_N);

    // Find local lime from spacecraft and Earth state messages
    calcLocalTime(r_BE_N, r_SE_N);
    
    // For loop to calculate each element of output flux vectors
    for (int i = 0; i < numOutputEnergies; i++)
    {
        // Convert energies to log10
        double logInputEnergy = log(inputEnergies[i]);
                
        // ELECTRONS: Find nearest neighbors in energy
        double eHigher = 0.0;
        double eLower = 0.0;
        int eHigherIndex = 0;
        int eLowerIndex = 0;
        
        for (int j = 0; j < MAX_NUM_ENERGIES; j++)
        {
            if (logEnElec[j] > logInputEnergy)
            {
                int k = 0;
                if (j == 0)
                {
                    k = j+1;
                }
                else
                {
                    k = j;
                }
                eHigher = logEnElec[k];
                eLower = logEnElec[k-1];
                eHigherIndex = k;
                eLowerIndex = k-1;
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
        
        for (int m = 0; m < MAX_NUM_ENERGIES; m++)
        {
            if (logEnProt[m] > logInputEnergy)
            {
                int k;
                if (m == 0)
                {
                    k = m+1;
                }
                else
                {
                    k = m;
                }
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
        
        int localTimeFloor = floor(this->localTime);
        int localTimeCeil = ceil(this->localTime);
        
        // Initialize flux variables
        double flux11 = 0.0;
        double flux12 = 0.0;
        double flux13 = 0.0;
        double flux14 = 0.0;
        
        // ELECTRON: Gather four nearest *MEAN* flux values
        flux11 = this->mean_e_flux[this->kpIndex][eLowerIndex][localTimeFloor];
        flux12 = this->mean_e_flux[this->kpIndex][eHigherIndex][localTimeFloor];
        flux13 = this->mean_e_flux[this->kpIndex][eLowerIndex][localTimeCeil];
        flux14 = this->mean_e_flux[this->kpIndex][eHigherIndex][localTimeCeil];
        
        // ELECTRON: Find flux
        finalElec = bilinear(localTimeFloor, localTimeCeil, logEnElec[eLowerIndex], logEnElec[eHigherIndex], logInputEnergy, flux11, flux12, flux13, flux14);
        
        // ION: Gather four nearest *MEAN* flux values
        flux11 = this->mean_i_flux[this->kpIndex][iLowerIndex][localTimeFloor];
        flux12 = this->mean_i_flux[this->kpIndex][iHigherIndex][localTimeFloor];
        flux13 = this->mean_i_flux[this->kpIndex][iLowerIndex][localTimeCeil];
        flux14 = this->mean_i_flux[this->kpIndex][iHigherIndex][localTimeCeil];
        
        // ION: Find flux
        finalIon = bilinear(localTimeFloor, localTimeCeil, logEnProt[iLowerIndex], logEnProt[iHigherIndex], logInputEnergy, flux11, flux12, flux13, flux14);
        
        // Store the output message
        fluxOutMsgBuffer.meanElectronFlux[i] = finalElec;
        fluxOutMsgBuffer.meanIonFlux[i] = finalIon;
        fluxOutMsgBuffer.energies[i] = inputEnergies[i];
    }
    
    // Write to the output message
    this->fluxOutMsg.write(&fluxOutMsgBuffer, this->moduleID, CurrentSimNanos);
}

/*! method to calculate the local time of the spacecraft within the GEO belt
    @param r_SE_N sun position vector relative to the Earth
    @param r_BE_N spacecraft position vector relative to the Earth
    @return void
*/
void DentonFluxModel::calcLocalTime(double r_SE_N[3], double r_BE_N[3])
{
    // r_SE_N and r_BE_N are projected onto the equatorial plane to compute angle, thus only x and y components are used (z component is perpendicular to equator)
    double r_BE_N_hat[2];       /* unit vector from Earth to spacecraft */
    double r_SE_N_hat[2];       /* unit vector from Earth to Sun */
    v2Normalize(r_BE_N, r_BE_N_hat);
    v2Normalize(r_SE_N, r_SE_N_hat);
    
    // Determine Local Time: Using atan2()
    double x = v2Dot(r_BE_N_hat, r_SE_N_hat);
    double y = r_BE_N_hat[0]*r_SE_N_hat[1] - r_BE_N_hat[1]*r_SE_N_hat[0];
    double theta = atan2(y,x);

    if (x <= -1.0)
    {
        this->localTime = 0.0;    //!<  Data files are from 0-23 LT, this results in 24h being 0h
    }
    else
    {
        this->localTime = 12.00 + (theta / (2.*M_PI))*24;
    }
    
    return;
    
}

/*! Bilinear interpolation method
    @return void
*/
double DentonFluxModel::bilinear(int x1, int x2, double y1, double y2, double y, double f11, double f12, double f13, double f14)
{
    // Define variables
    double R1, R2, bilinear = 0.0;
    double x = this->localTime;

    if (x1 != x2)
    {
        R1 = ( (x2 - x) / (x2 - x1) ) * f11 + ( (x - x1) / (x2 - x1) ) * f13;
        R2 = ( (x2 - x) / (x2 - x1) ) * f12 + ( (x - x1) / (x2 - x1) ) * f14;
        bilinear = ( (y2 - y ) / (y2 - y1) ) * R1 + ( (y - y1) / (y2 - y1) ) * R2;
    }
    else
    {
        bilinear = ( (y2 - y ) / (y2 - y1) ) * f11 + ( (y - y1) / (y2 - y1) ) * f13;
    }
    return bilinear;

}


/*! Read in the Denton data file
    @param fileName data file name
    @param outArray data array pointer
    @return void
*/
void DentonFluxModel::readDentonDataFile(std::string fileName, double data[MAX_NUM_KPS][MAX_NUM_ENERGIES][MAX_NUM_LOCAL_TIMES])
{
    double temp = 0.0;
    
    // Input file stream object
    std::ifstream inputFile;
    
    // Read data from file:
    inputFile.open(this->dataPath + fileName);
    
    // Read information into array: Data includes information about mean, standard deviation, median and percentiles (7 types of values in total). Only mean is relevant for this module
    if (inputFile.is_open()) {
        for (int i = 0; i < MAX_NUM_KPS*MAX_NUM_VALUE_TYPES; i++)
        {   for (int j = 0; j < MAX_NUM_ENERGIES; j++)
            {   for (int k = 0; k < MAX_NUM_LOCAL_TIMES; k++)
                {
                    // MEAN corresponds to every 7th index
                    if (i%MAX_NUM_VALUE_TYPES == 0)
                    {
                        inputFile >> data[i/MAX_NUM_VALUE_TYPES][j][k];
                    } else {
                        inputFile >> temp;
                    }
                    
                }
            }
        }
    } else {
        bskLogger.bskLog(BSK_ERROR, ("Could not open " + this->dataPath + fileName).c_str());
    }
    
    // Close file
    inputFile.close();
    
    return;
}

