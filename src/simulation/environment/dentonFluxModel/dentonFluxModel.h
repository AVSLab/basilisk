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

#ifndef DENTONFLUXMODEL_H
#define DENTONFLUXMODEL_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/PlasmaFluxMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

#define MAX_NUM_KPS             28
#define MAX_NUM_ENERGIES        40
#define MAX_NUM_LOCAL_TIMES     24
#define MAX_NUM_VALUE_TYPES     7

/*! @brief This module provides the 10-year averaged GEO elecon and ion flux as discussed in the paper by Denton.
 */
class DentonFluxModel: public SysModel {
public:
    // Constructor And Destructor
    DentonFluxModel();
    ~DentonFluxModel();

    // Methods
    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;
    
    /* public variables */
    int numOutputEnergies = -1; //!< number of energy bins used in the output message
    std::string kpIndex = ""; //!< Kp index
    std::string dataPath = ""; //!< -- String with the path to the Denton GEO data
    std::string eDataFileName = "model_e_array_all.txt"; //!< file name of the electron data file
    std::string iDataFileName = "model_i_array_all.txt"; //!< file name of the ion data file

    ReadFunctor<SCStatesMsgPayload> scStateInMsg; //!<  spacecraft state input message
    ReadFunctor<SpicePlanetStateMsgPayload> earthStateInMsg; //!< Earth planet state input message
    ReadFunctor<SpicePlanetStateMsgPayload> sunStateInMsg; //!< sun state input message

    Message<PlasmaFluxMsgPayload> fluxOutMsg; //!< output message with ion and electron fluxes

    BSKLogger bskLogger; //!< -- BSK Logging

private:
    void calcLocalTime(double v1[3], double v2[3]);
    double bilinear(int, int, double, double, double, double, double, double, double);
    void readDentonDataFile(std::string fileName, double data[MAX_NUM_KPS][MAX_NUM_ENERGIES][MAX_NUM_LOCAL_TIMES]);

    int kpIndexCounter; //!< Kp index counter (betweeen 0 and 27)
    double localTime; //!< spacecraft location time relative to sun heading at GEO
    double logEnElec[MAX_NUM_ENERGIES]; //!< log of the electron energies
    double logEnProt[MAX_NUM_ENERGIES]; //!< log of the proton energies
    double inputEnergies[MAX_NUM_ENERGIES]; //!< input energies considered in this module

    //!< Electron Flux:
    double mean_e_flux[MAX_NUM_KPS][MAX_NUM_ENERGIES][MAX_NUM_LOCAL_TIMES];
    
    //!< Ion Flux:
    double mean_i_flux[MAX_NUM_KPS][MAX_NUM_ENERGIES][MAX_NUM_LOCAL_TIMES];
        
    //!< Fill average centre energies, normalized by satellite
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
};

#endif
