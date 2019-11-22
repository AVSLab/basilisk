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


#ifndef MSIS_ATMOSPHERE_H
#define MSIS_ATMOSPHERE_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../../_GeneralModuleFiles/sys_model.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/atmoPropsSimMsg.h"
#include "../_GeneralModuleFiles/atmosphereBase.h"
#include "simMessages/swDataSimMsg.h"
#include "simMessages/epochSimMsg.h"
#include "utilities/bskPrint.h"

extern "C" {
  #include "nrlmsise-00.h"
}


class MsisAtmosphere: public AtmosphereBase {
public:
    MsisAtmosphere();
    ~MsisAtmosphere();

private:
    void customCrossInit();
    void customWriteMessages(uint64_t CurrentClock);
    bool customReadMessages();
    bool ReadInputs();
    void updateInputParams();
    void updateSwIndices();
    void evaluateAtmosphereModel(AtmoPropsSimMsg *msg, double currentTime);
    void customSetEpochFromVariable();

public:
    int epochDoy;                       //!< [day] Day-of-Year at epoch
    std::string epochInMsgName;
    BSKPrint bskPrint;                      //!< -- BSK Logging


private:
    Eigen::Vector3d currentLLA; //!< [-] Container for local Latitude, Longitude, Altitude geodetic position; units are rad and km respectively.
    std::vector<SwDataSimMsg> swDataList; //!< Vector of space weather messages
    std::vector<std::string> swDataInMsgNames; //!< Vector of space weather message names
    int64_t swDataInMsgIds[23];

    // NRLMSISE-00 Specific attributes
    nrlmsise_input msisInput; //!< Struct containing NRLMSISE-00 input values; see their doc for details.
    nrlmsise_output msisOutput; //!< Struct containing NRLMSISE-00 output values; see their doc for details.
    nrlmsise_flags msisFlags;
    ap_array aph;
    double ap;
    double f107;
    double f107A;


};


#endif /* EXPONENTIAL_ATMOSPHERE_H */
