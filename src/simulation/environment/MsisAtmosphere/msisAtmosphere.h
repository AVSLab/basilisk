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


#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../../_GeneralModuleFiles/sys_model.h"
#include "../../simMessages/spicePlanetStateSimMsg.h"
#include "../../simMessages/scPlusStatesSimMsg.h"
#include "../../simMessages/atmoPropsSimMsg.h"
#include "../../simMessages/swDataSimMsg.h"
#include "../_GeneralModuleFiles/planetEnvironmentModel.h"

extern "C" {
  #include "nrlmsise-00.h"
}

/*! \addtogroup SimModelGroup
 * @{
 */

//! @brief Container for the properties of a simple exponential atmosphere model, such that different planets can be tested. */
typedef struct {
    double baseDensity;                 //!< [kg/m^3] Density at sea level
    double scaleHeight;                 //!< [m]      Altitude where base density has decreased by factor of e
}exponentialProperties;

#define MODEL_EXPONENTIAL "exponential"
#define MODEL_MSISE "nrlmsise_00"

//! @brief Atmosphere class used to calculate temperature / density above a body using multiple models.
/*! This class is used to hold relevant atmospheric properties and to compute the density for a given set of spacecraft
relative to a specified planet. Planetary parameters, including position and input message, are settable by the user.
Internal support is provided for Venus, Earth, and Mars. In a given simulation, each planet of interest should have only
one Atmosphere model associated with it linked to the spacecraft in orbit about that body.  For more information see the [PDF Description](Basilisk-atmosphere-20190221.pdf).*/
class Atmosphere: public SysModel, public PlanetEnvironmentModel {
public:
    Atmosphere();
    ~Atmosphere();
    void SelfInit();
    void CrossInit();
    void setEnvType(std::string inputType);
    void setEpoch(double julianDate);
    void addSpacecraftToModel(std::string tmpScMsgName);
    void UpdateState(uint64_t CurrentSimNanos);
    void updateLocalAtmo(double currentTime);

private:
    void WriteOutputMessages(uint64_t CurrentClock);
    bool ReadInputs();
    void updateRelativePos(SpicePlanetStateSimMsg  *planetState, SCPlusStatesSimMsg *scState);
    void runExponentialModel(double tmpAltitude, AtmoPropsSimMsg *msg);
    void updateInputParams();
    void updateSwIndices();
    void runNrlmsise00Model(double currentTime, AtmoPropsSimMsg *msg);

public:
    double localAtmoTemp;                   //!< [K] Local atmospheric temperature, SET TO BE CONSTANT
    double epochDate;                       //!< [JD2000] Specified epoch date.
    // Message ID attributes
    std::vector<std::string> swDataInMsgNames;
    std::vector<int64_t>  envOutMsgIds;     //!< vector of module output messages
    std::vector<int64_t> scStateInMsgIds;   //!< vector of spacecraft state message IDs
    int64_t swDataInMsgIds[23];
    int64_t planetPosInMsgId;               //!< ID of the planet state message
    // Message struct attributes
    std::vector<SCPlusStatesSimMsg> scStates;//!< vector of the spacecraft state messages
    std::vector<SwDataSimMsg> swDataList; //!< Vector of space weather messages
    SpicePlanetStateSimMsg planetState;     //!< planet state message

    exponentialProperties exponentialParams;//! < -- Struct containing exponential atmosphere properties
    double planetRadius;                    //!< [m]      Radius of the local atmospheric body; altitude is computed as |r| - planetRadius

    // NRLMSISE-00 Specific attributes
    nrlmsise_input msisInput; //!< Struct containing NRLMSISE-00 input values; see their doc for details.
    nrlmsise_output msisOutput; //!< Struct containing NRLMSISE-00 output values; see their doc for details.
    nrlmsise_flags msisFlags;
    ap_array aph;
    double startTime; //!< [s] Time of simulation start in seconds
    double ap;
    double f107;
    double f107A;
    double sec;
    double alt;
    double year;
    double g_lat;
    double g_long;
    double lst;
    uint64_t doy;
    uint64_t startDoy; //!< [day] Day-of-Year at simulation start


    std::string expString;
    std::string msisString;

private:
    Eigen::Vector3d relativePos_N;          //!< [-] Container for local position vector in inertial frame
    uint64_t OutputBufferCount;	            //!< number of output buffers for messaging system
    std::vector<AtmoPropsSimMsg> atmoOutBuffer; //!< -- Message buffer for density messages
    std::vector<SwDataSimMsg> swDataBuffer;
    double currentAlt; //!< [m] Current s/c altitude

    Eigen::Vector3d relativePos; //!< [-] Container for local position
    Eigen::Vector3d currentLLA; //!< [-] Container for local Latitude, Longitude, Altitude geodetic position; units are rad and km respectively.


};

/*! @} */

#endif /* EXPONENTIAL_ATMOSPHERE_H */
