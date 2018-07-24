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


#ifndef EXPONENTIAL_ATMOSPHERE_H
#define EXPONENTIAL_ATMOSPHERE_H

#include <Eigen/Dense>
#include <vector>
#include "../../_GeneralModuleFiles/sys_model.h"
#include "../../simMessages/spicePlanetStateSimMsg.h"
#include "../../simMessages/scPlusStatesSimMsg.h"
#include "../../simMessages/atmoPropsSimMsg.h"

/*! \addtogroup SimModelGroup
 * @{
 */

//! @brief Container for the properties of a simple exponential atmosphere model, such that different planets can be tested. */
typedef struct {
    double baseDensity;                 //!< kg/m^3 Density at sea level
    double scaleHeight;                    //!< m   Altitude where base density has decreased by factor of e
    double planetRadius;                //!< m Radius of the local atmospheric body; altitude is computed as |r| - planetRadius
}exponentialProperties;

/*! This structure is used in the messaging system to communicate what the
 state of the vehicle is currently.*/



//! @brief Exponential atmosphere class used to calculate temperature / density above a body.
/*! This class is used to hold relevant atmospheric properties and to compute the density for a given set of spacecraft 
relative to a specified planet. Planetary parameters, including position and input message, are settable by the user. 
Internal support is provided for Venus, Earth, and Mars. In a given simulation, each planet of interest should have only
one exponentialAtmosphere model associated with it linked to the spacecraft in orbit about that body.*/
class ExponentialAtmosphere: public SysModel {
public:
    ExponentialAtmosphere();
    ~ExponentialAtmosphere();
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    //void computeStateContribution(double integTime);
    void WriteOutputMessages(uint64_t CurrentClock);
    bool ReadInputs();
    void updateLocalAtmo(double currentTime);
    void updateRelativePos(SpicePlanetStateSimMsg& planetState, SCPlusStatesSimMsg& scState);
    void addSpacecraftToModel(std::string tmpScMsgName);
    void setPlanet(std::string newPlanetName);
private:
    void setBaseDensity(double newBaseDens);
    void setScaleHeight(double newScaleHeight);
    void setPlanetRadius(double newPlanetRadius);



public:
    atmoPropsSimMsg tmpAtmo;
    double localAtmoDens; //!< [kg/m^3] Local neutral atmospheric density (computed)
    double localAtmoTemp; //!< [K] Local atmospheric temperature, SET TO BE CONSTANT
    double currentAlt; //!< [m] Current s/c altitude
    std::string planetName;
    std::vector<std::string> atmoDensOutMsgNames; //!< Vector of strings containing atmospheric output message names
    std::vector<std::string> scStateInMsgNames;	//!< Vector of the spacecraft position/velocity message names
    std::string planetPosInMsgName;			//!< Message name for the planet's SPICE position message
    std::vector<int64_t> atmoDensOutMsgIds;
    std::vector<int64_t> scStateInMsgIds;
    int64_t planetPosInMsgId;
    std::vector<SCPlusStatesSimMsg> scStates;
    SpicePlanetStateSimMsg bodyState;
    Eigen::Vector3d relativePos; //!< [-] Container for local position
    exponentialProperties atmosphereProps; //! < -- Struct containing exponential atmosphere properties

private:
    double tmpPosMag;	//<! [m/s] Magnitude of the spacecraft's current position
    uint64_t OutputBufferCount;	//!< number of output buffers for messaging system
    std::vector<atmoPropsSimMsg> atmoOutBuffer; //!< -- Message buffer for density messages

};

#endif /* EXPONENTIAL_ATMOSPHERE_H */
