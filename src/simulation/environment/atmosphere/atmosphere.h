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
#include "../_GeneralModuleFiles/planetEnvironmentModel.h"

/*! \addtogroup SimModelGroup
 * @{}
 */

//! @brief Container for the properties of a simple exponential atmosphere model, such that different planets can be tested. */
typedef struct {
    double baseDensity;                 //!< kg/m^3 Density at sea level
    double scaleHeight;                    //!< m   Altitude where base density has decreased by factor of e
    double planetRadius;                //!< m Radius of the local atmospheric body; altitude is computed as |r| - planetRadius
}exponentialProperties;




//! @brief Atmosphere class used to calculate temperature / density above a body using multiple models.
/*! This class is used to hold relevant atmospheric properties and to compute the density for a given set of spacecraft 
relative to a specified planet. Planetary parameters, including position and input message, are settable by the user. 
Internal support is provided for Venus, Earth, and Mars. In a given simulation, each planet of interest should have only
one Atmosphere model associated with it linked to the spacecraft in orbit about that body.*/
class Atmosphere: public SysModel, public PlanetEnvironmentModel {
public:
    Atmosphere();//! [-] Constructor
    ~Atmosphere();//! [-] Destructor
    void SelfInit(); //! [-] Initializes published messages
    void CrossInit(); //![-] Subscribes to desired methods
    void setEnvType(std::string inputType); //!< [string] Sets the model used to compute atmospheric density/temperature; must be set before init
    void setEpoch(double julianDate); //!< [JulianDate2000] Sets the epoch date used by some models. This is converted automatically to the desired units.
    void addSpacecraftToModel(std::string tmpScMsgName); //! [string] Adds the spacecraft message name to a vector of sc message names and automatically creates an output message name. Must be called after ``setEnvType''.
    void UpdateState(uint64_t CurrentSimNanos); //! [nanoseconds] Computes the current atmospheric parameters for each spacecraft and writes their respective messages.

private:
    void WriteOutputMessages(uint64_t CurrentClock); //! [nanoseconds] Iterates through outputMessageIDs and writes output messages.
    bool ReadInputs(); //! [-] Reads respective inputs required by a given atmospheric model
    void updateLocalAtmo(double currentTime); //! [nanoseconds] Computes the local atmospheric density and temperature.
    void updateRelativePos(SpicePlanetStateSimMsg& planetState, SCPlusStatesSimMsg& scState); //! [-] Computes the planet-relative position of a given spacecraft.

public:
    std::vector<std::string> scStateInMsgNames;	//!< Vector of the spacecraft position/velocity message names
    std::vector<std::string> envOutMsgNames; //!< Vector of message names to be written out by the environment
    std::string planetPosInMsgName;			//!< Message name for the planet's SPICE position message
    double envMinReach = 0; //!< [m] Minimum planet-relative position needed for the environment to work
    double envMaxReach = 1e12; //!< [m] Maximum distance at which the environment will be calculated
    AtmoPropsSimMsg tmpAtmo;
    double localAtmoDens; //!< [kg/m^3] Local neutral atmospheric density (computed)
    double localAtmoTemp; //!< [K] Local atmospheric temperature, SET TO BE CONSTANT
    double currentAlt; //!< [m] Current s/c altitude
    double epochDate; //!< [JD2000] Specified epoch date.
    std::vector<int64_t>  envOutMsgIds;
    std::vector<int64_t> scStateInMsgIds;
    int64_t planetPosInMsgId;
    std::vector<SCPlusStatesSimMsg> scStates;
    SpicePlanetStateSimMsg bodyState;
    Eigen::Vector3d relativePos; //!< [-] Container for local position
    exponentialProperties exponentialParams; //! < -- Struct containing exponential atmosphere properties

private:
    double tmpPosMag;	//<! [m/s] Magnitude of the spacecraft's current position
    uint64_t OutputBufferCount;	//!< number of output buffers for messaging system
    std::vector<AtmoPropsSimMsg> atmoOutBuffer; //!< -- Message buffer for density messages

};

#endif /* EXPONENTIAL_ATMOSPHERE_H */
