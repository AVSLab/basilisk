/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "../../_GeneralModuleFiles/sys_model.h"
#include "../spice/spice_planet_state.h"
#include "../../dynamics/spacecraftPlus/spacecraftPlusMsg.h"
#include <Eigen/Dense>
#include <vector>



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
typedef struct {
    double neutralDensity;                               //!< kg/m^3 Local neutral particle density
    double localTemp;                           //!< K Local avg particle temperature
}AtmoOutputData;


//! @brief Thruster dynamics class used to provide thruster effects on body
/*! This class is used to hold and operate a set of thrusters that are located
 on the spacecraft.  It contains all of the configuration data for the thruster
 set, reads an array of on-time requests (double precision in seconds).  It is
 intended to be attached to the dynamics plant in the system using the
 DynEffector interface and as such, does not directly write the current force
 or torque into the messaging system.  The nominal interface to dynamics are the
 dynEffectorForce and dynEffectorTorque arrays that are provided by the DynEffector base class.
 There is technically double inheritance here, but both the DynEffector and
 SysModel classes are abstract base classes so there is no risk of diamond.*/
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
    void ComputeLocalAtmo(double currentTime);
    void ComputeRelativePos(SpicePlanetState& planetState, SCPlusOutputStateData& scState);
    void AddSpacecraftToModel(std::string tmpScMsgName);


public:
    AtmoOutputData tmpAtmo;
    double localAtmoDens; //!< [kg/m^3] Local neutral atmospheric density (computed)
    double localAtmoTemp; //!< [K] Local atmospheric temperature, SET TO BE CONSTANT
    double currentAlt; //!< [m] Current s/c altitude
    std::string planetName;
    std::vector<std::string> atmoDensOutMsgNames;
    std::vector<std::string> scStateInMsgNames;
    std::string planetPosInMsgName;
    std::vector<uint64_t> atmoDensOutMsgIds;
    std::vector<uint64_t> scStateInMsgIds;
    uint64_t planetPosInMsgId;
    std::vector<SCPlusOutputStateData> scStates;
    SpicePlanetState bodyState;
    Eigen::Vector3d relativePos; //!< [-] Container for local position

private:
    exponentialProperties atmosphereProps; //! < -- Struct containing exponential atmosphere properties
    double tmpPosMag;
    uint64_t OutputBufferCount;
    std::vector<AtmoOutputData> atmoOutBuffer; //!< -- Message buffer for thruster data

};

#endif /* EXPONENTIAL_ATMOSPHERE_H */
