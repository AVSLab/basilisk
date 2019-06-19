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


#ifndef MAGNETIC_FIELD_BASE_H
#define MAGNETIC_FIELD_BASE_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/magneticFieldSimMsg.h"

/*! \addtogroup SimModelGroup
 * @{
 */



//! @brief General magnetic field base class used to calculate the magnetic field above a planet using multiple models.
/*! The MagneticField class is used to calculate the magnetic field vector above a body using multiple models.
 This base class is used to hold relevant planetary magnetic field properties to compute answers for a given set of spacecraft locations
 relative to a specified planet.  Specific magnetic field models are implemented as classes that inherit from this base class. Planetary parameters, including position and input message, are settable by the user. In a given simulation, each planet of interest should have only one magnetic field  model associated with it linked to the spacecraft in orbit about that body.*/
class MagneticFieldBase: public SysModel  {
public:
    MagneticFieldBase();
    ~MagneticFieldBase();
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    void setEpoch(double julianDate);
    void addSpacecraftToModel(std::string tmpScMsgName); 
    void UpdateState(uint64_t CurrentSimNanos); 

protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    void updateLocalMagField(double currentTime);
    void updateRelativePos(SpicePlanetStateSimMsg  *planetState, SCPlusStatesSimMsg *scState); 
    virtual void evaluateMagneticFieldModel(MagneticFieldSimMsg *msg, double currentTime) = 0;
    virtual void customSelfInit();
    virtual void customCrossInit();
    virtual void customReset(uint64_t CurrentClock);
    virtual void customWriteMessages(uint64_t CurrentClock);
    virtual bool customReadMessages();

public:
    std::vector<std::string> scStateInMsgNames;    //!< Vector of the spacecraft position/velocity message names
    std::vector<std::string> envOutMsgNames; //!< Vector of message names to be written out by the environment
    std::string planetPosInMsgName;          //!< Message name for the planet's SPICE position message
    double envMinReach; //!< [m] Minimum planet-relative position needed for the environment to work, default is off (neg. value)
    double envMaxReach; //!< [m] Maximum distance at which the environment will be calculated, default is off (neg. value)
    double epochDate;                       //!< [JD2000] Specified epoch date, default is a Julian date format
    double planetRadius;                    //!< [m]      Radius of the planet

protected:
    Eigen::Vector3d r_BP_N;                 //!< [m] sc position vector relative to planet in inertial N frame components
    Eigen::Vector3d r_BP_P;                 //!< [m] sc position vector relative to planet in planet-fixed frame components
    double orbitRadius;                     //!< [m] sc orbit radius about planet
    uint64_t OutputBufferCount;                //!< number of output buffers for messaging system
    std::vector<MagneticFieldSimMsg> magFieldOutBuffer; //!< -- Message buffer for magnetic field messages
    std::vector<int64_t>  envOutMsgIds;     //!< vector of module output message IDs
    std::vector<int64_t> scStateInMsgIds;   //!< vector of spacecraft state message IDs
    int64_t planetPosInMsgId;               //!< ID of the planet state message
    std::vector<SCPlusStatesSimMsg> scStates;//!< vector of the spacecraft state messages
    SpicePlanetStateSimMsg planetState;     //!< planet state message

};

/*! @} */

#endif /* MAGNETIC_FIELD_BASE_H */
