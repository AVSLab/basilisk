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
#include <time.h>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/MagneticFieldMsgPayload.h"
#include "architecture/msgPayloadDefC/EpochMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief magnetic field base class */
class MagneticFieldBase: public SysModel  {
public:
    MagneticFieldBase();
    ~MagneticFieldBase();
    void Reset(uint64_t CurrentSimNanos);
    void addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg);
    void UpdateState(uint64_t CurrentSimNanos);

protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    void updateLocalMagField(double currentTime);
    void updateRelativePos(SpicePlanetStateMsgPayload  *planetState, SCStatesMsgPayload *scState);
    virtual void evaluateMagneticFieldModel(MagneticFieldMsgPayload *msg, double currentTime) = 0; //!< class method
    virtual void customReset(uint64_t CurrentClock);
    virtual void customWriteMessages(uint64_t CurrentClock);
    virtual bool customReadMessages();
    virtual void customSetEpochFromVariable();

public:
    std::vector<ReadFunctor<SCStatesMsgPayload>> scStateInMsgs; //!< Vector of the spacecraft position/velocity input message
    std::vector<Message<MagneticFieldMsgPayload>*> envOutMsgs;       //!< Vector of message names to be written out by the environment
    ReadFunctor<SpicePlanetStateMsgPayload> planetPosInMsg;         //!< Message name for the planet's SPICE position message
    ReadFunctor<EpochMsgPayload> epochInMsg;                        //!< (optional) epoch date/time input message

    double envMinReach; //!< [m] Minimum planet-relative position needed for the environment to work, default is off (neg. value)
    double envMaxReach; //!< [m] Maximum distance at which the environment will be calculated, default is off (neg. value)
    double planetRadius;                     //!< [m]      Radius of the planet
    BSKLogger bskLogger;                      //!< -- BSK Logging

protected:
    Eigen::Vector3d r_BP_N;                 //!< [m] sc position vector relative to planet in inertial N frame components
    Eigen::Vector3d r_BP_P;                 //!< [m] sc position vector relative to planet in planet-fixed frame components
    double orbitRadius;                     //!< [m] sc orbit radius about planet

    std::vector<MagneticFieldMsgPayload> magFieldOutBuffer; //!< -- Message buffer for magnetic field messages
    std::vector<SCStatesMsgPayload> scStates;//!< vector of the spacecraft state messages
    SpicePlanetStateMsgPayload planetState;     //!< planet state message
    struct tm epochDateTime;                //!< time/date structure containing the epoch information using a Gregorian calendar
};


#endif /* MAGNETIC_FIELD_BASE_H */
