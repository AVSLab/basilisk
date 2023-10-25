/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef FLYBY_POINT_H
#define FLYBY_POINT_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "cMsgCInterface/AttRefMsg_C.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"


typedef enum flybyModel{
    rectilinear = 0,
    cwEquations = 1
} FlybyModel;

typedef enum singularityFlag{
    minusInfinity = -1,
    nonSingular   =  0,
    plusInfinity  =  1
} SingularityFlag;


/*! @brief A class to perform flyby pointing */
class FlybyPoint: public SysModel {
public:
    FlybyPoint();
    ~FlybyPoint();
    void SelfInit();                   //!< Self initialization for C-wrapped messages
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    
    double     dtFilterData = 0;       //!< time between two subsequent reads of the filter information
    double     epsilon = 0;            //!< tolerance for singular conditions when position and velocity are collinear
    int64_t    signOfOrbitNormalFrameVector = 1;  //!< Sign of orbit normal vector to complete reference frame
    FlybyModel flybyModel;             //!< flag to indicate which flyby model is being used

    ReadFunctor<NavTransMsgPayload>  filterInMsg;               //!< input msg relative position w.r.t. asteroid
    ReadFunctor<EphemerisMsgPayload> asteroidEphemerisInMsg;    //!< input asteroid ephemeris msg
    Message<AttRefMsgPayload> attRefOutMsg;                     //!< Attitude reference output message
    AttRefMsg_C attRefOutMsgC = {};                             //!< C-wrapped attitude reference output message

private:
    bool            firstRead;           //!< variable to attest if this is the first read after a Reset
    double          f0;                  //!< ratio between relative velocity and position norms at time of read [Hz]
    double          gamma0;              //!< flight path angle of the spacecraft at time of read [rad]
    double          R0N[3][3];           //!< inertial-to-reference DCM at time of read
    SingularityFlag singularityFlag;     //!< +1 or -1 during singular configurations, 0 otherwise
    uint64_t        lastFilterReadTime;  //!< time of last filter read

    BSKLogger bskLogger;               //!< BSK Logging
};


#endif
