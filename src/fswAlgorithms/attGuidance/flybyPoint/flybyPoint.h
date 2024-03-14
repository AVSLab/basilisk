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

#include <Eigen/Dense>

/*! @brief A class to perform flyby pointing */
class FlybyPoint: public SysModel {
public:
    FlybyPoint();
    ~FlybyPoint();
    void SelfInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

    double getTimeBetweenFilterData() const;
    void setTimeBetweenFilterData(double timeBetweenFilterData);
    double getToleranceForCollinearity() const;
    void setToleranceForCollinearity(double toleranceForCollinearity);
    int64_t getSignOfOrbitNormalFrameVector() const;
    void setSignOfOrbitNormalFrameVector(int64_t signOfOrbitNormalFrameVector);

    ReadFunctor<NavTransMsgPayload>  filterInMsg;               //!< input msg relative position w.r.t. asteroid
    ReadFunctor<EphemerisMsgPayload> asteroidEphemerisInMsg;    //!< input asteroid ephemeris msg
    Message<AttRefMsgPayload> attRefOutMsg;                     //!< Attitude reference output message
    AttRefMsg_C attRefOutMsgC = {};                             //!< C-wrapped attitude reference output message

private:
    double     timeBetweenFilterData = 0;       //!< time between two subsequent reads of the filter information
    double     toleranceForCollinearity = 0;            //!< tolerance for singular conditions when position and velocity are collinear
    int64_t    signOfOrbitNormalFrameVector = 1;  //!< Sign of orbit normal vector to complete reference frame
    FlybyModel chosenFlybyModel = rectilinear;              //!< enum to indicate which flyby model is being used

    bool            firstRead;           //!< variable to attest if this is the first read after a Reset
    double          f0;                  //!< ratio between relative velocity and position norms at time of read [Hz]
    double          gamma0;              //!< flight path angle of the spacecraft at time of read [rad]
    Eigen::Matrix3d R0N;           //!< inertial-to-reference DCM at time of read
    Eigen::Vector3d r_BN_N;           //!< filter spacecraft position estimate in inertial coordinates
    Eigen::Vector3d v_BN_N;           //!< filter spacecraft velocity estimate in inertial coordinates
    SingularityFlag singularityFlag;     //!< +1 or -1 during singular configurations, 0 otherwise
    uint64_t        lastFilterReadTime;  //!< time of last filter read


    BSKLogger bskLogger;               //!< BSK Logging
};


#endif
