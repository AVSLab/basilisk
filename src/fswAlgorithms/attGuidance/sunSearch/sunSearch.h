/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _SUN_SEARCH_
#define _SUN_SEARCH_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "cMsgCInterface/AttGuidMsg_C.h"

struct SlewProperties {
    // user-requested properties
    double    slewTime;             //!< [s] total time for the three-axes maneuver
    double    slewAngle;            //!< [rad] total angle sweep around one axis
    double    slewMaxRate;          //!< [rad/s] maximum spacecraft body rate norm
    int       slewRotAxis;          //!< [-] axes about which to perform the Sun search
    // computed properties
    double    slewAngAcc;           //!< [rad/s^2] angular accelerations about each rotation axis
    double    slewOmegaMax;         //!< [rad/s] highes angualr rate about each rotation axis
    double    slewThrustTime;       //!< [s] control time of each rotation
    double    slewTotalTime;        //!< [s] total slew time of each rotation
};

/*! @brief A class to perform EMA SEP pointing */
class SunSearch: public SysModel {
public:
    SunSearch();
    ~SunSearch();
    void SelfInit();                                             //!< Self initialization for C-wrapped messages
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

    void setSlewTime(double const t1, const double t2, const double t3);
    void setSlewAngle(double const theta1, double const theta2, double const theta3);
    void setMaxRate(double const omega1, double const omega2, double const omega3);
    void setMaxTorque(double const u1, double const u2, double const u3);
    void setRotAxis(int const a1, int const a2, int const a3);

    ReadFunctor<NavAttMsgPayload>          attNavInMsg;          //!< input msg measured attitude
    ReadFunctor<VehicleConfigMsgPayload>   vehConfigInMsg;       //!< input veh config msg
    Message<AttGuidMsgPayload>             attGuidOutMsg;        //!< Attitude reference output message
    AttGuidMsg_C                           attGuidOutMsgC = {};  //!< C-wrapped attitude guidance output message

private:
    SlewProperties slewProperties[3];
    double         slewMaxTorque[3];     //!< [Nm] maximum deliverable torque along each principal body axis
    double         principleInertias[3]; //!< [kg m^2] inertias about the three principal axes
    uint64_t       resetTime;            //!< time at which reset is called
    BSKLogger      bskLogger;            //!< BSK Logging

    void computeKinematicProperties(int const index);
};

#endif
