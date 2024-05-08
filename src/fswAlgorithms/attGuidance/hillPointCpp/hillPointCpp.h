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

#ifndef _HILL_POINT_CPP_H_
#define _HILL_POINT_CPP_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>

/* Required module input messages */
#include "cMsgCInterface/EphemerisMsg_C.h"
#include "cMsgCInterface/NavTransMsg_C.h"
#include "cMsgCInterface/AttRefMsg_C.h"

/*! @brief Hill Point attitude guidance class. */
class HillPointCpp: public SysModel {
public:

    HillPointCpp() = default;                                   //!< Constructor
    ~HillPointCpp() override = default;                         //!< Destructor

    void Reset(uint64_t CurrentSimNanos) override;              //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;        //!< Update member function

    AttRefMsg_C attRefOutMsg;               //!<        The name of the output message
    NavTransMsg_C transNavInMsg;            //!<        The name of the incoming attitude command
    EphemerisMsg_C celBodyInMsg;            //!<        The name of the celestial body message
//
    BSKLogger *bskLogger;                   //!<        BSK Logging

private:
    int planetMsgIsLinked;                  //!<        flag if the planet message is linked

    static void computeHillPointingReference(Eigen::Vector3d r_BN_N,
                                      Eigen::Vector3d v_BN_N,
                                      Eigen::Vector3d celBdyPositionVector,
                                      Eigen::Vector3d celBdyVelocityVector,
                                      AttRefMsgPayload *attRefOut);

};

#endif
