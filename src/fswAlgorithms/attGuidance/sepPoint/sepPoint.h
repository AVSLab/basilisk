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

#ifndef _SEP_POINT_
#define _SEP_POINT_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/BodyHeadingMsgPayload.h"
#include "architecture/msgPayloadDefC/InertialHeadingMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "cMsgCInterface/AttRefMsg_C.h"
#include "fswAlgorithms/attGuidance/_GeneralModuleFiles/constrainedAxisPointingLibrary.h"


/*! @brief A class to perform EMA SEP pointing */
class SepPoint: public SysModel {
public:
    SepPoint();
    ~SepPoint();
    void SelfInit();                                               //!< Self initialization for C-wrapped messages
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

    AlignmentPriority alignmentPriority;                           //!< flag to indicate which flyby model is being used
    double            a1Hat_B[3];                                  //!< solar array drive axis in B-frame coordinates
    double            a2Hat_B[3];                                  //!< Sun-constrained axis in B-frame coordinates
    double            beta;                                        //!< Sun-constraint half-cone angle

    ReadFunctor<NavAttMsgPayload>          attNavInMsg;            //!< input msg measured attitude
    ReadFunctor<BodyHeadingMsgPayload>     bodyHeadingInMsg;       //!< input body heading msg
    ReadFunctor<InertialHeadingMsgPayload> inertialHeadingInMsg;   //!< input inertial heading msg
    Message<AttRefMsgPayload>              attRefOutMsg;           //!< Attitude reference output message
    AttRefMsg_C                            attRefOutMsgC = {};     //!< C-wrapped attitude reference output message

private:
    int               callCount;                                   //!< count variable used in the finite difference logic
    uint64_t          T1NanoSeconds;                               //!< callTime one update step prior
    uint64_t          T2NanoSeconds;                               //!< callTime two update steps prior
    double            sigma_RN_1[3];                               //!< reference attitude one update step prior
    double            sigma_RN_2[3];                               //!< reference attitude two update steps prior
    BSKLogger                              bskLogger;              //!< BSK Logging
};


#endif
