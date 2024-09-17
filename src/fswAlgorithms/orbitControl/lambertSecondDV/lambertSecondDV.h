/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef LAMBERTSECONDDV_H
#define LAMBERTSECONDDV_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/LambertSolutionMsgPayload.h"
#include "architecture/msgPayloadDefC/DesiredVelocityMsgPayload.h"
#include "architecture/msgPayloadDefC/DvBurnCmdMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"

/*! @brief This module computes the second Delta-V maneuver for the Lambert problem
 */
class LambertSecondDV: public SysModel {
public:
    LambertSecondDV();
    ~LambertSecondDV();

    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;

    ReadFunctor<LambertSolutionMsgPayload> lambertSolutionInMsg;            //!< lambert solution input message
    ReadFunctor<DesiredVelocityMsgPayload> desiredVelocityInMsg;            //!< desired velocity input message
    Message<DvBurnCmdMsgPayload> dvBurnCmdOutMsg;                           //!< Delta-V burn command message

    BSKLogger bskLogger;                                                    //!< BSK Logging

    /** setter for `lambertSolutionSpecifier` */
    void setLambertSolutionSpecifier(const double value);
    /** getter for `lambertSolutionSpecifier` */
    double getLambertSolutionSpecifier() const {return this->lambertSolutionSpecifier;}

private:
    void readMessages();
    void writeMessages(uint64_t currentSimNanos);

    double lambertSolutionSpecifier = 1; //!< [-] which Lambert solution (1 or 2), if applicable, should be used
    Eigen::Vector3d vExpected_N; //!< [m/s] Expected velocity in inertial frame N components
    bool validLambert = false; //!< [-] valid Lambert solution if true
    Eigen::Vector3d vDesired_N; //!< [m/s] Desired velocity in inertial frame N
    double maneuverTime{}; //!< [s] time at which maneuver should be executed
    Eigen::Vector3d dv_N; //!< [m/s] requested Delta-V in N frame components
};

#endif
