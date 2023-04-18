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


#ifndef LAMBERTVALIDATOR_H
#define LAMBERTVALIDATOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertProblemMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertSolutionMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertPerformanceMsgPayload.h"
#include "architecture/msgPayloadDefC/DvBurnCmdMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"

#define NUM_INITIALSTATES 27 // number of initial states

/*! @brief This module validates if the Lambert velocity solution violates any constraints and, if not,
    creates the Delta-V burn command message
 */
class LambertValidator: public SysModel {
public:
    LambertValidator();
    ~LambertValidator();

    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;

    ReadFunctor<NavTransMsgPayload> navTransInMsg;                          //!< translational navigation input message
    ReadFunctor<LambertProblemMsgPayload> lambertProblemInMsg;              //!< lambert problem input message
    ReadFunctor<LambertSolutionMsgPayload> lambertSolutionInMsg;            //!< lambert solution input message
    ReadFunctor<LambertPerformanceMsgPayload> lambertPerformanceInMsg;      //!< lambert performance input message
    Message<DvBurnCmdMsgPayload> dvBurnCmdOutMsg;                           //!< Delta-V burn command message

    BSKLogger bskLogger;                                                    //!< -- BSK Logging

    double lambertSolutionSpecifier = 1; //!< [s] which Lambert solution (1 or 2), if applicable, should be used
    double finalTime{}; //!< [s] time at which target position should be reached
    double maneuverTime{}; //!< [s] time at which maneuver should be executed
    double maxDistanceTarget{}; //!< [m] maximum acceptable distance from target location at final time
    double minOrbitRadius{}; //!< [m] minimum acceptable orbit radius
    //!< 6x6 matrix square root of the covariance matrix to apply errors with, in Hill (Orbit) frame components
    Eigen::MatrixXd uncertaintyStates;
    double uncertaintyDV = 1; //!< [%] uncertainty of the Delta-V magnitude
    double dvConvergenceTolerance = 1e-3; //!< [m/s] tolerance on difference between DeltaV solutions between time steps
};

#endif
