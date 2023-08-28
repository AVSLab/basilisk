/*
 ISC License
 
 Copyright (c) 2023, Laboratory  for Atmospheric and Space Physics, University of Colorado at Boulder
 
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

#include "thrustCMEstimation.h"
#include <cmath>

ThrustCMEstimation::ThrustCMEstimation() = default;

ThrustCMEstimation::~ThrustCMEstimation() = default;

/*! Initialize C-wrapped output messages */
void ThrustCMEstimation::SelfInit(){
    VehicleConfigMsg_C_init(&this->vehConfigOutMsgC);
}

/*! Reset the flyby OD filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void ThrustCMEstimation::Reset(uint64_t CurrentSimNanos)
{
    /*! - Check if the required message has not been connected */
    if (!this->thrusterConfigBInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,  " thrusterConfigInMsg wasn't connected.");
    }
    if (!this->intFeedbackTorqueInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,  " intFeedbackTorqueInMsg wasn't connected.");
    }
    if (!this->attGuidInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,  " attGuidInMsg wasn't connected.");
    }
    if (this->vehConfigInMsg.isLinked()) {
        this->cmKnowledge = true;
    }
    else {
        this->cmKnowledge = false;
    }

    /*! Initialize estimated state and covariances based on user inputs */
    this->P.setZero();
    this->R.setZero();
    this->I.setZero();
    for (int i=0; i<3; ++i) {
        this->P(i,i) = this->P0[i];
        this->R(i,i) = this->R0[i];
        this->I(i,i) = 1;
    }
    this->r_CB_est = this->r_CB_B;
}

/*! Take the relative position measurements and outputs an estimate of the
 spacecraft states in the inertial frame.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void ThrustCMEstimation::UpdateState(uint64_t CurrentSimNanos)
{
    
}
