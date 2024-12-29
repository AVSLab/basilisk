/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "simulation/deviceInterface/hingedBodyLinearProfiler/hingedBodyLinearProfiler.h"
#include "architecture/utilities/macroDefinitions.h"
#include <iostream>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
HingedBodyLinearProfiler::HingedBodyLinearProfiler()
{
    this->startTheta = 0.0;
    this->endTheta = 0.0;
    this->startTime = 0;
    this->endTime = 0;

}

/*! Module Destructor */
HingedBodyLinearProfiler::~HingedBodyLinearProfiler()
{
}

/*! This method is used to reset the module and checks that required input messages are connected.

*/
void HingedBodyLinearProfiler::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if(this->endTime-this->startTime > 0){
        this->deploymentSlope = (this->endTheta-this->startTheta) / ((this->endTime-this->startTime) * NANO2SEC);
    } else{
        bskLogger.bskLog(BSK_ERROR, "Delta between end time and start time of deployment must exist and be positive.");
    }
}


/*! This is the main method that gets called every time the module is updated. Outputs a reference theta and theta dot based on the current simulation time
    relative to the start and stop times for the linear deployment.

*/
void HingedBodyLinearProfiler::UpdateState(uint64_t CurrentSimNanos)
{
    double refTheta;
    double refThetaDot;
    HingedRigidBodyMsgPayload hingedRigidBodyReferenceOutMsgBuffer;  //!< local copy of message buffer

    //!< always zero the output message buffers before assigning values
    hingedRigidBodyReferenceOutMsgBuffer = this->hingedRigidBodyReferenceOutMsg.zeroMsgPayload;

    if(CurrentSimNanos < this->startTime) { //!< if deployment has not started
        refTheta = this->startTheta;
        refThetaDot = 0.0;
    } else if (CurrentSimNanos <= this->endTime){ //!< if deployment is in progress
        refThetaDot = this->deploymentSlope;
        refTheta = this->startTheta + ((CurrentSimNanos-this->startTime) * NANO2SEC) * refThetaDot;

    } else { //!< if deployment is over
        refTheta = this->endTheta;
        refThetaDot = 0.0;
    }

    hingedRigidBodyReferenceOutMsgBuffer.theta = refTheta;
    hingedRigidBodyReferenceOutMsgBuffer.thetaDot = refThetaDot;


    //!<  write to the output messages
    this->hingedRigidBodyReferenceOutMsg.write(&hingedRigidBodyReferenceOutMsgBuffer, this->moduleID, CurrentSimNanos);
}
