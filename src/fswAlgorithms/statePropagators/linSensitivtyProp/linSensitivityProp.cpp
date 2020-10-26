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

/* modify the path to reflect the new module names */
#include "linSensitivityProp.h"
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include <iostream>
/*! The constructor for the HoughCircles module. It also sets some default values at its creation.  */
LinSensProp::LinSensProp()
{
    this->hillStateInMsgName="";
    this->depAttNavMsgName="";
    this->chiefAttNavMsgName="";
    this->sensOutMsgName="";

    for(int ind=0; ind<3; ++ind){
        this->sensOutMsg.r_DC_H =0;
        this->sensOutMsg.v_DC_H = 0;
    }
}

/*! Selfinit performs the first stage of initialization for this module.
 It's primary function is to create messages that will be written to.
 @return void
 */
void LinSensProp::SelfInit()
{
    /*! - Create output message for module */
    this->sensOutMsgId= SystemMessaging::GetInstance()->CreateNewMessage(this->sensOutMsgName,sizeof(HillStateFswMsg),2,"HillStateFswMsg",this->moduleID);
}


/*! CrossInit performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 */
void LinSensProp::CrossInit()
{
    /*! - Get the image data message ID*/
    this->hillStateInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->hillStateInMsgName,sizeof(HillRelStateFswMsg), moduleID);
    this->depAttInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->depAttInMsgName,sizeof(NavAttIntMsg), moduleID);
    this->chiefAttInMsgId  = SystemMessaging::GetInstance()->subscribeToMessage(this->chiefAttInMsgName,sizeof(NavAttIntMsg), moduleID);
}

/*! This is the destructor */
LinSensProp::~LinSensProp()
{
    return;
}


/*! This method performs a complete reset of the module.  Local module variables that retain time varying states between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void LinSensProp::Reset(uint64_t CurrentSimNanos)
{
    //  Set the sensitivities back to zero:
    for(int ind=0; ind<3; ++ind){
        this->sensOutMsg.r_DC_H =0;
        this->sensOutMsg.v_DC_H = 0;
    }
}

void LinSensProp::ReadMessages(uint64_t CurrentSimNanos){
    SingleMessageHeader localHeader;
    //  Read in the relative state and chief attitude messages
    SystemMessaging::GetInstance()->ReadMessage(this->hillStateInMsgId, &localHeader,
                                                sizeof(HillRelStateFswMsg),
                                                reinterpret_cast<uint8_t*>(&this->hillStateInMsg),
                                                this->moduleID);
    SystemMessaging::GetInstance()->ReadMessage(this->chiefAttInMsgId, &localHeader,
                                                sizeof(NavAttIntMsg),
                                                reinterpret_cast<uint8_t*>(&this->chiefAttInMsg),
                                                this->moduleID);
    SystemMessaging::GetInstance()->ReadMessage(this->depAttInMsgId, &localHeader,
                                                sizeof(NavAttIntMsg),
                                                reinterpret_cast<uint8_t*>(&this->depAttInMsg),
                                                this->moduleID);
}

void LinSensProp::WriteMessages(uint64_t CurrentSimNanos){
    //  Write the reference message
    SystemMessaging::GetInstance()->WriteMessage(this->sensOutMsgId,
                                                 CurrentSimNanos, sizeof(HillStateFswMsg),
                                                 reinterpret_cast<uint8_t *>(&this->sensOutMsg),
                                                 this->moduleID);
}
    //  Combine the relative attitude with the chief inertial attitude to get the reference attitude


/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding. 
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void LinSensProp::UpdateState(uint64_t CurrentSimNanos) {

    this->ReadMessages(CurrentSimNanos);

    //  Set up relative att, hill inputs and compute them from messages
    double relativeAtt[3];

    Eigen::Vector6d hillState;
    //  Create a state vector based on the current Hill positions
    for(int ind=0; ind<3; ind++){
        hillState[ind] = this->hillStateInMsg.r_DC_H[ind];
        hillState[ind+3] = this->hillStateInMsg.v_DC_H[ind];
    }
    subMRP(this-depAttInMsg.sigma_BN, this->chiefStateInMsg.sigma_BN, relativeAtt);
    Eigen::Vector3d relativeAttVec = Eigen::Vector3d cArray2EigenVector3d(relativeAtt);

    // Compute the new sensitivity with an euler step
    auto sens_dot = this->A * sensitivityState + this->C * hillState + this->D * this->relativeAttVec;

    this->sensitivityState = this->sensitivityState + sens_dot * dt;

    //  Create a state vector based on the current Hill positions
    for(int ind=0; ind<3; ind++){
        this->sensOutMsg.r_DC_H[ind] = this->sensitivityState[ind]
        this->sensOutMsg.r_DC_H[ind+3] = this->sensitivityState[ind];
    }

    //  Convert that to an inertial attitude and write the attRef msg

    this->WriteMessages(CurrentSimNanos);

    this->matrixIndex += 1;
}

