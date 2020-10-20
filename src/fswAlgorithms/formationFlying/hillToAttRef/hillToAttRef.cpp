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
#include "hillToAttRef.h"
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include <iostream>
/*! The constructor for the HoughCircles module. It also sets some default values at its creation.  */
HillToAttRef::HillToAttRef()
{
    this->hillStateInMsgName="";
    this->attStateInMsgName="";
    this->attRefOutMsgName="";
    for(int ind=0; ind<3; ++ind){
        this->attRefOutMsg.sigma_RN[ind] =0;
        this->attRefOutMsg.omega_RN_N[ind] = 0;
        this->attRefOutMsg.domega_RN_N[ind] = 0;
    }
}

/*! Selfinit performs the first stage of initialization for this module.
 It's primary function is to create messages that will be written to.
 @return void
 */
void HillToAttRef::SelfInit()
{
    /*! - Create output message for module */
    this->attRefOutMsgId= SystemMessaging::GetInstance()->CreateNewMessage(this->attRefOutMsgName,sizeof(AttRefFswMsg),2,"AttRefFswMsg",this->moduleID);
    this->matrixIndex = 0;
    this->gainMatrixVecLen = this->gainMatrixVec.size();
}


/*! CrossInit performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 */
void HillToAttRef::CrossInit()
{
    /*! - Get the image data message ID*/
    this->hillStateInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->hillStateInMsgName,sizeof(HillRelStateFswMsg), moduleID);

    /*! - Get the image data message ID*/
    this->attStateInMsgId  = SystemMessaging::GetInstance()->subscribeToMessage(this->attStateInMsgName,sizeof(NavAttIntMsg), moduleID);
}

/*! This is the destructor */
HillToAttRef::~HillToAttRef()
{
    return;
}


/*! This method performs a complete reset of the module.  Local module variables that retain time varying states between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void HillToAttRef::Reset(uint64_t CurrentSimNanos)
{
    this->matrixIndex = 0;//    Start back at the initial gain matrix
    this->gainMatrixVecLen = this->gainMatrixVec.size(); // Update this in case gainMatrixVec changed size
    return;
}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding. 
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void HillToAttRef::UpdateState(uint64_t CurrentSimNanos)
{
    SingleMessageHeader localHeader;
    //  Read in the relative state and chief attitude messages
    SystemMessaging::GetInstance()->ReadMessage(this->hillStateInMsgId, &localHeader,
                                                sizeof(HillRelStateFswMsg),
                                                reinterpret_cast<uint8_t*>(&this->hillStateInMsg),
                                                this->moduleID);
    SystemMessaging::GetInstance()->ReadMessage(this->attStateInMsgId, &localHeader,
                                                sizeof(NavAttIntMsg),
                                                reinterpret_cast<uint8_t*>(&this->attStateInMsg),
                                                this->moduleID);

    //  Apply the gain matrix to get a relative attitude

    double relativeAtt[3];
    double hillState[6];
    double gainMat[3][6];
    std::vector<std::vector<double>> currentMat;

    for(int ind=0; ind<3; ind++){
        hillState[ind] = this->hillStateInMsg.r_DC_H[ind];
        hillState[ind+3] = this->hillStateInMsg.v_DC_H[ind];
    }

    if(this->matrixIndex >= this->gainMatrixVecLen){
        this->matrixIndex = this->gainMatrixVecLen-1;   //  Hold at the last value if we've overrun the vector
    }

    currentMat = this->gainMatrixVec[this->matrixIndex];
    std::vector<std::vector<double>>::const_iterator row;
    std::vector<double>::const_iterator col;

    int row_ind = 0;
    int col_ind = 0;
    for(row = currentMat.begin(); row != currentMat.end(); ++row, ++row_ind){
        col_ind = 0;
        for (col = row->begin(); col!= row->end(); ++col, ++col_ind){
            gainMat[row_ind][col_ind] = *col;
            }
    }

    mMultV(gainMat, 3, 6,
                   hillState,
                   relativeAtt);
    //  Combine the relative attitude with the chief inertial attitude to get the reference attitude
    addMRP(this->attStateInMsg.sigma_BN, relativeAtt, this->attRefOutMsg.sigma_RN);
    for(int ind=0; ind<3; ++ind){
        this->attRefOutMsg.omega_RN_N[ind] = 0;
        this->attRefOutMsg.domega_RN_N[ind] = 0;
    }

    //  Write the reference message
    SystemMessaging::GetInstance()->WriteMessage(this->attRefOutMsgId,
                                                 CurrentSimNanos, sizeof(AttRefFswMsg),
                                                 reinterpret_cast<uint8_t*>(&this->attRefOutMsg),
                                                 this->moduleID);

    this->matrixIndex += 1;
}

