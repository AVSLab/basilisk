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
#include "desenHillToAttRef.h"
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include <iostream>
/*! The constructor for the HoughCircles module. It also sets some default values at its creation.  */
DesenHillToAttRef::DesenHillToAttRef()
{
    this->hillStateInMsgName="";
    this->attStateInMsgName="";
    this->attRefOutMsgName="";
    for(int ind=0; ind<3; ++ind){
        this->attRefOutMsg.sigma_RN[ind] =0;
        this->attRefOutMsg.omega_RN_N[ind] = 0;
        this->attRefOutMsg.domega_RN_N[ind] = 0;
    }
    this->relMRPMax = 2;
    this->relMRPMin = -2;
}

/*! Selfinit performs the first stage of initialization for this module.
 It's primary function is to create messages that will be written to.
 @return void
 */
void DesenHillToAttRef::SelfInit()
{
    /*! - Create output message for module */
    this->attRefOutMsgId= SystemMessaging::GetInstance()->CreateNewMessage(this->attRefOutMsgName,sizeof(AttRefFswMsg),2,"AttRefFswMsg",this->moduleID);
    this->matrixIndex = 0;
    this->gainMatrixVecLen = this->stateGainVec.size();

    std::vector<std::vector<double>>::const_iterator row;
    std::vector<double>::const_iterator col;

    //  Copy B, R_inv into C arrays
    int row_ind = 0;
    int col_ind = 0;
    for(row = this->Rinv.begin(); row != this->Rinv.end(); ++row, ++row_ind){
        col_ind = 0;
        for (col = row->begin(); col!= row->end(); ++col, ++col_ind){
            this->Rinv_arr[row_ind][col_ind] = *col;
        }
    }

    row_ind = 0;
    col_ind = 0;
    for(row = this->B.begin(); row != this->B.end(); ++row, ++row_ind){
        col_ind = 0;
        for (col = row->begin(); col!= row->end(); ++col, ++col_ind){
            this->B_arr[row_ind][col_ind] = *col;
        }
    }
}


/*! CrossInit performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 */
void DesenHillToAttRef::CrossInit()
{
    /*! - Get the image data message ID*/
    this->hillStateInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->hillStateInMsgName,sizeof(HillRelStateFswMsg), moduleID);
    this->sensInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->sensInMsgName,sizeof(HillRelStateFswMsg), moduleID);

    /*! - Get the image data message ID*/
    this->attStateInMsgId  = SystemMessaging::GetInstance()->subscribeToMessage(this->attStateInMsgName,sizeof(NavAttIntMsg), moduleID);
}

/*! This is the destructor */
DesenHillToAttRef::~DesenHillToAttRef()
{
    return;
}


/*! This method performs a complete reset of the module.  Local module variables that retain time varying states between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void DesenHillToAttRef::Reset(uint64_t CurrentSimNanos)
{
    this->matrixIndex = 0;//    Start back at the initial gain matrix
    this->gainMatrixVecLen = this->stateGainVec.size(); // Update this in case gainMatrixVec changed size
    return;
}

void DesenHillToAttRef::ReadMessages(uint64_t CurrentSimNanos){
    SingleMessageHeader localHeader;
    //  Read in the relative state and chief attitude messages
    SystemMessaging::GetInstance()->ReadMessage(this->hillStateInMsgId, &localHeader,
                                                sizeof(HillRelStateFswMsg),
                                                reinterpret_cast<uint8_t*>(&this->hillStateInMsg),
                                                this->moduleID);
    SystemMessaging::GetInstance()->ReadMessage(this->sensInMsgId, &localHeader,
                                                sizeof(HillRelStateFswMsg),
                                                reinterpret_cast<uint8_t*>(&this->sensInMsg),
                                                this->moduleID);
    SystemMessaging::GetInstance()->ReadMessage(this->attStateInMsgId, &localHeader,
                                                sizeof(NavAttIntMsg),
                                                reinterpret_cast<uint8_t*>(&this->attStateInMsg),
                                                this->moduleID);
}

void DesenHillToAttRef::WriteMessages(uint64_t CurrentSimNanos){
    //  Write the reference message
    SystemMessaging::GetInstance()->WriteMessage(this->attRefOutMsgId,
                                                 CurrentSimNanos, sizeof(AttRefFswMsg),
                                                 reinterpret_cast<uint8_t *>(&this->attRefOutMsg),
                                                 this->moduleID);
}

void DesenHillToAttRef::RelativeToInertialMRP(double relativeAtt[3]){

    //  Check to see if the relative attitude components exceed specified bounds (by default these are non-physical and should never be reached)
    for(int ind=0; ind<3; ++ind){
        relativeAtt[ind] = std::max(relativeAtt[ind], this->relMRPMin);
        relativeAtt[ind] = std::min(relativeAtt[ind], this->relMRPMax);
    }

    //  Combine the relative attitude with the chief inertial attitude to get the reference attitude
    addMRP(this->attStateInMsg.sigma_BN, relativeAtt, this->attRefOutMsg.sigma_RN);
    for(int ind=0; ind<3; ++ind){
        this->attRefOutMsg.omega_RN_N[ind] = 0;
        this->attRefOutMsg.domega_RN_N[ind] = 0;
    }

}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding. 
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void DesenHillToAttRef::UpdateState(uint64_t CurrentSimNanos) {

    this->ReadMessages(CurrentSimNanos);

    double relativeAtt[3] = {0,0,0};
    double stateContrib[6] = {0,0,0,0,0,0};
    double sensContrib[6]= {0,0,0,0,0,0};
    double btContrib[3]= {0.0,0.0,0.0};

    double hillState[6] ={0,0,0,0,0,0};;
    double sensState[6]= {0,0,0,0,0,0};;
    double stateGainMat[6][6];
    double sensGainMat[6][6];
    std::vector<std::vector<double>> currentStateMat;
    std::vector<std::vector<double>> currentSensMat;

    //  Create a state vector based on the current Hill positions
    for(int ind=0; ind<3; ind++){
        hillState[ind] = this->hillStateInMsg.r_DC_H[ind];
        hillState[ind+3] = this->hillStateInMsg.v_DC_H[ind];
        sensState[ind] = this->sensInMsg.r_DC_H[ind];
        sensState[ind+3] = this->sensInMsg.v_DC_H[ind];
    }

    // Check overrun condition; if user supplied insufficient gain matrices, fix at last supplied matrix.
    if(this->matrixIndex >= this->gainMatrixVecLen){
        this->matrixIndex = this->gainMatrixVecLen-1;   //  Hold at the last value if we've overrun the vector
    }

    //  Get the current matrix (assume 1 per update) and convert it to a standard C matrix
    currentStateMat = this->stateGainVec[this->matrixIndex];
    currentSensMat = this->sensGainVec[this->matrixIndex];
    std::vector<std::vector<double>>::const_iterator row;
    std::vector<double>::const_iterator col;

    //  Copy the gain matrix elements into C arrays
    int row_ind = 0;
    int col_ind = 0;

    //  Print out the dimensions of currentmats
    for(row = currentStateMat.begin(); row != currentStateMat.end(); ++row, ++row_ind){
        col_ind = 0;
        for (col = row->begin(); col!= row->end(); ++col, ++col_ind){
            stateGainMat[row_ind][col_ind] = *col; //   WIP - this line segfaults? Might be fucking up python initialization.
            }
    }
    row_ind = 0;
    col_ind = 0;
    for(row = currentSensMat.begin(); row != currentSensMat.end(); ++row, ++row_ind){
        col_ind = 0;
        for (col = row->begin(); col!= row->end(); ++col, ++col_ind){
            sensGainMat[row_ind][col_ind] = *col;
        }
    }

    //  Apply the gainMat to the relative state to produce a chief-relative attitude
    //  Implement u = -Rinv * B_t * (state_gain*state_err + sens_gain*sens_err) 
    mMultV(stateGainMat, 6, 6,
            hillState,
            stateContrib);
    mMultV(sensGainMat, 6, 6,
           sensState,
           sensContrib);
    mAdd(stateContrib, 6, 1, sensContrib, btContrib);
    mtMultV(this->B_arr, 6, 3,
            btContrib,
            btContrib);
    mMultV(this->Rinv_arr, 3,3, btContrib, relativeAtt);

    vScale(-1.0, relativeAtt, 3, relativeAtt);
    //  Convert that to an inertial attitude and write the attRef msg
    this->RelativeToInertialMRP(relativeAtt);
    this->WriteMessages(CurrentSimNanos);

    this->matrixIndex += 1;
}

