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
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <iostream>
/*! The constructor for the HoughCircles module. It also sets some default values at its creation.  */
HillToAttRef::HillToAttRef()
{
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
    // this->matrixIndex = 0;//    Start back at the initial gain matrix
    // this->gainMatrixVecLen = this->gainMatrixVec.size(); // Update this in case gainMatrixVec changed size
    return;
}

AttRefMsgPayload HillToAttRef::RelativeToInertialMRP(double relativeAtt[3], double sigma_XN[3]){
    
    AttRefMsgPayload attRefOut;
    //  Check to see if the relative attitude components exceed specified bounds (by default these are non-physical and should never be reached)
    for(int ind=0; ind<3; ++ind){
        relativeAtt[ind] = std::max(relativeAtt[ind], this->relMRPMin);
        relativeAtt[ind] = std::min(relativeAtt[ind], this->relMRPMax);
    }

    //  Combine the relative attitude with the chief inertial attitude to get the reference attitude
    addMRP(sigma_XN, relativeAtt, attRefOut.sigma_RN);
    for(int ind=0; ind<3; ++ind){
        attRefOut.omega_RN_N[ind] = 0;
        attRefOut.domega_RN_N[ind] = 0;
    }
    return(attRefOut);
}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding. 
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void HillToAttRef::UpdateState(uint64_t CurrentSimNanos) {

    HillRelStateMsgPayload hillStateInPayload;
    NavAttMsgPayload attStateInPayload;
    AttRefMsgPayload attRefInPayload;
    AttRefMsgPayload attRefOutPayload;
    
    double baseSigma[3];
    double relativeAtt[3];
    double hillState[6];
    double gainMat[3][6];
    std::vector<std::vector<double>> currentMat;

    // Do message reads
    hillStateInPayload = this->hillStateInMsg();
    if(this->attRefInMsg.isLinked()){
        attRefInPayload = this->attRefInMsg();
        v3Copy(attRefInPayload.sigma_RN, baseSigma);
    }
    else if(this->attStateInMsg.isLinked()){
        attStateInPayload = this->attStateInMsg();
        v3Copy(attStateInPayload.sigma_BN, baseSigma);
    }

    //  Create a state vector based on the current Hill positions
    for(int ind=0; ind<3; ind++){
        hillState[ind] = hillStateInPayload.r_DC_H[ind];
        hillState[ind+3] = hillStateInPayload.v_DC_H[ind];
    }

    //  Get the current matrix (assume 1 per update) and convert it to a standard C matrix
    //currentMat = this->gainMatrix;
    std::vector<std::vector<double>>::const_iterator row;
    std::vector<double>::const_iterator col;

    int row_ind = 0;
    int col_ind = 0;
    for(row = this->gainMatrix.begin(); row != this->gainMatrix.end(); ++row, ++row_ind){
        col_ind = 0;
        for (col = row->begin(); col!= row->end(); ++col, ++col_ind){
            gainMat[row_ind][col_ind] = *col;
            }
    }
    // std::cout<<"Current relative state: "<<hillState[0]<<" "<<hillState[1]<<" "<<hillState[2]<<" "<<hillState[3]<<" "<<hillState[4]<<" "<<hillState[5]<<std::endl;
    // std::cout<<"Printing current gain matrix:"<<std::endl;
    // std::cout<<gainMat[0][0]<<" "<<gainMat[0][1]<<" "<<gainMat[0][2]<<" "<<gainMat[0][3]<<" "<<gainMat[0][4]<<" "<<gainMat[0][5]<<std::endl;
    // std::cout<<gainMat[1][0]<<" "<<gainMat[1][1]<<" "<<gainMat[1][2]<<" "<<gainMat[1][3]<<" "<<gainMat[1][4]<<" "<<gainMat[1][5]<<std::endl;
    // std::cout<<gainMat[2][0]<<" "<<gainMat[2][1]<<" "<<gainMat[2][2]<<" "<<gainMat[2][3]<<" "<<gainMat[2][4]<<" "<<gainMat[2][5]<<std::endl;

    //  Apply the gainMat to the relative state to produce a chief-relative attitude
    mMultV(gainMat, 3, 6,
                   hillState,
                   relativeAtt);
                   
    // std::cout<<"Relative att components: "<<relativeAtt[0]<<" "<<relativeAtt[1]<<" "<<relativeAtt[2]<<std::endl;
    //  Convert that to an inertial attitude and write the attRef msg
    attRefOutPayload = this->RelativeToInertialMRP(relativeAtt, baseSigma);
    this->attRefOutMsg.write(&attRefOutPayload, this->moduleID, CurrentSimNanos);

    // this->matrixIndex += 1;
}

