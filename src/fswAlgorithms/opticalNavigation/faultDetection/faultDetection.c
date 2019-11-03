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

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "faultDetection.h"



/*! This method transforms pixel, line, and diameter data into heading data for orbit determination or heading determination.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void SelfInit_faultDetection(FaultDetectionData *configData, int64_t moduleID)
{
    configData->stateOutMsgID = CreateNewMessage(configData->opNavOutMsgName,
                                                 sizeof(OpNavFswMsg),
                                                 "OpNavFswMsg",
                                                 moduleID);
}

/*! This method subscribes to the camera and circle messages
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void CrossInit_faultDetection(FaultDetectionData *configData, int64_t moduleID)
{
    /*! Read two messages that need to be compared */
    configData->navMeas1MsgID = subscribeToMessage(configData->navMeasPrimaryMsgName,sizeof(OpNavFswMsg),moduleID);
    configData->navMeas2MsgID = subscribeToMessage(configData->navMeasSecondaryMsgName, sizeof(OpNavFswMsg),moduleID);

    /*! Read camera specs and attitude for frame modifications */
    configData->cameraMsgID = subscribeToMessage(configData->cameraConfigMsgName, sizeof(OpNavFswMsg),moduleID);
    configData->attInMsgID = subscribeToMessage(configData->attInMsgName, sizeof(OpNavFswMsg),moduleID);

}

/*! This resets the module to original states.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Reset_faultDetection(FaultDetectionData *configData, uint64_t callTime, int64_t moduleID)
{

}

/*! This method reads in the camera and circle messages and extracts navigation data from them. It outputs the heading (norm and direction) to the celestial body identified in the inertial frame. It provides the heading to the most robust circle identified by the image processing algorithm.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Update_faultDetection(FaultDetectionData *configData, uint64_t callTime, int64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    OpNavFswMsg opNavIn1;
    OpNavFswMsg opNavIn2;
    OpNavFswMsg opNavMsgOut;
    CameraConfigMsg cameraSpecs;
    NavAttIntMsg attInfo;
    double dcm_NC[3][3], dcm_CB[3][3], dcm_BN[3][3];

    memset(&cameraSpecs, 0x0, sizeof(CameraConfigMsg));
    memset(&attInfo, 0x0, sizeof(NavAttIntMsg));
    memset(&opNavIn1, 0x0, sizeof(OpNavFswMsg));
    memset(&opNavIn2, 0x0, sizeof(OpNavFswMsg));
    memset(&opNavMsgOut, 0x0, sizeof(OpNavFswMsg));

    /*! - read input opnav messages */
    ReadMessage(configData->navMeas1MsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(OpNavFswMsg), &opNavIn1, moduleID);
    ReadMessage(configData->navMeas2MsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(OpNavFswMsg), &opNavIn2, moduleID);
    /*! - read dcm messages */
    ReadMessage(configData->cameraMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CameraConfigMsg), &cameraSpecs, moduleID);
    ReadMessage(configData->attInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(NavAttIntMsg), &attInfo, moduleID);
    /*! - compute dcms */
    MRP2C(cameraSpecs.sigma_CB, dcm_CB);
    MRP2C(attInfo.sigma_BN, dcm_BN);
    m33MultM33(dcm_CB, dcm_BN, dcm_NC);
    m33Transpose(dcm_NC, dcm_NC);
    
    /*! Begin fault detection logic */
    /*! - If none of the message contain valid nav data, unvalidate the nav and populate a zero message */
    if (opNavIn1.valid == 0 && opNavIn2.valid == 0){
        opNavMsgOut.valid = 0;
        WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                     &opNavMsgOut, moduleID);
    }
    /*! - If only one of two are valid use that one */
    else if (opNavIn1.valid == 1 && opNavIn2.valid == 0){
        memcpy(&opNavMsgOut, &opNavIn1, sizeof(OpNavFswMsg));
        WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                     &opNavMsgOut, moduleID);
    }
    else if (opNavIn1.valid == 0 && opNavIn2.valid == 1){
        /*! - If secondary measurments are trusted use them as primary */
        if (configData->faultMode==1){
            memcpy(&opNavMsgOut, &opNavIn2, sizeof(OpNavFswMsg));
            WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                         &opNavMsgOut, moduleID);
        }
        /*! - If secondaries are not trusted, do not risk corrupting measurment */
        if (configData->faultMode==0){
            opNavMsgOut.valid = 0;
            WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                         &opNavMsgOut, moduleID);
        }
    }
    /*! - If they are both valid proceed to the fault detection */
    else if (opNavIn1.valid == 1 && opNavIn2.valid == 1){
        opNavMsgOut.valid = 1;
        double error = 0;
        /*! -- Dissimilar mode compares the two measurements: if the mean +/- 3-sigma covariances overlap use the nominal nav solution */
        double faultDirection[3];
        double faultNorm;
        
        /*! Get the direction and norm between the the two measurements in camera frame*/
        v3Subtract(opNavIn2.r_BN_C, opNavIn1.r_BN_C, faultDirection);
        faultNorm = v3Norm(faultDirection);
        v3Normalize(faultDirection, faultDirection);
        
        /*! If the difference between vectors is beyond the covariances, detect a fault and use secondary */
        if (faultNorm > configData->sigmaFault*(vNorm(opNavIn1.covar_C, 9) + vNorm(opNavIn2.covar_C, 9))){
            error = 1;
            memcpy(&opNavMsgOut, &opNavIn2, sizeof(OpNavFswMsg));
            opNavMsgOut.faultDetected = 1;
            WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                         &opNavMsgOut, moduleID);
        }
        /*! If the difference between vectors is low, use primary */
        else if (configData->faultMode==0){
            /*! Bring all the measurements and covariances into their respective frames */
            memcpy(&opNavMsgOut, &opNavIn1, sizeof(OpNavFswMsg));
            WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                         &opNavMsgOut, moduleID);
        }
        /*! -- Merge mode combines the two measurements and uncertainties if they are similar */
        else if (configData->faultMode==1){
            double P1invX1[3], P2invX2[3], X_C[3], P_C[3][3], P_B[3][3], P_N[3][3];
            double P1inv[3][3], P2inv[3][3];
            
            /*! The covariance merge is given by P = (P1^{-1} + P2^{-1})^{-1} */
            m33Inverse(RECAST3X3 opNavIn1.covar_C, P1inv);
            m33Inverse(RECAST3X3 opNavIn2.covar_C, P2inv);
            m33Add(P1inv, P2inv, P_C);
            m33Inverse(P_C, P_C);

            /*! The estimage merge is given by x = P (P1^{-1}x1 + P2^{-1}x2) */
            m33MultV3(P1inv, opNavIn1.r_BN_C, P1invX1);
            m33MultV3(P2inv, opNavIn2.r_BN_C, P2invX2);
            v3Add(P1invX1, P2invX2, X_C);
            m33MultV3(P_C, X_C, X_C);
            v3Copy(X_C, opNavMsgOut.r_BN_C);
            
            /*! Bring all the measurements and covariances into their respective frames */
            m33MultV3(dcm_NC, X_C, opNavMsgOut.r_BN_N);
            mCopy(P_C, 3, 3, opNavMsgOut.covar_C);
            m33tMultM33(dcm_CB, P_C, P_B);
            m33MultM33(dcm_NC, P_C, P_N);
            m33MultM33(P_B, dcm_CB, P_B);
            m33MultM33t(P_N, dcm_NC , P_N);
            mCopy(P_N, 3, 3, opNavMsgOut.covar_N);
            mCopy(P_B, 3, 3, opNavMsgOut.covar_B);
            m33tMultV3(dcm_CB, X_C, opNavMsgOut.r_BN_B);
            m33MultV3(dcm_NC, X_C, opNavMsgOut.r_BN_N);
            opNavMsgOut.timeTag = opNavIn1.timeTag;
        }
        WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                     &opNavMsgOut, moduleID);
    }

    
    return;

}
