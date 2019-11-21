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
#include "pixelLineConverter.h"



/*! This method transforms pixel, line, and diameter data into heading data for orbit determination or heading determination.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void SelfInit_pixelLineConverter(PixelLineConvertData *configData, int64_t moduleID)
{
    configData->bskPrint = _BSKPrint();
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
void CrossInit_pixelLineConverter(PixelLineConvertData *configData, int64_t moduleID)
{
    configData->cameraConfigMsgID = subscribeToMessage(configData->cameraConfigMsgName,sizeof(CameraConfigMsg),moduleID);
    configData->circlesInMsgID = subscribeToMessage(configData->circlesInMsgName, sizeof(CirclesOpNavMsg),moduleID);
    configData->attInMsgID = subscribeToMessage(configData->attInMsgName, sizeof(NavAttIntMsg),moduleID);
}

/*! This resets the module to original states.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Reset_pixelLineConverter(PixelLineConvertData *configData, uint64_t callTime, int64_t moduleID)
{

}

/*! This method reads in the camera and circle messages and extracts navigation data from them. It outputs the heading (norm and direction) to the celestial body identified in the inertial frame. It provides the heading to the most robust circle identified by the image processing algorithm.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Update_pixelLineConverter(PixelLineConvertData *configData, uint64_t callTime, int64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    double dcm_NC[3][3], dcm_CB[3][3], dcm_BN[3][3];
    double reCentered[2];
    CameraConfigMsg cameraSpecs;
    CirclesOpNavMsg circlesIn;
    OpNavFswMsg opNavMsgOut;
    NavAttIntMsg attInfo;
    memset(&cameraSpecs, 0x0, sizeof(CameraConfigMsg));
    memset(&attInfo, 0x0, sizeof(NavAttIntMsg));
    memset(&circlesIn, 0x0, sizeof(CirclesOpNavMsg));
    memset(&opNavMsgOut, 0x0, sizeof(OpNavFswMsg));

    /*! - read input messages */
    ReadMessage(configData->cameraConfigMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CameraConfigMsg), &cameraSpecs, moduleID);
    ReadMessage(configData->circlesInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CirclesOpNavMsg), &circlesIn, moduleID);
    ReadMessage(configData->attInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(NavAttIntMsg), &attInfo, moduleID);

    if (circlesIn.valid == 0){
        opNavMsgOut.valid = 0;
        WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                     &opNavMsgOut, moduleID);
        return;
    }
    reCentered[0] = circlesIn.circlesCenters[0] - cameraSpecs.resolution[0]/2 + 0.5;
    reCentered[1] = circlesIn.circlesCenters[1] - cameraSpecs.resolution[1]/2 + 0.5;
    configData->planetTarget = circlesIn.planetIds[0];
    MRP2C(cameraSpecs.sigma_CB, dcm_CB);
    MRP2C(attInfo.sigma_BN, dcm_BN);
    m33MultM33(dcm_CB, dcm_BN, dcm_NC);
    m33Transpose(dcm_NC, dcm_NC);

    /*! - Find pixel size using camera specs */
    double X, Y;
    X = cameraSpecs.sensorSize[0]/cameraSpecs.resolution[0];
    Y = cameraSpecs.sensorSize[1]/cameraSpecs.resolution[1];

    /*! - Get the heading */
    double rtilde_C[2];
    double rHat_BN_C[3], rHat_BN_N[3], rHat_BN_B[3];
    double rNorm = 1;
    double planetRad, denom;
    double covar_map_C[3*3], covar_In_C[3*3], covar_In_B[3*3];
    double covar_In_N[3*3];
    double x_map, y_map, rho_map;
    rtilde_C[0] = (X*reCentered[0])/cameraSpecs.focalLength;
    rtilde_C[1] = (Y*reCentered[1])/cameraSpecs.focalLength;
    v3Set(rtilde_C[0], rtilde_C[1], 1.0, rHat_BN_C);
    v3Scale(-1, rHat_BN_C, rHat_BN_C);
    v3Normalize(rHat_BN_C, rHat_BN_C);

    m33MultV3(dcm_NC, rHat_BN_C, rHat_BN_N);
    m33tMultV3(dcm_CB, rHat_BN_C, rHat_BN_B);

    if(configData->planetTarget > 0){
        if(configData->planetTarget ==1){
            planetRad = REQ_EARTH;//in km
            opNavMsgOut.planetID = configData->planetTarget;
        }
        if(configData->planetTarget ==2){
            planetRad = REQ_MARS;//in km
            opNavMsgOut.planetID = configData->planetTarget;
        }
        if(configData->planetTarget ==3){
            planetRad = REQ_JUPITER;//in km
            opNavMsgOut.planetID = configData->planetTarget;
        }

        denom = sin(atan(X*circlesIn.circlesRadii[0]/cameraSpecs.focalLength));
        rNorm = planetRad/denom; //in km

        /*! - Compute the uncertainty */
        x_map = planetRad/denom*(X/cameraSpecs.focalLength);
        y_map = planetRad/denom*(Y/cameraSpecs.focalLength);
        rho_map = planetRad*(X/(cameraSpecs.focalLength*sqrt(1 + pow(circlesIn.circlesRadii[0]*X/cameraSpecs.focalLength,2)))-cameraSpecs.focalLength*sqrt(1 + pow(circlesIn.circlesRadii[0]*X/cameraSpecs.focalLength,2))/(pow(circlesIn.circlesRadii[0], 2)*X));
        mSetIdentity(covar_map_C, 3, 3);
        covar_map_C[0] = x_map;
        covar_map_C[4] = y_map;
        covar_map_C[8] = rho_map;
        mCopy(circlesIn.uncertainty, 3, 3, covar_In_C);
        mMultM(covar_map_C, 3, 3, covar_In_C, 3, 3, covar_In_C);
        mMultMt(covar_In_C, 3, 3, covar_map_C, 3, 3, covar_In_C);
        /*! - Changer the mapped covariance to inertial frame */
        mMultM(dcm_NC, 3, 3, covar_In_C, 3, 3, covar_In_N);
        mMultMt(covar_In_N, 3, 3, dcm_NC, 3, 3, covar_In_N);
        /*! - Changer the mapped covariance to body frame */
        mtMultM(dcm_CB, 3, 3, covar_In_C, 3, 3, covar_In_B);
        mMultM(covar_In_B, 3, 3, dcm_CB, 3, 3, covar_In_B);
    }

    /*! - write output message */
    v3Scale(rNorm*1E3, rHat_BN_N, opNavMsgOut.r_BN_N); //in m
    v3Scale(rNorm*1E3, rHat_BN_C, opNavMsgOut.r_BN_C); //in m
    v3Scale(rNorm*1E3, rHat_BN_B, opNavMsgOut.r_BN_B); //in m
    mCopy(covar_In_N, 3, 3, opNavMsgOut.covar_N);
    vScale(1E6, opNavMsgOut.covar_N, 3*3, opNavMsgOut.covar_N);//in m
    mCopy(covar_In_C, 3, 3, opNavMsgOut.covar_C);
    vScale(1E6, opNavMsgOut.covar_C, 3*3, opNavMsgOut.covar_C);//in m
    mCopy(covar_In_B, 3, 3, opNavMsgOut.covar_B);
    vScale(1E6, opNavMsgOut.covar_B, 3*3, opNavMsgOut.covar_B);//in m
    opNavMsgOut.timeTag = circlesIn.timeTag;
    opNavMsgOut.valid =1;
    WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                 &opNavMsgOut, moduleID);

    return;
}
