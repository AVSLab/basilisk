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
void SelfInit_pixelLineConverter(PixelLineConvertData *configData, uint64_t moduleID)
{
    configData->stateOutMsgID = CreateNewMessage(configData->opNavOutMsgName,
                                                 sizeof(OpnavFswMsg),
                                                 "OpnavFsw",
                                                 moduleID);
}

/*! This method subscribes to the camera and circle messages
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void CrossInit_pixelLineConverter(PixelLineConvertData *configData, uint64_t moduleID)
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
void Reset_pixelLineConverter(PixelLineConvertData *configData, uint64_t callTime, uint64_t moduleID)
{

}

/*! This method reads in the camera and circle messages and extracts navigation data from them. It outputs the heading (norm and direction) to the celestial body identified in the body frame. It provides the heading to the most robust circle identified by the image processing algorithm. 
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Update_pixelLineConverter(PixelLineConvertData *configData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    double dcm_NC[3][3];
    CameraConfigMsg cameraSpecs;
    CirclesOpNavMsg circlesIn;
    OpnavFswMsg opNavMsgOut;
    NavAttIntMsg attInfo;
    memset(&cameraSpecs, 0x0, sizeof(CameraConfigMsg));
    memset(&attInfo, 0x0, sizeof(NavAttIntMsg));
    memset(&circlesIn, 0x0, sizeof(CirclesOpNavMsg));
    memset(&opNavMsgOut, 0x0, sizeof(OpnavFswMsg));

    /*! - read input messages */
    ReadMessage(configData->cameraConfigMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CameraConfigMsg), &cameraSpecs, moduleID);
    ReadMessage(configData->circlesInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CirclesOpNavMsg), &circlesIn, moduleID);
    ReadMessage(configData->attInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(NavAttIntMsg), &attInfo, moduleID);
    
    v3Scale(-1, attInfo.sigma_BN, attInfo.sigma_BN); // sigma_NB now
    addMRP(attInfo.sigma_BN, cameraSpecs.sigma_BC, attInfo.sigma_BN); // sigma_NC now
    MRP2C(attInfo.sigma_BN, dcm_NC);

    /*! - Find pixel size using camera specs */
    double X, Y;
    X = cameraSpecs.sensorSize[0]*0.001/cameraSpecs.resolution[0]; // mm to meters
    Y = cameraSpecs.sensorSize[1]*0.001/cameraSpecs.resolution[1];

    /*! - Get the heading */
    double rtilde_C[2];
    double rHat_C[3], rHat_N[3];
    double rNorm = 1;
    double planetRad, denom;
    double covar_map[3*3], covar_In[3*3];
    double x_map, y_map, rho_map;
    double scale_map[3];
    
    rtilde_C[0] = 1./cameraSpecs.focalLength*(X*circlesIn.circlesCenters[0]);
    rtilde_C[1] = 1./cameraSpecs.focalLength*(Y*circlesIn.circlesCenters[1]);
    v2Set(rtilde_C[0], rtilde_C[1], rHat_C);
    rHat_C[2] = 1;
    v3Normalize(rHat_C, rHat_C);
    
    m33MultV3(dcm_NC, rHat_C, rHat_N);

    if(configData->planetTarget > 0){
        if(configData->planetTarget ==1){planetRad = REQ_EARTH;} //in km
        if(configData->planetTarget ==2){planetRad = REQ_MARS;} //in km
        if(configData->planetTarget ==3){planetRad = REQ_JUPITER;} //in km
        
        denom = sin(atan(X*circlesIn.circlesRadii[0]/cameraSpecs.focalLength));
        rNorm = planetRad/denom; //in km
        
        /*! - Compute the uncertainty */
        x_map =   planetRad/denom*(X/cameraSpecs.focalLength);
        y_map =  planetRad/denom*(Y/cameraSpecs.focalLength);
        rho_map = planetRad*(X/(cameraSpecs.focalLength*sqrt(1 + pow(circlesIn.circlesRadii[0]*X/cameraSpecs.focalLength,2)))-cameraSpecs.focalLength*sqrt(1 + pow(circlesIn.circlesRadii[0]*X/cameraSpecs.focalLength,2))/(pow(circlesIn.circlesRadii[0], 2)*X));
        v3Set(x_map, y_map, rho_map, scale_map);
        m33MultV3(RECAST3X3 covar_map, scale_map, covar_map);
        mCopy(circlesIn.uncertainty, 3, 3, covar_In);
        mMultM(covar_map, 3, 3, covar_In, 3, 3, covar_In);
        mMultMt(covar_In, 3, 3, covar_map, 3, 3, covar_In);
        /*! - Changer the mapped covariance to inertial frame */
        mMultM(dcm_NC, 3, 3, covar_In, 3, 3, covar_In);
        mMultMt(covar_In, 3, 3, dcm_NC, 3, 3, covar_In);
    }
    
    /*! - write output message */
    v3Scale(rNorm, rHat_N, opNavMsgOut.r_B);
    mCopy(covar_In, 3, 3, opNavMsgOut.covar);
    WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpnavFswMsg),
                 &opNavMsgOut, moduleID);

    return;
}
