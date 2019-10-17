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
/*
    Camera Module

    Note:   This module simulates a camera. It writes a camera message with it's specs and image requests, as well as provides a template for image coruption
    Author: Thibaud Teil
    Date:   October 03, 2019
 
 */

/* modify the path to reflect the new module names */
#include <Eigen/Dense>
#include <string.h>
#include "camera.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"

/*! The constructor for the Camera module. It also sets some default values at its creation.  */
Camera::Camera()
{
    this->OutputBufferCount = 2;

    /*! Default values for the camera.  */
    strcpy(this->parentName, "spacecraft");
    this->cameraID = 1;
    this->fieldOfView = 0.7;
    this->resolution[0]=512;
    this->resolution[1]=512;
    this->renderRate = 60*1E9;
    v2Set(1E-3, 1E-3, this->sensorSize);
    v3SetZero(this->cameraPos_B);
    v3SetZero(this->sigma_CB);
    this->cameraIsOn = 0;
    this->focalLength = this->sensorSize[0]/2/tan(this->fieldOfView/2.);
    strcpy(this->skyBox, "black");

    /*! Default values for the perturbations.  */
    
    return;
}

/*! Selfinit performs the first stage of initialization for this module.
 It's primary function is to create messages that will be written to.
 @return void
 */
void Camera::SelfInit()
{
    /*! - Create output message of image */
    this->imageOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(this->imageOutMsgName,sizeof(CameraImageMsg),this->OutputBufferCount,"CameraImageMsg",this->moduleID);
    /*! - Create output message for camera */
    this->cameraOutID = SystemMessaging::GetInstance()->CreateNewMessage(this->cameraOutMsgName,sizeof(CameraConfigMsg),this->OutputBufferCount,"CameraConfigMsg",this->moduleID);
}


/*! CrossInit performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 */
void Camera::CrossInit()
{
    /*! - Get the image data message ID*/
    this->imageInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->imageInMsgName,sizeof(CameraImageMsg), this->moduleID);
}

/*! This is the destructor */
Camera::~Camera()
{
    return;
}


/*! This method performs a complete reset of the module.  Local module variables that retain time varying states between function calls are reset to their default values.
 @return void
 @param this The configuration data associated with the module
 */
void Camera::Reset(uint64_t CurrentSimNanos)
{
    return;
}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding. 
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void Camera::UpdateState(uint64_t CurrentSimNanos)
{
    CameraImageMsg imageBuffer;
    CameraConfigMsg cameraMsg;
    memset(&imageBuffer, 0x0, sizeof(CameraImageMsg));
    memset(&cameraMsg, 0x0, sizeof(CameraConfigMsg));
    
    /*! - Populate the camera message */
    cameraMsg.cameraID = this->cameraID;
    strcpy(cameraMsg.parentName, this->parentName);
    cameraMsg.fieldOfView = this->fieldOfView;
    cameraMsg.resolution[0] = this->resolution[0];
    cameraMsg.resolution[1] = this->resolution[1];
    cameraMsg.renderRate = this->renderRate;
    cameraMsg.focalLength = this->focalLength;
    v2Copy(this->sensorSize, cameraMsg.sensorSize);
    v3Copy(this->cameraPos_B, cameraMsg.cameraPos_B);
    v3Copy(this->sigma_CB, cameraMsg.sigma_CB);
    strcpy(cameraMsg.skyBox, this->skyBox);
    
    /*! - Update the camera config data no matter if an image is present*/
    SystemMessaging::GetInstance()->WriteMessage(this->cameraOutID, CurrentSimNanos, sizeof(CameraConfigMsg), reinterpret_cast<uint8_t *>(&cameraMsg), this->moduleID);
    
    cv::Mat imageCV, blurred;
    if (this->saveDir !=""){
        this->saveDir = this->saveDir + std::to_string(CurrentSimNanos*1E-9) + ".jpg";
    }
    /*! - Read in the bitmap*/
    SingleMessageHeader localHeader;
    if(this->imageInMsgName != "")
    {
        SystemMessaging::GetInstance()->ReadMessage(this->imageInMsgID, &localHeader,
                                                    sizeof(CameraImageMsg), reinterpret_cast<uint8_t*>(&imageBuffer), this->moduleID);
        this->sensorTimeTag = localHeader.WriteClockNanos;
    }
    /* Added for debugging purposes*/
    if (!this->filename.empty()){
        imageCV = imread(this->filename, cv::IMREAD_COLOR);
    }
    else if(imageBuffer.valid == 1 && imageBuffer.timeTag >= CurrentSimNanos){
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer((char*)imageBuffer.imagePointer, (char*)imageBuffer.imagePointer + imageBuffer.imageBufferLength);
        imageCV = cv::imdecode(vectorBuffer, cv::IMREAD_COLOR);
        if (this->saveImages == 1){
            cv::imwrite(this->saveDir, imageCV);
        }
    }
    else{
        /*! - If no image is present, write zeros in message */
        SystemMessaging::GetInstance()->WriteMessage(this->imageOutMsgID, CurrentSimNanos, sizeof(CameraImageMsg), reinterpret_cast<uint8_t *>(&imageBuffer), this->moduleID);
        return;}
 
    /*! - Output the saved image */
    SystemMessaging::GetInstance()->WriteMessage(this->imageOutMsgID, CurrentSimNanos, sizeof(CameraImageMsg), reinterpret_cast<uint8_t *>(&imageBuffer), this->moduleID);
    
    return;
}

