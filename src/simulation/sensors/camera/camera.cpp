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

/*!
 * Adds gaussian noise to an image.
 * Can be used to add color noise and dark current.
 * @param cv::Mat source image
 * @param cv::Mat destination of modified image
 * @param double mean pixel value
 * @param double standard deviation of pixel value
 * @return void
 */
void Camera::AddGaussianNoise(const cv::Mat mSrc, cv::Mat &mDst, double Mean, double StdDev)
{
    cv::Mat mSrc_16SC;
    //CV_16SC3 means signed 16 bit shorts three channels
    cv::Mat mGaussian_noise = cv::Mat(mSrc.size(), CV_16SC3);
    randn(mGaussian_noise, cv::Scalar::all(Mean), cv::Scalar::all(StdDev));
    
    mSrc.convertTo(mSrc_16SC, CV_16SC3);
    addWeighted(mSrc_16SC, 1.0, mGaussian_noise, 1.0, 0.0, mSrc_16SC);
    mSrc_16SC.convertTo(mDst,mSrc.type());
}

/*!
 * Adds dead and hot pixels to an image.
 * @param cv::Mat source image
 * @param cv::Mat destination of modified image
 * @param float probability of dead pixels
 * @param float probability of hot pixels
 * @return void
 */
void Camera::AddSaltPepper(const cv::Mat mSrc, cv::Mat &mDst, float pa, float pb){
    // use this to have different stuck pixels every time
    // uint64 initValue = time(0);
    // RNG rng(initValue);
    
    // use this to always have the same stuck pixels
    cv::RNG rng;
    
    int amount1 = mSrc.rows * mSrc.cols * pa;
    int amount2 = mSrc.rows * mSrc.cols * pb;
    
    cv::Mat mSaltPepper = cv::Mat(mSrc.size(), mSrc.type());
    mSrc.convertTo(mSaltPepper, mSrc.type());
    
    cv::Vec3b black;
    black.val[0] = 0;
    black.val[1] = 0;
    black.val[2] = 0;
    
    cv::Vec3b white;
    white.val[0] = 255;
    white.val[1] = 255;
    white.val[2] = 255;
    
    for(int counter = 0; counter < amount1; counter++){
        mSaltPepper.at<cv::Vec3b>(rng.uniform(0, mSaltPepper.rows), rng.uniform(0, mSaltPepper.cols)) = black;
    }
    
    for(int counter = 0; counter < amount2; counter++){
        mSaltPepper.at<cv::Vec3b>(rng.uniform(0, mSaltPepper.rows), rng.uniform(0, mSaltPepper.cols)) = white;
    }
    
    mSaltPepper.convertTo(mDst, mSrc.type());
}

/*!
 * Adds a cosmic ray to an image.
 * @param cv::Mat source image
 * @param cv::Mat destination of modified image
 * @param float probability of getting a ray each frame
 * @return void
 */
void Camera::AddCosmicRay(const cv::Mat mSrc, cv::Mat &mDst, float probThreshhold){
    uint64 initValue = time(0);
    cv::RNG rng(initValue);
    std::cout<<time(0)<<std::endl;
    
    float prob = rng.uniform(0.0, 1.0);
    if (prob > probThreshhold) {
    cv::Mat mCosmic = cv::Mat(mSrc.size(), mSrc.type());
    mSrc.convertTo(mCosmic, mSrc.type());
    
    cv::Point p1 = cv::Point(rng.uniform(0, mCosmic.rows), rng.uniform(0, mCosmic.cols));
    cv::Point p2 = cv::Point(rng.uniform(0, mCosmic.rows), rng.uniform(0, mCosmic.cols));
    
    line(mCosmic, p1, p2, cv::Scalar(255, 255, 255), 1, cv::LINE_8);
    
    mCosmic.convertTo(mDst, mSrc.type());
    }
}

void Camera::ApplyFilters(cv::Mat mSource, cv::Mat &mDst, int gaussian, int darkCurrent, int saltPepper, int cosmicRay, float gaussianP, float darkCurrentP, float saltPepperP, float cosmicRayP){
    
    cv::Mat mFilters(mSource.size(), mSource.type());
    
    if (gaussian == 1){
        AddGaussianNoise(mSource, mFilters, 0, 50.0);
    }
    if(darkCurrent == 1){
        AddGaussianNoise(mFilters, mFilters, 20, 0.0);
    }
    if (saltPepper == 1){
        AddSaltPepper(mFilters, mFilters, 0.01, 0.01);
    }
    if(cosmicRay == 1){
        AddCosmicRay(mFilters, mFilters, 0);
    }
    mFilters.convertTo(mDst, mSource.type());
}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding. 
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void Camera::UpdateState(uint64_t CurrentSimNanos)
{
    std::string localPath;
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
    cameraMsg.isOn = this->cameraIsOn;
    v2Copy(this->sensorSize, cameraMsg.sensorSize);
    v3Copy(this->cameraPos_B, cameraMsg.cameraPos_B);
    v3Copy(this->sigma_CB, cameraMsg.sigma_CB);
    strcpy(cameraMsg.skyBox, this->skyBox);
    
    /*! - Update the camera config data no matter if an image is present*/
    SystemMessaging::GetInstance()->WriteMessage(this->cameraOutID, CurrentSimNanos, sizeof(CameraConfigMsg), reinterpret_cast<uint8_t *>(&cameraMsg), this->moduleID);
    
    cv::Mat imageCV, blurred;
    if (this->saveDir !=""){
        localPath = this->saveDir + std::to_string(CurrentSimNanos*1E-9) + ".jpg";
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
        ApplyFilters(imageCV, blurred, 1, 1, 1, 1, 0, 0, 0, 0);
        if (this->saveImages == 1){
            cv::imwrite(localPath, blurred);
        }
    }
    else if(imageBuffer.valid == 1 && imageBuffer.timeTag >= CurrentSimNanos){
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer((char*)imageBuffer.imagePointer, (char*)imageBuffer.imagePointer + imageBuffer.imageBufferLength);
        imageCV = cv::imdecode(vectorBuffer, cv::IMREAD_COLOR);
        ApplyFilters(imageCV, blurred, 1, 1, 1, 1, 0, 0, 0, 0);
        if (this->saveImages == 1){
            cv::imwrite(localPath, blurred);
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

