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
    Hough Circle Finder

    Note:   This module takes an image and writes out the circles that are found in the image by OpenCV's HoughCricle Transform.
    Author: Thibaud Teil
    Date:   February 13, 2019
 
 */

/* modify the path to reflect the new module names */
#include <string.h>
#include "houghCircles.h"


/*! The constructor for the HoughCircles module. It also sets some default values at its creation.  */
HoughCircles::HoughCircles()
{
    this->filename = "";
    this->saveImages = 0;
    this->noiseSF = 4;
    this->blurrSize = 5;
    this->dpValue = 1;
    this->OutputBufferCount = 2;
    this->expectedCircles = MAX_CIRCLE_NUM;
    this->saveDir = "";
    this->cannyThresh = 200;
    this->voteThresh = 20;
    this->houghMinDist = 50;
    this->houghMinRadius = 0;
    this->houghMaxRadius = 0; // Maximum circle radius. If <= 0, uses the maximum image dimension. If < 0, returns centers without finding the radius
}

/*! Selfinit performs the first stage of initialization for this module.
 It's primary function is to create messages that will be written to.
 @return void
 */
void HoughCircles::SelfInit()
{
    /*! - Create output message for module */
    this->opnavCirclesOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(this->opnavCirclesOutMsgName,sizeof(CirclesOpNavMsg),this->OutputBufferCount,"CirclesOpNavMsg",moduleID);
}


/*! CrossInit performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 */
void HoughCircles::CrossInit()
{
    /*! - Get the image data message ID*/
    this->imageInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->imageInMsgName,sizeof(CameraImageMsg), moduleID);
}

/*! This is the destructor */
HoughCircles::~HoughCircles()
{
    return;
}


/*! This method performs a complete reset of the module.  Local module variables that retain time varying states between function calls are reset to their default values.
 @return void
 @param this The configuration data associated with the module
 */
void HoughCircles::Reset(uint64_t CurrentSimNanos)
{
    return;
}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding. 
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void HoughCircles::UpdateState(uint64_t CurrentSimNanos)
{
    CameraImageMsg imageBuffer;
    CirclesOpNavMsg circleBuffer;
    memset(&imageBuffer, 0x0, sizeof(CameraImageMsg));
    memset(&circleBuffer, 0x0, sizeof(CirclesOpNavMsg));
    
    cv::Mat imageCV, blurred;
    int circlesFound=0;
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
        SystemMessaging::GetInstance()->WriteMessage(this->opnavCirclesOutMsgID, CurrentSimNanos, sizeof(CirclesOpNavMsg), reinterpret_cast<uint8_t *>(&circleBuffer), this->moduleID);
        return;}

    cv::cvtColor( imageCV, imageCV, cv::COLOR_BGR2GRAY);
    cv::threshold(imageCV, imageCV, 15, 255, cv::THRESH_BINARY_INV);
    cv::blur(imageCV, blurred, cv::Size(this->blurrSize,this->blurrSize) );
    
    std::vector<cv::Vec4f> circles;
    /*! - Apply the Hough Transform to find the circles*/
    cv::HoughCircles( blurred, circles, cv::HOUGH_GRADIENT, this->dpValue, this->houghMinDist, this->cannyThresh,this->voteThresh, this->houghMinRadius, this->houghMaxRadius );

    circleBuffer.timeTag = this->sensorTimeTag;
    circleBuffer.cameraID = imageBuffer.cameraID;
    for( size_t i = 0; i < this->expectedCircles && i<circles.size(); i++ )
    {
        circleBuffer.circlesCenters[2*i] = circles[i][0];
        circleBuffer.circlesCenters[2*i+1] = circles[i][1];
        circleBuffer.circlesRadii[i] = circles[i][2];
        for(int j=0; j<3; j++){
            circleBuffer.uncertainty[j+3*j] = this->noiseSF*circles[i][3]/this->voteThresh;
        }
        circlesFound+=1;
    }
    /*!- If no circles are found do not validate the image as a measurement */
    if (circlesFound >0){
        circleBuffer.valid = 1;
        circleBuffer.planetIds[0] = 2;
    }
    
    SystemMessaging::GetInstance()->WriteMessage(this->opnavCirclesOutMsgID, CurrentSimNanos, sizeof(CirclesOpNavMsg), reinterpret_cast<uint8_t *>(&circleBuffer), this->moduleID);

//    free(imageBuffer.imagePointer);
    return;
}

