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

    Note:   this module takes a bit-map image and writes out the circles that are found in the image by OpenCV's HoughCricle Transform.
    Author: Thibaud Teil
    Date:   February 13, 2019
 
 */

/* modify the path to reflect the new module names */
#include <string.h>
#include "houghCircles.h"


/*! This method constructs the module.
It also sets some default values at its creation */
HoughCircles::HoughCircles()
{
    this->blurrSize = 5;
    this->OutputBufferCount = 2;
    this->cannyThresh1 = 200;
    this->cannyThresh2 = 20;
    this->houghMinDist = 50;
    this->houghMinRadius = 0;
    this->houghMaxRadius = 0; // Maximum circle radius. If <= 0, uses the maximum image dimension. If < 0, returns centers without finding the radius
}

/*! This method performs the first stage of initialization for this module.
 It's primary function is to create messages that will be written to.
 @return void
 */
void HoughCircles::SelfInit()
{
    /*! - Create output message for module */
    this->opnavCirclesOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(this->opnavCirclesOutMsgName,
                                                  sizeof(CirclesOpNavMsg),
                                                  this->OutputBufferCount,
                                                  "CirclesOpNavMsg",
                                                  moduleID);
}


/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 */
void HoughCircles::CrossInit()
{
    /*! - Get the image data message ID*/
    this->imageInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->imageInMsgName,
                                                sizeof(CameraImageMsg),
                                                moduleID);
}

/*! This is the destructor
 @return void
 */
HoughCircles::~HoughCircles()
{
    return;
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param this The configuration data associated with the module
 */
void HoughCircles::Reset(uint64_t CurrentSimNanos)
{
    return;
}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle Transform.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void HoughCircles::UpdateState(uint64_t CurrentSimNanos)
{
    CameraImageMsg imageBuffer;
    CirclesOpNavMsg circleBuffer;
    cv::Mat canny, grey, blurred;
    /*! - Read in the bitmap*/
    SingleMessageHeader localHeader;
    memset(&imageBuffer, 0x0, sizeof(CameraImageMsg));
    if(this->imageInMsgID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(this->imageInMsgID, &localHeader,
                                                    sizeof(CameraImageMsg), reinterpret_cast<uint8_t*>(&imageBuffer), this->moduleID);
        this->sensorTimeTag = localHeader.WriteClockNanos;
    }

    /*! - Recast image pointer to Eigen type*/
//    double* pMat[imageBuffer.imageWidth][imageBuffer.imageHeight] = static_cast<double>(*imageBuffer.imagePointer);
    uint32_t* pMat = (uint32_t *) imageBuffer.imagePointer;
    cv::Mat src(imageBuffer.imageWidth,imageBuffer.imageHeight,CV_32SC4);
    std::memcpy(src.data, pMat, imageBuffer.imageWidth*imageBuffer.imageHeight*imageBuffer.imageType*sizeof(uint8_t));
//    src = cv::imdecode(imageBuffer.imagePointer, cv::IMREAD_ANYCOLOR);
    cv::blur(grey, blurred, cv::Size(this->blurrSize,this->blurrSize) );
//    cv::imshow( "Hough Circle Transform Demo", src );
//    cv::waitKey(0);

    std::vector<cv::Vec4f> circles;
    /*! - Apply the Hough Transform to find the circles*/
    cv::HoughCircles( blurred, circles, CV_HOUGH_GRADIENT, 1, src.rows/2, this->cannyThresh1,this->cannyThresh2, this->houghMinRadius, this->houghMaxRadius );
    
    circleBuffer.timeTag = this->sensorTimeTag;
    circleBuffer.cameraID = imageBuffer.cameraID;
    for( size_t i = 0; i < MAX_CIRCLE_NUM; i++ )
    {
        circleBuffer.circlesCenters[2*i] = circles[i][0];
        circleBuffer.circlesCenters[2*i+1] = circles[i][1];
        circleBuffer.circlesRadii[i] = circles[i][2];
    }
    SystemMessaging::GetInstance()->WriteMessage(this->opnavCirclesOutMsgID, CurrentSimNanos, sizeof(CirclesOpNavMsg), reinterpret_cast<uint8_t *>(&circleBuffer), this->moduleID);
    
    return;
}

