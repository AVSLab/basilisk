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

#include <string.h>
#include "centerOfBrightness.h"

/*! The constructor for the CenterOfBrightness module. It also sets some default values at its creation.  */
CenterOfBrightness::CenterOfBrightness()
{
    this->filename = "";
    this->saveImages = 1;
    this->blurrSize = 5;
    this->threshold = 50;
    this->saveDir = "";
}

/*! This is the destructor */
CenterOfBrightness::~CenterOfBrightness()
{
    return;
}

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::Reset(uint64_t CurrentSimNanos)
{
    // check that the required message has not been connected
    if (!this->imageInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CenterOfBrightness.imageInMsg wasn't connected.");
    }
}

/*! This module reads an OpNav image and extracts the weighted center of brightness. It performs a greyscale, a blurr,
 * and a threshold on the image before summing the weighted pixel intensities in order to average them with the
 * total detected intensity. This provides the center of brightness measurement (as well as the total number of
 * bright pixels)
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::UpdateState(uint64_t CurrentSimNanos)
{
    std::string dirName;
    CameraImageMsgPayload imageBuffer;
    OpNavCOBMsgPayload cobBuffer;

    imageBuffer = this->imageInMsg.zeroMsgPayload;
    cobBuffer = this->opnavCOBOutMsg.zeroMsgPayload;

    cv::Mat imageCV, blurred, imageGray;
    if (this->saveDir != ""){
        dirName = this->saveDir + std::to_string(CurrentSimNanos*1E-9) + ".jpg";
    }
    else{dirName = "./"+ std::to_string(CurrentSimNanos*1E-9) + ".jpg";}
    
    /*! - Read in the image*/
    if(this->imageInMsg.isLinked())
    {
        imageBuffer = this->imageInMsg();
        this->sensorTimeTag = this->imageInMsg.timeWritten();
    }
    /* Added for debugging purposes*/
    if (!this->filename.empty()){
        imageCV = cv::imread(this->filename, cv::IMREAD_COLOR);
    }
    else if(imageBuffer.valid == 1 && imageBuffer.timeTag >= CurrentSimNanos){
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer((char*)imageBuffer.imagePointer, (char*)imageBuffer.imagePointer + imageBuffer.imageBufferLength);
        imageCV = cv::imdecode(vectorBuffer, cv::IMREAD_COLOR);
        if (this->saveImages == 1){
            if (!cv::imwrite(dirName, imageCV)) {
                bskLogger.bskLog(BSK_WARNING, "CenterOfBrightness: wasn't able to save images.");
            }
        }
    }
    else{
        /*! - If no image is present, write zeros in message */
        this->opnavCOBOutMsg.write(&cobBuffer, this->moduleID, CurrentSimNanos);
        return;
    }
    cv::cvtColor( imageCV, imageGray, cv::COLOR_BGR2GRAY);
    cv::blur(imageGray, blurred, cv::Size(this->blurrSize,this->blurrSize) );
    cv::threshold(blurred, imageCV, this->threshold, 255, cv::THRESH_BINARY);
    
    std::vector<cv::Vec2i> locations;
    /*! - Find all the non-zero pixels in the image*/
    cv::findNonZero(imageCV, locations);
    
    cobBuffer.timeTag = this->sensorTimeTag;
    cobBuffer.cameraID = imageBuffer.cameraID;
    uint32_t weight, weightSum;
    weightSum = 0;
    /*!- If no lit pixels are found do not validate the image as a measurement */
    if ((int) locations.size() > 0){
        for( int i = 0; i<(int) locations.size(); i++ )
        {
            /*! Individual pixel intensity used as the weight for the contribution to the solution*/
            weight = (uint32_t)imageGray.at<unsigned char>(locations[i][1], locations[i][0]);
            cobBuffer.centerOfBrightness[0] += weight*locations[i][0];
            cobBuffer.centerOfBrightness[1] += weight*locations[i][1];
            weightSum += weight; // weighted sum of all the pixels
        }

        cobBuffer.valid = 1;
        cobBuffer.centerOfBrightness[0]/=weightSum;
        cobBuffer.centerOfBrightness[1]/=weightSum;
        cobBuffer.pixelsFound =(int)locations.size();
    }
    
    this->opnavCOBOutMsg.write(&cobBuffer, this->moduleID, CurrentSimNanos);

    return;
}

/*! Method extracts the bright pixels (above a given threshold) by first grayscaling then bluring image.
 @return std 2 vector of integers
 @param image openCV matrix of the input image
 */
std::vector<cv::Vec2i> CenterOfBrightness::extractPixels(cv::Mat image)
{
    cv::Mat blured;
    std::vector<cv::Vec2i> locations;

    /*! - Grayscale, blur, and threshold iamge*/
    cv::cvtColor(image, this->imageGray, cv::COLOR_BGR2GRAY);
    cv::blur(this->imageGray, blured, cv::Size(this->blurSize,this->blurSize) );
    cv::threshold(blured, image, this->threshold, 255, cv::THRESH_BINARY);

    /*! - Find all the non-zero pixels in the image*/
    cv::findNonZero(image, locations);

    return locations;
}

/*! Method computes the weighted center of brightness out of the non-zero pixel coordinates.
 @return Eigen 2 vector of pixel values
 @param vector integer pixel coordinates of bright pixels
 */
Eigen::Vector2d CenterOfBrightness::weightedCOB(std::vector<cv::Vec2i> nonZeros)
{
    uint32_t weight, weightSum;
    Eigen::Vector2d coordinates;
    coordinates.setZero();
    weightSum = 0;
    for( int i = 0; i<(int) nonZeros.size(); i++ ) {
        /*! Individual pixel intensity used as the weight for the contribution to the solution*/
        weight = (uint32_t) this->imageGray.at<unsigned char>(nonZeros[i][1], nonZeros[i][0]);
        coordinates[0] += weight * nonZeros[i][0];
        coordinates[1] += weight * nonZeros[i][1];
        weightSum += weight; // weighted sum of all the pixels
    }
    coordinates /= weightSum;
    return coordinates;
}