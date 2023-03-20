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

#include "centerOfBrightness.h"

CenterOfBrightness::CenterOfBrightness() = default;

CenterOfBrightness::~CenterOfBrightness() = default;

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states
 * between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::Reset(uint64_t CurrentSimNanos)
{
    if (!this->imageInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CenterOfBrightness.imageInMsg wasn't connected.");
    }
}

/*! This module reads an OpNav image and extracts the weighted center of brightness. It performs a grayscale, a blur,
 * and a threshold on the image before summing the weighted pixel intensities in order to average them with the
 * total detected intensity. This provides the center of brightness measurement (as well as the total number of
 * bright pixels)
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::UpdateState(uint64_t CurrentSimNanos)
{
    CameraImageMsgPayload imageBuffer = this->imageInMsg.zeroMsgPayload;
    OpNavCOBMsgPayload cobBuffer = this->opnavCOBOutMsg.zeroMsgPayload;
    cv::Mat imageCV;

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
    }
    else{
        /*! - If no image is present, write zeros in message */
        this->opnavCOBOutMsg.write(&cobBuffer, this->moduleID, CurrentSimNanos);
        return;
    }

    std::string dirName;
    /*! - Save image to prescribed path if requested */
    if (this->saveImages) {
        dirName = this->saveDir + std::to_string(CurrentSimNanos * 1E-9) + ".png";
        if (!cv::imwrite(dirName, imageCV)) {
            bskLogger.bskLog(BSK_WARNING, "CenterOfBrightness: wasn't able to save images.");
        }
    }

    std::vector<cv::Vec2i> locations = this->extractBrightPixels(imageCV);

    /*!- If no lit pixels are found do not validate the image as a measurement */
    if (!locations.empty()){
        Eigen::Vector2d cobCoordinates;
        cobCoordinates = this->weightedCenterOfBrightness(locations);

        cobBuffer.valid = 1;
        cobBuffer.timeTag = this->sensorTimeTag;
        cobBuffer.cameraID = imageBuffer.cameraID;
        cobBuffer.centerOfBrightness[0] = cobCoordinates[0];
        cobBuffer.centerOfBrightness[1] = cobCoordinates[1];
        cobBuffer.pixelsFound = locations.size();
    }
    
    this->opnavCOBOutMsg.write(&cobBuffer, this->moduleID, CurrentSimNanos);

}

/*! Method extracts the bright pixels (above a given threshold) by first grayscaling then bluring image.
 @return std 2 vector of integers
 @param image openCV matrix of the input image
 */
std::vector<cv::Vec2i> CenterOfBrightness::extractBrightPixels(cv::Mat image)
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
Eigen::Vector2d CenterOfBrightness::weightedCenterOfBrightness(std::vector<cv::Vec2i> nonZeroPixels)
{
    uint32_t weight, weightSum;
    Eigen::Vector2d coordinates;
    coordinates.setZero();
    weightSum = 0;
    for(auto & pixel : nonZeroPixels) {
        /*! Individual pixel intensity used as the weight for the contribution to the solution*/
        weight = (uint32_t) this->imageGray.at<unsigned char>(pixel[1], pixel[0]);
        coordinates[0] += weight * pixel[0];
        coordinates[1] += weight * pixel[1];
        weightSum += weight; // weighted sum of all the pixels
    }
    coordinates /= weightSum;
    return coordinates;
}
