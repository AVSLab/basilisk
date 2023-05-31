/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "opticalFlow.h"

OpticalFlow::OpticalFlow() = default;

OpticalFlow::~OpticalFlow() = default;

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states
 * between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void OpticalFlow::Reset(uint64_t CurrentSimNanos)
{
    // check that the required message has not been connected
    if (!this->imageInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "opticalFlow.imageInMsg wasn't connected.");
    }
    // check that the required message has not been connected
    if (!this->ephemerisMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "opticalFlow.ephemerisMsg wasn't connected.");
    }
}

/*! This method reads the images and follows the following high level logic:
 * If no image has been saved from previous time steps and a second image is present, save it and make a mask
 * If an image was previously saved and a second image arrives, perform the feature matching
 * Return the features and clear the memory of both previous images
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void OpticalFlow::UpdateState(uint64_t CurrentSimNanos)
{
    std::string dirName;
    CameraImageMsgPayload imageBuffer;
    PairedKeyPointsMsgPayload featurePayload;

    imageBuffer = this->imageInMsg.zeroMsgPayload;
    featurePayload = this->keyPointsMsg.zeroMsgPayload;

    /*! - Read in the bitmap*/
    imageBuffer = this->imageInMsg();

    this->sensorTimeTag = 0;
    /* Added for debugging purposes*/
    if (!this->filename.empty()){
        this->secondImage = cv::imread(this->filename, cv::IMREAD_COLOR);
        this->sensorTimeTag = CurrentSimNanos;
        this->secondImagePresent = true;
        this->filename = "";
    }
    else if(imageBuffer.valid == 1 && imageBuffer.timeTag > this->firstTimeTag){
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer((char*)imageBuffer.imagePointer,
                                                (char*)imageBuffer.imagePointer + imageBuffer.imageBufferLength);
        this->secondImage = cv::imdecode(vectorBuffer, cv::IMREAD_COLOR);

        this->sensorTimeTag = imageBuffer.timeTag;
        this->secondImagePresent = true;
    }

    NavAttMsgPayload navAttBuffer;
    navAttBuffer = this->attitudeMsg();
    EphemerisMsgPayload ephemMsgBuffer;
    ephemMsgBuffer = this->ephemerisMsg();
    double dtBetweenImagesSeconds;
    dtBetweenImagesSeconds = (double)(this->sensorTimeTag - this->firstTimeTag)*NANO2SEC;
    /*! - If there is a second image and an first image, write the paired features message */
    if (this->firstImagePresent && this->secondImagePresent && dtBetweenImagesSeconds >= this->minTimeBetweenPairs){
        cv::cvtColor(this->secondImage, this->secondImage, cv::COLOR_BGR2GRAY);

        std::vector<uchar> status;
        std::vector<float> err;
        auto criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                                     this->criteriaMaxCount,
                                                     this->criteriaEpsilon);
        cv::calcOpticalFlowPyrLK(this->firstImage,
                                 this->secondImage,
                                 this->firstFeatures,
                                 this->secondFeatures,
                                 status,
                                 err,
                                 cv::Size(this->flowSearchSize,this->flowSearchSize),
                                 this->flowMaxLevel,
                                 criteria);

        featurePayload.timeTag_secondImage = this->sensorTimeTag;
        featurePayload.timeTag_firstImage = this->firstTimeTag;

        v3Copy(this->firstSpacecraftAttitude, featurePayload.sigma_BN_firstImage);
        v3Copy(navAttBuffer.sigma_BN, featurePayload.sigma_BN_secondImage);
        v3Copy(this->firstTargetEphemAttitude, featurePayload.sigma_TN_firstImage);
        v3Copy(ephemMsgBuffer.sigma_BN, featurePayload.sigma_TN_secondImage);

        for (int i=0; i<this->secondFeatures.size(); i++){
            featurePayload.keyPoints_firstImage[2*i] = this->firstFeatures[i][0];
            featurePayload.keyPoints_firstImage[2*i+1] = this->firstFeatures[i][1];
            featurePayload.keyPoints_secondImage[2*i] = this->secondFeatures[i][0];
            featurePayload.keyPoints_secondImage[2*i+1] = this->secondFeatures[i][1];
        }
        featurePayload.cameraID = imageBuffer.cameraID;
        featurePayload.keyPointsFound = (uint64_t)this->secondFeatures.size();
        featurePayload.valid = true;

        this->keyPointsMsg.write(&featurePayload, this->moduleID, CurrentSimNanos);

        this->secondImagePresent = false;
        this->firstFeatures = this->secondFeatures;
        /*! Then reset values for next pair of images */
        if (this->slidingWindowImages){
            this->firstImagePresent = true;
            v3Copy(navAttBuffer.sigma_BN, this->firstSpacecraftAttitude);
            v3Copy(ephemMsgBuffer.sigma_BN, this->firstTargetEphemAttitude);
            this->firstTimeTag = this->sensorTimeTag;
        }
        else{
            this->firstImagePresent = false;
            v3SetZero(this->firstSpacecraftAttitude);
            v3SetZero(this->firstTargetEphemAttitude);
            this->firstTimeTag = 0;
        }
    }
    /*! - If there is a second image but also a first image, populate the first image buffers */
    else if(this->secondImagePresent){
        cv::cvtColor(this->secondImage, this->firstImage, cv::COLOR_BGR2GRAY);
        cv::Mat mask(this->firstImage.size(), CV_8UC1, cv::Scalar(255));

        OpticalFlow::makeMask(this->firstImage, mask);
        cv::goodFeaturesToTrack(this->firstImage,
                                this->secondFeatures,
                                this->maxNumberFeatures,
                                this->qualityLevel,
                                this->minumumFeatureDistance,
                                 mask,
                                this->blockSize,
                                false,
                                0.04);

        v3Copy(navAttBuffer.sigma_BN, this->firstSpacecraftAttitude);
        v3Copy(ephemMsgBuffer.sigma_BN, this->firstTargetEphemAttitude);
        this->firstTimeTag = this->sensorTimeTag;
        this->firstFeatures = this->secondFeatures;
        this->firstImagePresent = true;
        this->secondImagePresent = false;
    }
    /*! - If no second image is present, write zeros in message and set valid to false*/
    else{
        featurePayload.valid = false;
        this->keyPointsMsg.write(&featurePayload, this->moduleID, CurrentSimNanos);
        }

}

/*! This method reads an black and white image and makes a mask in order to remove the limb of the target.
 * First the contours are found and drawn on an image.
 * Second the main contour is filled with black
 * Third the image is blured and thresholded to remove roughness on the limb
 * Finally the edge is dilated to increase the margin off the limb
 @return void
 @param inputBWImage cv::Mat of the input image
 @param mask cv::Mat of the output mask (binary black and white image)
 */
void OpticalFlow::makeMask (cv::Mat const & inputBWImage, cv::Mat mask) const {

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(inputBWImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    cv::drawContours(mask, contours, -1, 1);

    for(auto const &contour: contours){
        cv::fillConvexPoly(mask, contour, 0);
    }

    cv::Mat kernel;
    kernel = cv::Mat::ones(cv::Size(this->dilutionMask, this->dilutionMask), CV_8UC1);

    cv::Mat blur;
    cv::blur(mask, blur, cv::Size(this->dilutionMask,this->dilutionMask));
    cv::Mat thresh;
    cv::threshold(blur, thresh, this->thresholdMask, 255, cv::THRESH_BINARY);
    cv::dilate(thresh, blur, kernel, cv::Point(-1,-1), 2);
    cv::bitwise_not(blur, mask);
}
