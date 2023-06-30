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
    CameraImageMsgPayload imageBuffer;
    PairedKeyPointsMsgPayload featurePayload;

    imageBuffer = this->imageInMsg.zeroMsgPayload;
    featurePayload = this->keyPointsMsg.zeroMsgPayload;

    /*! - Read in the bitmap*/
    imageBuffer = this->imageInMsg();

    this->sensorTimeTag = 0;
    /*! - Read option which reads images from files*/
    if (!this->directoryName.empty()){
        std::string filename = this->directoryName + std::to_string(CurrentSimNanos) + this->imageFileExtension;
        std::ifstream imageFile(filename);
        if (imageFile.good()){
            this->secondImage = cv::imread(filename, cv::IMREAD_GRAYSCALE);
            this->sensorTimeTag = CurrentSimNanos;
            this->secondImagePresent = true;
        }
    }
    else if(imageBuffer.valid == 1 && imageBuffer.timeTag > this->firstTimeTag){
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer((char*)imageBuffer.imagePointer,
                                                (char*)imageBuffer.imagePointer + imageBuffer.imageBufferLength);
        this->secondImage = cv::imdecode(vectorBuffer, cv::IMREAD_GRAYSCALE);

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

        /*! If optical flow matches features between the images */
        if (!this->secondFeatures.empty()){
            featurePayload.timeTag_secondImage = this->sensorTimeTag;
            featurePayload.timeTag_firstImage = this->firstTimeTag;

            v3Copy(this->firstSpacecraftAttitude, featurePayload.sigma_BN_firstImage);
            v3Copy(navAttBuffer.sigma_BN, featurePayload.sigma_BN_secondImage);
            v3Copy(this->firstTargetEphemAttitude, featurePayload.sigma_TN_firstImage);
            v3Copy(ephemMsgBuffer.sigma_BN, featurePayload.sigma_TN_secondImage);

            for (size_t i=0; i<this->secondFeatures.size(); i++){
                featurePayload.keyPoints_firstImage[2*i] = this->firstFeatures[i][0];
                featurePayload.keyPoints_firstImage[2*i+1] = this->firstFeatures[i][1];
                featurePayload.keyPoints_secondImage[2*i] = this->secondFeatures[i][0];
                featurePayload.keyPoints_secondImage[2*i+1] = this->secondFeatures[i][1];
            }
            featurePayload.cameraID = imageBuffer.cameraID;
            featurePayload.keyPointsFound = (int)this->secondFeatures.size();
            featurePayload.valid = true;

            this->keyPointsMsg.write(&featurePayload, this->moduleID, CurrentSimNanos);

            this->secondImagePresent = false;
            /*! Then reset values for next pair of images depending on to the sliding window setting */
            if (this->slidingWindowImages){
                this->firstImagePresent = true;
                v3Copy(navAttBuffer.sigma_BN, this->firstSpacecraftAttitude);
                v3Copy(ephemMsgBuffer.sigma_BN, this->firstTargetEphemAttitude);
                this->firstTimeTag = this->sensorTimeTag;
                this->firstFeatures = this->secondFeatures;
            }
            else{
                this->firstImagePresent = false;
                this->firstTimeTag = 0;
                this->firstFeatures.clear();
            }
        }
        /*! If optical flow did not succeed, reset features to allow for another attempt next image */
        else{
            this->firstImagePresent = false;
            this->firstTimeTag = 0;
            this->firstFeatures.clear();
            this->secondFeatures.clear();
        }
    }
    /*! - If there is a second image but also a first image, populate the first image buffers */
    else if(this->secondImagePresent){
        this->firstImage = this->secondImage.clone();
        cv::Mat mask(this->firstImage.size(), CV_8UC1, cv::Scalar(255));

        OpticalFlow::makeMask(this->firstImage, mask);
        cv::goodFeaturesToTrack(this->firstImage,
                                this->firstFeatures,
                                this->maxNumberFeatures,
                                this->qualityLevel,
                                this->minumumFeatureDistance,
                                 mask,
                                this->blockSize,
                                false,
                                0.04);

        /*! If features are found, save the data*/
        if (!this->firstFeatures.empty()) {
            v3Copy(navAttBuffer.sigma_BN, this->firstSpacecraftAttitude);
            v3Copy(ephemMsgBuffer.sigma_BN, this->firstTargetEphemAttitude);
            this->firstTimeTag = this->sensorTimeTag;
            this->firstImagePresent = true;
            this->secondImagePresent = false;
        }
        /*! If no features are found, reset data to attempt again */
        else{
            this->firstImagePresent = false;
            this->secondImagePresent = false;
        }
        /*! -Write a zero message if only one image was processed*/
        featurePayload.valid = false;
        this->keyPointsMsg.write(&featurePayload, this->moduleID, CurrentSimNanos);
    }
    /*! - If no second image is present, write zeros in message and set valid to false*/
    else{
        featurePayload.valid = false;
        this->keyPointsMsg.write(&featurePayload, this->moduleID, CurrentSimNanos);
        }

}

/*! This method reads an black and white image and makes a mask in order to remove the limb of the target.
 * First the distance transform is computed from the input Image to get the distance of each point from the
 * dark (zero) pixel background.
 * Second the image is thresholded
 * Third the image is eroded by a masking value (eroding away N pixels from the edge of the limb)
 * Finally the mask is converted to the correct type
 @return void
 @param inputBWImage cv::Mat of the input image
 @param mask cv::Mat of the output mask (binary black and white image)
 */
void OpticalFlow::makeMask (cv::Mat const &inputBWImage, cv::Mat &mask) const {

    cv::Mat distanceImage(inputBWImage.size(), CV_8UC1);
    cv::distanceTransform(inputBWImage, distanceImage, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    cv::Mat thresholded(inputBWImage.size(), CV_8UC1);
    cv::threshold(distanceImage, thresholded, this->thresholdMask, 255, cv::THRESH_BINARY);

    cv::Mat erosionKernel = cv::Mat::ones(this->limbMask, this->limbMask, CV_8U);
    cv::erode(thresholded, thresholded, erosionKernel);
    thresholded.convertTo(mask, CV_8UC1);
    assert(mask.type() == CV_8UC1);
    assert(mask.size() == inputBWImage.size());
}
