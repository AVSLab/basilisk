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

    Note:   This module simulates a camera. It writes a camera message with its specs and image requests,
    as well as provides a template for image corruption
    Author: Thibaud Teil
    Date:   October 03, 2019

 */

/* modify the path to reflect the new module names */
#include <string.h>
#include "camera.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"

/*! The constructor for the Camera module. It also sets some default values at its creation.  */
Camera::Camera()
{
    this->renderRate = (uint64_t) (60*1E9);
    v3SetZero(this->cameraPos_B);
    v3SetZero(this->sigma_CB);
}

/*! This is the destructor */
Camera::~Camera() = default;

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states
 * between function calls are reset to their default values.

 @param currentSimNanos current time (ns)
 */
void Camera::Reset(uint64_t currentSimNanos)
{
}

/*!
 * Adjusts the HSV values of each pixel.
 * Can be used to shift the hue, saturation, and brightness of an image.
 * @param mSrc source image
 * @param mDst destination of modified image
 *
 */
void Camera::hsvAdjust(const cv::Mat& mSrc, cv::Mat &mDst){
    cv::Mat localHsv;
    cvtColor(mSrc, localHsv, cv::COLOR_BGR2HSV);

    for (int j = 0; j < mSrc.rows; j++) {
        for (int i = 0; i < mSrc.cols; i++) {
            // Saturation is hsv.at<Vec3b>(j, i)[1] range: [0-255]
            // Value is hsv.at<Vec3b>(j, i)[2] range: [0-255]

            // convert radians to degrees and multiply by 2
            // user assumes range hue range is 0-2pi and not 0-180
            int input_degrees = (int) (this->hsv[0] * R2D);
            int h_360 = (localHsv.at<cv::Vec3b>(j, i)[0] * 2) + input_degrees;
            h_360 = (int) (h_360 -  360 * std::floor(h_360 * (1. / 360.)));
            h_360 = h_360/2;
            if(h_360 == 180){ h_360 = 0; }
            localHsv.at<cv::Vec3b>(j, i)[0] = (unsigned char) h_360;

            int values[3];
            for(int k = 1; k < 3; k++){
                values[k] = (int) (localHsv.at<cv::Vec3b>(j, i)[k] * (this->hsv[k] / 100. + 1.));
                // saturate S and V values to [0,255]
                if(values[k] < 0){ values[k] = 0; }
                if(values[k] > 255){ values[k] = 255; }
                localHsv.at<cv::Vec3b>(j, i)[k] = (unsigned char) values[k];
            }
        }
    }
    cvtColor(localHsv, mDst, cv::COLOR_HSV2BGR);
}

/*!
 * Adjusts the BGR values of each pixel by a percent value.
 * Can be used to simulate a sensor with different sensitivities to B, G, and R.
 * @param mSrc source image
 * @param mDst destination of modified image
 *
 */
void Camera::bgrAdjustPercent(const cv::Mat& mSrc, cv::Mat &mDst){
    cv::Mat mBGR = cv::Mat(mSrc.size(), mSrc.type());
    mSrc.convertTo(mBGR, mSrc.type());

    // BGR values range [0, 255]
    // if value after adjustment is < 0 take 0
    // if value after is > 255 take 255
    for (int j = 0; j < mSrc.rows; j++) {
        for (int i = 0; i < mSrc.cols; i++) {
            int values[3];
            for(int k = 0; k < 3; k++){
                values[k] = (int) (mBGR.at<cv::Vec3b>(j,i)[k] * (this->bgrPercent[k]/100. + 1.));
                // deal with overflow
                if(values[k] < 0){ values[k] = 0; }
                if(values[k] > 255){ values[k] = 255; }
                mBGR.at<cv::Vec3b>(j,i)[k] = (unsigned char) values[k];
            }
        }
    }
    mBGR.convertTo(mDst, mSrc.type());
}

/*!
 * Adds gaussian noise to an image.
 * Can be used to add color noise and dark current.
 * @param mSrc source image
 * @param mDst destination of modified image
 * @param Mean mean pixel value
 * @param StdDev standard deviation of pixel value
 *
 */
void Camera::addGaussianNoise(const cv::Mat& mSrc, cv::Mat &mDst, double Mean, double StdDev)
{
    cv::Mat mSrc_16SC;
    //CV_16SC3 means signed 16 bit shorts three channels
    cv::Mat mGaussian_noise = cv::Mat(mSrc.size(), CV_16SC3);
    /*!  Generates random noise in a normal distribution.*/
    randn(mGaussian_noise, cv::Scalar::all(Mean), cv::Scalar::all(StdDev));

    mSrc.convertTo(mSrc_16SC, CV_16SC3);
    /*!  Adds the noise layer to the image layer with a weighted add to prevent overflowing the pixels.*/
    addWeighted(mSrc_16SC, 1.0, mGaussian_noise, 1.0, 0.0, mSrc_16SC);
    mSrc_16SC.convertTo(mDst,mSrc.type());
}

/*!
 * Adds dead and hot pixels to an image.
 * @param mSrc source image
 * @param mDst destination of modified image
 * @param pa probability of dead pixels
 * @param pb probability of hot pixels
 *
 */
void Camera::addSaltPepper(const cv::Mat& mSrc, cv::Mat &mDst, float pa, float pb){
    /*! These lines will make the hot and dead pixels different every time.*/
    // uint64 initValue = time(0);
    // RNG rng(initValue);

    /*! This line makes the hot and dead pixels the same frame to frame.*/
    cv::RNG rng;

    /*!  Determines the amount of hot/dead pixels based on the input probabilities.*/
    int amount1 = (int) (mSrc.rows * mSrc.cols * pa);
    int amount2 = (int) (mSrc.rows * mSrc.cols * pb);

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

    /*!  Chooses random pixels to be stuck or dead in the amount calculated previously.*/
    for(int counter = 0; counter < amount1; counter++){
        mSaltPepper.at<cv::Vec3b>(rng.uniform(0, mSaltPepper.rows),
                                  rng.uniform(0, mSaltPepper.cols)) = black;
    }

    for(int counter = 0; counter < amount2; counter++){
        mSaltPepper.at<cv::Vec3b>(rng.uniform(0, mSaltPepper.rows),
                                  rng.uniform(0, mSaltPepper.cols)) = white;
    }

    mSaltPepper.convertTo(mDst, mSrc.type());
}

/*!
 * Adds a cosmic ray to an image. The ray is modelled as a single pixel wide white line.
 * @param mSrc source image
 * @param mDst destination of modified image
 * @param probThreshhold probability of getting a ray each frame
 * @param randOffset if adding multiple rays pass in the number of each to guarantee a random ray
 * @param maxSize max length of cosmic ray
 *
 */
void Camera::addCosmicRay(const cv::Mat& mSrc, cv::Mat &mDst, float probThreshhold, double randOffset, int maxSize){
    /*! Uses the current sim time and the random offset to ensure a different ray every time.*/
    uint64 initValue = this->localCurrentSimNanos;
    cv::RNG rng((uint64) (initValue + time(0) + randOffset));

    float prob = (float) (rng.uniform(0.0, 1.0));
    if (prob > probThreshhold) {
        cv::Mat mCosmic = cv::Mat(mSrc.size(), mSrc.type());
        mSrc.convertTo(mCosmic, mSrc.type());

        /*!  Chooses a random point on the image. Then chooses a second random point within 50 pixels
         * in either direction.*/
        int x = rng.uniform(0, mCosmic.rows);
        int y = rng.uniform(0, mCosmic.cols);
        int deltax = rng.uniform(-maxSize/2, maxSize/2);
        int deltay = rng.uniform(-maxSize/2, maxSize/2);

        cv::Point p1 = cv::Point(x, y);
        cv::Point p2 = cv::Point(x + deltax, y + deltay);
        line(mCosmic, p1, p2, cv::Scalar(255, 255, 255), 1, cv::LINE_8);

        mCosmic.convertTo(mDst, mSrc.type());
    }
}

/*!
 * Adds a user specified number of cosmic rays to an image. Rays are modelled as a single pixel wide white line.
 * The maximum length is hard-coded as 50 pixels.
 * @param mSrc source image
 * @param mDst destination of modified image
 * @param num number of cosmic rays to be added
 *
 */
void Camera::addCosmicRayBurst(const cv::Mat& mSrc, cv::Mat &mDst, double num){
    cv::Mat mCosmic = cv::Mat(mSrc.size(), mSrc.type());
    mSrc.convertTo(mCosmic, mSrc.type());
    for(int i = 0; i < std::round(num); i++){
        /*! Threshold defined such that 1 provides a 1/50 chance of getting a ray, and 10 will get about
         * 5 rays per image. Currently length is limited to 50 pixels*/
        this->addCosmicRay(mCosmic,
                           mCosmic,
                           (float) (1/(std::pow(num,2))),
                           i+1,
                           50);
    }
    mCosmic.convertTo(mDst, mSrc.type());
}

/*!
 * Applies all of the various perturbations to an image with user specified levels.
 * Each parameter is a double scaling actor. A parameter of 0 will result in the respective perturbation not
 * being applied.
 * @param mSource source image
 * @param mDst destination of modified image
 *
 */
void Camera::applyFilters(cv::Mat &mSource, cv::Mat &mDst){

    cv::Mat mFilters(mSource.size(), mSource.type());
    mSource.convertTo(mFilters, mSource.type());

    if (this->gaussian > 0){
        float scale = 2;
        this->addGaussianNoise(mFilters, mFilters, 0, this->gaussian * scale);
        cv::threshold(mFilters, mFilters, this->gaussian*6, 255, cv::THRESH_TOZERO);
    }
    if(this->blurParam > 0){
        int blurSize = (int) std::round(this->blurParam);
        if (blurSize%2 == 0){ blurSize+=1; }
        cv::blur(mFilters,
                 mFilters,
                 cv::Size(blurSize, blurSize),
                 cv::Point(-1 , -1));
    }
    if(this->darkCurrent > 0){
        float scale = 15;
        this->addGaussianNoise(mFilters, mFilters, this->darkCurrent * scale, 0.0);
    }
    if (this->hsv.cwiseAbs().sum() > 0.00001) {
        this->hsvAdjust(mFilters, mFilters);
    }
    if (this->bgrPercent.cwiseAbs().sum() != 0) {
        this->bgrAdjustPercent(mFilters, mFilters);
    }
    if (this->saltPepper > 0){
        float scale = 0.00002f;
        this->addSaltPepper(mFilters,
                            mFilters,
                            (float) (this->saltPepper * scale),
                            (float) (this->saltPepper * scale));
    }
    if(this->cosmicRays > 0){
        this->addCosmicRayBurst(mFilters, mFilters, std::round(this->cosmicRays));
    }

    mFilters.convertTo(mDst, mSource.type());
}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle
 * Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding.

 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void Camera::UpdateState(uint64_t currentSimNanos)
{
    this->localCurrentSimNanos = currentSimNanos;
    std::string localPath;
    CameraImageMsgPayload imageBuffer = {};
    CameraImageMsgPayload imageOut;
    CameraConfigMsgPayload cameraMsg;

    /* zero output messages */
    imageOut = this->imageOutMsg.zeroMsgPayload;
    cameraMsg = this->cameraConfigOutMsg.zeroMsgPayload;

    /*! - Populate the camera message */
    cameraMsg.cameraID = this->cameraID;
    strcpy(cameraMsg.parentName, this->parentName);
    cameraMsg.resolution[0] = this->resolution[0];
    cameraMsg.resolution[1] = this->resolution[1];
    cameraMsg.renderRate = this->renderRate;
    cameraMsg.fieldOfView = this->fieldOfView;
    cameraMsg.isOn = this->cameraIsOn;
    v3Copy(this->cameraPos_B, cameraMsg.cameraPos_B);
    v3Copy(this->sigma_CB, cameraMsg.sigma_CB);
    strcpy(cameraMsg.skyBox, this->skyBox);
    cameraMsg.postProcessingOn = this->postProcessingOn;
    cameraMsg.ppAperture = this->ppAperture;
    cameraMsg.ppFocalLength = this->ppFocalLength;
    cameraMsg.ppFocusDistance = this->ppFocusDistance;
    cameraMsg.ppMaxBlurSize = this->ppMaxBlurSize;

    /*! - Update the camera config data no matter if an image is present*/
    this->cameraConfigOutMsg.write(&cameraMsg, this->moduleID, currentSimNanos);

    cv::Mat imageCV;
    cv::Mat blurred;
    if (this->saveDir != ""){
        localPath = this->saveDir + std::to_string(currentSimNanos*1E-9) + ".png";
    }
    /*! - Read in the bitmap*/
    if(this->imageInMsg.isLinked())
    {
        imageBuffer = this->imageInMsg();
        this->sensorTimeTag = this->imageInMsg.timeWritten();
    }
    /* Added for debugging purposes*/
    if (!this->filename.empty()){
        imageCV = imread(this->filename, cv::IMREAD_COLOR);
        this->applyFilters(imageCV, blurred);
        if (this->saveImages == 1){
            if (!cv::imwrite(localPath, blurred)) {
                bskLogger.bskLog(BSK_WARNING, "camera: wasn't able to save the camera module image" );
            }
        }
    }
    else if(imageBuffer.valid == 1 && imageBuffer.timeTag >= currentSimNanos){
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer((char*)imageBuffer.imagePointer,
                                                (char*)imageBuffer.imagePointer + imageBuffer.imageBufferLength);
        imageCV = cv::imdecode(vectorBuffer, cv::IMREAD_COLOR);

        this->applyFilters(imageCV, blurred);
        if (this->saveImages == 1){
            if (!cv::imwrite(localPath, blurred)) {
                bskLogger.bskLog(BSK_WARNING, "camera: wasn't able to save the camera module image" );
            }
        }
        /*! If the permanent image buffer is not populated, it will be equal to null*/
        if (this->pointImageOut != nullptr) {
            free(this->pointImageOut);
            this->pointImageOut = nullptr;
        }
        /*! - Encode the cv mat into a png for the future modules to decode it the same way */
        std::vector<unsigned char> buf;
        std::vector<int> compression;
        compression.push_back(0);
        cv::imencode(".png", blurred, buf, compression);
        /*! - Output the saved image */
        imageOut.valid = 1;
        imageOut.timeTag = imageBuffer.timeTag;
        imageOut.cameraID = imageBuffer.cameraID;
        imageOut.imageType = imageBuffer.imageType;
        imageOut.imageBufferLength = (int32_t)buf.size();
        this->pointImageOut = malloc(imageOut.imageBufferLength*sizeof(char));
        memcpy(this->pointImageOut, &buf[0], imageOut.imageBufferLength*sizeof(char));
        imageOut.imagePointer = this->pointImageOut;

        this->imageOutMsg.write(&imageOut, this->moduleID, currentSimNanos);
    }
    else{
        /*! - If no image is present, write zeros in message */
        this->imageOutMsg.write(&imageOut, this->moduleID, currentSimNanos);
    }
}
