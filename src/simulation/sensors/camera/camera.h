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

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <stdint.h>
#include <math.h>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"

#include "architecture/msgPayloadDefC/CameraImageMsgPayload.h"
#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief visual camera class */
class Camera: public SysModel {
public:
    Camera();
    ~Camera();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    void HSVAdjust(const cv::Mat, cv::Mat &mDst);
    void BGRAdjustPercent(const cv::Mat, cv::Mat &mDst);
    void AddGaussianNoise(const cv::Mat, cv::Mat &mDst, double, double);
    void AddSaltPepper(const cv::Mat, cv::Mat &mDst, float, float);
    void AddCosmicRay(const cv::Mat, cv::Mat &mDst, float, double, int);
    void AddCosmicRayBurst(const cv::Mat, cv::Mat &mDst, double);
    void ApplyFilters(cv::Mat, cv::Mat &mDst, double gaussian, double darkCurrent, double saltPepper, double cosmicRays, double blurparam);
public:
    std::string filename;                //!< Filename for module to read an image directly
    ReadFunctor<CameraImageMsgPayload> imageInMsg;      //!< camera image input message
    Message<CameraImageMsgPayload> imageOutMsg;         //!< camera image output message
    Message<CameraConfigMsgPayload> cameraConfigOutMsg; //!< The name of the CameraConfigMsg output message
    std::string saveDir;                 //!< The name of the directory to save images
    uint64_t sensorTimeTag;              //!< [ns] Current time tag for sensor out
    int32_t saveImages;                  //!< [-] 1 to save images to file for debugging
    
    /*! Camera parameters */
    char parentName[MAX_STRING_LENGTH];  //!< [-] Name of the parent body to which the camera should be attached
    int cameraIsOn; //!< [-] Is the camera currently taking images
    int cameraID; //!< [-] Is the camera currently taking images
    int resolution[2];         //!< [-] Camera resolution, width/height in pixels (pixelWidth/pixelHeight in Unity) in pixels
    uint64_t renderRate;       //!< [ns] Frame time interval at which to capture images in units of nanosecond
    double fieldOfView;        //!< [r] camera y-axis field of view edge-to-edge
    double cameraPos_B[3];     //!< [m] Camera position in body frame
    double sigma_CB[3];        //!< [-] MRP defining the orientation of the camera frame relative to the body frame
    char skyBox[MAX_STRING_LENGTH]; //!< [-] name of skyboz in use
    
    /*! Noise paramters */
    double gaussian;        //!< Gaussian noise level
    double darkCurrent;    //!< Dark current intensity
    double saltPepper;    //!< Stuck and Dark pixels probability
    double cosmicRays;        //!< Random cosmic rays (number)
    double blurParam;        //!< Blur over image in pixels
    std::vector<double> hsv;    //!< (double) HSV color correction, H (-pi/pi) hue shift, S and V are percent multipliers
    std::vector<int> bgrPercent; //!< (int) BGR color correction values as percent

    BSKLogger bskLogger;                      //!< -- BSK Logging
private:
    uint64_t CurrentSimNanos;
    void* pointImageOut;      //!< void pointer for image memory passing
};

/* @} */

#endif

