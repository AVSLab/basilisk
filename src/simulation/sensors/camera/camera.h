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
    
    void UpdateState(uint64_t currentSimNanos) override;
    void Reset(uint64_t currentSimNanos) override;
    void hsvAdjust(const cv::Mat&, cv::Mat &mDst);
    void bgrAdjustPercent(const cv::Mat&, cv::Mat &mDst);
    void addGaussianNoise(const cv::Mat&, cv::Mat &mDst, double, double);
    void addSaltPepper(const cv::Mat&, cv::Mat &mDst, float, float);
    void addCosmicRay(const cv::Mat&, cv::Mat &mDst, float, double, int);
    void addCosmicRayBurst(const cv::Mat&, cv::Mat &mDst, double);
    void applyFilters(cv::Mat &mSource, cv::Mat &mDst);

public:
    std::string filename{};                //!< Filename for module to read an image directly
    ReadFunctor<CameraImageMsgPayload> imageInMsg;      //!< camera image input message
    Message<CameraImageMsgPayload> imageOutMsg;         //!< camera image output message
    Message<CameraConfigMsgPayload> cameraConfigOutMsg; //!< The name of the CameraConfigMsg output message
    std::string saveDir{};                 //!< The name of the directory to save images
    uint64_t sensorTimeTag{};              //!< [ns] Current time tag for sensor out
    int32_t saveImages{};                  //!< [-] 1 to save images to file for debugging
    
    /*! Camera parameters */
    char parentName[MAX_STRING_LENGTH]{};  //!< [-] Name of the parent body to which the camera should be attached
    int cameraIsOn{}; //!< [-] Is the camera currently taking images
    int cameraID{1}; //!< [-] Is the camera currently taking images
    int resolution[2]{512, 512};         //!< [-] Camera resolution, width/height in pixels (pixelWidth/pixelHeight in Unity) in pixels
    uint64_t renderRate{};       //!< [ns] Frame time interval at which to capture images in units of nanosecond
    double fieldOfView{0.7};       //!< [r] camera y-axis field of view edge-to-edge
    double cameraPos_B[3]{};     //!< [m] Camera position in body frame
    double sigma_CB[3]{};        //!< [-] MRP defining the orientation of the camera frame relative to the body frame
    char skyBox[MAX_STRING_LENGTH]{"black"}; //!< [-] name of skyboz in use
    int postProcessingOn{};       //!< Enable post-processing of camera image. Value of 0 (protobuffer default) to use viz default which is off, -1 for false, 1 for true
    double ppFocusDistance{};     //!< Distance to the point of focus, minimum value of 0.1, Value of 0 to turn off this parameter entirely.
    double ppAperture{};          //!<  Ratio of the aperture (known as f-stop or f-number). The smaller the value is, the shallower the depth of field is. Valid Setting Range: 0.05 to 32. Value of 0 to turn off this parameter entirely.
    double ppFocalLength{};       //!< [m] Valid setting range: 0.001m to 0.3m. Value of 0 to turn off this parameter entirely.
    int ppMaxBlurSize{};          //!< Convolution kernel size of the bokeh filter, which determines the maximum radius of bokeh. It also affects the performance (the larger the kernel is, the longer the GPU time is required). Depth textures Value of 1 for Small, 2 for Medium, 3 for Large, 4 for Extra Large. Value of 0 to turn off this parameter entirely.

    /*! Noise paramters */
    double gaussian{};        //!< Gaussian noise level
    double darkCurrent{};    //!< Dark current intensity
    double saltPepper{};    //!< Stuck and Dark pixels probability
    double cosmicRays{};        //!< Random cosmic rays (number)
    double blurParam{};        //!< Blur over image in pixels
    Eigen::Vector3d hsv{Eigen::Vector3d::Zero()};    //!< (double) HSV color correction, H (-pi/pi) hue shift, S and V are percent multipliers
    Eigen::Vector3d bgrPercent{Eigen::Vector3d::Zero()}; //!< (int) BGR color correction values as percent

    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    uint64_t localCurrentSimNanos{};
    void* pointImageOut{nullptr};      //!< void pointer for image memory passing
};

/* @} */
#endif
