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
#include <Eigen/Dense>
#include "architecture/messaging/system_messaging.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "../simulation/simFswInterfaceMessages/cameraImageMsg.h"
#include "../simulation/simFswInterfaceMessages/cameraConfigMsg.h"
#include "../simulation/_GeneralModuleFiles/sys_model.h"
#include "../simulation/utilities/avsEigenMRP.h"



/*! \addtogroup SimModelGroup

 @brief Module reads in a message containing a pointer to an image and writes out the circles that are found in the image by OpenCV's HoughCricle Transform.

 The module
 [PDF Description](Basilisk-camera-20191003.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.

 @{
 */

class Camera: public SysModel {
public:
    Camera();
    ~Camera();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    void AddGaussianNoise(const cv::Mat, cv::Mat &mDst, double, double);
    void AddSaltPepper(const cv::Mat, cv::Mat &mDst, float, float);
    void AddCosmicRay(const cv::Mat, cv::Mat &mDst, float, double, int);
    void AddCosmicRayBurst(const cv::Mat, cv::Mat &mDst, double);
    void ApplyFilters(cv::Mat, cv::Mat &mDst, double gaussian, double darkCurrent, double saltPepper, double cosmicRays, double blurparam);
public:
    std::string filename;                //!< Filename for module to read an image directly
    std::string imageInMsgName;          //!< The name of the ImageFswMsg output message
    std::string imageOutMsgName;          //!< The name of the ImageFswMsg output message
    std::string cameraOutMsgName;          //!< The name of the ImageFswMsg output message
    std::string saveDir;                //!< The name of the directory to save images
    uint64_t sensorTimeTag;              //!< [ns] Current time tag for sensor out
    int32_t saveImages;                  //!< [-] 1 to save images to file for debugging
    
    /*! Camera parameters */
    char parentName[MAX_MESSAGE_SIZE];  //!< [-] Name of the parent body to which the camera should be attached
    int cameraIsOn; //!< [-] Is the camera currently taking images
    int cameraID; //!< [-] Is the camera currently taking images
    double fieldOfView;        //!< [rad]   Camera Field of View */
    int resolution[2];         //!< [-] Camera resolution, width/height in pixels (pixelWidth/pixelHeight in Unity) in pixels*/
    uint64_t renderRate;       //!< [ns] Frame time interval at which to capture images in units of nanosecond */
    double focalLength;        //!< [m] Camera Focal Length in meters*/
    double sensorSize[2];      //!< [m] Size of the camera sensor-paired with resolution gives you pixel size in mm*/
    double cameraPos_B[3];     //!< [m] Camera position in body frame */
    double sigma_CB[3];        //!< [-] MRP defining the orientation of the camera frame relative to the body frame */
    char skyBox[MAX_MESSAGE_SIZE]; //!< [-] name of skyboz in use */
    
    /*! Noise paramters */
    double gaussian;        //!< Gaussian  */
    double darkCurrent;    //!< Dark current intensity  */
    double saltPepper;    //!< Stuck and Dark pixels */
    double cosmicRays;        //!< Random cosmic rays (number)*/
    double blurParam;        //!< Blur over image in pixels */
private:
    uint64_t OutputBufferCount;          //!< [-] Count on the number of output message buffers
    int32_t imageInMsgID;                //!< ID for the outgoing message
    int32_t imageOutMsgID;                //!< ID for the outgoing message
    int32_t cameraOutID;                //!< ID for the outgoing message
    uint64_t CurrentSimNanos;
    void* pointImageOut;      //!< void pointer for image memory passing
};

/* @} */

#endif

