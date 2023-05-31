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

#ifndef _OPTICAL_FLOW_H_
#define _OPTICAL_FLOW_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "architecture/messaging/messaging.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"

#include "architecture/msgPayloadDefC/CameraImageMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefCpp/PairedKeyPointsMsgPayload.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief feature tracking module */
class OpticalFlow: public SysModel {
public:
    OpticalFlow();
    ~OpticalFlow();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentSimNanos);
    void makeMask(cv::Mat const & inputBWImage, cv::Mat mask) const;

private:
    cv::Mat firstImage;
    cv::Mat secondImage;
    bool firstImagePresent = false;
    bool secondImagePresent = false;
    double firstSpacecraftAttitude[3];
    double firstTargetEphemAttitude[3];
    uint64_t firstTimeTag;
    std::vector<cv::Vec2f> secondFeatures;
    std::vector<cv::Vec2f> firstFeatures;

public:
    std::string filename = "";  //!< Filename for module to read an image directly

    Message<PairedKeyPointsMsgPayload> keyPointsMsg;  //!< The name of the output message containing key points
    ReadFunctor<CameraImageMsgPayload> imageInMsg;  //!< The name of the camera output message containing images
    ReadFunctor<NavAttMsgPayload> attitudeMsg;  //!< The name of the input attitude information
    ReadFunctor<EphemerisMsgPayload> ephemerisMsg;  //!< The name of the input central target ephemeris data
    uint64_t sensorTimeTag; //!< [ns] Current time tag for sensor out

    double minTimeBetweenPairs = 1; //!< [s] Minimum time between pairs of images
    bool slidingWindowImages = false; //!< [bool] Minimum time between pairs of images

    /*! OpenCV specific arguments needed for Shi-Tomasi "good features to track" */
    int32_t maxNumberFeatures = 100;
    double qualityLevel = 0.3;
    int32_t minumumFeatureDistance = 5;
    int32_t blockSize = 7;

    /*! OpenCV specific arguments needed for OpticalFlow */
    int32_t criteriaMaxCount = 10;
    double criteriaEpsilon = 0.03;
    int32_t flowSearchSize = 10;
    int32_t flowMaxLevel = 2;

    /*! OpenCV specific arguments needed for masking */
    int32_t thresholdMask = 20;
    int32_t dilutionMask = 20;

    BSKLogger bskLogger;
};

#endif
