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

#ifndef _INITSICP_H_
#define _INITSICP_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "fswAlgorithms/pointCloudProcessing/SICP/sicpDefinitions.h"
#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDefCpp/SICPMsgPayload.h"
#include "architecture/msgPayloadDefCpp/PointCloudMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief Scaling iterative Closest Point Algorithm */
class InitializeICP: public SysModel {
public:
    InitializeICP();
    ~InitializeICP();
    
    void UpdateState(uint64_t CurrentSimNanos) override;
    void Reset(uint64_t CurrentSimNanos) override;

    ReadFunctor<SICPMsgPayload> inputSICPData;  //!< The output algorithm data
    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg; //!< ephemeris input message
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg; //!< camera configuration input message
    ReadFunctor<PointCloudMsgPayload> inputMeasuredPointCloud;          //!< The input measured data
    Message<PointCloudMsgPayload> measuredPointCloud;  //!< The output fitted point cloud
    Message<SICPMsgPayload> initializeSICPMsg;  //!< The output algorithm data

    BSKLogger bskLogger;                //!< -- BSK Logging

    double maxTimeBetweenMeasurements = 600;
    bool normalizeMeasuredCloud = false;

private:
    void normalizePointCloud();
    void setInitialConditions(uint64_t CurrentSimNanos);
    void writeOutputMessages(uint64_t CurrentSimNanos);

    PointCloudMsgPayload normalizedCloudBuffer;
    SICPMsgPayload outputIcpBuffer;
    double averageNorm = 0;
    Eigen::VectorXd averagePoint;
    Eigen::VectorXd referencePoint;
    bool initialPhase = true;

    //!< Logged results that will be used when this module is called again
    Eigen::MatrixXd R_logged = Eigen::MatrixXd::Identity(POINT_DIM, POINT_DIM);
    Eigen::MatrixXd t_logged = Eigen::VectorXd::Zero(POINT_DIM);
    double s_logged = 1;
    uint64_t previousTimeTag = 0;

};

#endif
