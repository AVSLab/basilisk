/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#ifndef POINTCLOUDTRIANGULATION_H
#define POINTCLOUDTRIANGULATION_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefCpp/DirectionOfMotionMsgPayload.h"
#include "architecture/msgPayloadDefCpp/PairedKeyPointsMsgPayload.h"
#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
#include "architecture/msgPayloadDefCpp/PointCloudMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/linearAlgebra.h"
#include <vector>
#include <array>


/*! @brief This module triangulates a camera position from a point cloud
 */
class PointCloudTriangulation: public SysModel {
public:
    PointCloudTriangulation();
    ~PointCloudTriangulation();

    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;

    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg; //!< ephemeris input message
    ReadFunctor<NavTransMsgPayload> navTransInMsg; //!< translational navigation input message
    ReadFunctor<DirectionOfMotionMsgPayload> directionOfMotionInMsg; //!< direction of motion input message
    ReadFunctor<PairedKeyPointsMsgPayload> keyPointsInMsg; //!< key (feature) points input message
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg; //!< camera configuration input message
    Message<PointCloudMsgPayload> pointCloudOutMsg; //!< point cloud output message

    BSKLogger bskLogger; //!< -- BSK Logging

    //!< number of times (time steps) the module should use the ephemeris message before using the navigation message
    int numberTimeStepsInitialPhase = 5;

private:
    void readMessages();
    void writeMessages(uint64_t currentSimNanos);
    Eigen::Vector3d triangulation(std::vector<Eigen::Vector3d> knownLocations,
                                  std::vector<Eigen::Vector2d> imagePoints,
                                  const Eigen::Matrix3d& cameraCalibrationInverse,
                                  std::vector<Eigen::Matrix3d> dcmCamera) const;

    int numberTimesCalled{}; //!< number of times (time steps) the module has been called
    bool initialPhase{}; //!< indicates if the module is still in the initial phase (using ephemeris message)
    double vScaleFactor{}; //!< velocity scale factor to be applied to direction of motion
    Eigen::Vector3d v_C1_hat; //!< [-] camera direction of motion
    int numberKeyPoints{}; //!< [-] number of key points (features)
    std::vector<Eigen::Vector2d> keyPoints1; //!< [-] key point pixel coordinates for 1st camera position
    uint64_t timeTag1{}; //!< [ns] vehicle time-tag associated with images for 1st camera position
    std::vector<Eigen::Vector2d> keyPoints2; //!< [-] key point pixel coordinates for 2nd camera position
    uint64_t timeTag2{}; //!< [ns] vehicle time-tag associated with images for 2nd camera position
    Eigen::Matrix3d cameraCalibrationMatrixInverse; //!< [-] inverse of camera calibration matrix
    Eigen::Matrix3d dcm_C2C1; //!< [-] direction cosine matrix (DCM) from camera frame C1 to camera frame C2
    bool valid; //!< [-] validity flag for point cloud triangulation
    int pointCloudSize{}; //!< [-] number of points in point cloud
    std::vector<Eigen::Vector3d> measuredPointCloud{}; //!< [-] measured point cloud
};

#endif
