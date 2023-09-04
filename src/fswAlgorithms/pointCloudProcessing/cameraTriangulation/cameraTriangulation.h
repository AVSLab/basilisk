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

#ifndef CAMERATRIANGULATION_H
#define CAMERATRIANGULATION_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefCpp/PointCloudMsgPayload.h"
#include "architecture/msgPayloadDefCpp/PairedKeyPointsMsgPayload.h"
#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CameraLocalizationMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/linearAlgebra.h"
#include <vector>
#include <array>


/*! @brief This module triangulates a camera position from a point cloud
 */
class CameraTriangulation: public SysModel {
public:
    CameraTriangulation();
    ~CameraTriangulation();

    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;

    ReadFunctor<PointCloudMsgPayload> pointCloudInMsg; //!< point cloud input message
    ReadFunctor<PairedKeyPointsMsgPayload> keyPointsInMsg; //!< key (feature) points input message
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg; //!< camera configuration input message
    Message<CameraLocalizationMsgPayload> cameraLocationOutMsg; //!< estimated camera location output message

    BSKLogger bskLogger; //!< -- BSK Logging

private:
    void readMessages();
    void writeMessages(uint64_t currentSimNanos);
    Eigen::Vector3d triangulation(Eigen::MatrixXd knownLocations,
                                  std::vector<Eigen::Vector2d> imagePoints,
                                  const Eigen::Matrix3d& cameraCalibrationInverse,
                                  std::vector<Eigen::Matrix3d> dcmCamera) const;

    Eigen::MatrixXd pointCloud{}; //!< [-] point cloud
    std::vector<Eigen::Vector2d> keyPoints{}; //!< [-] key point pixel coordinates
    Eigen::Matrix3d cameraCalibrationMatrixInverse{}; //!< [-] inverse of camera calibration matrix
    Eigen::Matrix3d dcm_CN{}; //!< [-] direction cosine matrix (DCM) from inertial frame N to camera frame C
    Eigen::MRPd sigma_BN{}; //!< [-] MRP orientation of spacecraft body B w.r.t. inertial frame N when images where taken
    int64_t cameraID{}; //!< [-] ID of the camera that took the images
    uint64_t timeTag{}; //!< [ns] vehicle time-tag associated with images
    bool validInputs; //!< [-] validity flag for triangulation (true if information from input messages is as expected)
    Eigen::Vector3d estimatedCameraLocation{}; //!< [m] triangulated camera location in inertial frame
};

#endif
