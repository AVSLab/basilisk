/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric Space Physics, University of Colorado Boulder

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


#ifndef VIS_ODOMETRY_H
#define VIS_ODOMETRY_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefCpp/PairedKeyPointsMsgPayload.h"
#include "architecture/msgPayloadDefCpp/DirectionOfMotionMsgPayload.h"
#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"

#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/astroConstants.h"
#include <vector>

/*! @brief Odometry module which reads key-point pairs and outputs a spacecraft direction of motion
 */
class VisualOdometry: public SysModel {
public:
    VisualOdometry();
    ~VisualOdometry();

    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;

    ReadFunctor<PairedKeyPointsMsgPayload> keyPointPairInMsg;                      //!< translational navigation input message
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg;
    Message<DirectionOfMotionMsgPayload> dirOfMotionMsgOutput;             //!< lambert problem output message

    BSKLogger bskLogger;                                                //!< -- BSK Logging

    double errorTolerance = 1E-5;
    double sigma_uv = 0.01;
    double deltaKsi_tilde = 0.25;
    int m_max = 10;

private:
    bool readMessages();
    void writeMessages(Eigen::Vector3d sPrime, Eigen::Matrix3d covar, uint64_t currentSimNanos);
    void computeX();
    void computeRinv(Eigen::Vector3d& sPrime);
    void computeHTH();
    Eigen::Vector3d svdLastColumn(Eigen::Matrix3d& A) const;
    void computeKsis(int i);
    void computeGammas(int i);
    void computeSampsonPartials(int i);
    void computeAframeDCM();
    void computeCameraMatrix();
    int cheiralityTest(Eigen::Vector3d sPrime);
    Eigen::Matrix3d computeCovariance() const;

    PairedKeyPointsMsgPayload keyPointBuffer;
    CameraConfigMsgPayload cameraBuffer;
    DirectionOfMotionMsgPayload dirMotionBuffer;

    int numberFeatures;

    Eigen::Matrix3d cameraCalibrationMatrixInverse;
    Eigen::MatrixXd oldFeaturesPixels;
    Eigen::MatrixXd newFeaturesPixels;

    Eigen::Matrix3d HtransposeH;
    Eigen::Matrix3d Rinv;
    Eigen::Matrix3d X;
    Eigen::MatrixXd pixelCoord_firstImage;
    Eigen::MatrixXd pixelCoord_secondImage;
    std::vector<Eigen::Matrix3d> dhdu_firstImage;
    std::vector<Eigen::Matrix3d> dhdu_secondImage;
    std::vector<Eigen::Matrix3d> gammas;
    std::vector<Eigen::Matrix3d> ksis;
    std::vector<double> sTKsiS;
    std::vector<double> sTGammaS;
    Eigen::Matrix3d CkCkmin1;
    Eigen::Matrix3d R_uv;

};

#endif
