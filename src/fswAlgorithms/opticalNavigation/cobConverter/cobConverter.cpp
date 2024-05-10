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

#include "cobConverter.h"

CobConverter::CobConverter() = default;

CobConverter::~CobConverter() = default;

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states
 * between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CobConverter::Reset(uint64_t CurrentSimNanos)
{
    // check that the required message has not been connected
    if (!this->opnavCOBInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CobConverter.opnavCOBInMsg wasn't connected.");
    }
    if (!this->cameraConfigInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CobConverter.cameraConfigInMsg wasn't connected.");
    }
    if (!this->navAttInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CobConverter.navAttInMsg wasn't connected.");
    }
}

/*! During an update, this module transforms pixel values for the center of brightness into a unit vector
 * direction in several frames (inertial, Camera, and Body).
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CobConverter::UpdateState(uint64_t CurrentSimNanos)
{
    CameraConfigMsgPayload cameraSpecs = this->cameraConfigInMsg();
    OpNavCOBMsgPayload cobMsgBuffer = this->opnavCOBInMsg();
    NavAttMsgPayload navAttBuffer = this->navAttInMsg();

    OpNavUnitVecMsgPayload uVecCOBMsgBuffer;
    uVecCOBMsgBuffer = this->opnavUnitVecCOBOutMsg.zeroMsgPayload;

    if (cobMsgBuffer.valid && cobMsgBuffer.pixelsFound != 0){
        /*! - Extract rotations from relevant messages */
        Eigen::Matrix3d dcm_CB, dcm_BN, dcm_NC;
        double CB[3][3], BN[3][3];
        MRP2C(cameraSpecs.sigma_CB, CB);
        dcm_CB = c2DArray2EigenMatrix3d(CB);
        MRP2C(navAttBuffer.sigma_BN, BN);
        dcm_BN = c2DArray2EigenMatrix3d(BN);

        dcm_NC = dcm_BN.transpose() * dcm_CB.transpose();

        /*! - camera parameters */
        double alpha = 0;
        double fieldOfView = cameraSpecs.fieldOfView;
        double resolutionX = cameraSpecs.resolution[0];
        double resolutionY = cameraSpecs.resolution[1];
        double pX = 2.*tan(fieldOfView/2.0);
        double pY = 2.*tan(fieldOfView*resolutionY/resolutionX/2.0);
        double dX = resolutionX/pX;
        double dY = resolutionY/pY;
        double up = resolutionX/2;
        double vp = resolutionY/2;
        double X = 1/dX;
        double Y = 1/dY;
        /*! - build inverse K^-1 of camera calibration matrix K */
        Eigen::Matrix3d cameraCalibrationMatrixInverse;
        cameraCalibrationMatrixInverse << 1./dX, -alpha/(dX*dY), (alpha*vp - dY*up)/(dX*dY),
                                          0., 1./dY, -vp/dY,
                                          0., 0., 1.;

        /*! - Center of Brightness in pixel space */
        Eigen::Vector3d centerOfBrightness;
        centerOfBrightness[0] = cobMsgBuffer.centerOfBrightness[0];
        centerOfBrightness[1] = cobMsgBuffer.centerOfBrightness[1];
        centerOfBrightness[2] = 1.0;

        /*! - Get the heading in the image plane */
        Eigen::Vector3d rhat_COB_C = cameraCalibrationMatrixInverse * centerOfBrightness;

        /*! - Retrieve the vector from target to camera and normalize */
        rhat_COB_C *= - 1;
        rhat_COB_C.normalize();

        /*! - Rotate the vector into frames of interest */
        Eigen::Vector3d rhat_COB_N = dcm_NC * rhat_COB_C;
        Eigen::Vector3d rhat_COB_B = dcm_CB.transpose() * rhat_COB_C;

        /*! - Define diagonal terms of the covariance */
        Eigen::Matrix3d covar_C;
        covar_C.setZero();
        covar_C(0,0) = pow(X,2);
        covar_C(1,1) = pow(Y,2);
        covar_C(2,2) = 1;
        /*! - define and rotate covariance using number of pixels found */
        double scaleFactor;
        scaleFactor = sqrt(cobMsgBuffer.pixelsFound)/(6.28); // division by 2pi
        covar_C *= 1./scaleFactor;
        Eigen::Matrix3d covar_N, covar_B;
        covar_N = dcm_NC * covar_C * dcm_NC.transpose();
        covar_B = dcm_CB.transpose() * covar_C * dcm_CB;

        /*! - output messages */
        eigenMatrix3d2CArray(covar_N, uVecCOBMsgBuffer.covar_N);
        eigenMatrix3d2CArray(covar_C, uVecCOBMsgBuffer.covar_C);
        eigenMatrix3d2CArray(covar_B, uVecCOBMsgBuffer.covar_B);
        eigenVector3d2CArray(rhat_COB_N, uVecCOBMsgBuffer.rhat_BN_N);
        eigenVector3d2CArray(rhat_COB_C, uVecCOBMsgBuffer.rhat_BN_C);
        eigenVector3d2CArray(rhat_COB_B, uVecCOBMsgBuffer.rhat_BN_B);
        uVecCOBMsgBuffer.timeTag = (double) cobMsgBuffer.timeTag * NANO2SEC;
        uVecCOBMsgBuffer.valid = true;
    }

    this->opnavUnitVecCOBOutMsg.write(&uVecCOBMsgBuffer, this->moduleID, CurrentSimNanos);

}
