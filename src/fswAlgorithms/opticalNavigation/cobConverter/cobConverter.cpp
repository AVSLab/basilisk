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
    OpNavUnitVecMsgPayload uVecMsgBuffer = this->opnavUnitVecOutMsg.zeroMsgPayload;
    NavAttMsgPayload navAttBuffer = this->navAttInMsg();
    
    if (cobMsgBuffer.valid){
        /*! - Extract rotations from relevant messages */
        Eigen::Matrix3d dcm_CB, dcm_BN, dcm_NC;
        double CB[3][3], BN[3][3];
        MRP2C(cameraSpecs.sigma_CB, CB);
        dcm_CB = c2DArray2EigenMatrix3d(CB);
        MRP2C(navAttBuffer.sigma_BN, BN);
        dcm_BN = c2DArray2EigenMatrix3d(BN);

        dcm_NC = dcm_BN.transpose() * dcm_CB.transpose();

        /*! - Find pixel size using camera specs */
        double pX, pY;
        /* compute sensorSize/focalLength = 2*tan(FOV/2) */
        pX = 2.*tan(cameraSpecs.fieldOfView*cameraSpecs.resolution[0]/cameraSpecs.resolution[1]/2.0);
        pY = 2.*tan(cameraSpecs.fieldOfView/2.0);
        double X = pX/cameraSpecs.resolution[0];
        double Y = pY/cameraSpecs.resolution[1];

        /*! - Get the heading in the image plane */
        Eigen::Vector3d rhat_BN_C;
        rhat_BN_C[0] = (cobMsgBuffer.centerOfBrightness[0] - cameraSpecs.resolution[0]/2 + 0.5)*X;
        rhat_BN_C[1] = (cobMsgBuffer.centerOfBrightness[1] - cameraSpecs.resolution[1]/2 + 0.5)*Y;
        rhat_BN_C[2] = 1.0; // Image plane

        /*! - Retrieve the vector from target to camera and normalize */
        rhat_BN_C *= - 1;
        rhat_BN_C.normalize();

        /*! - Rotate the vector into frames of interest */
        Eigen::Vector3d rhat_BN_N, rhat_BN_B;
        rhat_BN_N = dcm_NC * rhat_BN_C;
        rhat_BN_B = dcm_CB.transpose() * rhat_BN_C;

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
        /*! - write output message */
        eigenMatrix3d2CArray(covar_N, uVecMsgBuffer.covar_N);
        eigenMatrix3d2CArray(covar_C, uVecMsgBuffer.covar_C);
        eigenMatrix3d2CArray(covar_B, uVecMsgBuffer.covar_B);
        eigenVector3d2CArray(rhat_BN_N, uVecMsgBuffer.rhat_BN_N);
        eigenVector3d2CArray(rhat_BN_C, uVecMsgBuffer.rhat_BN_C);
        eigenVector3d2CArray(rhat_BN_B, uVecMsgBuffer.rhat_BN_B);

        uVecMsgBuffer.timeTag = (double) cobMsgBuffer.timeTag;
        uVecMsgBuffer.valid = true;
    }

    this->opnavUnitVecOutMsg.write(&uVecMsgBuffer, this->moduleID, CurrentSimNanos);

}
