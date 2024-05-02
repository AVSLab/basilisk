/*
 ISC License

 Copyright (c) 2024, University of Colorado at Boulder

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

#include "fswAlgorithms/attGuidance/flybyPoint/flybyPoint.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"

FlybyPoint::FlybyPoint() = default;

FlybyPoint::~FlybyPoint() = default;

/*! Initialize C-wrapped output messages */
void FlybyPoint::SelfInit(){
    AttRefMsg_C_init(&this->attRefOutMsgC);
}

/*! This method is used to reset the module.
 @return void
 */
void FlybyPoint::Reset(uint64_t CurrentSimNanos)
{
    this->lastFilterReadTime = 0;
    this->firstRead = true;
}


/*! This method is the main carrier for the boresight calculation routine.  If it detects
 that it needs to re-init (direction change maybe) it will re-init itself.
 Then it will compute the angles away that the boresight is from the celestial target.
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void FlybyPoint::UpdateState(uint64_t CurrentSimNanos)
{
    /*! create and zero the output message */
    AttRefMsgPayload attMsgBuffer = this->attRefOutMsg.zeroMsgPayload;
    NavTransMsgPayload relativeState = this->filterInMsg();

    /*! compute dt from current time and last filter read time [s] */
    double dt = (CurrentSimNanos - this->lastFilterReadTime)*NANO2SEC;
    bool invalidSolution = false;

    if ((dt >= this->timeBetweenFilterData) || this->firstRead) {
        /*! set firstRead to false if this was the first read after a reset */
        if (this->firstRead) {
            this->firstRead = false;
        }
        /*! compute velocity/radius ratio at time of read */
        this->r_BN_N = Eigen::Map<Eigen::Vector3d>(relativeState.r_BN_N);
        this->v_BN_N = Eigen::Map<Eigen::Vector3d>(relativeState.v_BN_N);
        this->f0 = this->v_BN_N.norm() / this->r_BN_N.norm();

        /*! compute radial (ur_N), velocity (uv_N), along-track (ut_N), and out-of-plane (uh_N) unit direction vectors */
        Eigen::Vector3d ur_N = this->r_BN_N.normalized();
        Eigen::Vector3d uv_N = this->v_BN_N.normalized();
        /*! assert r and v are not collinear (collision trajectory) */
        if (std::abs(1 - ur_N.dot(uv_N)) < this->toleranceForCollinearity) {
            invalidSolution = false;
        }
        else {
            invalidSolution = true;
        }

        Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
        Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

        // compute flight path angle at the time of read
        this->gamma0 = std::atan(this->v_BN_N.dot(ur_N) / this->v_BN_N.dot(ut_N));

        /*! compute inertial-to-reference DCM at time of read */
        this->R0N.row(0) = ur_N;
        this->R0N.row(1) = ut_N;
        this->R0N.row(2) = uh_N;

        /*! update lastFilterReadTime to current time and dt to zero */
        this->lastFilterReadTime = CurrentSimNanos;
        dt = 0;
    }

    double theta = 0;
    if (!invalidSolution) {
        theta = std::atan(std::tan(this->gamma0) + this->f0 / std::cos(this->gamma0) * dt) - this->gamma0;
    }

    /*! compute DCM (RtR0) of reference frame from last read time */
    Eigen::Vector3d PRV_theta;
    PRV_theta << 0, 0, theta;
    Eigen::Matrix3d RtR0 = prvToDcm(PRV_theta);

    /*! compute DCM of reference frame at time t_0 + dt with respect to inertial frame */
    Eigen::Matrix3d RtN = RtR0*this->R0N;

    /*! compute scalar angular rate and acceleration of the reference frame in R-frame coordinates */
    double den = (this->f0*this->f0*dt*dt + 2*this->f0*sin(this->gamma0)*dt + 1);
    double thetaDot = this->f0 * cos(this->gamma0) / den;
    double thetaDDot = -2*this->f0*this->f0*cos(this->gamma0) * (this->f0*dt + sin(this->gamma0)) / (den*den);
    Eigen::Vector3d omega_RN_R;
    omega_RN_R << 0, 0, thetaDot;
    Eigen::Vector3d omegaDot_RN_R;
    omegaDot_RN_R << 0, 0, thetaDDot;

    /*! populate attRefOut with reference frame information */
    Eigen::Vector3d sigma_RN = dcmToMrp(RtN);

    if (this->signOfOrbitNormalFrameVector == -1) {
        Eigen::Vector3d halfRotationX;
        halfRotationX << 1, 0, 0;
        sigma_RN = addMrp(sigma_RN, halfRotationX);
    }
    Eigen::Vector3d omega_RN_N = RtN.transpose()*omega_RN_R;
    Eigen::Vector3d omegaDot_RN_N = RtN.transpose()*omegaDot_RN_R;

    eigenVector3d2CArray(sigma_RN, attMsgBuffer.sigma_RN);
    eigenVector3d2CArray(omega_RN_N, attMsgBuffer.omega_RN_N);
    eigenVector3d2CArray(omegaDot_RN_N, attMsgBuffer.domega_RN_N);

    /*! Write the output messages */
    this->attRefOutMsg.write(&attMsgBuffer, this->moduleID, CurrentSimNanos);
    /*! Write the C-wrapped output messages */
    AttRefMsg_C_write(&attMsgBuffer, &this->attRefOutMsgC, this->moduleID, CurrentSimNanos);
}

double FlybyPoint::getTimeBetweenFilterData() const {
    return this->timeBetweenFilterData;
}

void FlybyPoint::setTimeBetweenFilterData(double time) {
    this->timeBetweenFilterData = time;
}

double FlybyPoint::getToleranceForCollinearity() const {
    return this->toleranceForCollinearity;
}

void FlybyPoint::setToleranceForCollinearity(double tolerance) {
    this->toleranceForCollinearity = tolerance;
}

int64_t FlybyPoint::getSignOfOrbitNormalFrameVector() const {
    return this->signOfOrbitNormalFrameVector;
}

void FlybyPoint::setSignOfOrbitNormalFrameVector(int64_t sign) {
    this->signOfOrbitNormalFrameVector = sign;
}
