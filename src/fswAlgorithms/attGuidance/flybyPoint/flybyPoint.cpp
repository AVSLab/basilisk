/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "fswAlgorithms/attGuidance/flybyPoint/flybyPoint.h"
#include <cmath>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

/*! Module constructor */
FlybyPoint::FlybyPoint() = default;


/*! Module destructor */
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
    if (!this->filterInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, ".filterInMsg wasn't connected.");
    }
    if (this->flybyModel == cwEquations && !this->asteroidEphemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, ".asteroidEphemerisInMsg wasn't connected.");
    }

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

    /*! compute dt from current time and last filter read time [s] */
    double dt = (CurrentSimNanos - this->lastFilterReadTime)*NANO2SEC;

    if ((dt >= this->dtFilterData) || this->firstRead) {
        /*! set firstRead to false if this was the first read after a reset */
        if (this->firstRead) {
            this->firstRead = false;
        }

        /*! read and allocate the attitude navigation message */
        NavTransMsgPayload relativeState = this->filterInMsg();

        /*! compute velocity/radius ratio at time of read */
        this->f0 = v3Norm(relativeState.v_BN_N) / v3Norm(relativeState.r_BN_N);

        /*! compute radial (ur_N), velocity (uv_N), along-track (ut_N), and out-of-plane (uh_N) unit direction vectors */
        double ur_N[3];
        v3Normalize(relativeState.r_BN_N, ur_N);
        double uv_N[3];
        v3Normalize(relativeState.v_BN_N, uv_N);
        double uh_N[3];
        double ut_N[3];
        if (1 - v3Dot(ur_N, uv_N) < this->epsilon) {
            v3Perpendicular(ur_N, uh_N);
            v3Normalize(uh_N, uh_N);
            v3Cross(uh_N, ur_N, ut_N);
            // compute flight path angle at the time of read
            this->gamma0 = M_PI_2;
            this->singularityFlag = plusInfinity;
        }
        else if (v3Dot(ur_N, uv_N) + 1 < this->epsilon) {
            v3Perpendicular(ur_N, uh_N);
            v3Normalize(uh_N, uh_N);
            v3Cross(uh_N, ur_N, ut_N);
            // compute flight path angle at the time of read
            this->gamma0 = -M_PI_2;
            this->singularityFlag = minusInfinity;
        }
        else {
            v3Cross(ur_N, uv_N, uh_N);
            v3Normalize(uh_N, uh_N);
            v3Cross(uh_N, ur_N, ut_N);
            // compute flight path angle at the time of read
            this->gamma0 = atan(v3Dot(relativeState.v_BN_N, ur_N) / v3Dot(relativeState.v_BN_N, ut_N));
            this->singularityFlag = nonSingular;
        }

        /*! compute inertial-to-reference DCM at time of read */
        for (int i=0; i<3; i++) {
            this->R0N[0][i] = ur_N[i];
            this->R0N[1][i] = ut_N[i];
            this->R0N[2][i] = uh_N[i];
        }

        /*! update lastFilterReadTime to current time and dt to zero */
        this->lastFilterReadTime = CurrentSimNanos;
        dt = 0;
    }

    /*! compute rotation angle of reference frame from last read time */
    double theta;
    if (this->singularityFlag == nonSingular) {
        theta = atan(tan(this->gamma0) +this->f0 / cos(this->gamma0) * dt) - this->gamma0;
    }
    else {
        theta = 0;
    }

    /*! compute DCM (RtR0) of reference frame from last read time */
    double PRV_theta[3] = {0, 0, theta};
    double RtR0[3][3];
    PRV2C(PRV_theta, RtR0);

    /*! compute DCM of reference frame at time t_0 + dt with respect to inertial frame */
    double RtN[3][3];
    m33MultM33(RtR0, this->R0N, RtN);

    /*! compute scalar angular rate and acceleration of the reference frame in R-frame coordinates */
    double den = (this->f0*this->f0*dt*dt + 2*this->f0*sin(this->gamma0)*dt + 1);
    double thetaDot = this->f0 * cos(this->gamma0) / den;
    double thetaDDot = -2*this->f0*this->f0*cos(this->gamma0) * (this->f0*dt + sin(this->gamma0)) / (den*den);
    double omega_RN_R[3] = {0, 0, thetaDot};
    double omegaDot_RN_R[3] = {0, 0, thetaDDot};

    /*! populate attRefOut with reference frame information */
    C2MRP(RtN, attMsgBuffer.sigma_RN);
    if (this->signOfOrbitNormalFrameVector == -1) {
        double halfRotationX[3] = {1, 0, 0};
        addMRP(attMsgBuffer.sigma_RN, halfRotationX, attMsgBuffer.sigma_RN);
    }
    m33tMultV3(RtN, omega_RN_R, attMsgBuffer.omega_RN_N);
    m33tMultV3(RtN, omegaDot_RN_R, attMsgBuffer.domega_RN_N);

    /*! Write the output messages */
    this->attRefOutMsg.write(&attMsgBuffer, this->moduleID, CurrentSimNanos);

    /*! Write the C-wrapped output messages */
    AttRefMsg_C_write(&attMsgBuffer, &this->attRefOutMsgC, this->moduleID, CurrentSimNanos);
}
