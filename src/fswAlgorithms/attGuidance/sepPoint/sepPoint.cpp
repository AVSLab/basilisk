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

#include "sepPoint.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

const double epsilon = 1e-12;                           // module tolerance for zero

/*! Module constructor */
SepPoint::SepPoint() = default;


/*! Module destructor */
SepPoint::~SepPoint() = default;


/*! Initialize C-wrapped output messages */
void SepPoint::SelfInit(){
    AttRefMsg_C_init(&this->attRefOutMsgC);
}

/*! This method is used to reset the module.
 @return void
 */
void SepPoint::Reset(uint64_t CurrentSimNanos)
{
    if (!this->attNavInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, ".attNavInMsg wasn't connected.");
    }
    if (!this->bodyHeadingInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, ".bodyHeadingInMsg wasn't connected.");
    }
    if (!this->inertialHeadingInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, ".inertialHeadingInMsg wasn't connected.");
    }
    this->callCount = 0;
}


/*! This method is the main carrier for the boresight calculation routine.  If it detects
 that it needs to re-init (direction change maybe) it will re-init itself.
 Then it will compute the angles away that the boresight is from the celestial target.
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void SepPoint::UpdateState(uint64_t CurrentSimNanos)
{
    /*! create and zero the output message */
    AttRefMsgPayload attRefOut = this->attRefOutMsg.zeroMsgPayload;

    /*! read and allocate the input attitude navigation message */
    NavAttMsgPayload attNavIn = this->attNavInMsg();

    /*! read and allocate the input inertial heading message */
    BodyHeadingMsgPayload bodyHeadingIn = this->bodyHeadingInMsg();
    double hRefHat_B[3];
    v3Normalize(bodyHeadingIn.rHat_XB_B, hRefHat_B);

    /*! read and allocate the input inertial heading message */
    InertialHeadingMsgPayload inertialHeadingIn = this->inertialHeadingInMsg();
    double hReqHat_N[3];
    v3Normalize(inertialHeadingIn.rHat_XN_N, hReqHat_N);

    /*! define the body frame orientation DCM BN */
    double BN[3][3];
    MRP2C(attNavIn.sigma_BN, BN);

    /*! get the solar array drive direction in body frame coordinates */
    double a1Hat_B[3];
    v3Normalize(this->a1Hat_B, a1Hat_B);

    /*! get the Sun-constrained axis in body frame coordinates */
    double a2Hat_B[3];
    v3Normalize(this->a2Hat_B, a2Hat_B);

    /*! read Sun direction in B frame from the attNav message */
    double rHat_SB_B[3];
    v3Copy(attNavIn.vehSunPntBdy, rHat_SB_B);

    /*! compute reference frame RN */
    double RN[3][3];
    computeReferenceFrame(hRefHat_B, hReqHat_N, rHat_SB_B, a1Hat_B, a2Hat_B, this->beta, BN,
                          this->alignmentPriority, epsilon, RN);

    /*! compute reference MRP */
    double sigma_RN[3];
    C2MRP(RN, sigma_RN);
    v3Copy(sigma_RN, attRefOut.sigma_RN);

    /*! compute reference angular rates and accelerations via finite differences */
    double omega_RN_R[3];
    double omegaDot_RN_R[3];
    finiteDifferencesRatesAndAcc(sigma_RN,
                                 this->sigma_RN_1,
                                 this->sigma_RN_2,
                                 &CurrentSimNanos,
                                 &this->T1NanoSeconds,
                                 &this->T2NanoSeconds,
                                 &this->callCount,
                                 omega_RN_R,
                                 omegaDot_RN_R);

    /*! compute angular rates and accelerations in N frame and store in buffer msg */
    m33tMultV3(RN, omega_RN_R, attRefOut.omega_RN_N);
    m33tMultV3(RN, omegaDot_RN_R, attRefOut.domega_RN_N);

    /*! Write the output messages */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, CurrentSimNanos);

    /*! Write the C-wrapped output messages */
    AttRefMsg_C_write(&attRefOut, &this->attRefOutMsgC, this->moduleID, CurrentSimNanos);
}

