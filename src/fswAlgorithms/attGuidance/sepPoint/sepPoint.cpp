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
#include <math.h>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "fswAlgorithms/attGuidance/_GeneralModuleFiles/attitudePointingLibrary.h"

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

    /*! map requested heading into B frame */
    double hReqHat_B[3];
    m33MultV3(BN, hReqHat_N, hReqHat_B);

    /*! compute intermediate rotation DB to align the boresight */
    double DB[3][3];
    boresightAlignment(hRefHat_B, hReqHat_B, epsilon, DB);

    /*! map Sun direction vector to intermediate frame */
    double rHat_SB_D[3];
    m33MultV3(DB, rHat_SB_B, rHat_SB_D);

    /*! define the coefficients of the quadratic equation A, B, and C for the solar array drive axis */
    double e_psi[3];
    v3Copy(hRefHat_B, e_psi);
    double b3[3];
    v3Cross(rHat_SB_D, e_psi, b3);
    double A = 2 * v3Dot(rHat_SB_D, e_psi) * v3Dot(e_psi, a1Hat_B) - v3Dot(a1Hat_B, rHat_SB_D);
    double B = 2 * v3Dot(a1Hat_B, b3);
    double C = v3Dot(a1Hat_B, rHat_SB_D);

    /*! define the coefficients of the quadratic equation D, E, and F for the Sun-constrained axis */
    double D = 2 * v3Dot(rHat_SB_D, e_psi) * v3Dot(e_psi, a2Hat_B) - v3Dot(a2Hat_B, rHat_SB_D);
    double E = 2 * v3Dot(a2Hat_B, b3);
    double F = v3Dot(a2Hat_B, rHat_SB_D);

    /*! compute the solution(s) to the optimized solar array alignment problem */
    SolutionSpace solarArraySolutions(A, B, C, epsilon);

    double PRV_psi[3];
    switch (this->alignmentPriority) {

        case solarArrayAlign :
            if (solarArraySolutions.numberOfZeros() == 2) {
                double psi1 = solarArraySolutions.returnAbsMin(1);
                double psi2 = solarArraySolutions.returnAbsMin(2);
                double PRV_psi1[3];
                v3Scale(psi1, e_psi, PRV_psi1);
                double PRV_psi2[3];
                v3Scale(psi2, e_psi, PRV_psi2);
                double P1D[3][3];
                PRV2C(PRV_psi1, P1D);
                double P2D[3][3];
                PRV2C(PRV_psi2, P2D);
                double rHat_SB_P1[3];
                m33MultV3(P1D, rHat_SB_D, rHat_SB_P1);
                double rHat_SB_P2[3];
                m33MultV3(P2D, rHat_SB_D, rHat_SB_P2);
                if (fabs(v3Dot(a2Hat_B, rHat_SB_P2) - v3Dot(a2Hat_B, rHat_SB_P1)) > 0) {
                    v3Scale(psi2, e_psi, PRV_psi);
                }
                else {
                    v3Scale(psi1, e_psi, PRV_psi);
                }
            }
            else {
                double psi = solarArraySolutions.returnAbsMin(1);
                v3Scale(psi, e_psi, PRV_psi);
            }
            break;

        case sunConstrAxisAlign :
            double k = cos(this->beta);
            SolutionSpace sunConstAxisSolutions(D-k, E, F-k, epsilon);
            if (sunConstAxisSolutions.isEmpty()) {
                double psi = sunConstAxisSolutions.returnAbsMin(1);
                v3Scale(psi, e_psi, PRV_psi);
            }
            else if (solarArraySolutions.numberOfZeros() == 2) {
                double psi1 = solarArraySolutions.returnAbsMin(1);
                double psi2 = solarArraySolutions.returnAbsMin(2);
                double deltaPsi1 = psi1 - sunConstAxisSolutions.passThrough(psi1);
                double deltaPsi2 = psi2 - sunConstAxisSolutions.passThrough(psi2);
                if (fabs(deltaPsi2 - deltaPsi1) > 0) {
                    v3Scale(psi1, e_psi, PRV_psi);
                } else {
                    v3Scale(psi2, e_psi, PRV_psi);
                }
            }
            else {
                double psi = solarArraySolutions.returnAbsMin(1);
                psi = sunConstAxisSolutions.passThrough(psi);
                v3Scale(psi, e_psi, PRV_psi);
            }
            break;
    }

    /*! map PRV to RD direction cosine matrix */
    double RD[3][3];
    PRV2C(PRV_psi, RD);
    double RB[3][3];
    m33MultM33(RD, DB, RB);
    double RN[3][3];
    m33MultM33(RB, BN, RN);

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
                                 CurrentSimNanos,
                                 this->T1NanoSeconds,
                                 this->T2NanoSeconds,
                                 this->callCount,
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

