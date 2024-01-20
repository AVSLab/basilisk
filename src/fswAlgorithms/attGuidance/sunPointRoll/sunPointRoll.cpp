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


#include "sunPointRoll.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

const double epsilon = 1e-12;                           // module tolerance for zero

/*! Module constructor */
SunPointRoll::SunPointRoll() = default;


/*! Module destructor */
SunPointRoll::~SunPointRoll() = default;


/*! Initialize C-wrapped output messages */
void SunPointRoll::SelfInit(){
    AttRefMsg_C_init(&this->attRefOutMsgC);
}

/*! This method is used to reset the module.
 @return void
 */
void SunPointRoll::Reset(uint64_t CurrentSimNanos)
{
    if (!this->attNavInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, ".attNavInMsg wasn't connected.");
    }
    this->resetTime = CurrentSimNanos;
}


/*! This method is the main carrier for the boresight calculation routine.  If it detects
 that it needs to re-init (direction change maybe) it will re-init itself.
 Then it will compute the angles away that the boresight is from the celestial target.
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void SunPointRoll::UpdateState(uint64_t CurrentSimNanos)
{
    /*! create and zero the output message */
    AttRefMsgPayload attRefOut = this->attRefOutMsg.zeroMsgPayload;

    /*! read and allocate the input attitude navigation message */
    NavAttMsgPayload attNavIn = this->attNavInMsg();

    /*! define the body frame orientation DCM BN */
    double BN[3][3];
    MRP2C(attNavIn.sigma_BN, BN);

    /*! get the solar array drive direction in body frame coordinates */
    double a1Hat_B[3];
    v3Normalize(this->a1Hat_B, a1Hat_B);

    /*! read Sun direction in N frame from the attNav message */
    double rHat_SB_N[3];
    m33tMultV3(BN, attNavIn.vehSunPntBdy, rHat_SB_N);

    /*! compute intermediate reference frame DN to align vector with Sun */
    double d1[3];
    double d2[3];
    double d3[3];
    v3Normalize(this->hRefHat_B, d1);
    v3Cross(d1, a1Hat_B, d2);
    v3Normalize(d2, d2);
    v3Cross(d1, d2, d3);

    double n1[3];
    double n2[3] = {0, 0, 1};
    double n3[3];
    v3Copy(rHat_SB_N, n1);
    v3Cross(n1, n2, n3);
    v3Normalize(n3, n3);
    v3Cross(n3, n1, n2);

    double DT[3][3];
    double NT[3][3];
    for (int i=0; i<3; ++i) {
        DT[i][0] = d1[i];
        DT[i][1] = d2[i];
        DT[i][2] = d3[i];
        NT[i][0] = n1[i];
        NT[i][1] = n2[i];
        NT[i][2] = n3[i];
    }

    double DN[3][3];
    m33MultM33t(DT, NT, DN);

    /*! compute intermediate reference RD that accounts for the roll about the sunline */
    double pra = this->omegaRoll * (double (CurrentSimNanos - this->resetTime));
    double prv_RD[3];
    v3Scale(pra, a1Hat_B, prv_RD);

    double RD[3][3];
    PRV2C(prv_RD, RD);

    /*! compute final reference frame RN */
    double RN[3][3];
    m33MultM33(RD, DN, RN);

    /*! compute reference MRP */
    double sigma_RN[3];
    C2MRP(RN, sigma_RN);
    v3Copy(sigma_RN, attRefOut.sigma_RN);

    /*! write omega roll into the reference message */
    double omega_RN_R[3];
    v3Scale(this->omegaRoll, this->hRefHat_B, omega_RN_R);
    m33tMultV3(RN, omega_RN_R, attRefOut.omega_RN_N);

    /*! Write the output messages */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, CurrentSimNanos);

    /*! Write the C-wrapped output messages */
    AttRefMsg_C_write(&attRefOut, &this->attRefOutMsgC, this->moduleID, CurrentSimNanos);
}

