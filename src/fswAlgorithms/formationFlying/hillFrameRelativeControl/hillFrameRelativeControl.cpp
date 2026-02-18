/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#include "fswAlgorithms/formationFlying/hillFrameRelativeControl/hillFrameRelativeControl.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/orbitalMotion.h"

/*! This is the constructor for the module class. It sets default variable values.
 */
HillFrameRelativeControl::HillFrameRelativeControl() = default;

/*! Module Destructor
 */
HillFrameRelativeControl::~HillFrameRelativeControl() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
 */
void HillFrameRelativeControl::Reset(uint64_t currentSimNanos)
{
    (void) currentSimNanos;
    if (!this->chiefTransInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl.chiefTransInMsg was not linked.");
    }

    const bool hillStateLinked = this->hillStateInMsg.isLinked();
    const bool deputyStateLinked = this->deputyTransInMsg.isLinked();
    if (hillStateLinked == deputyStateLinked) {
        this->bskLogger.bskLog(BSK_ERROR,
                               "hillFrameRelativeControl requires exactly one of hillStateInMsg or deputyTransInMsg to be linked.");
    }
    if (!this->deputyVehicleConfigInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl.deputyVehicleConfigInMsg was not linked.");
    } else {
        const VehicleConfigMsgPayload deputyVehicleConfig = this->deputyVehicleConfigInMsg();
        if (deputyVehicleConfig.massSC <= 0.0) {
            this->bskLogger.bskLog(
                BSK_ERROR,
                "hillFrameRelativeControl.deputyVehicleConfigInMsg.massSC must be positive."
            );
        }
    }
    if (this->mu <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl.mu must be positive.");
    }
    if (!this->setKFlag) {
        this->bskLogger.bskLog(BSK_ERROR, "HillFrameRelativeControl.K must be set before simulation start.");
    }
    if (!this->setPFlag) {
        this->bskLogger.bskLog(BSK_ERROR, "HillFrameRelativeControl.P must be set before simulation start.");
    }
}

/*! This is the main method that gets called every time the module is updated.
 */
void HillFrameRelativeControl::UpdateState(uint64_t currentSimNanos)
{
    this->readMessages();
    this->computeForceCommand();
    this->writeMessages(currentSimNanos);
}

/*! This method self initializes the C-wrapped output message.
 */
void HillFrameRelativeControl::SelfInit()
{
    CmdForceInertialMsg_C_init(&this->forceOutMsgC);
}

/*! Reads the input messages
 */
void HillFrameRelativeControl::readMessages()
{
    this->chiefNavBuffer = this->chiefTransInMsg();
    this->deputyVehicleBuffer = this->deputyVehicleConfigInMsg();
    if (this->hillStateInMsg.isLinked()) {
        this->hillStateBuffer = this->hillStateInMsg();
    } else {
        this->deputyNavBuffer = this->deputyTransInMsg();
    }
}

/*! Computes the inertial force command from Hill-frame relative-state tracking errors.
 */
void HillFrameRelativeControl::computeForceCommand()
{
    v3SetZero(this->forceCmd_N);  // Reset commanded inertial force vector before recomputing this cycle. [N]

    if (this->mu <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl.mu must be positive.");
        return;
    }

    const double rChiefNorm = v3Norm(this->chiefNavBuffer.r_BN_N);  // Chief orbital radius magnitude. [m]
    if (rChiefNorm <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl chief radius norm is zero.");
        return;
    }

    double hChief_N[3];  // Chief specific angular momentum vector in inertial frame. [m^2/s]
    v3Cross(this->chiefNavBuffer.r_BN_N, this->chiefNavBuffer.v_BN_N, hChief_N);
    const double hChiefNorm = v3Norm(hChief_N);  // Chief specific angular momentum magnitude. [m^2/s]
    if (hChiefNorm <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl chief angular momentum norm is zero.");
        return;
    }

    double dcm_HN[3][3];  // Direction cosine matrix from inertial frame N to Hill frame H. [-]
    hillFrame(this->chiefNavBuffer.r_BN_N, this->chiefNavBuffer.v_BN_N, dcm_HN);

    double rRel_H[3];  // Deputy relative position in Hill frame (deputy with respect to chief). [m]
    double vRel_H[3];  // Deputy relative velocity in Hill frame (deputy with respect to chief). [m/s]
    if (this->hillStateInMsg.isLinked()) {
        v3Copy(this->hillStateBuffer.r_DC_H, rRel_H);
        v3Copy(this->hillStateBuffer.v_DC_H, vRel_H);
    } else {
        rv2hill(this->chiefNavBuffer.r_BN_N, this->chiefNavBuffer.v_BN_N,
                this->deputyNavBuffer.r_BN_N, this->deputyNavBuffer.v_BN_N,
                rRel_H, vRel_H);
    }

    const double thetaDot = hChiefNorm / (rChiefNorm * rChiefNorm);  // Chief true-latitude rate. [rad/s]
    const double rDot = v3Dot(this->chiefNavBuffer.r_BN_N, this->chiefNavBuffer.v_BN_N) / rChiefNorm;  // Chief radial rate. [m/s]
    const double thetaDDot = -2.0 * rDot * thetaDot / rChiefNorm;  // Chief true-latitude acceleration. [rad/s^2]
    const double muOverR3 = this->mu / (rChiefNorm * rChiefNorm * rChiefNorm);  // Gravitational parameter divided by chief radius cubed. [1/s^2]

    double ffA1[3];  // A1*x feedforward term from relative dynamics model. [m/s^2]
    ffA1[0] = (2.0 * muOverR3 + thetaDot * thetaDot) * rRel_H[0] + thetaDDot * rRel_H[1];
    ffA1[1] = -thetaDDot * rRel_H[0] + (thetaDot * thetaDot - muOverR3) * rRel_H[1];
    ffA1[2] = -muOverR3 * rRel_H[2];

    double ffA2[3];  // A2*v feedforward term from relative dynamics model. [m/s^2]
    ffA2[0] = 2.0 * thetaDot * vRel_H[1];
    ffA2[1] = -2.0 * thetaDot * vRel_H[0];
    ffA2[2] = 0.0;

    double rErr_H[3];  // Hill-frame position tracking error (actual minus reference). [m]
    double vErr_H[3];  // Hill-frame velocity tracking error (actual minus reference). [m/s]
    v3Subtract(rRel_H, this->rRef_H.data(), rErr_H);
    v3Subtract(vRel_H, this->vRef_H.data(), vErr_H);

    double kTerm[3];  // Proportional feedback acceleration term K*rErr_H. [m/s^2]
    double pTerm[3];  // Derivative feedback acceleration term P*vErr_H. [m/s^2]
    m33MultV3(this->K, rErr_H, kTerm);
    m33MultV3(this->P, vErr_H, pTerm);

    double aCmd_H[3];  // Commanded control acceleration in Hill frame. [m/s^2]
    for (int i = 0; i < 3; i++) {
        const double ffTerm = -(ffA1[i] + ffA2[i]);  // Net feedforward acceleration contribution for axis i. [m/s^2]
        aCmd_H[i] = -kTerm[i] - pTerm[i] + ffTerm;
    }

    double forceCmd_N_unscaled[3];  // Commanded inertial acceleration before mass scaling. [m/s^2]
    m33tMultV3(dcm_HN, aCmd_H, forceCmd_N_unscaled);
    v3Scale(this->deputyVehicleBuffer.massSC, forceCmd_N_unscaled, this->forceCmd_N);  // Final inertial force command after multiplying by deputy mass. [N]
}

/*! Writes the output message
 */
void HillFrameRelativeControl::writeMessages(uint64_t currentSimNanos)
{
    CmdForceInertialMsgPayload forceOutMsgBuffer = this->forceOutMsg.zeroMsgPayload;
    v3Copy(this->forceCmd_N, forceOutMsgBuffer.forceRequestInertial);
    this->forceOutMsg.write(&forceOutMsgBuffer, this->moduleID, currentSimNanos);
    CmdForceInertialMsg_C_write(&forceOutMsgBuffer, &this->forceOutMsgC, this->moduleID, currentSimNanos);
}

/*! Sets the position feedback gain matrix.
 */
void HillFrameRelativeControl::setK(const std::vector<double>& gain)
{
    if (gain.size() != 9) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl: K must contain exactly 9 elements.");
        return;
    }

    double kCandidate[3][3];
    m33Set(gain[0], gain[1], gain[2],
           gain[3], gain[4], gain[5],
           gain[6], gain[7], gain[8], kCandidate);
    double kTranspose[3][3];
    m33Transpose(kCandidate, kTranspose);
    if (!m33IsEqual(kCandidate, kTranspose, 1e-12)) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl: K must be symmetric positive definite.");
        return;
    }
    double kEigVals[3];
    m33EigenValues(kCandidate, kEigVals);
    if (!(kEigVals[0] > 1e-12 && kEigVals[1] > 1e-12 && kEigVals[2] > 1e-12)) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl: K must be symmetric positive definite.");
        return;
    }
    m33Copy(kCandidate, this->K);
    this->setKFlag = true;
}

/*! Sets the velocity feedback gain matrix.
 */
void HillFrameRelativeControl::setP(const std::vector<double>& gain)
{
    if (gain.size() != 9) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl: P must contain exactly 9 elements.");
        return;
    }

    double pCandidate[3][3];
    m33Set(gain[0], gain[1], gain[2],
           gain[3], gain[4], gain[5],
           gain[6], gain[7], gain[8], pCandidate);
    double pTranspose[3][3];
    m33Transpose(pCandidate, pTranspose);
    if (!m33IsEqual(pCandidate, pTranspose, 1e-12)) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl: P must be symmetric positive definite.");
        return;
    }
    double pEigVals[3];
    m33EigenValues(pCandidate, pEigVals);
    if (!(pEigVals[0] > 1e-12 && pEigVals[1] > 1e-12 && pEigVals[2] > 1e-12)) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl: P must be symmetric positive definite.");
        return;
    }
    m33Copy(pCandidate, this->P);
    this->setPFlag = true;
}

/*! Sets the Hill-frame relative position reference.
 */
void HillFrameRelativeControl::setReferencePosition(const std::vector<double>& rRef_H)
{
    if (rRef_H.size() != 3) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl.setReferencePosition requires a length-3 vector.");
        return;
    }
    double rRefArray[3];
    v3Set(rRef_H[0], rRef_H[1], rRef_H[2], rRefArray);
    v3Copy(rRefArray, this->rRef_H.data());
}

/*! Sets the Hill-frame relative velocity reference.
 */
void HillFrameRelativeControl::setReferenceVelocity(const std::vector<double>& vRef_H)
{
    if (vRef_H.size() != 3) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl.setReferenceVelocity requires a length-3 vector.");
        return;
    }
    double vRefArray[3];
    v3Set(vRef_H[0], vRef_H[1], vRef_H[2], vRefArray);
    v3Copy(vRefArray, this->vRef_H.data());
}

void HillFrameRelativeControl::setMu(double mu)
{
    if (mu <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "hillFrameRelativeControl: mu must be positive.");
    } else {
        this->mu = mu;
    }
}

std::vector<double> HillFrameRelativeControl::getK() const
{
    return {
        this->K[0][0], this->K[0][1], this->K[0][2],
        this->K[1][0], this->K[1][1], this->K[1][2],
        this->K[2][0], this->K[2][1], this->K[2][2]
    };
}

std::vector<double> HillFrameRelativeControl::getP() const
{
    return {
        this->P[0][0], this->P[0][1], this->P[0][2],
        this->P[1][0], this->P[1][1], this->P[1][2],
        this->P[2][0], this->P[2][1], this->P[2][2]
    };
}

std::vector<double> HillFrameRelativeControl::getReferencePosition() const
{
    return {this->rRef_H[0], this->rRef_H[1], this->rRef_H[2]};
}

std::vector<double> HillFrameRelativeControl::getReferenceVelocity() const
{
    return {this->vRef_H[0], this->vRef_H[1], this->vRef_H[2]};
}

double HillFrameRelativeControl::getMu() const
{
    return this->mu;
}
