/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "spinningBodyNDOFStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string>

SpinningBodyNDOFStateEffector::SpinningBodyNDOFStateEffector()
{
    // Zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);
    
    this->nameOfThetaState = "spinningBodyTheta" + std::to_string(SpinningBodyNDOFStateEffector::effectorID);
    this->nameOfThetaDotState = "spinningBodyThetaDot" + std::to_string(SpinningBodyNDOFStateEffector::effectorID);
    SpinningBodyNDOFStateEffector::effectorID++;
}

uint64_t SpinningBodyNDOFStateEffector::effectorID = 1;

SpinningBodyNDOFStateEffector::~SpinningBodyNDOFStateEffector()
{
    SpinningBodyNDOFStateEffector::effectorID --;    /* reset the panel ID*/
}

void SpinningBodyNDOFStateEffector::Reset(uint64_t CurrentClock)
{
    if (this->spinningBodyVec.back().mass <= 0.0)
        bskLogger.bskLog(BSK_ERROR, "The mass of the last element must be greater than 0.");
}

void SpinningBody::setMass(double mass) {
    if (mass >= 0.0)
        this->mass = mass;
    else {
        bskLogger.bskLog(BSK_ERROR, "Mass must be greater than or equal to 0.");
    }
}

void SpinningBody::setSHat_S(Eigen::Vector3d sHat_S) {
    if (sHat_S.norm() > 0.01) {
        this->sHat_S = sHat_S.normalized();
    }
    else {
        bskLogger.bskLog(BSK_ERROR, "Norm of sHat must be greater than 0.");
    }
}

void SpinningBody::setK(double k) {
    if (k >= 0.0)
        this->k = k;
    else {
        bskLogger.bskLog(BSK_ERROR, "k must be greater than or equal to 0.");
    }
}

void SpinningBody::setC(double c) {
    if (c >= 0.0)
        this->c = c;
    else {
        bskLogger.bskLog(BSK_ERROR, "c must be greater than or equal to 0.");
    }
}

void SpinningBodyNDOFStateEffector::addSpinningBody(const SpinningBody& newBody) {
    spinningBodyVec.push_back(newBody);
    this->numberOfDegreesOfFreedom++;

    this->spinningBodyConfigLogOutMsgs.push_back(new Message<SCStatesMsgPayload>);
    this->spinningBodyOutMsgs.push_back(new Message<HingedRigidBodyMsgPayload>);
    this->spinningBodyRefInMsgs.push_back(ReadFunctor<HingedRigidBodyMsgPayload>());

    this->ATheta.conservativeResize(this->ATheta.rows()+1, 3);
    this->BTheta.conservativeResize(this->BTheta.rows()+1, 3);
    this->CTheta.conservativeResize(this->CTheta.rows()+1);
}

void SpinningBodyNDOFStateEffector::readInputMessages()
{
    if (this->motorTorqueInMsg.isLinked() && this->motorTorqueInMsg.isWritten()) {
        ArrayMotorTorqueMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorTorqueInMsg();
        int spinningBodyIndex = 0;
        for(auto& spinningBody: this->spinningBodyVec) {
            spinningBody.u = incomingCmdBuffer.motorTorque[spinningBodyIndex];
            spinningBodyIndex++;
        }
    }

    if (this->motorLockInMsg.isLinked() && this->motorLockInMsg.isWritten()) {
        ArrayEffectorLockMsgPayload incomingLockBuffer;
        incomingLockBuffer = this->motorLockInMsg();
        int spinningBodyIndex = 0;
        for(auto& spinningBody: this->spinningBodyVec) {
            spinningBody.isAxisLocked = incomingLockBuffer.effectorLockFlag[spinningBodyIndex];
            spinningBodyIndex++;
        }
    }

    int spinningBodyIndex = 0;
    for(auto& spinningBody: this->spinningBodyVec) {
        if (this->spinningBodyRefInMsgs[spinningBodyIndex].isLinked() && this->spinningBodyRefInMsgs[spinningBodyIndex].isWritten()) {
            HingedRigidBodyMsgPayload incomingRefBuffer;
            incomingRefBuffer = this->spinningBodyRefInMsgs[spinningBodyIndex]();
            spinningBody.thetaRef = incomingRefBuffer.theta;
            spinningBody.thetaDotRef = incomingRefBuffer.thetaDot;
        }
        spinningBodyIndex++;
    }
}

void SpinningBodyNDOFStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    int spinningBodyIndex = 0;
    for(auto& spinningBody: this->spinningBodyVec) {
        if (this->spinningBodyOutMsgs[spinningBodyIndex]->isLinked()) {
            HingedRigidBodyMsgPayload spinningBodyBuffer = this->spinningBodyOutMsgs[spinningBodyIndex]->zeroMsgPayload;

            spinningBodyBuffer.theta = spinningBody.theta;
            spinningBodyBuffer.thetaDot = spinningBody.thetaDot;
            this->spinningBodyOutMsgs[spinningBodyIndex]->write(&spinningBodyBuffer, this->moduleID, CurrentClock);
        }

        if (this->spinningBodyConfigLogOutMsgs[spinningBodyIndex]->isLinked()) {
            SCStatesMsgPayload configLogMsg = this->spinningBodyConfigLogOutMsgs[spinningBodyIndex]->zeroMsgPayload;

            // Logging the S frame is the body frame B of that object
            eigenVector3d2CArray(spinningBody.r_ScN_N, configLogMsg.r_BN_N);
            eigenVector3d2CArray(spinningBody.v_ScN_N, configLogMsg.v_BN_N);
            eigenVector3d2CArray(spinningBody.sigma_SN, configLogMsg.sigma_BN);
            eigenVector3d2CArray(spinningBody.omega_SN_S, configLogMsg.omega_BN_B);
            this->spinningBodyConfigLogOutMsgs[spinningBodyIndex]->write(&configLogMsg, this->moduleID, CurrentClock);
        }
        spinningBodyIndex++;
    }
}

void SpinningBodyNDOFStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfThetaState = this->nameOfSpacecraftAttachedTo + this->nameOfThetaState;
    this->nameOfThetaDotState = this->nameOfSpacecraftAttachedTo + this->nameOfThetaDotState;
}

void SpinningBodyNDOFStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "r_BN_N");
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "v_BN_N");
}

void SpinningBodyNDOFStateEffector::registerStates(DynParamManager& states)
{
    this->thetaState = states.registerState(numberOfDegreesOfFreedom, 1, this->nameOfThetaState);
    this->thetaDotState = states.registerState(numberOfDegreesOfFreedom, 1, this->nameOfThetaDotState);

    Eigen::MatrixXd thetaInitMatrix(numberOfDegreesOfFreedom,1);
    Eigen::MatrixXd thetaDotInitMatrix(numberOfDegreesOfFreedom,1);
    int i = 0;
    for(const auto& spinningBody: this->spinningBodyVec) {
        thetaInitMatrix(i,0) = spinningBody.thetaInit;
        thetaDotInitMatrix(i,0) = spinningBody.thetaDotInit;
        i++;
    }
    this->thetaState->setState(thetaInitMatrix);
    this->thetaDotState->setState(thetaDotInitMatrix);
}

void SpinningBodyNDOFStateEffector::updateEffectorMassProps(double integTime)
{
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B = Eigen::Vector3d::Zero();
    this->effProps.rEffPrime_CB_B = Eigen::Vector3d::Zero();
    this->effProps.IEffPntB_B = Eigen::Matrix3d::Zero();
    this->effProps.IEffPrimePntB_B = Eigen::Matrix3d::Zero();

    int spinningBodyIndex = 0;
    for(auto& spinningBody: this->spinningBodyVec) {
        this->computeAttitudeProperties(spinningBody, spinningBodyIndex);
        this->computeAngularVelocityProperties(spinningBody, spinningBodyIndex);
        this->computePositionProperties(spinningBody, spinningBodyIndex);
        this->computeVelocityProperties(spinningBody, spinningBodyIndex);
        this->computeInertiaProperties(spinningBody);

        Eigen::Matrix3d rPrimeTilde_ScB_B = eigenTilde(spinningBody.rPrime_ScB_B);
        this->effProps.mEff += spinningBody.mass;
        this->effProps.rEff_CB_B += spinningBody.mass * spinningBody.r_ScB_B;
        this->effProps.rEffPrime_CB_B += spinningBody.mass * spinningBody.rPrime_ScB_B;
        this->effProps.IEffPntB_B += spinningBody.ISPntSc_B - spinningBody.mass * spinningBody.rTilde_ScB_B * spinningBody.rTilde_ScB_B;
        this->effProps.IEffPrimePntB_B += spinningBody.IPrimeSPntSc_B - spinningBody.mass * (rPrimeTilde_ScB_B * spinningBody.rTilde_ScB_B + spinningBody.rTilde_ScB_B * rPrimeTilde_ScB_B);

        spinningBodyIndex++;
    }
    // Divide by the total mass once all bodies have been accounted for
    this->effProps.rEff_CB_B /= this->effProps.mEff;
    this->effProps.rEffPrime_CB_B /= this->effProps.mEff;
}

void SpinningBodyNDOFStateEffector::computeAttitudeProperties(SpinningBody& spinningBody, int spinningBodyIndex) const
{
    if (spinningBody.isAxisLocked)
    {
        auto thetaDotVector = this->thetaDotState->getState();
        thetaDotVector(spinningBodyIndex) = 0.0;
        this->thetaDotState->setState(thetaDotVector);
    }

    spinningBody.theta = this->thetaState->getState()(spinningBodyIndex);
    spinningBody.thetaDot = this->thetaDotState->getState()(spinningBodyIndex);

    double dcm_S0S[3][3];
    double prv_S0S_array[3];
    Eigen::Vector3d prv_S0S = - spinningBody.theta * spinningBody.sHat_S;
    eigenVector3d2CArray(prv_S0S, prv_S0S_array);
    PRV2C(prv_S0S_array, dcm_S0S);
    if (spinningBodyIndex == 0) {
        spinningBody.dcm_BS = spinningBody.dcm_S0P.transpose() * c2DArray2EigenMatrix3d(dcm_S0S);
    } else {
        spinningBody.dcm_BS = this->spinningBodyVec[spinningBodyIndex-1].dcm_BS * spinningBody.dcm_S0P.transpose() * c2DArray2EigenMatrix3d(dcm_S0S);
    }
    spinningBody.sHat_B = spinningBody.dcm_BS * spinningBody.sHat_S;
}

void SpinningBodyNDOFStateEffector::computeAngularVelocityProperties(SpinningBody& spinningBody, int spinningBodyIndex) const
{
    spinningBody.omega_SP_B = spinningBody.thetaDot * spinningBody.sHat_B;
    spinningBody.omegaTilde_SP_B = eigenTilde(spinningBody.omega_SP_B);
    if (spinningBodyIndex == 0) {
        spinningBody.omega_SB_B = spinningBody.omega_SP_B;
    } else {
        spinningBody.omega_SB_B = spinningBody.omega_SP_B + this->spinningBodyVec[spinningBodyIndex-1].omega_SB_B;
    }
    spinningBody.omegaTilde_SB_B = eigenTilde(spinningBody.omega_SB_B);
}

void SpinningBodyNDOFStateEffector::computePositionProperties(SpinningBody& spinningBody, int spinningBodyIndex) const
{
    spinningBody.r_ScS_B = spinningBody.dcm_BS * spinningBody.r_ScS_S;
    if (spinningBodyIndex == 0) {
        spinningBody.r_SP_B = spinningBody.r_SP_P;
        spinningBody.r_SB_B = spinningBody.r_SP_P;
    } else {
        spinningBody.r_SP_B = this->spinningBodyVec[spinningBodyIndex-1].dcm_BS * spinningBody.r_SP_P;
        spinningBody.r_SB_B = spinningBody.r_SP_B + this->spinningBodyVec[spinningBodyIndex-1].r_SB_B;
    }
    spinningBody.r_ScB_B = spinningBody.r_ScS_B + spinningBody.r_SB_B;
    spinningBody.rTilde_ScB_B = eigenTilde(spinningBody.r_ScB_B);
}

void SpinningBodyNDOFStateEffector::computeVelocityProperties(SpinningBody& spinningBody, int spinningBodyIndex) const
{
    spinningBody.rPrime_ScS_B = spinningBody.omegaTilde_SB_B * spinningBody.r_ScS_B;
    if (spinningBodyIndex == 0) {
        spinningBody.rPrime_SP_B = Eigen::Vector3d::Zero();
        spinningBody.rPrime_SB_B = spinningBody.rPrime_SP_B;
    } else {
        spinningBody.rPrime_SP_B = this->spinningBodyVec[spinningBodyIndex-1].omegaTilde_SB_B * spinningBody.r_SP_B;
        spinningBody.rPrime_SB_B = spinningBody.rPrime_SP_B + this->spinningBodyVec[spinningBodyIndex-1].rPrime_SB_B;
    }
    spinningBody.rPrime_ScB_B = spinningBody.rPrime_ScS_B + spinningBody.rPrime_SB_B;
}

void SpinningBodyNDOFStateEffector::computeInertiaProperties(SpinningBody& spinningBody) const
{
    spinningBody.ISPntSc_B = spinningBody.dcm_BS * spinningBody.ISPntSc_S * spinningBody.dcm_BS.transpose();
    spinningBody.IPrimeSPntSc_B = spinningBody.omegaTilde_SB_B * spinningBody.ISPntSc_B - spinningBody.ISPntSc_B * spinningBody.omegaTilde_SB_B;
}

void SpinningBodyNDOFStateEffector::updateContributions(double integTime,
                                                        BackSubMatrices& backSubContr,
                                                        Eigen::Vector3d sigma_BN,
                                                        Eigen::Vector3d omega_BN_B,
                                                        Eigen::Vector3d g_N)
{
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    Eigen::MatrixXd MTheta = Eigen::MatrixXd::Zero(this->numberOfDegreesOfFreedom, this->numberOfDegreesOfFreedom);
    Eigen::MatrixXd AThetaStar = Eigen::MatrixXd::Zero(this->numberOfDegreesOfFreedom, 3);
    Eigen::MatrixXd BThetaStar = Eigen::MatrixXd::Zero(this->numberOfDegreesOfFreedom, 3);
    Eigen::VectorXd CThetaStar = Eigen::VectorXd::Zero(this->numberOfDegreesOfFreedom);

    this->computeMTheta(MTheta);
    this->computeAThetaStar(AThetaStar);
    this->computeBThetaStar(BThetaStar);
    this->computeCThetaStar(CThetaStar, g_N);

    this->ATheta = MTheta.inverse() * AThetaStar;
    this->BTheta = MTheta.inverse() * BThetaStar;
    this->CTheta = MTheta.inverse() * CThetaStar;

    this->computeBackSubMatrices(backSubContr);
    this->computeBackSubVectors(backSubContr);
}

void SpinningBodyNDOFStateEffector::computeMTheta(Eigen::MatrixXd& MTheta)
{
    for (int n = 0; n<this->numberOfDegreesOfFreedom; n++) {
        this->spinningBodyVec[n].omega_SN_B = this->spinningBodyVec[n].omega_SB_B + this->omega_BN_B;

        for (int i = 0; i<this->numberOfDegreesOfFreedom; i++) {
            // Remove cross-coupling terms when axis is locked
            if ((this->spinningBodyVec[n].isAxisLocked || this->spinningBodyVec[i].isAxisLocked) && n != i)
                continue;

            for (int j = (i<=n) ? n : i; j<this->numberOfDegreesOfFreedom; j++) {
                Eigen::Vector3d r_ScjSn_B = this->spinningBodyVec[j].r_ScB_B - this->spinningBodyVec[n].r_SB_B;
                Eigen::Matrix3d rTilde_ScjSn_B = eigenTilde(r_ScjSn_B);
                Eigen::Vector3d r_ScjSi_B = this->spinningBodyVec[j].r_ScB_B - this->spinningBodyVec[i].r_SB_B;
                Eigen::Matrix3d rTilde_ScjSi_B = eigenTilde(r_ScjSi_B);

                MTheta(n,i) += this->spinningBodyVec[n].sHat_B.transpose()
                               * (this->spinningBodyVec[j].ISPntSc_B
                                  - this->spinningBodyVec[j].mass * rTilde_ScjSn_B * rTilde_ScjSi_B)
                               * this->spinningBodyVec[i].sHat_B;
            }
        }
    }
}

void SpinningBodyNDOFStateEffector::computeAThetaStar(Eigen::MatrixXd& AThetaStar)
{
    for (int n = 0; n<this->numberOfDegreesOfFreedom; n++) {
        if (this->spinningBodyVec[n].isAxisLocked)
            continue;

        for (int i = n; i<this->numberOfDegreesOfFreedom; i++) {
            Eigen::Vector3d r_SciSn_B = this->spinningBodyVec[i].r_ScB_B - this->spinningBodyVec[n].r_SB_B;
            Eigen::Matrix3d rTilde_SciSn_B = eigenTilde(r_SciSn_B);

            AThetaStar.row(n) -= this->spinningBodyVec[n].sHat_B.transpose() * this->spinningBodyVec[i].mass * rTilde_SciSn_B;
        }
    }
}

void SpinningBodyNDOFStateEffector::computeBThetaStar(Eigen::MatrixXd& BThetaStar)
{
    for (int n = 0; n<this->numberOfDegreesOfFreedom; n++) {
        if (this->spinningBodyVec[n].isAxisLocked)
            continue;

        for (int i = n; i < this->numberOfDegreesOfFreedom; i++) {
            Eigen::Vector3d r_SciSn_B = this->spinningBodyVec[i].r_ScB_B - this->spinningBodyVec[n].r_SB_B;
            Eigen::Matrix3d rTilde_SciSn_B = eigenTilde(r_SciSn_B);
            Eigen::Matrix3d rTilde_SciB_B = eigenTilde(this->spinningBodyVec[i].r_ScB_B);

            BThetaStar.row(n) -= this->spinningBodyVec[n].sHat_B.transpose() * (this->spinningBodyVec[i].ISPntSc_B
                                                                                - this->spinningBodyVec[i].mass *
                                                                                  rTilde_SciSn_B * rTilde_SciB_B);
        }
    }
}

void SpinningBodyNDOFStateEffector::computeCThetaStar(Eigen::VectorXd& CThetaStar,
                                                      const Eigen::Vector3d& g_N)
{
    Eigen::Vector3d g_B = this->dcm_BN * g_N;

    for (int n = 0; n<this->numberOfDegreesOfFreedom; n++) {
        if (this->spinningBodyVec[n].isAxisLocked)
            continue;

        CThetaStar(n) += this->spinningBodyVec[n].u
                - this->spinningBodyVec[n].k * (this->spinningBodyVec[n].theta - this->spinningBodyVec[n].thetaRef)
                - this->spinningBodyVec[n].c * (this->spinningBodyVec[n].thetaDot - this->spinningBodyVec[n].thetaDotRef);

        for (int i = n; i<this->numberOfDegreesOfFreedom; i++) {
            Eigen::Vector3d r_SciSn_B = this->spinningBodyVec[i].r_ScB_B - this->spinningBodyVec[n].r_SB_B;
            Eigen::Matrix3d rTilde_SciSn_B = eigenTilde(r_SciSn_B);
            Eigen::Matrix3d omegaTilde_SiN_B = eigenTilde(this->spinningBodyVec[i].omega_SN_B);

            CThetaStar(n) -= this->spinningBodyVec[n].sHat_B.transpose() * (
                    omegaTilde_SiN_B * this->spinningBodyVec[i].ISPntSc_B * this->spinningBodyVec[i].omega_SN_B
                    - this->spinningBodyVec[i].ISPntSc_B * this->spinningBodyVec[i].omegaTilde_SB_B * this->omega_BN_B
                    + this->spinningBodyVec[i].mass * rTilde_SciSn_B * (
                            - g_B
                            + this->omegaTilde_BN_B * this->omegaTilde_BN_B * this->spinningBodyVec[i].r_ScB_B
                            + 2 * this->omegaTilde_BN_B * this->spinningBodyVec[i].rPrime_ScB_B
                            + this->spinningBodyVec[i].omegaTilde_SP_B * this->spinningBodyVec[i].rPrime_ScS_B));

            for(int j=0; j<=i-1; j++) {
                Eigen::Vector3d omega_SiSj_B = this->spinningBodyVec[i].omega_SB_B - this->spinningBodyVec[j].omega_SB_B;
                Eigen::Matrix3d omegaTilde_SiSj_B = eigenTilde(omega_SiSj_B);
                Eigen::Vector3d r_SciSj1 = this->spinningBodyVec[i].r_ScB_B - this->spinningBodyVec[j+1].r_SB_B;
                Eigen::Matrix3d rTilde_SciSj1 = eigenTilde(r_SciSj1);
                Eigen::Vector3d rPrime_SciSj_B = this->spinningBodyVec[i].rPrime_ScB_B - this->spinningBodyVec[j].rPrime_SB_B;

                CThetaStar(n) -= this->spinningBodyVec[n].sHat_B.transpose() * (
                        - this->spinningBodyVec[i].ISPntSc_B * omegaTilde_SiSj_B * this->spinningBodyVec[j].omega_SP_B
                        + this->spinningBodyVec[i].mass * rTilde_SciSn_B * (
                                this->spinningBodyVec[j].omegaTilde_SP_B * rPrime_SciSj_B
                                - rTilde_SciSj1 * this->spinningBodyVec[j].omegaTilde_SB_B * this->spinningBodyVec[j+1].omega_SP_B));
            }
        }
    }
}

void SpinningBodyNDOFStateEffector::computeBackSubMatrices(BackSubMatrices& backSubContr) const
{
    for (int i = 0; i<this->numberOfDegreesOfFreedom; i++) {
        for (int j = i; j < this->numberOfDegreesOfFreedom; j++) {
            Eigen::Vector3d r_ScjSi = this->spinningBodyVec[j].r_ScB_B - this->spinningBodyVec[i].r_SB_B;
            Eigen::Matrix3d rTilde_ScjSi = eigenTilde(r_ScjSi);
            Eigen::Matrix3d rTilde_ScjB = eigenTilde(this->spinningBodyVec[j].r_ScB_B);

            backSubContr.matrixA -= this->spinningBodyVec[j].mass * rTilde_ScjSi * this->spinningBodyVec[i].sHat_B * this->ATheta.row(i);
            backSubContr.matrixB -= this->spinningBodyVec[j].mass * rTilde_ScjSi * this->spinningBodyVec[i].sHat_B * this->BTheta.row(i);
            backSubContr.matrixC += (this->spinningBodyVec[j].ISPntSc_B
                                     - this->spinningBodyVec[j].mass * rTilde_ScjB * rTilde_ScjSi)
                                    * this->spinningBodyVec[i].sHat_B * this->ATheta.row(i);
            backSubContr.matrixD += (this->spinningBodyVec[j].ISPntSc_B
                                     - this->spinningBodyVec[j].mass * rTilde_ScjB * rTilde_ScjSi)
                                    * this->spinningBodyVec[i].sHat_B * this->BTheta.row(i);
        }
    }
}

void SpinningBodyNDOFStateEffector::computeBackSubVectors(BackSubMatrices &backSubContr) const
{
    for (int i = 0; i<this->numberOfDegreesOfFreedom; i++) {
        Eigen::Matrix3d omegaTilde_SiN_B = eigenTilde(this->spinningBodyVec[i].omega_SN_B);
        backSubContr.vecRot -= omegaTilde_SiN_B * this->spinningBodyVec[i].ISPntSc_B * this->spinningBodyVec[i].omega_SB_B
                + this->spinningBodyVec[i].mass * this->omegaTilde_BN_B * this->spinningBodyVec[i].rTilde_ScB_B * this->spinningBodyVec[i].rPrime_ScB_B;

        for(int j=0; j<=i-1; j++) {
            Eigen::Vector3d omega_SiSj_B = this->spinningBodyVec[i].omega_SB_B - this->spinningBodyVec[j].omega_SB_B;
            Eigen::Matrix3d omegaTilde_SiSj_B = eigenTilde(omega_SiSj_B);
            Eigen::Vector3d r_SciSj1 = this->spinningBodyVec[i].r_ScB_B - this->spinningBodyVec[j+1].r_SB_B;
            Eigen::Matrix3d rTilde_SciSj1 = eigenTilde(r_SciSj1);
            Eigen::Vector3d rPrime_SciSj_B = this->spinningBodyVec[i].rPrime_ScB_B - this->spinningBodyVec[j].rPrime_SB_B;

            backSubContr.vecTrans -= this->spinningBodyVec[i].mass * (this->spinningBodyVec[j].omegaTilde_SP_B * rPrime_SciSj_B
                    - rTilde_SciSj1 * this->spinningBodyVec[j].omegaTilde_SB_B * this->spinningBodyVec[j+1].omega_SP_B);
            backSubContr.vecRot -= - this->spinningBodyVec[i].ISPntSc_B * omegaTilde_SiSj_B * this->spinningBodyVec[j].omega_SP_B
                    + this->spinningBodyVec[i].mass * this->spinningBodyVec[i].rTilde_ScB_B * (
                        this->spinningBodyVec[j].omegaTilde_SP_B * rPrime_SciSj_B
                        - rTilde_SciSj1 * this->spinningBodyVec[j].omegaTilde_SB_B * this->spinningBodyVec[j+1].omega_SP_B);
        }
        backSubContr.vecTrans -= this->spinningBodyVec[i].mass * this->spinningBodyVec[i].omegaTilde_SP_B * this->spinningBodyVec[i].rPrime_ScS_B;
        backSubContr.vecRot -= this->spinningBodyVec[i].mass * this->spinningBodyVec[i].rTilde_ScB_B * this->spinningBodyVec[i].omegaTilde_SP_B * this->spinningBodyVec[i].rPrime_ScS_B;

        for (int j = i; j < this->numberOfDegreesOfFreedom; j++) {
            Eigen::Vector3d r_ScjSi = this->spinningBodyVec[j].r_ScB_B - this->spinningBodyVec[i].r_SB_B;
            Eigen::Matrix3d rTilde_ScjSi = eigenTilde(r_ScjSi);
            Eigen::Matrix3d rTilde_ScjB = eigenTilde(this->spinningBodyVec[j].r_ScB_B);

            backSubContr.vecTrans += this->spinningBodyVec[j].mass * rTilde_ScjSi * this->spinningBodyVec[i].sHat_B * this->CTheta.row(i);
            backSubContr.vecRot -= (this->spinningBodyVec[j].ISPntSc_B
                                    - this->spinningBodyVec[j].mass * rTilde_ScjB * rTilde_ScjSi)
                                   * this->spinningBodyVec[i].sHat_B * this->CTheta.row(i);
        }
    }
}

void SpinningBodyNDOFStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    Eigen::Vector3d rDDotLocal_BN_B = this->dcm_BN * rDDot_BN_N;

    Eigen::VectorXd thetaDDot = this->ATheta * rDDotLocal_BN_B + this->BTheta * omegaDot_BN_B + this->CTheta;
    this->thetaState->setDerivative(this->thetaDotState->getState());
    this->thetaDotState->setDerivative(thetaDDot);
}

void SpinningBodyNDOFStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    rotAngMomPntCContr_B = Eigen::Vector3d::Zero();
    rotEnergyContr = 0.0;

    for(auto& spinningBody: this->spinningBodyVec) {
        spinningBody.omega_SN_B = spinningBody.omega_SB_B + this->omega_BN_B;
        spinningBody.rDot_ScB_B = spinningBody.rPrime_ScB_B + this->omegaTilde_BN_B * spinningBody.r_ScB_B;

        rotAngMomPntCContr_B += spinningBody.ISPntSc_B * spinningBody.omega_SN_B + spinningBody.mass * spinningBody.rTilde_ScB_B * spinningBody.rDot_ScB_B;
        rotEnergyContr += 1.0 / 2.0 * spinningBody.omega_SN_B.dot(spinningBody.ISPntSc_B * spinningBody.omega_SN_B)
                        + 1.0 / 2.0 * spinningBody.mass * spinningBody.rDot_ScB_B.dot(spinningBody.rDot_ScB_B)
                        + 1.0 / 2.0 * spinningBody.k * (spinningBody.theta - spinningBody.thetaRef) * (spinningBody.theta - spinningBody.thetaRef);
    }
}

void SpinningBodyNDOFStateEffector::computeSpinningBodyInertialStates()
{
    for(auto& spinningBody: this->spinningBodyVec) {
        // Compute the rotational properties
        Eigen::Matrix3d dcm_SN = spinningBody.dcm_BS.transpose() * this->dcm_BN;
        spinningBody.sigma_SN = eigenMRPd2Vector3d(eigenC2MRP(dcm_SN));
        spinningBody.omega_SN_S = spinningBody.dcm_BS.transpose() * spinningBody.omega_SN_B;

        // Compute the translation properties
        spinningBody.r_ScN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * spinningBody.r_ScB_B;
        spinningBody.v_ScN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * spinningBody.rDot_ScB_B;
    }
}

void SpinningBodyNDOFStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    this->computeSpinningBodyInertialStates();
    this->writeOutputStateMessages(CurrentSimNanos);
}
