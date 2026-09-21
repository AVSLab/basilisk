/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef ROT_TRANS_TWO_DOF_STATE_EFFECTOR_H
#define ROT_TRANS_TWO_DOF_STATE_EFFECTOR_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorForceMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/LinearTranslationRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"

class RotTransTwoDOFStateEffector final: public StateEffector, public SysModel {
public:
    // Setters for the first body
    void setMass1(const double mass1) {this->mass1 = mass1;};
    void setIPntSc_S(const Eigen::Matrix3d ISPntSc_S) {this->ISPntSc_S = ISPntSc_S;};
    void setR_ScS_S(const Eigen::Vector3d r_ScS_S) {this->r_ScS_S = r_ScS_S;};
    void setR_SB_B(Eigen::Vector3d r_SB_B) {this->r_SB_B = r_SB_B;};
    void setDCM_S0B(Eigen::Matrix3d dcm_S0B) {this->dcm_S0B = dcm_S0B;};
    void setSHat_S(Eigen::Vector3d sHat_S) {this->sHat_S = sHat_S;};
    void setThetaInit(const double thetaInit) {this->thetaInit = thetaInit;};
    void setThetaDotInit(const double thetaDotInit) {this->thetaDotInit = thetaDotInit;};
    void setK1(const double k1) {this->k1 = k1;};
    void setC1(const double c1) {this->c1 = c1;};

    // Setters for the second body
    void setMass2(const double mass2) {this->mass2 = mass2;};
    void setITPntTc_T(const Eigen::Matrix3d ITPntTc_T) {this->ITPntTc_T = ITPntTc_T;};
    void setR_TcT_T(const Eigen::Vector3d r_TcT_T) {this->r_TcT_T = r_TcT_T;};
    void setR_T0S_S(Eigen::Vector3d r_T0S_S) {this->r_T0S_S = r_T0S_S;};
    void setDCM_TS(Eigen::Matrix3d dcm_TS) {this->dcm_TS = dcm_TS;};
    void setTHat_T(Eigen::Vector3d tHat_T) {this->tHat_T = tHat_T;};
    void setRhoInit(const double rhoInit) {this->rhoInit = rhoInit;};
    void setRhoDotInit(const double rhoDotInit) {this->rhoDotInit = rhoDotInit;};
    void setK2(const double k2) {this->k2 = k2;};
    void setC2(const double c2) {this->c2 = c2;};

    // Getters for the first body
    double getMass1() const {return this->mass1;};
    const Eigen::Matrix3d getIPntSc_S() const {return this->ISPntSc_S;};
    const Eigen::Vector3d getR_ScS_S() const {return this->r_ScS_S;};
    Eigen::Vector3d getR_SB_B() const {return this->r_SB_B;};
    Eigen::Matrix3d getDCM_S0B() const {return this->dcm_S0B;};
    Eigen::Vector3d getSHat_S() const {return this->sHat_S;};
    double getThetaInit() const {return this->thetaInit;};
    double getThetaDotInit() const {return this->thetaDotInit;};
    double getK1() const {return this->k1;};
    double getC1() const {return this->c1;};

    // Getters for the second body
    double getMass2() const {return this->mass2;};
    const Eigen::Matrix3d getITPntTc_T() const {return this->ITPntTc_T;};
    const Eigen::Vector3d getR_TcT_T() const {return this->r_TcT_T;};
    Eigen::Vector3d getR_T0S_S() const {return this->r_T0S_S;};
    Eigen::Matrix3d getDCM_TS() const {return this->dcm_TS;};
    Eigen::Vector3d getTHat_T() const {return this->tHat_T;};
    double getRhoInit() const {return this->rhoInit;};
    double getRhoDotInit() const {return this->rhoDotInit;};
    double getK2() const {return this->k2;};
    double getC2() const {return this->c2;};

    std::string nameOfThetaState;
    std::string nameOfThetaDotState;
    std::string nameOfRhoState;
    std::string nameOfRhoDotState;

    ReadFunctor<HingedRigidBodyMsgPayload> spinningBodyRefInMsg;
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorqueInMsg;
    ReadFunctor<LinearTranslationRigidBodyMsgPayload> translatingBodyRefInMsg;
    ReadFunctor<ArrayMotorForceMsgPayload> motorForceInMsg;

    Message<HingedRigidBodyMsgPayload> spinningBodyOutMsg;
    Message<LinearTranslationRigidBodyMsgPayload> translatingBodyOutMsg;
    std::vector<Message<SCStatesMsgPayload>*> bodyConfigLogOutMsgs {new Message<SCStatesMsgPayload>,
            new Message<SCStatesMsgPayload>};

    RotTransTwoDOFStateEffector();
    ~RotTransTwoDOFStateEffector() = default;
    void Reset(uint64_t CurrentClock) override;
    void writeOutputStateMessages(uint64_t CurrentClock) override;
    void UpdateState(uint64_t CurrentSimNanos) override;
    void registerStates(DynParamManager& statesIn) override;
    void linkInStates(DynParamManager& states) override;
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr,
                             Eigen::MRPd sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::MRPd sigma_BN) override;
    void updateEffectorMassProps(double integTime) override;
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d& rotAngMomPntCContr_B,
                                      double& rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override;
    void computeBodyInertialStates();

private:

    double mass1{};
    Eigen::Matrix3d ISPntSc_S{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d r_ScS_S{Eigen::Vector3d::Zero()};
    Eigen::Vector3d r_SB_B{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d dcm_S0B{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d sHat_S{1.0,0.0,0.0};
    double thetaInit{};
    double thetaDotInit{};
    double k1{};
    double c1{};

    double mass2{1.0};
    Eigen::Matrix3d ITPntTc_T{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d r_TcT_T{Eigen::Vector3d::Zero()};
    Eigen::Vector3d r_T0S_S{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d dcm_TS{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d tHat_T{1.0,0.0,0.0};
    double rhoInit{};
    double rhoDotInit{};
    double k2{};
    double c2{};


    static uint64_t effectorID;
    double u = 0.0;
    double f = 0.0;
    double thetaRef = 0.0;
    double thetaDotRef = 0.0;
    double rhoRef = 0.0;
    double rhoDotRef = 0.0;
    double mass = 1.0;

    Eigen::Matrix<double, 2, 3> ABeta;
    Eigen::Matrix<double, 2, 3> BBeta;
    Eigen::Vector2d CBeta{0.0,0.0};

    Eigen::Matrix3d dcm_BN{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d dcm_BS{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d omega_BN_B{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rDot_ScB_B{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rDot_TcB_B{Eigen::Vector3d::Zero()};

    Eigen::Vector3d r_ScN_N{Eigen::Vector3d::Zero()};
    Eigen::Vector3d r_TcN_N{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_ScN_N{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_TcN_N{Eigen::Vector3d::Zero()};
    Eigen::Vector3d sigma_SN{Eigen::Vector3d::Zero()};
    Eigen::Vector3d sigma_TN{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_SN_S{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_TN_T{Eigen::Vector3d::Zero()};

    double theta = 0.0;
    double thetaDot = 0.0;
    double rho = 0.0;
    double rhoDot = 0.0;
    Eigen::MatrixXd* inertialPositionProperty = nullptr;
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;
    StateData* thetaState = nullptr;
    StateData* thetaDotState = nullptr;
    StateData* rhoState = nullptr;
    StateData* rhoDotState = nullptr;
};

#endif /* ROT_TRANS_TWO_DOF_STATE_EFFECTOR_H */
