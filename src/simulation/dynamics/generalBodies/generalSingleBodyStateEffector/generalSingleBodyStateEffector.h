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

#ifndef GENERAL_SINGLE_BODY_STATE_EFFECTOR_H
#define GENERAL_SINGLE_BODY_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ArrayMotorForceMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/LinearTranslationRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

struct DOF {
public:
    enum class Type { ROTATION, TRANSLATION };

    void setDOFAxis(Eigen::Vector3d axis_G) {this->axis_G = axis_G;};
    void setBetaInit(double betaInit) {this->betaInit = betaInit;};
    void setBetaDotInit(double betaDotInit) {this->betaDotInit = betaDotInit;};
    void setSpringConstantK(double k) {this->k = k;};
    void setDampingConstantC(double c) {this->c = c;};
    void setScrewConstant(double screwConstant) {this->screwConstant = screwConstant;};

    Type getDOFType() const {return this->type;};
    Eigen::Vector3d getDOFAxis() const {return this->axis_G;};
    double getBetaInit() const {return this->betaInit;};
    double getBetaDotInit() const {return this->betaDotInit;};
    double getSpringConstantK() const {return this->k;};
    double getDampingConstantC() const {return this->c;};
    double getScrewConstant() const {return this->screwConstant;};

private:
    friend class GeneralSingleBodyStateEffector;

    Type type{Type::ROTATION};
    Eigen::Vector3d axis_G{1.0, 0.0, 0.0};
    double betaInit{};
    double betaDotInit{};
    double k{};
    double c{};
    double screwConstant{};

    uint64_t index{};
    double beta{};
    double betaDot{};
    double betaRef{};
    double betaDotRef{};
    Eigen::Vector3d r_GB_B{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_GB_B{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d dcm_GB{Eigen::Matrix3d::Identity()};
    double u{};
    double f{};
};

/*! @brief General rigid body state effector class */
class GeneralSingleBodyStateEffector: public StateEffector, public SysModel {
public:
    GeneralSingleBodyStateEffector();
    ~GeneralSingleBodyStateEffector();

    void setMass(const double mass) {this->mass = mass;};
    void setIPntGc_G(const Eigen::Matrix3d IPntGc_G) {this->IPntGc_G = IPntGc_G;};
    void setR_GcG_G(const Eigen::Vector3d r_GcG_G) {this->r_GcG_G = r_GcG_G;};
    void setR_G0B_B(Eigen::Vector3d r_G0B_B) {this->r_G0B_B = r_G0B_B;};
    void setDCM_G0B(Eigen::Matrix3d dcm_G0B) {this->dcm_G0B = dcm_G0B;};
    double getMass() const {return this->mass;};
    const Eigen::Matrix3d getIPntGc_G() const {return this->IPntGc_G;};
    const Eigen::Vector3d getR_GcG_G() const {return this->r_GcG_G;};
    Eigen::Vector3d getR_G0B_B() const {return this->r_G0B_B;};
    Eigen::Matrix3d getDCM_G0B() const {return this->dcm_G0B;};

    void addRotDOF(std::shared_ptr<DOF> newDOF);
    void addTransDOF(std::shared_ptr<DOF> newDOF);
    std::shared_ptr<DOF> getDOF(uint64_t index) {return this->jointDOFList.at(index);};
    void clearDOFs();

    void Reset(uint64_t currentSimNanos) override;
    void registerStates(DynParamManager& statesIn) override;
    void linkInStates(DynParamManager& states) override;
    void UpdateState(uint64_t currentSimNanos) override;
    void computeGeneralBodyInertialStates();
    void writeOutputStateMessages(uint64_t currentSimNanos) override;
    void updateEffectorMassProps(double integTime) override;
    void updateContributions(double integTime,
                             BackSubMatrices & backSubContr,
                             Eigen::MRPd sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::MRPd sigma_BN) override;
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d & rotAngMomPntCContr_B,
                                      double & rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override;

    std::vector<ReadFunctor<HingedRigidBodyMsgPayload>> spinningBodyRefInMsg;
    std::vector<ReadFunctor<ArrayMotorTorqueMsgPayload>> motorTorqueInMsg;
    std::vector<ReadFunctor<LinearTranslationRigidBodyMsgPayload>> translatingBodyRefInMsgs;
    std::vector<ReadFunctor<ArrayMotorForceMsgPayload>> motorForceInMsg;

    std::vector<Message<HingedRigidBodyMsgPayload>*> spinningBodyOutMsgs;
    std::vector<Message<LinearTranslationRigidBodyMsgPayload>*> translatingBodyOutMsgs;
    Message<SCStatesMsgPayload> generalSingleBodyConfigLogOutMsg;

private:
    double mass{};
    Eigen::Matrix3d IPntGc_G{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d r_GcG_G{Eigen::Vector3d::Zero()};
    Eigen::Vector3d r_G0B_B{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d dcm_G0B{Eigen::Matrix3d::Identity()};
    std::vector<std::shared_ptr<DOF>> jointDOFList;

    uint64_t numDOF{};
    uint64_t numRotDOF{};
    uint64_t numTransDOF{};
    std::vector<double> betaInitList;
    std::vector<double> betaDotInitList;

    Eigen::VectorXd beta;
    Eigen::VectorXd betaDot;
    StateData* betaState = nullptr;
    StateData* betaDotState = nullptr;
    Eigen::MatrixXd* inertialPositionProperty = nullptr;
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;

    Eigen::MatrixXd TMat;
    Eigen::MatrixXd TPrimeMat;
    Eigen::MatrixXd UMat;
    Eigen::VectorXd vVec{Eigen::VectorXd::Zero(6)};
    Eigen::Vector3d r_GB_B{Eigen::Vector3d::Zero()};
    Eigen::Vector3d r_GcN_N{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_GcN_N{Eigen::Vector3d::Zero()};
    Eigen::Vector3d sigma_GN{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_GN_G{Eigen::Vector3d::Zero()};

    Eigen::MatrixXd ABeta;
    Eigen::MatrixXd BBeta;
    Eigen::VectorXd CBeta;

    Eigen::Matrix3d dcm_BN{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d omega_BN_B{Eigen::Vector3d::Zero()};

    static uint64_t effectorID;
    BSKLogger bskLogger;

    std::string nameOfBetaState;
    std::string nameOfBetaDotState;

    void addDOF(std::shared_ptr<DOF> newDOF);
    bool isDOFAdded(const std::shared_ptr<DOF>& newDOF) const;
};

#endif /* GENERAL_SINGLE_BODY_STATE_EFFECTOR_H */
