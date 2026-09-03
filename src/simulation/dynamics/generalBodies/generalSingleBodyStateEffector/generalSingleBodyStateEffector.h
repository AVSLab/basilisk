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
    /** Enumeration indicating the joint DOF type */
    enum class Type { ROTATION, TRANSLATION };

    /** Setter for `axis_G` property */
    void setDOFAxis(Eigen::Vector3d axis_G) {this->axis_G = axis_G;};
    /** Setter for `betaInit` property */
    void setBetaInit(double betaInit) {this->betaInit = betaInit;};
    /** Setter for `betaDotInit` property */
    void setBetaDotInit(double betaDotInit) {this->betaDotInit = betaDotInit;};
    /** Setter for `k` property */
    void setSpringConstantK(double k) {this->k = k;};
    /** Setter for `c` property */
    void setDampingConstantC(double c) {this->c = c;};
    /** Setter for `screwConstant` property */
    void setScrewConstant(double screwConstant) {this->screwConstant = screwConstant;};

    /** Getter for `type` property */
    Type getDOFType() const {return this->type;};
    /** Getter for `axis_G` property */
    Eigen::Vector3d getDOFAxis() const {return this->axis_G;};
    /** Getter for `betaInit` property */
    double getBetaInit() const {return this->betaInit;};
    /** Getter for `betaDotInit` property */
    double getBetaDotInit() const {return this->betaDotInit;};
    /** Getter for `k` property */
    double getSpringConstantK() const {return this->k;};
    /** Getter for `c` property */
    double getDampingConstantC() const {return this->c;};
    /** Getter for `screwConstant` property */
    double getScrewConstant() const {return this->screwConstant;};

private:
    friend class GeneralSingleBodyStateEffector;

    Type type{Type::ROTATION};  //!< [-] DOF type
    Eigen::Vector3d axis_G{1.0, 0.0, 0.0};  //!< [-] DOF axis
    double betaInit{};  //!< [-] Initial DOF state
    double betaDotInit{};  //!< [-] Initial DOF ratc
    double k{};  //!< [N * m / rad] DOF spring constant
    double c{};  //!< [N * m * s / rad] DOF damping constant
    double screwConstant{};  //!< [-] DOF screw Constant

    uint64_t index{};  //!< [-] DOF index
    double beta{};  //!< [-] DOF state
    double betaDot{};  //!< [-] DOF rate
    double betaRef{};  //!< [-] DOF reference state
    double betaDotRef{};  //!< [-] DOF reference rate
    Eigen::Vector3d r_GB_B{Eigen::Vector3d::Zero()};  //!< [m] Position vector of joint frame G relative to the hub frame B
    Eigen::Vector3d omega_GB_B{Eigen::Vector3d::Zero()};  //!< [rad/s] Angular velocity vector of the joint body frame G relative to the hub frame B
    Eigen::Matrix3d dcm_GB{Eigen::Matrix3d::Identity()};  //!< [-] Initial effector body frame G attitude relative to the hub frame B
    double u{};  //!< [N * m] Joint DOF torque
    double f{};  //!< [N] Joint DOF force
};

/*! @brief General rigid body state effector class */
class GeneralSingleBodyStateEffector: public StateEffector, public SysModel {
public:
    GeneralSingleBodyStateEffector();  //!< Class constructor
    ~GeneralSingleBodyStateEffector();  //!< Class destructor

    /** Setter for `mass` property */
    void setMass(const double mass) {this->mass = mass;};
    /** Setter for `IPntGc_G` property */
    void setIPntGc_G(const Eigen::Matrix3d IPntGc_G) {this->IPntGc_G = IPntGc_G;};
    /** Setter for `r_GcG_G` property */
    void setR_GcG_G(const Eigen::Vector3d r_GcG_G) {this->r_GcG_G = r_GcG_G;};
    /** Setter for `r_G0B_B` property */
    void setR_G0B_B(Eigen::Vector3d r_G0B_B) {this->r_G0B_B = r_G0B_B;};
    /** Setter for `dcm_G0B` property */
    void setDCM_G0B(Eigen::Matrix3d dcm_G0B) {this->dcm_G0B = dcm_G0B;};

    /** Getter for `mass` property */
    double getMass() const {return this->mass;};
    /** Getter for `IPntGc_G` property */
    const Eigen::Matrix3d getIPntGc_G() const {return this->IPntGc_G;};
    /** Getter for `r_GcG_G` property */
    const Eigen::Vector3d getR_GcG_G() const {return this->r_GcG_G;};
    /** Getter for `r_G0B_B` property */
    Eigen::Vector3d getR_G0B_B() const {return this->r_G0B_B;};
    /** Getter for `dcm_G0B` property */
    Eigen::Matrix3d getDCM_G0B() const {return this->dcm_G0B;};

    /** Method for adding a new sequential rotational degree of freedom */
    void addRotDOF(std::shared_ptr<DOF> newDOF);
    /** Method for adding a new sequential translational degree of freedom */
    void addTransDOF(std::shared_ptr<DOF> newDOF);
    /** Getter for a specific sequential degree of freedom */
    std::shared_ptr<DOF> getDOF(uint64_t index) {return this->jointDOFList.at(index);};
    /** Method to clear all degrees of freedom */
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

    std::vector<ReadFunctor<HingedRigidBodyMsgPayload>> spinningBodyRefInMsg;  //!< Input message for rotational degree of freedom reference angles
    std::vector<ReadFunctor<ArrayMotorTorqueMsgPayload>> motorTorqueInMsg;  //!< Input message for rotational degree of freedom motor torque
    std::vector<ReadFunctor<LinearTranslationRigidBodyMsgPayload>> translatingBodyRefInMsgs;  //!< Input message for translational degree of freedom reference displacements
    std::vector<ReadFunctor<ArrayMotorForceMsgPayload>> motorForceInMsg;  //!< Input message for translational degree of freedom motor force
    std::vector<Message<HingedRigidBodyMsgPayload>*> spinningBodyOutMsgs;  //!< Output message for rotational degree of freedom scalar states
    std::vector<Message<LinearTranslationRigidBodyMsgPayload>*> translatingBodyOutMsgs;  //!< Output message for translational degree of freedom scalar states
    Message<SCStatesMsgPayload> generalSingleBodyConfigLogOutMsg;  //!< Output message for the effector inertial states

private:
    double mass{};  //!< [kg] General body mass
    Eigen::Matrix3d IPntGc_G{Eigen::Matrix3d::Identity()};    //!< [kg * m^2] Effector inertia
    Eigen::Vector3d r_GcG_G{Eigen::Vector3d::Zero()};  //!< [m] Position vector of the effector center of mass point Gc relative to its body frame origin point G
    Eigen::Vector3d r_G0B_B{Eigen::Vector3d::Zero()};  //!< [m] Initial position vector of the effector body frame origin point G relative to the hub frame origin point B
    Eigen::Matrix3d dcm_G0B{Eigen::Matrix3d::Identity()};  //!< [-] Initial effector body frame G attitude relative to the hub frame B
    std::vector<std::shared_ptr<DOF>> jointDOFList;  //!< [-] Effector sequential DOF list

    uint64_t numDOF{};  //!< [-] Total number of effector degrees of freedom
    uint64_t numRotDOF{};  //!< [-] Total number of effector rotational degrees of freedom
    uint64_t numTransDOF{};  //!< [-] Total number of effector translational degrees of freedom
    std::vector<double> betaInitList;   //!< [-] Initial state vector
    std::vector<double> betaDotInitList;   //!< [-] Initial state vector rate

    Eigen::VectorXd beta;   //!< [-] Effector state vector
    Eigen::VectorXd betaDot;   //!< [-] Effector state vector rate
    StateData* betaState = nullptr;  //!< [-] State data container for the effector state vector
    StateData* betaDotState = nullptr;  //!< [-] State data container for the effector state vector rate
    Eigen::MatrixXd* inertialPositionProperty = nullptr;  //!< [m] Identifier for the hub inertial position
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;  //!< [m/s] Identifier for the hub inertial velocity

    Eigen::MatrixXd TMat;  //!< [-] Effector mapping vectrix
    Eigen::MatrixXd TPrimeMat;  //!< [-] Effector mapping vectrix rate
    Eigen::MatrixXd UMat;  //!< [-] Kinematics storage data matrix
    Eigen::VectorXd vVec{Eigen::VectorXd::Zero(6)}; //!< [-] Kinematics storage data vector
    Eigen::Vector3d r_GB_B{Eigen::Vector3d::Zero()};  //!< [m] Position vector of the effector body frame origin point G relative to the hub frame origin point B
    Eigen::Vector3d r_GcN_N{Eigen::Vector3d::Zero()};  //!< [m] Position vector of the effector center of mass point Gc relative to the inertial frame origin point N
    Eigen::Vector3d v_GcN_N{Eigen::Vector3d::Zero()};  //!< [m] Velocity vector of the effector center of mass point Gc relative to the inertial frame origin point N
    Eigen::Vector3d sigma_GN{Eigen::Vector3d::Zero()};  //!< [-] MRP attitude of the effector body frame G relative to the inertial frame N
    Eigen::Vector3d omega_GN_G{Eigen::Vector3d::Zero()};  //!< [rad/s] Angular velocity vector of the effector body frame G relative to the inertial frame N

    Eigen::MatrixXd ABeta;  //!< [-] Backsubstitution contribution [A]
    Eigen::MatrixXd BBeta;  //!< [-] Backsubstitution contribution [B]
    Eigen::VectorXd CBeta;  //!< [-] Backsubstitution contribution [C]

    Eigen::Matrix3d dcm_BN{Eigen::Matrix3d::Identity()};  //!< [-] Hub body frame B attitude relative to the inertial frame N
    Eigen::Vector3d omega_BN_B{Eigen::Vector3d::Zero()};  //!< [rad/s] Angular velocity vector of the hub body frame B relative to the inertial frame N

    static uint64_t effectorID;
    BSKLogger bskLogger;

    std::string nameOfBetaState;   //!< [-] String for the effector state vector name
    std::string nameOfBetaDotState;   //!< [-] String for the effector state vector rate name

    /** Internal method for setting up a new sequential degree of freedom */
    void addDOF(std::shared_ptr<DOF> newDOF);
    /** Internal method checks if a new degree of freedom has already been added */
    bool isDOFAdded(const std::shared_ptr<DOF>& newDOF) const;
    /** Internal method to compute all joint attitude properties */
    void computeAttitudeProperties();
    /** Internal method to compute all joint position properties */
    void computePositionProperties();
    /** Internal method to compute all joint angular velocity properties */
    void computeAngularVelocityProperties();
};

#endif /* GENERAL_SINGLE_BODY_STATE_EFFECTOR_H */
