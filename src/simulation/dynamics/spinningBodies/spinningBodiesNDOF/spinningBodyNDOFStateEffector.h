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

#ifndef SPINNING_BODY_N_DOF_STATE_EFFECTOR_H
#define SPINNING_BODY_N_DOF_STATE_EFFECTOR_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"

#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayEffectorLockMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

/*! Struct containing all the spinning bodies variables. */
struct SpinningBody {
public:
    /** setter for `mass` property */
    void setMass(double mass);
    /** setter for `r_SP_P` property */
    void setR_SP_P(Eigen::Vector3d r_SP_P) {this->r_SP_P = r_SP_P;};
    /** setter for `r_ScS_S` property */
    void setR_ScS_S(Eigen::Vector3d r_ScS_S) {this->r_ScS_S = r_ScS_S;};
    /** setter for `ISPntSc_S` property */
    void setISPntSc_S(const Eigen::Matrix3d& ISPntSc_S) {this->ISPntSc_S = ISPntSc_S;};
    /** setter for `sHat_S` property */
    void setSHat_S(Eigen::Vector3d sHat_S);
    /** setter for `dcm_S0P` property */
    void setDCM_S0P(const Eigen::Matrix3d& dcm_S0P) {this->dcm_S0P = dcm_S0P;};
    /** setter for `k` property */
    void setK(double k);
    /** setter for `c` property */
    void setC(double c);
    /** setter for `thetaInit` property */
    void setThetaInit(double thetaInit) {this->thetaInit = thetaInit;};
    /** setter for `thetaDotInit` property */
    void setThetaDotInit(double thetaDotInit) {this->thetaDotInit = thetaDotInit;};

    /** getter for `mass` property */
    double getMass() const {return this->mass;};
    /** getter for `r_SP_P` property */
    Eigen::Vector3d getR_SP_P() const {return this->r_SP_P;};
    /** getter for `r_ScS_S` property */
    Eigen::Vector3d getR_ScS_S() const {return this->r_ScS_S;};
    /** getter for `ISPntSc_S` property */
    Eigen::Matrix3d setISPntSc_S() const {return this->ISPntSc_S;};
    /** getter for `sHat_S` property */
    Eigen::Vector3d getSHat_S() const {return this->sHat_S;};
    /** getter for `dcm_S0P` property */
    Eigen::Matrix3d getDCM_S0P() const {return this->dcm_S0P;};
    /** getter for `k` property */
    double getK() const {return this->k;};
    /** getter for `c` property */
    double getC() const {return this->c;};
    /** getter for `thetaInit` property */
    double getThetaInit() const {return this->thetaInit;};
    /** getter for `thetaDotInit` property */
    double getThetaDotInit() const {return this->thetaDotInit;};

private:
    // Make sure the effector can access each spinning body's private and protected members
    friend class SpinningBodyNDOFStateEffector;

    double thetaInit = 0.0;           //!< [rad] initial spinning body angle
    double thetaDotInit = 0.0;        //!< [rad/s] initial spinning body angle rate
    double k = 0.0;                   //!< [N-m/rad] torsional spring constant
    double c = 0.0;                   //!< [N-m-s/rad] rotational damping coefficient
    double mass = 1.0;                //!< [kg] spinning body mass
    Eigen::Vector3d r_SP_P = Eigen::Vector3d::Zero();            //!< [m] vector pointing from parent frame P origin to spinning frame S origin in P frame components
    Eigen::Vector3d r_ScS_S = Eigen::Vector3d::Zero();           //!< [m] vector pointing from spinning frame S origin to point Sc (center of mass of the spinner) in S frame components
    Eigen::Vector3d sHat_S = {1.0, 0.0, 0.0};           //!< spinning axis in S frame components
    Eigen::Matrix3d dcm_S0P = Eigen::Matrix3d::Identity();       //!< DCM from the parent frame to the S0 frame (S frame for theta=0)
    Eigen::Matrix3d ISPntSc_S = Eigen::Matrix3d::Identity();     //!< [kg-m^2] Inertia of spinning body about point Sc in S frame components

    double theta = 0.0;       //!< [rad] current spinning body angle
    double thetaDot = 0.0;    //!< [rad/s] current spinning body angle rate
    double thetaRef = 0.0;    //!< [rad] reference spinning body angle
    double thetaDotRef = 0.0; //!< [rad/s] reference spinning body angle rate
    double u = 0.0;           //!< [N-m] initial spinning body angle
    bool isAxisLocked = false;       //!< axis lock flag

    Eigen::Vector3d sHat_B = {1.0, 0.0, 0.0};            //!< spinning axis in B frame components
    Eigen::Vector3d r_SP_B = Eigen::Vector3d::Zero();            //!< [m] vector pointing from parent frame P origin to spinning frame S origin in B frame components
    Eigen::Vector3d r_SB_B = Eigen::Vector3d::Zero();            //!< [m] vector pointing from body frame B origin to spinning frame S origin in B frame components
    Eigen::Vector3d r_ScS_B = Eigen::Vector3d::Zero();           //!< [m] vector pointing from spinning frame S origin to point Sc (center of mass of the spinner) in B frame components
    Eigen::Vector3d r_ScB_B = Eigen::Vector3d::Zero();           //!< [m] vector pointing from body frame B origin to point Sc (center of mass of the spinner) in B frame components
    Eigen::Vector3d rPrime_SP_B = Eigen::Vector3d::Zero();       //!< [m/s] body frame time derivative of r_SP_B in B frame components
    Eigen::Vector3d rPrime_SB_B = Eigen::Vector3d::Zero();       //!< [m/s] body frame time derivative of r_SB_B in B frame components
    Eigen::Vector3d rPrime_ScS_B = Eigen::Vector3d::Zero();      //!< [m/s] body frame time derivative of r_ScS_B in B frame components
    Eigen::Vector3d rPrime_ScB_B = Eigen::Vector3d::Zero();      //!< [m/s] body frame time derivative of r_ScB_B in B frame components
    Eigen::Vector3d rDot_ScB_B = Eigen::Vector3d::Zero();        //!< [m/s] inertial time derivative of r_ScB_B in B frame components
    Eigen::Vector3d omega_SP_B = Eigen::Vector3d::Zero();        //!< [rad/s] angular velocity of the S frame wrt the P frame in B frame components
    Eigen::Vector3d omega_SB_B = Eigen::Vector3d::Zero();        //!< [rad/s] angular velocity of the S frame wrt the B frame in B frame components
    Eigen::Vector3d omega_SN_B = Eigen::Vector3d::Zero();        //!< [rad/s] angular velocity of the S frame wrt the N frame in B frame components

    Eigen::Matrix3d ISPntSc_B = Eigen::Matrix3d::Identity();     //!< [kg-m^2] inertia of spinning body about point Sc in S frame components
    Eigen::Matrix3d IPrimeSPntSc_B = Eigen::Matrix3d::Zero();    //!< [kg-m^2] body frame derivative of the inertia of spinning body about point Sc in S frame components
    Eigen::Matrix3d dcm_BS = Eigen::Matrix3d::Identity();        //!< DCM from spinner frame to body frame
    Eigen::Matrix3d rTilde_ScB_B = Eigen::Matrix3d::Zero();      //!< [m] tilde matrix of r_ScB_B
    Eigen::Matrix3d omegaTilde_SP_B = Eigen::Matrix3d::Zero();   //!< [rad/s] tilde matrix of omega_SP_B
    Eigen::Matrix3d omegaTilde_SB_B = Eigen::Matrix3d::Zero();   //!< [rad/s] tilde matrix of omega_SB_B

    Eigen::Vector3d r_ScN_N = Eigen::Vector3d::Zero();           //!< [m] position vector of the spinning body center of mass Sc relative to the inertial frame origin N
    Eigen::Vector3d v_ScN_N = Eigen::Vector3d::Zero();           //!< [m/s] inertial velocity vector of Sc relative to inertial frame
    Eigen::Vector3d sigma_SN = Eigen::Vector3d::Zero();          //!< MRP attitude of frame S relative to inertial frame
    Eigen::Vector3d omega_SN_S = Eigen::Vector3d::Zero();        //!< [rad/s] inertial spinning body frame angular velocity vector

    BSKLogger bskLogger;
};

/*! spinning rigid body state effector class */
class SpinningBodyNDOFStateEffector: public StateEffector, public SysModel {
public:
    std::vector<Message<HingedRigidBodyMsgPayload>*> spinningBodyOutMsgs;       //!< state output message
    std::vector<Message<SCStatesMsgPayload>*> spinningBodyConfigLogOutMsgs;     //!< spinning body state config log message
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorqueInMsg;                   //!< (optional) motor torque input message
    ReadFunctor<ArrayEffectorLockMsgPayload> motorLockInMsg;                    //!< (optional) lock flag input message
    std::vector<ReadFunctor<HingedRigidBodyMsgPayload>> spinningBodyRefInMsgs;  //!< (optional) reference state input message

    SpinningBodyNDOFStateEffector();            //!< Constructor
    ~SpinningBodyNDOFStateEffector() override;  //!< Destructor

    /** method for adding a new spinning body */
    void addSpinningBody(SpinningBody const& newBody);
    /** setter for `nameOfThetaState` property */
    void setNameOfThetaState(const std::string& nameOfThetaState) {this->nameOfThetaState = nameOfThetaState;};
    /** setter for `nameOfThetaDotState` property */
    void setNameOfThetaDotState(const std::string& nameOfThetaDotState) {this->nameOfThetaDotState = nameOfThetaDotState;};
    /** getter for `nameOfThetaState` property */
    std::string getNameOfThetaState() const {return this->nameOfThetaState;};
    /** getter for `nameOfThetaDotState` property */
    std::string getNameOfThetaDotState() const {return this->nameOfThetaDotState;};

private:
    static uint64_t effectorID;

    int numberOfDegreesOfFreedom = 0;
    std::vector<SpinningBody> spinningBodyVec;

    Eigen::MatrixXd ATheta;
    Eigen::MatrixXd BTheta;
    Eigen::VectorXd CTheta;

    Eigen::Vector3d omega_BN_B = Eigen::Vector3d::Zero();
    Eigen::MRPd sigma_BN;
    Eigen::Matrix3d dcm_BN = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d omegaTilde_BN_B = Eigen::Matrix3d::Zero();

    Eigen::MatrixXd* inertialPositionProperty = nullptr;
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;
    StateData* thetaState = nullptr;
    StateData* thetaDotState = nullptr;

    std::string nameOfThetaState{};
    std::string nameOfThetaDotState{};

    void Reset(uint64_t CurrentClock) override;
    void writeOutputStateMessages(uint64_t CurrentClock) override;
    void UpdateState(uint64_t CurrentSimNanos) override;
    void registerStates(DynParamManager& statesIn) override;
    void linkInStates(DynParamManager& states) override;
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN) override;
    void updateEffectorMassProps(double integTime) override;
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d& rotAngMomPntCContr_B,
                                      double& rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override;
    void prependSpacecraftNameToStates() override;

    void readInputMessages();
    void computeSpinningBodyInertialStates();
    void computeAttitudeProperties(SpinningBody& spinningBody, int spinningBodyIndex) const;
    void computeAngularVelocityProperties(SpinningBody& spinningBody, int spinningBodyIndex) const;
    void computePositionProperties(SpinningBody& spinningBody, int spinningBodyIndex) const;
    void computeVelocityProperties(SpinningBody& spinningBody, int spinningBodyIndex) const;
    void computeInertiaProperties(SpinningBody& spinningBody) const;
    void computeMTheta(Eigen::MatrixXd& MTheta);
    void computeAThetaStar(Eigen::MatrixXd& AThetaStar);
    void computeBThetaStar(Eigen::MatrixXd& BThetaStar);
    void computeCThetaStar(Eigen::VectorXd& CThetaStar, const Eigen::Vector3d& g_N);
    void computeBackSubMatrices(BackSubMatrices& backSubContr) const;
    void computeBackSubVectors(BackSubMatrices& backSubContr) const;
};

#endif /* SPINNING_BODY_N_DOF_STATE_EFFECTOR_H */
