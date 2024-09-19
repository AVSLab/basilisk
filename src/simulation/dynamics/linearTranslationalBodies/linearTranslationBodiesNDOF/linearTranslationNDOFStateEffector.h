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

#ifndef LINEAR_TRANSLATION_N_DOF_STATE_EFFECTOR_H
#define LINEAR_TRANSLATION_N_DOF_STATE_EFFECTOR_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/ArrayMotorForceMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayEffectorLockMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/LinearTranslationRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

/*! @brief translating body structure */
struct translatingBody {
public:
    /** setter for `mass` property */
    void setMass(double mass);
    /** setter for `k` property */
    void setK(double k);
    /** setter for `c` property */
    void setC(double c);
    /** setter for `rhoInit` property */
    void setRhoInit(double rhoInit) {this->rhoInit = rhoInit;};
    /** setter for `rhoDotInit` property */
    void setRhoDotInit(double rhoDotInit) {this->rhoDotInit = rhoDotInit;};
    /** setter for `fHat_P` property */
    void setFHat_P(Eigen::Vector3d fHat_P);
    /** setter for `r_FcF_F` property */
    void setR_FcF_F(Eigen::Vector3d r_FcF_F) {this->r_FcF_F = r_FcF_F;};
    /** setter for `r_F0B_B` property */
    void setR_F0P_P(Eigen::Vector3d r_F0P_P) {this->r_F0P_P = r_F0P_P;};
    /** setter for `IPntFc_F` property */
    void setIPntFc_F(Eigen::Matrix3d IPntFc_F) {this->IPntFc_F = IPntFc_F;};
    /** setter for `dcm_FB` property */
    void setDCM_FP(Eigen::Matrix3d dcm_FP) {this->dcm_FP = dcm_FP;};

    /** getter for `mass` property */
    double getMass() const {return this->mass;};
    /** getter for `k` property */
    double getK() const {return this->k;};
    /** getter for `c` property */
    double getC() const {return this->c;};
    /** getter for `rhoInit` property */
    double getRhoInit() const {return this->rhoInit;};
    /** getter for `rhoDotInit` property */
    double getRhoDotInit() const {return this->rhoDotInit;};
    /** getter for `fHat_P` property */
    Eigen::Vector3d getFHat_P() const {return this->fHat_P;};
    /** getter for `r_FcF_F` property */
    Eigen::Vector3d getR_FcF_F() const {return this->r_FcF_F;};
    /** getter for `r_F0P_P` property */
    Eigen::Vector3d getR_F0P_P() const {return this->r_F0P_P;};
    /** getter for `IPntFc_F` property */
    Eigen::Matrix3d getIPntFc_F() const {return IPntFc_F;};
    /** getter for `dcm_FP` property */
    Eigen::Matrix3d getDCM_FP() const {return dcm_FP;};

private:
    friend class linearTranslationNDOFStateEffector;

    // user-defined properties
    double mass = 0.0;             //!< [kg] mass of translating arm
    double k = 0.0;                //!< [N/m] translational spring constant
    double c = 0.0;                //!< [N-s/m] translational damping coefficient
    double rhoInit = 0.0;          //!< [m] initial translating body distance from equilibrium
    double rhoDotInit = 0.0;       //!< [m/s] initial translating body velocity of F frame wrt F0 frame
    double rhoRef = 0.0;           //!< [m] reference translating body distance from equilibrium
    double rhoDotRef = 0.0;        //!< [m/s] reference translating body velocity of F frame wrt F0 frame
    double u = 0.0;                //!< [N] motor force acting along the translating axis of the body
    bool isAxisLocked = false;     //!< -- lock flag
    Eigen::Matrix3d IPntFc_F = Eigen::Matrix3d::Identity();   //!< [kg-m^2] Inertia of body about point Fc in F frame components
    Eigen::Vector3d r_FcF_F = Eigen::Vector3d::Zero();        //!< [m] vector pointing from translating frame F origin to point Fc (center of mass of arm) in F frame components
    Eigen::Vector3d r_F0P_P = Eigen::Vector3d::Zero();        //!< [m] vector pointing from parent origin to translating frame F origin in F frame components
    Eigen::Vector3d fHat_P{1.0, 0.0, 0.0};           //!< -- translating axis in parent frame components.
    Eigen::Matrix3d dcm_FP = Eigen::Matrix3d::Identity();     //!< -- DCM from parent frame to current F frame

    // Scalar Properties
    double rho = 0.0;              //!< [m] translating body distance from equilibrium
    double rhoDot = 0.0;           //!< [m/s] translating body velocity of F frame wrt F0 frame

    // Vector quantities
    Eigen::Vector3d r_FF0_B;            //!< [m] vector pointing from translating frame F to translating frame F0 (magnitude rho)
    Eigen::Vector3d r_F0P_B;            //!< [m] vector pointing from parent translating frame P to translating frame F0
    Eigen::Vector3d fHat_B;             //!< -- translating axis in B frame components.
    Eigen::Vector3d r_FcF_B;            //!< [m] vector pointing from translating frame F origin to point Fc (center of mass of arm) in B frame components
    Eigen::Vector3d r_FB_B;             //!< [m] vector pointing from body frame B origin to F frame in B frame components
    Eigen::Vector3d r_FcB_B;            //!< [m] vector pointing from body frame B origin to Fc in B frame components
    Eigen::Vector3d r_FP_B;             //!< [m] vector from parent frame to current F frame in B frame componentes
    Eigen::Vector3d r_FP_P;             //!< [m] vector from parent frame to current F frame in parent frame components
    Eigen::Vector3d rPrime_FB_B;        //!< [m/s] body frame time derivative of r_FB_B
    Eigen::Vector3d rPrime_FcF_B;       //!< [m/s] body frame time derivative of r_FcF_B
    Eigen::Vector3d rPrime_FcB_B;       //!< [m/s] body frame time derivative of r_FcB_B
    Eigen::Vector3d rPrime_FP_B;        //!< [m/s] body frame time derivative of r_FP_B
    Eigen::Vector3d rPrime_FF0_B;       //!< [m/s] body frame time derivative of r_FF0_B
    Eigen::Vector3d rDot_FcB_B;         //!< [m/s] inertial frame time derivative of r_FcB_B
    Eigen::Vector3d omega_FN_B;         //!< [rad/s] angular velocity of the F frame wrt the N frame in B frame components

    // Matrix quantities
    Eigen::Matrix3d dcm_FB;            //!< -- DCM from body frame to F frame
    Eigen::Matrix3d IPntFc_B;          //!< [kg-m^2] Inertia of body about point Fc in B frame components
    Eigen::Matrix3d IPrimePntFc_B;     //!< [kg-m^2/s] body frame time derivative of IPntFc_B
    Eigen::Matrix3d rTilde_FcB_B;      //!< [m] tilde matrix of r_FcB_B
    Eigen::Matrix3d omegaTilde_FB_B;   //!< [rad/s] tilde matrix of omega_FB_B

    // Inertial properties
    Eigen::Vector3d r_FcN_N;            //!< [m] position vector of translating body's center of mass Fc relative to the inertial frame origin N
    Eigen::Vector3d v_FcN_N;            //!< [m/s] inertial velocity vector of Fc relative to inertial frame
    Eigen::Vector3d sigma_FN;           //!< -- MRP attitude of frame S relative to inertial frame
    Eigen::Vector3d omega_FN_F;         //!< [rad/s] inertial translating body frame angular velocity vector

    BSKLogger bskLogger;
};

/*! @brief translating body state effector class */
class linearTranslationNDOFStateEffector: public StateEffector, public SysModel {
public:

    linearTranslationNDOFStateEffector();         //!< -- Constructor
    ~linearTranslationNDOFStateEffector() final;  //!< -- Destructor

    std::vector<Message<LinearTranslationRigidBodyMsgPayload>*> translatingBodyOutMsgs;       //!< vector of state output messages
    std::vector<Message<SCStatesMsgPayload>*> translatingBodyConfigLogOutMsgs;                //!< vector of translating body state config log messages
    std::vector<ReadFunctor<LinearTranslationRigidBodyMsgPayload>> translatingBodyRefInMsgs;  //!< (optional) reference state input message
    ReadFunctor<ArrayMotorForceMsgPayload> motorForceInMsg;           //!< -- (optional) motor force input message name
    ReadFunctor<ArrayEffectorLockMsgPayload> motorLockInMsg;          //!< -- (optional) motor lock input message name

    /** method for adding a new translating body */
    void addTranslatingBody(translatingBody const& newBody);
    /** setter for `nameOfRhoState` property */
    void setNameOfRhoState(const std::string& nameOfRhoState) { this->nameOfRhoState = nameOfRhoState; };
    /** setter for `nameOfRhoDotState` property */
    void setNameOfRhoDotState(const std::string& nameOfRhoDotState) { this->nameOfRhoDotState = nameOfRhoDotState; };
    /** getter for `nameOfRhoState` property */
    std::string getNameOfRhoState() const { return this->nameOfRhoState; };
    /** getter for `nameOfRhoDotState` property */
    std::string getNameOfRhoDotState() const { return this->nameOfRhoDotState; };

private:
    static uint64_t effectorID;    //!< [] ID number of this effector
    int N = 0;    //!< -- number of translating body axes defined in the system
    std::vector<translatingBody> translatingBodyVec; //!< -- vector of TB effector structs

    // Terms needed for back substitution
    Eigen::MatrixXd ARho;     //!< -- rDDot_BN term for back substitution
    Eigen::MatrixXd BRho;     //!< -- omegaDot_BN term for back substitution
    Eigen::VectorXd CRho;     //!< -- scalar term for back substitution

    // Hub properties
    Eigen::Vector3d omega_BN_B;   //!< [rad/s] angular velocity of the B frame wrt the N frame in B frame components
    Eigen::MRPd sigma_BN;         //!< -- body frame attitude wrt to the N frame in MRPs
    Eigen::Matrix3d dcm_BN;       //!< -- DCM from inertial frame to body frame

    // States
    Eigen::MatrixXd* inertialPositionProperty = nullptr;    //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;    //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase
    StateData* rhoState = nullptr;
    StateData* rhoDotState = nullptr;
    std::string nameOfRhoState;        //!< -- identifier for the theta state data container
    std::string nameOfRhoDotState;     //!< -- identifier for the thetaDot state data container

    // module functions
    void Reset(uint64_t CurrentClock) final;
    void readInputMessages();
    void writeOutputStateMessages(uint64_t CurrentClock) final;
    void UpdateState(uint64_t CurrentSimNanos) final;
    void registerStates(DynParamManager& statesIn) final;
    void linkInStates(DynParamManager& states) final;
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) final;
    void computeMRho(Eigen::MatrixXd& MRho);
    void computeARhoStar(Eigen::MatrixXd& ARhoStar);
    void computeBRhoStar(Eigen::MatrixXd& BRhoStar);
    void computeCRhoStar(Eigen::VectorXd& CRhoStar, const Eigen::Vector3d& g_N);
    void computeBackSubContributions(BackSubMatrices& backSubContr) const;
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN) final;
    void updateEffectorMassProps(double integTime) final;
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d& rotAngMomPntCContr_B,
                                      double& rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) final;
    void prependSpacecraftNameToStates() final;
    void computeTranslatingBodyInertialStates();
};

#endif /* LINEAR_TRANSLATION_N_DOF_STATE_EFFECTOR_H */
