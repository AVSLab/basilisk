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


#ifndef LINEAR_TRANSLATION_ONE_DOF_STATE_EFFECTOR_H
#define LINEAR_TRANSLATION_ONE_DOF_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/ArrayMotorForceMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayEffectorLockMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/LinearTranslationRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

/*! @brief linear spring mass damper state effector class */
class linearTranslationOneDOFStateEffector :
	public StateEffector, public SysModel
{
public:
    Message<LinearTranslationRigidBodyMsgPayload> translatingBodyOutMsg;        //!< state output message
    Message<SCStatesMsgPayload> translatingBodyConfigLogOutMsg;                 //!< translating body state config log message
    ReadFunctor<ArrayMotorForceMsgPayload> motorForceInMsg;                     //!< (optional) motor force input message
    ReadFunctor<LinearTranslationRigidBodyMsgPayload> translatingBodyRefInMsg;  //!< (optional) reference state input message
    ReadFunctor<ArrayEffectorLockMsgPayload> motorLockInMsg;                    //!< (optional) lock flag input message

    linearTranslationOneDOFStateEffector();           //!< Constructor
	~linearTranslationOneDOFStateEffector();          //!< Destructor

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
    /** setter for `fHat_B` property */
    void setFHat_B(Eigen::Vector3d fHat_B);
    /** setter for `r_FcF_F` property */
    void setR_FcF_F(Eigen::Vector3d r_FcF_F) {this->r_FcF_F = r_FcF_F;};
    /** setter for `r_F0B_B` property */
    void setR_F0B_B(Eigen::Vector3d r_F0B_B) {this->r_F0B_B = r_F0B_B;};
    /** setter for `IPntFc_F` property */
    void setIPntFc_F(Eigen::Matrix3d IPntFc_F) {this->IPntFc_F = IPntFc_F;};
    /** setter for `dcm_FB` property */
    void setDCM_FB(Eigen::Matrix3d dcm_FB) {this->dcm_FB = dcm_FB;};

    /** setter for `mass` property */
    double getMass() const {return this->mass;};
    /** setter for `k` property */
    double getK() const {return this->k;};
    /** setter for `c` property */
    double getC() const {return this->c;};
    /** setter for `rhoInit` property */
    double getRhoInit() const {return this->rhoInit;};
    /** setter for `rhoDotInit` property */
    double getRhoDotInit() const {return this->rhoDotInit;};
    /** setter for `fHat_B` property */
    Eigen::Vector3d getFHat_B() const {return this->fHat_B;};
    /** setter for `r_FcF_F` property */
    Eigen::Vector3d getR_FcF_F() const {return this->r_FcF_F;};
    /** setter for `r_F0B_B` property */
    Eigen::Vector3d getR_F0B_B() const {return this->r_F0B_B;};
    /** setter for `IPntFc_F` property */
    Eigen::Matrix3d getIPntFc_F() const {return IPntFc_F;};
    /** setter for `dcm_FB` property */
    Eigen::Matrix3d getDCM_FB() const {return dcm_FB;};

private:
    double mass = 1.0;              //!< [kg] mass of effector
    double k = 0;                   //!< [N/m] linear spring constant
    double c = 0;                   //!< [N-s/m] linear damping term
    double rhoInit = 0;             //!< [m] initial displacement offset
    double rhoDotInit = 0;          //!< [m/s] Initial displacement rate offset
	Eigen::Vector3d fHat_B {1.0, 0.0, 0.0};         //!< unit vector axis of translation in B frame components.
    Eigen::Vector3d r_FcF_F = Eigen::Vector3d::Zero();        //!< [m] vector pointing from location F to FC in F frame components
    Eigen::Vector3d r_F0B_B = Eigen::Vector3d::Zero();        //!< [m] vector pointing from body frame B origin to point to F0 origin of F frame in B frame components
    Eigen::Matrix3d IPntFc_F = Eigen::Matrix3d::Identity();   //!< [kg-m^2] Inertia of pc about point Fc in F frame component
    Eigen::Matrix3d dcm_FB = Eigen::Matrix3d::Identity();     //!< DCM from the F frame to the body frame
    std::string nameOfRhoState{};     //!< Identifier for the rho state data container
    std::string nameOfRhoDotState{};  //!< Identifier for the rhoDot state data container

    bool isAxisLocked = false;    //!< flag for locking the translation axis
    double rho = 0.0;             //!< [m] displacement from equilibrium
    double rhoDot = 0.0;          //!< [m/s] time derivative of displacement from equilibrium
    double rhoRef = 0.0;          //!< [m] translating body reference position
    double rhoDotRef = 0.0;       //!< [m/s] translating body reference velocity
    double motorForce = 0.0;      //!< [N] optional motor force
    Eigen::Vector3d r_FcB_B = Eigen::Vector3d::Zero();            //!< [m] position vector from B to center of mass location of effector
    Eigen::Vector3d r_FcF0_B = Eigen::Vector3d::Zero();           //!< [m] vector pointing from point p0 origin of F frame to center of mass location of effector in B frame components
    Eigen::Matrix3d rTilde_FcF_B = Eigen::Matrix3d::Zero();       //!< [m] tilde matrix of r_FcF_B
	Eigen::Vector3d rPrime_FcF_B = Eigen::Vector3d::Zero();       //!< [m/s] Body time derivative of r_FcF_B
	Eigen::Matrix3d rPrimeTilde_FcF_B = Eigen::Matrix3d::Zero();  //!< [m/s] Tilde matrix of rPrime_FcF_B
	Eigen::Matrix3d rTilde_FcB_B = Eigen::Matrix3d::Zero();       //!< [m] tilde matrix of r_FcB_B
	Eigen::Vector3d rPrime_FcB_B = Eigen::Vector3d::Zero();       //!< [m/s] Body time derivative of r_FcB_B
	Eigen::Matrix3d rPrimeTilde_FcB_B = Eigen::Matrix3d::Zero();  //!< [m/s] Tilde matrix of rPrime_FcB_B
	Eigen::Matrix3d IPntFc_B = Eigen::Matrix3d::Identity();       //!< [kg-m^2] Inertia of Fc about point B in B frame components
	Eigen::Matrix3d dcm_BN = Eigen::Matrix3d::Identity();         //!< DCM from the B frame to the N frame
    Eigen::Vector3d omega_BN_B = Eigen::Vector3d::Zero();         //!< [rad/s] angular velocity of the B frame wrt the N frame in B frame components.
    Eigen::Matrix3d omegaTilde_BN_B = Eigen::Matrix3d::Zero();    //!< [rad/s] tilde matrix of omega_BN_B

    Eigen::Vector3d aRho = Eigen::Vector3d::Zero();          //!< Term needed for back-sub method
    Eigen::Vector3d bRho = Eigen::Vector3d::Zero();          //!< Term needed for back-sub method
    double cRho = 0.0;                   //!< Term needed for back-sub method

    StateData *rhoState = nullptr;		    //!< state data for displacement from equilibrium
    StateData *rhoDotState = nullptr;	    //!< state data for time derivative of rho;
    Eigen::MatrixXd *g_N = nullptr;         //!< [m/s^2] gravitational acceleration in N frame components
    Eigen::MatrixXd* inertialPositionProperty = nullptr;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase
    static uint64_t effectorID;    //!< ID number of this panel

    Eigen::Vector3d r_FcN_N = Eigen::Vector3d::Zero();            //!< [m] position vector of translating body's center of mass Fc relative to the inertial frame origin N
    Eigen::Vector3d v_FcN_N = Eigen::Vector3d::Zero();            //!< [m/s] inertial velocity vector of Fc relative to inertial frame
    Eigen::Vector3d sigma_FN = Eigen::Vector3d::Zero();           //!< MRP attitude of frame F relative to inertial frame
    Eigen::Vector3d omega_FN_F = Eigen::Vector3d::Zero();         //!< [rad/s] inertial translating body frame angular velocity vector

    void Reset(uint64_t CurrentClock) override;
	void registerStates(DynParamManager& states) override;
	void linkInStates(DynParamManager& states) override;
    void writeOutputStateMessages(uint64_t CurrentSimNanos) override;
    void updateEffectorMassProps(double integTime) override;
    void updateContributions(double integTime,
                             BackSubMatrices & backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B) override;
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

    void computeTranslatingBodyInertialStates();
    void computeBackSubContributions(BackSubMatrices& backSubContr, const Eigen::Vector3d& F_g);
    void readInputMessages();
};

#endif /* LINEAR_TRANSLATION_ONE_DOF_STATE_EFFECTOR_H */
