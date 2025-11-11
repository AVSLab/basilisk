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

#ifndef PRESCRIBED_MOTION_STATE_EFFECTOR_H
#define PRESCRIBED_MOTION_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/PrescribedTranslationMsgPayload.h"
#include "architecture/msgPayloadDefC/PrescribedRotationMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"

/*! @brief prescribed motion state effector class */
class PrescribedMotionStateEffector: public StateEffector, public SysModel {
public:
    PrescribedMotionStateEffector();
    ~PrescribedMotionStateEffector();
    void setMass(const double mass);    //!< Setter method for the effector mass
    void setIPntPc_P(const Eigen::Matrix3d IPntPc_P);    //!< Setter method for IPntPc_P
    void setR_PcP_P(const Eigen::Vector3d r_PcP_P);    //!< Setter method for r_PcP_P
    void setR_PM_M(const Eigen::Vector3d r_PM_M);    //!< Setter method for r_PM_M
    void setRPrime_PM_M(const Eigen::Vector3d rPrime_PM_M);    //!< Setter method for rPrime_PM_M
    void setRPrimePrime_PM_M(const Eigen::Vector3d rPrimePrime_PM_M);    //!< Setter method for rPrimePrime_PM_M
    void setOmega_PM_P(const Eigen::Vector3d omega_PM_P);    //!< Setter method for omega_PM_P
    void setOmegaPrime_PM_P(const Eigen::Vector3d omegaPrime_PM_P);    //!< Setter method for omegaPrime_PM_P
    void setSigma_PM(const Eigen::MRPd sigma_PM);    //!< Setter method for sigma_PM
    void setR_MB_B(const Eigen::Vector3d r_MB_B);    //!< Setter method for r_MB_B
    void setSigma_MB(const Eigen::MRPd sigma_MB);    //!< Setter method for sigma_MB

    double getMass() const;    //!< Getter method for the effector mass
    const Eigen::Matrix3d getIPntPc_P() const;    //!< Getter method for IPntPc_P
    const Eigen::Vector3d getR_PcP_P() const;    //!< Getter method for r_PcP_P
    const Eigen::Vector3d getR_PM_M() const;    //!< Getter method for r_PM_M
    const Eigen::Vector3d getRPrime_PM_M() const;    //!< Getter method for rPrime_PM_M
    const Eigen::Vector3d getRPrimePrime_PM_M() const;    //!< Getter method for rPrimePrime_PM_M
    const Eigen::Vector3d getOmega_PM_P() const;    //!< Getter method for omega_PM_P
    const Eigen::Vector3d getOmegaPrime_PM_P() const;    //!< Getter method for omegaPrime_PM_P
    const Eigen::MRPd getSigma_PM() const;    //!< Getter method for sigma_PM
    const Eigen::Vector3d getR_MB_B() const;    //!< Getter method for r_MB_B
    const Eigen::MRPd getSigma_MB() const;    //!< Getter method for sigma_MB

    void Reset(uint64_t currentClock) override;                      //!< Method for reset
    void writeOutputStateMessages(uint64_t currentClock) override;   //!< Method for writing the output messages
	void UpdateState(uint64_t currentSimNanos) override;             //!< Method for updating the effector states
    void registerStates(DynParamManager& statesIn) override;         //!< Method for registering the effector's states
    void linkInStates(DynParamManager& states) override;             //!< Method for giving the effector access to hub states
    void registerProperties(DynParamManager& states) override;       //!< Method for registering the prescribed motion properties
    void updateContributions(double integTime,
                             BackSubMatrices & backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;          //!< Method for computing the effector's back-substitution contributions
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN) override;      //!< Method for effector to compute its state derivatives
    void updateEffectorMassProps(double integTime) override;         //!< Method for calculating the effector mass props and prop rates
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d & rotAngMomPntCContr_B,
                                      double & rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override;    //!< Method for computing the energy and momentum of the effector
    void computePrescribedMotionInertialStates();       //!< Method for computing the effector's states relative to the inertial frame
    void addStateEffector(StateEffector *newStateEffector);          //!< Method to attach a state effector to prescribed motion

    ReadFunctor<PrescribedTranslationMsgPayload> prescribedTranslationInMsg;      //!< Input message for the effector's translational prescribed states
    ReadFunctor<PrescribedRotationMsgPayload> prescribedRotationInMsg;            //!< Input message for the effector's rotational prescribed states
    Message<PrescribedTranslationMsgPayload> prescribedTranslationOutMsg;         //!< Output message for the effector's translational prescribed states
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg;               //!< Output message for the effector's rotational prescribed states
    Message<SCStatesMsgPayload> prescribedMotionConfigLogOutMsg;                  //!< Output config log message for the effector's states

private:
    double currentSimTimeSec;                           //!< [s] Current simulation time, updated at the dynamics frequency
    double mass;                                        //!< [kg] Effector mass
    Eigen::Matrix3d IPntPc_P;                           //!< [kg-m^2] Inertia of the effector about its center of mass in P frame components
    Eigen::Vector3d r_MB_B;                             //!< [m] Position of point M relative to point B in B frame components
    Eigen::Vector3d r_PcP_P;                            //!< [m] Position of the effector center of mass relative to point P in P frame components
    Eigen::MRPd sigma_MB;                               //!< MRP attitude of frame M relative to frame B

    // Prescribed parameters
    Eigen::Vector3d r_PM_M;                             //!< [m] Position of point P relative to point M in M frame components
    Eigen::Vector3d rPrime_PM_M;                        //!< [m/s] B frame time derivative of r_PM_M in M frame components
    Eigen::Vector3d rPrimePrime_PM_M;                   //!< [m/s^2] B frame time derivative of rPrime_PM_M in M frame components
    Eigen::Vector3d omega_PM_P;                         //!< [rad/s] Angular velocity of frame P relative to frame M in P frame components
    Eigen::Vector3d omegaPrime_PM_P;                    //!< [rad/s^2] B frame time derivative of omega_PM_P in P frame components
    Eigen::MRPd sigma_PM;                               //!< MRP attitude of frame P relative to frame M
    std::string nameOfsigma_PMState;                    //!< Identifier for the sigma_PM state data container

    static uint64_t effectorID;                                         //!< ID number of this panel

    // Given quantities from user in python
    Eigen::Matrix3d IPntPc_B;                           //!< [kg-m^2] Inertia of the effector about its center of mass in B frame components
    Eigen::Vector3d r_PcP_B;                            //!< [m] Position of the effector center of mass relative to point P in B frame components

    // Prescribed parameters in body frame components
    Eigen::Vector3d r_PM_B;                             //!< [m] Position of point P relative to point M in B frame components
    Eigen::Vector3d rPrime_PM_B;                        //!< [m/s] B frame time derivative of position r_PM_B in B frame components
    Eigen::Vector3d rPrimePrime_PM_B;                   //!< [m/s^2] B frame time derivative of rPrime_PM_B in B frame components
    Eigen::Vector3d omega_PM_B;                         //!< [rad/s] Angular velocity of P frame relative to the M frame in B frame components
    Eigen::Vector3d omegaPrime_PM_B;                    //!< [rad/s^2] B frame time derivative of omega_PB_B in B frame components
    Eigen::Vector3d omega_PB_B;                         //!< [rad/s] Angular velocity of frame P relative to frame B in B frame components
    Eigen::Vector3d omegaPrime_PB_B;                    //!< [rad/s^2] B frame time derivative of omega_PB_B in B frame components

    // Other vector quantities
    Eigen::Vector3d r_PcM_B;                            //!< [m] Position of frame P center of mass relative to point M in B frame components
    Eigen::Vector3d r_PcB_B;                            //!< [m] Position of frame P center of mass relative to point B in B frame components
    Eigen::Vector3d rPrime_PcM_B;                       //!< [m/s] B frame time derivative of r_PcM_B in B frame components
    Eigen::Vector3d rPrime_PcB_B;                       //!< [m/s] B frame time derivative of r_PcB_B in B frame components
    Eigen::Vector3d rPrimePrime_PcB_B;                  //!< [m/s^2] B frame time derivative of rPrime_PcB_B in B frame components
    Eigen::Vector3d omega_BN_B;                         //!< [rad/s] Angular velocity of frame B relative to the inertial frame in B frame components
    Eigen::Vector3d omega_PN_B;                         //!< [rad/s] Angular velocity of frame P relative to the inertial frame in B frame components
    Eigen::Vector3d rDot_PcB_B;                         //!< [m/s] Inertial time derivative of r_PcB_B in B frame components
    Eigen::MRPd sigma_BN;                               //!< MRP attitude of frame B relative to the inertial frame

    // DCMs
    Eigen::Matrix3d dcm_BN;                             //!< DCM from inertial frame to frame B
    Eigen::Matrix3d dcm_BM;                             //!< DCM from frame M to frame B
    Eigen::Matrix3d dcm_PM;                             //!< DCM from frame M to frame P
    Eigen::Matrix3d dcm_BP;                             //!< DCM from frame P to frame B

    // Other matrix quantities
    Eigen::Matrix3d rTilde_PcB_B;                       //!< [m] Tilde cross product matrix of r_PcB_B
    Eigen::Matrix3d omegaTilde_BN_B;                    //!< [rad/s] Tilde cross product matrix of omega_BN_B
    Eigen::Matrix3d omegaTilde_PB_B;                    //!< [rad/s] Tilde cross product matrix of omega_PB_B

    // Effector properties relative to the inertial frame
    Eigen::Vector3d r_PcN_N;                            //!< [m] Position of frame P center of mass relative to the inertial frame in inertial frame components
    Eigen::Vector3d v_PcN_N;                            //!< [m/s] Inertial velocity of frame P center of mass relative to the inertial frame in inertial frame components
    Eigen::MatrixXd* r_PN_N;                            //!< [m] Position of frame P relative to the inertial frame in inertial frame components
    Eigen::MatrixXd* v_PN_N;                            //!< [m/s] Inertial velocity of frame P relative to the inertial frame in inertial frame components
    Eigen::MatrixXd* sigma_PN;                          //!< MRP attitude of frame P relative to the inertial frame
    Eigen::MatrixXd* omega_PN_P;                        //!< [rad/s] Angular velocity of frame P relative to the inertial frame in P frame components

    // Hub states
    Eigen::MatrixXd* inertialPositionProperty;          //!< [m] r_N Inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty;          //!< [m] v_N Inertial velocity relative to system spice zeroBase/refBase

    // Prescribed states at epoch (Dynamics time step)
    Eigen::Vector3d rEpoch_PM_M;                        //!< [m] Position of point P relative to point M in M frame components
    Eigen::Vector3d rPrimeEpoch_PM_M;                   //!< [m/s] B frame time derivative of r_PM_M in M frame components
    Eigen::Vector3d omegaEpoch_PM_P;                    //!< [rad/s] Angular velocity of frame P relative to frame M in P frame components
    StateData *sigma_PMState;                           //!< MRP attitude of frame P relative to frame M

    // Parameters required for effector branching
    std::string spacecraftName;                         //!< Name of prescribed object used for effector branching
    std::vector<StateEffector*> stateEffectors;         //!< Vector of attached state effectors

    Eigen::MatrixXd* r_PB_B;                            //!< [m] Position of point P relative to point B in B frame components
    Eigen::MatrixXd* rPrime_PB_B;                       //!< [m/s] B frame time derivative of r_PB_B in B frame components
    Eigen::MatrixXd* rPrimePrime_PB_B;                  //!< [m/s^2] B frame time derivative of rPrime_PB_B in B frame components
    Eigen::MatrixXd* sigma_PB;                          //!< MRP attitude of frame P relative to frame B
    Eigen::MatrixXd* omega_PB_P;                        //!< [rad/s] Angular velocity of frame P relative to frame B in P frame components
    Eigen::MatrixXd* omegaPrime_PB_P;                   //!< [rad/s] B frame time derivative of omega_PB_P in P frame components

    std::string nameOfPrescribedPositionProperty;         //!< Identifier for prescribed position r_PB_B
    std::string nameOfPrescribedVelocityProperty;         //!< Identifier for prescribed velocity rPrime_PB_B
    std::string nameOfPrescribedAccelerationProperty;     //!< Identifier for prescribed acceleration rPrimePrime_PB_B
    std::string nameOfPrescribedAttitudeProperty;         //!< Identifier for prescribed attitude sigma_PB
    std::string nameOfPrescribedAngVelocityProperty;      //!< Identifier for prescribed angular velocity omega_PB_P
    std::string nameOfPrescribedAngAccelerationProperty;  //!< Identifier for prescribed angular acceleration omegaPrime_PB_P

    std::string nameOfInertialPositionProperty;           //!< Identifier for prescribed motion inertial position r_PN_N
    std::string nameOfInertialVelocityProperty;           //!< Identifier for prescribed motion inertial velocity property v_PN_N
    std::string nameOfInertialAttitudeProperty;           //!< Identifier for the prescribed motion inertial attitude property sigma_PN
    std::string nameOfInertialAngVelocityProperty;        //!< Identifier for the prescribed motion inertial angular velocity property omega_PN_P

    template <typename Type>
    /** Assign the state engine parameter names to attached effectors*/
    void assignStateParamNames(Type effector) {
        effector->setPropName_inertialPosition(this->nameOfInertialPositionProperty);
        effector->setPropName_inertialVelocity(this->nameOfInertialVelocityProperty);

        effector->setPropName_prescribedPosition(this->nameOfPrescribedPositionProperty);
        effector->setPropName_prescribedVelocity(this->nameOfPrescribedVelocityProperty);
        effector->setPropName_prescribedAcceleration(this->nameOfPrescribedAccelerationProperty);
        effector->setPropName_prescribedAttitude(this->nameOfPrescribedAttitudeProperty);
        effector->setPropName_prescribedAngVelocity(this->nameOfPrescribedAngVelocityProperty);
        effector->setPropName_prescribedAngAcceleration(this->nameOfPrescribedAngAccelerationProperty);
    };
};

#endif /* PRESCRIBED_MOTION_STATE_EFFECTOR_H */
